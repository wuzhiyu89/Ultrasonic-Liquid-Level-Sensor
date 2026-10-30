/**
 * @file app_pair.c
 * @brief 从机配对/绑定模式实现：每 5s 发一帧 0x04，收到网关应答后绑定
 * @note  禁止在中断上下文调用（内部有 Flash 擦写 + 射频阻塞收发）
 */
#include "app_pair.h"
#include "app_cmd.h"
#include "app_dev_param.h"
#include "app_runtime_state.h"
#include "app_lora.h"
#include "crc16_modbus.h"
#include "protocol_util.h"
#include "LLCC68-board.h"       /* LLCC68GetTimerCount() 毫秒计时 */
#include "delay.h"              /* delay_ms() */
#include "log.h"

/* ==================== 配对时序 ==================== */
/* 发送周期(ms)：需求是"每 5s 发送一次" */
#ifndef APP_PAIR_PERIOD_MS
#define APP_PAIR_PERIOD_MS      5000U
#endif
/* 接收窗口(ms)：从"准备发这一帧"开始算，一直收到下一帧该发的时刻。
 * 默认 = 发送周期，也就是【发完就把接收盖满整个间隔】，中间不留空档：
 * 发一帧本身要 ~0.5s(收发模式切换 LORA_TX_MODE_DELAY_MS)，所以实际在听约 4.5s，
 * 到点立刻发下一帧 -> 空口上就是严格的 5s 一帧。
 * ⚠️ 不要把窗口调小：网关回 ACK 可能带 3s 延时(GW_REPLY_ACK_DELAY_MS，网关侧参数)，
 *    窗口和周期之间留空档的话，ACK 会每次都落在空档里，永远配不上。 */
#ifndef APP_PAIR_RX_WIN_MS
#define APP_PAIR_RX_WIN_MS      5000U
#endif
/* 等 TxDone 超时(ms) */
#define APP_PAIR_TX_TIMEOUT_MS  3000U
/* 接收缓存 */
#define APP_PAIR_RX_BUF_MAX     64U

/* 配对相关无载荷帧总长 = 协议最小帧长：
 * 帧头2 + 版本1 + 网关13 + 设备13 + 功能码1 + data_len1 + CRC2 = 33 */
#define APP_PAIR_FRAME_LEN      PROTO_FRAME_MIN_LEN

/* ==================== 绑定成功之后的行为 ====================
 * 绑定成功不"一直发/一直等"，只持续发 APP_PAIR_ACK_AFTER_BOUND_MS(10s) 的 0x80 ACK
 * （帧头带已绑定的网关地址 + 本机地址），然后返回 true 由调用方复位；
 * 复位后 Flash 里已有有效网关地址 -> 直接进主流程（第一轮必上报一帧液位）。
 *
 *   APP_PAIR_ACK_AFTER_BOUND_MS  发 ACK 的时长（默认 10s）
 *   APP_PAIR_ACK_PERIOD_MS       这 10s 内每隔多久发一帧（默认 2s -> 5 帧）
 *   APP_PAIR_WAIT_ANSWER_MS      等网关应答的最长时间（默认 60s），超时返回 false
 * ============================================================================ */
#ifndef APP_PAIR_WAIT_ANSWER_MS
#define APP_PAIR_WAIT_ANSWER_MS     60000U
#endif
#ifndef APP_PAIR_ACK_AFTER_BOUND_MS
#define APP_PAIR_ACK_AFTER_BOUND_MS 10000U
#endif
#ifndef APP_PAIR_ACK_PERIOD_MS
#define APP_PAIR_ACK_PERIOD_MS      2000U
#endif

static uint8_t s_req[APP_PAIR_FRAME_LEN];
static uint8_t s_rx[APP_PAIR_RX_BUF_MAX];

/* 一包16进制打印（日志统一英文，避免中文字符串乱码） */
static void app_pair_dump(const char *tag, const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    log_info("%s(%u):", tag, (unsigned)len);
    for (i = 0U; i < len; i++)
    {
        log_info(" %02X", (unsigned)buf[i]);
    }
    log_info("\r\n");
}

/* 组配对请求帧：AA 55 | 01 | 13×FF(任意网关) | 本机设备地址(0x21+UID, 13B) | 04 | 00 | CRC
 * CRC = Modbus(版本字节..数据域末尾)，帧头 AA 55 不参与，低字节在前。
 * 例(占位 UID：设备 01..0C)：
 *   AA 55 01 | FF×13 | 21 01 02 03 04 05 06 07 08 09 0A 0B 0C | 04 00 | C8 46 */
static void app_pair_build_request(uint8_t *buf, const uint8_t dev_addr[PROTO_ADDR_LEN])
{
    uint8_t i;

    buf[0] = PROTO_HEADER_0;
    buf[1] = PROTO_HEADER_1;
    buf[2] = PROTO_VER_V1;
    for (i = 0U; i < PROTO_ADDR_LEN; i++)
    {
        buf[PROTO_OFF_GW_ADDR + i] = 0xFFU;                     /* 目的网关：13×FF = 任意网关 */
    }
    proto_addr_copy(&buf[PROTO_OFF_DEV_ADDR], dev_addr);        /* 自己的设备地址：0x21+UID */
    buf[PROTO_OFF_FUNC]    = (uint8_t)CMD_FUNC_PAIR_BIND;       /* 0x04 配对绑定 */
    buf[PROTO_OFF_DATALEN] = (uint8_t)APP_CMD_PAIR_DLEN;        /* 载荷为空 */
    /* CRC 覆盖 版本字节..数据域末尾（固定头去掉帧头 2 字节），落在帧尾 2 字节 */
    crc16_modbus_append(&buf[2], (uint16_t)(APP_PAIR_FRAME_LEN - 2U - PROTO_CRC_LEN));
}

/* 绑定：先写 Flash 成功再更新 RAM。
 * 顺序很重要 —— Flash 失败但 RAM 已经"绑上了"，会表现为"这次跑主程序，一重启又回配对"。 */
static bool app_pair_bind(const uint8_t gw_addr[PROTO_ADDR_LEN])
{
    if (!app_dev_param_set_gw_addr(gw_addr))
    {
        log_info("[pair] bind FAILED: flash write error, keep pairing\r\n");
        return false;
    }
    app_runtime_state_set_gw_addr(gw_addr);
    return true;
}

/* 校验一帧是否是"发给本机的完整有效帧"（帧头/版本/长度/CRC/目的地址/网关地址）
 * @param rx       收到的原始帧
 * @param len      长度
 * @param dev_addr 本机设备地址
 * @retval true = 有效且是发给本机的（帧头 gw_addr 字段(13B) 也是有效网关地址）
 * @note  绑定判定和"绑定后回 ACK"两条路径共用这一份校验，避免两处判据不一致。 */
static bool app_pair_validate(const uint8_t *rx, uint16_t len, const uint8_t dev_addr[PROTO_ADDR_LEN])
{
    uint8_t  data_len;
    uint16_t calc;

    if (len < PROTO_FRAME_MIN_LEN)
    {
        log_info("[pair] rx %u bytes: too short, ignore\r\n", (unsigned)len);
        return false;
    }
    if ((rx[0] != PROTO_HEADER_0) || (rx[1] != PROTO_HEADER_1))
    {
        log_info("[pair] rx: bad header, ignore\r\n");
        return false;
    }
    if (rx[2] != PROTO_VER_V1)
    {
        log_info("[pair] rx: unsupported ver 0x%02X, ignore\r\n", (unsigned)rx[2]);
        return false;
    }
    data_len = rx[PROTO_OFF_DATALEN];
    if ((uint16_t)(PROTO_FIXED_HEAD_LEN + data_len + PROTO_CRC_LEN) != len)
    {
        log_info("[pair] rx: bad dlen=%u for %u bytes, ignore\r\n",
                 (unsigned)data_len, (unsigned)len);
        return false;
    }
    if (!crc16_modbus_check(&rx[2], len - 2U))
    {
        calc = crc16_modbus(&rx[2], len - 2U);
        log_info("[pair] rx: CRC error, expect %02X %02X, got %02X %02X\r\n",
                 (unsigned)(calc & 0xFFU), (unsigned)((calc >> 8U) & 0xFFU),
                 (unsigned)rx[len - 2U], (unsigned)rx[len - 1U]);
        return false;
    }
    /* 目的地址：本机 或 广播(13×FF) 都收 */
    if ((!proto_addr_equal(&rx[PROTO_OFF_DEV_ADDR], dev_addr)) &&
        (!proto_addr_is_broadcast(&rx[PROTO_OFF_DEV_ADDR])))
    {
        log_info("[pair] rx: not for this device, ignore\r\n");
        return false;
    }
    /* 网关地址必须有效：应答帧头的 gw_addr 就是网关自己的地址，绑定的就是它。
     * 网关把 13×FF 原样回填的话取不到地址，只能继续重试。 */
    if (!proto_addr_is_valid_gw(&rx[PROTO_OFF_GW_ADDR]))
    {
        app_pair_dump("[pair] rx gw_addr invalid (gateway must fill its own 0x11+UID)",
                      &rx[PROTO_OFF_GW_ADDR], PROTO_ADDR_LEN);
        return false;
    }
    return true;
}

/* 回一帧 0x80 ACK（33 字节）：AA 55 | 01 | gw(13) | 本机dev(13) | 80 | 00 | CRC
 * @note  同步等 TX_DONE 再返回，避免"发一半被切走"；发完由调用方重开接收。 */
static void app_pair_send_ack(const uint8_t gw_addr[PROTO_ADDR_LEN],
                              const uint8_t dev_addr[PROTO_ADDR_LEN])
{
    uint8_t ack[APP_PAIR_FRAME_LEN];

    ack[0]  = PROTO_HEADER_0;
    ack[1]  = PROTO_HEADER_1;
    ack[2]  = PROTO_VER_V1;
    proto_addr_copy(&ack[PROTO_OFF_GW_ADDR], gw_addr);
    proto_addr_copy(&ack[PROTO_OFF_DEV_ADDR], dev_addr);
    ack[PROTO_OFF_FUNC]    = (uint8_t)CMD_FUNC_ACK_OK;
    ack[PROTO_OFF_DATALEN] = 0U;
    /* CRC 覆盖 版本字节..数据域末尾 */
    crc16_modbus_append(&ack[2], (uint16_t)(APP_PAIR_FRAME_LEN - 2U - PROTO_CRC_LEN));

    app_pair_dump("[pair] tx ack", ack, (uint16_t)APP_PAIR_FRAME_LEN);
    if (LoraApp_SendSync(ack, (uint8_t)APP_PAIR_FRAME_LEN, APP_PAIR_TX_TIMEOUT_MS))
    {
        log_info("[pair] ACK sent\r\n");
    }
    else
    {
        log_info("[pair] !! ACK send FAILED (no TX_DONE)\r\n");
    }
}

/* 处理配对模式收到的一帧
 * @retval true = 配对成功，可以回主程序；false = 继续等 */
static bool app_pair_handle_rx(const uint8_t *rx, uint16_t len,
                               const uint8_t dev_addr[PROTO_ADDR_LEN])
{
    uint8_t func;

    if (!app_pair_validate(rx, len, dev_addr))
    {
        return false;
    }

    func = rx[PROTO_OFF_FUNC];
    if (func == (uint8_t)CMD_FUNC_ACK_ERR_RETRY)
    {
        log_info("[pair] gateway rejected (0x81), keep pairing\r\n");
        return false;
    }
    /* 0x80 = 正常应答（网关通用 ACK）；也有网关按原功能码 0x04 回，两种都认。
     * 载荷不看：绑定的地址取自帧头的 gw_addr 字段（13 字节 = 0x11 + 网关 UID）。 */
    if ((func != (uint8_t)CMD_FUNC_ACK_OK) && (func != (uint8_t)CMD_FUNC_PAIR_BIND))
    {
        log_info("[pair] rx: func=0x%02X is not a pairing response, ignore\r\n", (unsigned)func);
        return false;
    }

    log_info("[pair] gateway answered: func=0x%02X\r\n", (unsigned)func);
    app_pair_dump("[pair] gw_addr", &rx[PROTO_OFF_GW_ADDR], PROTO_ADDR_LEN);
    return app_pair_bind(&rx[PROTO_OFF_GW_ADDR]);
}

/* 绑定成功后：持续 APP_PAIR_ACK_AFTER_BOUND_MS 主动发 0x80 ACK（带已绑定的网关地址）；
 * 这期间收到网关的下行也回 ACK。到点返回，由调用方复位进主流程。
 * @note 用 LLCC68GetTimerCount() 计时（TIM3 在 LoraApp_Init 之后已经跑起来了） */
static void app_pair_ack_window(const uint8_t dev_addr[PROTO_ADDR_LEN])
{
    AppRuntimeState_t rt;
    uint32_t t0;
    uint32_t elapsed;
    uint32_t next_tx;
    uint16_t rx_len;

    app_runtime_state_get(&rt);             /* 里面有刚绑上的 gw_addr */
    log_info("[pair] bound: send 0x80 every %ums for %ums, then reboot into main flow\r\n",
             (unsigned)APP_PAIR_ACK_PERIOD_MS, (unsigned)APP_PAIR_ACK_AFTER_BOUND_MS);

    t0      = LLCC68GetTimerCount();
    next_tx = 0U;                           /* 从 0 开始：绑定成功立刻先发一帧 */
    LoraApp_ReciveMode();

    while (1)
    {
        elapsed = (uint32_t)(LLCC68GetTimerCount() - t0);
        if (elapsed >= APP_PAIR_ACK_AFTER_BOUND_MS)
        {
            break;
        }

        /* 到点：主动发一帧 0x80（帧头 = 已绑定的网关地址 + 本机设备地址） */
        if (elapsed >= next_tx)
        {
            app_pair_send_ack(rt.gw_addr, dev_addr);
            next_tx += APP_PAIR_ACK_PERIOD_MS;
            LoraApp_ReciveMode();
        }

        /* 这 10s 内网关下发的有效下行，也回一帧 ACK */
        LoraApp_ProcessIrq();
        rx_len = LoraApp_GetRx(s_rx, (uint16_t)sizeof(s_rx));
        if (rx_len > 0U)
        {
            app_pair_dump("[pair] rx", s_rx, rx_len);
            if (app_pair_validate(s_rx, rx_len, dev_addr))
            {
                app_pair_send_ack(&s_rx[PROTO_OFF_GW_ADDR], dev_addr);
            }
            LoraApp_ReciveMode();
        }
        delay_ms(1U);
    }

    LoraApp_Standby();
    log_info("[pair] ACK window done (%ums)\r\n", (unsigned)APP_PAIR_ACK_AFTER_BOUND_MS);
}

/* 配对主循环
 * @retval true  = 绑定成功且已发完 APP_PAIR_ACK_AFTER_BOUND_MS 的 0x80 ACK -> 调用方复位
 *         false = APP_PAIR_WAIT_ANSWER_MS 内没等到网关应答 -> 调用方回去睡
 */
bool app_pair_run(void)
{
    AppRuntimeState_t rt;
    uint16_t rx_len;
    uint32_t t_start;
    uint32_t t_enter;
    uint32_t win_ms;
    bool     got_any;

    app_runtime_state_get(&rt);
    app_pair_build_request(s_req, rt.dev_addr);

    log_info("\r\n===== pairing mode (unbound) =====\r\n");
    app_pair_dump("[pair] dev_addr (0x21+UID)", rt.dev_addr, PROTO_ADDR_LEN);
    app_pair_dump("[pair] gw_addr (13xFF = unbound)", rt.gw_addr, PROTO_ADDR_LEN);
    log_info("[pair] -> need binding; wait answer at most %ums\r\n",
             (unsigned)APP_PAIR_WAIT_ANSWER_MS);
    log_info("[pair] send 0x04 every %ums, listen %ums each time\r\n",
             (unsigned)APP_PAIR_PERIOD_MS, (unsigned)APP_PAIR_RX_WIN_MS);
    app_pair_dump("[pair] request frame", s_req, (uint16_t)APP_PAIR_FRAME_LEN);

    LoraApp_Init();
    LoraApp_Standby();
    t_enter = LLCC68GetTimerCount();

    while (1)
    {
        /* ---- ⓪ 超时检查：总共最多等 APP_PAIR_WAIT_ANSWER_MS ---- */
        if ((uint32_t)(LLCC68GetTimerCount() - t_enter) >= APP_PAIR_WAIT_ANSWER_MS)
        {
            log_info("[pair] no answer within %ums -> give up this attempt\r\n",
                     (unsigned)APP_PAIR_WAIT_ANSWER_MS);
            LoraApp_Standby();
            return false;
        }

        t_start = LLCC68GetTimerCount();

        /* ---- ① 发配对请求 ----
         * SendSync 只在【芯片报了 TX_DONE】时返回 true（PA ramp + 调制已跑完、
         * 数据真的出去了），所以这行日志就是"我们到底发没发"的分界线：
         * "sent" = 发了，收不到要去对方/射频参数上找；"TX FAILED" = 芯片没发完。 */
        if (LoraApp_SendSync(s_req, (uint8_t)APP_PAIR_FRAME_LEN, APP_PAIR_TX_TIMEOUT_MS))
        {
            log_info("[pair] pairing request sent (chip reports TX_DONE)\r\n");
        }
        else
        {
            log_info("[pair] !! TX FAILED: no TX_DONE (nothing left the chip) - check power/antenna/PA\r\n");
        }

        /* ---- ② 开接收窗口：窗口 = APP_PAIR_RX_WIN_MS，到点回来发下一帧 ---- */
        got_any = false;
        win_ms  = APP_PAIR_RX_WIN_MS;

        LoraApp_ReciveMode();
        while ((uint32_t)(LLCC68GetTimerCount() - t_start) < win_ms)
        {
            LoraApp_ProcessIrq();
            rx_len = LoraApp_GetRx(s_rx, (uint16_t)sizeof(s_rx));
            if (rx_len > 0U)
            {
                got_any = true;
                app_pair_dump("[pair] rx", s_rx, rx_len);

                if (app_pair_handle_rx(s_rx, rx_len, rt.dev_addr))
                {
                    /* ---- ③ 绑定成功：网关地址已写进 Flash/RAM ---- */
                    app_runtime_state_get(&rt);     /* 重新取，里面才是刚绑上的地址 */
                    app_pair_dump("[pair] BOUND to gw_addr", rt.gw_addr, PROTO_ADDR_LEN);

                    /* ---- ④ 持续发 10s 的 0x80 ACK（带已绑定的网关地址），然后返回 ---- */
                    app_pair_ack_window(rt.dev_addr);
                    return true;
                }

                /* 这一帧不是有效应答 -> 继续把窗口收满 */
                LoraApp_ResumeRx();
                continue;
            }
            delay_ms(1U);
        }
        LoraApp_Standby();

        /* ---- ⑤ 补满周期：保证空口上始终是"每 APP_PAIR_PERIOD_MS 一帧" ---- */
        while ((uint32_t)(LLCC68GetTimerCount() - t_start) < APP_PAIR_PERIOD_MS)
        {
            delay_ms(1U);
        }
        if (!got_any)
        {
            log_info("[pair] no answer this round, retry\r\n");
        }
    }
}

