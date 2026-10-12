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

/* ==================== 配对时序（测试阶段：不考虑低功耗） ==================== */
/* 发送周期(ms)：需求是"每 5s 发送一次" */
#ifndef APP_PAIR_PERIOD_MS
#define APP_PAIR_PERIOD_MS      5000U
#endif
/* 接收窗口(ms)：从"准备发这一帧"开始算，一直收到下一帧该发的时刻。
 * 默认 = 发送周期，也就是【发完就把接收盖满整个间隔】，中间不留空档：
 * 发一帧本身要 ~0.5s(收发模式切换 LORA_TX_MODE_DELAY_MS)，所以实际在听约 4.5s，
 * 到点立刻发下一帧 -> 空口上就是严格的 5s 一帧。
 * ⚠️ 不要把窗口调小：网关回 ACK 可能带 3s 延时(GW_REPLY_ACK_DELAY_MS)，
 *    窗口和周期之间留空档的话，ACK 会每次都落在空档里，永远配不上。 */
#ifndef APP_PAIR_RX_WIN_MS
#define APP_PAIR_RX_WIN_MS      5000U
#endif
/* 等 TxDone 超时(ms) */
#define APP_PAIR_TX_TIMEOUT_MS  3000U
/* 接收缓存 */
#define APP_PAIR_RX_BUF_MAX     64U

/* 配对请求帧总长：帧头2 + 版本1 + 网关3 + 设备3 + 功能码1 + data_len1 + CRC2 = 13 */
#define APP_PAIR_FRAME_LEN      13U

/* ==================== 绑定成功之后的行为（2026-09-28 新增需求） ====================
 * 需求原话："网关回应ack后将从机内部存储网关值改为该网关，之后从机要一直回ack，
 *           等待下次上电就可以进入正常流程"
 *
 *   APP_PAIR_STAY_AFTER_BOUND = 1（【默认，按需求】）
 *       绑定成功【不返回、不进正常流程】：留在配对模式里一直守着，
 *       对收到的每一帧有效下行都回一帧 0x80 ACK；直到【下次上电】
 *       因为 Flash 里已有有效网关地址，才直接进正常流程（上报/休眠）。
 *   APP_PAIR_STAY_AFTER_BOUND = 0
 *       老行为：绑定成功就返回，本次上电立刻进正常流程。
 *
 *   APP_PAIR_KEEP_REQUEST_AFTER_BOUND = 1（【默认】）
 *       绑定成功后仍然每 APP_PAIR_PERIOD_MS 发一次 0x04 配对请求。
 *       好处：网关重启/丢了绑定关系时能自动重新绑上；代价：空口上一直有配对帧。
 *   = 0
 *       绑定成功后不再发 0x04，只"一直收 + 一直回 ACK"（严格按"之后一直回ack"）。
 * ============================================================================ */
#ifndef APP_PAIR_STAY_AFTER_BOUND
#define APP_PAIR_STAY_AFTER_BOUND           1
#endif
#ifndef APP_PAIR_KEEP_REQUEST_AFTER_BOUND
#define APP_PAIR_KEEP_REQUEST_AFTER_BOUND   1
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

/* 组配对请求帧：AA 55 | 01 | FF FF FF(任意网关) | 本机设备地址 | 04 | 00 | CRC
 * CRC = Modbus(版本字节..数据域末尾)，帧头 AA 55 不参与，低字节在前。
 * 例：设备 21 00 01 -> AA 55 01 FF FF FF 21 00 01 04 00 BC 63 */
static void app_pair_build_request(uint8_t *buf, const uint8_t dev_addr[PROTO_ADDR_LEN])
{
    buf[0] = PROTO_HEADER_0;
    buf[1] = PROTO_HEADER_1;
    buf[2] = PROTO_VER_V1;
    buf[3] = 0xFFU;                                     /* 目的网关：FF FF FF = 任意 */
    buf[4] = 0xFFU;
    buf[5] = 0xFFU;
    buf[6] = dev_addr[0];                               /* 自己的设备地址 */
    buf[7] = dev_addr[1];
    buf[8] = dev_addr[2];
    buf[9] = (uint8_t)CMD_FUNC_PAIR_BIND;               /* 0x04 配对绑定 */
    buf[10] = (uint8_t)APP_CMD_PAIR_DLEN;               /* 载荷为空 */
    /* 覆盖 buf[2..10] 共 9 字节，CRC 落在 buf[11]/buf[12] */
    crc16_modbus_append(&buf[2], APP_PAIR_FRAME_LEN - 4U);
}

/* 目的地址是不是广播(FF FF FF)：有的网关回包用广播地址发 */
static bool app_pair_is_broadcast(const uint8_t addr[PROTO_ADDR_LEN])
{
    return ((addr[0] == 0xFFU) && (addr[1] == 0xFFU) && (addr[2] == 0xFFU));
}

/* 绑定：更新运行时状态（+ 可选落 Flash）
 * @param gw_addr    配对得到的网关地址
 * @param save_flash true  = 正式流程：先写 Flash 成功再更新 RAM。
 *                           顺序很重要，否则 Flash 失败但 RAM 已经"绑上了"，
 *                           表现为"这次跑主程序，一重启又回配对"。
 *                   false = 台架测试（MAIN_MODE_PAIR_TEST）：只改 RAM，不写 Flash，
 *                           所以下次上电还要重新配对。 */
static bool app_pair_bind(const uint8_t gw_addr[PROTO_ADDR_LEN], bool save_flash)
{
    if (save_flash)
    {
        if (!app_dev_param_set_gw_addr(gw_addr))
        {
            log_info("[pair] bind FAILED: flash write error, keep pairing\r\n");
            return false;
        }
    }
    app_runtime_state_set_gw_addr(gw_addr);
    return true;
}

/* 校验一帧是否是"发给本机的完整有效帧"（帧头/版本/长度/CRC/目的地址/网关地址）
 * @param rx       收到的原始帧
 * @param len      长度
 * @param dev_addr 本机设备地址
 * @retval true = 有效且是发给本机的（帧头 gw_addr[3..5] 也是有效地址）
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
    data_len = rx[10];
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
    /* 目的地址：本机 或 广播(FF FF FF) 都收 */
    if ((!proto_addr_equal(&rx[6], dev_addr)) && (!app_pair_is_broadcast(&rx[6])))
    {
        log_info("[pair] rx: not for this device, ignore\r\n");
        return false;
    }
    /* 网关地址必须有效：应答帧头 [3..5] 就是网关自己的地址，配对绑定的就是它。
     * 如果网关把目的地址 FF FF FF 原样回填，就取不到它的地址，只能继续重试。 */
    if (!proto_addr_is_valid(&rx[3]))
    {
        log_info("[pair] rx: gw_addr=%02X%02X%02X invalid (gateway must fill its own addr)\r\n",
                 (unsigned)rx[3], (unsigned)rx[4], (unsigned)rx[5]);
        return false;
    }
    return true;
}

/* 已绑定之后：回一帧 0x80 ACK（13 字节）
 *   AA 55 | 01 | gw(3，取自收到的帧) | 本机dev(3) | 80 | 00 | CRC
 * @note  同步等 TX_DONE 再返回，避免"发一半被切走"；发完由调用方 LoraApp_ResumeRx()
 *        把接收重新打开。 */
static void app_pair_send_ack(const uint8_t gw_addr[PROTO_ADDR_LEN],
                              const uint8_t dev_addr[PROTO_ADDR_LEN])
{
    uint8_t ack[APP_PAIR_FRAME_LEN];

    ack[0]  = PROTO_HEADER_0;
    ack[1]  = PROTO_HEADER_1;
    ack[2]  = PROTO_VER_V1;
    ack[3]  = gw_addr[0];
    ack[4]  = gw_addr[1];
    ack[5]  = gw_addr[2];
    ack[6]  = dev_addr[0];
    ack[7]  = dev_addr[1];
    ack[8]  = dev_addr[2];
    ack[9]  = (uint8_t)CMD_FUNC_ACK_OK;
    ack[10] = 0U;
    /* 覆盖 [2..10] 共 9 字节，CRC 落在 [11]/[12] */
    crc16_modbus_append(&ack[2], APP_PAIR_FRAME_LEN - 4U);

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
 * @param rx         收到的原始帧
 * @param len        长度
 * @param dev_addr   本机设备地址
 * @param save_flash 绑定是否落 Flash（见 app_pair_bind）
 * @retval true = 配对成功，可以回主程序；false = 继续等 */
static bool app_pair_handle_rx(const uint8_t *rx, uint16_t len,
                               const uint8_t dev_addr[PROTO_ADDR_LEN], bool save_flash)
{
    uint8_t func;

    if (!app_pair_validate(rx, len, dev_addr))
    {
        return false;
    }

    func = rx[9];
    if (func == (uint8_t)CMD_FUNC_ACK_ERR_RETRY)
    {
        log_info("[pair] gateway rejected (0x81), keep pairing\r\n");
        return false;
    }
    /* 0x80 = 正常应答（网关通用 ACK）；也有网关按原功能码 0x04 回，两种都认。
     * 载荷不看：绑定的地址取自帧头的 gw_addr[3..5]。 */
    if ((func != (uint8_t)CMD_FUNC_ACK_OK) && (func != (uint8_t)CMD_FUNC_PAIR_BIND))
    {
        log_info("[pair] rx: func=0x%02X is not a pairing response, ignore\r\n", (unsigned)func);
        return false;
    }

    log_info("[pair] gateway answered: func=0x%02X gw_addr=%02X%02X%02X\r\n",
             (unsigned)func, (unsigned)rx[3], (unsigned)rx[4], (unsigned)rx[5]);
    return app_pair_bind(&rx[3], save_flash);
}

/* 配对模式主循环（内部实现）
 * @param save_flash true = 绑定时写 Flash（正式流程）；false = 只改 RAM（台架测试）
 * @note  行为由两个宏决定（见文件上方说明）：
 *          APP_PAIR_STAY_AFTER_BOUND = 1（默认）：绑定成功后【不返回】，
 *            留在本函数里一直回 ACK，直到下次上电才进正常流程；
 *          = 0：绑定成功就返回（老行为）。
 *        一直绑不上就一直循环，不返回。 */
static void app_pair_run_ex(bool save_flash)
{
    AppRuntimeState_t rt;
    uint16_t rx_len;
    uint32_t t_start;
    uint32_t win_ms;
    bool     got_any;
    bool     bound = false;             /* false = 还没绑上；true = 已绑，之后只回 ACK */

    app_runtime_state_get(&rt);
    app_pair_build_request(s_req, rt.dev_addr);

    log_info("\r\n===== pairing mode (unbound) =====\r\n");
    log_info("[pair] dev_addr=%02X%02X%02X gw_addr=%02X%02X%02X -> need binding\r\n",
             (unsigned)rt.dev_addr[0], (unsigned)rt.dev_addr[1], (unsigned)rt.dev_addr[2],
             (unsigned)rt.gw_addr[0], (unsigned)rt.gw_addr[1], (unsigned)rt.gw_addr[2]);
    log_info("[pair] send 0x04 every %ums, listen %ums each time\r\n",
             (unsigned)APP_PAIR_PERIOD_MS, (unsigned)APP_PAIR_RX_WIN_MS);
    log_info("[pair] bind target: %s; after bound: %s\r\n",
             save_flash ? "runtime + flash (production)" : "runtime only, NOT saved to flash (bench)",
             (APP_PAIR_STAY_AFTER_BOUND != 0)
                 ? "stay + keep ACKing (normal flow starts at next power-on)"
                 : "leave pairing mode and run normal flow now");
    app_pair_dump("[pair] request frame", s_req, (uint16_t)APP_PAIR_FRAME_LEN);

    LoraApp_Init();
    LoraApp_Standby();

    while (1)
    {
        t_start = LLCC68GetTimerCount();

        /* ---- ① 发配对请求 ----
         * SendSync 只有在【芯片报了 TX_DONE】时才返回 true：那一刻 PA ramp + 调制
         * 已经跑完，数据真的从芯片发出去了（天线/匹配网络是另一回事）。
         * 所以这一行日志就是"我们这边到底发没发"的分界线：
         *   打 "sent"                -> 我们发了，收不到要去对方/射频参数上找原因；
         *   打 "TX FAILED: no TX_DONE" -> 芯片没发完，查供电/天线/PA 配置。
         * 绑定之后是否还继续发：见 APP_PAIR_KEEP_REQUEST_AFTER_BOUND。 */
        if ((!bound) || (APP_PAIR_KEEP_REQUEST_AFTER_BOUND != 0))
        {
            if (LoraApp_SendSync(s_req, (uint8_t)APP_PAIR_FRAME_LEN, APP_PAIR_TX_TIMEOUT_MS))
            {
                log_info("[pair] pairing request sent (chip reports TX_DONE)\r\n");
            }
            else
            {
                log_info("[pair] !! TX FAILED: no TX_DONE (nothing left the chip) - check power/antenna/PA\r\n");
            }
        }

        /* ---- ② 开接收窗口 ----
         * 未绑定 / 绑定后还要继续发请求：窗口 = APP_PAIR_RX_WIN_MS，到点回来发下一帧。
         * 绑定后不再发请求：窗口 = 无限，一直收着，随时回 ACK（不再回外层循环）。 */
        got_any = false;
        win_ms  = APP_PAIR_RX_WIN_MS;
        if (bound && (APP_PAIR_KEEP_REQUEST_AFTER_BOUND == 0))
        {
            win_ms = 0xFFFFFFFFU;
            log_info("[pair] bound: keep listening + ACKing (no more 0x04)\r\n");
        }

        LoraApp_ReciveMode();
        while ((uint32_t)(LLCC68GetTimerCount() - t_start) < win_ms)
        {
            LoraApp_ProcessIrq();
            rx_len = LoraApp_GetRx(s_rx, (uint16_t)sizeof(s_rx));
            if (rx_len > 0U)
            {
                got_any = true;
                app_pair_dump("[pair] rx", s_rx, rx_len);

                if (!bound)
                {
                    if (app_pair_handle_rx(s_rx, rx_len, rt.dev_addr, save_flash))
                    {
                        /* ---- ③ 绑定成功：网关地址已写进 Flash/RAM ---- */
                        bound = true;
                        app_runtime_state_get(&rt);     /* 重新取，里面才是刚绑上的地址 */
                        log_info("[pair] BOUND to gw_addr=%02X%02X%02X (%s)\r\n",
                                 (unsigned)rt.gw_addr[0], (unsigned)rt.gw_addr[1],
                                 (unsigned)rt.gw_addr[2],
                                 save_flash ? "saved to flash" : "RAM only, not saved");

#if (APP_PAIR_STAY_AFTER_BOUND)
                        /* 按需求：绑定后【不返回、不进正常流程】，留在这里一直回 ACK，
                         * 下次上电因为 Flash 里已有有效网关地址，才直接进正常流程。 */
                        log_info("[pair] stay in pairing mode, keep ACKing; REBOOT to run normal flow\r\n");
#else
                        log_info("[pair] leave pairing mode -> main flow\r\n");
                        LoraApp_Standby();
                        return;
#endif
                    }
                }
                else
                {
                    /* ---- ④ 已绑定：对收到的每一帧有效下行都回一帧 0x80 ACK ----
                     * 判据与绑定判定共用 app_pair_validate()（帧头/版本/长度/CRC/目的地址/网关地址），
                     * 所以"垃圾帧/别人的帧/网关地址非法"都不会被回 ACK。 */
                    if (app_pair_validate(s_rx, rx_len, rt.dev_addr))
                    {
                        log_info("[pair] got func=0x%02X from gw=%02X%02X%02X -> reply ACK\r\n",
                                 (unsigned)s_rx[9],
                                 (unsigned)s_rx[3], (unsigned)s_rx[4], (unsigned)s_rx[5]);
                        app_pair_send_ack(&s_rx[3], rt.dev_addr);
                    }
                }

                /* 这一帧处理完（含回 ACK，回包已经把发射做完）-> 继续把窗口收满 */
                LoraApp_ResumeRx();
                continue;
            }
            delay_ms(1U);
        }
        LoraApp_Standby();

        /* ---- ⑤ 补满周期：窗口若被调小于周期，这里把剩下的时间等满，
         *      保证空口上始终是"每 APP_PAIR_PERIOD_MS 一帧"。默认两者相等，这里不等待。 ---- */
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

/* ---- 对外两个入口：正式流程落 Flash / 台架测试只改 RAM ---- */
void app_pair_run(void)
{
    app_pair_run_ex(true);
}

void app_pair_run_ramonly(void)
{
    app_pair_run_ex(false);
}
