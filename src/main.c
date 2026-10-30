/**
 * @file main.c
 * @brief N32G031 液位传感器节点 —— 主流程（配对 / 判水 / 测液位 / LoRa 上报 / 低功耗）
 *
 * 一轮：
 *   ① 未绑定 -> 进 app_factory_run()（等水唤醒，短接触发配对，不返回）
 *   ② 恢复外设域电源 -> ③ 探针判水（顺带决定休眠态 A/B）
 *   ④ 有水才测液位，记录本周期最大值 -> ⑤ 上报判定（超阈值立即报 / 到点才报）
 *   ⑥ 仅上报轮开 win1/win2/win3 三段接收窗（收指令并回 ACK）
 *   ⑦ 睡前把云端配置落 Flash，再进 STOP（有水按 d 巡检，无水睡到上报点靠 EXTI 等水）
 *
 * 周期口径：网关只下发上报间隔 report（60~16777215s），检测间隔 d = ceil(report/3)，
 *   只有 d > 2h 时才夹到 2h（规则见 app_cmd.h【检测间隔的推导规则】）。
 *   上报发本周期最大值，发完清零；任何一次检测超阈值就立刻上报。
 *   未校时没有墙上时间，退化成"检测够 ceil(report/d) 次就报一次"（首帧仍立刻发）。
 *
 * 硬件铁律（踩过坑，别动）：
 *   - PB7 外设总电源（低=上电）。驱动高/纯高阻都会让外设域失电并触发整板 POR/BOR 复位；
 *     STOP 期间配输入上拉（P-MOS 关断、外设域 0V、几 µA）。见 bsp_gpio.h。
 *   - PA7 探针激励、PB1 探针检测；PB1 只允许输入+上下拉，永不做推挽输出。
 *   - PA9 液位模块 TRIG。进 STOP 前必须钉低，否则悬空会被噪声误触发 -> 电流脉冲 -> 复位。
 *   - USART1 一线两用：日志 115200 / 超声 9600。Level_UART_Init() 会把波特率切到 9600，
 *     采完必须 Level_UartSetBaud(LOG_BAUD) 切回来，否则后续打印全是乱码。
 *   - 射频固定走 SX1262 / HP PA 分支：模块自报 SX1261，但射频通路接在 SX1262 那侧。
 *   - 不能开 IWDG：它挂在 LSI 上，STOP 期间照样计数，睡久了必被复位。
 *   - RTC(Line20) 的 EXTI 挂起位必须在中断里手动清，见 n32g031_it.c。
 *
 * 日志约定：所有打印字符串用英文，避免中文字符串被编辑器按 ANSI 存成乱码。
 */
#include "n32g031_usart.h"
#include "n32g031_gpio.h"
#include "n32g031_rcc.h"
#include "misc.h"
#include "project_config.h"
#include "log.h"
#include "delay.h"
#include "LLCC68-board.h"
#include <string.h>                     /* memcpy */

#include "app_lora.h"
#include "app_sensor.h"
#include "app_cmd.h"
#include "app_pair.h"                   /* 配对模式 */
#include "app_time.h"
#include "app_dev_param.h"
#include "app_runtime_state.h"
#include "app_lowpower.h"
#include "protocol_util.h"              /* proto_addr_is_valid_gw()：判断网关地址是否已绑定 */
#include "bsp_gpio.h"
#include "bsp_adc.h"
#include "bsp_rtc.h"
#include "water_cu.h"
#include "level_uart.h"

/* ==================== 通用参数 ==================== */
#define APP_TX_DONE_TIMEOUT_MS  3000U   /* 等 TxDone 的超时(ms) */
#define APP_LEVEL_TIMEOUT_MS    1000U   /* 超声测液位最长等待(ms) */
#define APP_RX_BUF_MAX          128U

/* ==================== 外设域供电（PB7 高边 P-MOS，低=上电） ====================
 *   醒着：BSP_Periph_Power_On() 驱动低 -> 外设域 3.3V
 *   STOP：BSP_Periph_Power_PullUp() 配输入上拉 -> 栅极拉高、外设域 0V（几 uA）
 *   唤醒：驱动低拉回 3.3V + 延时等 POR + 重新 Init（统一走 app_periph_power_restore）
 * ⚠️ 探针两根脚(PA7/PB1)是 MCU 自己的 IO，不挂外设域，所以关断不影响"有水自动唤醒"。
 * ============================================================================ */
#ifndef APP_PERIPH_POR_MS
/* 唤醒后等外设 POR 冷启动的延时(ms)。
 * 睡前外设域被关断到 0V，睡 40s~5min 后 500ms 不够（实测超声波没反应），所以用 1000；
 * 还不够就继续加到 2000/3000（超声波模组自己的 MCU 启动 + 晶振稳定要时间）。 */
#define APP_PERIPH_POR_MS       1000U
#endif

/* 睡前进 STOP（PB7 输入上拉，外设域关断到 0V） */
static void app_flow_enter_stop(void)
{
    app_lowpower_enter_stop_pullup();
}

/* 唤醒后第一件事：恢复外设域电源。
 * 顺序讲究：① 先把 TRIG(PA9) 钉成空闲低 —— 模块冷启动 POR 期间 TRIG 若悬空，
 *              可能被它当成一次触发，一上电就空打一发（大电流脉冲 + 状态错乱）；
 *            ② 再 PB7 驱动低，把外设域从 0V 拉回 3.3V；
 *            ③ 等 POR 冷启动（APP_PERIPH_POR_MS）。
 * RTC 唤醒和探针唤醒都必须走这里（每轮第一步，比配对检查还早）。 */
static void app_periph_power_restore(void)
{
    /* 睡前外设域被关断到 0V，唤醒后必须先驱动低拉回 3.3V + 等 POR，再碰外设 */
    Level_TrigHoldLow();
    BSP_Periph_Power_On();
    log_info("[app] periph restore: TRIG=low, PB7=drive-low, %ums POR settle\r\n",
             (unsigned)APP_PERIPH_POR_MS);
    delay_ms(APP_PERIPH_POR_MS);
}

/* ==================== 定义在后面的函数（先声明） ==================== */
static void app_dump_hex(const uint8_t *buf, uint16_t len);

/* ============================================================================
 *                                时钟 / 复位原因
 * ============================================================================ */
void SystemClock_Config(void)
{
    uint32_t timeout_value = 0xFFFFFF;

    RCC_DeInit();                       /* 复位RCC配置 */
    RCC_EnableHsi(ENABLE);              /* 内部高速晶振 HSI (8MHz) */

    if (RCC_WaitHsiStable() == SUCCESS)
    {
        FLASH_PrefetchBufSet(FLASH_PrefetchBuf_EN);
        FLASH_SetLatency(FLASH_LATENCY_0);

        RCC_ConfigHclk(RCC_SYSCLK_DIV1);
        RCC_ConfigPclk2(RCC_HCLK_DIV1);
        RCC_ConfigPclk1(RCC_HCLK_DIV1);

        RCC_ConfigSysclk(RCC_SYSCLK_SRC_HSI);
        while (RCC_GetSysclkSrc() != RCC_CFG_SCLKSTS_HSI)
        {
            if ((timeout_value--) == 0)
            {
                break;
            }
        }
    }
}

/* 打印上次复位原因（定位"反复重启"）：POR/BOR=供电跌落, IWDG=看门狗, ... */
static void print_reset_cause(void)
{
    const char *cause = "UNKNOWN";

    if (RCC_GetFlagStatus(RCC_CTRLSTS_FLAG_IWDGRSTF) != RESET)      { cause = "IWDG"; }
    else if (RCC_GetFlagStatus(RCC_CTRLSTS_FLAG_WWDGRSTF) != RESET) { cause = "WWDG"; }
    else if (RCC_GetFlagStatus(RCC_CTRLSTS_FLAG_PORRSTF) != RESET)  { cause = "POR/BOR"; }
    else if (RCC_GetFlagStatus(RCC_CTRLSTS_FLAG_PINRSTF) != RESET)  { cause = "NRST_PIN"; }
    else if (RCC_GetFlagStatus(RCC_CTRLSTS_FLAG_SFTRSTF) != RESET)  { cause = "SOFTWARE"; }
    else if (RCC_GetFlagStatus(RCC_CTRLSTS_FLAG_LPWRRSTF) != RESET) { cause = "LOWPOWER"; }

    log_info("RST_CAUSE=%s\r\n", cause);
    RCC_ClrFlag();                      /* 清掉标志，下次复位才能判断正确 */
}

/* ==================== 打印小工具 ==================== */
static void app_dump_hex(const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    for (i = 0U; i < len; i++)
    {
        log_info("%02X ", (unsigned)buf[i]);
    }
    log_info("\r\n");
}

static void app_print_params(void)
{
    AppRuntimeState_t rt;

    app_runtime_state_get(&rt);
    /* 地址是 13 字节(0x21/0x11 + 96 位 UID)，逐字节打出来核对；网关地址 13×FF = 未绑定 */
    log_info("[app] dev_addr (0x21+UID)   :");
    for (uint8_t i = 0U; i < PROTO_ADDR_LEN; i++)
    {
        log_info(" %02X", (unsigned)rt.dev_addr[i]);
    }
    log_info("\r\n[app] gw_addr  (0x11+UID)   :");
    for (uint8_t i = 0U; i < PROTO_ADDR_LEN; i++)
    {
        log_info(" %02X", (unsigned)rt.gw_addr[i]);
    }
    log_info("  (%s)\r\n", proto_addr_is_valid_gw(rt.gw_addr) ? "BOUND" : "UNBOUND");
    if (!proto_addr_is_valid_dev(rt.dev_addr))
    {
        log_info("[app] !! dev_addr looks invalid (expect 0x21 prefix + non-zero UID)\r\n");
    }
    /* 设备字段（0x06 写入 / 0x07 读取）：上电打一次，方便核对掉电是否保住 */
    {
        uint8_t field[APP_DEV_FIELD_LEN];
        uint8_t fi;

        app_dev_param_get_field(field);
        log_info("[app] field (10B ASCII)   :");
        for (fi = 0U; fi < APP_DEV_FIELD_LEN; fi++)
        {
            log_info(" %02X", (unsigned)field[fi]);
        }
        log_info("\r\n");
    }
    /* detect 不落 Flash（是 report 的派生量），这里现算一行给现场核对 */
    log_info("[app] cfg: report=%lus -> detect=%us same_lvl_report=%u alarm_thresh=%umm\r\n",
             (unsigned long)rt.sensor_cfg.report_interval_s,
             (unsigned)app_cmd_detect_from_report_s((uint32_t)rt.sensor_cfg.report_interval_s),
             (unsigned)rt.sensor_cfg.same_level_report_en,
             (unsigned)rt.sensor_cfg.level_alarm_thresh_mm);
    /* 0x01 的测量类参数：决定模块读数怎么变成上报值（见 app_sensor_convert_level）
     *   LEVEL：读数直接当液位；DIST：液位 = 高度×10 − 距离
     *   offset：叠加到最终值的校准偏移（可负）；lowbat：只存不用，判定在网关侧 */
    log_info("[app] cfg2: measure_mode=%u(%s) install_height=%ucm calib_offset=%dmm low_batt=%u%%\r\n",
             (unsigned)rt.sensor_cfg.measure_mode,
             (rt.sensor_cfg.measure_mode == DEV_MEASURE_MODE_DIST) ? "DIST" : "LEVEL",
             (unsigned)rt.sensor_cfg.install_height_cm,
             (int)rt.sensor_cfg.calib_offset_mm,
             (unsigned)rt.sensor_cfg.low_battery_pct);
    /* 启动时把推导规则打一次：0x01 只带上报间隔，检测间隔永远现算
     * （d = ceil(report/3)，超过 2h 夹到 2h；0x01 载荷固定 18 字节，长度不符回 0x86） */
    log_info("[app] rule: 0x01(18B) report in [%u,%lu]s -> detect = ceil(report/%u), cap %us\r\n",
             (unsigned)APP_CMD_REPORT_INTERVAL_MIN_S,
             (unsigned long)APP_CMD_REPORT_INTERVAL_MAX_S,
             (unsigned)APP_CMD_DETECT_FROM_REPORT_DIV,
             (unsigned)APP_CMD_DETECT_INTERVAL_MAX_S);
}

static void app_print_time(void)
{
    bsp_rtc_calendar_t cal;
    bsp_rtc_calendar_t bj;

    if (app_time_is_valid() != 0U)
    {
        /* RTC 存 UTC（0x01 校时把北京时间 -8h 存进来），两个都打出来免得误判；
         * 协议字段用的是北京时间 */
        app_time_get_calendar(&cal);
        log_info("[app] time(UTC)=%04u-%02u-%02u %02u:%02u:%02u week%u ts=%u\r\n",
                 (unsigned)cal.year, (unsigned)cal.month, (unsigned)cal.day,
                 (unsigned)cal.hour, (unsigned)cal.minute, (unsigned)cal.second,
                 (unsigned)cal.week, (unsigned)app_time_get_unix_ts());
        if (app_time_get_bj_calendar(&bj) == 0U)
        {
            log_info("[app] time(BJ+8h)=%04u-%02u-%02u %02u:%02u (上报帧用这个)\r\n",
                     (unsigned)bj.year, (unsigned)bj.month, (unsigned)bj.day,
                     (unsigned)bj.hour, (unsigned)bj.minute);
        }
    }
    else
    {
        log_info("[app] time=NOT-SYNCED ts=%u\r\n", (unsigned)app_time_get_unix_ts());
    }
}

/* 打印一遍射频参数，便于与配置表逐项核对 */
static void app_print_rf_cfg(void)
{
    log_info("[rf] freq=%luHz SF=%u BWidx=%u CR=%u preamble=%u fixlen=%u iq=%u power=%udBm\r\n",
             (unsigned long)LORA_FRE, (unsigned)LORA_SPREADING_FACTOR,
             (unsigned)LORA_BANDWIDTH, (unsigned)LORA_CODINGRATE,
             (unsigned)LORA_PREAMBLE_LENGTH, (unsigned)LORA_FIX_LENGTH_PAYLOAD_ON,
             (unsigned)LORA_IQ_INVERSION_ON, (unsigned)LORA_TX_OUTPUT_POWER);
}

/* ============================================================================
 *   正式流程 —— 配对 + 液位检测 + 上报接收 + 低功耗
 * ============================================================================ */
/* ==================== 探针 / 液位 / 接收窗参数 ==================== */
/* 电压法判水（water_cu.c）与超声测液位都是常开业务，没有编译期开关：
 * 台面没接硬件时，判水会自然判"干"、测液位会连续读到 0xFFFF（保留上次值），
 * 行为可控，不需要再引入一套条件编译分支。 */

/* 接收窗口与应答时序（测试阶段不考虑功耗，窗口给足） */
#ifndef APP_RX_WIN_MS
#define APP_RX_WIN_MS           10000U  /* 单个接收窗口(ms)：win1/win2/win3 各一个 */
#endif
#ifndef APP_CMD_ACK_DELAY_MS
#define APP_CMD_ACK_DELAY_MS    3000U   /* 收到网关指令后等多久再回 ACK(ms) */
#endif

/* ==================== 0x05 解绑的应答节奏 ====================
 * 收到解绑后先回两次 ACK（间隔 5s），再清网关地址、进 STOP 等下次上电。
 * 为什么两次：网关发完下行要切到接收，单次 ACK 可能正好落在它的切换盲区里；
 * 隔 5s 再发一次，两次总有一次能被收到（也顺便覆盖网关的重发/阶段机）。
 * 顺序铁律：两次 ACK 都发完【之后】才能清 gw_addr —— 清了地址就再也组不出应答帧。
 * ============================================================================ */
#ifndef APP_UNBIND_ACK_COUNT
#define APP_UNBIND_ACK_COUNT        2U
#endif
#ifndef APP_UNBIND_ACK_GAP_MS
#define APP_UNBIND_ACK_GAP_MS       5000U   /* 两次 ACK 之间的间隔(ms) */
#endif
/* 检测间隔兜底值(s)：只在推导结果异常时兜底（正常路径由 app_flow_get_sleep_s() 推导） */
#ifndef APP_SLEEP_DEFAULT_S
#define APP_SLEEP_DEFAULT_S     600U
#endif
/* STOP 唤醒周期的【最小安全值】(s)：RTC 唤醒周期太小会让设备一直在醒着。
 * 这里只是"防 0"的硬地板，不是业务规则：检测间隔已经完全由云端上报间隔推导
 * （见 app_cmd.h），60s 上报就是 20s 检测，不会再被这里拉高。 */
#ifndef APP_SLEEP_MIN_S
#define APP_SLEEP_MIN_S         5U
#endif

/* ==================== 双态休眠：定时唤醒 / 有水唤醒 ====================
 * 两种休眠按"探针有没有水"切换，不并存：
 *   B 态（有水，定时唤醒）：RTC 周期 = d，每次醒都测液位并记最大值；
 *        到上报点或超阈值才开射频；探针 EXTI 撤防（已经湿了，不用等）。
 *   A 态（无水，有水唤醒，默认）：RTC 只睡到下一个上报点，醒来判一次水、报当前值；
 *        探针 EXTI 武装（上升沿），水一来立刻叫醒并切回 B。
 *        两次上报之间不按 d 巡检 —— 没水可测，多醒纯属浪费。
 * 切换：B -> A 需连续 APP_DRY_CONFIRM_CNT 次判无水（防水面波动误判）；A -> B 任一次判有水。
 * ============================================================================ */
#ifndef APP_DRY_CONFIRM_CNT
#define APP_DRY_CONFIRM_CNT     2U      /* 连续几次判无水才退回"有水唤醒" */
#endif

/* ==================== 上报节奏 ====================
 * 网关只下发上报间隔，检测间隔 = ceil(上报/3)（一个周期检测 3 次，第 3 次就是开窗上报那次）。
 * 每次苏醒（= 一次检测）只有两种结果：
 *   ① level >= 阈值 -> 立刻上报，且持续超限就每次检测都报（APP_ALARM_REPEAT）
 *   ② 没超 -> 只有到上报点才报：已校时按"距上次上报 >= 上报间隔"，
 *             未校时按"距上次上报已检测 det_per_report 次"（正常 3 次）
 *   不报的轮次只测液位、更新最大值，不开射频、不开接收窗。
 * 时间基准用 RTC 秒，RTC 在 STOP 下继续走，跨轮/跨睡都在。
 * ============================================================================ */
/* 上报周期兜底(s)：运行时状态里的上报间隔非法时用（出厂默认 30min，见 app_dev_param.c） */
#ifndef APP_REPORT_DEFAULT_S
#define APP_REPORT_DEFAULT_S    1800U
#endif
/* 为"对齐上报点"而缩短睡眠时的最小睡眠(s)：防止算出 0 秒把 RTC 唤醒周期设成 0 */
#ifndef APP_SLEEP_ALIGN_MIN_S
#define APP_SLEEP_ALIGN_MIN_S   1U
#endif

/* --- 本轮收发缓存（static：本板栈只有 1KB，Stack_Size=0x400，见 startup_n32g031.s） --- */
static uint8_t  s_rx_buf[APP_RX_BUF_MAX];
static uint8_t  s_ack_buf[APP_CMD_BUF_MAX];
static uint16_t s_rx_len = 0U;

/* 待发送的应答（"收到指令后等 3s 再回 ACK"用） */
static uint8_t  s_reply_buf[APP_CMD_BUF_MAX];
static uint16_t s_reply_len = 0U;
static uint32_t s_reply_due = 0U;       /* 该发的时刻(LLCC68 毫秒计时) */
static bool     s_reply_wait = false;   /* true = 有一帧应答在等时间到 */

/* 时间到/被强制时，把待发的应答发出去（同步等 TxDone）
 * @param force     true = 不管到没到点都发（窗口收尾兜底）
 * @param resume_rx true = 发完重新打开接收（窗口内用）；false = 收尾，后面就进睡 */
static bool app_cmd_tick_ack(bool force, bool resume_rx)
{
    uint32_t now = LLCC68GetTimerCount();

    if (!s_reply_wait)
    {
        return false;
    }
    /* 无符号回绕安全的"到点了"判断 */
    if ((!force) && ((uint32_t)(now - s_reply_due) >= 0x80000000U))
    {
        return false;
    }

    s_reply_wait = false;
    log_info("[app] %s: send ACK %u bytes: ", force ? "window-end flush" : "delay done",
             (unsigned)s_reply_len);
    app_dump_hex(s_reply_buf, s_reply_len);

    /* 必须同步等 TxDone：LoraApp_Send() 是异步的，紧接着切 Standby 会在包发完前
     * 把芯片切回待机，表现为"收到消息但 ACK 发不出去、网关收不到回应" */
    if (LoraApp_SendSync(s_reply_buf, (uint8_t)s_reply_len, APP_TX_DONE_TIMEOUT_MS))
    {
        log_info("[app] ACK sent\r\n");
    }
    else
    {
        log_info("[app] ACK send FAILED (no TxDone)\r\n");
    }

    if (resume_rx)
    {
        /* SendSync 内部的 LoraApp_Send() 会把 g_rxForever 清成 false，
         * 之后 LoraApp_ProcessIrq() 不再自动续接收 —— 必须补一句恢复接收，
         * 否则窗口剩下的时间全是"假收"。 */
        LoraApp_ResumeRx();
    }
    return true;
}

/* ============================================================================
 *   0x07 GET_FIELD 的应答流程（网关要读"设备字段"）
 * ----------------------------------------------------------------------------
 *   收到 0x07（dlen=0）后：
 *     ① 立刻发一帧 0x08（dlen=10，data=字段原文），
 *        gw_uid 用【0x07 帧里带来的网关 UID】，dev_uid 用本机；
 *     ② 开 5s 接收窗口，等网关的 0x80；
 *     ③ 5s 内没等到 -> 再发一次 0x08 + 再等 5s；
 *     ④ 之后不管收没收到都回主流程（最多发 2 次 0x08）。
 * ============================================================================ */
#define APP_FIELD_WIN_MS        5000U   /* 每轮等网关 0x80 的窗口 */
#define APP_FIELD_MAX_SENDS     2U      /* 0x08 最多发几次 */

static void app_field_response(const uint8_t *req)
{
    AppRuntimeState_t rt;
    uint8_t  frame[PROTO_FIXED_HEAD_LEN + APP_DEV_FIELD_LEN + PROTO_CRC_LEN];
    uint16_t flen = 0U;
    uint8_t  gw[PROTO_ADDR_LEN];
    uint8_t  field[APP_DEV_FIELD_LEN];
    uint8_t  n;
    uint32_t t0;
    uint16_t rx_len;

    app_runtime_state_get(&rt);
    app_dev_param_get_field(field);
    /* 注意：req 指向 s_rx_buf，下面开窗收包会覆盖它 —— 先把它里面的网关 UID 拷出来 */
    memcpy(gw, &req[PROTO_OFF_GW_ADDR], PROTO_ADDR_LEN);

    if (!app_cmd_build_ack_frame(frame, (uint16_t)sizeof(frame), &flen,
                                 (uint8_t)CMD_FUNC_FIELD_RESP,
                                 gw, rt.dev_addr, field, (uint8_t)APP_DEV_FIELD_LEN))
    {
        log_info("[field] build 0x08 failed\r\n");
        return;
    }
    log_info("[field] reply 0x08 (len=%u), field(hex=ASCII):", (unsigned)flen);
    for (n = 0U; n < (uint8_t)APP_DEV_FIELD_LEN; n++)
    {
        log_info(" %02X", (unsigned)field[n]);   /* 用 hex 打印：小 printf 不依赖 %c */
    }
    log_info("\r\n");

    for (n = 0U; n < (uint8_t)APP_FIELD_MAX_SENDS; n++)
    {
        if (LoraApp_SendSync(frame, (uint8_t)flen, APP_TX_DONE_TIMEOUT_MS))
        {
            log_info("[field] 0x08 sent (%u/%u), wait gateway 0x80 up to %ums\r\n",
                     (unsigned)(n + 1U), (unsigned)APP_FIELD_MAX_SENDS, (unsigned)APP_FIELD_WIN_MS);
        }
        else
        {
            log_info("[field] !! 0x08 send FAILED (no TxDone) (%u/%u)\r\n",
                     (unsigned)(n + 1U), (unsigned)APP_FIELD_MAX_SENDS);
        }

        /* 开窗等网关的 0x80（目的地址 = 本机或广播都算） */
        LoraApp_ReciveMode();
        t0 = LLCC68GetTimerCount();
        while ((uint32_t)(LLCC68GetTimerCount() - t0) < APP_FIELD_WIN_MS)
        {
            LoraApp_ProcessIrq();
            rx_len = LoraApp_GetRx(s_rx_buf, (uint16_t)sizeof(s_rx_buf));
            if (rx_len >= PROTO_FRAME_MIN_LEN)
            {
                if ((s_rx_buf[PROTO_OFF_FUNC] == (uint8_t)CMD_FUNC_ACK_OK) &&
                    (proto_addr_equal(&s_rx_buf[PROTO_OFF_DEV_ADDR], rt.dev_addr) ||
                     proto_addr_is_broadcast(&s_rx_buf[PROTO_OFF_DEV_ADDR])))
                {
                    log_info("[field] gateway ACKed 0x08\r\n");
                    LoraApp_Standby();
                    return;
                }
                log_info("[field] rx func=0x%02X is not 0x80-for-me, keep waiting\r\n",
                         (unsigned)s_rx_buf[PROTO_OFF_FUNC]);
                LoraApp_ResumeRx();
            }
            delay_ms(1U);
        }
        LoraApp_Standby();
    }
    log_info("[field] no ACK after %u sends -> back to main flow\r\n", (unsigned)APP_FIELD_MAX_SENDS);
}

/* app_handle_rx_frame() 的返回：接收窗只把【有效帧】算成"收到东西"，
 * 坏帧/噪声不能当网关应答，否则会误跳过 win2 重传窗 */
#define APP_RX_RESULT_INVALID   0U      /* 帧头/版本/长度/CRC/目的地址任一项不过 -> 丢弃 */
#define APP_RX_RESULT_VALID     1U      /* 有效帧（指令已执行 / 已发 0x08 / 解绑） */
#define APP_RX_RESULT_ACK       2U      /* 有效帧，且是网关对本机上报的应答 0x80/0x81 */

/* 处理一帧收到的下行帧：CRC/目的地址校验 -> 打印 -> 执行命令 -> 登记延时 ACK
 * @retval APP_RX_RESULT_INVALID / _VALID / _ACK，见上面的说明 */
static uint8_t app_handle_rx_frame(void)
{
    uint16_t ack_len = 0U;
    AppCmdRet_t ret;
    bool is_ack;

    /* 功能码在固定头的 func 字段：AA 55 | ver | gw(13) | dev(13) | func | len
     * 偏移见 protocol_util.h 的 PROTO_OFF_FUNC（= 29） */
    is_ack = ((s_rx_len >= PROTO_FRAME_MIN_LEN) &&
              ((s_rx_buf[PROTO_OFF_FUNC] == CMD_FUNC_ACK_OK) ||
               (s_rx_buf[PROTO_OFF_FUNC] == CMD_FUNC_ACK_ERR_RETRY)));

    /* app_cmd_process 内部做 CRC + 目的地址 + 帧格式 全量校验 */
    ret = app_cmd_process(s_rx_buf, s_rx_len, s_ack_buf, sizeof(s_ack_buf), &ack_len);

    if (ret == CMD_RET_ERR_CRC)
    {
        log_info("[app] discard: CRC error (%u bytes)\r\n", (unsigned)s_rx_len);
        return APP_RX_RESULT_INVALID;
    }
    if (ret == CMD_RET_ERR_DST_ADDR)
    {
        /* 把整帧打出来：一眼就能看出网关这次用的【目的地址】到底是什么
         * （本机 21.. / 广播 FF×13 / 还是它自己的 11..），对不上就找网关改。 */
        log_info("[app] discard: not for this device (%u bytes): ", (unsigned)s_rx_len);
        app_dump_hex(s_rx_buf, s_rx_len);
        log_info("[app]   this dev = 21 ..; frame dst = ");
        {
            uint8_t di;
            for (di = 0U; di < PROTO_ADDR_LEN; di++)
            {
                log_info(" %02X", (unsigned)s_rx_buf[PROTO_OFF_DEV_ADDR + di]);
            }
        }
        log_info("\r\n");
        return APP_RX_RESULT_INVALID;
    }

    /* ---- 0x07 GET_FIELD：不用普通 ACK，走专门的 0x08 应答流程 ---- */
    if (ret == CMD_RET_FIELD_REQ)
    {
        app_field_response(s_rx_buf);
        return APP_RX_RESULT_VALID;
    }

    /* 帧校验通过且是发给本机的 */
    log_info("[app] got %u bytes -> ", (unsigned)s_rx_len);
    app_dump_hex(s_rx_buf, s_rx_len);
    log_info("[app] parse ret=%u, reply_len=%u\r\n", (unsigned)ret, (unsigned)ack_len);

    app_print_time();                   /* 网关校时后时间会变，顺手打一下 */

    if ((ret == CMD_RET_OK) && (ack_len > 0U))
    {
        bool is_unbind = ((s_rx_len >= PROTO_FRAME_MIN_LEN) &&
                          (s_rx_buf[PROTO_OFF_FUNC] == CMD_FUNC_UNBIND));

        if (is_unbind)
        {
            uint8_t ack_i;

            /* 0x05 解绑三步走：
             *   ① 回 ACK 两次，间隔 APP_UNBIND_ACK_GAP_MS(5s) —— 必须在清地址之前发完
             *   ② 清 gw_addr = 13×FF（Flash + RAM）
             *   ③ 关 RTC 唤醒 + 关射频 -> 复位（重启后走出厂流程：等水唤醒、短接可重新绑定）
             * 本次运行内不再发 0x04：让网关能判定"设备已解绑"；重新绑定走复位后的出厂流程。 */
            if (s_reply_wait)
            {
                (void)app_cmd_tick_ack(true, true);   /* 先把排队的 ACK 立刻发掉 */
            }

            for (ack_i = 0U; ack_i < (uint8_t)APP_UNBIND_ACK_COUNT; ack_i++)
            {
                if (ack_i != 0U)
                {
                    /* 两次之间等 5s：给网关留出"发完下行 -> 切到接收"的时间 */
                    log_info("[app] unbind: wait %ums before ACK #%u\r\n",
                             (unsigned)APP_UNBIND_ACK_GAP_MS, (unsigned)(ack_i + 1U));
                    delay_ms(APP_UNBIND_ACK_GAP_MS);
                }
                log_info("[app] 0x05 unbind: send ACK #%u/%u (%u bytes): ",
                         (unsigned)(ack_i + 1U), (unsigned)APP_UNBIND_ACK_COUNT,
                         (unsigned)ack_len);
                app_dump_hex(s_ack_buf, ack_len);
                if (LoraApp_SendSync(s_ack_buf, (uint8_t)ack_len, APP_TX_DONE_TIMEOUT_MS))
                {
                    log_info("[app] unbind ACK #%u sent\r\n", (unsigned)(ack_i + 1U));
                }
                else
                {
                    log_info("[app] unbind ACK #%u send FAILED (still continue)\r\n",
                             (unsigned)(ack_i + 1U));
                }
            }

            /* ---- ② 解绑流程：清网关地址(Flash + RAM -> 13×FF) ---- */
            if (app_dev_param_clear_gw_addr())
            {
                uint8_t unbound_addr[PROTO_ADDR_LEN];
                proto_addr_set_unbound(unbound_addr);
                app_runtime_state_set_gw_addr(unbound_addr);
                log_info("[app] unbound: gw_addr saved =");
                for (uint8_t i = 0U; i < PROTO_ADDR_LEN; i++)
                {
                    log_info(" %02X", (unsigned)unbound_addr[i]);
                }
                log_info(" (13x FF)\r\n");
            }
            else
            {
                log_info("[app] unbind: flash write FAILED (gw_addr may persist on next boot)\r\n");
            }

            /* ---- ③ 解绑完成：复位，重启后走"出厂流程"（等水唤醒；短接可重新绑定）----
             * 用复位而不是原地跳转：和"绑定成功后复位"一致，状态最干净。 */
            log_info("[app] unbind done -> reboot into factory flow (wait for water)\r\n");
            app_lowpower_config_wakeup(0U);
            LoraApp_Sleep();
            delay_ms(100U);                 /* 让串口把日志发完 */
            NVIC_SystemReset();
            /* 不会走到这里 */
        }

        /* 延时回 ACK：给网关留出"发完下行 -> 切到接收"的时间。
         * 若上一帧应答还没发出去又来了一帧，先把上一帧立刻发掉，保证每条指令都有 ACK。 */
        if (s_reply_wait)
        {
            (void)app_cmd_tick_ack(true, true);
        }
        memcpy(s_reply_buf, s_ack_buf, ack_len);
        s_reply_len  = ack_len;
        s_reply_due  = LLCC68GetTimerCount() + APP_CMD_ACK_DELAY_MS;
        s_reply_wait = true;
        log_info("[app] cmd executed, ACK after %ums\r\n", (unsigned)APP_CMD_ACK_DELAY_MS);
    }

    return (is_ack ? APP_RX_RESULT_ACK : APP_RX_RESULT_VALID);
}

/* 接收窗口：窗口内什么都收，收到就解析打印
 * @retval true = 至少收到一帧【有效帧】（CRC/目的地址都过），噪声/坏帧不算 */
static bool app_rx_window(uint32_t window_ms, const char *tag)
{
    uint32_t t0      = LLCC68GetTimerCount();
    bool     got_any = false;
    uint16_t frames  = 0U;

    log_info("[app] %s: open RX window %ums\r\n", tag, (unsigned)window_ms);
    LoraApp_ReciveMode();

    while ((uint32_t)(LLCC68GetTimerCount() - t0) < window_ms)
    {
        LoraApp_ProcessIrq();
        s_rx_len = LoraApp_GetRx(s_rx_buf, sizeof(s_rx_buf));

        if (s_rx_len > 0U)
        {
            /* 收到满长度 = 可能被射频层 LORA_RX_BUF_LEN 截断（当前最长下行 43B，还够用） */
            if (s_rx_len >= LORA_RX_BUF_LEN)
            {
                log_info("[app] !! rx frame may be truncated at %u bytes (LORA_RX_BUF_LEN)\r\n",
                         (unsigned)s_rx_len);
            }
            if (app_handle_rx_frame() != APP_RX_RESULT_INVALID)
            {
                got_any = true;         /* 只有 CRC/地址都过的帧才算"收到" */
                frames++;
            }
            else
            {
                log_info("[app] %s: invalid frame ignored (not counted)\r\n", tag);
            }
            LoraApp_ResumeRx();         /* 收完继续把窗口剩下的时间收满 */
            continue;
        }

        (void)app_cmd_tick_ack(false, true);    /* 3s 到点就把 ACK 发出去 */
        delay_ms(1U);
    }

    LoraApp_Standby();
    log_info("[app] %s: RX window closed, %s (frames=%u)\r\n",
             tag, got_any ? "got frame(s)" : "no frame at all", (unsigned)frames);
    return got_any;
}

/* 发一帧液位上报并等 TxDone */
static bool app_send_report(uint16_t level_mm)
{
    uint16_t frame_len = 0U;

    if (!app_sensor_build_level_report(level_mm, s_ack_buf, &frame_len))
    {
        log_info("[app] build report frame failed (level=%u)\r\n", (unsigned)level_mm);
        return false;
    }

    log_info("[app] tx report len=%u: ", (unsigned)frame_len);
    app_dump_hex(s_ack_buf, frame_len);

    if (!LoraApp_SendSync(s_ack_buf, (uint8_t)frame_len, APP_TX_DONE_TIMEOUT_MS))
    {
        log_info("[app] send FAILED (no TxDone)\r\n");
        return false;
    }
    log_info("[app] send done, %u bytes\r\n", (unsigned)frame_len);
    return true;
}

/* 探针判定：本轮是否有水（电压法，见 water_cu.c） */
static bool app_probe_has_water(void)
{
    return (WaterDetect_Scan() == WATER_STATUS_HAS_WATER);
}

/* 本次唤醒是谁叫醒的（打印用）。
 * ⚠️ 有副作用：内部会【取走并清除】唤醒标志（WaterDetect_TakeWakeFlag /
 *    app_time_take_wakeup_event 都是"读后清"），每轮只能调用一次。
 * 顺序很重要：先看"是不是水叫醒的"（EXTI 置的标志），再看 RTC ——
 * 两者可能同时发生，但"水来了"这个信息更有价值，必须先取。 */
static const char *app_wakeup_source_take_str(void)
{
    if (WaterDetect_TakeWakeFlag() != 0U)
    {
        return "WATER";                 /* 探针上升沿：水碰到电极，提前叫醒 */
    }
    return app_time_take_wakeup_event() ? "RTC" : "other";
}

/* 液位采样参数：连采 N 次取平均；模块要求两次触发间隔 > 1s */
#ifndef LEVEL_SAMPLE_COUNT
#define LEVEL_SAMPLE_COUNT      5U
#endif
#ifndef LEVEL_SAMPLE_GAP_MS
#define LEVEL_SAMPLE_GAP_MS     1100U
#endif
typedef char level_sample_cnt_check[(LEVEL_SAMPLE_COUNT <= 8U) ? 1 : -1];

/* 整轮全是 0xFFFF 时的补采：睡前外设域被关断到 0V，超声波模组每次醒来都是冷启动，
 * 个别模组 POR / 串口就绪比 1s 慢，第一轮 5 个采样会全废。这时把外设域重新关断再上电
 * （等价于再做一次冷启动）补采一轮，只补一次。 */
#ifndef APP_LEVEL_REPOWER_LOW_MS
#define APP_LEVEL_REPOWER_LOW_MS 200U   /* 补采前先把外设域关断到 0V 保持多久(ms) */
#endif

/* 采一轮：连采 LEVEL_SAMPLE_COUNT 次，每次都真触发超声波模组。
 * 返回 out_val[] 里的有效个数（0xFFFF = 这一次没收到有效帧，不参与平均）。 */
static uint8_t app_level_sample_pass(uint16_t *out_val)
{
    uint8_t i;
    uint8_t ok = 0U;

    for (i = 0U; i < LEVEL_SAMPLE_COUNT; i++)
    {
        if (i != 0U)
        {
            delay_ms(LEVEL_SAMPLE_GAP_MS);  /* 模块要求两次触发间隔 >1s */
        }

        out_val[i] = Level_Measure(APP_LEVEL_TIMEOUT_MS);   /* 触发脉冲 + 等帧都在里面 */

        if (out_val[i] != 0xFFFFU)
        {
            ok++;
        }
    }
    return ok;
}

/* 测一次液位：连采 LEVEL_SAMPLE_COUNT(默认 5) 次取平均，内部负责波特率切换 */
static uint16_t app_read_level(uint16_t last_level)
{
    uint16_t s[8];
    uint32_t sum = 0U;
    uint8_t  ok = 0U;
    uint8_t  i;
    uint8_t  drop_first = 0U;
    uint8_t  retried = 0U;
    uint16_t level;

    Level_UART_Init();                  /* PA10 重配 + USART1 -> 9600（模组要工作，必须初始化） */

    ok = app_level_sample_pass(s);

    if (ok == 0U)
    {
        /* 整轮全废 -> 补一次冷启动：PB7 上拉关断外设域到 0V -> 驱动低拉回 3.3V 触发 POR -> 重新 Init */
        BSP_Periph_Power_PullUp();
        delay_ms(APP_LEVEL_REPOWER_LOW_MS);
        Level_TrigHoldLow();
        BSP_Periph_Power_On();
        delay_ms(APP_PERIPH_POR_MS);
        Level_UART_Init();
        ok = app_level_sample_pass(s);        /* 上一轮全是 65535，没有信息量，直接覆盖 */
        retried = 1U;
    }

    Level_UartSetBaud(LOG_BAUD);        /* 【必须】切回日志波特率再打印 */

    if (retried != 0U)
    {
        log_info("[app] pass1 all invalid -> module re-power/re-init, pass2 shown below\r\n");
    }
    log_info("[app] level samples(%u):", (unsigned)LEVEL_SAMPLE_COUNT);
    for (i = 0U; i < LEVEL_SAMPLE_COUNT; i++)
    {
        log_info(" %u", (unsigned)s[i]);    /* 65535 = 本次没读到 */
    }

    if (ok == 0U)
    {
        log_info("  -> all invalid, keep last=%umm\r\n", (unsigned)last_level);
        return last_level;
    }

    /* 【已知现象·按需求处理】第一次采样固定读到 0（模块上电后的首帧问题），
     * 不追根因，直接把"第一位"舍弃：
     *   条件 = s[0]==0 且后面至少有一个"非 0 且非无效"的读数；
     *   否则（全 0，或后面全是 65535）不丢，0 就是真实读数。 */
    if (s[0] == 0U)
    {
        for (i = 1U; i < LEVEL_SAMPLE_COUNT; i++)
        {
            if ((s[i] != 0xFFFFU) && (s[i] != 0U))
            {
                drop_first = 1U;
                break;
            }
        }
    }

    /* 平均值 = 有效采样之和 / 实际参与的个数
     * （无效的 65535 不参与；被舍弃的第一位也不参与） */
    {
        uint8_t used = 0U;

        for (i = 0U; i < LEVEL_SAMPLE_COUNT; i++)
        {
            if ((drop_first != 0U) && (i == 0U))
            {
                continue;                   /* 舍弃第一位 */
            }
            if (s[i] != 0xFFFFU)
            {
                sum += s[i];
                used++;
            }
        }
        if (used == 0U)
        {
            log_info("  -> no usable sample, keep last=%umm\r\n", (unsigned)last_level);
            return last_level;              /* 兜底：防除 0 */
        }
        level = (uint16_t)(sum / (uint32_t)used);
        if (drop_first != 0U)
        {
            log_info("  -> drop first sample (s[0]=0), avg of the remaining %u\r\n",
                     (unsigned)used);
        }
        log_info("  -> avg=%umm (valid %u/%u)\r\n",
                 (unsigned)level, (unsigned)ok, (unsigned)LEVEL_SAMPLE_COUNT);
    }
    return level;
}

/* 上报周期取值：运行时状态里的 report_interval_s，非法就回默认 30min
 * 注：上报周期是云端的唯一权威输入（检测间隔由它推导，见 app_cmd.h【检测间隔的推导规则】）。
 * ⚠️ 必须返回 uint32：字段是 uint24（最大 16777215s ≈ 194 天），
 *    用 uint16 返回会把"7 天(604800s)"静默截成 6656s，而且编译器不会报警。 */
static uint32_t app_flow_get_report_s(const AppRuntimeState_t *rt)
{
    uint32_t s = rt->sensor_cfg.report_interval_s;

    /* 0x01 已把合法区间卡在 [60, 16777215]；这里兜的是 Flash 里的脏值（擦写失败/结构体错位）。
     * 落到区间外一律回出厂默认 30min，绝不带着脏值跑。 */
    if ((s < APP_CMD_REPORT_INTERVAL_MIN_S) || (s > APP_CMD_REPORT_INTERVAL_MAX_S))
    {
        s = (uint32_t)APP_REPORT_DEFAULT_S;
    }
    return s;
}

/* 巡检周期（= 这一觉睡多久）取值：
 *   B 态（有水）：d = ceil(上报/3)，只有"除 3 超过 2h"才夹到 2h，没有下限；
 *   A 态（无水）：不巡检，先返回上报间隔，调用方的 min(remain) 会把它压成"距下次上报的剩余秒数"。
 * 检测间隔永远现算、不读 Flash：它是 report 的派生量，存下来只会和 report 打架。
 * 下限兜底 APP_SLEEP_MIN_S(5s)：RTC 唤醒周期不能是 0/过小（会变成一直在醒）。 */
static uint16_t app_flow_get_sleep_s(const AppRuntimeState_t *rt, bool is_dry)
{
    uint32_t report_s = (uint32_t)app_flow_get_report_s(rt);
    uint32_t s;

    if (is_dry)
    {
        s = report_s;                       /* 状态 A：睡到上报点（下面会被 remain 压缩） */
    }
    else
    {
        s = (uint32_t)app_cmd_detect_from_report_s(report_s);
    }

    if (s == 0U)
    {
        s = (uint32_t)APP_SLEEP_DEFAULT_S;              /* 推导异常时的兜底（正常走不到） */
    }
    if (s < (uint32_t)APP_SLEEP_MIN_S)
    {
        s = (uint32_t)APP_SLEEP_MIN_S;
    }
    if (s > (uint32_t)APP_CMD_SLEEP_INTERVAL_MAX_S)
    {
        s = (uint32_t)APP_CMD_SLEEP_INTERVAL_MAX_S;
    }
    return (uint16_t)s;
}

/* 一个上报周期里要检测几次（= ceil(上报间隔 / 巡检周期)）：
 *   状态 B 正常 3 次（第 3 次就是开上报窗口那一次）；上报 >6h（检测被 2h 上限顶住）时更多；
 *   状态 A 无水可测，等于 1（每次醒都是上报点，醒一次报一次）。
 * 用途：RTC 还没校时没有墙上时间可用，就按"检测够这么多次就上报"维持内部节奏。 */
static uint16_t app_flow_get_detect_per_report(const AppRuntimeState_t *rt, bool is_dry)
{
    uint32_t report_s = (uint32_t)app_flow_get_report_s(rt);
    uint32_t detect_s = (uint32_t)app_flow_get_sleep_s(rt, is_dry);
    uint32_t n;

    if (detect_s == 0U)
    {
        return 1U;
    }
    n = (report_s + detect_s - 1U) / detect_s;      /* 向上取整 */
    return (n == 0U) ? 1U : (uint16_t)n;
}

/* 正式主流程（不返回）
 *   一轮 = ① 网关地址检查(没绑就先配对) -> ② 探针判水(顺带决定休眠模式 A/B)
 *        -> ③ 测液位(有水才测，更新本周期最大值)
 *        -> ④ 上报判定(超阈值立刻报 / 到上报点才报, 其余轮次只测不发)
 *        -> ⑤(仅上报轮) win1/win2/win3 三段接收窗口(收指令并回 ACK)
 *        -> ⑥ 本轮被解绑就进 STOP 等下次上电
 *        -> ⑦ 睡前定时：有水按 min(d, 距上报剩余)；无水直接睡到上报点
 *   （双态休眠的规则见上面 APP_DRY_CONFIRM_CNT 那段说明） */
/* ============================================================================
 *   【出厂 / 解绑后流程】等水唤醒 -> 短接触发 -> 配对 -> 绑定后复位进主流程
 * ----------------------------------------------------------------------------
 *   未绑定（出厂烧录后、或解绑成功后）的设备：
 *     ① 干态（探针开路）：睡。睡前武装探针唤醒（EXTI 双边沿，加了 1M 下拉后
 *        短接立刻能醒），另配 APP_FACTORY_POLL_S(30s) 的 RTC 心跳兜底（防漏边沿）。
 *     ② 短接探针（或碰水）-> 唤醒 -> 判到 wet -> 进配对：
 *          每 5s 发一帧 0x04，等网关应答；
 *          绑定成功 -> 网关地址落 Flash -> 发 10s 的 0x80 ACK -> 复位 -> 主流程。
 *     ③ 【关键行为】配对一轮没结果时【不回睡】：
 *        只要探针还湿/短接，就继续下一轮配对（网关配对窗口随时打开都能接上，
 *        下一帧 0x04 最多 5s 就到）；只有探针变干（人手松开）才回到 ① 去睡。
 *        ⚠️ 否则"短接着就睡下去"，探针电平不再变化 -> 没有新边沿 -> 再也醒不来。
 * @note 本函数不返回（只在复位后重启进主流程）。
 * ============================================================================ */
#ifndef APP_FACTORY_POLL_S
#define APP_FACTORY_POLL_S      30U     /* 干态心跳：每 30s 醒一次复查探针（防漏边沿） */
#endif

static void app_factory_run(void)
{
    bool     wet;
    bool     last_wet = false;
    bool     first    = true;
    uint16_t raw;
    uint8_t  heartbeat = 0U;

    WaterDetect_WakeAlwaysArm(1U);      /* 等水：不做任何保护判断 */
    WaterDetect_ProbeAlwaysOn();        /* PA7 常 3.3V；PB1 常检测 */

    log_info("[factory] unbound: short the probe to start pairing\r\n");

    while (1)
    {
        /* ---- 醒着/开机：【立刻】读探针 ----
         * ⚠️ 这里【不能】先调 app_periph_power_restore()：那个函数要等 1000ms 外设域 POR，
         *    而探针是 MCU 自己的 IO（PA7/PB1），不需要外设域上电。先等 1 秒会把
         *    人手"短接一下"（<1s）直接吞掉，表现为"短接没反应、要按住才进绑定"。 */
        raw = WaterDetect_ReadVoltageRaw();
        wet = (raw >= WATER_DETECT_WET_RAW_THRESHOLD);
        WaterDetect_ProbeAlwaysOn();    /* 采样会释放两脚，恢复"常驱动/常检测" */

        /* 日志：状态变化/首次立刻打，其余每约 30 轮打一次心跳。
         * raw/t100/ms 三个数一起打——泡在水里判"干"时，靠它们区分
         *   "回路真断"(全 0、ms 走满) 和 "充电太慢/太纯的水"(t100 小、raw 逐渐大、ms 偏长)。 */
        if (first || (wet != last_wet) || (heartbeat >= 30U))
        {
            log_info("[factory] probe raw=%u/t100=%u ms=%u -> %s\r\n",
                     (unsigned)raw, (unsigned)WaterDetect_LastRawFast(),
                     (unsigned)WaterDetect_LastChargeMs(),
                     wet ? "WET(shorted)" : "dry");
            heartbeat = 0U;
        }
        else
        {
            heartbeat++;
        }
        first    = false;
        last_wet = wet;

        /* ---- 干态：睡，等下一次探针变化（或 30s 心跳复查）---- */
        if (!wet)
        {
            app_lowpower_config_wakeup(APP_FACTORY_POLL_S);
            delay_ms(5U);
            app_flow_enter_stop();      /* 武装探针唤醒 + STOP */
            continue;
        }

        /* ---- 湿/短接：要把射频用起来 -> 这时才拉外设域 + 等 POR ---- */
        app_periph_power_restore();

        /* 配对一轮；没结果也【不回睡】，还湿就再来一轮 */
        log_info("[factory] probe wet -> pairing round (stay awake while wet)\r\n");
        if (app_pair_run())
        {
            log_info("[factory] BOUND ok -> reboot into main flow\r\n");
            delay_ms(100U);             /* 让串口把日志发完 */
            NVIC_SystemReset();
        }
        log_info("[factory] pairing round ended; probe still wet -> retry, dry -> back to sleep\r\n");
    }
}

static void app_production_run(void)
{
    AppRuntimeState_t rt;
    uint16_t sleep_s;
    uint16_t new_sleep_s;
    uint16_t level;
    uint16_t max_level = 0U;            /* 本上报周期内的最大水位：上报发它，发完清零重算 */
    uint16_t det_per_report = 3U;       /* 一个上报周期检测几次（正常 3；未校时时按它计数上报） */
    uint16_t since_report_cnt = 0U;     /* 距上次上报已经检测了几次（RTC 未校时用它维持节奏） */
    bool     has_water;
    bool     is_dry = false;            /* true = 状态 A（连续判无水）：只等上报点/等水 */
    uint8_t  dry_cnt = 0U;              /* 连续判无水的次数，达到 APP_DRY_CONFIRM_CNT 才切 A */
    bool     got_ack;
    bool     do_report;
    bool     time_ok;
    const char *why;
    const char *wake_src;
    uint32_t now_s;
    uint32_t report_period_s;
    uint32_t last_report_ts = 0U;       /* 上次"上报时刻"的 RTC 秒 */
    bool     report_ts_valid = false;   /* false = 还没有基准 -> 第一轮必报 */
    bool     report_ts_synced = false;  /* 建立基准时时间是否已校时（见下方的 "sync" 分支） */
    bool     alarm_active;
    uint16_t last_level = 0U;           /* 最新有效【原始】读数；全采样无效时兜底 */

    /* 上电初始化顺序：参数 -> 运行时状态 -> RTC -> 低功耗 -> 外设电源脚。
     * 运行时状态必须在 LoRa 起来之前初始化：app_cmd_process() 用 rt.dev_addr(0x21+UID)
     * 做目的地址匹配，没初始化就是全 0（UID 没读到），网关下发的帧会被直接丢掉。 */
    app_dev_param_init();
    app_runtime_state_init();
    (void)app_time_init();
    app_lowpower_init();
    WaterDetect_Init();                 /* 探针脚先置高阻 */

    log_info("\r\n===== production: pair + level + report + lowpower =====\r\n");
    app_print_params();
    app_print_time();
    app_print_rf_cfg();
    log_info("[app] flow: rx_win=%ums ack_delay=%ums level_samples=%u\r\n",
             (unsigned)APP_RX_WIN_MS, (unsigned)APP_CMD_ACK_DELAY_MS,
             (unsigned)LEVEL_SAMPLE_COUNT);

    /* ---- 未绑定（出厂烧录后 / 刚解绑）: 不进主循环 ----
     * 走"等水唤醒（短接探针）-> 配对 -> 绑定成功复位 -> 主流程"，
     * 见上面的 app_factory_run()。本函数不会返回。 */
    app_runtime_state_get(&rt);
    if (!proto_addr_is_valid_gw(rt.gw_addr))
    {
        app_factory_run();
    }

    while (1)
    {
        /* 先读唤醒源（读后清标志，纯粹为了日志顺序好看：先报"谁叫醒的"，再报"电源恢复"）。
         * ⚠️ 每轮只调一次：内部会清唤醒标志，重复调用会读到"other"。 */
        wake_src = app_wakeup_source_take_str();

        /* ---- ⓪ 唤醒后第一件事：恢复外设域电源 ----
         * 睡前 PB7 是输入上拉、外设域被关断到 0V（外设处于复位态），
         * 所以这里必须先驱动低拉回 3.3V + 等 POR，后面才能碰液位/LoRa。
         * 放在 ① 之前：配对模式也要用射频，别让 app_pair_run() 拿到没上电的模组。 */
        app_periph_power_restore();

        app_runtime_state_get(&rt);

        /* ---- ① 走到这里就一定是"已绑定"（未绑定的在前面已经进 app_factory_run 了）---- */

        /* 本轮的时间基准（一次读定，后面判定/定时都用它，避免跨秒误差）：
         *   now_s    = RTC 秒；time_ok = RTC 是否已校时（未校时改用"检测次数"做调度） */
        now_s           = app_time_get_unix_ts();
        time_ok         = (app_time_is_valid() != 0U);
        report_period_s = (uint32_t)app_flow_get_report_s(&rt);
        log_info("\r\n[app] wakeup (%s) report=%lus rtc=%s\r\n",
                 wake_src, (unsigned long)report_period_s, time_ok ? "ok" : "NOT-SYNCED");

        /* ---- ② 探针判水：有水才在 ③ 真触发超声波 ---- */
        has_water = app_probe_has_water();

        /* ---- ②b 双态休眠的状态切换 ----
         *   B(有水) -> A(无水)：连续 APP_DRY_CONFIRM_CNT 次判无水才切，防水面波动误判；
         *   A(无水) -> B(有水)：任一次判到有水立刻切（通常就是 EXTI 把 MCU 叫醒的）。
         * 切到 A 之后不再按 d 巡检：睡眠只对齐到下一个上报点，中间靠 EXTI 等水。
         * 注：判水本身已经带"数字电平=高就算有水"的兜底（见 WaterDetect_Scan），
         *     所以这里判出的无水，一定是引脚也为低 —— 睡下去挂上升沿才叫得醒。 */
        if (has_water)
        {
            dry_cnt = 0U;
        }
        else if (dry_cnt < APP_DRY_CONFIRM_CNT)
        {
            dry_cnt++;
        }
        is_dry = (dry_cnt >= APP_DRY_CONFIRM_CNT);

        sleep_s        = app_flow_get_sleep_s(&rt, is_dry);
        det_per_report = app_flow_get_detect_per_report(&rt, is_dry);
        /* 探针日志口径：raw / t100(首点) / ms(充电用时) / votes(投票) 一起打，
         * 现场"泡在水里却判 DRY"时能直接定性：
         *   raw≈0 且 t100≈0 且 ms 走满 -> 回路真断（电极钝化/虚焊/没泡到水）
         *   t100 小但 raw 明显大        -> 正常 RC 充电（慢但充得上），判水可信
         *   t100 就接近满量程           -> 电极被短接（τ≈0，瞬时满量程）
         *   raw 接近门限但 votes 不足    -> 水阻偏大，考虑降门限或改硬件
         * ms 是本轮充电时间常数：短接 200~300ms，自来水 400~900ms。 */
        log_info("[app] probe=%s raw=%u/t100=%u ms=%u lvl=%u wet=%u/%u (dry %u/%u) -> mode=%s period=%us (%u detect/report)\r\n",
                 has_water ? "WET" : "DRY",
                 (unsigned)WaterDetect_LastRaw(), (unsigned)WaterDetect_LastRawFast(),
                 (unsigned)WaterDetect_LastChargeMs(), (unsigned)WaterDetect_LastLvl(),
                 (unsigned)WaterDetect_LastWetVotes(), (unsigned)PROBE_ROUNDS,
                 (unsigned)dry_cnt, (unsigned)APP_DRY_CONFIRM_CNT,
                 is_dry ? "A:water-wake" : "B:timer-wake",
                 (unsigned)sleep_s, (unsigned)det_per_report);

        /* ---- ③ 测液位 ----
         * 有水：真触发超声波模块连采取平均；
         * 无水：不碰超声波模块，上报 0（省电），且【不覆盖 last_level】。 */
        if (has_water)
        {
            uint16_t raw_mm = app_read_level(last_level);   /* 原始读数：可能是"距离"，看测量模式 */

            /* 换算（测量模式 + 安装高度 + 校准偏移）放在这里而不是组帧里：
             * 超阈值判定和本周期最大值用的必须是【换算后的液位】，否则阈值物理含义对不上。 */
            level = app_sensor_convert_level(raw_mm);
            /* 缓存【原始】读数：换算依赖配置，缓存换算值会在配置变化时被重复叠加偏移 */
            last_level = raw_mm;
            if (level == 0xFFFFU)
            {
                level = 0U;                 /* 理论上走不到（app_read_level 已兜底返回上次值） */
            }
        }
        else
        {
            level = 0U;
            /* 干态把 last_level 也清掉：否则"水退 -> 水再来、且这一轮超声冷启动没读到"
             * 时会兜底复用上一次的旧液位，报出一个假值（比报 0 更隐蔽）。 */
            last_level = 0U;
            log_info("[app] probe=dry -> skip ultrasonic, report level 0\r\n");
        }

        /* ---- ③b 记录本周期最大值 ----
         * 每次检测都参与最大值统计（干态那几次 level=0，不改变结果）；
         * 上报时发的是它，发完清零重新累计。 */
        if (level > max_level)
        {
            max_level = level;
        }
        since_report_cnt++;                 /* 本轮这次检测计入"距上次上报已检测次数" */

        /* ---- ④ 上报判定 ----
         *   ① 超阈值 -> 立刻上报（每次检测都判，持续超限就每次检测都报）
         *      阈值 = 0 视为关闭告警：否则 level >= 0 恒真，会退化成每次苏醒都报
         *   ② 否则到上报点才报：已校时按"距上次上报 >= 上报间隔"；
         *      未校时按"距上次上报已检测 det_per_report 次"（正常 3 次）
         *   ③ 上电/配对后的第一帧立刻报（否则打不开接收窗，永远收不到网关 0x01）
         *   ④ 不报的轮次只做液位检测：不开射频、不开接收窗口（省电） */
        alarm_active = (rt.sensor_cfg.level_alarm_thresh_mm != 0U) &&
                       (level >= rt.sensor_cfg.level_alarm_thresh_mm);

        do_report = false;
        why = "";
        if (alarm_active)
        {
            do_report = true;
            why = "alarm";
        }
        if (!do_report)
        {
            if (!report_ts_valid)
            {
                do_report = true;               /* 上电/配对后的第一帧立刻发 */
                why = "first";
            }
            else if (time_ok)
            {
                if (!report_ts_synced)
                {
                    /* 【刚校时：旧基准作废】未校时时 app_time_get_unix_ts() 返回的是
                     * 2000-01-01 的占位值(9466848xx)，校时成功后 now_s 会跳到真实时间，
                     * elapsed 变成一个天文数字（现场日志里那句 "elapsed=846038318s/60s"），
                     * 结果之后每轮都判"到点该报"，上报节奏彻底失控。
                     * 这里改判一次"校时后的第一帧"，同时也把基准重建到真实时间轴上。 */
                    do_report = true;
                    why = "sync";
                }
                else if ((uint32_t)(now_s - last_report_ts) >= report_period_s)
                {
                    do_report = true;           /* 时间到点（睡前还会把睡眠对齐到这个点） */
                    why = "period";
                }
            }
            else if (since_report_cnt >= det_per_report)
            {
                do_report = true;               /* 未校时：检测够 det_per_report 次就报一次 */
                why = "count";
            }
        }

        log_info("[app] water=%u level=%umm max=%umm thresh=%umm alarm=%u cnt=%u/%u -> %s\r\n",
                 (unsigned)(has_water ? 1U : 0U), (unsigned)level, (unsigned)max_level,
                 (unsigned)rt.sensor_cfg.level_alarm_thresh_mm,
                 (unsigned)(alarm_active ? 1U : 0U),
                 (unsigned)since_report_cnt, (unsigned)det_per_report,
                 do_report ? "REPORT" : "skip report");

        if (do_report)
        {
            log_info("[app] report why=%s: level_now=%umm max=%umm elapsed=%lus/%lus (detect x%u)\r\n",
                     why, (unsigned)level, (unsigned)max_level,
                     (unsigned long)(report_ts_valid ? (uint32_t)(now_s - last_report_ts) : 0U),
                     (unsigned long)report_period_s, (unsigned)since_report_cnt);

            /* 【射频健康门】LoraApp_Init() 返回非 0 = BUSY 一直拉不下来，模组没响应
             * （PB7 没上电 / 模组坏 / SPI 断）。此时继续跑只会把整轮时间耗在超时上，
             * 发出去的帧也不可信，所以本轮【发送 + 三段接收窗】全跳过，直接去睡；
             * max_level / since_report_cnt / last_report_ts 都不推进，数据留到下轮一起报。 */
            if (LoraApp_Init() != 0U)
            {
                log_info("[app] !! radio dead (BUSY timeout) -> skip tx/rx this round, retry next wake\r\n");
            }
            else
            {
                LoraApp_Standby();
                if (!app_send_report(max_level))        /* 发【本周期最大值】，不是本次读数 */
                {
                    /* 发送失败也照样开接收窗：可能是芯片放弃了这一发(供电/天线)，
                     * 但接收链路通常还是好的，网关也可能重发指令过来。 */
                    log_info("[app] report send FAILED, still open rx window\r\n");
                }
                /* 这一帧发出去之后本周期就结束了：最大值清零重新累计（从下一轮检测开始新周期），
                 * 未校时用的苏醒计数同样清零。 */
                max_level = 0U;
                since_report_cnt = 0U;
                /* 上报基准 = 这一帧真发出去的时刻（不是本轮开始时刻）：
                 * 这样"上报周期"从帧发出那一刻算起，收发窗口那几十秒也不会累积成偏差。
                 * 发送失败也照样推进基准：失败就当"这轮报过了"，下一轮(检测周期)再试，
                 * 否则 remain 会一直算成 0，退化成 1 秒一轮的热循环。
                 * report_ts_synced 记下"建立这个基准时时间校没校过"，见上面判定里的说明。 */
                last_report_ts    = app_time_get_unix_ts();
                report_ts_valid   = true;
                report_ts_synced  = time_ok;

                /* ---- ⑤ 三段接收窗口 ----
                 *   win1 10s 收上报应答 -> 一帧都没收到再开 win2 10s
                 *   win1/win2 收到过(ACK) -> 再开 win3 10s 专门收网关指令
                 *   收到指令 -> 解析执行 -> 等 3s -> 回 ACK
                 *   ⚠️ 只有【上报轮】才开窗口：网关本来就是收到上报后才下发命令/应答的，
                 *      "只检测不上报"的那几轮开了也是白耗电。
                 *   ⚠️ 测试阶段的节奏，不是低功耗版本：做低功耗要把窗口收短。 */
                got_ack = app_rx_window(APP_RX_WIN_MS, "win1: wait report ACK");
                if (!got_ack)
                {
                    got_ack = app_rx_window(APP_RX_WIN_MS, "win2: retry (win1 silent)");
                }
                if (got_ack)
                {
                    (void)app_rx_window(APP_RX_WIN_MS, "win3: wait gateway cmd");
                }
                (void)app_cmd_tick_ack(true, false);    /* 兜底：还在等的 ACK 立刻发掉 */
                LoraApp_Sleep();                        /* 射频进深睡，别停在 Standby */

                if (LoraApp_TakeBusyFault())
                {
                    log_info("[app] !! BUSY timeout during tx/rx (radio lost) - report may not be sent\r\n");
                }

                /* 校时结果顺手落一次 Flash（内部节流到 1 小时一次）：
                 * 上报调度靠 RTC，掉电重启后要先从 Flash 恢复时间，否则只能"每轮都报"。 */
                app_time_save_to_flash();
            }
        }
        else
        {
            /* 只检测、不上报：连射频都不初始化，直接去睡 */
            if (time_ok)
            {
                log_info("[app] no report: max=%umm, elapsed=%lus < period=%lus -> level check only (no RF)\r\n",
                         (unsigned)max_level,
                         (unsigned long)((uint32_t)(now_s - last_report_ts)),
                         (unsigned long)report_period_s);
            }
            else
            {
                log_info("[app] no report: max=%umm, rtc NOT-SYNCED, detected %u/%u -> level check only (no RF)\r\n",
                         (unsigned)max_level, (unsigned)since_report_cnt, (unsigned)det_per_report);
            }
        }

        /* ---- ⑥ 防御兜底：本轮 gw_addr 变无效(理论上 0x05 已在 app_handle_rx_frame
         *      里发完 ACK+清完+STOP，不会走到这里) -> 同样进 STOP 等下次上电，
         *      不回配对(避免被解绑后又在本次运行内重新配对，干扰网关阶段2验证)。 ---- */
        app_runtime_state_get(&rt);
        if (!proto_addr_is_valid_gw(rt.gw_addr))
        {
            log_info("[app] gw_addr invalid after rx window -> STOP until next power-on\r\n");
            app_lowpower_config_wakeup(0);
            LoraApp_Sleep();
            while (1)
            {
                app_flow_enter_stop();
            }
        }

        /* ---- ⑦ 睡前定时 ----
         *   状态 B（有水）：sleep = min(检测间隔 d, 距下次上报的剩余)
         *   状态 A（无水）：sleep = 距下次上报的剩余（不按 d 巡检）
         * 为什么状态 B 不能只按 d 睡：上报点必须落在"正好到点"的那次苏醒上。
         * 例：上报 1800s、检测 600s 时整除没问题；上报 2000s、检测 667s（向上取整）时
         *     按剩余时间睡，第 3 次苏醒正好落在 2000s（667 + 667 + 666），不会多出第 4 次。
         * 状态 A 的"睡到上报点"也是靠下面这段 remain 压缩实现的（上面先给了个上报间隔当上限）。
         * 时间没校准时没法算剩余，就只按巡检周期睡，靠"检测次数"计数到点上报。 */
        new_sleep_s = app_flow_get_sleep_s(&rt, is_dry);
        if (new_sleep_s != sleep_s)
        {
            log_info("[app] sleep interval changed: %us -> %us (mode=%s)\r\n",
                     (unsigned)sleep_s, (unsigned)new_sleep_s,
                     is_dry ? "A:water-wake" : "B:timer-wake");
            sleep_s = new_sleep_s;
        }
        report_period_s = (uint32_t)app_flow_get_report_s(&rt);
        /* ⚠️ 必须【重新读一次】RTC 校时状态，不能用本轮开头那次快照 time_ok：
         * 本轮很可能刚收到带校时的 0x01，开头读时还是 NOT-SYNCED -> 对齐被跳过 ->
         * 睡 300s 而上报周期只有 60s。 */
        if (app_time_is_valid() != 0U)
        {
            uint32_t elapsed = (uint32_t)(app_time_get_unix_ts() - last_report_ts);
            uint32_t remain  = (elapsed >= report_period_s) ? 0U : (report_period_s - elapsed);

            if (remain < (uint32_t)sleep_s)
            {
                sleep_s = (uint16_t)((remain < (uint32_t)APP_SLEEP_ALIGN_MIN_S)
                                         ? (uint32_t)APP_SLEEP_ALIGN_MIN_S : remain);
                log_info("[app] sleep shortened to %us to hit the report point (due in %lus)\r\n",
                         (unsigned)sleep_s, (unsigned long)remain);
            }
        }

        /* ---- 持久化云端配置 ----
         * 0x01 下发的 report/阈值只在 RAM 里用，掉电/复位就回到 Flash 里的出厂默认，
         * 所以睡前落一次 Flash。
         * 为什么放在这里而不是收到 0x01 的那一刻：Flash 擦写会停总线 ~20-40ms，
         * 别在收包窗口里做（此时射频已睡、接收窗口已关）。
         * app_dev_param_set_sensor_cfg() 内部有去重：配置没变直接返回、不擦写，
         * 所以不怕网关每轮重发同一条 0x01。 */
        if (!app_dev_param_set_sensor_cfg(&rt.sensor_cfg))
        {
            log_info("[app] !! persist sensor cfg FAILED (flash write)\r\n");
        }

        app_lowpower_config_wakeup(sleep_s);
        log_info("[app] enter STOP, wake in %us (mode=%s period=%us report=%lus max_now=%umm)\r\n",
                 (unsigned)sleep_s,
                 is_dry ? "A:water-wake" : "B:timer-wake",
                 (unsigned)app_flow_get_sleep_s(&rt, is_dry),
                 (unsigned long)report_period_s, (unsigned)max_level);
        /* 探针武装状态统一问 WaterDetect_WouldArm()：用双态休眠状态去猜会和
         * app_lowpower_enter_stop_pullup() 的真实条件不一致（日志说 off、实际 EXTI 已挂）。 */
        log_info("[app] sleep: PB7 pull-up -> periph domain 0V (few uA), probe-wake=%s\r\n",
                 WaterDetect_WouldArm() ? "ARMED" : "off");
        delay_ms(5U);
        app_flow_enter_stop();          /* 阻塞，直到 RTC 周期唤醒（睡前 PB7 输入上拉） */
    }
}


/* ============================================================================
 *                                  main
 * ============================================================================ */
int main(void)
{
    SystemClock_Config();
    log_init();

    BSP_Periph_Power_Init();            /* 外设电源脚 PB7：默认【驱动为低】= 外设域真正上电
                                         * （高阻时实测只有 1.2V，超声波模组不能用；见 bsp_gpio.h） */
    print_reset_cause();

    /* PB7 = 外设总电源使能：驱动低(=0) = 外设上电。读到 1 说明档位不对或被外部拉高，
     * 外设会没电。STOP 那一档不在 init 里，睡前由 BSP_Periph_Power_PullUp() 单独配。 */
    log_info("[pwr] PB7(periph power EN) level=%u (0=drive-low=on)\r\n",
             (unsigned)BSP_Periph_PowerPeek());

    log_info("\r\n===== N32G031 level node: production =====\r\n");
    app_production_run();               /* 不返回：配对 + 液位 + 上报 + 低功耗 */
}
