/**
 * @file main.c
 * @brief N32G031 液位传感器节点 —— 主流程（四选一，编译期宏切换）
 *
 * ============================ 【怎么切模式】 ============================
 * 改下面 MAIN_MODE 那一个数字，或者在 Keil 的
 * Options for Target -> C/C++ -> Define 里定义 MAIN_MODE=x 覆盖（本宏是 #ifndef 保护的）。
 *
 *  1 = 低功耗测试模式：醒 5s -> STOP 20s -> 循环（量"醒/睡"两个电流值用）
 *  2 = LoRa 调试模式  ：三选一（一直接收 / 一直发送 / 发一帧+收 10s 循环）
 *  3 = 配对网关模式  ：每 5s 发 0x04 配对请求，配对成功即停；
 *                      【不写 Flash】，所以下次上电还会重新配对
 *  4 = 正式模式      ：网关地址不对先配对 -> 液位检测 -> LoRa 上报/接收 -> 低功耗
 *  5 = 【旧台架】只读液位（2s 一次，不睡）
 *  6 = 【旧台架】探针标定（只看 PA7/PB1 电压，不跑 LoRa）
 *
 * ⚠️⚠️ 【模式号含义和以前不一样了！】老版本的编号是
 *      1=液位单测 / 2=上报 / 3=只睡 / 4=探针标定，
 *      现在是 1=低功耗测试 / 2=LoRa调试 / 3=配对测试 / 4=正式。
 *      如果你在 Keil 的 Define 里还留着老的 "MAIN_MODE=2"，行为会变（变成 LoRa 调试），
 *      请把那里的定义删掉，只在本文件里改。
 *      （旧版 main.c 已备份为 src/main.c.bak_before_rewrite_20260927）
 * ======================================================================
 *
 * 【硬件要点（踩过的坑，别动）】
 *   - PB7 = 外设总电源使能（低=上电）。本板【不能翻转】这个脚：切换会让整板
 *     POR/BOR 复位。固件开机输出一次低电平后全程保持，进 STOP 也不动它。见 bsp_gpio.h。
 *   - PA7 = 探针激励脚（配 PB1 做 ADC）；PB1 只允许输入+内部上下拉，不能推挽输出。
 *   - PA9 = 液位模块 TRIG。模块由 PB7 域常供电，所以【进 STOP 前必须把它钉在低电平】，
 *     否则悬空会被噪声误触发、反复测距 -> 电流脉冲 -> 复位。见 Level_TrigHoldLow()。
 *   - 液位模块 RX 在 PA10 = USART1_RX，而日志 TX 在 PB6 也是 USART1：
 *     一个 USART 只有一个波特率。Level_UART_Init() 会把 USART1 切到 9600，
 *     【采完必须 Level_UartSetBaud(LOG_BAUD) 切回来】，否则后面所有打印都是乱码。
 *   - LoRa 用 SX1261 参数（芯片自报 SX1261，不是 SX1262），见 radio.c 的 LLCC68GetDeviceId()。
 *   - STOP 唤醒的坑：RTC(Line20) 的 EXTI 挂起位必须在中断里手动清，见 n32g031_it.c。
 *
 * 【日志约定】所有打印字符串统一用英文，避免中文字符串被编辑器按 ANSI 存成乱码。
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
#include "app_power.h"
#include "crc16_modbus.h"               /* 调试模式发帧自动补 CRC */
#include "protocol_util.h"              /* proto_addr_is_valid()：判断网关地址是否已绑定 */
#include "bsp_gpio.h"
#include "bsp_adc.h"
#include "bsp_rtc.h"
#include "water_cu.h"
#include "level_uart.h"

/* ============================================================================
 *                          【模式选择：改这里】
 * ============================================================================ */
#define MAIN_MODE_LOWPOWER_TEST     1   /* 低功耗测试：醒 5s + 睡 20s */
#define MAIN_MODE_LORA_DEBUG        2   /* LoRa 调试：一直收 / 一直发 / 发+收 */
#define MAIN_MODE_PAIR_TEST         3   /* 配对测试：配对成功即停，不写 Flash */
#define MAIN_MODE_PRODUCTION        4   /* 正式：配对 + 液位 + 上报接收 + 低功耗 */
#define MAIN_MODE_LEVEL_BENCH       5   /* 旧台架：只读液位 */
#define MAIN_MODE_PROBE_BENCH       6   /* 旧台架：探针标定 */
#define MAIN_MODE_LP_LEVEL_LORA_LOOP 7 /* 低功耗+液位5次+LoRa收发(N次×8s RX)循环 */

#ifndef MAIN_MODE
#define MAIN_MODE                   MAIN_MODE_PRODUCTION
#endif

#if (MAIN_MODE != MAIN_MODE_LOWPOWER_TEST) && (MAIN_MODE != MAIN_MODE_LORA_DEBUG) && \
    (MAIN_MODE != MAIN_MODE_PAIR_TEST)     && (MAIN_MODE != MAIN_MODE_PRODUCTION) && \
    (MAIN_MODE != MAIN_MODE_LEVEL_BENCH)   && (MAIN_MODE != MAIN_MODE_PROBE_BENCH) && \
    (MAIN_MODE != MAIN_MODE_LP_LEVEL_LORA_LOOP)
#error "MAIN_MODE must be 1(lowpower) 2(lora debug) 3(pair) 4(production) 5(level) 6(probe) 7(lp+level+lora)"
#endif

/* ==================== 通用参数 ==================== */
#define APP_PERIPH_SETTLE_MS    200U    /* 外设上电后等稳定(ms) */
#define APP_TX_DONE_TIMEOUT_MS  3000U   /* 等 TxDone 的超时(ms) */
#define APP_LEVEL_TIMEOUT_MS    1000U   /* 超声测液位最长等待(ms) */
#define APP_RX_BUF_MAX          128U

/* ==================== 外设域供电：方案B（输入上拉，几 uA） ====================
 * 外设域（超声波液位 + LoRa + 探针供电）由 PB7 的高边 P-MOS 开关控制（低=上电）。
 *   醒着：BSP_Periph_Power_On() 驱动低 → 外设域 3.3V 正常工作。
 *   STOP：BSP_Periph_Power_PullUp() 配输入上拉（内部~40k）→ 栅极被拉高、
 *         P-MOS 关断、外设域掉到 0V（几 uA）。
 *   唤醒：BSP_Periph_Power_On() 驱动低拉回 3.3V + 延时等 POR + 重新 Init。
 *         （由 app_periph_power_restore() 统一处理，RTC/探针唤醒都走它。）
 * ⚠️ 探针两根脚(PA7 激励 / PB1 检测)是 MCU 自己的 IO，不挂 PB7 外设域上，
 *    所以 PB7 关断外设域不影响"有水自动唤醒"。
 * （方案A 纯高阻释放已删除：本板实测 1.2V 半开 + POR 复位循环，不可用。）
 * ============================================================================ */
#ifndef APP_PERIPH_POR_MS
/* 唤醒后等外设 POR 冷启动的延时(ms)。
 * 500ms 在模式7(只睡 20s)够用，但模式4 睡 40s~5min 后实测超声波没反应，所以加大到 1000；
 * 还不够就继续加到 2000/3000（超声波模组自己的 MCU 启动 + 晶振稳定要时间）。 */
#define APP_PERIPH_POR_MS       1000U
#endif

/* 睡前进 STOP：统一走方案B（PB7 输入上拉，外设域关断到 0V，几 uA） */
static void app_flow_enter_stop(void)
{
    app_lowpower_enter_stop_pullup();
}

/* 唤醒后第一件事：恢复外设域电源。
 * 顺序讲究：① 先把 TRIG(PA9) 钉成空闲低 —— 模块冷启动 POR 期间 TRIG 若悬空，
 *              可能被它当成一次触发，一上电就空打一发（大电流脉冲 + 状态错乱）；
 *            ② 再 PB7 驱动低，把外设域从 ~1.2V 拉回 3.3V；
 *            ③ 等 POR 冷启动（APP_PERIPH_POR_MS）。
 * RTC 唤醒和探针唤醒都必须走这里（模式4 放在每轮第一步，比配对检查还早）。 */
static void app_periph_power_restore(void)
{
    /* 方案B：睡前外设域被关断到 0V，唤醒后必须先驱动低拉回 3.3V + 等 POR，再碰外设 */
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
    log_info("[app] dev_addr=%02X%02X%02X gw_addr=%02X%02X%02X (%s)\r\n",
             (unsigned)rt.dev_addr[0], (unsigned)rt.dev_addr[1], (unsigned)rt.dev_addr[2],
             (unsigned)rt.gw_addr[0], (unsigned)rt.gw_addr[1], (unsigned)rt.gw_addr[2],
             proto_addr_is_valid(rt.gw_addr) ? "BOUND" : "UNBOUND");
    log_info("[app] cfg: detect=%us report=%us same_lvl_report=%u alarm_thresh=%umm\r\n",
             (unsigned)rt.sensor_cfg.detect_interval_s,
             (unsigned)rt.sensor_cfg.report_interval_s,
             (unsigned)rt.sensor_cfg.same_level_report_en,
             (unsigned)rt.sensor_cfg.level_alarm_thresh_mm);
#if (APP_CMD_DETECT_DERIVE_FROM_REPORT)
    /* 把"检测间隔怎么来的"在启动时打一次：收到 0x01 后 detect 由它算出来，
     * 上面那行的 detect 在收到 0x01 之前只是出厂/本地默认值。 */
    log_info("[app] rule: 0x01 detect := report/%u, clamped to [%u,%u]s (payload detect ignored)\r\n",
             (unsigned)APP_CMD_DETECT_FROM_REPORT_DIV,
             (unsigned)APP_CMD_DETECT_INTERVAL_MIN_S,
             (unsigned)APP_CMD_DETECT_INTERVAL_MAX_S);
#endif
}

static void app_print_time(void)
{
    bsp_rtc_calendar_t cal;
    bsp_rtc_calendar_t bj;

    if (app_time_is_valid() != 0U)
    {
        /* RTC 里存的是 UTC（0x01 校时时把北京时间 -8h 存进去），所以这里把两个都打出来，
         * 免得现场看到 09:07 而墙上是 17:07 以为是错误。协议字段用的是北京时间。 */
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

/* 打印一遍射频参数（台架对参数用，一眼能核对） */
static void app_print_rf_cfg(void)
{
    log_info("[rf] freq=%luHz SF=%u BWidx=%u CR=%u preamble=%u fixlen=%u iq=%u power=%udBm\r\n",
             (unsigned long)LORA_FRE, (unsigned)LORA_SPREADING_FACTOR,
             (unsigned)LORA_BANDWIDTH, (unsigned)LORA_CODINGRATE,
             (unsigned)LORA_PREAMBLE_LENGTH, (unsigned)LORA_FIX_LENGTH_PAYLOAD_ON,
             (unsigned)LORA_IQ_INVERSION_ON, (unsigned)LORA_TX_OUTPUT_POWER);
}

/* ============================================================================
 *   模式 1：低功耗测试 —— 醒 LP_TEST_AWAKE_MS，睡 LP_TEST_SLEEP_S，循环
 * ----------------------------------------------------------------------------
 * 用途：量两个电流。日志口每轮打一行，方便对着电流表读：
 *      "[lp] awake N ms (measure now)" 之后这 5s 是醒着电流
 *      "[lp] enter STOP 20s" 之后这 20s 是睡态电流
 * 这个模式默认【不碰射频/液位/探针】，量的是 MCU 底线；
 * 想看"带射频"的两个电流，把 LP_TEST_RADIO_ON 改成 1。
 * ============================================================================ */
#ifndef LP_TEST_AWAKE_MS
#define LP_TEST_AWAKE_MS    5000U       /* 每轮醒着多久(ms) */
#endif
#ifndef LP_TEST_SLEEP_S
#define LP_TEST_SLEEP_S     20U         /* 每轮睡多久(s) */
#endif
#ifndef LP_TEST_RADIO_ON
#define LP_TEST_RADIO_ON    0           /* 1 = 醒着的这段时间也把射频拉起来(量带射频的电流) */
#endif

static void app_mode_lowpower_test(void)
{
    uint32_t cycle = 0U;

    (void)app_time_init();              /* RTC(LSI)：STOP 的唤醒源 */
    app_lowpower_init();                /* PWR 时钟 */

    log_info("\r\n===== mode 1: lowpower test (awake %ums + STOP %us) =====\r\n",
             (unsigned)LP_TEST_AWAKE_MS, (unsigned)LP_TEST_SLEEP_S);

    while (1)
    {
        cycle++;
        /* 方案A：醒来第一件事就是把外设域从 ~1.2V 拉回 3.3V 并等 POR。
         * 本模式不测液位，但 LP_TEST_RADIO_ON=1 时要拉射频，所以统一走这里。 */
        app_periph_power_restore();
        log_info("[lp] cycle %lu: awake %ums (measure now)\r\n",
                 (unsigned long)cycle, (unsigned)LP_TEST_AWAKE_MS);

#if (LP_TEST_RADIO_ON)
        LoraApp_Init();
        LoraApp_Standby();              /* 射频待机态，约占 2.4mA */
#endif

        delay_ms(LP_TEST_AWAKE_MS);

#if (LP_TEST_RADIO_ON)
        LoraApp_Sleep();                /* 射频进最深睡，别停在待机 */
#endif

        app_lowpower_config_wakeup(LP_TEST_SLEEP_S);
        log_info("[lp] enter STOP %us (measure now)\r\n", (unsigned)LP_TEST_SLEEP_S);
        log_info("[lp] sleep: PB7 pull-up -> periph domain 0V (few uA)\r\n");
        delay_ms(5U);
        app_flow_enter_stop();          /* 阻塞，直到 RTC 唤醒；方案B 睡前 PB7 输入上拉 */

        log_info("[lp] wakeup (%s)\r\n", app_time_take_wakeup_event() ? "RTC" : "other");
    }
}

/* ============================================================================
 *   模式 2：LoRa 调试 —— 三选一
 * ----------------------------------------------------------------------------
 *   LORA_DBG_CONT_RX : 一直收，收到什么就原样打出来，不看协议
 *   LORA_DBG_CONT_TX : 一直发，每 LORA_DBG_TX_PERIOD_MS 一条，只发不收
 *   LORA_DBG_TX_RX   : 发一条 -> 开 LORA_DBG_RX_WIN_MS 接收窗口 -> 再发，循环
 * 发送内容改 s_dbg_tx_data[] 数组即可（长度自动跟着变）。
 * ============================================================================ */
#define LORA_DBG_CONT_RX    0U
#define LORA_DBG_CONT_TX    1U
#define LORA_DBG_TX_RX      2U

#ifndef LORA_DBG_STATE
#define LORA_DBG_STATE      LORA_DBG_TX_RX
#endif
#ifndef LORA_DBG_TX_PERIOD_MS
#define LORA_DBG_TX_PERIOD_MS   5000U   /* "一直发送"的周期 */
#endif
#ifndef LORA_DBG_RX_WIN_MS
#define LORA_DBG_RX_WIN_MS      10000U  /* 发完之后的接收窗口 */
#endif

/* ==================== 发送内容（改这里） ====================
 * LORA_DBG_TX_RAW = 0（默认）：数组是【不含 CRC 的协议帧】，发送前由 crc16_modbus
 *     自动把 2 字节 CRC 补在后面 —— 你改数据不用自己算 CRC，就不会出现
 *     "改了数据忘了改 CRC" → 网关严格校验判坏帧 → 现象是"网关解析不了"。
 * LORA_DBG_TX_RAW = 1：数组就是空口上要发的每一个字节，一个都不加、不改。
 *     测纯射频 / 故意发不合法内容时用（比如 20 个 0xAA）。
 *
 * 默认内容 = 协议参考上报帧（不含 CRC 的 18 字节）：
 *   AA 55 | 01 | 11 00 01 | 21 00 01 | 02 | 07 | 04 D2 64 0A 0B 0C 0D
 *   帧头    版本  网关地址    设备地址   上报 长度  液位1234 100% 月日时分
 *   -> CRC 自动补上，空口上一共 20 字节
 *
 * ⚠️【为什么要动那 4 个月日时分字节】网关的 RAW 自环测试帧和这条帧本来【逐字节一样】
 *    (都是 04 D2 64 00 00 00 00)，于是网关日志里收到一帧 20 字节根本分不清是
 *    "收到了液位的" 还是 "收到了自己的"。这里把月日时分写成 0A 0B 0C 0D 当【身份标记】：
 *    网关日志里出现 "04 D2 64 0A 0B 0C 0D" 就一定是液位发的（字段值本身合法：
 *    10月11日12:13）。网关那条自环帧仍是 00 00 00 00，一眼可分。 */
#ifndef LORA_DBG_TX_RAW
#define LORA_DBG_TX_RAW     0
#endif

// 不用自己算 CRC，会自动补上
static const uint8_t s_dbg_tx_data[] =
{
    0xAAU, 0x55U, 0x01U,
    0x11U, 0x00U, 0x01U,
    0x21U, 0x00U, 0x01U,
    0x02U, 
    0x07U,
    0x04U, 0xD2U, 0x64U, 0x0AU, 0x0BU, 0x0CU, 0x0DU
};

/* 组出真正要发到空口上的字节；返回总长度，0 = 配置有问题 */
static uint16_t app_dbg_build_tx(uint8_t *out, uint16_t out_max)
{
    uint16_t n = (uint16_t)sizeof(s_dbg_tx_data);

    if (out == NULL)
    {
        return 0U;
    }

#if (LORA_DBG_TX_RAW)
    if (n > out_max)
    {
        return 0U;
    }
    memcpy(out, s_dbg_tx_data, n);
    return n;
#else
    if ((uint16_t)(n + 2U) > out_max)
    {
        return 0U;
    }
    if (n < 13U)                        /* 至少 帧头2+ver1+gw3+dev3+func1+dlen1+CRC2 */
    {
        return 0U;
    }
    memcpy(out, s_dbg_tx_data, n);
    crc16_modbus_append(&out[2], (uint16_t)(n - 2U));   /* CRC 覆盖 版本..数据末尾 */
    return (uint16_t)(n + 2U);
#endif
}

/* 调试模式收窗口：收到就打印，不管内容对不对
 * @param ms  窗口时长(ms)；传 0xFFFFFFFF 就是"一直收"
 * @retval 窗口内收到的帧数
 * @note  这里只打印长度+字节（RSSI/SNR 在驱动内部，本层拿不到） */
static uint16_t app_dbg_rx_window(uint32_t ms, const char *tag)
{
    static uint8_t buf[APP_RX_BUF_MAX];     /* static：本板栈只有 4KB */
    uint32_t t0 = LLCC68GetTimerCount();
    uint16_t rx_len;
    uint16_t frames = 0U;

    LoraApp_ReciveMode();
    while ((uint32_t)(LLCC68GetTimerCount() - t0) < ms)
    {
        LoraApp_ProcessIrq();
        rx_len = LoraApp_GetRx(buf, sizeof(buf));
        if (rx_len > 0U)
        {
            /* 收帧的 RSSI/SNR：芯片刚收完的包状态寄存器，用来对比两个方向的链路电平。
             * 注意这是【驱动已经读过一次】的同一个寄存器，不会打断接收。 */
            PacketStatus_t ps;
            LLCC68GetPacketStatus(&ps);
            frames++;
            log_info("[dbg] %s rx %u bytes rssi=%d snr=%d: ", tag, (unsigned)rx_len,
                     (int)ps.Params.LoRa.RssiPkt, (int)ps.Params.LoRa.SnrPkt);
            app_dump_hex(buf, rx_len);
            LoraApp_ResumeRx();             /* 收完继续把窗口收满 */
            continue;
        }
        delay_ms(1U);
    }
    LoraApp_Standby();
    return frames;
}

static void app_mode_lora_debug(void)
{
    static uint8_t tx_frame[64];        /* static：本板栈只有 4KB */
    uint16_t tx_len;
    uint32_t t_start;
    uint16_t frames;

    LoraApp_Init();
    LoraApp_Standby();

    log_info("\r\n===== mode 2: lora debug =====\r\n");
    app_print_rf_cfg();

    /* 先把要发的字节组好（RAW=0 时自动补 CRC），并打印出来核对 */
    tx_len = app_dbg_build_tx(tx_frame, (uint16_t)sizeof(tx_frame));
    if (tx_len == 0U)
    {
        log_info("[dbg] !! tx frame build FAILED (array too big, or too short for a frame)\r\n");
    }
    else
    {
        log_info("[dbg] tx frame (%u bytes, RAW=%u): ", (unsigned)tx_len, (unsigned)LORA_DBG_TX_RAW);
        app_dump_hex(tx_frame, tx_len);
    }

#if (LORA_DBG_STATE == LORA_DBG_CONT_RX)
    /* ---- 一直接收 ---- */
    log_info("[dbg] state = CONTINUOUS RX (receive everything, no protocol check)\r\n");
    (void)app_dbg_rx_window(0xFFFFFFFFU, "cont");    /* 不返回 */

#elif (LORA_DBG_STATE == LORA_DBG_CONT_TX)
    /* ---- 一直发送 ---- */
    log_info("[dbg] state = CONTINUOUS TX, every %ums\r\n", (unsigned)LORA_DBG_TX_PERIOD_MS);

    while (1)
    {
        t_start = LLCC68GetTimerCount();

        if ((tx_len > 0U) &&
            LoraApp_SendSync(tx_frame, (uint8_t)tx_len, APP_TX_DONE_TIMEOUT_MS))
        {
            log_info("[dbg] tx done (%u bytes)\r\n", (unsigned)tx_len);
        }
        else
        {
            log_info("[dbg] !! tx FAILED (no TX_DONE, or bad frame) - check power/antenna/PA\r\n");
        }

        /* 补满周期：空口上就是稳定的 LORA_DBG_TX_PERIOD_MS 一帧 */
        while ((uint32_t)(LLCC68GetTimerCount() - t_start) < LORA_DBG_TX_PERIOD_MS)
        {
            delay_ms(1U);
        }
    }

#else
    /* ---- 发一条 + 收一个窗口，循环 ---- */
    log_info("[dbg] state = TX then RX window %ums, loop\r\n", (unsigned)LORA_DBG_RX_WIN_MS);

    while (1)
    {
        if ((tx_len > 0U) &&
            LoraApp_SendSync(tx_frame, (uint8_t)tx_len, APP_TX_DONE_TIMEOUT_MS))
        {
            log_info("[dbg] tx done (%u bytes)\r\n", (unsigned)tx_len);
        }
        else
        {
            log_info("[dbg] !! tx FAILED (no TX_DONE, or bad frame) - check power/antenna/PA\r\n");
        }

        frames = app_dbg_rx_window(LORA_DBG_RX_WIN_MS, "txrx");
        log_info("[dbg] rx window closed, frames=%u -> send again\r\n", (unsigned)frames);
    }
#endif
}

/* ============================================================================
 *   模式 3：配对网关模式 —— 配对成功即停（停在"一直回 ACK"上），且【不写 Flash】
 * ----------------------------------------------------------------------------
 * 每 5s 发一帧 0x04 配对请求（目的地址 FF FF FF），收到网关应答后：
 *   把网关地址写进【RAM 运行时状态】（不写 Flash）-> 打印 BOUND
 *   -> 【留在配对模式里一直回 0x80 ACK】（app_pair.c 的 APP_PAIR_STAY_AFTER_BOUND=1）。
 * 因为没落 Flash，下次上电网关地址还是 FF FF FF，还会重新配对 —— 这正是
 * 你要求的"配对成功停止，但是下次上电还要配对"。
 * 顺带一提：本模式【不看 Flash 里已存的绑定】，即使以前配过也照样重新配。
 * ============================================================================ */
static void app_mode_pair_test(void)
{
    app_dev_param_init();
    app_runtime_state_init();

    log_info("\r\n===== mode 3: pair-to-gateway test =====\r\n");
    log_info("[pairtest] pair once, then STAY replying ACK. binding is RAM-only (not saved)\r\n");
    app_print_params();

    app_pair_run_ramonly();             /* 不返回：绑上后一直回 ACK；不写 Flash */

    /* 只有把 APP_PAIR_STAY_AFTER_BOUND 改成 0 才会走到这里 */
    log_info("[pairtest] pairing returned (APP_PAIR_STAY_AFTER_BOUND=0)\r\n");
    while (1)
    {
        delay_ms(1000U);
    }
}

/* ============================================================================
 *   模式 4：正式模式 —— 配对 + 液位检测 + 上报接收 + 低功耗
 * ============================================================================ */
/* --- 4.1 液位/探针开关 ---
 * APP_FLOW_USE_PROBE : 1 = 电压法判水（water_cu.c）；0 = 不判水，一律当有水
 * APP_FLOW_USE_LEVEL : 1 = 真读液位（触发 PA9、临时把 USART1 切 9600）；
 *                      0 = 不碰液位模块，上报 APP_FIXED_REPORT_LEVEL_MM 这个固定值
 * 正式模式默认两个都开（"液位检测功能"）；台面没接硬件时把对应开关改 0 即可。 */
#ifndef APP_FLOW_USE_PROBE
#define APP_FLOW_USE_PROBE      1
#endif
#ifndef APP_FLOW_USE_LEVEL
#define APP_FLOW_USE_LEVEL      1
#endif
#ifndef APP_FIXED_REPORT_LEVEL_MM
#define APP_FIXED_REPORT_LEVEL_MM   0U
#endif
/* 探针驱动开关（只在 APP_FLOW_USE_PROBE=1 时有意义） */
#ifndef WATER_DETECT_ENABLE
#define WATER_DETECT_ENABLE     1
#endif

/* 接收窗口与应答时序（测试阶段不考虑功耗，窗口给足） */
#ifndef APP_RX_WIN_MS
#define APP_RX_WIN_MS           10000U  /* 单个接收窗口(ms)：win1/win2/win3 各一个 */
#endif
#ifndef APP_CMD_ACK_DELAY_MS
#define APP_CMD_ACK_DELAY_MS    3000U   /* 收到网关指令后等多久再回 ACK(ms) */
#endif

/* ==================== 0x05 解绑的应答节奏（2026-10-07 需求） ====================
 * 需求原话："收到解绑后回复 ack 两次，每次间隔 5s，之后进入解绑流程
 *           把网关地址改成 FF FF FF，等下次上电"
 * 为什么是两次：网关发完下行要切到接收，单次 ACK 有可能正好落在它的切换盲区里；
 *              隔 5s 再发一次，两次总有一次能被收到（也顺便覆盖网关那边的重发/阶段机）。
 * 顺序铁律：先把【两次 ACK 都发完】再清 gw_addr —— 清了地址就再也组不出应答帧了。
 * ============================================================================ */
#ifndef APP_UNBIND_ACK_COUNT
#define APP_UNBIND_ACK_COUNT        2U
#endif
#ifndef APP_UNBIND_ACK_GAP_MS
#define APP_UNBIND_ACK_GAP_MS       5000U   /* 两次 ACK 之间的间隔(ms) */
#endif
/* 休眠周期兜底值(s)：Flash 里的值非法时用 */
#ifndef APP_SLEEP_DEFAULT_S
#define APP_SLEEP_DEFAULT_S     30U
#endif
/* 存储里检测间隔的【合法性下限】(s)：只用来判断 Flash/RAM 里的值是不是垃圾，
 * ⚠️ 不要用 APP_CMD_DETECT_INTERVAL_MIN_S(300s)！那个 300s 是"云端推导出来的
 *    检测间隔"的下限（见 app_cmd.h【检测间隔的推导规则】），而出厂默认还是
 *    30s/30s（没收到云端配置之前台架上照样 30s 一轮），两者不是一回事。 */
#ifndef APP_SLEEP_MIN_S
#define APP_SLEEP_MIN_S         5U
#endif

/* ==================== 上报节奏（2026-10-08 需求） ====================
 * 需求原话："严格按照云端的来上报，我们的检测时间间隔会苏醒只是为了检测水位
 *           看有没有超出阈值，超出就上报，其他就按上报时间间隔上报"
 * 所以每次苏醒只有两种结果：
 *   ① 超阈值(level >= level_alarm_thresh_mm) -> 立刻上报
 *   ② 没超 -> 看时间：距上次上报 >= 上报周期 才上报，其余轮次只测液位、不开射频
 * 时间基准用 RTC 秒(app_time_get_unix_ts)：RTC 在 STOP 下继续走，跨轮/跨睡都在。
 * 还没校时(RTC 无效，比如台架刚上电还没收到网关 0x01)就退回"每轮都报"的旧行为，
 * 否则台架上会一轮都不发，看不出东西。
 * ============================================================================ */
/* 上报周期兜底值(s)：运行时状态里的上报间隔非法时用 */
#ifndef APP_REPORT_DEFAULT_S
#define APP_REPORT_DEFAULT_S    30U
#endif
/* 超阈值期间的上报节奏：
 *   1 = 【当前】只要还超着，每次苏醒都再报一次（苏醒周期 = 上报/2，即正常节奏的 2 倍）
 *   0 = 只在"刚超阈值"那一次立刻报，之后不再重复，等落回阈值以下再重新触发 */
#ifndef APP_ALARM_REPEAT_EVERY_WAKE
#define APP_ALARM_REPEAT_EVERY_WAKE 1
#endif
/* 为"对齐上报点"而缩短睡眠时的最小睡眠(s)：防止算出 0 秒把 RTC 唤醒周期设成 0 */
#ifndef APP_SLEEP_ALIGN_MIN_S
#define APP_SLEEP_ALIGN_MIN_S   1U
#endif
/* 液位告警阈值 = 0 视为【关闭告警】：否则 level >= 0 恒真，会变成每次苏醒都报 */
#ifndef APP_ALARM_THRESH_ZERO_MEANS_OFF
#define APP_ALARM_THRESH_ZERO_MEANS_OFF 1
#endif

/* --- 4.2 本轮收发缓存（static：本板栈只有 4KB） --- */
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

/* 处理一帧收到的下行帧：CRC/目的地址校验 -> 打印 -> 执行命令 -> 登记延时 ACK
 * @retval true = 收到的是上报应答(0x80/0x81)，false = 其它 */
static bool app_handle_rx_frame(void)
{
    uint16_t ack_len = 0U;
    AppCmdRet_t ret;
    bool is_ack;

    /* 功能码在第10字节(下标9): AA 55 | ver | gw(3) | dev(3) | func */
    is_ack = ((s_rx_len >= 10U) &&
              ((s_rx_buf[9] == CMD_FUNC_ACK_OK) || (s_rx_buf[9] == CMD_FUNC_ACK_ERR_RETRY)));

    /* app_cmd_process 内部做 CRC + 目的地址 + 帧格式 全量校验 */
    ret = app_cmd_process(s_rx_buf, s_rx_len, s_ack_buf, sizeof(s_ack_buf), &ack_len);

    if (ret == CMD_RET_ERR_CRC)
    {
        log_info("[app] discard: CRC error (%u bytes)\r\n", (unsigned)s_rx_len);
        return false;
    }
    if (ret == CMD_RET_ERR_DST_ADDR)
    {
        log_info("[app] discard: not for this device (%u bytes)\r\n", (unsigned)s_rx_len);
        return false;
    }

    /* 帧校验通过且是发给本机的 */
    log_info("[app] got %u bytes -> ", (unsigned)s_rx_len);
    app_dump_hex(s_rx_buf, s_rx_len);
    log_info("[app] parse ret=%u, reply_len=%u\r\n", (unsigned)ret, (unsigned)ack_len);

    app_print_time();                   /* 网关校时后时间会变，顺手打一下 */

    if ((ret == CMD_RET_OK) && (ack_len > 0U))
    {
        bool is_unbind = ((s_rx_len >= 10U) && (s_rx_buf[9] == CMD_FUNC_UNBIND));

        if (is_unbind)
        {
            uint8_t ack_i;

            /* 0x05 解绑（2026-10-07 改版）：
             *   ① 回 ACK 两次，间隔 APP_UNBIND_ACK_GAP_MS(5s)
             *   ② 再走解绑流程：清 gw_addr = FF FF FF（Flash + RAM）
             *   ③ 关 RTC 唤醒 + 关射频 -> 反复进 STOP 等下次上电（不回配对、不发 0x04）
             * 顺序不能颠倒：清了网关地址就组不出应答帧了，所以两次 ACK 必须都在清之前发完。
             * 不回配对的原因：解绑后本次运行内不再发 0x04，让网关那边能判定"设备已解绑"；
             * 设备自己等下次上电时 app_runtime_state_init() 读到 gw_addr 无效，自动进配对流程。 */
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

            /* ---- ② 解绑流程：清网关地址(Flash + RAM -> FF FF FF) ---- */
            if (app_dev_param_clear_gw_addr())
            {
                uint8_t unbound_addr[PROTO_ADDR_LEN];
                proto_addr_set_unbound(unbound_addr);
                app_runtime_state_set_gw_addr(unbound_addr);
                log_info("[app] unbound: gw_addr=%02X%02X%02X saved\r\n",
                         (unsigned)unbound_addr[0], (unsigned)unbound_addr[1],
                         (unsigned)unbound_addr[2]);
            }
            else
            {
                log_info("[app] unbind: flash write FAILED (gw_addr may persist on next boot)\r\n");
            }

            /* 解绑后是"一条死路"，不再需要被水叫醒；顺手把探针唤醒撤防，
             * 免得设备一直不插拔时电极还被直流激着（撤防后 PA7/PB1 回高阻）。 */
            WaterDetect_WakeDisable();

            /* ---- ③ 进 STOP 等下次上电：关 RTC 周期唤醒(防上一轮 ⑦ 配的周期唤醒) +
             * 关射频(防 DIO1 EXTI 唤醒)；反复进 STOP，只有复位能退出。 ---- */
            log_info("[app] unbind done, enter STOP until next power-on\r\n");
            app_lowpower_config_wakeup(0);
            LoraApp_Sleep();
            while (1)
            {
                app_flow_enter_stop();      /* 方案A：解绑后长时间躺尸也走几 µA 那一档 */
            }
            /* 不会返回 */
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

    return is_ack;
}

/* 接收窗口：什么都收、收到就打印，不挑不判
 * @retval true = 窗口内至少收到一帧 */
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
            (void)app_handle_rx_frame();
            got_any = true;
            frames++;
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

/* 探针判定：本轮是否有水（电压法：空气≈0mV，触水=高电压） */
#if (APP_FLOW_USE_PROBE)
static bool app_probe_has_water(void)
{
#if (WATER_DETECT_ENABLE)
    return (WaterDetect_Scan() == WATER_STATUS_HAS_WATER);
#else
    return true;                        /* 探针驱动关掉时：当作有水 */
#endif
}
#endif

/* 本次唤醒是谁叫醒的（打印用）。
 * 顺序很重要：先看"是不是水叫醒的"（EXTI 置的标志），再看 RTC ——
 * 两者可能同时发生，但"水来了"这个信息更有价值，必须先取。 */
static const char *app_wakeup_source_str(void)
{
    if (WaterDetect_TakeWakeFlag() != 0U)
    {
        return "WATER";                 /* 探针上升沿：水碰到电极，提前叫醒 */
    }
    return app_time_take_wakeup_event() ? "RTC" : "other";
}

/* 液位采样参数：连采 N 次取平均；模块要求两次触发间隔 > 1s */
#if (APP_FLOW_USE_LEVEL)
#ifndef LEVEL_SAMPLE_COUNT
#define LEVEL_SAMPLE_COUNT      5U
#endif
#ifndef LEVEL_SAMPLE_GAP_MS
#define LEVEL_SAMPLE_GAP_MS     1100U
#endif
typedef char level_sample_cnt_check[(LEVEL_SAMPLE_COUNT <= 8U) ? 1 : -1];

/* 【整轮测不到时的重试】(2026-10-09 加，为方案A 的冷启动兜底)
 * 方案A 让超声波模组每次醒来都是"从 1.2V 冷启动"，个别模组 POR/串口就绪比 1s 慢，
 * 于是第一轮 5 个采样全废。这时再补一次"冷启动"重采一轮，只重试一次。
 *   1 = 开（默认）；0 = 关（回到"测不到就保持上次值"的老行为） */
#ifndef APP_LEVEL_RETRY_ON_FAIL
#define APP_LEVEL_RETRY_ON_FAIL 1
#endif
#ifndef APP_LEVEL_REPOWER_LOW_MS
#define APP_LEVEL_REPOWER_LOW_MS 200U   /* 重试时先把外设域放回 1.2V 保持多久(ms) */
#endif
#ifndef APP_LEVEL_RETRY_SETTLE_MS
#define APP_LEVEL_RETRY_SETTLE_MS 1000U /* 非方案A 时重试前的等待(ms) */
#endif

/* ==================== 台架用：伪随机液位源（2026-10-09） ====================
 * 用途：超声波模组【照常工作】（每次都真触发、真收帧），但上报【不采纳】它的数据，
 *       改用按时间戳算的伪随机值，让网关和云端先看到一组会变化的液位。
 *       需求原话："超声波模组要工作，不过我们不采纳他的数据"。
 * 取值：伪随机数，落在 [10, 250] mm 闭区间。
 * 模组真实读数不丢，会以 "module raw(5): ..." 打在日志里，方便看模组到底有没有回数据。
 * ⚠️ 量产必须把 APP_LEVEL_FAKE_TEST 改回 0 —— 打开时上报的是随机数，不是真液位！
 *    打开时每轮日志都会打 "[app] !! level source = FAKE ..."，忘不了。 */
#ifndef APP_LEVEL_FAKE_TEST
#define APP_LEVEL_FAKE_TEST     1
#endif
#ifndef APP_FAKE_LEVEL_MIN_MM
#define APP_FAKE_LEVEL_MIN_MM   10U     /* 取值范围下限(mm) */
#endif
#ifndef APP_FAKE_LEVEL_MAX_MM
#define APP_FAKE_LEVEL_MAX_MM   250U    /* 取值范围上限(mm) */
#endif
/* 上报值不采纳模组读数，所以"整轮全废就重试"这条路径没意义（模组哑不哑都不影响上报），
 * 关掉它省一轮时间。要看模组本身的状态请看日志里的 "module raw(5)"。 */
#if (APP_LEVEL_FAKE_TEST)
#undef  APP_LEVEL_RETRY_ON_FAIL
#define APP_LEVEL_RETRY_ON_FAIL 1
#endif

/**
 * @brief 台架用伪随机液位(mm)，范围 [APP_FAKE_LEVEL_MIN_MM, APP_FAKE_LEVEL_MAX_MM]
 * @return uint16_t 液位毫米值（10~250）
 * @note  三个源混着用，保证"同一秒内连采 5 次"也能拿到 5 个不同的值：
 *          ① RTC 的 unix 秒        —— 换轮次会变（时间戳，主种子）；
 *          ② LLCC68 的 1ms 自由计数 —— 每次调用基本都在变，当毫秒级时间戳用；
 *          ③ 上一次结果自己递推(LCG) —— 前两个源"恰好都没变"时输出仍然不同。
 *        只用 32 位无符号乘加，不调 rand()/srand()（省 Flash/栈，也不依赖库）。
 */
static uint16_t app_fake_level_mm(void)
{
    static uint32_t s_lcg = 0U;
    uint32_t x;

    if (s_lcg == 0U)                                /* 首次调用：用时间戳当种子 */
    {
        s_lcg = app_time_get_unix_ts() ^ 0x5A5A5A5AU;
    }
    s_lcg = (s_lcg * 1664525U) + 1013904223U;       /* 经典 LCG 常数(Numerical Recipes) */
    x     = s_lcg ^ (app_time_get_unix_ts() * 2654435761U) ^ (uint32_t)LLCC68GetTimerCount();

    /* x % (max-min+1) -> 0..240，再加 min 就是 10..250 */
    return (uint16_t)(APP_FAKE_LEVEL_MIN_MM +
                      (x % (uint32_t)(APP_FAKE_LEVEL_MAX_MM - APP_FAKE_LEVEL_MIN_MM + 1U)));
}

/* 采一轮：连采 LEVEL_SAMPLE_COUNT 次。
 *   out_val[] : 【参与平均/上报】的值。假数据模式下这里是伪随机值；
 *   out_raw[] : 超声波模组的【真实读数】（假数据模式下只用来打日志排查；可传 NULL）
 * 返回 out_val[] 里的有效个数。 */
static uint8_t app_level_sample_pass(uint16_t *out_val, uint16_t *out_raw)
{
    uint8_t i;
    uint8_t ok = 0U;

    for (i = 0U; i < LEVEL_SAMPLE_COUNT; i++)
    {
        uint16_t v;

        if (i != 0U)
        {
            delay_ms(LEVEL_SAMPLE_GAP_MS);  /* 模块要求两次触发间隔 >1s */
        }

        /* 【每次都真触发超声波模组】—— 模组要工作，哪怕数据不采纳 */
        v = Level_Measure(APP_LEVEL_TIMEOUT_MS);
        if (out_raw != NULL)
        {
            out_raw[i] = v;
        }

#if (APP_LEVEL_FAKE_TEST)
        /* 不采纳模组读数：上报值改填伪随机数（10~250mm） */
        out_val[i] = app_fake_level_mm();
#else
        out_val[i] = v;
#endif

        if (out_val[i] != 0xFFFFU)
        {
            ok++;
        }
    }
    return ok;
}

/* 测一次液位：连采 LEVEL_SAMPLE_COUNT(默认5) 次取平均，内部负责波特率切换 */
static uint16_t app_read_level(uint16_t last_level)
{
    uint16_t s[8];
    uint32_t sum = 0U;
    uint8_t  ok = 0U;
    uint8_t  i;
    uint16_t level;
#if (APP_LEVEL_FAKE_TEST)
    uint16_t raw[8];                    /* 超声波模组的真实读数（只打日志，不上报） */
#endif
#if (APP_LEVEL_RETRY_ON_FAIL)
    uint8_t  retried = 0U;
#endif

#if (APP_LEVEL_FAKE_TEST)
    /* 台架模式：模组照常工作，但上报值不采纳它的读数。每轮打一行醒目提示，免得量产忘了关。 */
    log_info("[app] !! level source = FAKE %u-%umm (module still triggered, data NOT used; 量产必须改 0)\r\n",
             (unsigned)APP_FAKE_LEVEL_MIN_MM, (unsigned)APP_FAKE_LEVEL_MAX_MM);
#endif
    Level_UART_Init();                  /* PA10 重配 + USART1 -> 9600（模组要工作，必须初始化） */

#if (APP_LEVEL_FAKE_TEST)
    ok = app_level_sample_pass(s, raw);
#else
    ok = app_level_sample_pass(s, NULL);
#endif

#if (APP_LEVEL_RETRY_ON_FAIL)
    if (ok == 0U)
    {
        /* 整轮全废 -> 补一次冷启动：PB7 上拉关断外设域到 0V -> 驱动低拉回 3.3V 触发 POR -> 重新 Init */
        BSP_Periph_Power_PullUp();
        delay_ms(APP_LEVEL_REPOWER_LOW_MS);
        Level_TrigHoldLow();
        BSP_Periph_Power_On();
        delay_ms(APP_PERIPH_POR_MS);
        Level_UART_Init();
        ok = app_level_sample_pass(s, NULL);  /* 上一轮全是 65535，没有信息量，直接覆盖 */
        retried = 1U;
    }
#endif

    Level_UartSetBaud(LOG_BAUD);        /* 【必须】切回日志波特率再打印 */

#if (APP_LEVEL_RETRY_ON_FAIL)
    if (retried != 0U)
    {
        log_info("[app] pass1 all invalid -> module re-power/re-init, pass2 shown below\r\n");
    }
#endif
#if (APP_LEVEL_FAKE_TEST)
    /* 模组到底有没有回数据，看这一行（65535 = 这一发没收到帧）。它不参与上报。 */
    log_info("[app] module raw(%u):", (unsigned)LEVEL_SAMPLE_COUNT);
    for (i = 0U; i < LEVEL_SAMPLE_COUNT; i++)
    {
        log_info(" %u", (unsigned)raw[i]);
    }
    log_info("\r\n");
#endif
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

    /* 平均值 = 有效采样之和 / 有效个数（无效的 65535 不参与） */
    for (i = 0U; i < LEVEL_SAMPLE_COUNT; i++)
    {
        if (s[i] != 0xFFFFU)
        {
            sum += s[i];
        }
    }
    level = (uint16_t)(sum / (uint32_t)ok);
    log_info("  -> avg=%umm (valid %u/%u)\r\n",
             (unsigned)level, (unsigned)ok, (unsigned)LEVEL_SAMPLE_COUNT);
    return level;
}
#endif /* APP_FLOW_USE_LEVEL */

/* 休眠周期取值：运行时状态里的 detect_interval_s，非法就回默认 30s
 * 注：合法的下限用 APP_SLEEP_MIN_S(5s)，不是 APP_CMD_DETECT_INTERVAL_MIN_S(300s)，
 *     理由见上面 APP_SLEEP_MIN_S 的注释。 */
static uint16_t app_flow_get_sleep_s(const AppRuntimeState_t *rt)
{
    uint16_t s = rt->sensor_cfg.detect_interval_s;

    if ((s < APP_SLEEP_MIN_S) || (s > APP_CMD_SLEEP_INTERVAL_MAX_S))
    {
        s = APP_SLEEP_DEFAULT_S;
    }
    return s;
}

/* 上报周期取值：运行时状态里的 report_interval_s，非法就回默认 30s
 * 注：上报周期是云端唯一下发、也是唯一可信的间隔；检测周期由它推导(上报/2)，
 *     见 app_cmd.h【检测间隔的推导规则】。 */
static uint16_t app_flow_get_report_s(const AppRuntimeState_t *rt)
{
    uint16_t s = rt->sensor_cfg.report_interval_s;

    if ((s == 0U) || (s > APP_CMD_SLEEP_INTERVAL_MAX_S))
    {
        s = APP_REPORT_DEFAULT_S;
    }
    return s;
}

/* 正式主流程（不返回）
 *   一轮 = ① 网关地址检查(没绑就先配对) -> ② 探针判水 -> ③ 测液位
 *        -> ④ 上报判定(超阈值/到上报点才发, 其余轮次只测不发)
 *        -> ⑤(仅上报轮) win1/win2/win3 三段接收窗口(收指令并回 ACK)
 *        -> ⑥ 本轮被解绑就进 STOP 等下次上电
 *        -> ⑦ 睡前定时 = min(检测周期, 距下次上报的剩余时间)，再进 STOP */
static void app_mode_production(void)
{
    AppRuntimeState_t rt;
    uint16_t sleep_s;
    uint16_t new_sleep_s;
    uint16_t level;
    bool     has_water;
    bool     got_ack;
    bool     do_report;
    bool     time_ok;
    const char *why;
    const char *wake_src;
    uint32_t now_s;
    uint32_t report_period_s;
    uint32_t last_report_ts = 0U;       /* 上次"上报时刻"的 RTC 秒 */
    bool     report_ts_valid = false;   /* false = 还没有基准 -> 第一轮必报 */
    bool     alarm_active;
#if (!APP_ALARM_REPEAT_EVERY_WAKE)
    bool     alarm_latched = false;     /* 边沿触发用：只在刚超阈值那一次报 */
#endif
#if (APP_FLOW_USE_LEVEL)
    uint16_t last_level = 0U;           /* 最新有效液位；全采样无效时兜底 */
#endif

    /* 上电初始化顺序：参数 -> 运行时状态 -> RTC -> 低功耗 -> 外设电源脚。
     * 运行时状态必须在 LoRa 起来之前初始化：app_cmd_process() 用 rt.dev_addr 做
     * 目的地址匹配，没初始化就是 00 00 00，网关下发的帧会被直接丢掉。 */
    app_dev_param_init();
    app_runtime_state_init();
    (void)app_time_init();
    app_lowpower_init();
#if (APP_FLOW_USE_PROBE)
    WaterDetect_Init();                 /* 探针脚先置高阻 */
#endif

    log_info("\r\n===== mode 4: production (pair + level + report + lowpower) =====\r\n");
    app_print_params();
    app_print_time();
    app_print_rf_cfg();
    log_info("[app] flow: probe=%u level_sensor=%u fixed_level=%umm win=%ums ack_delay=%ums fake_level=%u\r\n",
             (unsigned)APP_FLOW_USE_PROBE, (unsigned)APP_FLOW_USE_LEVEL,
             (unsigned)APP_FIXED_REPORT_LEVEL_MM,
             (unsigned)APP_RX_WIN_MS, (unsigned)APP_CMD_ACK_DELAY_MS,
             (unsigned)APP_LEVEL_FAKE_TEST);

    while (1)
    {
        /* 先读唤醒源（读后清标志，纯粹为了日志顺序好看：先报"谁叫醒的"，再报"电源恢复"） */
        wake_src = app_wakeup_source_str();

        /* ---- ⓪ 唤醒后第一件事：恢复外设域电源 ----
         * 方案A 下睡前 PB7 是高阻、外设域只有 ~1.2V（外设处于复位态），
         * 所以这里必须先驱动低拉回 3.3V + 等 POR，后面才能碰液位/LoRa。
         * 放在 ① 之前：配对模式也要用射频，别让 app_pair_run() 拿到没上电的模组。 */
        app_periph_power_restore();

        app_runtime_state_get(&rt);

        /* ---- ① 网关地址无效 = 还没配对 -> 进配对模式 ----
         * 注意：按需求，配对成功之后【不会从这里返回】—— 设备会留在配对模式里
         * 一直回 ACK，直到下次上电（那时 Flash 里已有有效网关地址）才走到下面
         * 的探针/液位/上报/休眠。见 app_pair.c 的 APP_PAIR_STAY_AFTER_BOUND。 */
        if (!proto_addr_is_valid(rt.gw_addr))
        {
            log_info("[app] gw_addr=%02X%02X%02X unbound -> pairing mode (0x04 every 5s)\r\n",
                     (unsigned)rt.gw_addr[0], (unsigned)rt.gw_addr[1], (unsigned)rt.gw_addr[2]);
            app_pair_run();             /* 写 Flash 绑定；之后留在此处一直回 ACK，不返回 */
            continue;                   /* 只有 APP_PAIR_STAY_AFTER_BOUND=0 才会走到 */
        }

        sleep_s = app_flow_get_sleep_s(&rt);
        /* 本轮的时间基准（一次读定，后面判定/定时都用它，避免跨秒误差）：
         *   now_s    = RTC 秒；time_ok = RTC 是否已校时（未校时不能用它做调度） */
        now_s           = app_time_get_unix_ts();
        time_ok         = (app_time_is_valid() != 0U);
        report_period_s = (uint32_t)app_flow_get_report_s(&rt);
        log_info("\r\n[app] wakeup (%s) detect_cfg=%us report_cfg=%lus rtc=%s\r\n",
                 wake_src, (unsigned)sleep_s, (unsigned long)report_period_s,
                 time_ok ? "ok" : "NOT-SYNCED");

        /* ---- ② 探针判水 ----
         * ⚠️【2026-09-30 修正】这里原来写的是 has_water = false，
         *    也就是"探针关掉"= 当作【无水】-> 下面直接 level=0、【根本不触发超声波模块】。
         *    于是把 APP_FLOW_USE_PROBE 改 0 想单独测超声波时，看到的永远是
         *    "water=0 level=0mm"，很容易误判成"超声波模块坏了"。
         *    语义统一成：探针关掉 = 不判水、直接测液位（与 WATER_DETECT_ENABLE=0
         *    时 app_probe_has_water() 返回"有水"一致）。 */
#if (APP_LEVEL_FAKE_TEST)
        /* 台架模式：不判水。因为探针干着（现场常态）时按原逻辑会直接 level=0、
         * 【根本不触发超声波模组】—— 那就既看不到模组工作，也走不到 5 次采样。
         * 所以这里当成"有水"，让模组每轮都被触发（数据仍不采纳，只打 module raw 日志）。
         * 要"探针判水 + 模组工作"同时存在，把这两行换成 app_probe_has_water() 即可。 */
        has_water = true;
        log_info("[app] fake-level mode: skip probe -> ultrasonic is triggered every round\r\n");
#elif (APP_FLOW_USE_PROBE)
        has_water = app_probe_has_water();
#else
        has_water = true;                   /* 探针不开：不判水，直接测液位 */
#endif

        /* ---- ③ 测液位 ----
         * 有水：真触发超声波模块连采取平均；
         * 无水：不碰超声波模块，上报液位 0（省电），并且【不覆盖 last_level】。 */
#if (APP_FLOW_USE_LEVEL)
        if (has_water)
        {
            level = app_read_level(last_level);
            last_level = level;             /* 只在真读到液位时更新"上次有效值" */
        }
        else
        {
            level = 0U;
            log_info("[app] probe=dry -> skip ultrasonic, report level 0\r\n");
        }
#else
        level = APP_FIXED_REPORT_LEVEL_MM;
#endif

        /* ---- ④ 上报判定 ----
         *   超阈值 -> 立刻上报；否则只有"距上次上报 >= 上报周期"才上报。
         *   不报的轮次只做液位检测：不开射频、不开接收窗口（省电，也不会干扰上报轮）。 */
#if (APP_ALARM_THRESH_ZERO_MEANS_OFF)
        alarm_active = (rt.sensor_cfg.level_alarm_thresh_mm != 0U) &&
                       (level >= rt.sensor_cfg.level_alarm_thresh_mm);
#else
        alarm_active = (level >= rt.sensor_cfg.level_alarm_thresh_mm);
#endif

        do_report = false;
        why = "";
        if (alarm_active)
        {
#if (APP_ALARM_REPEAT_EVERY_WAKE)
            do_report = true;
            why = "alarm";
#else
            if (!alarm_latched)
            {
                do_report = true;
                why = "alarm(edge)";
            }
#endif
        }
        if (!do_report)
        {
            if (!time_ok)
            {
                do_report = true;               /* 没校时：退回"每轮都报"，台架才看得到 */
                why = "rtc-not-synced";
            }
            else if (!report_ts_valid)
            {
                do_report = true;               /* 上电/配对后的第一帧立刻发 */
                why = "first";
            }
            else if ((uint32_t)(now_s - last_report_ts) >= report_period_s)
            {
                do_report = true;
                why = "period";
            }
        }

        log_info("[app] water=%u level=%umm thresh=%umm alarm=%u -> %s\r\n",
                 (unsigned)(has_water ? 1U : 0U), (unsigned)level,
                 (unsigned)rt.sensor_cfg.level_alarm_thresh_mm,
                 (unsigned)(alarm_active ? 1U : 0U),
                 do_report ? "REPORT" : "skip report");

        if (do_report)
        {
            log_info("[app] report why=%s: elapsed=%lus/%lus\r\n",
                     why,
                     (unsigned long)(report_ts_valid ? (uint32_t)(now_s - last_report_ts) : 0U),
                     (unsigned long)report_period_s);

            LoraApp_Init();             /* 只有真要收发才拉射频 */
            LoraApp_Standby();
            if (!app_send_report(level))
            {
                log_info("[app] report send FAILED, still open rx window\r\n");
            }
            /* 上报基准 = 这一帧真发出去的时刻（不是本轮开始时刻）：
             * 这样"上报周期"从帧发出那一刻算起，收发窗口那几十秒也不会累积成偏差。
             * 发送失败也照样推进基准：失败就当"这轮报过了"，下一轮(检测周期)再试，
             * 否则 remain 会一直算成 0，退化成 1 秒一轮的热循环。 */
            last_report_ts  = app_time_get_unix_ts();
            report_ts_valid = true;

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

            /* 校时结果顺手落一次 Flash（内部节流到 1 小时一次）：
             * 上报调度靠 RTC，掉电重启后要先从 Flash 恢复时间，否则只能"每轮都报"。 */
            app_time_save_to_flash();
        }
        else
        {
            /* 只检测、不上报：连射频都不初始化，直接去睡 */
            log_info("[app] no report: elapsed=%lus < period=%lus -> level check only (no RF)\r\n",
                     (unsigned long)((uint32_t)(now_s - last_report_ts)),
                     (unsigned long)report_period_s);
        }
#if (!APP_ALARM_REPEAT_EVERY_WAKE)
        if (alarm_active)
        {
            alarm_latched = true;       /* 边沿触发：超阈值只报一次，落回后重新武装 */
        }
#endif

        /* ---- ⑥ 防御兜底：本轮 gw_addr 变无效(理论上 0x05 已在 app_handle_rx_frame
         *      里发完 ACK+清完+STOP，不会走到这里) -> 同样进 STOP 等下次上电，
         *      不回配对(避免被解绑后又在本次运行内重新配对，干扰网关阶段2验证)。 ---- */
        app_runtime_state_get(&rt);
        if (!proto_addr_is_valid(rt.gw_addr))
        {
            log_info("[app] gw_addr invalid after rx window -> STOP until next power-on\r\n");
            app_lowpower_config_wakeup(0);
            LoraApp_Sleep();
            while (1)
            {
                app_flow_enter_stop();
            }
        }

        /* ---- ⑦ 睡前定时：sleep = min(检测周期, 距下次上报的剩余时间) ----
         * 为什么不能只按检测周期睡：上报点必须落在"正好到点"的那次苏醒上。
         * 例：上报 900s、检测 450s 时两者整除没问题，但上报 601s、检测 300s 时
         *     只按 300s 睡会睡成 300/600/900...，上报会晚到第 3 次苏醒(900s)；
         *     按剩余时间睡就正好在 601s 那次醒来发（300 + 301）。
         * 时间没校准时没法算剩余，就只按检测周期睡（此时本来就是每轮都报）。 */
        new_sleep_s = app_flow_get_sleep_s(&rt);
        if (new_sleep_s != sleep_s)
        {
            log_info("[app] sleep interval changed by gateway: %us -> %us\r\n",
                     (unsigned)sleep_s, (unsigned)new_sleep_s);
            sleep_s = new_sleep_s;
        }
        report_period_s = (uint32_t)app_flow_get_report_s(&rt);
        /* ⚠️【2026-10-10 修 bug】这里必须【重新读一次】RTC 校时状态，不能用本轮开头那次
         * 快照 time_ok：本轮很可能刚刚才收到网关 0x01（里面就带校时），本轮开头读的时候
         * 还是 NOT-SYNCED -> 对齐被跳过 -> 睡 300s，而上报周期只有 60s
         * （现场日志就是：0x01 report=60s 之后 "enter STOP, wake in 300s"）。 */
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

        /* ---- 【持久化云端配置】(2026-10-10 修) ----
         * 0x01 下发的 detect/report/阈值 原来【只在 RAM 里用】，掉电/复位就回到 Flash 里的
         * 出厂默认(detect=30s report=30s) —— 现场表现就是："云端把上报设成 120s，
         * 断电再上电后睡眠从 100s 变成 10s"（因为周期变回 30s，对齐算出来就是 ~10s）。
         * 这里睡前落一次 Flash。
         * 为什么放在这里而不是收到 0x01 的那一刻：Flash 擦写会停总线 ~20-40ms，
         * 别在收包窗口里做（此时射频已睡、接收窗口已关）。
         * app_dev_param_set_sensor_cfg() 内部有去重：配置没变直接返回、不擦写，
         * 所以不怕网关每轮重发同一条 0x01。 */
        if (!app_dev_param_set_sensor_cfg(&rt.sensor_cfg))
        {
            log_info("[app] !! persist sensor cfg FAILED (flash write)\r\n");
        }

        app_lowpower_config_wakeup(sleep_s);
        log_info("[app] enter STOP, wake in %us (detect_cfg=%us report_cfg=%lus)\r\n",
                 (unsigned)sleep_s, (unsigned)app_flow_get_sleep_s(&rt),
                 (unsigned long)report_period_s);
        log_info("[app] sleep: PB7 pull-up -> periph domain 0V (few uA)\r\n");
        delay_ms(5U);
        app_flow_enter_stop();          /* 阻塞，直到 RTC 周期唤醒；方案B 睡前 PB7 输入上拉 */
    }
}

/* ============================================================================
 *   模式 5：只读液位（不睡，2s 一次，方便边动传感器边看数值）
 * ----------------------------------------------------------------------------
 * 已确认：波特率 9600，帧 = FF + 距离高 + 距离低 + 校验(前三字节之和低8位)，单位 mm。
 *
 * 【读不到时按这个顺序查】
 *   ① raw(...) 一直是 0 字节 -> PA10 上一个字节都没来，查：
 *        模块有没有供电(PB7 外设域)、TRIG(PA9) 有没有接到模块的 RX、
 *        模块 TX 有没有接到 PA10、模块是不是被跳线设成了"非 UART 输出"模式。
 *   ② raw 有字节但从来不是合法帧(FF + H + L + sum) -> 波特率或帧格式不对：
 *        把 LEVEL_TEST_SCAN 改成 1，它会 9600/115200/19200/38400/57600/4800/2400
 *        逐个试并打印每档抓到的原始字节，哪一档能解析出来就是它。
 *   ③ 能解析但一直是 0mm -> 换能器前没有有效回波(贴桌面/被挡/对着远处)，
 *        放一本书或纸板在正前方 30~50cm 再看数值会不会变。
 *
 * LEVEL_TEST_SLEEP_S：0 = 不睡连续读（调传感器用）；N = 读完睡 N 秒。
 * ============================================================================ */
#ifndef LEVEL_TEST_SLEEP_S
#define LEVEL_TEST_SLEEP_S  0U
#endif
#ifndef LEVEL_TEST_PERIOD_MS
#define LEVEL_TEST_PERIOD_MS    2000U
#endif
/* 0 = 正常读（默认，9600）；1 = 波特率扫描（换未知模块/怀疑波特率时用）
 * ⚠️ 这个扫描开关是重写 main.c 时【漏掉又补回来】的：查超声波不确认波特率时它最省事。 */
#ifndef LEVEL_TEST_SCAN
#define LEVEL_TEST_SCAN     0
#endif

static void app_mode_level_bench(void)
{
    uint8_t  raw[32];
    uint16_t n;
    uint16_t level;
    uint16_t k;

#if (LEVEL_TEST_SLEEP_S > 0U)
    (void)app_time_init();              /* RTC(LSI)，STOP 的唤醒源 */
    app_lowpower_init();
#endif
    delay_ms(APP_PERIPH_SETTLE_MS);

    Level_UART_Init();                  /* PA10 = USART1_RX(AF4) + 触发脚 PA9 */

#if (LEVEL_TEST_SCAN)
    /* ---------------- 波特率扫描 ---------------- */
    static const uint32_t s_baud_list[] = {9600U, 115200U, 19200U, 38400U, 57600U, 4800U, 2400U};
    uint32_t bi;

    log_info("\r\n===== mode 5: level bench BAUD SCAN =====\r\n");
    log_info("[level] TRIG=PA9 RX=PA10  log_baud=%lu  candidates:",
             (unsigned long)LOG_BAUD);
    for (bi = 0U; bi < (uint32_t)(sizeof(s_baud_list) / sizeof(s_baud_list[0])); bi++)
    {
        log_info(" %lu", (unsigned long)s_baud_list[bi]);
    }
    log_info("\r\n");

    while (1)
    {
        for (bi = 0U; bi < (uint32_t)(sizeof(s_baud_list) / sizeof(s_baud_list[0])); bi++)
        {
            Level_UartSetBaud(s_baud_list[bi]);     /* 1) 切到候选波特率 */
            Level_RawClear();
            level = Level_Measure(APP_LEVEL_TIMEOUT_MS);
            Level_UartSetBaud(LOG_BAUD);            /* 2) 切回日志波特率再打印 */

            n = Level_RawTake(raw, (uint16_t)sizeof(raw));
            log_info("[level] baud=%lu parsed=%u raw(%u): ",
                     (unsigned long)s_baud_list[bi], (unsigned)level, (unsigned)n);
            for (k = 0U; k < n; k++)
            {
                log_info("%02X ", (unsigned)raw[k]);
            }
            log_info("\r\n");

            delay_ms(400);                          /* 两次触发间隔必须 >1s */
        }
    }
#else
    /* ---------------- 正常读（9600） ---------------- */
    uint32_t ok   = 0U;
    uint32_t fail = 0U;

    log_info("\r\n===== mode 5: level bench (baud=%lu, sleep=%us) =====\r\n",
             (unsigned long)LEVEL_UART_BAUD, (unsigned)LEVEL_TEST_SLEEP_S);
    log_info("[level] TRIG=PA9  RX=PA10(USART1_RX,AF4)\r\n");

    while (1)
    {
        Level_UART_Init();              /* 唤醒后重新把 PA10 配成 USART1_RX */
        Level_RawClear();
        level = Level_Measure(APP_LEVEL_TIMEOUT_MS);
        Level_UartSetBaud(LOG_BAUD);    /* 打印前切回日志波特率 */

        n = Level_RawTake(raw, (uint16_t)sizeof(raw));

        if (level == 0xFFFFU)
        {
            fail++;
            log_info("[level] no frame   ok=%lu fail=%lu  raw(%u): ",
                     (unsigned long)ok, (unsigned long)fail, (unsigned)n);
        }
        else
        {
            ok++;
            log_info("[level] %umm   ok=%lu fail=%lu  raw(%u): ",
                     (unsigned)level, (unsigned long)ok, (unsigned long)fail, (unsigned)n);
        }
        for (k = 0U; k < n; k++)
        {
            log_info("%02X ", (unsigned)raw[k]);
        }
        log_info("\r\n");

        if (n == 0U)
        {
            /* 一个字节都没来：不是解析问题，是链路/供电/模式问题（见本函数顶部排查清单） */
            log_info("[level]  raw=0 -> no byte from module: check VCC(PB7 domain)/TRIG(PA9)/module TX->PA10/UART mode\r\n");
        }

#if (LEVEL_TEST_SLEEP_S > 0U)
        app_lowpower_config_wakeup(LEVEL_TEST_SLEEP_S);
        log_info("[level] sleep %us\r\n", (unsigned)LEVEL_TEST_SLEEP_S);
        delay_ms(5U);
        app_lowpower_enter_stop_pullup();
        /* 唤醒后外设域被关断到0V，先恢复上电+等POR再重新Init液位 */
        app_periph_power_restore();
        Level_UART_Init();
#else
        delay_ms(LEVEL_TEST_PERIOD_MS); /* 两次触发间隔必须 >1s */
#endif
    }
#endif
}

/* ============================================================================
 *   模式 6：【旧台架】探针标定（每秒一组，看 H/L 两相电压）
 * ----------------------------------------------------------------------------
 * 可调参数都在 water_cu.h：
 *   PROBE_SETTLE_US（相位稳定时间）、PROBE_SAMPLES（每相位采样次数）、
 *   PROBE_STRONG_DRIVE（0=内部弱拉激励 / 1=PA7 推挽强激励）、
 *   WATER_DETECT_WET_RAW_THRESHOLD（判水阈值，标定完再填）
 * 自检：PA7 完全不驱动时 Pu 应≈4095、Pd 应≈0；不对说明内部上下拉没生效或
 *       网络上有别的器件在拉（硬件问题），那时电压法也测不准。
 * ============================================================================ */
#ifndef PROBE_TEST_PERIOD_MS
#define PROBE_TEST_PERIOD_MS    1000U
#endif

static uint16_t probe_raw_to_mv(uint16_t raw, uint16_t vdd_mv)
{
    return (uint16_t)(((uint32_t)raw * (uint32_t)vdd_mv) / 4095U);
}

static void app_mode_probe_bench(void)
{
    uint16_t v_raw;
    uint16_t v_mv;
    uint16_t base_raw;
    uint16_t base_mv;
    uint16_t vdd;
    uint8_t  dig;
    uint8_t  wet;
    uint16_t mx[6];

    delay_ms(APP_PERIPH_SETTLE_MS);
    WaterDetect_Init();                 /* PA7(激励) / PB1(检测) 先都置高阻 */

    log_info("\r\n===== mode 6: probe bench (voltage method) =====\r\n");
    log_info("[probe] settle=%uus samples=%u thresh_raw=%u(about %umV@3.3V)\r\n",
             (unsigned)PROBE_SETTLE_US, (unsigned)PROBE_SAMPLES,
             (unsigned)WATER_DETECT_WET_RAW_THRESHOLD,
             (unsigned)(((uint32_t)WATER_DETECT_WET_RAW_THRESHOLD * 3300U) / 4095U));
    log_info("[probe] expect: air V~0mV, water V higher, base ~0 in both states\r\n");

    while (1)
    {
        vdd      = app_power_get_battery_mv();
        v_raw    = WaterDetect_ReadVoltageRaw();    /* 电压法：空气≈0，触水=高 */
        v_mv     = probe_raw_to_mv(v_raw, vdd);
        base_raw = WaterDetect_ReadBaseRaw();       /* 不激励底噪：应≈0 */
        base_mv  = probe_raw_to_mv(base_raw, vdd);
        dig      = WaterDetect_ReadDigital();
        wet      = (v_raw >= WATER_DETECT_WET_RAW_THRESHOLD) ? 1U : 0U;

        log_info("[probe] V=%umV raw=%u -> %s | dig=%u | base=%umV raw=%u | vdd=%umV\r\n",
                 (unsigned)v_mv, (unsigned)v_raw, (wet != 0U) ? "WATER" : "air(dry)",
                 (unsigned)dig, (unsigned)base_mv, (unsigned)base_raw, (unsigned)vdd);

        WaterDetect_ReadMatrix(mx);
        log_info("[probe2] selfcheck Pu=%u(expect~4095) Pd=%u(expect~0) | ppH_Pd=%u ppL_Pu=%u\r\n",
                 (unsigned)mx[0], (unsigned)mx[1], (unsigned)mx[2], (unsigned)mx[3]);

        delay_ms(PROBE_TEST_PERIOD_MS);
    }
}

/* ============================================================================
 *   模式 7：低功耗 + 液位(连采取均) + LoRa 收发(N次×RX窗口) 循环
 * ----------------------------------------------------------------------------
 *   流程：STOP 唤醒 -> 测液位(app_read_level 内部连采 LEVEL_SAMPLE_COUNT
 *        次取平均,默认 5 次,每次间隔 1.1s,并打印每个采样值和均值)
 *        -> LoRa 发 LP_LORA_TEST_TX_N 次,每次发完开 LP_LORA_TEST_RX_WIN_MS
 *        接收窗口 -> 进 STOP,RTC 唤醒回到开头。
 *   用法：MAIN_MODE 改 7,或 Keil Define MAIN_MODE=7。
 *   可调：LP_LORA_TEST_TX_N(发送次数) LP_LORA_TEST_RX_WIN_MS(每次 RX 窗口)
 *        LP_LORA_TEST_SLEEP_S(STOP 秒数)；液位采样次数改 LEVEL_SAMPLE_COUNT。
 *   注：液位/上报/接收窗口复用 production 的 app_read_level/app_send_report/
 *       app_rx_window；未配对也能跑(app_send_report 用 rt 里的地址组帧,
 *       gw_addr=FF FF FF 时发广播帧,网关仍可收)。 */
#define MAIN_MODE_LP_LEVEL_LORA_LOOP  7

#ifndef LP_LORA_TEST_TX_N
#define LP_LORA_TEST_TX_N             2U       /* 发送次数：每发一次开一个 RX 窗口 */
#endif
#ifndef LP_LORA_TEST_RX_WIN_MS
#define LP_LORA_TEST_RX_WIN_MS        8000U    /* 每次发送后的接收窗口(ms) */
#endif
#ifndef LP_LORA_TEST_SLEEP_S
#define LP_LORA_TEST_SLEEP_S          20U      /* STOP 唤醒间隔(秒) */
#endif
/* 唤醒后 PB7 恢复驱动低后,等外设域 VCC 爬升 + 外设 POR 的延时(ms)。
 * 超声波模块上电到能响应通常要几百 ms,先 500ms,测不出再加长。 */
#ifndef LP_LORA_TEST_HIZ_SETTLE_MS
#define LP_LORA_TEST_HIZ_SETTLE_MS    500U
#endif

static void app_mode_lp_level_lora_loop(void)
{
    uint16_t level;
    uint16_t last_level = 0U;
    uint32_t cycle = 0U;
    uint16_t tx_i;

    app_dev_param_init();
    app_runtime_state_init();
    (void)app_time_init();
    app_lowpower_init();

    log_info("\r\n===== mode 7: lp + level(5 samples avg) + lora tx*%u(rx %ums) + STOP %us =====\r\n",
             (unsigned)LP_LORA_TEST_TX_N, (unsigned)LP_LORA_TEST_RX_WIN_MS,
             (unsigned)LP_LORA_TEST_SLEEP_S);
    app_print_params();
    app_print_time();
    app_print_rf_cfg();

    while (1)
    {
        cycle++;
        log_info("\r\n[lp-lora] cycle %lu: wakeup (%s)\r\n",
                 (unsigned long)cycle,
                 app_time_take_wakeup_event() ? "RTC" : "other");

        /* 方案B:STOP 期间 PB7 输入上拉关断外设域,唤醒后必须先恢复电源 + 延时等 POR。
         * 顺序:先把 TRIG(PA9) 钉低再上电,免得模组冷启动 POR 期间 TRIG 悬空误触发。 */
        Level_TrigHoldLow();
        BSP_Periph_Power_On();
        log_info("[lp-lora] PB7 restored to drive-low, settle %ums\r\n",
                 (unsigned)LP_LORA_TEST_HIZ_SETTLE_MS);
        delay_ms(LP_LORA_TEST_HIZ_SETTLE_MS);

        /* ---- ① 液位检测：app_read_level 内部连采 LEVEL_SAMPLE_COUNT(默认 5)
         *      次取平均,每次间隔 1.1s,并打印每个采样值和均值 ---- */
#if (APP_FLOW_USE_LEVEL)
        level = app_read_level(last_level);
        if (level != 0U)
        {
            last_level = level;
        }
#else
        level = APP_FIXED_REPORT_LEVEL_MM;
        log_info("[lp-lora] level sensor off, fixed level=%umm\r\n", (unsigned)level);
#endif
        log_info("[lp-lora] level=%umm -> lora tx*%u\r\n",
                 (unsigned)level, (unsigned)LP_LORA_TEST_TX_N);

        /* ---- ② LoRa 收发：发 N 次,每次发送后开 8s 接收窗口 ----
         *   app_rx_window 内部 LoraApp_ReciveMode 开收、收到帧调
         *   app_handle_rx_frame(解析+回ACK)、窗口结束 LoraApp_Standby。 */
        LoraApp_Init();
        LoraApp_Standby();
        for (tx_i = 0U; tx_i < LP_LORA_TEST_TX_N; tx_i++)
        {
            log_info("[lp-lora] tx %u/%u: send report (level=%umm)\r\n",
                     (unsigned)(tx_i + 1U), (unsigned)LP_LORA_TEST_TX_N, (unsigned)level);
            if (!app_send_report(level))
            {
                log_info("[lp-lora]   send FAILED, still open rx\r\n");
            }
            (void)app_rx_window(LP_LORA_TEST_RX_WIN_MS, "rx: wait ACK/cmd");
            (void)app_cmd_tick_ack(true, false);   /* 窗口里排队的 ACK 立刻发掉 */
        }
        LoraApp_Sleep();                            /* 射频进深睡,别停在 Standby */

        /* ---- ③ 进 STOP,RTC 唤醒后回到 ① ---- */
        app_lowpower_config_wakeup(LP_LORA_TEST_SLEEP_S);
        log_info("[lp-lora] enter STOP %us (PB7 PullUp, plan B, few uA)\r\n",
                 (unsigned)LP_LORA_TEST_SLEEP_S);
        delay_ms(5U);
        app_lowpower_enter_stop_pullup();
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

    /* PB7 = 外设总电源使能。这里把"当前实际电平"读出来打一行：
     * 驱动低(=0) = 外设上电；读到 1 说明档位不对/被外部拉高，外设会没电。
     * PERIPH_PWR_FORCE：0=高阻(外设只有1.2V) 1=驱动低(默认,外设上电) 2=驱动高(会复位,别用)。 */
    log_info("[pwr] PB7(periph power EN) level=%u, PERIPH_PWR_FORCE=%u (0=hi-Z 1=drive-low 2=drive-high)\r\n",
             (unsigned)BSP_Periph_PowerPeek(), (unsigned)PERIPH_PWR_FORCE);

    log_info("\r\n===== N32G031 level node: MAIN_MODE=%u =====\r\n", (unsigned)MAIN_MODE);

#if   (MAIN_MODE == MAIN_MODE_LOWPOWER_TEST)
    app_mode_lowpower_test();           /* 不返回：醒 5s + 睡 20s */
#elif (MAIN_MODE == MAIN_MODE_LORA_DEBUG)
    app_mode_lora_debug();              /* 不返回：一直接收/一直发送/发+收 */
#elif (MAIN_MODE == MAIN_MODE_PAIR_TEST)
    app_mode_pair_test();               /* 不返回：配对成功即停(不写 Flash) */
#elif (MAIN_MODE == MAIN_MODE_PRODUCTION)
    app_mode_production();              /* 不返回：配对 + 液位 + 上报 + 低功耗 */
#elif (MAIN_MODE == MAIN_MODE_LEVEL_BENCH)
    app_mode_level_bench();             /* 不返回：旧台架，只读液位 */
#elif (MAIN_MODE == MAIN_MODE_LP_LEVEL_LORA_LOOP)
    app_mode_lp_level_lora_loop();     /* 不返回：低功耗+液位5次+LoRa收发循环 */
#else
    app_mode_probe_bench();             /* 不返回：旧台架，探针标定 */
#endif
}
