/**
 * @file app_lora.c
 * @brief LoRa(LLCC68) 收发实现
 *
 * 只做两件事：发送（Radio.Send -> 等 TxDone）与接收（DIO1 把包放进缓存，
 * 主循环 LoraApp_ProcessIrq() 里取）。组帧/打包在业务层，整包传进来即可。
 */
#include <string.h>
#include "n32g031.h"
#include "n32g031_rcc.h"    /* 读回真实主频，用来校验时间基准 */
#include "app_lora.h"
#include "radio.h"
#include "LLCC68-board.h"
#include "log.h"
#include "delay.h"      /* delay_ms(): 按寄存器实测主频计时，可用来校验时间基准 */

/* radio 事件标志（只有 TxTimeout/RxTimeout 是中断里置位，其余在主循环） */
static volatile bool g_txDone    = false;
static volatile bool g_txTimeout = false;
static volatile bool g_rxTimeout = false;
static volatile bool g_rxFlag    = false;   /* 收到一包 */

static uint8_t  g_rxBuf[LORA_RX_BUF_LEN];
static uint16_t g_rxLen = 0U;
static bool     g_rxForever = false;        /* true: 持续接收模式 */

/* ==================== 射频"还活着吗"：BUSY 超时监测 ====================
 * 板级 LLCC68WaitOnBusy() 有 50ms 硬超时，超时就把计数 +1；这里把"计数变了"
 * 翻译成"自上次检查后射频没响应过"，交给 main 决定跳过本轮收发。
 *
 * 【为什么不用看门狗兜底】IWDG 挂在 LSI 上，STOP 期间照样计数，一睡 10 分钟必被复位；
 * N32G031 选项字节里也没有"IWDG 在 STOP 冻结"这一位。所以挂死只能靠
 * "每条阻塞等待都自带超时"来解决。 */
static uint16_t s_busyFaultSeen = 0U;

bool LoraApp_TakeBusyFault(void)
{
    uint16_t now = LLCC68GetBusyTimeoutCount();

    if (now == s_busyFaultSeen)
    {
        return false;
    }
    s_busyFaultSeen = now;
    return true;
}

static void OnTxDone(void)
{
    g_txDone = true;
}

static void OnTxTimeout(void)
{
    g_txTimeout = true;      /* 中断上下文，只置标志 */
}

static void OnRxTimeout(void)
{
    g_rxTimeout = true;      /* 中断上下文，只置标志 */
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    (void)rssi;
    (void)snr;

    if(size > LORA_RX_BUF_LEN)
    {
        size = LORA_RX_BUF_LEN;
    }
    memcpy(g_rxBuf, payload, size);
    g_rxLen  = size;
    g_rxFlag = true;
}

static RadioEvents_t g_events =
{
    .TxDone    = OnTxDone,
    .RxDone    = OnRxDone,
    .TxTimeout = OnTxTimeout,
    .RxTimeout = OnRxTimeout,
    .RxError   = NULL,
    .CadDone   = NULL,
    .FhssChangeChannel = NULL,
    .GnssDone  = NULL,
    .WifiDone  = NULL
};

/* ==================== LoRa 同步字：显式设置 + 回读 ====================
 * 驱动里的写入口（RadioSetPublicNetwork）只在"公有网络标志变化"时才被调，
 * 而该标志初始化就是 false，所以寄存器 0x0740 一直等于芯片复位默认值（实测 0x1424）。
 * 换批次芯片默认值不一定相同，现场也无法从日志确认 —— 所以这里显式写一次，
 * 并在第一次初始化时把回读值打出来。
 *
 * LORA_USE_PUBLIC_SYNCWORD：1 = 公有 0x3444；0 = 私有 0x1424（当前）。
 * ⚠️ 设备与网关必须用同一个值，差一位就永远收不到，而且两边都不报错。 */
#ifndef LORA_USE_PUBLIC_SYNCWORD
#define LORA_USE_PUBLIC_SYNCWORD    0
#endif

/* 写同步字 + 回读确认（第一次调用时打一行日志） */
static void app_lora_set_syncword(void)
{
    static uint8_t s_printed = 0U;
    uint8_t  rb[2];
    uint16_t sw;

    Radio.SetPublicNetwork(LORA_USE_PUBLIC_SYNCWORD != 0);   /* 内部写 0x0740 */

    LLCC68ReadRegisters(REG_LR_SYNCWORD, rb, 2);             /* 回读 0x0740 */
    sw = (uint16_t)(((uint16_t)rb[0] << 8) | (uint16_t)rb[1]);

    if (s_printed == 0U)
    {
        s_printed = 1U;
        log_info("[lora] syncword: set=%s, readback 0x%04X %s\r\n",
                 (LORA_USE_PUBLIC_SYNCWORD != 0) ? "PUBLIC(0x3444)" : "PRIVATE(0x1424)",
                 (unsigned)sw,
                 (sw == (uint16_t)LORA_MAC_PRIVATE_SYNCWORD) ? "(private)" :
                 ((sw == (uint16_t)LORA_MAC_PUBLIC_SYNCWORD) ? "(public)" : "(NEITHER! check chip)"));
    }
}

/* 打印芯片自报型号 + 驱动实际用的 PA 分支（一次性）。
 * ⚠️ 两者不一定一致，别用版本串去推断分支：本模块自报 "SX1261"，但射频通路接在
 * SX1262/HP PA 那侧。走 SX1261 分支时射频从 RFO_LP 出去、没接到天线，现象是
 * "TX_DONE 正常、GetErrors 全 0、自己收也正常，但空口上对方收不到"。
 * radio.c 的 LLCC68GetDeviceId() 已固定返回 SX1262，这里只如实打出来避免日志误导。 */
static void app_lora_print_chip(void)
{
    static uint8_t s_printed = 0U;
    uint8_t  ver[6];
    uint8_t  is1261;
    uint8_t  pa_id;

    if (s_printed != 0U)
    {
        return;
    }
    s_printed = 1U;

    LLCC68ReadRegisters(0x0320, ver, 6);        /* 版本串：芯片自报 */
    is1261 = ((ver[0] == 'S') && (ver[1] == 'X') && (ver[2] == '1') &&
              (ver[3] == '2') && (ver[4] == '6') && (ver[5] == '1')) ? 1U : 0U;
    pa_id  = LLCC68GetDeviceId();               /* 驱动实际用的分支（可能就是被固定成 SX1262） */

    log_info("[lora] chip 0x0320 = %02X %02X %02X %02X %02X %02X -> self-report %s\r\n",
             (unsigned)ver[0], (unsigned)ver[1], (unsigned)ver[2],
             (unsigned)ver[3], (unsigned)ver[4], (unsigned)ver[5],
             (is1261 != 0U) ? "SX1261" : "SX1262/other");
    log_info("[lora] PA branch used by driver = %s (deviceSel=%u, max %s); 0x08D8 有 TxClamp 补丁 = %s\r\n",
             (pa_id == SX1261) ? "SX1261" : "SX1262",
             (unsigned)((pa_id == SX1261) ? 1U : 0U),
             (pa_id == SX1261) ? "+14dBm" : "+22dBm",
             (((LLCC68ReadRegister(0x08D8)) & 0x1EU) != 0U) ? "yes" : "no");
    log_info("[lora] reg 0x08D8(TxClamp)=0x%02X 0x08E7(OCP)=0x%02X\r\n",
             (unsigned)LLCC68ReadRegister(0x08D8), (unsigned)LLCC68ReadRegister(0x08E7));
}

uint8_t LoraApp_Init(void)
{
    Radio.Init(&g_events);

    /* 初始化本身就撞上 BUSY 超时 = 模组没响应（PB7 没上电 / 模组坏 / SPI 断）。
     * 后面的 SetTxConfig/SetRxConfig/SetChannel 都是在给一颗不在的芯片写寄存器，
     * 写进去也没意义 —— 直接返回错误让上层跳过本轮收发，别把时间全耗在超时上。 */
    if (LoraApp_TakeBusyFault())
    {
        log_info("[lora] !! radio not responding (BUSY timeout) -> init aborted\r\n");
        return 1U;
    }

    /* 发射参数：最后一个参数是发射超时(ms)，不能给 0，
     * 给 0 时板级 1ms 定时器会立刻把还没发完的包打断 */
    Radio.SetTxConfig(MODEM_LORA, LORA_TX_OUTPUT_POWER, 0U, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, false, 0U, LORA_IQ_INVERSION_ON,
                      LORA_ACK_TIMEOUT_MS);

    /* 接收参数：最后一位 false = 单次接收。此时驱动里 RxContinuous=false，
     * Radio.Rx(0) 不是"无限接收"而是一个有限 symbol 超时窗口；
     * LoraApp_ReciveMode() 会临时把 RxContinuous 打开，持续接收才成立。 */
    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0U, LORA_PREAMBLE_LENGTH,
                      LORA_LLCC68_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0U, true, false, 0U, LORA_IQ_INVERSION_ON, false);

    Radio.SetChannel(LORA_FRE);

    /* 放在最后：同步字显式写一次，并且把回读值打出来（见上面说明） */
    app_lora_set_syncword();

    /* 芯片型号/PA 分支/关键寄存器回读（只打一次） */
    app_lora_print_chip();

    return 0U;
}

void LoraApp_ProcessIrq(void)
{
    Radio.IrqProcess();

    if(g_rxForever)
    {
        /* 持续接收模式下：① 芯片自身接收也有上限（约 262s），超时需续上；
         * ② 收到一包后 IrqProcess 会把芯片切回 STDBY，必须重进接收，否则只能收一包。 */
        if(g_rxTimeout || g_rxFlag)
        {
            g_rxTimeout = false;
            Radio.Rx(0U);
        }
    }
}

/* 发射前必须清这两个标志：g_txDone 一旦被置位就再没人清，
 * 不清的话 wait_tx_done() 会在第一轮直接返回 true，紧接着的 Standby/Rx 会把
 * 这包 ACK 掐断在 PA ramp 阶段（现象：收到命令了，但 ACK 发不出去）。
 * 所以 LoraApp_Send() 在 Radio.Send() 前调一次。 */
static void tx_flags_clear(void)
{
    g_txDone    = false;
    g_txTimeout = false;
}

/* 等发送完成：true = 收到 TX_DONE（空口上确实发完了）；false = 超时或芯片放弃发射
 * 时间基准用 TIM3 的 1ms 自由计数（与 LoraApp_Send 里量延时同源）。 */
static bool wait_tx_done(uint32_t ms)
{
    uint32_t t0 = LLCC68GetTimerCount();

    while((uint32_t)(LLCC68GetTimerCount() - t0) < ms)
    {
        Radio.IrqProcess();

        if(g_txDone)
        {
            return true;
        }
        if(g_txTimeout)
        {
            /* 芯片自己放弃了这次发射：把它的 IRQ 状态和当前模式打出来，
             * 省得现场只看到一句"FAILED"没法查（irq 里有 TX_DONE/RX_TX_TIMEOUT 等位） */
            log_info("[lora] !! TX aborted by chip: irq=0x%04X mode=%u\r\n",
                     (unsigned)LLCC68GetIrqStatus(), (unsigned)LLCC68GetOperatingMode());
            return false;       /* 芯片放弃了这次发射: 查供电/天线/射频配置 */
        }
        delay_ms(1U);
    }
    /* 到点还没等到 TX_DONE：同样是"发送失败"，把芯片状态一起打出来 */
    log_info("[lora] !! TX no TX_DONE in %ums: irq=0x%04X mode=%u\r\n",
             (unsigned)ms, (unsigned)LLCC68GetIrqStatus(), (unsigned)LLCC68GetOperatingMode());
    return false;
}

uint8_t LoraApp_Send(const uint8_t *buf, uint8_t len)
{
    if((buf == NULL) || (len == 0U))
    {
        return 1U;
    }
    g_rxForever = false;

    /* 【模式切换等待】发射前留一段空档：刚收完一包时接收链路还在收尾，立刻切 TX
     * 容易把前导码发歪；对方也正好在这段时间从 TX 切回 RX，两边错开就不互相打断。
     * 日志把延时变成可测量的（打印前后 1ms 节拍与实际耗时）。 */
    {
        uint32_t t0 = LLCC68GetTimerCount();
        delay_ms(LORA_TX_MODE_DELAY_MS);
        log_info("[lora] tx settle: wait=%ums actual=%ums (tick %u->%u)\r\n",
                 (unsigned)LORA_TX_MODE_DELAY_MS,
                 (unsigned)(LLCC68GetTimerCount() - t0),
                 (unsigned)t0, (unsigned)LLCC68GetTimerCount());
    }

    Radio.Standby();
    /* 【发前清残留】先处理掉上一包/接收期遗留的 IRQ 状态，否则 DIO1 可能已被残留位拉高，
     * TX_DONE 出不来上升沿 -> 干等超时。顺序必须在 tx_flags_clear() 之前：
     * 这一步可能置上 g_txTimeout(残留超时)，放在后面清才保证等的是这一包的 TX_DONE。 */
    Radio.IrqProcess();
    tx_flags_clear();               /* 必须在 Send 前清, 否则 wait_tx_done 立刻返回假成功 */
    Radio.Send((uint8_t *)buf, len);
    return 0U;
}

bool LoraApp_SendSync(const uint8_t *buf, uint8_t len, uint32_t timeout_ms)
{
    if(LoraApp_Send(buf, len) != 0U)
    {
        return false;
    }
    return wait_tx_done(timeout_ms);    /* 复用文件前面的静态等待函数 */
}

void LoraApp_ReciveMode(void)
{
    /* 【模式切换等待】进接收前留空档：刚发完时 PA 还没完全关断，立刻切 RX 会让接收链路
     * 在前导码期间没稳定，表现为收不到对方紧接着的包。 */
    {
        uint32_t t0 = LLCC68GetTimerCount();
        delay_ms(LORA_RX_MODE_DELAY_MS);
        log_info("[lora] rx settle: wait=%ums actual=%ums\r\n",
                 (unsigned)LORA_RX_MODE_DELAY_MS,
                 (unsigned)(LLCC68GetTimerCount() - t0));
    }

    g_rxForever = true;
    Radio.Rx(0U);                    /* 0 = 一直收(配合 RxContinuous) */
}

void LoraApp_Standby(void)
{
    g_rxForever = false;
    g_rxTimeout = false;
    Radio.Standby();
}

/**
 * @brief 让射频进真正的 Sleep(比 Standby 更深)，进 STOP 前必须调
 * @note  STANDBY_RC 只是待机、内部还在活动；外设域断电后如果域还带电，
 *        只 Standby 的模组会持续耗电。Sleep(WarmStart) 是 LLCC68 最深的低功耗态，
 *        配置保留；醒来 LoraApp_Init() 本来就会完整重配，所以放睡是安全的。
 */
void LoraApp_Sleep(void)
{
    g_rxForever = false;
    g_rxTimeout = false;
    Radio.Sleep();
}

/* 同步发完一包后立刻重回持续接收 —— 专供"接收窗口内回完 ACK 还要继续收"用。
 * 不走 LoraApp_ReciveMode()：那里有 100ms 切换空档，而这里 SendSync 已确认 TX_DONE，
 * 再空等只会白白吃掉接收窗口。 */
void LoraApp_ResumeRx(void)
{
    g_rxForever = true;
    g_rxTimeout = false;
    Radio.Rx(0U);
}

uint16_t LoraApp_GetRx(uint8_t *buf, uint16_t maxLen)
{
    uint16_t len;

    if((buf == NULL) || (g_rxFlag == false))
    {
        return 0U;
    }
    g_rxFlag = false;

    len = g_rxLen;
    if(len > maxLen)
    {
        len = maxLen;
    }
    memcpy(buf, g_rxBuf, len);
    return len;
}
