/**
 * @file app_lora.c
 * @brief LoRa(LLCC68) 收发实现（精简版）
 *
 * 只做两件事：
 *   发送：Radio.Send() -> 等 TxDone -> 开接收窗口等ACK -> 超时没ACK就重发
 *   接收：DIO1 中断只置标志，主循环 LoraApp_ProcessIrq() 里取包
 *
 * 注意：组帧/打包不在本文件，业务层把整包数据传进来即可。
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
 * 【原来的坑】整个工程【没有任何地方】写过同步字寄存器 0x0740：
 *   - RadioSetPublicNetwork() 是唯一的写入口，但它只在 RadioSetModem() 发现
 *     RadioPublicNetwork.Current != Previous 时才被调；而这两者初始化都是 false
 *     (radio.c 的 static RadioPublicNetwork = { false })，所以【永远不执行】；
 *   - LLCC68SetSyncWord() 写的是 GFSK 那 8 字节(0x06C0)，LoRa 分支从不调用它。
 * 结果：同步字一直等于【芯片复位默认值】。实测(网关 #lora 回读 0x0740)默认是
 *       0x1424(私有网络)。靠默认值有两个隐患：
 *   ① 换批次/换芯片，默认值不一定还是 0x1424；
 *   ② 现场无法从日志确认到底是多少，只能"推断"。
 * 现在在这里显式写一次，并在第一次初始化时把【回读值】打出来。
 *
 * 【怎么改】把 LORA_USE_PUBLIC_SYNCWORD 改 1 = 公有 0x3444；0 = 私有 0x1424。
 * ⚠️ 两边(液位节点 / 网关)必须用同一个值，差一位就永远收不到，而且两边都不报错。 */
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
 * ⚠️ 2026-10-10 更正：这两个【不一定一致】，别再用版本串去推断分支。
 *   实测：本模块自报 "SX1261"，但模块的射频通路接在【SX1262 / HP PA】那一侧。
 *   按版本串走 SX1261 分支(deviceSel=1)时，射频从 RFO_LP 引脚出去、根本没接到天线上，
 *   网关侧只收到 −110dBm 量级的泄漏（1 米距离！），表现就是"TX_DONE 正常、GetErrors 全 0、
 *   自己接收也正常，但空口上对方收不到"。
 *   radio.c 的 LLCC68GetDeviceId() 已固定返回 SX1262（走 deviceSel=0 / HP PA），
 *   改完链路立刻正常 —— 所以这里改成直接问驱动"你实际会走哪个分支"，避免日志误导。 */
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

    /* 发射参数：最后一个参数是发射超时(ms)，不能给 0，
     * 给 0 时板级 1ms 定时器会立刻把还没发完的包打断 */
    Radio.SetTxConfig(MODEM_LORA, LORA_TX_OUTPUT_POWER, 0U, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, false, 0U, LORA_IQ_INVERSION_ON,
                      LORA_ACK_TIMEOUT_MS);

    /* 接收参数：最后一个参数 false = 单次接收(配合 Radio.Rx(等待时间) 用)
     * 注意: 这里 false 时驱动里 RxContinuous=false, 之后 Radio.Rx(0) 走的也不是
     * "无限接收", 而是一个有限 symbol 超时窗口。LoraApp_ReciveMode() 会临时把
     * RxContinuous 打开成 true, 所以持续接收是好的。 */
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
        /* 持续接收模式下:
         *   1) 芯片自身的接收也有上限(约262秒)，超时需续上；
         *   2) 收到一包后，当前 RxContinuous=false 时 IrqProcess
         *      会把芯片切回 STDBY，必须重新进入接收，否则只能收一包。 */
        if(g_rxTimeout || g_rxFlag)
        {
            g_rxTimeout = false;
            Radio.Rx(0U);
        }
    }
}

/* 发射前必须先清这两个标志。
 * 【为什么必须清】老实现靠 g_txDone 判断"发完了", 但这个标志在发送前从没被清过:
 *   设备第一次发上报时 g_txDone 被置 true, 之后【永远保持 true】。
 *   于是回 ACK 时 wait_tx_done() 第一轮就 return true ——
 *   根本没等发射完成, 紧接着的 Radio.Standby()/Radio.Rx() 就把这包 ACK
 *   掐断在 PA ramp 阶段, 空口上一个字节都没出去。
 *   现象正是"液位收到命令了, 但回应的 ACK 发不出来、网关等不到"。
 * 所以下面的 LoraApp_Send() / LoraApp_SendWaitAck() 都在 Radio.Send() 前调一次。 */
static void tx_flags_clear(void)
{
    g_txDone    = false;
    g_txTimeout = false;
}

/* 等发送完成: true=收到 TX_DONE(空口上确实发完了); false=超时或芯片放弃发射
 *
 * 时间基准用 TIM3 的 1ms 自由计数(LLCC68GetTimerCount), 和 LoraApp_Send 里
 * 量延时用的是同一个源; 不用 LLCC68DelayMs(1) 计数 —— 延时不准时超时会跟着走样。 */
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

/* 开接收窗口等ACK：返回 true=窗口内收到包 */
static bool wait_ack(uint32_t ms)
{
    g_rxFlag    = false;
    g_rxTimeout = false;

    Radio.Rx(ms);

    while((g_rxFlag == false) && (g_rxTimeout == false))
    {
        Radio.IrqProcess();
        LLCC68DelayMs(1U);
    }

    g_rxTimeout = false;
    Radio.Standby();                 /* 关接收，回到待机 */
    return g_rxFlag;
}

bool LoraApp_SendWaitAck(const uint8_t *buf, uint8_t len, uint8_t retry)
{
    uint8_t i;

    if((buf == NULL) || (len == 0U))
    {
        return false;
    }

    for(i = 0U; i <= retry; i++)
    {
        g_txDone    = false;
        g_txTimeout = false;
        g_rxFlag    = false;
        g_rxTimeout = false;
        g_rxForever = false;

        /* 【模式切换等待】这条路径自己发, 不走 LoraApp_Send(), 所以这里也要留空档 */
        delay_ms(LORA_TX_MODE_DELAY_MS);

        tx_flags_clear();            /* 必须先清, 否则 wait_tx_done 被上一包的残旗骗过 */
        Radio.Standby();             /* SetTx 只能在待机/FS 下下发 */
        Radio.IrqProcess();          /* 同上：发前把残留 IRQ 处理掉，别让 DIO1 卡在高电平 */
        tx_flags_clear();            /* 上面那次可能置上 g_txTimeout(残留超时)，再清一次 */
        Radio.Send((uint8_t *)buf, len);

        if(wait_tx_done(LORA_ACK_TIMEOUT_MS) && wait_ack(LORA_ACK_TIMEOUT_MS))
        {
            return true;             /* 发完并且收到ACK */
        }

        log_info("[lora] no ack, retry %u\r\n", (unsigned)(i + 1U));
    }
    return false;
}

uint8_t LoraApp_Send(const uint8_t *buf, uint8_t len)
{
    if((buf == NULL) || (len == 0U))
    {
        return 1U;
    }
    g_rxForever = false;

    /* 【模式切换等待】发射前留一段空档:
     * 刚收完一包时接收链路还在上电/正在收尾, 立刻切 TX 容易把前导码发歪;
     * 对方(网关)也正好在这段时间内从 TX 切回 RX, 两边错开就不会互相打断。
     *
     * 下面这行日志把延时"变成可测量的": 打印延时前后的 1ms 节拍和实际耗时,
     * 这样在串口上就能看出到底等了多久(日志没时间戳, 光看两行分不出 1ms 还是 1s)。 */
    {
        uint32_t t0 = LLCC68GetTimerCount();
        delay_ms(LORA_TX_MODE_DELAY_MS);
        log_info("[lora] tx settle: wait=%ums actual=%ums (tick %u->%u)\r\n",
                 (unsigned)LORA_TX_MODE_DELAY_MS,
                 (unsigned)(LLCC68GetTimerCount() - t0),
                 (unsigned)t0, (unsigned)LLCC68GetTimerCount());
    }

    Radio.Standby();
    /* 【发前清残留】先把上一包/接收期遗留的 IRQ 状态处理掉（配合 radio.c 里
     * RadioIrqProcess() 改成"直接问芯片"）。否则 DIO1 可能已被残留位拉高，
     * TX_DONE 就出不来上升沿 -> wait_tx_done() 干等超时 = "发送失败、对端收不到"。
     * 顺序必须在 tx_flags_clear() 之前：这一步可能置上 g_txTimeout(残留超时)，
     * 放在后面清，才能真正保证"等的是这一包的 TX_DONE"。 */
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
    /* 【模式切换等待】进接收前留空档: 刚发完(TX)时 PA 还没完全关断,
     * 立刻切 RX 会让接收链路在前导码期间还没稳定, 表现为收不到对方紧接着的包。
     * 同 LoraApp_Send 一样, 把延时打成可测量的日志。 */
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
 * @brief 让射频进真正的 Sleep(比 Standby 更深)，进STOP前必须调
 *
 * 为什么不能只 Standby:
 *   STANDBY_RC 只是"待机"，芯片内部还在活动；而 PA7 高阻之后，外设电源域到底
 *   断没断电取决于板上的偏置电阻 —— 如果域还带电，只 Standby 的模组会一直耗电。
 *   Sleep(WarmStart=1) 是 LLCC68 最深的低功耗态(≈1µA 级)，且配置保留，
 *   醒来后 LoraApp_Init() 本来就会 Radio.Init() 完整重配，所以放睡是安全的。
 */
void LoraApp_Sleep(void)
{
    g_rxForever = false;
    g_rxTimeout = false;
    Radio.Sleep();
}

/* 同步发完一包之后, 立刻重回持续接收 —— 专供"接收窗口内回完 ACK 还要继续收"用。
 *
 * 为什么不能直接用 LoraApp_ReciveMode():
 *   那个函数开头有 delay_ms(LORA_RX_MODE_DELAY_MS=500ms) 的空档, 那是给
 *   "刚发完就进接收(异步发射还没收尾)"准备的。而这里前提是 LoraApp_SendSync()
 *   已经等到了 TX_DONE(发射真的结束了), 再空等 500ms 只会白白吃掉接收窗口。 */
void LoraApp_ResumeRx(void)
{
    g_rxForever = true;
    g_rxTimeout = false;
    Radio.Rx(0U);
}

/* 兼容旧调用（app_sensor.c 在用）：只发送、不等ACK */
uint8_t LoraApp_SendAsync(uint8_t *buf, uint8_t len, uint32_t msTimeout)
{
    (void)msTimeout;
    return LoraApp_Send(buf, len);
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

/* ============================ 测试用 ============================ */
#if LORA_ENABLE_TEST

/* 测试发送：数组里直接写16进制数据 */
void LoraApp_TestSend(void)
{
    // static const uint8_t testData[] =
    // {
    //     0xAA, 0x55, 0x01, 0x00, 0x00, 0x00, 0x21, 0x00,
    //     0x01, 0x03, 0x0F, 0x01, 0x01, 0x05, 0x01, 0x08,
    //     0x03, 0x00, 0x07, 0xCB, 0xAE, 0xC9, 0xEE, 0x32,
    //     0x30, 0x6D, 0x2C, 0xFA
    // };

    static const uint8_t testData[] =
    {
        0xAA, 0x55, 0x01, 0x11, 0x00, 0x01, 0x21, 0x00,
    0x01, 0x80, 0x00, 0xB0, 0x49
    };

    LoraApp_Init();
    log_info("[test] send mode, %u bytes\r\n", (unsigned)sizeof(testData));

    while(1)
    {
        if(LoraApp_SendWaitAck(testData, (uint8_t)sizeof(testData), LORA_SEND_RETRY))
        {
            log_info("[test] ack ok\r\n");
        }
        else
        {
            log_info("[test] no ack\r\n");
        }
        LLCC68DelayMs(3000U);
    }
}

/* 测试接收：收到就打印，16进制，一个字节一行 */
void LoraApp_TestRecive(void)
{
    uint8_t  buf[LORA_RX_BUF_LEN];
    uint16_t len;
    uint16_t i;

    LoraApp_Init();
    log_info("[test] recive mode\r\n");
    LoraApp_ReciveMode();

    while(1)
    {
        LoraApp_ProcessIrq();

        len = LoraApp_GetRx(buf, sizeof(buf));
        for(i = 0U; i < len; i++)
        {
            log_info("%02X\r\n", buf[i]);
        }

        LLCC68DelayMs(1U);
    }
}
/* ==================== 发射诊断（临时用，定位"对方收不到"） ====================
 * 为什么需要它：
 *   RX 正常已经说明 SPI/DIO1中断/频率/带宽/SF/同步字/CRC 都是对的，
 *   所以问题只可能是下面两种之一：
 *   (a) 芯片把这次发射中止了 —— SX126x/LLCC68 在 PA ramp / PLL / XOSC 失败时
 *       *不会产生任何中断*，现象就是 IrqProcess 读回来 irq=0x0000、
 *       芯片模式从 6(TX) 自己回到 2(STDBY)。官方驱动里的 TxTimeout 就是这么被触发的。
 *   (b) 芯片真的发完了 —— 那问题在天线/射频开关/功率/供电（对方收不到）。
 * 本函数不靠 DIO1 中断，每 10ms 直接问芯片一次，把结果打出来。
 *
 * 判读（err 用原始两字节，按 LLCC68 数据手册 GetErrors(0x17) 的位定义）：
 *   irq=0x0001(TX_DONE)             -> 发完了 => 查射频/天线/供电/功率
 *   irq=0x0000                      -> 芯片中止了 => 看 err 哪一位是 1：
 *       err[0] bit5 XOSC起振失败 / bit6 PLL未锁定 / bit4 镜像校准 / bit3 ADC校准
 *       err[1] bit0 PA ramp 失败（+20dBm 发射瞬间约 100mA，供电/匹配扛不住会报这个）
 *   注意：驱动 LLCC68GetDeviceErrors() 里的字段映射把这两个字节弄反了，
 *         所以这里看原始字节，不要看 error.Fields.PaRamp。
 *   irq 有 TX_DONE 但 flags 里 txDone=0 -> DIO1/EXTI 通路问题（中断没进来）
 * ============================================================================ */
static void txdiag_read_errors(uint8_t *raw)
{
    /* 直接发命令 0x17(GetErrors)：opcode + 状态字节 + 2 个数据字节 */
    LLCC68WaitOnBusy();
    LLCC68SetNss(0U);
    (void)LLCC68SpiInOut(0x17U);
    (void)LLCC68SpiInOut(0x00U);
    raw[0] = LLCC68SpiInOut(0x00U);     /* MSB：校准/PLL/XOSC */
    raw[1] = LLCC68SpiInOut(0x00U);     /* LSB：bit0 = PA ramp */
    LLCC68SetNss(1U);
    LLCC68WaitOnBusy();
}

/* 手写一条 SPI 命令并抓芯片返回的状态字节：
 *   st1 = 发 opcode 期间 MISO 上的字节
 *   st2 = 紧随其后的那个字节（本驱动 ReadCommand 就是取这一字节当 status）
 * 状态字节: bit6:4 = 芯片模式, bit3:1 = 命令状态(2≈OK, 3=超时, 4/5=处理/执行失败)
 */
static uint8_t txdiag_cmd_status(uint8_t cmd, const uint8_t *data, uint8_t len, uint8_t *st2)
{
    uint8_t i;
    uint8_t st1;

    LLCC68WaitOnBusy();
    LLCC68SetNss(0U);
    st1  = LLCC68SpiInOut(cmd);
    *st2 = LLCC68SpiInOut(0x00U);
    for (i = 0U; i < len; i++)
    {
        (void)LLCC68SpiInOut(data[i]);
    }
    LLCC68SetNss(1U);
    LLCC68WaitOnBusy();
    return st1;
}

/* 诊断用测试帧（和官方 demo 一致） */
static const uint8_t txdiag_testData[] =
{
    0xAA, 0x55, 0x01, 0x00, 0x00, 0x00, 0x21, 0x00,
    0x01, 0x03, 0x0F, 0x01, 0x01, 0x05, 0x01, 0x08,
    0x03, 0x00, 0x07, 0xCB, 0xAE, 0xC9, 0xEE, 0x32,
    0x30, 0x6D, 0x2C, 0xFA
};

/* 按需要重配发射参数(带宽索引/前导码/头模式) */
static void txdiag_cfg(uint32_t bw, uint16_t pre, bool fixLen)
{
    Radio.SetTxConfig(MODEM_LORA, LORA_TX_OUTPUT_POWER, 0U, bw,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      pre, fixLen,
                      true, false, 0U, LORA_IQ_INVERSION_ON,
                      LORA_ACK_TIMEOUT_MS);
}

/* 发一包并等 TX_DONE，返回真实耗时(ms，按TIM3的1ms节拍)；超时返回0xFFFF */
static uint16_t txdiag_send_ms(uint8_t len)
{
    uint16_t i;
    uint32_t t0;
    uint32_t t1;

    t0 = LLCC68GetTimerCount();
    Radio.Send((uint8_t *)txdiag_testData, len);
    for (i = 0U; i < 400U; i++)
    {
        delay_ms(1U);
        if ((LLCC68GetIrqStatus() & IRQ_TX_DONE) != 0U)
        {
            break;
        }
    }
    t1 = LLCC68GetTimerCount();
    LLCC68ClearIrqStatus(0xFFFFU);

    if (i >= 400U)
    {
        return 0xFFFFU;         /* 没等到 TX_DONE */
    }
    return (uint16_t)(t1 - t0);
}

/* 手写整条发射的对比函数已删除：那版探针在写命令后多读了一个状态字节，
 * 会把 SPI 流错位，数据不可信。现在只走驱动路径 + 打印真实字节。 */

void LoraApp_TestTxDiag(void)
{
    uint8_t       err[2];
    uint8_t       sendLen;
    uint16_t      ms;
    uint32_t      round = 0U;
    RCC_ClocksType clk;

    LoraApp_Init();

    /* 时钟到底是多少：SystemCoreClock(驱动延时/TIM3用) vs 读寄存器得到的真实值 */
    RCC_GetClocksFreqValue(&clk);
    log_info("[txdiag] clk Core=%u rccSys=%u Hclk=%u Pclk1=%u Pclk2=%u\r\n",
             (unsigned)SystemCoreClock, (unsigned)clk.SysclkFreq,
             (unsigned)clk.HclkFreq, (unsigned)clk.Pclk1Freq, (unsigned)clk.Pclk2Freq);
    log_info("[txdiag] reg OCP=0x%02X 08D8=0x%02X 0889=0x%02X 0702=%02X\r\n",
             (unsigned)Radio.Read(REG_OCP),
             (unsigned)Radio.Read(0x08D8U),
             (unsigned)Radio.Read(0x0889U),
             (unsigned)Radio.Read(0x0702U));

    while (1)
    {
        /* 每轮先恢复统一配置(SF8/500k/前导10/显式头) */
        txdiag_cfg(2U, 10U, false);

        /* ---- 打印"驱动这次会发出去的字节" + 芯片自述 ---- */
        {
            uint8_t st1;
            uint8_t st2;
            uint8_t ver[8];
            uint8_t k;
            uint8_t d[6];

            for (k = 0U; k < 8U; k++)
            {
                ver[k] = Radio.Read((uint32_t)(0x0320U + k));       /* 版本字符串 */
            }
            /* 复现驱动 LLCC68SetPacketParams 里 buf[] 的构造方式 */
            d[0] = (uint8_t)((LLCC68.PacketParams.Params.LoRa.PreambleLength >> 8) & 0xFFU);
            d[1] = (uint8_t)(LLCC68.PacketParams.Params.LoRa.PreambleLength);
            d[2] = (uint8_t)LLCC68.PacketParams.Params.LoRa.HeaderType;
            d[3] = 28U;
            d[4] = (uint8_t)LLCC68.PacketParams.Params.LoRa.CrcMode;
            d[5] = (uint8_t)LLCC68.PacketParams.Params.LoRa.InvertIQ;
            log_info("[probe] ver=%02X%02X%02X%02X%02X%02X driverBuf=8C %02X %02X %02X %02X %02X %02X\r\n",
                     (unsigned)ver[0], (unsigned)ver[1], (unsigned)ver[2],
                     (unsigned)ver[3], (unsigned)ver[4], (unsigned)ver[5],
                     (unsigned)d[0], (unsigned)d[1], (unsigned)d[2],
                     (unsigned)d[3], (unsigned)d[4], (unsigned)d[5]);

            st1 = txdiag_cmd_status(0xC0U, NULL, 0U, &st2);         /* GetStatus */
            log_info("[probe] status=0x%02X mode=%u\r\n",
                     (unsigned)st1, (unsigned)((st1 & 0x70U) >> 4));
        }

        /* 预热：配置后第一发会哑(初始化后第一发同理)，先空发一次不计时 */
        g_txDone    = false;
        g_txTimeout = false;
        LoraApp_Standby();
        LLCC68ClearIrqStatus(0xFFFFU);
        LLCC68ClearDeviceErrors();
        (void)txdiag_send_ms(28U);

        /* 正式计时这一发：长度交替 28 / 5，用来判断驱动写的包长芯片有没有采纳 */
        sendLen = ((round & 1U) == 0U) ? 28U : 5U;
        LoraApp_Standby();
        LLCC68ClearIrqStatus(0xFFFFU);
        LLCC68ClearDeviceErrors();
        ms = txdiag_send_ms(sendLen);
        txdiag_read_errors(err);

        log_info("[txdiag] #%u len=%u -> airMs=%u model=%ums (driverPL=%u err=%02X%02X 08D8=0x%02X OCP=0x%02X)\r\n",
                 (unsigned)round, (unsigned)sendLen, (unsigned)ms,
                 (unsigned)Radio.TimeOnAir(MODEM_LORA, 2U, LORA_SPREADING_FACTOR,
                                           LORA_CODINGRATE, 10U,
                                           LORA_FIX_LENGTH_PAYLOAD_ON, sendLen, true),
                 (unsigned)LLCC68.PacketParams.Params.LoRa.PayloadLength,
                 (unsigned)err[0], (unsigned)err[1],
                 (unsigned)Radio.Read(0x08D8U),
                 (unsigned)Radio.Read(REG_OCP));

        round++;
        LLCC68DelayMs(1500U);
    }
}
#endif /* LORA_ENABLE_TEST */
