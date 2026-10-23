/*!
 * \file      LLCC68N32G031-board.c
 *
 * \brief     Target board LLCC68 driver implementation for N32G031 (Nations)
 *
 * 按照《LLCC68驱动demo使用说明.pdf》第2章"驱动移植"要求，参考 N32H482 版
 * LLCC68-board.c 移植到 N32G031 标准外设库，实现 GPIO、外部中断、SPI 和定时器。
 * 引脚定义见 project_config.h（那是唯一正确的引脚配置来源）。
 *
 * 资源占用（N32G031，SYSCLK=8MHz HSI）:
 *   - SPI1(APB2): PA0 SCK / PA4 MISO / PA5 MOSI, NSS=PA1 软件控制
 *                 （复用编号：PA0=AF0，PA4/PA5=AF5，见 LLCC68IoInit）
 *   - EXTI      : PC15 DIO1 上升沿中断，PC15 属于 EXTI4_15 共享中断向量
 *   - TIM3(APB1): 更新中断周期 = **1 ms**（见 LLCC68TimerInit）
 *                 驱动 LoRa 收发超时软定时器（TIM3 计数时钟 = PCLK1 = 8MHz）
 *   - SysTick   : LLCC68DelayMs 阻塞式延时（轮询 COUNTFLAG，不开中断），节拍也是 **1 ms**
 *
 *   【时基：别再被"6 倍"带偏】
 *     SystemCoreClock 是【编译期常数】= SYSCLK_FREQ（Keil 工程 Define 写的 8000000），
 *     不是运行时算的（SystemClock_Config() 里没有调 SystemCoreClockUpdate()）。
 *     于是：SysTick LOAD = 8e6/1000-1 = 7999，按 HCLK=8MHz 计数 -> 正好 1 ms；
 *           TIM3 Prescaler = 8e6/10000-1 = 799 -> 10kHz，Period=10 -> 正好 1 ms。
 *     实测：0x04 配对帧的空口间隔是 4.96 s（配置 5 s），证明节拍就是 1 ms。
 *     ⚠️ 只有换到"没定义 SYSCLK_FREQ 的工程"（例如本目录 EWARM 工程没写 Define）时，
 *        SystemCoreClock 才会落到默认 48e6，那时这两个定时器才会变成 6 ms/格。
 *   - 未用引脚  : PC4 TXEN / PC3 DIO2 / PC2 DIO3 / PC1 RXEN，配为浮空输入
 *   - EN 脚     : 厂商 demo 板写法是 PA7；本板不用 —— 本板 LoRa(含液位模块)供电走
 *                 PB7 外设总电源使能（低=上电，由 bsp_gpio.c 驱动），
 *                 PA7 是探针激励脚，本文件都不驱动
 *   - PA6 nRST  : 复位脚，复位时推挽输出拉低，之后切为输入上拉
 *
 * 说明: SPI1 在 N32G031 的复用编号：PA0(SCK)=AF0，PA4(MISO)/PA5(MOSI)=AF5
 */
#include <stdio.h>
#include "misc.h"
#include "n32g031_gpio.h"
#include "n32g031_rcc.h"
#include "n32g031_spi.h"
#include "n32g031_exti.h"
#include "n32g031_tim.h"
#include "project_config.h"
#include "radio.h"
#include "LLCC68-board.h"
#include "log.h"

/*------------------------- 软定时器 -------------------------*/
typedef struct __softTimer_s
{
    volatile uint8_t  isRun;    /* 0停止,1运行              */
    volatile uint32_t delayMs;  /* 定时长度 ms              */
    volatile uint32_t startMs;  /* 定时开始的计数值         */
} SoftTimer_t;

static DioIrqHandler *dio1IrqCallback = NULL; /* 记录DIO1的回调(RadioOnDioIrq) */
static volatile uint32_t timer_count = 0;     /* TIM3 更新中断计数器(1ms 加一次，见 LLCC68TimerInit) */
static SoftTimer_t txTimerHandle, rxTimerHandle;

/*------------------------- GPIO 初始化 -------------------------*/
/* 初始化LLCC68需要的GPIO: NSS为推挽输出(软件控制), BUSY为输入,
 * SCK(PA0)=AF0、MOSI(PA5)/MISO(PA4)=AF5 复用, 其余未用模组引脚配置为浮空输入
 */
void LLCC68IoInit( void )
{
    GPIO_InitType GPIO_InitStructure;

    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, ENABLE);
    RCC_ConfigLse(RCC_LSE_DISABLE, 0);

    /* N32G031 的 GPIO 挂 APB2 */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOC |
                            RCC_APB2_PERIPH_AFIO, ENABLE);

    /* NSS(PA1): 推挽输出, 空闲保持高电平 */
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin            = RADIO_NSS_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Pull      = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStructure.GPIO_Current   = GPIO_DC_HIGH;
    GPIO_InitStructure.GPIO_Alternate = GPIO_NO_AF;
    GPIO_InitPeripheral(RADIO_NSS_PORT, &GPIO_InitStructure);
    GPIO_SetBits(RADIO_NSS_PORT, RADIO_NSS_PIN);

    /* BUSY(PC14): 上拉输入（模组忙时输出高） */
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin            = RADIO_DIO4_BUSY_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_INPUT;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Speed     = GPIO_SPEED_LOW;
    GPIO_InitStructure.GPIO_Current   = GPIO_DC_LOW;
    GPIO_InitStructure.GPIO_Alternate = GPIO_NO_AF;
    GPIO_InitPeripheral(RADIO_DIO4_BUSY_PORT, &GPIO_InitStructure);

    /* SCK(PA0): N32G031 数据手册 PA0 SPI1_SCK=AF0, 必须单独配置 */
    GPIO_InitStructure.Pin            = RADIO_SCK_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Pull      = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF0_SPI1; /* PA0-SCK=AF0 */
    GPIO_InitPeripheral(RADIO_SCK_PORT, &GPIO_InitStructure);

    /* MOSI(PA5): N32G031 PA5 SPI1_MOSI=AF5 */
    GPIO_InitStructure.Pin            = RADIO_MOSI_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Pull      = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF5_SPI1; /* PA5-MOSI=AF5 */
    GPIO_InitPeripheral(RADIO_MOSI_PORT, &GPIO_InitStructure);

    /* MISO(PA4): 主模式下必须为输入, PA4 SPI1_MISO=AF5 */
    GPIO_InitStructure.Pin            = RADIO_MISO_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Pull      = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF5_SPI1; /* PA4-MISO=AF5 */
    GPIO_InitPeripheral(RADIO_MISO_PORT, &GPIO_InitStructure);

    /* 模组未使用的引脚(TXEN=PC4/DIO2=PC3/DIO3=PC2/RXEN=PC1): 浮空输入 */
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin            = RADIO_DIO0_TXEN_PIN | RADIO_DIO2_PIN |
                                        RADIO_DIO3_PIN | RADIO_DIO5_RXEN_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_INPUT;
    GPIO_InitStructure.GPIO_Pull      = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Speed     = GPIO_SPEED_LOW;
    GPIO_InitStructure.GPIO_Current   = GPIO_DC_LOW;
    GPIO_InitStructure.GPIO_Alternate = GPIO_NO_AF;
    GPIO_InitPeripheral(RADIO_DIO0_TXEN_PORT, &GPIO_InitStructure);

    /* 使能SPI1时钟并初始化: 主模式/8bit/CPOL=0/CPHA=0/软NSS/MSB
     * SPI1挂APB2, 8MHz 主频下 4 分频 = 2MHz, 远低于 LLCC68 上限 16MHz */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_SPI1, ENABLE);
    {
        SPI_InitType SPI_InitStructure;

        SPI_InitStruct(&SPI_InitStructure);
        SPI_InitStructure.DataDirection = SPI_DIR_DOUBLELINE_FULLDUPLEX;
        SPI_InitStructure.SpiMode       = SPI_MODE_MASTER;
        SPI_InitStructure.DataLen       = SPI_DATA_SIZE_8BITS;
        SPI_InitStructure.CLKPOL        = SPI_CLKPOL_LOW;
        SPI_InitStructure.CLKPHA        = SPI_CLKPHA_FIRST_EDGE;
        SPI_InitStructure.NSS           = SPI_NSS_SOFT;
        SPI_InitStructure.BaudRatePres  = SPI_BR_PRESCALER_4;
        SPI_InitStructure.FirstBit      = SPI_FB_MSB;
        SPI_InitStructure.CRCPoly       = 7;
        SPI_Init(SPI1, &SPI_InitStructure);
        SPI_Enable(SPI1, ENABLE);
    }

    log_info("LLCC68IoInit init sucess\r\n");
}

/* 初始化 DIO1
 * DIO1 物理连接在 PC15 (见 project_config.h: RADIO_DIO1_PIN=GPIO_PIN_15),
 * 配置为外部中断 EXTI Line15 (上升沿触发), 回调为 dioIrq
 * (即 radio.c 中的 RadioOnDioIrq, 仅置位 IrqFired)
 * 注: N32G031 的 EXTI4~EXTI15 共用一个中断向量 EXTI4_15_IRQn
 */
void LLCC68IoIrqInit( DioIrqHandler dioIrq )
{
    GPIO_InitType  GPIO_InitStructure;
    EXTI_InitType  EXTI_InitStructure;
    NVIC_InitType  NVIC_InitStructure;

    dio1IrqCallback = dioIrq;

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOC | RCC_APB2_PERIPH_AFIO, ENABLE);

    /* DIO1(PC15): 下拉输入（模组空闲输出低） */
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin            = RADIO_DIO1_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_INPUT;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_DOWN;
    GPIO_InitStructure.GPIO_Speed     = GPIO_SPEED_LOW;
    GPIO_InitStructure.GPIO_Current   = GPIO_DC_LOW;
    GPIO_InitStructure.GPIO_Alternate = GPIO_NO_AF;
    GPIO_InitPeripheral(RADIO_DIO1_PORT, &GPIO_InitStructure);

    /* PC15 -> EXTI Line15（N32G031 库为2参数版本） */
    GPIO_ConfigEXTILine(GPIOC_PORT_SOURCE, GPIO_PIN_SOURCE15);

    EXTI_InitStructure.EXTI_Line    = EXTI_LINE15;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);

    /* N32G031 的 NVIC 只有 3 个成员：通道/优先级(0最高,3最低)/使能 */
    NVIC_InitStructure.NVIC_IRQChannel         = EXTI4_15_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 2U;
    NVIC_InitStructure.NVIC_IRQChannelCmd      = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    EXTI_ClrITPendBit(EXTI_LINE15);
}

/* DIO1(PC15) 外部中断服务函数 —— 检查 Line15 */
void EXTI4_15_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_LINE15) != RESET)
    {
        if(GPIO_ReadInputDataBit(RADIO_DIO1_PORT, RADIO_DIO1_PIN) != 0)
        {
            if(NULL != dio1IrqCallback)
            {
                dio1IrqCallback(NULL);
            }
        }
        EXTI_ClrITPendBit(EXTI_LINE15);
    }
}

void LLCC68IoDeInit( void )
{
    /* GPIO去初始化(暂不需要) */
}

void LLCC68IoDbgInit( void )
{
    /* 调试引脚(暂不需要) */
}

/* 复位引脚初始化: 拉低RST至少20ms再释放为输入上拉 */
void LLCC68ResetInit( void )
{
    GPIO_InitType GPIO_InitStructure;

    LLCC68DelayMs( 10 );

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);

    /* RST(PA6): 推挽输出并拉低复位 */
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin            = RADIO_nRESET_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Pull      = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStructure.GPIO_Current   = GPIO_DC_HIGH;
    GPIO_InitStructure.GPIO_Alternate = GPIO_NO_AF;
    GPIO_InitPeripheral(RADIO_nRESET_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(RADIO_nRESET_PORT, RADIO_nRESET_PIN);

    LLCC68DelayMs( 20 );

    /* 释放RST: 切为输入上拉, 由模组内部上拉为高 */
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin            = RADIO_nRESET_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_INPUT;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Speed     = GPIO_SPEED_LOW;
    GPIO_InitStructure.GPIO_Current   = GPIO_DC_LOW;
    GPIO_InitStructure.GPIO_Alternate = GPIO_NO_AF;
    GPIO_InitPeripheral(RADIO_nRESET_PORT, &GPIO_InitStructure);

    LLCC68DelayMs( 10 );

}

/* 等 BUSY 拉低，【硬超时】LLCC68_BUSY_TIMEOUT_MS 后放弃并累计一次超时。
 *
 * 【为什么必须有超时】(2026-10-22)
 *   旧实现是 while(BUSY==1){ ...; LLCC68DelayMs(1); } 的纯死循环，注释却写"超过 50ms
 *   硬复位射频并置超时"—— 复位和置标志都没实现。后果：模组没供电(PB7 配错/外设域没上电)、
 *   模组坏、SPI 断线时，任何一个 SPI 命令(读状态、发命令、读寄存器)都会永久卡在这里，
 *   设备既不睡也不报，串口也没输出，现场只能拔电池。
 *   本工程【不能靠看门狗兜底】：IWDG 用的是 LSI，STOP 期间照样计数，10 分钟睡一觉必然
 *   被它复位；N32G031 也没有"IWDG 在 STOP 冻结"的选项位（OB 里只有 nRST_STOP）。
 *   所以超时只能在这里做，把"卡死"降级成"这一次射频操作失败"，由上层跳过本轮收发。
 *
 * 计数用 LLCC68GetBusyTimeoutCount() 取，上层(app_lora)据此判断"射频还在不在"。
 * 注意：这里【不打印日志】—— 模组真没响应时每个 SPI 事务都会走到这，打印会刷屏，
 *       而且日志走 USART 阻塞发送，会把本轮时序拖得更乱。 */
#define LLCC68_BUSY_TIMEOUT_MS   50U

static volatile uint16_t s_busyTimeoutCnt = 0U;

void LLCC68WaitOnBusy( void )
{
    uint32_t waitedMs = 0U;

    while( GPIO_ReadInputDataBit(RADIO_DIO4_BUSY_PORT, RADIO_DIO4_BUSY_PIN) == 1 )
    {
        if(waitedMs >= LLCC68_BUSY_TIMEOUT_MS)
        {
            s_busyTimeoutCnt++;         /* 放弃本次等待：射频没响应，交给上层处理 */
            return;
        }
        LLCC68DelayMs(1);
        waitedMs++;
    }
}

/* BUSY 超时累计次数（只增不减）：上层用"自上次检查后有没有变"判断射频是否还活着 */
uint16_t LLCC68GetBusyTimeoutCount( void )
{
    return s_busyTimeoutCnt;
}

/* 控制SPI的NSS引脚电平: 0低电平选中, 非0高电平释放 */
void LLCC68SetNss(uint8_t lev)
{
    if(lev)
    {
        GPIO_SetBits(RADIO_NSS_PORT, RADIO_NSS_PIN);    /* 输出高电平 */
    }
    else
    {
        GPIO_ResetBits(RADIO_NSS_PORT, RADIO_NSS_PIN);  /* 输出低电平 */
    }
}

/* SPI收发一个字节的数据 */
uint8_t LLCC68SpiInOut(uint8_t data)
{
    /* 等待发送缓冲空 */
    while(SPI_I2S_GetStatus(SPI1, SPI_I2S_TE_FLAG) == RESET);
    SPI_I2S_TransmitData(SPI1, data);
    /* 等待接收缓冲非空(一个字节收发完成) */
    while(SPI_I2S_GetStatus(SPI1, SPI_I2S_RNE_FLAG) == RESET);
    return (uint8_t)SPI_I2S_ReceiveData(SPI1);
}

/* 检查频率是否需要支持, 本模组直接返回true */
bool LLCC68CheckRfFrequency( uint32_t frequency )
{
    return true;
}

/*------------------------- 延时 -------------------------*/
/* SysTick 轮询实现阻塞延时(COUNTFLAG 锁存, 不使能SysTick中断)
 * 特点: 不产生中断, 因此在任何上下文(含中断)里调用都安全, 也不会错过计数周期
 * 节拍 = 1 ms: LOAD = SystemCoreClock/1000-1，而 SystemCoreClock = SYSCLK_FREQ = 8e6
 * （Keil 工程 Define 定义），SysTick 又按 HCLK=8MHz 计数 -> 8e6/1000 个计数正好 1 ms。
 * ⚠️ 前提是工程里定义了 SYSCLK_FREQ=8000000；换到没定义它的工程时 SystemCoreClock
 *    会落到默认 48e6，那时才是"标称值的 6 倍"（本工程不是这种情况，实测 5 s 周期准）。
 * 注意: 若工程中将SysTick用于RTOS心跳, 请改用工程自身的delay实现
 */
void LLCC68DelayMs(uint32_t ms)
{
    SysTick->LOAD = SystemCoreClock / 1000U - 1U;
    SysTick->VAL  = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    while(ms--)
    {
        while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0U);
    }

    SysTick->CTRL = 0U;
}

/*------------------------- 收发超时定时器 -------------------------*/
/* 使用 TIM3 产生周期更新中断（N32G031 没有 GTIM8，用通用 TIM3）
 * 周期 = 1 ms（Prescaler/Period 都按 SystemCoreClock=8e6 算，见 LLCC68TimerInit），
 * 所以 timer_count 的"1"就是 1 ms，下面的 Tx/Rx 超时比较可以直接按 ms 用。
 * 到期后直接回调 RadioOnTxTimeoutIrq/RadioOnRxTimeoutIrq，
 * 所以 radio 层的事件回调(TxTimeout/RxTimeout)运行在中断上下文里，
 * 只能置标志，真正的处理必须回到主循环（见 app_lora.c 的 LoraApp_ProcessIrq）
 */
void TIM3_IRQHandler(void)
{
    uint32_t diffMs = 0;

    if(TIM_GetIntStatus(TIM3, TIM_INT_UPDATE) != RESET)
    {
        timer_count++;
        TIM_ClrIntPendingBit(TIM3, TIM_INT_UPDATE);

        /* 处理tx超时定时器 */
        if(txTimerHandle.isRun)
        {
            if(timer_count >= txTimerHandle.startMs)
            {
                diffMs = timer_count - txTimerHandle.startMs;
            }
            else /* 计数回绕 */
            {
                diffMs = 0xffffffff - txTimerHandle.startMs + timer_count;
            }
            if(diffMs > txTimerHandle.delayMs)
            {
                LLCC68TxTimerStop();
                txTimerHandle.startMs = timer_count;
                RadioOnTxTimeoutIrq(NULL);
            }
        }

        /* 处理rx超时定时器 */
        if(rxTimerHandle.isRun)
        {
            if(timer_count >= rxTimerHandle.startMs)
            {
                diffMs = timer_count - rxTimerHandle.startMs;
            }
            else /* 计数回绕 */
            {
                diffMs = 0xffffffff - rxTimerHandle.startMs + timer_count;
            }
            if(diffMs > rxTimerHandle.delayMs)
            {
                LLCC68RxTimerStop();
                rxTimerHandle.startMs = timer_count;
                RadioOnRxTimeoutIrq(NULL);
            }
        }
    }
}

/* 初始化定时器(TIM3, 产生周期更新中断)，周期 = **1 ms**
 * 注: TIM3 挂在 APB1; APB1 预分频=1 时 TIM3 计数时钟 = PCLK1 = 8MHz。
 *     Prescaler = SystemCoreClock/10000-1 = 8e6/10000-1 = 799
 *       -> 计数频率 = 8MHz/800 = 10 kHz
 *     Period = 10-1 = 9（即每 10 个计数溢出一次）
 *       -> 更新中断频率 = 10kHz/10 = 1 kHz -> **1 ms 一次** ✔
 *     调用方（LLCC68TxTimerStart 等）按 ms 用完全正确，不用做换算。
 */
void LLCC68TimerInit(void)
{
    TIM_TimeBaseInitType TIM_TimeBaseStructure;
    NVIC_InitType        NVIC_InitStructure;

    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM3, ENABLE);

    TIM_InitTimBaseStruct(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.Prescaler = SystemCoreClock / 10000U - 1U;
    TIM_TimeBaseStructure.Period    = 10U - 1U;
    TIM_TimeBaseStructure.ClkDiv    = TIM_CLK_DIV1;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;
    TIM_InitTimeBase(TIM3, &TIM_TimeBaseStructure);

    /* N32G031 的 NVIC 只有 3 个成员：通道/优先级(0最高,3最低)/使能 */
    NVIC_InitStructure.NVIC_IRQChannel         = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 1U;
    NVIC_InitStructure.NVIC_IRQChannelCmd      = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 关闭软定时器 */
    txTimerHandle.isRun = 0;
    rxTimerHandle.isRun = 0;

    TIM_Enable(TIM3, ENABLE);
    TIM_ClrIntPendingBit(TIM3, TIM_INT_UPDATE);
    TIM_ConfigInt(TIM3, TIM_INT_UPDATE, ENABLE);
}


/* 注意: nMs 在 radio.c 里来自 SetTxConfig/SetRx 的 timeout 参数。
 * 若为 0 则视为"不使用软件超时"(见 LLCC68TxTimerStart 直接不开定时器)，
 * 否则 TIM3 到期会回调 TxTimeout/RxTimeout 把还没发完的包打断
 * （TIM3 节拍 = 1ms，见 LLCC68TimerInit；历史上出现过 TxTimeout=0
 *  导致 28 字节的包还没发完就被打断、对端收不到）
 */
void LLCC68SetTxTimerValue(uint32_t nMs)
{
    txTimerHandle.delayMs = nMs;
}

void LLCC68TxTimerStart(void)
{
    if(txTimerHandle.delayMs == 0U)
    {
        txTimerHandle.isRun = 0;   /* 0 = 不做软件发射超时，交给芯片自身超时 */
        return;
    }
    txTimerHandle.startMs = timer_count;
    txTimerHandle.isRun   = 1;
}

/* Stop 会被 TIM3 中断调用，禁止在这里打印/操作SPI */
void LLCC68TxTimerStop(void)
{
    txTimerHandle.isRun = 0;
}

void LLCC68SetRxTimerValue(uint32_t nMs)
{
    rxTimerHandle.delayMs = nMs;
}

void LLCC68RxTimerStart(void)
{
    if(rxTimerHandle.delayMs == 0U)
    {
        rxTimerHandle.isRun = 0;   /* 0 = 不限制接收时间 */
        return;
    }
    rxTimerHandle.startMs = timer_count;
    rxTimerHandle.isRun   = 1;
}

/* Stop 会被 TIM3 中断调用，禁止在这里打印/操作SPI */
void LLCC68RxTimerStop(void)
{
    rxTimerHandle.isRun = 0;
}

/* 给应用层用的自由运行节拍（TIM3 启动后一直计数，HSI 时基）
 * 单位 = **1 ms**（见 LLCC68TimerInit），调用方直接按 ms 用 */
uint32_t LLCC68GetTimerCount(void)
{
    return timer_count;
}
