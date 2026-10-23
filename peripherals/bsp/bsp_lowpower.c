#include "bsp_lowpower.h"
#include "n32g031_lptim.h"
#include "n32g031_exti.h"

/* STOP 期间不把 PA13/PA14(SWDIO/SWCLK) 设成模拟：休眠中还能直接被调试器 attach。
 * （设模拟只省这两根输入缓冲的几 µA，实测对总电流无影响。） */

/**
 * @brief 低功耗模块初始化，打开PWR时钟
 */
void BSP_LowPower_Init(void)
{
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, ENABLE);
}

/**
 * @brief 把 GPIO 设为模拟模式，消除 IO 漏电流；进 STOP 前调用
 * @note 模拟模式断开施密特触发器和上下拉，漏电最低。不是"全部"：
 *       PB7(外设电源)、PB6(日志 TX) 排除，PA13/PA14(SWD) 保留以便调试。
 *       唤醒后业务 GPIO 需要重新初始化。
 */
void BSP_GPIO_SetAllAnalog(void)
{
    GPIO_InitType GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode      = GPIO_MODE_ANALOG;
    GPIO_InitStruct.GPIO_Pull      = GPIO_NO_PULL;
    GPIO_InitStruct.GPIO_Speed     = GPIO_SPEED_LOW;
    GPIO_InitStruct.GPIO_Current   = GPIO_DC_LOW;
    GPIO_InitStruct.GPIO_Alternate = GPIO_NO_AF;

    /* GPIOA：除 PA13/PA14(SWD) 外全部模拟。PA7 是探针激励脚，设模拟 = 不驱动探针。 */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    GPIO_InitStruct.Pin = GPIO_PIN_ALL & (~(GPIO_PIN_13 | GPIO_PIN_14));
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStruct);

    /* GPIOB：除 PB7(外设电源) 与 PB6(日志 TX) 外全部设模拟。
     *   PB7：这里不设模拟，由 app_lowpower_enter_stop_pullup() 在关 GPIO 时钟前
     *        配成输入上拉，P-MOS 关断、外设域 0V（几 uA）。
     *   PB6 = USART1_TX(printf)：设成模拟等于把日志线剪掉，睡前最后那段日志
     *        （含 [lp] water-wake 武装状态）会全部丢失，所以保留并显式还原成 AF4。 */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    GPIO_InitStruct.Pin = GPIO_PIN_ALL & (~(GPIO_PIN_7 | GPIO_PIN_6));
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStruct);

    {
        GPIO_InitType logpin;

        /* 不能调 GPIO_InitStruct(&logpin)：本函数有同名局部变量会遮住函数名，
         * 下面 6 个字段已全部显式赋值。 */
        logpin.Pin            = GPIO_PIN_6;
        logpin.GPIO_Mode      = GPIO_MODE_AF_PP;
        logpin.GPIO_Pull      = GPIO_NO_PULL;
        logpin.GPIO_Speed     = GPIO_SPEED_HIGH;
        logpin.GPIO_Current   = GPIO_DC_HIGH;
        logpin.GPIO_Alternate = GPIO_AF4_USART1;
        GPIO_InitPeripheral(GPIOB, &logpin);
    }

    /* GPIOC：仅 PC13/PC14/PC15 有效。
     *   PC15 = LoRa DIO1、PC14 = LoRa BUSY 一并设模拟，STOP 期间收不到 DIO1 边沿，
     *   所以【不能靠 DIO1 唤醒】（可用唤醒源见 bsp_lowpower.h）。 */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOC, ENABLE);
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStruct);

    /* GPIOF：仅PF0 PF1 PF2 PF6 PF7有效 */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOF, ENABLE);
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitPeripheral(GPIOF, &GPIO_InitStruct);
}

/**
 * @brief 配置EXTI引脚外部中断唤醒STOP模式
 * @param GPIOx      GPIOA/GPIOB/GPIOC/GPIOF
 * @param pin        GPIO_PIN_x
 * @param edge       EXTI_Trigger_Falling / EXTI_Trigger_Rising / EXTI_Trigger_Rising_Falling
 * @note NVIC中断需要用户在应用层自己配置；STOP模式支持EXTI线唤醒
 */
void BSP_EXTI_WakeupPinConfig(GPIO_Module* GPIOx,uint16_t pin,EXTI_TriggerType edge)
{
    uint8_t portSrc,pinSrc;
    EXTI_InitType EXTI_InitStruct;
    GPIO_InitType GPIO_InitStruct;

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);

    portSrc = GPIO_GET_INDEX(GPIOx);
    for(pinSrc = 0; pinSrc < 16; pinSrc++)
    {
        if(pin & (1U << pinSrc))
            break;
    }

    /* IO配置带上拉，避免浮空 */
    GPIO_InitStruct.Pin             = pin;
    GPIO_InitStruct.GPIO_Mode       = GPIO_MODE_INPUT;
    GPIO_InitStruct.GPIO_Pull       = GPIO_PULL_UP;
    GPIO_InitStruct.GPIO_Speed      = GPIO_SPEED_LOW;
    GPIO_InitStruct.GPIO_Current    = GPIO_DC_LOW;
    GPIO_InitStruct.GPIO_Alternate  = GPIO_NO_AF;
    GPIO_InitPeripheral(GPIOx, &GPIO_InitStruct);

    /* GPIO映射EXTI线路，库标准API */
    GPIO_ConfigEXTILine(portSrc, pinSrc);

    EXTI_InitStruct.EXTI_Line      = 1U << pinSrc;
    EXTI_InitStruct.EXTI_Mode       = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger    = edge;
    EXTI_InitStruct.EXTI_LineCmd   = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStruct);
}

/**
 * @brief 进入低功耗模式
 * @param mode LOWPOWER_SLEEP / LOWPOWER_STOP / LOWPOWER_PD
 * @warning STOP唤醒后时钟切回HSI，应用层必须重新初始化时钟
 *          （本工程只开HSI、不开PLL，无PLL可恢复；
 *            app_lowpower.c 已在唤醒后调用 SystemClock_Config()）；
 *          PD模式唤醒直接复位，程序从头执行
 */
void BSP_EnterLowPower(LowPower_Mode_t mode)
{
    /* 清除唤醒标志 */
    PWR_ClearFlag(PWR_CLR_WKUP_FLAG);

    switch(mode)
    {
    case LOWPOWER_SLEEP:
        /* SLEEP模式，WFI进入 */
        PWR_EnterSLEEPMode(SLEEP_OFF_EXIT, PWR_SLEEPENTRY_WFI);
        break;

    case LOWPOWER_STOP:
        /* 清掉调试器残留的 STOP/PD 调试时钟使能（SWD连过会置位，
         * STOP下调试时钟继续跑，旁路正常时钟门控，漏电可达数百µA） */
        PWR->DBGCTRL = 0;

        PWR_EnterSTOPMode(PWR_STOPENTRY_WFI);
        break;

    case LOWPOWER_PD:
        /* PowerDown掉电待机模式，唤醒后复位 */
        PWR_EnterPDMode(PWR_PDEntry_WFI);
        break;

    default:
        break;
    }

    /*
     * !!!重要提醒!!!
     * LOWPOWER_STOP唤醒后硬件自动切回HSI内部高速时钟
     * 此处需要调用系统时钟初始化函数恢复SYSCLK，例如 SystemClock_Config()
     * （本工程只开HSI、不开PLL；实际调用点在 app_lowpower.c 唤醒后）
    */
}
