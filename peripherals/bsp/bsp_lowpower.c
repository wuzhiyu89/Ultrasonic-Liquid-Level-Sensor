#include "bsp_lowpower.h"
#include "n32g031_lptim.h"
#include "n32g031_exti.h"

/* LSI实测频率(Hz)：按RTC走时约1.3%偏快估算，30.4kHz */
#define LPTIM_LSI_HZ        30400U

/**
 * @brief 挑一个能让周期落在16位计数器里的分频
 * @param period_ms 目标周期
 * @param out_arr   输出ARR值
 * @retval 分频选择值；0xFF=周期太长，一次放不下
 */
static uint8_t lptim_pick_div(uint32_t period_ms, uint32_t *out_arr)
{
    static const uint32_t div_sel[8] = {LPTIM_PRESCALER_DIV1, LPTIM_PRESCALER_DIV2,
                                        LPTIM_PRESCALER_DIV4, LPTIM_PRESCALER_DIV8,
                                        LPTIM_PRESCALER_DIV16, LPTIM_PRESCALER_DIV32,
                                        LPTIM_PRESCALER_DIV64, LPTIM_PRESCALER_DIV128};
    static const uint32_t div_val[8] = {1U, 2U, 4U, 8U, 16U, 32U, 64U, 128U};
    uint32_t i;
    uint32_t ticks;

    for (i = 0U; i < 8U; i++)
    {
        /* ticks = period_ms * (LSI/1000) / 分频 */
        ticks = (period_ms * (LPTIM_LSI_HZ / 1000U)) / div_val[i];
        if ((ticks > 0U) && (ticks <= 65536U))
        {
            *out_arr = ticks - 1U;
            return (uint8_t)div_sel[i];
        }
    }
    return 0xFFU;
}

uint8_t BSP_LPTIM_WakeupInit(uint32_t period_ms)
{
    NVIC_InitType nvic_cfg;
    uint32_t arr = 0xFFFFU;
    uint8_t  div;

    div = lptim_pick_div(period_ms, &arr);
    if (div == 0xFFU)
    {
        return 1U;
    }

    /* LSI + LPTIM时钟源 */
    RCC_EnableLsi(ENABLE);
    while (RCC_GetFlagStatus(RCC_LSCTRL_FLAG_LSIRD) == RESET)
    {
    }
    RCC_ConfigLPTIMClk(RCC_LPTIMCLK_SRC_LSI);
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_LPTIM | RCC_APB1_PERIPH_LPTIMPCLK, ENABLE);

    LPTIM_Disable(LPTIM);
    LPTIM_SetPrescaler(LPTIM, div);
    LPTIM_EnableIT_CMPM(LPTIM);              /* 用比较匹配中断(和厂商例程一致) */
    LPTIM_Enable(LPTIM);                     /* 先使能 */

    /* 【关键】LPTIM使能后要等ARROK/CMPOK置位才能写ARR/CMP，否则写了不生效
     * (SDK的setter不等这两个标志，厂商例程也没等；不等的话ARR/CMP保持复位值，
     *  LPTIM就不会产生比较匹配中断 —— 这就是之前LPTIM完全不工作的原因)。
     * 这里带超时，避免卡死。 */
    {
        uint32_t guard = 100000U;

        while ((LPTIM_IsActiveFlag_ARROK(LPTIM) == 0U) && (guard != 0U))
        {
            guard--;
        }
        LPTIM_SetAutoReload(LPTIM, arr);      /* 周期 = (arr+1)/fs */
        LPTIM_ClearFlag_ARROK(LPTIM);

        guard = 100000U;
        while ((LPTIM_IsActiveFlag_CMPOK(LPTIM) == 0U) && (guard != 0U))
        {
            guard--;
        }
        LPTIM_SetCompare(LPTIM, arr / 2U);    /* 比较值放中间，每个重装周期触发一次 */
        LPTIM_ClearFlag_CMPOK(LPTIM);
    }

    LPTIM_ClearFLAG_CMPM(LPTIM);
    LPTIM_StartCounter(LPTIM, LPTIM_OPERATING_MODE_CONTINUOUS);

    /* NVIC：LPTIM/TIM6共用一个向量 */
    nvic_cfg.NVIC_IRQChannel         = LPTIM_TIM6_IRQn;
    nvic_cfg.NVIC_IRQChannelPriority = 2U;
    nvic_cfg.NVIC_IRQChannelCmd      = ENABLE;
    NVIC_Init(&nvic_cfg);

    /* EXTI23 = LPTIM唤醒线，STOP唤醒用；中断和事件掩码都开 */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);
    EXTI->IMASK  |= EXTI_LINE23;
    EXTI->EMASK  |= EXTI_LINE23;
    EXTI->RT_CFG |= EXTI_LINE23;
    EXTI_ClrITPendBit(EXTI_LINE23);

    return 0U;
}

void BSP_LPTIM_WakeupDeInit(void)
{
    LPTIM_DisableIT_CMPM(LPTIM);
    LPTIM_ClearFLAG_CMPM(LPTIM);
    LPTIM_Disable(LPTIM);
    EXTI->IMASK &= ~EXTI_LINE23;
    EXTI->EMASK &= ~EXTI_LINE23;
    EXTI_ClrITPendBit(EXTI_LINE23);
}
/* STOP 期间是否把 PA13/PA14(SWDIO/SWCLK) 也设成模拟态
 *   0 = 保留(默认)：休眠中也能直接被调试器attach，开发期方便
 *   1 = 设成模拟：省掉这两根输入缓冲的几 µA，但休眠中"不复位直接attach"会失败
 * 排障结论：它对那 430µA 没有影响（根因是 ADC 关断，见 bsp_adc.c），所以正式值取 0。 */
#ifndef LP_ANALOG_SWD_PINS
#define LP_ANALOG_SWD_PINS      0
#endif

/**
 * @brief 低功耗模块初始化，打开PWR时钟
 */
void BSP_LowPower_Init(void)
{
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, ENABLE);
}

/**
 * @brief 将全部GPIO设为模拟模式，消除IO漏电流；进入低功耗前调用
 * @note 模拟模式断开施密特触发器和上下拉电阻，漏电最低；
 *       唤醒后业务GPIO需要重新初始化
 */
void BSP_GPIO_SetAllAnalog(void)
{
    GPIO_InitType GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode      = GPIO_MODE_ANALOG;
    GPIO_InitStruct.GPIO_Pull      = GPIO_NO_PULL;
    GPIO_InitStruct.GPIO_Speed     = GPIO_SPEED_LOW;
    GPIO_InitStruct.GPIO_Current   = GPIO_DC_LOW;
    GPIO_InitStruct.GPIO_Alternate = GPIO_NO_AF;

    /* GPIOA：
     *   PA7  = 探针激励脚（不是电源脚！），一起设模拟 = 不驱动探针
     *   PA13/PA14 = SWDIO/SWCLK，由 LP_ANALOG_SWD_PINS 决定是否一起设模拟
     *   其余全部模拟 */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
#if (LP_ANALOG_SWD_PINS)
    GPIO_InitStruct.Pin = GPIO_PIN_ALL;
#else
    GPIO_InitStruct.Pin = GPIO_PIN_ALL & (~(GPIO_PIN_13 | GPIO_PIN_14));
#endif
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStruct);

    /* GPIOB：除 PB7 外全部设模拟。
     *   PB7 = 外设总电源使能：这里排除它不设模拟，由 app_lowpower_enter_stop_pullup()
     *   在关 GPIO 时钟前把它配成输入上拉（方案B），P-MOS 关断、外设域 0V（几 uA）。 */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    GPIO_InitStruct.Pin = GPIO_PIN_ALL & (~GPIO_PIN_7);
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStruct);

    /* GPIOC：仅PC13 PC14 PC15有效 */
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
 * @warning STOP唤醒后，系统时钟切回HSI，必须重新初始化SystemClock；
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
     * 此处需要调用你的系统时钟初始化函数恢复SYSCLK，例如 SystemClock_Init();
    */
}
