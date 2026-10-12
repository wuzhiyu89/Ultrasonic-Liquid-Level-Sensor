#include "bsp_adc.h"
#include "delay.h"

//==================== 液位探针 PB1 ADC_IH9 ====================
/**
 * @brief 只初始化 ADC 本体，**不动 PB1 的 GPIO 配置**
 * @note  探针测量必须用这个，而不是会把 PB1 设成模拟的那个版本：
 *        探针的分压关系靠"PB1 上挂的内部上拉/下拉"当负载电阻，
 *        一旦把 PB1 改成模拟(无上下拉)，负载就没了 —— 只要水有一点导通，
 *        节点就被激励脚直接顶到接近 VDD，干燥/有水都读满量程
 *        （这就是之前"探针读数一直是高电平"的原因）。
 */
void BSP_ADC_LevelProbe_AdcInit(void)
{
    ADC_InitType ADC_InitStructure;

    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC, ENABLE);
    // ADC时钟配置
    RCC_EnableHsi(ENABLE);
    while(RCC_WaitHsiStable() != SUCCESS);
    RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8);
    ADC_ConfigClk(ADC_CTRL3_CKMOD_AHB, RCC_ADCHCLK_DIV16);

    // ADC单次转换，软件触发
    ADC_InitStruct(&ADC_InitStructure);
    ADC_InitStructure.MultiChEn      = DISABLE;
    ADC_InitStructure.ContinueConvEn = DISABLE;
    ADC_InitStructure.ExtTrigSelect  = ADC_EXT_TRIGCONV_NONE;
    ADC_InitStructure.DatAlign       = ADC_DAT_ALIGN_R;
    ADC_InitStructure.ChsNumber      = 1;
    ADC_Init(ADC, &ADC_InitStructure);

    ADC_ConfigRegularChannel(ADC, ADC_CH_9, 1, ADC_SAMP_TIME_240CYCLES5);
    ADC_Enable(ADC, ENABLE);
    while(!ADC_GetFlagStatusNew(ADC, ADC_FLAG_RDY));
    while(ADC_GetFlagStatusNew(ADC, ADC_FLAG_PD_RDY));
}

/**
 * @brief 配置 PB1 为模拟输入 + 初始化 ADC（"只读ADC、不驱动"这类场景用）
 * @note  注意：这个函数会把 PB1 的内部上/下拉关掉。做"双向差分"测量时不要用它，
 *        要用 BSP_ADC_LevelProbe_AdcInit()，否则负载被拆掉、读数恒为高电平。
 */
void BSP_ADC_LevelProbe_Init(void)
{
    GPIO_InitType GPIO_InitStructure;

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);

    // PB1模拟输入
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin             = GPIO_PIN_1;
    GPIO_InitStructure.GPIO_Mode       = GPIO_MODE_ANALOG;
    GPIO_InitStructure.GPIO_Pull       = GPIO_NO_PULL;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    BSP_ADC_LevelProbe_AdcInit();
}

uint16_t BSP_ADC_LevelProbe_ReadRaw(void)
{
    ADC_EnableSoftwareStartConv(ADC, ENABLE);
    uint32_t timeout = 80000U;
    while((ADC_GetFlagStatus(ADC, ADC_FLAG_ENDC) == RESET) && (timeout>0U))
    {
        timeout--;
    }
    if(timeout == 0U)
    {
        return 0U;
    }
    uint16_t val = (uint16_t)ADC_GetDat(ADC);
    ADC_ClearFlag(ADC, ADC_FLAG_ENDC);
    return val;
}

void BSP_ADC_LevelProbe_DeInit(void)
{
    uint32_t guard;

    ADC_EnableTempSensor(DISABLE);
    ADC_Enable(ADC, DISABLE);

    /* 等 ADC 真正掉电完成(PDRDY 置位)再关时钟，否则掉电流程被打断 */
    guard = 200000U;
    while((ADC_GetFlagStatusNew(ADC, ADC_FLAG_PD_RDY) == 0U) && (guard != 0U))
    {
        guard--;
    }

    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC, DISABLE);
}

//==================== 电池电压检测 VREFINT ADC_CH_VREFINT ====================
void BSP_ADC_BatDetect_Init(void)
{
    ADC_InitType  ADC_InitStructure;
    // 打开ADC外设时钟
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC, ENABLE);
    // ADC时钟配置
    RCC_EnableHsi(ENABLE);
    while(RCC_WaitHsiStable() != SUCCESS);
    RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8);
    ADC_ConfigClk(ADC_CTRL3_CKMOD_AHB, RCC_ADCHCLK_DIV16);

    ADC_InitStruct(&ADC_InitStructure);
    ADC_InitStructure.MultiChEn      = DISABLE;
    ADC_InitStructure.ContinueConvEn = DISABLE;
    ADC_InitStructure.ExtTrigSelect  = ADC_EXT_TRIGCONV_NONE;
    ADC_InitStructure.DatAlign       = ADC_DAT_ALIGN_R;
    ADC_InitStructure.ChsNumber      = 1;
    ADC_Init(ADC, &ADC_InitStructure);

    // 选择内部VREFINT通道 ADC_CH_VREFINT
    ADC_ConfigRegularChannel(ADC, ADC_CH_VREFINT, 1, ADC_SAMP_TIME_240CYCLES5);

    // 使能内部1.21V参考电压源
    ADC_EnableVrefint(ENABLE);

    ADC_Enable(ADC, ENABLE);
    while(!ADC_GetFlagStatusNew(ADC, ADC_FLAG_RDY));
    while(ADC_GetFlagStatusNew(ADC, ADC_FLAG_PD_RDY));
    while(!ADC_GetFlagStatusNew(ADC, ADC_FLAG_VREF_RDY));
}

uint16_t BSP_ADC_BatDetect_ReadRaw(void)
{

    ADC_EnableSoftwareStartConv(ADC, ENABLE);

    uint32_t timeout = 80000U;
    while((ADC_GetFlagStatus(ADC, ADC_FLAG_ENDC) == RESET) && (timeout>0U))
    {
        timeout--;
    }
    if(timeout == 0U)
    {
        return 0U;
    }

    uint16_t val = (uint16_t)ADC_GetDat(ADC);
    ADC_ClearFlag(ADC, ADC_FLAG_ENDC);
    return val;
}

/**
 * @brief 关闭电池检测用的 ADC，必须彻底掉电
 * @note  【实测根因】原来这里是"关VREFINT -> 关AD_ON -> 立刻关时钟"，
 *        三件事都是对的、但顺序和等待不对，结果进 STOP 后 ADC 内部模拟电路
 *        一直带电，整机休眠电流 0.43mA。修法：
 *          1) CTRL3.VREFEN(bit1) 与 CTRL2.TEMPEN(bit23) 是【两个】位，
 *             名字里都带 VREFINT。SDK 里分别由 ADC_EnableVrefint /
 *             ADC_EnableTempSensor 控制，原代码只关了前者，后者漏关。
 *          2) 关 AD_ON 之后必须等 PDRDY(CTRL3 bit6, "powerdown ready")置位，
 *             确认 ADC 真的掉电完成；原代码立刻关时钟，把掉电流程打断了。
 */
void BSP_ADC_BatDetect_DeInit(void)
{
    uint32_t guard;

    /* 1) 关内部基准/温度传感器相关的两个使能位 */
    ADC_EnableVrefint(DISABLE);         /* CTRL3 bit1 */
    ADC_EnableTempSensor(DISABLE);      /* CTRL2 bit23: Temperature Sensor and VREFINT Enable */

    /* 2) 关 ADC 本体，硬件开始掉电 */
    ADC_Enable(ADC, DISABLE);

    /* 3) 等掉电真正完成（带超时，不卡死） */
    guard = 200000U;
    while((ADC_GetFlagStatusNew(ADC, ADC_FLAG_PD_RDY) == 0U) && (guard != 0U))
    {
        guard--;
    }

    /* 4) 时钟最后关（上面读标志位要用它） */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC, DISABLE);
}
