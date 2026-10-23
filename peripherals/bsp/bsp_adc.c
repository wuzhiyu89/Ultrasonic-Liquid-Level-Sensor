#include "bsp_adc.h"
#include "delay.h"

//==================== 液位探针 PB1 ADC_CH_9 ====================
/**
 * @brief 只初始化 ADC 本体，不动 PB1 的 GPIO 配置
 * @note  探针分压靠 PB1 上挂的上下拉当负载，测量期间必须保持；
 *        ⚠️ 一旦把 PB1 配成模拟输入（无上下拉），负载消失，干态/湿态都读满量程。
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

/* ⚠️ 探针测量只调 BSP_ADC_LevelProbe_AdcInit()，不要把 PB1 配成模拟输入：
 *    模拟模式会拆掉 PB1 上的负载，节点充电后不再泄放，干/湿态都读满量程。
 *    另：DeInit 里的 ADC_EnableTempSensor(DISABLE) 与 VREFINT 是两个位，不能合并。 */

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
 * @note ⚠️ VREFEN(CTRL3 bit1) 与 TEMPEN(CTRL2 bit23) 是两个位，都要关，否则休眠电流约 0.43mA；
 *       关 AD_ON 后须等 PDRDY(CTRL3 bit6) 置位再关时钟，否则掉电流程被打断。
 */
void BSP_ADC_BatDetect_DeInit(void)
{
    uint32_t guard;

    /* 关两个基准/温度使能位 */
    ADC_EnableVrefint(DISABLE);         /* CTRL3 bit1 */
    ADC_EnableTempSensor(DISABLE);      /* CTRL2 bit23 */

    /* 关 ADC 本体，硬件开始掉电 */
    ADC_Enable(ADC, DISABLE);

    /* 等掉电真正完成（带超时，不卡死） */
    guard = 200000U;
    while((ADC_GetFlagStatusNew(ADC, ADC_FLAG_PD_RDY) == 0U) && (guard != 0U))
    {
        guard--;
    }

    /* 时钟最后关（上面读标志位要用它） */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC, DISABLE);
}
