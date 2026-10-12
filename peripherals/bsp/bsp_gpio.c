#include "bsp_gpio.h"

/* 外设总电源使能脚：PB7（板上高边 P-MOS 开关，低 = 上电，高 = 断电）
 *
 * 三档实测：
 *   驱动为高（想断电） → 整板 POR/BOR 复位循环，约 3 秒一次        【不要用】
 *   释放为高阻        → 外设域只有 1.2V（模块不可用），且整板 POR 复位 【不要用】
 *   驱动为低          → 开关全开、外设域正常上电 3.3V ✅（醒着用这档，代价 ~1mA 偏置）
 *   输入上拉(~40k)    → 弱上拉把栅极拉高、P-MOS 关断、外设域掉到 0V ✅（STOP 睡着用这档，几 uA）
 *
 * 低功耗策略（方案B，2026-10-10 实测几 uA）：
 *   STOP 前 BSP_Periph_Power_PullUp() 把 PB7 配输入上拉，栅极被弱上拉拉高、
 *   P-MOS 关断、外设域掉到 0V（几 uA）。唤醒后 BSP_Periph_Power_On() 驱动低
 *   把外设域拉回 3.3V + 延时等 POR，再重新 Init 外设。
 *
 * ⚠️ 以前还试过方案A（纯高阻释放 PB7），实测外设域只到 1.2V 半开（0.43mA），
 *    且在本板会触发 POR/BOR 复位循环，已删除。方案B 输入上拉能把栅极拉到
 *    关断态才达到几 uA。
 *
 * ⚠️ 顺带更正：以前这里写的是 PA7（那是探针激励脚，配 PB1 做 ADC，见 water_cu.c），
 *    导致"外设电源"的 API 一直在驱动探针，而真正的电源脚 PB7 从未被驱动。
 */
#define PERIPH_PWR_PORT     GPIOB
#define PERIPH_PWR_PIN      GPIO_PIN_7
#define PERIPH_PWR_CLK      RCC_APB2_PERIPH_GPIOB

/* 把外设电源脚配成输入+内部上拉(~40k) —— STOP 睡前用。
 * 不主动驱动,用内部弱上拉被动拉高栅极,把 P-MOS 关断到 0V（外设域断电几 uA）。 */
static void periph_pwr_pin_pullup(void)
{
    GPIO_InitType GPIO_InitStructure;

    RCC_EnableAPB2PeriphClk(PERIPH_PWR_CLK, ENABLE);

    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin             = PERIPH_PWR_PIN;
    GPIO_InitStructure.GPIO_Mode       = GPIO_MODE_INPUT;     /* 输入,不主动驱动 */
    GPIO_InitStructure.GPIO_Pull       = GPIO_PULL_UP;         /* 内部 ~40k 上拉拉高栅极 */
    GPIO_InitStructure.GPIO_Speed      = GPIO_SPEED_LOW;
    GPIO_InitStructure.GPIO_Current    = GPIO_DC_LOW;
    GPIO_InitStructure.GPIO_Alternate  = GPIO_NO_AF;
    GPIO_InitPeripheral(PERIPH_PWR_PORT, &GPIO_InitStructure);
}

/**
 * @brief 强制驱动 PB7 到指定电平（只给 PERIPH_PWR_FORCE 排查用）
 * @param level 0 = 驱动低；1 = 驱动高
 * @note  ⚠️ 这是"驱动"，不是"释放"：按本板实测，驱动高会整板复位循环，
 *        驱动低会多吃约 1mA。正常固件醒着用驱动低（上电），STOP 用上拉（几 uA）。
 */
static void periph_pwr_pin_drive(uint8_t level)
{
    GPIO_InitType GPIO_InitStructure;

    RCC_EnableAPB2PeriphClk(PERIPH_PWR_CLK, ENABLE);

    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin             = PERIPH_PWR_PIN;
    GPIO_InitStructure.GPIO_Mode       = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Pull       = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Speed      = GPIO_SPEED_LOW;
    GPIO_InitStructure.GPIO_Current    = GPIO_DC_LOW;
    GPIO_InitStructure.GPIO_Alternate  = GPIO_NO_AF;
    GPIO_InitPeripheral(PERIPH_PWR_PORT, &GPIO_InitStructure);

    if (level != 0U)
    {
        GPIO_SetBits(PERIPH_PWR_PORT, PERIPH_PWR_PIN);
    }
    else
    {
        GPIO_ResetBits(PERIPH_PWR_PORT, PERIPH_PWR_PIN);
    }
}

/**
 * @brief 外设电源脚初始化：默认驱动低（外设上电）
 * @note  PERIPH_PWR_FORCE=1（默认）驱动低上电；=2 驱动高（台架排查，会复位）。
 */
void BSP_Periph_Power_Init(void)
{
#if   (PERIPH_PWR_FORCE == 2)
    periph_pwr_pin_drive(1U);       /* 台架反档试验：驱动高（会复位） */
#else
    periph_pwr_pin_drive(0U);       /* 默认：驱动低 = 外设上电 */
#endif
}

/**
 * @brief 确保外设上电 —— 把 PB7 驱动为低（幂等，可反复调）
 * @note  STOP 唤醒后必须先调它把外设域从 0V 拉回 3.3V + 延时等 POR，再重新 Init 外设。
 */
void BSP_Periph_Power_On(void)
{
#if (PERIPH_PWR_FORCE == 2)
    periph_pwr_pin_drive(1U);       /* 台架反档试验：驱动高 */
#else
    periph_pwr_pin_drive(0U);       /* 正常：驱动低 = 外设上电 */
#endif
}

/* 把外设电源脚配成输入+内部上拉 —— STOP 睡前调它关断外设域到 0V（几 uA）。
 * 唤醒后必须先 BSP_Periph_Power_On() 恢复 + 延时等 POR，再重新 Init。 */
void BSP_Periph_Power_PullUp(void)
{
    periph_pwr_pin_pullup();
}

/**
 * @brief 读 PB7 在"不驱动"状态下的实际电平（数字输入读一次，读完恢复驱动低）
 * @retval 0 = 低；1 = 高
 */
uint8_t BSP_Periph_PowerPeek(void)
{
    GPIO_InitType GPIO_InitStructure;
    volatile uint32_t d;
    uint8_t lv;

    RCC_EnableAPB2PeriphClk(PERIPH_PWR_CLK, ENABLE);

    /* 数字输入 + 无上下拉：只"看"这条网络，不驱动它 */
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin             = PERIPH_PWR_PIN;
    GPIO_InitStructure.GPIO_Mode       = GPIO_MODE_INPUT;
    GPIO_InitStructure.GPIO_Pull       = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Speed      = GPIO_SPEED_LOW;
    GPIO_InitStructure.GPIO_Current    = GPIO_DC_LOW;
    GPIO_InitStructure.GPIO_Alternate  = GPIO_NO_AF;
    GPIO_InitPeripheral(PERIPH_PWR_PORT, &GPIO_InitStructure);

    /* 网络上可能有电容/偏置电阻，等一小会儿再读，避免读到刚切过来的残值 */
    for (d = 0U; d < 20000U; d++)
    {
        __NOP();
    }
    lv = (GPIO_ReadInputDataBit(PERIPH_PWR_PORT, PERIPH_PWR_PIN) != 0) ? 1U : 0U;

    /* 读完恢复成驱动低（外设上电）——
     * 千万别在这里配成上拉/高阻：那会把外设电源关掉或搞复位。 */
    BSP_Periph_Power_On();
    return lv;
}
