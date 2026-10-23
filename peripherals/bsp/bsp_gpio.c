#include "bsp_gpio.h"

/* 外设总电源使能脚：PB7（板上高边 P-MOS 开关，低 = 上电，高 = 断电）
 *
 * 四档实测（决定只能用两档）：
 *   驱动为高       -> 整板 POR/BOR 复位循环，约 3 秒一次              【不要用】
 *   释放为高阻     -> 外设域只有 1.2V（模块不可用），且整板 POR 复位    【不要用】
 *   驱动为低       -> 开关全开、外设域 3.3V，代价 ~1mA 栅极偏置          【醒着用】
 *   输入上拉(~40k) -> 弱上拉拉高栅极、P-MOS 关断、外设域 0V，几 uA      【STOP 用】
 *
 * 所以低功耗只有一条路：睡前输入上拉关断外设域，唤醒后驱动低拉回 3.3V + 等 POR。
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

/* 把 PB7 配成推挽输出并驱动到指定电平（正常固件只用 level=0）
 * @param level 0 = 驱动低（外设上电）；1 = 驱动高（本板会复位循环，勿用） */
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
 * @brief 外设电源脚初始化：驱动低 = 外设上电
 */
void BSP_Periph_Power_Init(void)
{
    periph_pwr_pin_drive(0U);
}

/**
 * @brief 确保外设上电 —— 把 PB7 驱动为低（幂等，可反复调）
 * @note  STOP 唤醒后必须先调它把外设域从 0V 拉回 3.3V + 延时等 POR，再重新 Init 外设。
 */
void BSP_Periph_Power_On(void)
{
    periph_pwr_pin_drive(0U);
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
