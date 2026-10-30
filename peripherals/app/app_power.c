#include "app_power.h"
#include "bsp_adc.h"

/* 内部基准VREFINT的实测值(mV)：典型手册值1.21V，单颗有离散。
 * 这颗片子按实测校准为1.16V：
 *   3.3V供电 -> raw≈1450 -> 1160*4095/1450 = 3276mV
 *   5V  供电 -> raw≈950  -> 1160*4095/950  = 5000mV
 * 电池供电(3.6V)时 raw≈1319
 * 想更准就用标准电源校准后改这个值 */
#define VREFINT_MV          1160U

/* 合理电压范围(mV)，超出说明采样异常，直接返回0
 * 上限放到 5.5V 以兼容 5V/3.3V 供电场合 */
#define VDD_MIN_VALID_MV    2200U
#define VDD_MAX_VALID_MV    5500U

/* 电池：3.6V 锂亚一次电池(如 ER14505)
 *   满电 3.6V，空电 2.8V（3.6V电池放到2.8V基本就耗尽了）
 *   百分比 = (VDD - 2800) / (3600 - 2800) * 100
 * 换电池类型时只改这两个值即可 */
#define BAT_FULL_MV         3600U
#define BAT_EMPTY_MV        2800U

uint16_t app_power_get_battery_mv(void)
{
    uint16_t raw;
    uint32_t mv;

    /* 【开启 -> 采样 -> 立刻关闭】 */
    BSP_ADC_BatDetect_Init();
    raw = BSP_ADC_BatDetect_ReadRaw();
    BSP_ADC_BatDetect_DeInit();

    if (raw == 0U)
    {
        return 0U;                  /* 采样超时 */
    }

    /* VDD = VREFINT * 4095 / raw */
    mv = ((uint32_t)VREFINT_MV * 4095U) / (uint32_t)raw;
    if ((mv < VDD_MIN_VALID_MV) || (mv > VDD_MAX_VALID_MV))
    {
        return 0U;
    }
    return (uint16_t)mv;
}

uint8_t app_power_get_battery_percent(void)
{
    uint16_t mv = app_power_get_battery_mv();

    if (mv == 0U)
    {
        return 0U;                  /* 采样失败 */
    }
    if (mv <= BAT_EMPTY_MV)
    {
        return 0U;
    }
    if (mv >= BAT_FULL_MV)
    {
        return 100U;
    }
    return (uint8_t)(((uint32_t)(mv - BAT_EMPTY_MV) * 100U) / (BAT_FULL_MV - BAT_EMPTY_MV));
}
