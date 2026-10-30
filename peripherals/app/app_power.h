#ifndef __APP_POWER_H__
#define __APP_POWER_H__

#include <stdint.h>

/**
 * @brief 读取供电电压，单位mV（用内部VREFINT反算VDD，不需要外部引脚）
 * @return 电压mV；0 = 采样失败/超出合理范围(2200~5500mV)
 */
uint16_t app_power_get_battery_mv(void);

/**
 * @brief 电池电量百分比 0~100
 * @note  按 3.6V 锂亚一次电池(ER14505)折算：3.6V=100%，2.8V=0%，中间线性
 * @retval 0:采样失败或电压≤2.8V；100:电压≥3.6V
 */
uint8_t app_power_get_battery_percent(void);

#endif
