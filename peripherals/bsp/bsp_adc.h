#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "n32g031.h"

/******** 1.液位探针 PB1 ADC_CH9 ********/
/**
 * @brief 只初始化 ADC 本体，不动 PB1 的 GPIO 配置
 * @note  探针"双向差分"测量必须用它：分压靠 PB1 的内部上/下拉当负载，
 *        采样期间必须保持负载，不能把 PB1 改成模拟。
 */
void BSP_ADC_LevelProbe_AdcInit(void);

/**
 * @brief 液位探针ADC初始化：把 PB1 配成模拟输入 + 初始化 ADC
 * @note  会把 PB1 的内部上/下拉关掉，只适合"只读ADC、不驱动"的场景
 */
void BSP_ADC_LevelProbe_Init(void);

/**
 * @brief 读取液位探针原始ADC值
 * @retval 0~4095
 */
uint16_t BSP_ADC_LevelProbe_ReadRaw(void);

/**
 * @brief 反初始化液位探针ADC，关闭时钟
 */
void BSP_ADC_LevelProbe_DeInit(void);


/******** 2.电池电压检测（内部VREFINT ADC_CH14，无外部引脚） ********/
/**
 * @brief 电池电压检测ADC初始化
 * @note 开启ADC外设、开启内部VREFINT(1.21V基准)
 */
void BSP_ADC_BatDetect_Init(void);

/**
 * @brief 读取VREFINT原始ADC采样值
 * @retval 0~4095
 */
uint16_t BSP_ADC_BatDetect_ReadRaw(void);

/**
 * @brief 电池检测反初始化，关闭VREFINT、关闭ADC外设时钟
 */
void BSP_ADC_BatDetect_DeInit(void);

#endif
