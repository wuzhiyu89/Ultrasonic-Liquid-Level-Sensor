#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "n32g031.h"

/******** 1.液位探针 PB1 ADC_CH9 ********/
/**
 * @brief 只初始化 ADC 本体，不动 PB1 的 GPIO 配置
 * @note  探针分压靠 PB1 上下拉（本板外部 1M）当负载，采样期间必须保持，不能把 PB1 改成模拟。
 *        ⚠️ 把 PB1 配成模拟会让探针读数恒为高电平。
 */
void BSP_ADC_LevelProbe_AdcInit(void);

/**
 * @brief 读取液位探针原始ADC值
 * @retval 0~4095
 */
uint16_t BSP_ADC_LevelProbe_ReadRaw(void);

/**
 * @brief 反初始化液位探针ADC，关闭时钟
 */
void BSP_ADC_LevelProbe_DeInit(void);


/******** 2.电池电压检测（内部VREFINT ADC_CH13，无外部引脚） ********/
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
 * @brief 电池检测反初始化：关 VREFINT 与温度传感器两个使能位，关 ADC 本体与时钟
 */
void BSP_ADC_BatDetect_DeInit(void);

#endif
