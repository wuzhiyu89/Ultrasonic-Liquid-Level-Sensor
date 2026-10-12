#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include "n32g031.h"

/**
 * 【引脚角色】
 *   PB7 = 外设总电源使能（板上高边 P-MOS：低 = 上电，高 = 断电）
 *   PA7 = 探针激励脚（配 PB1 做 ADC），见 water_cu.c
 *   以前固件把这两个脚搞反了，已更正。
 *
 * 【本板关键结论】
 *   驱动为高 → 整板 POR/BOR 复位循环，不要用
 *   释放高阻 → 外设域只有 1.2V 且整板 POR 复位，不要用
 *   驱动为低 → 外设域正常上电 3.3V ✅（醒着用这档，代价 ~1mA 偏置电流）
 *   输入上拉 → P-MOS 关断、外设域 0V ✅（STOP 睡着用这档，几 uA）
 *
 * 【低功耗策略：方案B（输入上拉，几 uA）】
 *   STOP 前 BSP_Periph_Power_PullUp() → PB7 输入上拉关断外设域到 0V；
 *   唤醒后 BSP_Periph_Power_On() → 驱动低拉回 3.3V + 延时等 POR + 重新 Init。
 *   （方案A 纯高阻释放已删除：本板实测 1.2V 半开 + POR 复位循环，不可用。）
 */

/**
 * @brief 外设电源脚初始化：默认驱动低（外设上电）
 */
void BSP_Periph_Power_Init(void);

/**
 * @brief 确保外设上电（= 把 PB7 驱动为低；幂等，可反复调）
 * @note  STOP 唤醒后先调它把外设域从 0V 拉回 3.3V + 延时等 POR，再重新 Init 外设。
 */
void BSP_Periph_Power_On(void);

/**
 * @brief 把外设电源脚配成"输入+内部上拉"(~40k 弱上拉被动拉高栅极)
 * @note  STOP 睡前调它：内部弱上拉把 P-MOS 栅极拉高，外设域关断到 0V（几 uA）。
 *        唤醒后必须先 BSP_Periph_Power_On() 恢复 + 延时等 POR，再重新 Init 外设。
 */
void BSP_Periph_Power_PullUp(void);

/**
 * @brief 读 PB7 的实际电平（临时配数字输入读一次，读完恢复驱动低）
 * @retval 0 = 低电平；1 = 高电平
 */
uint8_t BSP_Periph_PowerPeek(void);

/* 【外设电源档位】
 *   1 = 驱动【低】 ← 【默认，外设上电 3.3V，醒着用】
 *   2 = 驱动【高】 —— 实测会整板 POR/BOR 复位循环，不要用
 *  STOP 期间统一走 BSP_Periph_Power_PullUp()（输入上拉，几 uA）。 */
#ifndef PERIPH_PWR_FORCE
#define PERIPH_PWR_FORCE    1
#endif

#endif
