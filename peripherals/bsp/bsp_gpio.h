#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include "n32g031.h"

/**
 * 【引脚角色】
 *   PB7 = 外设总电源使能（板上高边 P-MOS：低 = 上电，高 = 断电）
 *   PA7 = 探针激励脚（配 PB1 做检测），见 water_cu.c
 *
 * 【本板关键结论】
 *   驱动为高 → 整板 POR/BOR 复位循环，不要用
 *   释放高阻 → 外设域只有 1.2V 且整板 POR 复位，不要用
 *   驱动为低 → 外设域 3.3V ✅（醒着用这档，代价 ~1mA 偏置）
 *   输入上拉 → P-MOS 关断、外设域 0V ✅（STOP 用这档，几 uA）
 *
 * 【低功耗策略】STOP 前 BSP_Periph_Power_PullUp() 关断外设域；
 *   唤醒后 BSP_Periph_Power_On() 拉回 3.3V + 延时等 POR + 重新 Init 外设。
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

#endif
