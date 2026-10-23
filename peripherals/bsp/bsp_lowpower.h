#ifndef __BSP_LOWPOWER_H
#define __BSP_LOWPOWER_H

#include "n32g031.h"

/*
 * N32G031 低功耗模式：
 * SLEEP：内核停止，外设继续运行
 * STOP ：电压调节器低功耗，SRAM/寄存器保持，普通 TIM 停止，可被 EXTI/LPTIM/RTC 唤醒
 *        本工程只用到两个唤醒源：
 *          - RTC WakeUp(EXTI20)：周期唤醒（检测间隔/上报点）
 *          - PB1 探针 EXTI1：水碰到电极时立刻唤醒（WaterDetect_WakeArm 在 SetAllAnalog 之后重新配）
 *        注意 PC15(DIO1) 被 BSP_GPIO_SetAllAnalog() 设成模拟，收不到边沿，不能唤醒。
 * PD   ：掉电待机，唤醒后芯片复位
*/

typedef enum
{
    LOWPOWER_SLEEP,
    LOWPOWER_STOP,
    LOWPOWER_PD
}LowPower_Mode_t;

void BSP_LowPower_Init(void);
void BSP_EnterLowPower(LowPower_Mode_t mode);
void BSP_GPIO_SetAllAnalog(void);
void BSP_EXTI_WakeupPinConfig(GPIO_Module* GPIOx,uint16_t pin,EXTI_TriggerType edge);

#endif
