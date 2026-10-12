#ifndef __BSP_LOWPOWER_H
#define __BSP_LOWPOWER_H

#include "n32g031.h"

/*
 * N32G031低功耗模式说明：
 * SLEEP    ：内核停止，外设继续运行
 * STOP     ：电压调节器低功耗，SRAM、寄存器保持；EXTI/LPTIM/RTC可唤醒；普通TIM停止
 * PD       ：PowerDown掉电待机，唤醒后芯片复位
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

/**
 * @brief 用LPTIM做STOP模式的周期唤醒（LPTIM挂在LSI上，STOP下仍在跑）
 * @param period_ms 唤醒周期(ms)
 * @note  周期上限：LSI≈30.4kHz、最大128分频时约276秒；
 *        超过上限会返回0表示配置失败（需要软件分频计数）
 * @retval 0 成功；1 周期太长/配置失败
 */
uint8_t BSP_LPTIM_WakeupInit(uint32_t period_ms);

/**
 * @brief 停止LPTIM周期唤醒
 */
void BSP_LPTIM_WakeupDeInit(void);

#endif
