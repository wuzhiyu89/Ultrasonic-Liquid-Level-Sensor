/**
 * @file app_lowpower.h
 * @brief 应用层低功耗：STOP 进出 + RTC 周期唤醒
 * @note  可用唤醒源只有 RTC WakeUp(EXTI20，周期) 与探针 EXTI1(有水)。
 *        ⚠️ 不能靠 DIO1(PC15) 唤醒：睡前被 BSP_GPIO_SetAllAnalog() 设成模拟、收不到边沿。
 *        STOP 唤醒后由 SystemClock_Config() 重配时钟（只开 HSI、不开 PLL），
 *        并重新使能 TIM3 和日志串口。
 */
#ifndef __APP_LOWPOWER_H__
#define __APP_LOWPOWER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 低功耗模块初始化，打开PWR时钟
 */
void app_lowpower_init(void);

/**
 * @brief 配置RTC WakeUp周期唤醒间隔
 * @param sec 唤醒间隔秒，0关闭；最大65535
 * @note  使用RTC SPRE(1Hz)时钟，计数单位为秒
 */
void app_lowpower_config_wakeup(uint32_t sec);

/**
 * @brief 进入 STOP（阻塞），唤醒后自动恢复时钟
 * @note  睡前 PB7 输入上拉关断外设域（几 uA）；唤醒后调用者必须
 *        BSP_Periph_Power_On() 拉回 3.3V + 延时等 POR，再重新 Init 外设。
 *        调用前先 app_lowpower_config_wakeup() 设好 RTC 唤醒间隔。
 */
void app_lowpower_enter_stop_pullup(void);

#ifdef __cplusplus
}
#endif
#endif /* __APP_LOWPOWER_H__ */
