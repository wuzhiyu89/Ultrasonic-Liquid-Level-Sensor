/**
 * @file app_lowpower.h
 * @brief 应用层低功耗管理：STOP模式进出 + RTC周期唤醒
 * @note  唤醒源：RTC WakeUp(周期检测) / DIO1 EXTI(LoRa收到下行帧) / 探针 EXTI(有水)
 *        STOP唤醒后自动恢复系统时钟(PLL 48MHz)和TIM3
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
 * @brief 进入STOP模式（阻塞），唤醒后自动恢复时钟
 * @note  睡前 PB7 配输入上拉（方案B）：内部 ~40k 弱上拉把 P-MOS 栅极拉高，
 *        外设域关断到 0V（几 uA）。唤醒后【调用者】必须先 BSP_Periph_Power_On()
 *        把外设域拉回 3.3V + 延时（APP_PERIPH_POR_MS）等外设 POR，再重新 Init 外设。
 *        调用前需先 app_lowpower_config_wakeup() 设好 RTC 唤醒间隔。
 */
void app_lowpower_enter_stop_pullup(void);

#ifdef __cplusplus
}
#endif
#endif /* __APP_LOWPOWER_H__ */
