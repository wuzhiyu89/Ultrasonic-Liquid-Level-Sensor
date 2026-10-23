#ifndef __APP_TIME_H
#define __APP_TIME_H

#include "bsp_rtc.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 中断回调，给n32g031_it.c调用 */
void app_time_rtc_wakeup_cb(void);

/**
 * @brief app_time模块初始化
 * @retval 0 成功（已经有可信时间：热复位保留的 RTC，或冷启动从 Flash 恢复）
 *         非0 失败（RTC 起不来 / 还没有可信时间，等网关 0x01 校时）
 * @note  热复位(RTC 域未掉电)时【不会】用 Flash 时间戳覆盖 RTC，
 *        否则配对/解绑后的复位会把时间往回拨最多 1 小时。见 bsp_rtc_domain_kept()。
 */
uint8_t app_time_init(void);

/**
 * @brief 设置系统时间，输入unix时间戳
 * @param ts unix UTC时间戳
 * @retval 0成功
 */
uint8_t app_time_set_unix_ts(uint32_t ts);

/**
 * @brief 获取当前unix UTC时间戳
 * @return unix时间戳
 */
uint32_t app_time_get_unix_ts(void);

/* ==================== 时区：设备内部 UTC，协议边界用【北京时间】 ====================
 * RTC 存的是 UTC（0x01 校时把北京时间 -8h 存进去），但协议字段（0x01 下行时间、
 * 0x02 上报的 月/日/时/分）和串口日志都是北京时间。
 * ⚠️ 凡是往协议字段/日志里填时间，一律用 app_time_get_bj_calendar()，别直接拿 RTC 日历
 *   （直接用会整整差 8 小时）。改时区只改这一个宏。 */
#define APP_TIME_BJ_OFFSET_S    (8U * 3600U)        /* 北京时间 = UTC + 8 小时 */

/**
 * @brief 取【北京时间】日历（= RTC 的 UTC 时间戳 + APP_TIME_BJ_OFFSET_S）
 * @param p_cal 输出日历（年 月 日 周 时 分 秒）
 * @retval 0 成功；非0 失败（指针为空，或 RTC 还没校时）
 * @note  RTC 未校时时返回非0，调用者自己决定退回什么（例如网关最近下发的原始北京时间），
 *        以免把 RTC 的默认值 2000-01-01 当成真时间填进协议。
 */
uint8_t app_time_get_bj_calendar(bsp_rtc_calendar_t *p_cal);

/**
 * @brief 获取日历时间
 * @param p_cal 输出日历
 */
void app_time_get_calendar(bsp_rtc_calendar_t *p_cal);

/**
 * @brief unix时间戳转日历
 * @param ts 输入时间戳
 * @param p_cal 输出日历
 * @return 0成功
 */
uint8_t app_time_unix_to_calendar(uint32_t ts, bsp_rtc_calendar_t *p_cal);

/**
 * @brief 日历转unix时间戳
 * @param p_cal 输入日历
 * @return unix时间戳
 */
uint32_t app_time_calendar_to_unix(const bsp_rtc_calendar_t *p_cal);

/**
 * @brief 将当前时间保存到flash storage(app_dev_param)
 */
void app_time_save_to_flash(void);

/**
 * @brief 判断当前RTC时间是否有效(是否完成校时)
 */
uint8_t app_time_is_valid(void);

/**
 * @brief 取出并清除 RTC WakeUp 唤醒事件标志
 * @retval true = 本次唤醒由 RTC 周期定时器触发（检测周期到达）
 * @note  主循环调用，读后即清。用它区分"RTC 周期唤醒"与探针 EXTI1 唤醒。
 */
bool app_time_take_wakeup_event(void);

#ifdef __cplusplus
}
#endif

#endif
