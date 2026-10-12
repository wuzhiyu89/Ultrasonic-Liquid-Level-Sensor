#ifndef __APP_TIME_H
#define __APP_TIME_H

#include "bsp_rtc.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 中断回调，给n32g031_it.c调用 */
void app_time_rtc_wakeup_cb(void);
void app_time_rtc_alarm_a_cb(void);

/**
 * @brief app_time模块初始化
 * @retval 0成功，非0失败
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
 * RTC 里存的是【UTC】unix 时间戳（0x01 校时时把网关给的北京时间 -8h 存进去）。
 * 但协议三处的口径都是【北京时间】：
 *   ① 0x01 下行帧的 [年 月 日 周 时 分] —— 网关用 4G 的 AT+CCLK? 补的本地时间；
 *   ② 0x02 上行帧的 [月 日 时 分] —— 网关 app/protocol/lora_business.c 里明确注释
 *      "← 北京时间"，且网关取不到 4G 时间时会【直接采用设备报的这 4 个字节】；
 *   ③ 串口日志（给人看，当然要本地时间）。
 * ⚠️ 2026-10-09 修的 bug：上报帧原先是直接填 RTC 日历(=UTC)，整整差 8 小时。
 *    凡是往协议字段/日志里填时间，一律用 app_time_get_bj_calendar()，别直接拿 RTC 日历。
 * 改时区只改这一个宏。 */
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
 * @brief 在main while(1)循环调用，处理RTC中断产生的事件
 */
void app_time_process(void);

/**
 * @brief 取出并清除RTC WakeUp唤醒事件标志
 * @retval true 表示本次唤醒由RTC周期定时器触发（检测周期到达）
 * @note   主循环调用，读后即清；用于区分RTC唤醒和LoRa DIO1唤醒
 */
bool app_time_take_wakeup_event(void);

/**
 * @brief 置回RTC WakeUp唤醒事件标志
 * @note  供LoRa接收窗口等"临时消费事件后退避"的场景使用，
 *        避免事件被吞导致该轮检测丢失
 */
void app_time_set_wakeup_event(void);

#ifdef __cplusplus
}
#endif

#endif
