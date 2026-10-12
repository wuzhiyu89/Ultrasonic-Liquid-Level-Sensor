#ifndef __BSP_RTC_H
#define __BSP_RTC_H

#include "n32g031.h"

#ifdef __cplusplus
extern "C" {
#endif

/* BSP对外时间结构体，封装芯片库RTC日期时间，给上层app使用 */
typedef struct
{
    uint16_t year;      /* 完整公历，如2026 */
    uint8_t  month;     /* 1‑12 */
    uint8_t  day;       /* 1‑31 */
    uint8_t  week;      /* 1‑7 周一~周日 */
    uint8_t  hour;      /* 0‑23 */
    uint8_t  minute;    /* 0‑59 */
    uint8_t  second;    /* 0‑59 */
}bsp_rtc_calendar_t;

/**
 * @brief  RTC硬件初始化(LSI)，检测是否已经初始化过，避免电池供电下重复设置时间
 * @retval 0:成功  非0:失败
 */
uint8_t bsp_rtc_init(void);

/**
 * @brief  设置RTC日历时间
 * @param  p_cal:日历指针
 * @retval 0成功，1失败
 */
uint8_t bsp_rtc_set_calendar(const bsp_rtc_calendar_t *p_cal);

/**
 * @brief  读取当前RTC日历
 * @param  p_cal 输出日历
 */
void bsp_rtc_get_calendar(bsp_rtc_calendar_t *p_cal);

/**
 * @brief  配置RTC‑WakeUp定时器，用于Stop模式周期性定时唤醒
 * @param  cnt: 唤醒间隔计数值（近似秒），0关闭WakeUp；最大65535
 * @note   LSI(~30kHz)存在±20%温漂，cnt≠精确秒；温度/电压变化会导致定时偏差
 * @note   中断需要用户在it.c中写RTC中断服务，清除标志；会唤醒STOP模式
 */
void bsp_rtc_wakeup_config(uint32_t cnt);

/**
 * @brief  获取WakeUp中断标志
 */
FlagStatus bsp_rtc_get_wakeup_flag(void);

/**
 * @brief 清除WakeUp中断标志
 */
void bsp_rtc_clr_wakeup_flag(void);

/**
 * @brief 设置Alarm A闹钟（绝对时刻闹钟）
 */
uint8_t bsp_rtc_set_alarm_a(const bsp_rtc_calendar_t *p_alarm);

#ifdef __cplusplus
}
#endif

#endif
