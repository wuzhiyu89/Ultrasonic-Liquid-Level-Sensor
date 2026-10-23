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
    uint8_t  month;     /* 1-12 */
    uint8_t  day;       /* 1-31 */
    uint8_t  week;      /* 1-7 周一~周日 */
    uint8_t  hour;      /* 0-23 */
    uint8_t  minute;    /* 0-59 */
    uint8_t  second;    /* 0-59 */
}bsp_rtc_calendar_t;

/**
 * @brief  RTC硬件初始化(LSI)，检测是否已经初始化过，避免电池供电下重复设置时间
 * @retval 0:成功  非0:失败
 * @note   init 里会记录"RTC 域这次有没有被复位"，见 bsp_rtc_domain_kept()
 */
uint8_t bsp_rtc_init(void);

/**
 * @brief  RTC 域本次上电是否"还在走"(CTRL.BAKP 标记还在)
 * @retval 1 = 保留(热复位/短掉电未破 RTC 域)：日历可信，不要再用 Flash 时间戳覆盖
 *         0 = 已被复位(冷启动)：日历是默认值，需 Flash 恢复或等网关校时
 * @note   须先调 bsp_rtc_init()（标记在 init 里读）
 */
uint8_t bsp_rtc_domain_kept(void);

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
 * @brief  配置 RTC-WakeUp 定时器，用于 Stop 模式周期唤醒；cnt = 周期【秒】(1~65535)，0 关闭
 * @note   内部按 cnt-1 写计数值（器件从 0 数到 cnt 才触发）
 * @note   ⚠️ LSI(~30kHz) 温漂 ±20%，cnt 不等于精确秒；中断需在 it.c 清标志，会唤醒 STOP
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

/* 【AlarmA 无接口】：唤醒只走 WakeUp(WUT/EXTI20) 与探针(PB1/EXTI1) */

#ifdef __cplusplus
}
#endif

#endif
