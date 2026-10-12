#ifndef __TIME_CONVERT_H__
#define __TIME_CONVERT_H__
#include <stdint.h>
#include "bsp_rtc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief unix时间戳(UTC)转日历结构体bsp_rtc_calendar_t
 * @param ts unix时间戳
 * @param out_cal 输出日历结构体，不能为NULL
 * @retval 0:成功；1:入参NULL；2:时间超出支持范围(1970~2099)
 */
uint8_t time_convert_unix_to_calendar(uint32_t ts, bsp_rtc_calendar_t *out_cal);

/**
 * @brief 日历结构体bsp_rtc_calendar_t转unix时间戳(UTC)
 * @param in_cal 输入日历，不能为NULL
 * @return unix时间戳；输入非法返回0
 */
uint32_t time_convert_calendar_to_unix(const bsp_rtc_calendar_t *in_cal);

#ifdef __cplusplus
}
#endif
#endif
