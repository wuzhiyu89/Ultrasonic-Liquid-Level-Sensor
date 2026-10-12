#include "time_convert.h"
#include <stddef.h>

/* 闰年判断：返回1=闰年，0=平年 */
static uint8_t is_leap_year(uint16_t year)
{
    if ((year % 400U) == 0U)
        return 1U;
    if ((year % 100U) == 0U)
        return 0U;
    if ((year % 4U) == 0U)
        return 1U;
    return 0U;
}

/* 平年每个月天数，下标0不用，1‑12对应月份 */
static const uint8_t g_month_days[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

/**
 * @brief 计算星期：输入从1970‑01‑01起的总天数；返回1‑7(周一=1，周日=7)
 * @note 1970‑01‑01 是星期四
 */
static uint8_t calc_week_from_days(uint32_t total_days)
{
    /* 1970‑01‑01 星期四，距离该日的天数 */
    uint32_t day_of_week = (4U + total_days) % 7U;
    /* 0→周日(7), 1周一 … 6周六 */
    if(day_of_week == 0U)
    {
        return 7U;
    }
    return (uint8_t)day_of_week;
}

uint8_t time_convert_unix_to_calendar(uint32_t ts, bsp_rtc_calendar_t *out_cal)
{
    if(out_cal == NULL)
    {
        return 1U;
    }
    const uint32_t SEC_PER_DAY = 86400U;
    uint32_t days = ts / SEC_PER_DAY;
    uint32_t rem_sec = ts % SEC_PER_DAY;

    out_cal->hour   = (uint8_t)(rem_sec / 3600U);
    rem_sec         = rem_sec % 3600U;
    out_cal->minute = (uint8_t)(rem_sec / 60U);
    out_cal->second = (uint8_t)(rem_sec % 60U);

    /* 从1970年开始迭代 */
    uint16_t y = 1970U;
    while (y <= 2099U)
    {
        uint32_t y_days = is_leap_year(y) ? 366U : 365U;
        if (days >= y_days)
        {
            days -= y_days;
            y++;
        }
        else
        {
            break;
        }
    }
    if(y > 2099U)
    {
        return 2U;
    }
    out_cal->year = y;

    /* 月份 */
    uint8_t m;
    for (m = 1U; m <= 12U; m++)
    {
        uint8_t md = g_month_days[m];
        if (m == 2U && is_leap_year(y))
        {
            md = 29U;
        }
        if (days >= md)
        {
            days -= md;
        }
        else
        {
            break;
        }
    }
    out_cal->month = m;
    out_cal->day   = (uint8_t)(days + 1U);

    /* 计算星期 */
    out_cal->week = calc_week_from_days(ts / SEC_PER_DAY);

    return 0U;
}

uint32_t time_convert_calendar_to_unix(const bsp_rtc_calendar_t *in_cal)
{
    if(in_cal == NULL)
    {
        return 0U;
    }
    /* 简单范围校验 */
    if(in_cal->year < 1970U || in_cal->year > 2099U)
    {
        return 0U;
    }
    const uint32_t SEC_PER_DAY = 86400U;
    uint32_t total_days = 0U;

    /* 累加1970 ~ year‑1 的整年天数 */
    uint16_t y;
    for (y = 1970U; y < in_cal->year; y++)
    {
        total_days += is_leap_year(y) ? 366U : 365U;
    }

    /* 累加本年前面月份天数 */
    uint8_t m;
    for (m = 1U; m < in_cal->month; m++)
    {
        uint8_t md = g_month_days[m];
        if (m == 2U && is_leap_year(in_cal->year))
        {
            md = 29U;
        }
        total_days += md;
    }

    /* 累加本月已过天数 */
    total_days += (in_cal->day - 1U);

    uint32_t ts = total_days * SEC_PER_DAY;
    ts += (uint32_t)in_cal->hour * 3600U;
    ts += (uint32_t)in_cal->minute * 60U;
    ts += in_cal->second;

    return ts;
}
