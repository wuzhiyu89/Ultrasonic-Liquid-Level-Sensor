#include "app_time.h"
#include "bsp_rtc.h"
#include "app_dev_param.h"
#include "time_convert.h"
#include "log.h"
#include <string.h>

/* Flash写入最小间隔（秒），防止频繁擦写导致磨损 */
#define FLASH_SAVE_MIN_INTERVAL_S  3600U

static uint32_t s_last_saved_ts = 0U;
/* 全局中断标志，volatile 防止编译器优化 */
volatile uint8_t g_app_time_wakeup_flag = 0U;

/* 中断回调，仅置标志，中断上下文 */
void app_time_rtc_wakeup_cb(void)
{
    g_app_time_wakeup_flag = 1U;
}

/* 本地有效性标记：是否已经完成有效校时 */
static uint8_t s_rtc_time_valid = 0U;
uint8_t app_time_init(void)
{
    uint8_t ret = bsp_rtc_init();
    if (ret != 0U)
    {
        s_rtc_time_valid = 0U;
        return ret;
    }

    /* ---- ① RTC 域没掉电（热复位：NVIC_SystemReset / NRST / 短暂掉电）----
     * RTC 一直在走，日历就是准的，【绝对不能】再用 Flash 时间戳覆盖：
     * 那个时间戳节流到 1 小时才写一次，用它重设最多把时间往回拨 1 小时，
     * 上报帧时间会倒退、"距上次上报多久"也会算错。 */
    if (bsp_rtc_domain_kept() != 0U)
    {
        bsp_rtc_calendar_t cal;

        bsp_rtc_get_calendar(&cal);
        /* 日历合理性：从没校时过的芯片是 2000-01-01（year=2000 < 2020） */
        if ((cal.year >= 2020U) && (cal.month >= 1U) && (cal.month <= 12U) && (cal.day >= 1U))
        {
            s_rtc_time_valid = 1U;
            return 0U;
        }
        s_rtc_time_valid = 0U;          /* 域还在但还没校过时 -> 等网关 0x01 */
        return 1U;
    }

    /* ---- ② 冷启动（RTC 域已复位）：从 Flash 恢复上次校时结果 ----
     * 可能是几小时前的旧值（甚至电池换过），但比 2000-01-01 好得多；
     * 网关下一次 0x01 会把它校正。 */
    {
        uint32_t persist_ts = app_dev_param_get_persist_unix_ts();

        if (persist_ts >= 946684800U && persist_ts <= 4102444799U)
        {
            ret = app_time_set_unix_ts(persist_ts);
            if (ret == 0U)
            {
                log_info("[time] cold boot: restored ts=%lu from flash\r\n",
                         (unsigned long)persist_ts);
                return 0U;
            }
        }
    }
    /* RTC硬件初始化成功，但未恢复有效时间（Flash无记录或时间范围异常） */
    s_rtc_time_valid = 0U;
    return 1U;
}

uint8_t app_time_set_unix_ts(uint32_t ts)
{
    bsp_rtc_calendar_t cal;
    if (app_time_unix_to_calendar(ts, &cal) != 0U)
    {
        return 1U;
    }
    uint8_t ret = bsp_rtc_set_calendar(&cal);
    if (ret == 0U)
    {
        s_rtc_time_valid = 1U;
    }
    return ret;
}

uint32_t app_time_get_unix_ts(void)
{
    bsp_rtc_calendar_t cal;
    bsp_rtc_get_calendar(&cal);
    return app_time_calendar_to_unix(&cal);
}

void app_time_get_calendar(bsp_rtc_calendar_t *p_cal)
{
    if (p_cal == NULL)
        return;
    bsp_rtc_get_calendar(p_cal);
}

/**
 * @brief 取【北京时间】日历（RTC 内部是 UTC，这里 +8h）—— 见 app_time.h 的时区说明
 * @retval 0 成功；非0 失败（空指针 / RTC 没校时）
 */
uint8_t app_time_get_bj_calendar(bsp_rtc_calendar_t *p_cal)
{
    if ((p_cal == NULL) || (app_time_is_valid() == 0U))
    {
        return 1U;      /* 没校时就不给时间，别把 RTC 默认的 2000-01-01 填进协议 */
    }
    return app_time_unix_to_calendar(app_time_get_unix_ts() + APP_TIME_BJ_OFFSET_S, p_cal);
}

/**
 * @brief unix时间戳转日历，外层封装，调用公共工具函数
 */
uint8_t app_time_unix_to_calendar(uint32_t ts, bsp_rtc_calendar_t *p_cal)
{
    /* time_convert:0成功，1空指针，2时间越界；映射为本模块0成功/1失败 */
    uint8_t r = time_convert_unix_to_calendar(ts, p_cal);
    if(r == 0U)
    {
        return 0U;
    }
    return 1U;
}

/**
 * @brief 日历转unix时间戳，外层封装，调用公共工具函数
 */
uint32_t app_time_calendar_to_unix(const bsp_rtc_calendar_t *p_cal)
{
    return time_convert_calendar_to_unix(p_cal);
}

void app_time_save_to_flash(void)
{
    uint32_t now_ts = app_time_get_unix_ts();

    /* 时间未校准则不写Flash */
    if (s_rtc_time_valid == 0U)
    {
        return;
    }
    /* 节流：距离上次写入不足 1 小时(FLASH_SAVE_MIN_INTERVAL_S=3600s) 则跳过 */
    if (s_last_saved_ts != 0U && (now_ts - s_last_saved_ts) < FLASH_SAVE_MIN_INTERVAL_S)
    {
        return;
    }
    s_last_saved_ts = now_ts;
    (void)app_dev_param_set_persist_unix_ts(now_ts);
}

uint8_t app_time_is_valid(void)
{
    return s_rtc_time_valid;
}

bool app_time_take_wakeup_event(void)
{
    bool result = false;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (g_app_time_wakeup_flag != 0U)
    {
        g_app_time_wakeup_flag = 0U;
        result = true;
    }

    if (primask == 0U)
    {
        __enable_irq();
    }
    return result;
}