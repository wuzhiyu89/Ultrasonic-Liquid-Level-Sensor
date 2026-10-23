#include "bsp_rtc.h"
#include "n32g031_rtc.h"
#include "n32g031_rcc.h"
#include "n32g031_pwr.h"
#include "n32g031_exti.h"
#include "misc.h"
#include <stddef.h>
/* SDK 无 RTC_BKPxR（0x4C~0x9C 均为 reserved），初始化标记走 RTC->CTRL 的 BAKP 位(bit18)，
 * 见 RTC_STORE_OPERATION_SET。以下两个宏当前无代码引用，保留占位。 */
#define RTC_BKP_FLAG        0xA5A5U
#define RTC_BKP_INDEX       0U

/* LSI 内部 RC 典型 30kHz：(128)*(234)=29952，逼近 30000，输出≈1Hz
 * ⚠️ LSI 温漂 24~36kHz，日历会跑慢/跑快，WakeUp 计数值不等于真实秒
 * 不启用 LSE：PC14/PC15 留给 LLCC68 LoRa 当普通 GPIO
 */
#define RTC_ASYNC_PRE   0x7FU   /* 异步预分频 128 */
#define RTC_SYNC_PRE    233U    /* 同步预分频 233；+1后234 */

/* RTC 域本次上电是否"还在走"（init 里按 CTRL.BAKP 标记判断）：
 *   1 = 保留（热复位：看门狗/软复位/NRST/短暂掉电未破 RTC 域）-> 日历可信
 *   0 = 已被复位（冷启动）-> 日历是默认值，需从 Flash 恢复或等网关校时
 * ⚠️ 置 1 时上层不要用 Flash 的"最近上报时刻"覆盖 RTC，否则时间会倒退最多 1 小时。
 */
static uint8_t s_rtcDomainKept = 0U;

uint8_t bsp_rtc_domain_kept(void)
{
    return s_rtcDomainKept;
}

/**
 * @brief  RTC 硬件初始化，用内部 LSI RC，不使用外部 LSE 晶振
 * @note 板子已定型：PC14/PC15 分配给 LoRa BUSY/DIO1，不能开 LSE；
 *       ⚠️ LSI 温漂大，不适合高精度日历时间戳，仅适合 Stop 模式唤醒。
 */
uint8_t bsp_rtc_init(void)
{
    RTC_InitType rtc_init_cfg;
    /* 使能PWR外设时钟 */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, ENABLE);

    /* 判断RTC是否已经初始化（RTC_CTRL.BAKP 位，RTC域不掉电则标记保留） */
    if(RTC_GetStoreOperation() == RTC_STORE_OPERATION_SET)
    {
        s_rtcDomainKept = 1U;               /* RTC 一直在走：日历可信，上层别用 Flash 覆盖它 */
        RCC_EnableRtcClk(ENABLE);
        if(RTC_WaitForSynchro() != SUCCESS)
        {
            return 1;
        }
        /* RTC已初始化，仍需配置NVIC优先级（VBAT保持时NVIC可能复位） */
        {
            NVIC_InitType nvic_cfg;
            nvic_cfg.NVIC_IRQChannel         = RTC_IRQn;
            nvic_cfg.NVIC_IRQChannelPriority = 3U;
            nvic_cfg.NVIC_IRQChannelCmd      = ENABLE;
            NVIC_Init(&nvic_cfg);
        }
        return 0;
    }

    /************************ LSI初始化，不操作LSE！！ ************************/
    RCC_EnableLsi(ENABLE);
    uint32_t cnt = 0U;
    while(RCC_GetFlagStatus(RCC_LSCTRL_FLAG_LSIRD) == RESET)
    {
        cnt ++;
        if(cnt > 0x1FFFFU)
        {
            return 1; /* LSI启动失败 */
        }
    }
    /* 选择LSI作为RTC时钟源 */
    RCC_ConfigRtcClk(RCC_RTCCLK_SRC_LSI);
    RCC_EnableRtcClk(ENABLE);

    RTC_StructInit(&rtc_init_cfg);
    rtc_init_cfg.RTC_HourFormat    = RTC_24HOUR_FORMAT;
    rtc_init_cfg.RTC_AsynchPrediv  = RTC_ASYNC_PRE;
    rtc_init_cfg.RTC_SynchPrediv   = RTC_SYNC_PRE;

    /* 初始化RTC外设，时间日期传NULL */
    if(RTC_ConfigCalendar(RTC_FORMAT_BIN, &rtc_init_cfg, NULL, NULL, DISABLE) != SUCCESS)
    {
        return 1;
    }
    /* 设置备份标记：RTC已经初始化 */
    RTC_ConfigDayLightSaving(0, RTC_STORE_OPERATION_SET);
    if(RTC_WaitForSynchro() != SUCCESS)
    {
        return 1;
    }
    /* 首次初始化：配置NVIC，RTC中断优先级设为最低3，仅置标志位不抢占关键外设 */
    {
        NVIC_InitType nvic_cfg;
        nvic_cfg.NVIC_IRQChannel         = RTC_IRQn;
        nvic_cfg.NVIC_IRQChannelPriority = 3U;
        nvic_cfg.NVIC_IRQChannelCmd      = ENABLE;
        NVIC_Init(&nvic_cfg);
    }
    return 0;
}

/**
 * @brief 设置日历时间
 * @warning LSI 源 RTC 时间会漂移，不适合高精度时间记录
 * @note 复用原有预分频配置调用 RTC_ConfigCalendar，避免动到 WakeUp/AlarmA 配置
 */
uint8_t bsp_rtc_set_calendar(const bsp_rtc_calendar_t *p_cal)
{
    RTC_DateType rtc_date;
    RTC_TimeType rtc_time;
    RTC_InitType rtc_init_cfg;

    if(p_cal == NULL)
    {
        return 1;
    }

    rtc_date.Year    = p_cal->year % 100U;   /* N32G031 RTC year仅0‑99，代表2000~2099 */
    rtc_date.Month   = p_cal->month;
    rtc_date.Date    = p_cal->day;
    rtc_date.WeekDay = p_cal->week;

    rtc_time.Hours   = p_cal->hour;
    rtc_time.Minutes = p_cal->minute;
    rtc_time.Seconds = p_cal->second;
    rtc_time.H12     = RTC_AM_H12;

    /* 复用全局已经使用的RTC参数，防止修改预分频，保护WakeUp时钟 */
    rtc_init_cfg.RTC_HourFormat   = RTC_24HOUR_FORMAT;
    rtc_init_cfg.RTC_AsynchPrediv = RTC_ASYNC_PRE;
    rtc_init_cfg.RTC_SynchPrediv  = RTC_SYNC_PRE;

    RTC_EnableWriteProtection(DISABLE);

    if(RTC_ConfigCalendar(RTC_FORMAT_BIN, &rtc_init_cfg, &rtc_date, &rtc_time, DISABLE) != SUCCESS)
    {
        RTC_EnableWriteProtection(ENABLE);
        return 1;
    }

    RTC_EnableWriteProtection(ENABLE);
    if(RTC_WaitForSynchro() != SUCCESS)
    {
        return 1;
    }
    return 0;
}

/**
 * @brief 读取日历
 * @warning LSI源RTC时间会漂移，不适合高精度时间记录
 */
void bsp_rtc_get_calendar(bsp_rtc_calendar_t *p_cal)
{
    RTC_DateType rtc_date;
    RTC_TimeType rtc_time;
    if(p_cal == NULL)
    {
        return;
    }
    /* 库要求顺序：先读Time，再读Date */
    RTC_GetTime(RTC_FORMAT_BIN, &rtc_time);
    RTC_GetDate(RTC_FORMAT_BIN, &rtc_date);
    p_cal->year    = 2000U + rtc_date.Year;
    p_cal->month   = rtc_date.Month;
    p_cal->day     = rtc_date.Date;
    p_cal->week    = rtc_date.WeekDay;
    p_cal->hour    = rtc_time.Hours;
    p_cal->minute  = rtc_time.Minutes;
    p_cal->second  = rtc_time.Seconds;
}

/**
 * @brief RTC WakeUp 周期唤醒定时器（Stop 唤醒源），cnt = 周期【秒】(1~65535)，0 = 关闭
 * @note  ⚠️ 改时钟源/计数值前必须先 RTC_EnableWakeUp(DISABLE)，否则 WUT 不计数；
 *        顺序：ConfigWakeUpClock -> SetWakeUpCounter -> ConfigInt -> EnableWakeUp。CK_SPRE=1Hz，LSI 有温漂。
 */
void bsp_rtc_wakeup_config(uint32_t cnt)
{
    EXTI_InitType exti_cfg;

    /* 先关WUT：只有WUT关闭时才能改时钟源和计数值 */
    (void)RTC_EnableWakeUp(DISABLE);

    if(cnt == 0U)
    {
        RTC_ConfigInt(RTC_INT_WUT, DISABLE);
        return;
    }

    RTC_ConfigWakeUpClock(RTC_WKUPCLK_CK_SPRE_16BITS);
    RTC_SetWakeUpCounter(cnt - 1U);         /* 周期 = 计数值+1 秒 -> 这里 -1 才是 cnt 秒 */
    RTC_ConfigInt(RTC_INT_WUT, ENABLE);
    (void)RTC_EnableWakeUp(ENABLE);

    /* RTC Wakeup 内部连到 EXTI Line20，STOP唤醒必须使能该EXTI线 */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);
    EXTI_InitStruct(&exti_cfg);
    exti_cfg.EXTI_Line    = EXTI_LINE20;
    exti_cfg.EXTI_Mode    = EXTI_Mode_Interrupt;
    exti_cfg.EXTI_Trigger = EXTI_Trigger_Rising;
    exti_cfg.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&exti_cfg);

    EXTI->EMASK |= EXTI_LINE20;
    
    /* 清掉可能残留的挂起位，避免刚进低功耗就被"假唤醒" */
    EXTI_ClrITPendBit(EXTI_LINE20);
}

FlagStatus bsp_rtc_get_wakeup_flag(void)
{
    return RTC_GetFlagStatus(RTC_FLAG_WTF);
}

void bsp_rtc_clr_wakeup_flag(void)
{
    RTC_ClrFlag(RTC_FLAG_WTF);
}

/* 唤醒只走 RTC WakeUp(WUT/EXTI Line20) 与探针 PB1(EXTI Line1)，不用 AlarmA(EXTI Line17)。
 * n32g031_it.c 的 RTC_IRQHandler 仍清 ALRA 标志，用于防止硬件置位导致中断反复进入。 */
