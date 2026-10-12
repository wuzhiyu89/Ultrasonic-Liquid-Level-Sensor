#include "bsp_rtc.h"
#include "n32g031_rtc.h"
#include "n32g031_rcc.h"
#include "n32g031_pwr.h"
#include "n32g031_exti.h"
#include "misc.h"
#include <stddef.h>
/* RTC备份寄存器标记，代替STM32 BKP，判断RTC是否已经初始化完成 */
#define RTC_BKP_FLAG        0xA5A5U
/* N32G031 RTC有RTC_BKP0R ~ RTC_BKP19R，选用0号备份寄存器 */
#define RTC_BKP_INDEX       0U

/* LSI内部RC，典型30kHz；(128)*(234)=29952，逼近30000，输出≈1Hz
 * ⚠️重要：LSI温漂24~36kHz，日历会跑慢/跑快，WakeUp计数值不等于真实秒！
 * 本方案不启用LSE，PC14/PC15保留作为普通GPIO给LLCC68 LoRa使用
 */
#define RTC_ASYNC_PRE   0x7FU   /* 异步预分频 128 */
#define RTC_SYNC_PRE    233U    /* 同步预分频 233；+1后234 */

/**
 * @brief  RTC硬件初始化 使用内部LSI RC振荡器，不使用外部LSE晶振
 * @note 板子已定型 PC14/PC15分配给LoRa BUSY/DIO1，不能开启LSE；
 *       ⚠️LSI温漂大，不适合需要高精度日历时间戳；仅适合Stop模式唤醒。
 */
uint8_t bsp_rtc_init(void)
{
    RTC_InitType rtc_init_cfg;
    /* 使能PWR外设时钟 */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, ENABLE);

    /* 判断RTC是否已经初始化（备份寄存器标记，VBAT不掉电则标记保留） */
    if(RTC_GetStoreOperation() == RTC_STORE_OPERATION_SET)
    {
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
 * @warning LSI源RTC时间会漂移，不适合高精度时间记录
 * @note 改用RTC_SetDate / RTC_SetTime；不再调用RTC_ConfigCalendar，避免清除WakeUp/AlarmA配置
 */
/**
 * @brief 设置日历时间
 * @warning LSI源RTC时间会漂移，不适合高精度时间记录
 * @note 保存原有RTC预分频配置再调用RTC_ConfigCalendar，保护WakeUp/AlarmA配置
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

    /* 使用ConfigCalendar，但传入原有分频配置，不会改动WakeUp的时钟源 */
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
 * @brief RTC WakeUp周期唤醒定时器，Stop模式唤醒源
 * @param cnt: 计数器数值；0关闭WakeUp
 * @note 唤醒周期 = (cnt+1) 秒（CK_SPRE 1Hz时钟）；LSI有温漂，不是精确秒
 * @note 【关键】改时钟源/计数值前必须先 RTC_EnableWakeUp(DISABLE)，
 *       否则配置不生效(WUT根本不计数的坑就是这里)；使能顺序按厂商例程：
 *       ConfigWakeUpClock -> SetWakeUpCounter -> ConfigInt -> EnableWakeUp
 * @note 同时使能 EXTI Line20(RTC Wakeup)：STOP模式要靠EXTI线把MCU唤醒
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
    RTC_SetWakeUpCounter(cnt);
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

/**
 * @brief 设置AlarmA绝对时刻闹钟（也能把MCU从STOP唤醒）
 * @param p_alarm 目标时刻
 * @note AlarmA 内部连到 EXTI Line17，STOP唤醒需要使能该EXTI线
 */
uint8_t bsp_rtc_set_alarm_a(const bsp_rtc_calendar_t *p_alarm)
{
    RTC_AlarmType alarm_cfg;
    EXTI_InitType  exti_cfg;

    if(p_alarm == NULL)
    {
        return 1;
    }
    RTC_AlarmStructInit(&alarm_cfg);
    alarm_cfg.AlarmTime.Hours    = p_alarm->hour;
    alarm_cfg.AlarmTime.Minutes  = p_alarm->minute;
    alarm_cfg.AlarmTime.Seconds  = p_alarm->second;
    alarm_cfg.AlarmTime.H12      = RTC_AM_H12;
    alarm_cfg.DateWeekMode    = RTC_ALARM_SEL_WEEKDAY_DATE;
    alarm_cfg.DateWeekValue   = p_alarm->day;
    alarm_cfg.AlarmMask       = RTC_ALARMMASK_NONE;

    RTC_EnableWriteProtection(DISABLE);
    RTC_EnableAlarm(RTC_A_ALARM, DISABLE);
    RTC_SetAlarm(RTC_FORMAT_BIN, RTC_A_ALARM, &alarm_cfg);
    RTC_EnableAlarm(RTC_A_ALARM, ENABLE);
    RTC_ConfigInt(RTC_INT_ALRA, ENABLE);
    RTC_EnableWriteProtection(ENABLE);

    /* RTC Alarm 内部连到 EXTI Line17 */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);
    EXTI_InitStruct(&exti_cfg);
    exti_cfg.EXTI_Line    = EXTI_LINE17;
    exti_cfg.EXTI_Mode    = EXTI_Mode_Interrupt;
    exti_cfg.EXTI_Trigger = EXTI_Trigger_Rising;
    exti_cfg.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&exti_cfg);

    return 0;
}
