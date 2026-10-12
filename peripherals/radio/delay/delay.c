#include "delay.h"
#include "misc.h"
#include "n32g031_rcc.h"

/*
 * 本文件按国民技术官方 N32H47x/48x 的 delay.c 写法改写。
 * 这份写法正好解决我们前面踩的两个坑：
 *
 *   坑1：用"SysTick 中断累加的节拍"计时
 *        -> 在 TIM3 中断里被间接调用时，SysTick 中断（优先级更低）进不来，
 *           节拍不增加 -> 在中断里死等。
 *        本写法的对策：DBG_SysTick_Config 里刻意清掉 TICKINT，延时过程
 *        完全不产生中断，所以放在任何上下文里都安全。
 *
 *   坑2：用"SysTick->VAL 差值"比较，目标取到 LOAD
 *        -> 差值最大只有 LOAD，且只有一个 tick 的窗口能满足，采样跨过去
 *           就永久等待。
 *        本写法的对策：用 COUNTFLAG(CTRL bit16) 轮询。COUNTFLAG 是锁存的
 *        ——计数到 0 就置 1，直到被读走才清，CPU 来晚了也不会错过。
 *
 * 另外计时基准每次用 RCC_GetClocksFreqValue() 现取寄存器真实值，
 * 不依赖编译期的 SystemCoreClock；用完把 SysTick 关掉。
 */

/**
 *\*\name    delay_systick_config.
 *\*\fun     配置 SysTick 一个计数周期为 ticks 个时钟，且不产生中断
 *\*\param   ticks :一个周期的时钟数
 *\*\return  0 成功，非 0 失败
**/
static uint32_t delay_systick_config(uint32_t ticks)
{
    if ((ticks == 0U) || (ticks > (SysTick_LOAD_RELOAD_Msk + 1U)))
    {
        return 1U;                                              /* 参数非法，不配置 */
    }

    SysTick->LOAD = (ticks & SysTick_LOAD_RELOAD_Msk) - 1U;     /* 设置重载寄存器 */
    SysTick->VAL  = 0U;                                         /* 清计数器和 COUNTFLAG */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    SysTick->CTRL &= (~SysTick_CTRL_TICKINT_Msk);               /* 不产生中断 */
    return 0U;
}

/**
 *\*\name    delay_wait_one_period.
 *\*\fun     等待一个 SysTick 计数周期
 *\*\note    COUNTFLAG 锁存：计数减到 0 时置 1，读到才清 0，不会错过
**/
static void delay_wait_one_period(void)
{
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0U)
    {
    }
}

/**
 *\*\name    delay_us.
 *\*\fun     微秒级延时
 *\*\param   nus :微秒数
**/
void delay_us(u32 nus)
{
    RCC_ClocksType clk;
    uint32_t i;

    if (nus == 0U)
    {
        return;
    }

    RCC_GetClocksFreqValue(&clk);
    if (clk.SysclkFreq == 0U)
    {
        return;
    }

    /* 1us 一个计数周期：8MHz -> ticks=8 -> LOAD=7 */
    if (delay_systick_config(clk.SysclkFreq / 1000000U) != 0U)
    {
        return;
    }

    for (i = 0U; i < nus; i++)
    {
        delay_wait_one_period();
    }

    SysTick->CTRL &= (~SysTick_CTRL_ENABLE_Msk);                /* 关闭 SysTick */
}

/**
 *\*\name    delay_ms.
 *\*\fun     毫秒级延时
 *\*\param   nms :毫秒数
**/
void delay_ms(u16 nms)
{
    RCC_ClocksType clk;
    uint32_t i;

    if (nms == 0U)
    {
        return;
    }

    RCC_GetClocksFreqValue(&clk);
    if (clk.SysclkFreq == 0U)
    {
        return;
    }

    /* 1ms 一个计数周期：8MHz -> ticks=8000 -> LOAD=7999 */
    if (delay_systick_config(clk.SysclkFreq / 1000U) != 0U)
    {
        return;
    }

    for (i = 0U; i < (uint32_t)nms; i++)
    {
        delay_wait_one_period();
    }

    SysTick->CTRL &= (~SysTick_CTRL_ENABLE_Msk);                /* 关闭 SysTick */
}

/* 与官方 N32H4xx delay.c 同名的接口（demo 里用的就是这两个名字），
 * 内部转调 delay_ms/delay_us，方便不同平台的驱动代码直接拿来移植 */
void systick_delay_ms(uint32_t ms)
{
    while (ms > 65535U)
    {
        delay_ms(65535U);
        ms -= 65535U;
    }
    delay_ms((u16)ms);
}

void systick_delay_us(uint32_t us)
{
    delay_us(us);
}
