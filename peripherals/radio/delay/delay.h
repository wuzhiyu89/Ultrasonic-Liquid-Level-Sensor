#ifndef __DELAY_H
#define __DELAY_H
#include "n32g031.h"

/* 参考国民技术官方 delay.c 的写法：
 * 用 SysTick 的 COUNTFLAG 轮询，且刻意关闭 TICKINT（不产生中断），
 * 因此这两个延时在中断里调用也是安全的，且不会错过计数周期。 */
void delay_ms(u16 nms);
void delay_us(u32 nus);

/* 与官方 N32H4xx delay.c 同名的接口（demo 里用的是这两个名字），
 * 内部转调上面的 delay_ms/delay_us，便于不同平台的驱动代码直接移植 */
void systick_delay_ms(uint32_t ms);
void systick_delay_us(uint32_t us);

#endif
