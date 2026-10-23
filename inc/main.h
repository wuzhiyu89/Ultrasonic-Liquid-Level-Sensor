/**
 * @file main.h
 * @brief 系统时钟配置声明
 */
#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "n32g031.h"

/**
 * @brief 系统时钟配置：只用 HSI(8MHz) 当 SYSCLK，不开 PLL
 * @note  上电启动调用一次；STOP 唤醒后 app_lowpower.c 会再调一次
 *        分频全为 DIV1：SYSCLK=HCLK=PCLK1=PCLK2=8MHz，FLASH_LATENCY_0
 */
void SystemClock_Config(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */
