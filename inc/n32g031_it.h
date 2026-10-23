/**
 * @file n32g031_it.h
 * @brief 中断向量声明（实现见 src/n32g031_it.c）
 */
#ifndef __N32G031_IT_H__
#define __N32G031_IT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "n32g031.h"

void NMI_Handler(void);
void HardFault_Handler(void);
void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __N32G031_IT_H__ */
