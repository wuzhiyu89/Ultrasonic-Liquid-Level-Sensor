#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "n32g031.h"

/*
 * 液位模块 UART 接收：模块TX -> PA10 = USART1_RX（复用 AF4）
 *   【2026-09-21 更正】原来这里配的是 "PA10 = USART2_RX / AF10"，是错的。
 *   官方 USART/Printf 例程 inc/main.h 给出本芯片的权威映射：
 *       USART1: RxPin = PA10, TxPin = PA9,  AF = GPIO_AF4_USART1
 *       USART2: RxPin = PA3,  TxPin = PA2,  AF = GPIO_AF4_USART2
 *   即 PA10 属于 USART1。配错 AF/外设的结果是接收通路根本不存在 ——
 *   模块回什么数据都进不来（这就是"读不到液位"的根因）。
 *
 * 注意：日志 TX 在 PB6 = USART1_TX(AF4)，所以【PA10 的接收与日志共用 USART1】。
 *   一个 USART 只能有一个波特率：读液位时要临时切到模块的波特率，
 *   读完再切回日志波特率（LOG_BAUD）才能正常打印。见 BSP_LevelUart_SetBaud()。
 */

/**
 * @brief 初始化液位模块的接收通道（PA10=USART1_RX）+ 使能 RX 中断
 * @param baudrate 模块波特率
 */
void BSP_LevelUart_Init(uint32_t baudrate);

/**
 * @brief 运行时切换 USART1 波特率（收发都会跟着变）
 * @note  USART_Init 会清掉 RXDNEIE 和 UE，所以本函数内部会重新打开它们
 */
void BSP_LevelUart_SetBaud(uint32_t baudrate);

#endif
