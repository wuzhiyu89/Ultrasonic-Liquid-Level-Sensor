#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "n32g031.h"

/*
 * 液位模块 UART 接收：模块 TX -> PA10 = USART1_RX（复用 AF4）
 *   本芯片映射：USART1 = PA10(RX)/PA9(TX)/AF4；USART2 = PA3(RX)/PA2(TX)/AF4。
 *   ⚠️ PA10 属于 USART1，配到 USART2 上的话接收通路根本不存在（模块回的数据进不来）。
 *
 * 注意：日志 TX 在 PB6 = USART1_TX，所以【PA10 接收与日志共用 USART1】。
 *   一个 USART 只有一个波特率：读液位时临时切到模块波特率，读完必须切回 LOG_BAUD。
 */

/**
 * @brief 初始化液位模块的接收通道（PA10=USART1_RX）+ 使能 RX 中断
 * @param baudrate 模块波特率
 */
void BSP_LevelUart_Init(uint32_t baudrate);

/**
 * @brief 运行时切换 USART1 波特率（收发都会跟着变）
 * @note  本SDK的 USART_Init 只重写字长/校验/收发使能(CTRL1)、STOP位(CTRL2)、流控(CTRL3)，
 *        不会动 RXDNEIE/UE；函数内部仍显式重新打开一次，不依赖库实现细节
 */
void BSP_LevelUart_SetBaud(uint32_t baudrate);

#endif
