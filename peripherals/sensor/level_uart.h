#ifndef __LEVEL_UART_H
#define __LEVEL_UART_H
#include "n32g031.h"

/*
 * 超声波液位模块（UART 型，AJ-SR04M / DYP 类）
 *   MCU:  TRIG = PA9（普通 GPIO 输出，高脉冲后下降沿触发一次测量）
 *         模块TX -> PA10 = USART1_RX（复用 AF4！不是 USART2）
 *   帧格式：0xFF + 距离高字节 + 距离低字节 + 校验(前三字节之和的低8位)，单位 mm；
 *          输出的是【探头到液面的距离】，不是水深。
 *   ⚠️ 日志 TX 在 PB6 = USART1_TX(AF4)，与本接收共用 USART1，两者波特率相同：
 *      采数前切到模块波特率(9600)，采完必须 Level_UartSetBaud(LOG_BAUD) 切回日志波特率。
 *   模块挂在 PB7 外设域上；两次触发间隔必须 > 1s。
 */
#ifndef LEVEL_UART_BAUD
#define LEVEL_UART_BAUD     9600U       /* 模块波特率；未知型号时用扫描工具逐个试 */
#endif

/**
 * @brief 超声波液位模块初始化（PA10=USART1_RX + 触发脚 PA9）
 */
void Level_UART_Init(void);

/**
 * @brief 获取最新液位，单位mm
 * @retval 0~65535：有效液位(mm)；0XFFFF：无有效数据
 */
uint16_t Level_Get_Val(void);

/**
 * @brief 触发一次液位测量并等待一帧新数据；返回 0~65535 液位mm，0xFFFF 超时/无数据
 * @param timeout_ms 最长等待毫秒（PA9 先拉高再拉低触发，两次触发间隔必须大于 1s）
 */
uint16_t Level_Measure(uint32_t timeout_ms);

/**
 * @brief 进STOP前把触发脚(PA9)钉在低电平，进低功耗前必须调用
 * @note  TRIG 悬空会被噪声误触发 -> 测距电流脉冲 -> 电源跌落 -> POR/BOR 复位（细节见 level_uart.c）。
 */
void Level_TrigHoldLow(void);

/**
 * @brief 切换接收波特率（USART1 收发共用，日志波特率会跟着变）
 */
void Level_UartSetBaud(uint32_t baudrate);

//====给外部中断文件调用====
void Level_UART_RxByte(uint8_t ch);


void Level_UART_TimeoutPoll(void);

#endif
