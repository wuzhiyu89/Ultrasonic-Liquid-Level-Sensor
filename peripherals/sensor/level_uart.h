#ifndef __LEVEL_UART_H
#define __LEVEL_UART_H
#include "n32g031.h"

/*
 * 超声波液位模块（UART 型）
 *   MCU:  TRIG = PA9（普通 GPIO 输出，高脉冲后下降沿触发一次测量）
 *         模块TX -> PA10 = USART1_RX（复用 AF4！不是 USART2）
 *   帧格式（常见于 AJ-SR04M / DYP 类模块，可在本文件顶部状态机里改）：
 *         0xFF + 距离高字节 + 距离低字节 + 校验(前三字节之和的低8位)，单位 mm
 *
 * 注意：日志 TX 在 PB6 = USART1_TX(AF4)，与本接收共用 USART1，
 *       所以两者波特率相同。读液位时临时切到模块波特率，读完切回 LOG_BAUD 打印。
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
 * @brief 触发一次液位测量，等待一帧新的有效数据
 * @param timeout_ms 最长等待毫秒
 * @retval 0~65535 液位mm；0XFFFF 超时/无有效数据
 * @note  触发脚PA9先拉高再拉低(下降沿触发)，两次触发间隔必须大于1s
 */
uint16_t Level_Measure(uint32_t timeout_ms);

/**
 * @brief 进STOP前把触发脚(PA9)钉在确定低电平，进低功耗前必须调用
 * @note  【复位根因】超声模块由 PB7 域常供电；而 BSP_GPIO_SetAllAnalog() 会把 PA9
 *        也设成模拟(悬空)，TRIG 一悬空模块就可能被噪声误触发、反复测距，
 *        每次测距都是大电流脉冲 -> 电源跌落 -> RST_CAUSE=POR/BOR 复位。
 *        睡前把 TRIG 保持为"输出低"(模块的空闲电平)就不会误触发。
 */
void Level_TrigHoldLow(void);

/* ==================== 型号未知时的调试工具 ==================== */

/**
 * @brief 取走抓到的原始字节（取走后清空）
 * @param buf 输出缓存
 * @param max 缓存大小
 * @retval 实际字节数
 * @note  收到的每个字节都会存进 64 字节的抓包缓冲，和帧格式无关；
 *        用它可以看模块到底回了什么、反推波特率与协议。
 */
uint16_t Level_RawTake(uint8_t *buf, uint16_t max);

/**
 * @brief 丢弃已抓到的原始字节
 */
void Level_RawClear(void);

/**
 * @brief 切换接收波特率（USART1 收发共用，日志波特率会跟着变）
 */
void Level_UartSetBaud(uint32_t baudrate);

//====给外部中断文件调用====
void Level_UART_RxByte(uint8_t ch);


void Level_UART_TimeoutPoll(void);

#endif
