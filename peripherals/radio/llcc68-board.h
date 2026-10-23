/*!
 * \file      LLCC68-board.h
 *
 * \brief     Target board LLCC68 driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#ifndef __LLCC68_BOARD_H__
#define __LLCC68_BOARD_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "LLCC68/LLCC68.h"
#include "llcc68.h"

// 临界区宏：HAVE_OS=1 时用 FreeRTOS 的 vPortEnter/ExitCritical，否则编译成空操作
#if HAVE_OS
	#define CRITICAL_SECTION_BEGIN( )	vPortEnterCritical()
	#define CRITICAL_SECTION_END( )	vPortExitCritical()
#else
	#define CRITICAL_SECTION_BEGIN( )
	#define CRITICAL_SECTION_END( )
#endif

/*!
 * \brief Initializes the radio I/Os pins interface
 */
void LLCC68IoInit( void );

/*!
 * \brief Initializes DIO IRQ handlers
 *
 * \param [IN] irqHandlers Array containing the IRQ callback functions
 */
void LLCC68IoIrqInit( DioIrqHandler dioIrq );

/*!
 * \brief De-initializes the radio I/Os pins interface.
 *
 * \remark Useful when going in MCU low power modes
 */
void LLCC68IoDeInit( void );

/*!
 * \brief Initializes the radio debug pins.
 */
void LLCC68IoDbgInit( void );

/*!
 * \brief HW Reset of the radio
 */
void LLCC68ResetInit( void ); 
/*!
 * \brief Blocking loop to wait while the Busy pin in high
 *
 * \remark 内部有硬超时(50ms)：模组没供电/坏掉/SPI 断时不会永久卡死，
 *         超时后直接返回并让 LLCC68GetBusyTimeoutCount() 计数 +1
 */
void LLCC68WaitOnBusy( void );

/*!
 * \brief BUSY 等待超时的累计次数(只增不减)
 * \retval 0 = 从没超时(射频正常响应)；非 0 = 至少发生过一次
 * \note  上层用"这次读到的值比上次大"判定"射频无响应"，
 *        见 app_lora.c 的 LoraApp_TakeBusyFault()
 */
uint16_t LLCC68GetBusyTimeoutCount( void );

/*!
 * \brief Checks if the given RF frequency is supported by the hardware
 *
 * \param [IN] frequency RF frequency to be checked
 * \retval isSupported [true: supported, false: unsupported]
 */
bool LLCC68CheckRfFrequency( uint32_t frequency );

void LLCC68DelayMs(uint32_t ms);

/*
 * 收发超时定时器(TIM3)初始化：实现在 LLCC68N32G031-board.c（节拍 = 1ms）
 */
void LLCC68TimerInit(void);
void LLCC68SetTxTimerValue(uint32_t nMs);
void LLCC68TxTimerStart(void);
void LLCC68TxTimerStop(void);
void LLCC68SetRxTimerValue(uint32_t nMs);
void LLCC68RxTimerStart(void);
void LLCC68RxTimerStop(void);

void LLCC68SetNss(uint8_t lev );
uint8_t LLCC68SpiInOut(uint8_t data);

/*!
 * \brief 获取TIM3的自由运行节拍(HSI时基)，用于非阻塞计时
 * \note  单位 = **1ms**：Prescaler/Period 与 SystemCoreClock 同源
 *        （SystemCoreClock = SYSCLK_FREQ = 8e6，见 LLCC68TimerInit）；
 *        实测 0x04 配对帧空口间隔 4.96s（配置 5s）可佐证。
 * \note  ⚠️ 前提是工程定义了 SYSCLK_FREQ=8000000；若换到没定义它的工程
 *        （如 EWARM 工程没写 Define），SystemCoreClock 会落到默认 48e6，
 *        那时节拍才会变成 6ms。
 * \note  STOP睡眠期间TIM3停止计数，只能用于醒着时的时间差
 */
uint32_t LLCC68GetTimerCount( void );

/*!
 * Radio hardware and global parameters
 */
extern LLCC68_t LLCC68;

#ifdef __cplusplus
}
#endif

#endif // __LLCC68_BOARD_H__
