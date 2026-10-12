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

//������˳��ٽ�����������
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
 */
void LLCC68WaitOnBusy( void );

/*!
 * \brief Checks if the given RF frequency is supported by the hardware
 *
 * \param [IN] frequency RF frequency to be checked
 * \retval isSupported [true: supported, false: unsupported]
 */
bool LLCC68CheckRfFrequency( uint32_t frequency );

void LLCC68DelayMs(uint32_t ms);

/*
 * ��ʱ����ʼ�� 
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
 * \brief STOP唤醒后恢复板级GPIO/SPI/EXTI配置，不触发射频复位
 */
void LLCC68IoRecover( void );

/*!
 * \brief 获取TIM3的1ms自由运行节拍(HSI时基)，用于非阻塞计时
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
