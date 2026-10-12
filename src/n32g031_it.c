/*****************************************************************************
 * Copyright (c) 2019, ..\readme.txt Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * ..\readme.txt' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ..\readme.txt "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ..\readme.txt BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file n32g031_it.c
 * @author ..\readme.txt
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, ..\readme.txt Technologies Inc. All rights reserved.
 */
#include "n32g031_it.h"
#include "main.h"
#include "radio.h"
#include "project_config.h"
#include "LLCC68-board.h"
#include "level_uart.h"
#include "water_cu.h"           /* WaterDetect_WakeIsrHandler()：PB1 上升沿唤醒 */
#include "log.h"
#include "n32g031_lptim.h"

/* noinit段：上电不初始化，复位后main可以读取故障寄存器，仅调试使用 */
__attribute__((noinit)) uint32_t hfault_sp;
__attribute__((noinit)) uint32_t hfault_pc;
__attribute__((noinit)) uint32_t hfault_lr;


/** @addtogroup N32G031_StdPeriph_Template
 * @{
 */

extern uint8_t TxBuffer1[];
extern uint8_t TxBuffer2[];
extern uint8_t RxBuffer1[];
extern uint8_t RxBuffer2[];
extern __IO uint8_t TxCounter1;
extern __IO uint8_t TxCounter2;
extern __IO uint8_t RxCounter1;
extern __IO uint8_t RxCounter2;
extern uint8_t NbrOfDataToTransfer1;
extern uint8_t NbrOfDataToTransfer2;
extern uint8_t NbrOfDataToRead1;
extern uint8_t NbrOfDataToRead2;

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief  This function handles NMI exception.
 */
void NMI_Handler(void)
{
}

/**
 * @brief  This function handles Hard Fault exception.
 */
void HardFault_Handler(void)
{
    /* 汇编读取栈中：栈指针SP、出错PC地址(崩溃代码地址)、LR返回地址 */
    __asm volatile(
        "mov %0, sp\n"
        "ldr r1,[sp,#24]\n"
        "mov %1, r1\n"
        "ldr r2,[sp,#28]\n"
        "mov %2, r2\n"
        : "=r"(hfault_sp),"=r"(hfault_pc),"=r"(hfault_lr)
    );
    /* 把崩溃现场打出来，否则现象就是"串口突然没输出"，无法定位 */
    log_info("\r\n!! HARDFAULT !! pc=0x%08X xpsr=0x%08X sp=0x%08X\r\n",
             (unsigned)hfault_pc, (unsigned)hfault_lr, (unsigned)hfault_sp);
    /* 保存寄存器之后原地死循环；仿真器/串口在main中打印这几个变量定位故障 */
    while(1)
    {
    }
}


/**
 * @brief  This function handles SVCall exception.
 */
void SVC_Handler(void)
{
}

/**
 * @brief  This function handles PendSV_Handler exception.
 */
void PendSV_Handler(void)
{
}

/**
 * @brief  This function handles SysTick Handler.
 * @note   delay.c 的延时用 COUNTFLAG 轮询且关闭了 TICKINT，不会进这里
 */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 N32G031 Peripherals Interrupt Handlers                     */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_n32g031.s).                                                 */
/******************************************************************************/

/**
 * @brief  This function handles EXTI Line0/1 interrupt request.
 * @note   PB1 = 探针检测脚 -> EXTI 线 1：水碰到电极把 PB1 拉高 -> 上升沿
 *         -> 把 STOP 里的 MCU 叫醒（见 water_cu.c 的 WaterDetect_WakeArm）。
 *         线 0 是 PA0(SPI1_SCK)，本工程没给它配 EXTI，所以这里只管线 1。
 *         中断里只置标志，ADC/打印/上报全部留给主循环。
 */
void EXTI0_1_IRQHandler(void)
{
    if (EXTI_GetStatusFlag(EXTI_LINE1) != RESET)
    {
        EXTI_ClrITPendBit(EXTI_LINE1);
        WaterDetect_WakeIsrHandler();
    }
}

/**
 * @brief  This function handles USART1 global interrupt request.
 * @note   液位模块的接收走这里：模块TX -> PA10 = USART1_RX(AF4)。
 *         【2026-09-21 更正】这段代码原来在 USART2_IRQHandler 里，但 PA10 属于
 *         USART1（官方 USART/Printf 例程 inc/main.h 确认），挂在 USART2 上
 *         永远收不到数据 —— 这就是"读不到液位"的根因。
 *         日志 TX 在 PB6 上也是 USART1，但那是发送方向，互不影响。
 */
void USART1_IRQHandler(void)
{
    if(USART_GetIntStatus(USART1, USART_INT_RXDNE) != RESET)
    {
        USART_ClrIntPendingBit(USART1, USART_INT_RXDNE);
        uint8_t ch = (uint8_t)USART_ReceiveData(USART1);
        Level_UART_RxByte(ch);
    }
}

/**
 * @brief  This function handles USART2 global interrupt request.
 * @note   本工程未使用 USART2
 */
void USART2_IRQHandler(void)
{
}


extern void app_time_rtc_wakeup_cb(void);
extern void app_time_rtc_alarm_a_cb(void);

/* 诊断用：RTC中断进入次数，主循环打印出来看WUT到底有没有触发 */
volatile uint32_t g_rtc_irq_cnt = 0U;
/* LPTIM(LPTIM/TIM6共用向量)中断进入次数：STOP唤醒用 */
volatile uint32_t g_lptim_irq_cnt = 0U;
/* LPTIM中断事件标志：中断里只置标志 */
volatile uint8_t  g_lptim_tick = 0U;

/* LPTIM / TIM6 共用中断向量 */
void LPTIM_TIM6_IRQHandler(void)
{
    g_lptim_irq_cnt++;
    LPTIM_ClearFLAG_CMPM(LPTIM);      /* 与BSP里的比较匹配中断对应 */
    g_lptim_tick = 1U;
}

void RTC_IRQHandler(void)
{
    /* 必须先检查EXTI Line20状态并清除挂起位，否则中断死循环，WFI永远回不来 */
    if(EXTI_GetStatusFlag(EXTI_LINE20) != RESET)
    {
        EXTI_ClrITPendBit(EXTI_LINE20);
        g_rtc_irq_cnt++;

        if(RTC_GetITStatus(RTC_INT_WUT) != RESET)
        {
            RTC_ClrIntPendingBit(RTC_INT_WUT);
            app_time_rtc_wakeup_cb();
        }
        if(RTC_GetITStatus(RTC_INT_ALRA) != RESET)
        {
            RTC_ClrIntPendingBit(RTC_INT_ALRA);
            app_time_rtc_alarm_a_cb();
        }
    }
}


/**
 * @}
 */
