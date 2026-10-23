/**
 * @file n32g031_it.c
 * @brief 中断向量实现：HardFault 现场、探针 EXTI1、USART1(液位模块接收)、RTC 唤醒
 * @note  中断里只置标志/收字节，耗时的 ADC、打印、上报全部留给主循环。
 */
#include "n32g031_it.h"
#include "radio.h"
#include "project_config.h"
#include "LLCC68-board.h"
#include "level_uart.h"
#include "water_cu.h"           /* WaterDetect_WakeIsrHandler()：PB1 上升沿唤醒 */
#include "log.h"

/* noinit 段：上电不初始化，复位后内容保持，可用仿真器查看 */
__attribute__((noinit)) uint32_t hfault_sp;
__attribute__((noinit)) uint32_t hfault_pc;
__attribute__((noinit)) uint32_t hfault_xpsr;

/******************************************************************************/
/*                       Cortex-M0 内核异常                                    */
/******************************************************************************/

void NMI_Handler(void)
{
}

/**
 * @brief HardFault：把崩溃现场打出来再死循环
 * @note  汇编读取异常压栈帧（M0: [sp+24]=PC, [sp+28]=xPSR），否则现象只是"串口突然没输出"
 */
void HardFault_Handler(void)
{
    __asm volatile(
        "mov %0, sp\n"
        "ldr r1,[sp,#24]\n"
        "mov %1, r1\n"
        "ldr r2,[sp,#28]\n"
        "mov %2, r2\n"
        : "=r"(hfault_sp),"=r"(hfault_pc),"=r"(hfault_xpsr)
    );
    log_info("\r\n!! HARDFAULT !! pc=0x%08X xpsr=0x%08X sp=0x%08X\r\n",
             (unsigned)hfault_pc, (unsigned)hfault_xpsr, (unsigned)hfault_sp);
    while(1)
    {
    }
}

void SVC_Handler(void)
{
}

void PendSV_Handler(void)
{
}

/**
 * @brief SysTick
 * @note  delay.c 的延时用 COUNTFLAG 轮询且关闭了 TICKINT，不会进这里
 */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                       外设中断                                              */
/******************************************************************************/

/**
 * @brief EXTI Line0/1：PB1 = 探针检测脚（线 1）
 * @note  水碰到电极把 PB1 拉高 -> 上升沿 -> 把 STOP 里的 MCU 叫醒
 *        （见 water_cu.c 的 WaterDetect_WakeArm）。线 0 是 PA0(SPI1_SCK)，没配 EXTI。
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
 * @brief USART1：液位模块接收（模块 TX -> PA10 = USART1_RX / AF4）
 * @note  日志 TX 在 PB6 上也是 USART1，但那是发送方向，互不影响。
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
 * @brief USART2：本工程未使用
 */
void USART2_IRQHandler(void)
{
}

extern void app_time_rtc_wakeup_cb(void);

/* 诊断用：RTC 中断进入次数（没有读取者，调试时用仿真器看） */
volatile uint32_t g_rtc_irq_cnt = 0U;

/**
 * @brief RTC 唤醒中断（WUT 走 EXTI Line20）
 * @note  必须先清 EXTI Line20 挂起位，否则中断反复进入、WFI 永远回不来。
 */
void RTC_IRQHandler(void)
{
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
            /* AlarmA 全工程没有设置入口，正常不会进来；清标志防止反复进入 */
            RTC_ClrIntPendingBit(RTC_INT_ALRA);
        }
    }
}
