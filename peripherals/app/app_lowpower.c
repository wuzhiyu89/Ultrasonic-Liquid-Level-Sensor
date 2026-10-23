/**
 * @file app_lowpower.c
 * @brief 应用层低功耗：STOP 模式 + RTC 周期唤醒
 *
 * 睡前四步，随后 WFI 等待唤醒：
 *   1) 全部 GPIO 设模拟（消 IO 漏电），PB7 单独处理
 *   2) PA9(TRIG) 钉低（外设断电后悬空会被噪声误触发）
 *   3) PB7 输入上拉 -> P-MOS 栅极被 ~40k 弱上拉抬高，外设域关断到 0V（几 uA）
 *   4) 关停 STOP 下不需要的外设时钟
 * 唤醒后恢复系统时钟、GPIO 时钟、TIM3 与日志串口；业务外设（射频/液位模块）
 * 由调用者每轮先 BSP_Periph_Power_On() 拉回 3.3V + 等 POR，再重新初始化。
 */
#include "app_lowpower.h"
#include "bsp_lowpower.h"
#include "bsp_rtc.h"
#include "bsp_gpio.h"
#include "level_uart.h"
#include "water_cu.h"
#include "log.h"
#include "n32g031_rcc.h"
#include "n32g031_tim.h"

/* 本文件不引入 main.h，只声明需要的这一个函数 */
extern void SystemClock_Config(void);

void app_lowpower_init(void)
{
    BSP_LowPower_Init();
}

void app_lowpower_config_wakeup(uint32_t sec)
{
    bsp_rtc_clr_wakeup_flag();
    bsp_rtc_wakeup_config(sec);
}

/**
 * @brief 进入 STOP（阻塞），唤醒后自动恢复时钟
 * @note  睡前 PB7 输入上拉关断外设域；唤醒后【调用者】必须先 BSP_Periph_Power_On()
 *        拉回 3.3V + 延时等 POR，再重新 Init 外设。
 *        唤醒源只有 RTC WakeUp(EXTI20) 与探针 EXTI1。
 */
void app_lowpower_enter_stop_pullup(void)
{
    /* 清 RTC 唤醒标志，避免残留标志让下一轮一进 STOP 就立刻返回 */
    bsp_rtc_clr_wakeup_flag();

    /* GPIO 全设模拟消漏电（PB7 例外，下面单独配上拉） */
    BSP_GPIO_SetAllAnalog();

    /* PA9 钉低；必须在关 GPIO 时钟之前，否则写不进寄存器 */
    Level_TrigHoldLow();

    /* PB7 输入上拉 -> 外设域关断 */
    BSP_Periph_Power_PullUp();

    /* 武装"等水"回路（双态休眠的 A 态）：PA7 推挽高 + PB1 输入下拉 + EXTI1 上升沿。
     * 只有"判过水且上次无水"才真武装，已湿/没判过水会自动撤防（见 water_cu.c）。
     * 探针两根脚是 MCU 自己的 IO，不挂 PB7 外设域，断电不影响。 */
    log_info("[lp] water-wake: %s\r\n",
             (WaterDetect_WakeArm(1U) != 0U)
                 ? "ARMED (PA7=drive-high, PB1=input + EXTI1 rising)"
                 : "off (wet / not scanned / disabled)");

    /* 关停 STOP 下不用的外设时钟：保留 PWR（STOP 控制）与 LSI（RTC 唤醒） */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_SPI1 | RCC_APB2_PERIPH_USART1 |
                            RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB |
                            RCC_APB2_PERIPH_GPIOC | RCC_APB2_PERIPH_GPIOF |
                            RCC_APB2_PERIPH_AFIO, DISABLE);
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_USART2 | RCC_APB1_PERIPH_TIM3, DISABLE);

    BSP_EnterLowPower(LOWPOWER_STOP);

    /* 唤醒后：STOP 退出会切回 HSI，必须重配时钟；TIM3 计数器已停，需重新使能 */
    SystemClock_Config();

    /* 恢复 GPIO 时钟（STOP 前被关掉） */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB |
                            RCC_APB2_PERIPH_GPIOC | RCC_APB2_PERIPH_GPIOF |
                            RCC_APB2_PERIPH_AFIO, ENABLE);

    /* TIM3 供 radio 收发超时管理使用 */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM3, ENABLE);
    TIM_Enable(TIM3, ENABLE);

    /* 日志串口恢复（业务外设由调用者 BSP_Periph_Power_On + 延时后重新 Init） */
    log_init();
}
