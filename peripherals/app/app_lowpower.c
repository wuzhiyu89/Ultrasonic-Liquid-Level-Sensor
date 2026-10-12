/**
 * @file app_lowpower.c
 * @brief 应用层低功耗管理：STOP 模式 + RTC 周期唤醒
 *
 * 睡前做三件事，然后 WFI 等 RTC 唤醒：
 *   1) 全部 GPIO 设模拟模式（消除 IO 漏电），PB7 外设电源脚除外
 *      —— PB7 由本函数配成输入上拉（方案B）：内部 ~40k 弱上拉把 P-MOS 栅极
 *         拉高，外设域关断到 0V（几 uA）。
 *   2) 液位模块触发脚 PA9 钉低（外设断电后 TRIG 悬空会被噪声误触发）
 *   3) 关掉 STOP 下不需要的外设时钟。
 * 唤醒后恢复系统时钟、GPIO 时钟、TIM3 与日志串口；
 * 业务外设（射频/液位模块）由调用者在每轮开始时先 BSP_Periph_Power_On()
 * 把外设域拉回 3.3V + 延时等 POR，再重新初始化。
 */
#include "app_lowpower.h"
#include "bsp_lowpower.h"
#include "bsp_rtc.h"
#include "bsp_gpio.h"           /* BSP_Periph_Power_PullUp()：睡前 PB7 输入上拉关断外设域 */
#include "level_uart.h"
#include "water_cu.h"           /* WaterDetect_WakeArm()：睡前武装"有水自动唤醒" */
#include "log.h"
#include "n32g031_rcc.h"
#include "n32g031_tim.h"

/*
 * 注意：不包含main.h，
 * 用extern声明替代。
 */
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
 * @brief 进入STOP模式（阻塞），唤醒后自动恢复时钟
 * @note  睡前 PB7 配输入上拉（方案B）：内部 ~40k 弱上拉被动把 P-MOS 栅极拉高，
 *        外设域关断到 0V（几 uA）。唤醒后【调用者】必须先 BSP_Periph_Power_On()
 *        把外设域拉回 3.3V + 延时（APP_PERIPH_POR_MS）等外设 POR，再重新 Init 外设。
 *        唤醒源：RTC WakeUp(周期检测) / DIO1 EXTI(LoRa收到下行帧) / 探针 EXTI(有水)。
 *        STOP唤醒后自动恢复系统时钟(PLL 48MHz)和TIM3。
 */
void app_lowpower_enter_stop_pullup(void)
{
    /* 清除RTC唤醒标志，确保下一轮唤醒正常触发 */
    bsp_rtc_clr_wakeup_flag();

    /*
     * 进入STOP前将GPIO设为模拟模式，消除IO漏电流
     * 模拟模式断开施密特触发器和上下拉电阻，漏电最低
     * （PB7 外设电源脚会被本函数跳过，下面单独配上拉；PA13/PA14 SWD 见 bsp_lowpower.c）
     */
    BSP_GPIO_SetAllAnalog();

    /* 液位模块触发脚(PA9)钉在确定的低电平；必须在关 GPIO 时钟之前调用 */
    Level_TrigHoldLow();

    /* 方案B：睡前 PB7 配输入上拉 —— 内部 ~40k 弱上拉被动把 P-MOS 栅极拉高，
     * 外设域关断到 0V（几 uA）。唤醒后调用者必须 BSP_Periph_Power_On() 恢复 + 延时等 POR。 */
    BSP_Periph_Power_PullUp();

    /* 【有水自动唤醒】把"等水来"的检测回路挂上：PA7 推挽输出高 + PB1 输入下拉 + EXTI1 上升沿。
     * 只有"判过水且上次是无水"才真武装（已湿/没判过水会自动撤防，见 water_cu.c），
     * 所以低功耗台架模式（不判水）不会因此多耗电。干态 0 电流，触水那一下被叫醒。
     * 探针两根脚(PA7/PB1)是 MCU 自己的 IO，不挂 PB7 外设域上，断电不影响它。 */
    log_info("[lp] water-wake: %s\r\n",
             (WaterDetect_WakeArm(1U) != 0U)
                 ? "ARMED (PA7=drive-high, PB1=pull-down + EXTI1 rising)"
                 : "off (not scanned / already wet / disabled)");

    /* 关掉STOP下不需要的外设时钟，减少外设漏电
     * 保留：PWR（STOP控制本身需要）、LSI（RTC唤醒需要）
     * 关掉：SPI1(LoRa)、USART1(log)、USART2(level)、TIM3(radio超时)
     *        GPIOA/B/C/F、AFIO（模拟模式配好后不再需要时钟，输出锁存器保持） */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_SPI1 | RCC_APB2_PERIPH_USART1 |
                            RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB |
                            RCC_APB2_PERIPH_GPIOC | RCC_APB2_PERIPH_GPIOF |
                            RCC_APB2_PERIPH_AFIO, DISABLE);
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_USART2 | RCC_APB1_PERIPH_TIM3, DISABLE);

    /* 进入STOP模式，WFI阻塞直到中断唤醒 */
    BSP_EnterLowPower(LOWPOWER_STOP);

    /*
     * === 唤醒后恢复 ===
     * STOP唤醒后硬件自动切回HSI，需要重新配置系统时钟；
     * 外设寄存器在STOP下保持，但TIM3计数器停止，需重新使能
     */
    SystemClock_Config();

    /* 重新使能GPIO时钟（STOP前全关了，现在恢复） */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB |
                            RCC_APB2_PERIPH_GPIOC | RCC_APB2_PERIPH_GPIOF |
                            RCC_APB2_PERIPH_AFIO, ENABLE);

    /* 重新使能TIM3，供radio收发超时管理使用 */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM3, ENABLE);
    TIM_Enable(TIM3, ENABLE);

    /* 日志串口恢复（业务外设由调用者在 BSP_Periph_Power_On + 延时后重新 Init） */
    log_init();
}
