#include "bsp_uart.h"
#include "misc.h"

/*
 * 液位模块 UART 接收（PA10 = USART1_RX, AF4）
 *
 * 【为什么是 USART1】官方 USART/Printf 例程 inc/main.h：
 *     USART1: RxPin=PA10 TxPin=PA9  AF=GPIO_AF4_USART1
 *     USART2: RxPin=PA3  TxPin=PA2  AF=GPIO_AF4_USART2
 *   PA10 只挂在 USART1 上，配成 USART2 时接收通路根本不存在。
 *
 * 引脚配置照官方例程：AF_PP + 上拉 + AF4。
 * GPIOA 的 PA9 仍由 level_uart.c 当普通 GPIO 输出做触发脉冲（日志 TX 在 PB6，
 * 没有占用 PA9 的 USART1_TX 复用），互不冲突。
 */

static void level_uart_pin_config(void)
{
    GPIO_InitType gpio;

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_AFIO, ENABLE);

    GPIO_InitStruct(&gpio);
    gpio.Pin            = GPIO_PIN_10;
    gpio.GPIO_Mode      = GPIO_MODE_AF_PP;      /* 与官方例程一致 */
    gpio.GPIO_Pull      = GPIO_PULL_UP;         /* 空闲为高 */
    gpio.GPIO_Speed     = GPIO_SPEED_HIGH;
    gpio.GPIO_Current   = GPIO_DC_HIGH;
    gpio.GPIO_Alternate = GPIO_AF4_USART1;      /* PA10 = USART1_RX */
    GPIO_InitPeripheral(GPIOA, &gpio);
}

static void level_uart_usart_init(uint32_t baudrate)
{
    USART_InitType USART_InitStructure;

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_USART1, ENABLE);

    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.BaudRate            = baudrate;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;  /* TX 给日志用(PB6) */
    USART_Init(USART1, &USART_InitStructure);

    /* 本SDK的 USART_Init 只重写字长/校验/收发使能(CTRL1)、STOP位(CTRL2)、流控(CTRL3)，
     * 不会动 RXDNEIE(CTRL1 bit5) 和 UE(bit13)；这里仍显式重开一次，不依赖库实现细节 */
    USART_ConfigInt(USART1, USART_INT_RXDNE, ENABLE);
    USART_Enable(USART1, ENABLE);
}

void BSP_LevelUart_Init(uint32_t baudrate)
{
    NVIC_InitType NVIC_InitStruct;

    level_uart_pin_config();
    level_uart_usart_init(baudrate);

    /* NVIC：USART1 的接收中断（注意不是 USART2！） */
    NVIC_InitStruct.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority           = 2U;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

void BSP_LevelUart_SetBaud(uint32_t baudrate)
{
    level_uart_usart_init(baudrate);
}
