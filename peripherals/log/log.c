#if LOG_ENABLE
#include "log.h"        /* LOG_BAUD 等配置 */
#include "n32g031.h"
#include "n32g031_gpio.h"
#include "n32g031_usart.h"
#include "n32g031_rcc.h"
#include <stdio.h>

#define LOG_USARTx      USART1
#define LOG_PERIPH      RCC_APB2_PERIPH_USART1
#define LOG_GPIO        GPIOB
#define LOG_PERIPH_GPIO RCC_APB2_PERIPH_GPIOB
#define LOG_TX_PIN      GPIO_PIN_6
#define LOG_TX_AF       GPIO_AF4_USART1

void log_init(void)
{
    GPIO_InitType GPIO_InitStructure;
    USART_InitType USART_InitStructure;

    RCC_EnableAPB2PeriphClk(LOG_PERIPH_GPIO | RCC_APB2_PERIPH_AFIO | LOG_PERIPH, ENABLE);

    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin            = LOG_TX_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Pull      = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStructure.GPIO_Current   = GPIO_DC_HIGH;
    GPIO_InitStructure.GPIO_Alternate = LOG_TX_AF;
    GPIO_InitPeripheral(LOG_GPIO, &GPIO_InitStructure);

    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.BaudRate            = LOG_BAUD;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    /* Mode 必须包含 RX：液位模块的接收(PA10=USART1_RX)也用这个 USART1，
     * 只写 TX 会把接收方向关掉，液位就再也收不到数据了 */
    USART_InitStructure.Mode                = USART_MODE_TX | USART_MODE_RX;
    USART_Init(LOG_USARTx, &USART_InitStructure);
    USART_Enable(LOG_USARTx, ENABLE);

    /* wait UART stable, avoid first char garbled */
    for (volatile uint32_t i = 0U; i < 10000U; i++) { __NOP(); }
}

static int is_lr_sent = 0;
int fputc(int ch, FILE* f)
{
    if (ch == '\r')
    {
        is_lr_sent = 1;
    }
    else if (ch == '\n')
    {
        if (!is_lr_sent)
        {
            while (USART_GetFlagStatus(LOG_USARTx, USART_FLAG_TXDE) == RESET)
            {
            }
            USART_SendData(LOG_USARTx, (uint8_t)'\r');
        }
        is_lr_sent = 0;
    }
    else
    {
        is_lr_sent = 0;
    }
    while (USART_GetFlagStatus(LOG_USARTx, USART_FLAG_TXDE) == RESET)
    {
    }
    USART_SendData(LOG_USARTx, (uint8_t)ch);
    return ch;
}
#endif // LOG_ENABLE
