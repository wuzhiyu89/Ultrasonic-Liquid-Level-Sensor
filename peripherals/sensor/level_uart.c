#include "level_uart.h"
#include "bsp_uart.h"
#include "delay.h"
#include "n32g031_gpio.h"
#include "n32g031_rcc.h"
#include <stddef.h>     /* NULL */

/* 超声波模块触发脚：模块的RX输入，接在 PA9（PA9 的复用功能是 USART1_TX，
 * 本工程不用其串口复用，只当普通 GPIO 输出）。
 * 给一个高电平脉冲，下降沿触发一次测量；两次触发间隔必须 > 1s。 */
#define LEVEL_TRIG_PORT     GPIOA
#define LEVEL_TRIG_PIN      GPIO_PIN_9

static void level_trigger_init(void)
{
    GPIO_InitType gpio;

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);

    GPIO_InitStruct(&gpio);
    gpio.Pin            = LEVEL_TRIG_PIN;
    gpio.GPIO_Mode      = GPIO_MODE_OUTPUT_PP;
    gpio.GPIO_Pull      = GPIO_NO_PULL;
    gpio.GPIO_Speed     = GPIO_SPEED_HIGH;
    gpio.GPIO_Current   = GPIO_DC_HIGH;
    gpio.GPIO_Alternate = GPIO_NO_AF;
    GPIO_InitPeripheral(LEVEL_TRIG_PORT, &gpio);

    GPIO_ResetBits(LEVEL_TRIG_PORT, LEVEL_TRIG_PIN);   /* 空闲低 */
}

/* 解析状态机：0xFF 帧头 -> 距离高 -> 距离低 -> 校验和 */
typedef enum
{
    WAIT_HEADER,
    GET_DATA_H,
    GET_DATA_L,
    GET_SUM
}uart_state_t;

static volatile uart_state_t  g_uart_state = WAIT_HEADER;
static uint8_t       g_buf[4];
static volatile uint16_t      g_level_mm = 0xFFFF; // 0xFFFF代表无效
static volatile uint16_t      g_level_seq = 0U;    // 每收到一帧有效液位+1

/* 状态机超时保护：非 WAIT_HEADER 状态下连续 20 次轮询（10ms/次 ≈200ms）
 * 都没收到任何字节则强制复位；收到字节会立刻把计数清零(见 Level_UART_RxByte) */
#define UART_RX_TIMEOUT_CNT    20U
static volatile uint8_t  g_uart_timeout_cnt = 0U;

/**
 * @brief 串口接收单字节解析，中断回调入口
 */
void Level_UART_RxByte(uint8_t ch)
{
    g_uart_timeout_cnt = 0U;   /* 收到字节即清零超时计数 */

    switch(g_uart_state)
    {
        case WAIT_HEADER:
        {
            if(ch == 0xFF)
            {
                g_buf[0] = ch;
                g_uart_state = GET_DATA_H;
            }
            break;
        }
        case GET_DATA_H:
        {
            g_buf[1] = ch;
            g_uart_state = GET_DATA_L;
            break;
        }
        case GET_DATA_L:
        {
            g_buf[2] = ch;
            g_uart_state = GET_SUM;
            break;
        }
        case GET_SUM:
        {
            g_buf[3] = ch;
            //校验和计算 (0XFF + H + L) &0xFF
            uint8_t calc_sum = (g_buf[0] + g_buf[1] + g_buf[2]) & 0xFF;
            if(calc_sum == g_buf[3])
            {
                //校验正确，更新液位 = Data_H*256 + Data_L (mm)
                //例: FF 07 A1 A7 -> 0xFF+0x07+0xA1=0x1A7 低8位0xA7匹配, 液位=7*256+161=1953mm
                g_level_mm = ((uint16_t)g_buf[1] << 8) | g_buf[2];
                g_level_seq++;      /* 帧序号+1，供 Level_Measure 判断"收到新帧" */
                g_uart_state = WAIT_HEADER;
            }
            else if(ch == 0xFF)
            {
                //校验失败但这一个字节是帧头：当成新帧的起点，重新同步
                g_buf[0] = ch;
                g_uart_state = GET_DATA_H;
            }
            else
            {
                g_uart_state = WAIT_HEADER;
            }
            break;
        }
        default:
            g_uart_state = WAIT_HEADER;
            break;
    }
}

/**
 * @brief 切换接收波特率（USART1 收发共用，日志会跟着变）
 */
void Level_UartSetBaud(uint32_t baudrate)
{
    BSP_LevelUart_SetBaud(baudrate);
}

void Level_UART_Init(void)
{
    /* PA10 = USART1_RX(AF4)，波特率见 LEVEL_UART_BAUD。
     * 注意：日志 TX 在 PB6 上也是 USART1，两者共享同一个波特率。 */
    BSP_LevelUart_Init(LEVEL_UART_BAUD);

    /* 触发脚(PA9)配成普通输出 */
    level_trigger_init();

    g_uart_state = WAIT_HEADER;
    g_level_mm   = 0xFFFF;
    g_level_seq  = 0U;
}

/**
 * @brief 进STOP前把触发脚(PA9)钉在低电平；须在 BSP_GPIO_SetAllAnalog() 之后调用
 * @note  模块在 PB7 域上、STOP 期间断电，TRIG 悬空会被噪声误触发 -> 测距电流脉冲 -> POR/BOR 复位。
 */
void Level_TrigHoldLow(void)
{
    GPIO_InitType gpio;

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);

    GPIO_InitStruct(&gpio);
    gpio.Pin            = LEVEL_TRIG_PIN;
    gpio.GPIO_Mode      = GPIO_MODE_OUTPUT_PP;
    gpio.GPIO_Pull      = GPIO_NO_PULL;
    gpio.GPIO_Speed     = GPIO_SPEED_LOW;
    gpio.GPIO_Current   = GPIO_DC_LOW;
    gpio.GPIO_Alternate = GPIO_NO_AF;
    GPIO_InitPeripheral(LEVEL_TRIG_PORT, &gpio);

    GPIO_ResetBits(LEVEL_TRIG_PORT, LEVEL_TRIG_PIN);   /* 空闲低 */
}

/**
 * @brief 获取最新液位
 * @retval 0~65535 有效液位mm；0XFFFF 无有效帧
 */
uint16_t Level_Get_Val(void)
{
    return g_level_mm;
}

/**
 * @brief 状态机超时轮询（只有 Level_Measure() 在等帧时每10ms调用一次）
 * @note  非 WAIT_HEADER 状态连续 200ms 没收到字节（收到即清零计数）则强制复位
 */
void Level_UART_TimeoutPoll(void)
{
    if (g_uart_state != WAIT_HEADER)
    {
        g_uart_timeout_cnt++;
        if (g_uart_timeout_cnt >= UART_RX_TIMEOUT_CNT)
        {
            g_uart_state = WAIT_HEADER;
            g_uart_timeout_cnt = 0U;
        }
    }
}

/**
 * @brief 触发一次液位测量并等待一帧新数据；返回 0~65535 液位mm，0xFFFF 超时/无数据
 * @param timeout_ms 最长等待时间；PA9 先拉高再拉低（下降沿触发），两次触发间隔必须 > 1s
 */
uint16_t Level_Measure(uint32_t timeout_ms)
{
    uint16_t seq;
    uint32_t t;

    level_trigger_init();            /* 上电/STOP唤醒后GPIO会变，每次重新配 */

    GPIO_SetBits(LEVEL_TRIG_PORT, LEVEL_TRIG_PIN);     /* 高电平触发脉冲 */
    delay_ms(1U);
    GPIO_ResetBits(LEVEL_TRIG_PORT, LEVEL_TRIG_PIN);   /* 下降沿 -> 模块测一次 */

    seq = g_level_seq;
    for (t = 0U; t < timeout_ms; t++)
    {
        if (g_level_seq != seq)          /* 收到新帧 */
        {
            return g_level_mm;
        }
        if ((t % 10U) == 0U)             /* 状态机超时保护按10ms轮询 */
        {
            Level_UART_TimeoutPoll();
        }
        delay_ms(1U);
    }
    return 0xFFFF;
}
