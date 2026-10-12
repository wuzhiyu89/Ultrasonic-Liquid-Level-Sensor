#ifndef __PROJECT_CONFIG_H__
#define __PROJECT_CONFIG_H__

//基于semtech官网驱动移植
//官网驱动网址 https://github.com/Lora-net/LoRaMac-node/tree/master/src/radio  下载日期 2021/2/3
#define SOFT_VERSION	"LLCC68 driver for stm32f103 V0.0.0"

//--------------------------------------------- 测试默认配置 ---------------------------------------------
#define LORA_FRE 433000000               // 收发频率
/* 发射功率：本模块芯片自报 SX1261(版本串 0x0320="SX1261")，SX1261 分支上限就是 +14dBm。
 * ⚠️ 这里原来是 22 —— 那是【按 SX1262 配的】值。走错 PA 分支(把 SX1261 当 SX1262 配)的
 *    典型症状是：TX_DONE 正常、GetErrors 全 0、自己接收也正常，但空口上对方收不到/只能收到残帧。
 *    改成 14 之后【两个分支都安全】(SX1261 上限 14，SX1262 也远没到上限)。
 *    注：本机已实测走的是 SX1261 分支(0x0320=53 58 31 32 36 31)，所以 22 会被钳成 14，
 *        改不改功能上一样；写 14 只是让配置和注释一致，避免以后换芯片时踩坑。 */
#define LORA_TX_OUTPUT_POWER 14

/* ============================================================================
 *  ★★ 空口速率档位 —— 【改这里时，对端(网关)必须改成完全一样的两行！】★★
 * ----------------------------------------------------------------------------
 * 现场实测：网关侧 SNR=-13dB，而 SF8/BW500k 的解调门限约 -7dB → 差约 6dB，
 * 表现就是"20 字节里偶尔错 1 个字节、CRC 不过 → 对端判坏帧丢掉"。
 * 这里把空口调慢一档，把余量补回来（LoRaWAN 标准档就是 125kHz）：
 *
 *   带宽 500k(2) -> 125k(0) : 带内噪声少 6dB    -> +6.0 dB
 *   SF8 -> SF9              : 处理增益 +2.5 dB  -> +2.5 dB
 *                                      合计约    +8.5 dB（约剩 2.5dB 余量）
 *
 *  档位对照（20 字节帧的空口时长，供参考）：
 *     BW500k(2)+SF8  = 原档   0 dB     ~40 ms
 *     BW250k(1)+SF9  = 折中   +5.5 dB  ~75 ms
 *     BW125k(0)+SF9  = 当前   +8.5 dB  ~280 ms
 *     BW125k(0)+SF10 = 更稳   +11 dB   ~560 ms
 *
 * ⚠️ 两边的 LORA_BANDWIDTH 和 LORA_SPREADING_FACTOR 【必须一模一样】，
 *    只改一边 = 完全收不到（连前导码都认不出来）。
 * ⚠️ 空口变长只影响"发一帧要多久"，本工程上报周期是分钟级，完全够用。 */
#define LORA_BANDWIDTH 0                // [ 0: 125 kHz,	测试默认使用的带宽，LLCC68：[0: 125 kHz,1: 250 kHz,2: 500 kHz,3: Reserved]
#define LORA_SPREADING_FACTOR 9          // 测试默认使用的扩频因子范围7~12
#define LORA_CODINGRATE 2                // 测试默认使用的纠错编码率[1: 4/5,2: 4/6,3: 4/7,4: 4/8]（还嫌不够可改 4=4/8，+1.5dB）
#define LORA_PREAMBLE_LENGTH 10           // 前导码长度
#define LORA_LLCC68_SYMBOL_TIMEOUT 0     // Symbols(LLCC68用到的是0,127x用到的是5)
#define LORA_FIX_LENGTH_PAYLOAD_ON false // 是否为固定长度包(暂时只是LLCC68用到了)
#define LORA_IQ_INVERSION_ON false       // 这个应该是设置是否翻转中断电平的(暂时只是LLCC68用到了)
#define LORA_RX_TIMEOUT_VALUE 5000
#define TCXO_CTRL_VOLTAGE 2             // setting TCXO voltage supply source [0: 1.6v, 1: 1.7v, 2: 1.8v, 3: 2.2v, 4: 2.4v, 5: 2.7v, 6: 3.0v, 7: 3.5v]
#define RF_WAKEUP_TIME 100              // setting RF wakeup time,in second
#define LORA_DATA_SEND_OR_RECEIV_MODE 0 // 数据收发模式，1为发送模式，0为接收模式

/*!
 * Board MCU pins definitions
 */
//SPI引脚(SPI1: SCK=PA0 MISO=PA4 MOSI=PA5, 均为AF1; NSS=PA1软件控制)
#define RADIO_NSS_PIN       GPIO_PIN_1
#define RADIO_NSS_PORT      GPIOA
#define RADIO_MOSI_PIN      GPIO_PIN_5
#define RADIO_MOSI_PORT     GPIOA
#define RADIO_MISO_PIN      GPIO_PIN_4
#define RADIO_MISO_PORT     GPIOA
#define RADIO_SCK_PIN       GPIO_PIN_0
#define RADIO_SCK_PORT      GPIOA
//RST复位脚
#define RADIO_nRESET_PIN    GPIO_PIN_6
#define RADIO_nRESET_PORT   GPIOA
//DIO1 引脚
#define RADIO_DIO1_PIN      GPIO_PIN_15
#define RADIO_DIO1_PORT     GPIOC
//BUSY 引脚
#define RADIO_DIO4_BUSY_PIN      GPIO_PIN_14
#define RADIO_DIO4_BUSY_PORT     GPIOC

//  EN 引脚。⚠️注意：真正的"外设电源使能"是 PB7(GPIOB)，不是 PA7！
//  PA7 实际是探针激励脚(配 PB1 做 ADC)，见 water_cu.c / bsp_gpio.c。
//  极性资料自相矛盾：这条注释说"高电平lora上电"，而 bsp_gpio.c 按 P-MOS 高边
//  开关写的是"低=上电"。代码里目前不使用本宏（没人引用），要用前先确认原理图。
#define RADIO_EN_PIN      GPIO_PIN_7
#define RADIO_EN_PORT     GPIOB

//下面这几个引脚没用用到，设置为浮空输入模式
//TXEN 引脚
#define RADIO_DIO0_TXEN_PIN      GPIO_PIN_4
#define RADIO_DIO0_TXEN_PORT     GPIOC
//DIO2
#define RADIO_DIO2_PIN      GPIO_PIN_3
#define RADIO_DIO2_PORT     GPIOC
//DIO3
#define RADIO_DIO3_PIN      GPIO_PIN_2
#define RADIO_DIO3_PORT     GPIOC
//RXEN
#define RADIO_DIO5_RXEN_PIN      GPIO_PIN_1
#define RADIO_DIO5_RXEN_PORT     GPIOC


#endif // __PROJECT_CONFIG_H__
