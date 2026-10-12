/**
 * @file app_lora.h
 * @brief LoRa(LLCC68) 收发接口（精简版）
 *
 * 分工：
 *   组帧/打包（液位+时间+电压等）不在这里，在业务层做好；
 *   app_lora 只负责：发送 -> 开接收等ACK -> 没ACK就重发；以及把收到的包交出去。
 */
#ifndef __APP_LORA_H
#define __APP_LORA_H

#include <stdint.h>
#include <stdbool.h>
#include "project_config.h"

/* 等ACK超时(ms) 与 默认重发次数
 * 排障阶段放大到 10s: 收发切换就占 5s(收到->等5s->发), 窗口给足才不至于
 * 因为切换等待把应答窗口吃掉。确认链路稳定后再往回收。 */
#define LORA_ACK_TIMEOUT_MS     10000U
#define LORA_SEND_RETRY         3U

/* ===== 收发模式切换等待(ms) =====
 * LLCC68 从接收切到发射要重新走 PLL/PA ramp 校准, 从发射切回接收也要重新上电
 * 接收链路, 都不是"写完命令就绪"。这段切换时间里:
 *   - 正在收: 收到的包会被丢(所以回 ACK 前要先让对方发完、自己再切);
 *   - 正在发: 前导码可能不完整, 对方解不出来。
 *
 * 两边约定(2026-xx 调整, 与网关 app/protocol/lora_proto.h 对齐):
 *   - 收到消息 -> 切到发射 -> 等 LORA_TX_MODE_DELAY_MS 再发 ACK/应答;
 *     这个值就是"收到消息到发出应答"之间的全部等待, app_cmd.c 里不再另加延时。
 *   - 发完切回接收也要留同样的空档。
 *
 * 台架排障阶段: 发完切 TX 等 500ms。
 *
 * ⚠️ LORA_RX_MODE_DELAY_MS 必须【明显小于】网关的 LORA_TX_SETTLE_MS(500ms),
 *    否则两边会撞在同一时刻:
 *      设备发完上报 -> 等 500ms 才开接收;  网关收到上报 -> 等 500ms 才发 ACK
 *      => 网关的前导码正好压在设备接收链路刚上电的瞬间, 前导码解不出来, ACK 白丢。
 *    现在设备只等 100ms, 比网关早 400ms 进接收, 稳稳把 ACK 收下。
 *    (设备发完已经用 LoraApp_SendSync 确认过 TX_DONE, 这 100ms 只是余量,
 *     不再需要 500ms 那么久。)
 * 用日志里的 "[lora] tx settle: wait=.. actual=.." 验证实际耗时。 */
#define LORA_TX_MODE_DELAY_MS   500U
#define LORA_RX_MODE_DELAY_MS   100U

/* 接收缓存长度 */
#define LORA_RX_BUF_LEN         64U

/**
 * @brief LoRa 初始化（配置频率/调制/收发参数）
 * @retval 0 成功
 */
uint8_t LoraApp_Init(void);

/**
 * @brief 主循环必须周期调用：处理 radio 事件（发送完成/收到包/超时）
 */
void LoraApp_ProcessIrq(void);

/**
 * @brief  发送一包数据并等待ACK，没等到就重发
 * @param  buf   数据（业务层打包好的完整帧）
 * @param  len   长度 1~255
 * @param  retry 重发次数（0=只发一次）
 * @retval true 收到ACK；false 重发完仍未收到
 */
bool LoraApp_SendWaitAck(const uint8_t *buf, uint8_t len, uint8_t retry);

/**
 * @brief  发送一包数据（不等待，异步）
 * @retval 0 已开始发送
 */
uint8_t LoraApp_Send(const uint8_t *buf, uint8_t len);

/**
 * @brief  发送一包并等芯片 TxDone（不等待ACK）
 * @param  buf        数据
 * @param  len        长度
 * @param  timeout_ms 等 TxDone 的超时(ms)
 * @retval true=已发完；false=发送失败或等超时
 * @note   要和"发完再开接收窗口"配合时必须用它，否则接收会把还没发完的包打断
 */
bool LoraApp_SendSync(const uint8_t *buf, uint8_t len, uint32_t timeout_ms);

/**
 * @brief 兼容旧调用（app_sensor.c 在用）：只发送、不等ACK
 */
uint8_t LoraApp_SendAsync(uint8_t *buf, uint8_t len, uint32_t msTimeout);

/**
 * @brief 进入持续接收模式（一直收，不超时）
 */
void LoraApp_ReciveMode(void);

/**
 * @brief 同步发完一包后立刻重回持续接收（无 500ms 空档）
 * @note  只能在 LoraApp_SendSync() 返回 true 之后调 —— 那时 TX_DONE 已确认,
 *        不存在"发射还没收尾就被接收打断"的问题。
 *        用途: 接收窗口里回完 ACK 仍要继续把窗口剩下的时间收满。
 */
void LoraApp_ResumeRx(void);

/**
 * @brief 让LoRa回待机（停止接收、降低功耗），进STOP前调用
 */
void LoraApp_Standby(void);

/**
 * @brief 让LoRa进真正的 Sleep(比 Standby 更深)，进STOP前调用
 * @note  PA7 高阻后外设域是否真断电取决于板上偏置；域还带电时，
 *        只 Standby 的模组会持续耗电，必须用这个打进最深低功耗态
 */
void LoraApp_Sleep(void);

/**
 * @brief  取走最近收到的一包数据（取走后标志清空）
 * @param  buf    输出缓存
 * @param  maxLen 缓存大小
 * @retval 0 没有新数据；>0 实际长度
 */
uint16_t LoraApp_GetRx(uint8_t *buf, uint16_t maxLen);

/* ======================= 测试用（测试阶段结束后删掉即可） ======================= */
/* 1=编译测试函数(占Flash)，测试完改成0可省约1KB */
#ifndef LORA_ENABLE_TEST
#define LORA_ENABLE_TEST        1
#endif

#if LORA_ENABLE_TEST
/**
 * @brief 测试用发送：数组里是16进制数据，每3秒发一次，发完等ACK，没ACK自动重发
 * @note  不返回
 */
void LoraApp_TestSend(void);

/**
 * @brief 测试用接收：一直收，收到就打印，16进制、一个字节一行
 * @note  不返回
 */
void LoraApp_TestRecive(void);

/**
 * @brief 发射诊断：不靠DIO1中断，轮询芯片IRQ + 读GetErrors原始两字节
 * @note  不返回；判读方法见 app_lora.c 里的注释
 */
void LoraApp_TestTxDiag(void);
#endif

#endif /* __APP_LORA_H */
