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

/* 发射超时(ms)：作为 Radio.SetTxConfig 的发射超时参数。
 * 业务路径等 TxDone 的超时由各调用点自己传（APP_TX_DONE_TIMEOUT_MS = 3s）。 */
#define LORA_ACK_TIMEOUT_MS     10000U

/* ===== 收发模式切换等待(ms) =====
 * LLCC68 收发切换要重走 PLL / PA ramp / 接收链路上电，不是"写完命令就绪"：
 * 切换期间收到的包会被丢，发出去的包前导码可能不完整。
 *
 *   LORA_TX_MODE_DELAY_MS：收到消息 -> 切发射 -> 等这段时间再发 ACK/应答。
 *       这就是"收到消息到发出应答"之间的全部等待，app_cmd.c 里不再另加延时。
 *   LORA_RX_MODE_DELAY_MS：发完切回接收也要留空档，但不必和发射侧一样长。
 *
 * ⚠️ RX 侧必须【明显小于】网关的 LORA_TX_SETTLE_MS(500ms，网关侧常量)，
 *    否则两边会撞在一起：设备发完上报等 500ms 才开接收、网关收到上报等 500ms 才发 ACK，
 *    网关前导码正好压在设备接收链路刚上电的瞬间，ACK 白丢。设备只等 100ms，
 *    比网关早 400ms 进接收（固件发完已用 SendSync 确认 TX_DONE，100ms 只是余量）。
 * 用日志 "[lora] tx/rx settle: wait=.. actual=.." 验证实际耗时。 */
#define LORA_TX_MODE_DELAY_MS   500U
#define LORA_RX_MODE_DELAY_MS   100U

/* 接收缓存长度 */
#define LORA_RX_BUF_LEN         64U

/**
 * @brief LoRa 初始化（配置频率/调制/收发参数）
 * @retval 0 成功；1 射频无响应（BUSY 一直为高，见下面的 LoraApp_TakeBusyFault）
 */
uint8_t LoraApp_Init(void);

/**
 * @brief 取"自上次调用以来射频有没有 BUSY 超时"(模组没响应: 没供电/坏了/SPI 断)
 * @retval true = 期间出现过超时 -> 本轮不要收发，直接去睡；false = 正常
 * @note  板级 LLCC68WaitOnBusy() 有 50ms 硬超时；本工程不能用 IWDG 兜底
 *        （它挂在 LSI 上，STOP 期间照样计数）。
 */
bool LoraApp_TakeBusyFault(void);

/**
 * @brief 主循环必须周期调用：处理 radio 事件（发送完成/收到包/超时）
 */
void LoraApp_ProcessIrq(void);

/**
 * @brief 发送一包数据（不等待，异步）
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
 * @brief 进入持续接收模式（一直收，不超时）
 */
void LoraApp_ReciveMode(void);

/**
 * @brief 同步发完一包后立刻重回持续接收（不再等收发切换的那段空档）
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
 * @brief 让 LoRa 进真正的 Sleep(比 Standby 更深)，进 STOP 前调用
 * @note  外设域是否真断电取决于板上偏置；域还带电时只 Standby 会持续耗电
 */
void LoraApp_Sleep(void);

/**
 * @brief  取走最近收到的一包数据（取走后标志清空）
 * @param  buf    输出缓存
 * @param  maxLen 缓存大小
 * @retval 0 没有新数据；>0 实际长度
 */
uint16_t LoraApp_GetRx(uint8_t *buf, uint16_t maxLen);

#endif /* __APP_LORA_H */
