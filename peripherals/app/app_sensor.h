/**
 * @file app_sensor.h
 * @brief 传感器相关接口
 * @note 发送传感器数据到网关
 * 完整上报帧格式：
 * | 帧头2B |版本1B|网关地址3B|设备地址3B|功能码1B|数据长度1B|数据7B|CRC16(Modbus)2B|
 * |0xAA,0x55|0x01 |gw[0‑2]  |dev[0‑2]  |0x02上报|0x07      |数据域7字节 |CRC低、高|
 * 数据域7字节说明：
 *      offset0‑1:液位高度(uint16_t mm，大端)
 *      offset2:电池剩余电量百分比(uint8_t)
 *      offset3:月份 MM(uint8_t)
 *      offset4:日期 DD(uint8_t)
 *      offset5:小时 HH(uint8_t)
 *      offset6:分钟 MM(uint8_t)
 * @attention 时间来源于网关下发的 YY.MM.DD.WW.HH.MM，只上报MM/DD/HH/MM；
 *            网关从未校时时全部为0。
 */
#ifndef __APP_SENSOR_H__
#define __APP_SENSOR_H__
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 组装液位上报帧，发送并等待网关ACK（没ACK自动重发）
 * @return true 组装成功且收到ACK；false 失败/没收到ACK
 */
bool app_sensor_level_report_send(void);

/**
 * @brief 只组装液位上报帧（不发送）
 * @param level_mm 液位高度(mm)；台面测试没接液位传感器时传固定值
 * @param out_buf  输出缓冲(至少 20 字节: 13字节头+CRC + 7字节数据)
 * @param out_len  输出帧长度
 * @retval true 组装成功；false 参数错误/液位无效(0xFFFF)
 * @note  给上层自己控制"发 -> 开接收窗口"时序用
 */
bool app_sensor_build_level_report(uint16_t level_mm, uint8_t *out_buf, uint16_t *out_len);

/**
 * @brief 用指定的液位值组装上报帧，发送并等待网关ACK
 * @param level_mm 液位高度(mm)
 * @return true 收到ACK；false 失败/没收到ACK
 */
bool app_sensor_level_report_send_ex(uint16_t level_mm);

#ifdef __cplusplus
}
#endif
#endif // __APP_SENSOR_H__
