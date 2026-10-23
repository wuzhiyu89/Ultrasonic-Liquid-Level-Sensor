/**
 * @file app_sensor.h
 * @brief 液位上报（0x02）帧的换算与组装
 *
 * 帧格式（地址 13 字节 = 1 字节前缀 + 96 位 UID），总长 40 字节：
 *   [0..1]AA 55 [2]ver [3..15]gw(13) [16..28]dev(13) [29]func=0x02
 *   [30]dlen=7  [31..37]数据域  [38][39]CRC16/Modbus(低字节在前)
 * 数据域 7 字节：
 *   off0-1 液位(uint16，大端，mm)   off2 电量%(0~100)
 *   off3 月  off4 日  off5 时  off6 分（北京时间）
 * 样例（占位 UID，液位 1234mm/100%/10-11 12:13）：
 *   AA 55 01 | 11 21..2C | 21 01..0C | 02 07 | 04 D2 64 0A 0B 0C 0D | 18 D0
 * 未校时时时间字段沿用网关最近下发值（通常全 0）。
 */
#ifndef __APP_SENSOR_H__
#define __APP_SENSOR_H__
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 把超声模块读数换算成最终上报用的液位（测量模式 + 安装高度 + 校准偏移）
 * @param raw_mm 模块原始读数(mm)；0xFFFF 表示无效
 * @retval 最终液位(mm)；0xFFFF = 无效（原样透传）
 * @note  LEVEL 模式原样使用；DIST 模式 液位 = 安装高度(cm)×10 − 距离(mm)；
 *        再叠加 calib_offset_mm（int16，可负），结果按 [0, 65534] 饱和。
 * @note  配置来自 app_runtime_state，收到 0x01 即生效。
 *        ⚠️ 干态"上报 0"的路径不要调它 —— 否则校准偏移会把 0 抬成正数。
 */
uint16_t app_sensor_convert_level(uint16_t raw_mm);

/**
 * @brief 只组装液位上报帧（不发送）
 * @param level_mm 液位高度(mm)【已换算、已含校准偏移】；台面测试可传固定值
 * @param out_buf  输出缓冲(至少 40 字节: 固定头31 + CRC2 + 7字节数据)
 * @param out_len  输出帧长度
 * @retval true 组装成功；false 参数错误/液位无效(0xFFFF)
 * @note  给上层自己控制"发 -> 开接收窗口"时序用（main.c 就是这么用的）
 */
bool app_sensor_build_level_report(uint16_t level_mm, uint8_t *out_buf, uint16_t *out_len);

#ifdef __cplusplus
}
#endif
#endif // __APP_SENSOR_H__
