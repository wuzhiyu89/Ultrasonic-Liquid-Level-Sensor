/**
 * @file crc16_modbus.h
 * @brief common 通用工具 Modbus CRC16
 * @note 多项式 0xA001，初始0xFFFF，输出低字节在前
 */
#ifndef __CRC16_MODBUS_H__
#define __CRC16_MODBUS_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Modbus CRC16计算
 * @param data 待计算数据缓冲区
 * @param len 数据长度，不包含末尾 2 字节 CRC 本身
 * @return 原始crc16值；发送帧时：低字节在前，高字节在后
 */
uint16_t crc16_modbus(const uint8_t *data, uint16_t len);

/**
 * @brief 校验一帧完整数据的CRC
 * @param frame 完整帧，末尾2字节为CRC
 * @param frame_len 完整帧总长度
 * @return true:crc校验通过 false:校验失败
 */
bool crc16_modbus_check(const uint8_t *frame, uint16_t frame_len);

/**
 * @brief 把crc16值填入缓冲区末尾
 * @param buf 数据缓冲区，前面是有效数据，末尾预留2字节放crc
 * @param data_len 有效数据长度（不含CRC）
 */
void crc16_modbus_append(uint8_t *buf, uint16_t data_len);

#ifdef __cplusplus
}
#endif

#endif
