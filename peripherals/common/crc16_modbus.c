/**
 * @file crc16_modbus.c
 * @brief common 通用工具 Modbus CRC16
 */
#include "crc16_modbus.h"
#include <stdbool.h>

uint16_t crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i, j;

    for (i = 0U; i < len; i++)
    {
        crc ^= data[i];
        for (j = 0U; j < 8U; j++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc >>= 1U;
                crc ^= 0xA001U;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }
    return crc;
}

bool crc16_modbus_check(const uint8_t *frame, uint16_t frame_len)
{
    /* 帧长度至少要大于2字节，否则直接失败 */
    if (frame_len < 2U)
    {
        return false;
    }
    uint16_t calc_crc = crc16_modbus(frame, frame_len - 2U);
    /* 帧末尾低字节在前 */
    uint8_t crc_low  = frame[frame_len - 2U];
    uint8_t crc_high = frame[frame_len - 1U];
    uint16_t recv_crc = ((uint16_t)crc_high << 8U) | crc_low;

    return (calc_crc == recv_crc);
}

void crc16_modbus_append(uint8_t *buf, uint16_t data_len)
{
    uint16_t crc = crc16_modbus(buf, data_len);
    /* modbus 低字节在前 */
    buf[data_len]     = (uint8_t)(crc & 0x00FFU);
    buf[data_len + 1U] = (uint8_t)((crc >> 8U) & 0x00FFU);
}
