/**
 * @file protocol_util.h
 * @brief 协议通用工具宏定义
 */
#ifndef __PROTOCOL_UTIL_H__
#define __PROTOCOL_UTIL_H__

#include <stdint.h>
#include <stdbool.h>

/* 协议基础常量 */
#define PROTO_FRAME_HEADER0         0xAAU
#define PROTO_FRAME_HEADER1         0x55U
#define PROTO_VERSION_V1            0x01U

/* 帧固定头部长度：帧头2 +版本1 +网关3 +设备3 +功能码1 +data_len1 */
#define PROTO_FIXED_HEAD_LEN        (2U+1U+3U+3U+1U+1U)
#define PROTO_CRC_LEN               2U
/* 完整帧最小长度：固定头 + crc(当data长度=0) */
#define PROTO_FRAME_MIN_LEN         (PROTO_FIXED_HEAD_LEN + PROTO_CRC_LEN)

/* 地址长度：网关/设备地址都是3字节 */
#define PROTO_ADDR_LEN              3U

/**
 * @brief 3字节地址比较
 * @param a 地址A[3]
 * @param b 地址B[3]
 * @return true相等，false不相等
 */
static inline bool proto_addr_equal(const uint8_t a[3], const uint8_t b[3])
{
    return ((a[0]==b[0]) && (a[1]==b[1]) && (a[2]==b[2]));
}

/**
 * @brief 拷贝3字节地址
 */
static inline void proto_addr_copy(uint8_t dst[3], const uint8_t src[3])
{
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
}

/**
 * @brief 判断3字节地址是不是"有效地址"（配对/绑定用）
 * @param a 3字节地址
 * @return true=有效；false=未绑定
 * @note  两种都算未绑定：
 *          FF FF FF —— 出厂值 / 已被 0x05 解绑（同时也是配对请求的"任意网关"目的地址）
 *          00 00 00 —— Flash 空白/损坏，不能当有效地址用
 */
static inline bool proto_addr_is_valid(const uint8_t a[3])
{
    if ((a[0] == 0xFFU) && (a[1] == 0xFFU) && (a[2] == 0xFFU))
    {
        return false;               /* 未绑定 */
    }
    if ((a[0] == 0x00U) && (a[1] == 0x00U) && (a[2] == 0x00U))
    {
        return false;               /* 空白/损坏 */
    }
    return true;
}

/**
 * @brief 把一个3字节地址写成"未绑定"值 FF FF FF
 */
static inline void proto_addr_set_unbound(uint8_t a[3])
{
    a[0] = 0xFFU;
    a[1] = 0xFFU;
    a[2] = 0xFFU;
}

#endif
