/**
 * @file protocol_util.h
 * @brief 协议通用常量与地址工具
 *
 * 地址（设备/网关统一 13 字节）：[0] 类型前缀（设备 0x21，网关 0x11）+ [1..12] 12 字节 UID
 *   （N32G031 UID_BASE=0x1FFFF4FC，UID_LENGTH=12，直读 UID_BASE+0..11 物理顺序）
 * 广播/未绑定 = 13 字节全 0xFF：设备出厂或被 0x05 解绑后 Flash 里存的网关地址即此值，
 *   0x04 配对请求的目的网关地址也填它（任意网关，谁收到谁应答）
 * 地址有效性：网关 = 前缀 0x11 且非全 FF 且非全 00；设备 = 前缀 0x21 且非全 00
 *   全 FF 只当广播/未绑定，全 00 当 Flash 空白/损坏，两者都不算有效地址
 */
#ifndef __PROTOCOL_UTIL_H__
#define __PROTOCOL_UTIL_H__

#include <stdint.h>
#include <stdbool.h>

/* ==================== 帧格式常量 ====================
 * AA 55 | ver(1) | gw_addr(13) | dev_addr(13) | func(1) | data_len(1) | data | CRC16(2)
 */
#define PROTO_ADDR_PREFIX_LEN       1U
#define PROTO_UID_LEN               12U
#define PROTO_ADDR_LEN              (PROTO_ADDR_PREFIX_LEN + PROTO_UID_LEN)     /* = 13 */

/* 地址类型前缀 */
#define PROTO_ADDR_PREFIX_DEVICE    0x21U   /* 从设备（液位节点 0x21 + UID） */
#define PROTO_ADDR_PREFIX_GATEWAY   0x11U   /* 网关（0x11 + UID） */

/* 帧固定头部长度：帧头2 +版本1 +网关13 +设备13 +功能码1 +data_len1 = 31 */
#define PROTO_FIXED_HEAD_LEN        (2U + 1U + PROTO_ADDR_LEN + PROTO_ADDR_LEN + 1U + 1U)
#define PROTO_CRC_LEN               2U
/* 完整帧最小长度：固定头 + CRC（data_len = 0 时）= 33 */
#define PROTO_FRAME_MIN_LEN         (PROTO_FIXED_HEAD_LEN + PROTO_CRC_LEN)

/* 帧内各字段的偏移（各模块统一引用，不要手写下标） */
#define PROTO_OFF_GW_ADDR           (2U + 1U)                                   /* = 3  */
#define PROTO_OFF_DEV_ADDR          (PROTO_OFF_GW_ADDR + PROTO_ADDR_LEN)        /* = 16 */
#define PROTO_OFF_FUNC              (PROTO_OFF_DEV_ADDR + PROTO_ADDR_LEN)       /* = 29 */
#define PROTO_OFF_DATALEN           (PROTO_OFF_FUNC + 1U)                       /* = 30 */
#define PROTO_OFF_DATA              (PROTO_OFF_DATALEN + 1U)                    /* = 31 */

/**
 * @brief 13 字节地址比较
 * @return true 相等
 */
static inline bool proto_addr_equal(const uint8_t a[PROTO_ADDR_LEN], const uint8_t b[PROTO_ADDR_LEN])
{
    uint8_t i;
    for (i = 0U; i < PROTO_ADDR_LEN; i++)
    {
        if (a[i] != b[i])
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief 拷贝 13 字节地址
 */
static inline void proto_addr_copy(uint8_t dst[PROTO_ADDR_LEN], const uint8_t src[PROTO_ADDR_LEN])
{
    uint8_t i;
    for (i = 0U; i < PROTO_ADDR_LEN; i++)
    {
        dst[i] = src[i];
    }
}

/**
 * @brief 是不是"广播/未绑定"地址（13 字节全 FF）
 */
static inline bool proto_addr_is_broadcast(const uint8_t a[PROTO_ADDR_LEN])
{
    uint8_t i;
    for (i = 0U; i < PROTO_ADDR_LEN; i++)
    {
        if (a[i] != 0xFFU)
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief 是不是全 0（Flash 空白/损坏、UID 读取失败）
 */
static inline bool proto_addr_is_all_zero(const uint8_t a[PROTO_ADDR_LEN])
{
    uint8_t i;
    for (i = 0U; i < PROTO_ADDR_LEN; i++)
    {
        if (a[i] != 0x00U)
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief 网关地址是否有效（= 已绑定，配对判定用）
 * @note  前缀必须是 0x11；全 FF（未绑定/广播）和全 00（空白/损坏）都不算有效
 */
static inline bool proto_addr_is_valid_gw(const uint8_t a[PROTO_ADDR_LEN])
{
    if (a[0] != PROTO_ADDR_PREFIX_GATEWAY)
    {
        return false;
    }
    if (proto_addr_is_broadcast(a))
    {
        return false;
    }
    if (proto_addr_is_all_zero(a))
    {
        return false;
    }
    return true;
}

/**
 * @brief 设备地址是否有效（本机自己组帧前自检用）
 * @note  前缀必须是 0x21；全 00 视为无效（UID 没读到）
 */
static inline bool proto_addr_is_valid_dev(const uint8_t a[PROTO_ADDR_LEN])
{
    if (a[0] != PROTO_ADDR_PREFIX_DEVICE)
    {
        return false;
    }
    if (proto_addr_is_all_zero(a))
    {
        return false;
    }
    return true;
}

/**
 * @brief 把一个 13 字节地址写成"未绑定/广播"值（全 FF）
 */
static inline void proto_addr_set_unbound(uint8_t a[PROTO_ADDR_LEN])
{
    uint8_t i;
    for (i = 0U; i < PROTO_ADDR_LEN; i++)
    {
        a[i] = 0xFFU;
    }
}

#endif /* __PROTOCOL_UTIL_H__ */
