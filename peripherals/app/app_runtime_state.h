/**
 * @file app_runtime_state.h
 * @brief 全局运行时状态，全部RAM，唯一运行时数据源
 * @note 上电初始化必须调用 app_runtime_state_init();
 *       持久化数据来源 app_dev_param；网关下发的校时时间掉电丢失
 */
#ifndef __APP_RUNTIME_STATE_H__
#define __APP_RUNTIME_STATE_H__

#include <stdint.h>
#include <stdbool.h>
#include "app_dev_param.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 网关下发校时时间，仅RAM，掉电丢失，second恒等于0
 */
typedef struct
{
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t week;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
}RuntimeGatewayTime_t;

/**
 * @brief 全局运行时状态结构体
 */
typedef struct
{
    uint8_t dev_addr[PROTO_ADDR_LEN];     /* 本机设备地址：0x21+96位UID，app_dev_param 现算 */
    uint8_t gw_addr[PROTO_ADDR_LEN];      /* 网关地址：0x11+网关UID；13×FF=未绑定 */
    DevSensorConfig_t sensor_cfg;         /* 传感器配置运行时副本，来自app_dev_param */
    RuntimeGatewayTime_t gateway_time;     /* 网关下发校时时间，掉电丢失 */
}AppRuntimeState_t;

/**
 * @brief 获取全局运行时状态（拷贝输出，防止外部直接修改内部变量）
 * @param out 输出结构体指针
 */
void app_runtime_state_get(AppRuntimeState_t *out);

/**
 * @brief 初始化运行时状态，从app_dev_param加载持久化参数
 * 【上电在app_dev_param_init()之后调用】
 */
void app_runtime_state_init(void);

/**
 * @brief 更新网关下发的校时时间（app_cmd解析到网关校时命令调用）
 * @param year   两位年份(2000+)
 * @param month  1..12
 * @param day    1..31
 * @param week   1=周一..7=周日，0=网关没填
 * @param hour   0..23
 * @param minute 0..59
 * @note second自动置0；仅更新RAM，不写Flash
 */
void app_runtime_state_set_gateway_time(uint8_t year, uint8_t month, uint8_t day, uint8_t week, uint8_t hour, uint8_t minute);

/**
 * @brief 设置传感器配置（这里只更新RAM，不写Flash）
 * @param cfg 新配置（网关下发）
 * @note  本函数不落Flash：网关 0x01 下来时只更新RAM，Flash 落盘由 main.c 睡前
 *        调 app_dev_param_set_sensor_cfg() 负责（避开收包窗口，且内部去重）
 */
void app_runtime_state_set_sensor_cfg(const DevSensorConfig_t *cfg);

/**
 * @brief 更新运行时网关地址（配对成功写入新地址 / 解绑写成 13×FF）
 * @param gw_addr 13字节网关地址（0x11 + 网关 UID）
 * @note  本函数【只改RAM】，Flash 落盘由 app_dev_param_set_gw_addr() /
 *        app_dev_param_clear_gw_addr() 负责，调用顺序：
 *          先落 Flash 成功，再更新这里（避免 Flash 写失败但 RAM 已经变了）
 */
void app_runtime_state_set_gw_addr(const uint8_t gw_addr[PROTO_ADDR_LEN]);


#ifdef __cplusplus
}
#endif
#endif
