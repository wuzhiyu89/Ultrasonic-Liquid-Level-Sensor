/**
 * @file app_runtime_state.c
 * @brief 全局运行时状态RAM，唯一运行时数据源
 */
#include "app_runtime_state.h"
#include "app_dev_param.h"
#include <string.h>

/* 私有全局运行时状态，外部不能直接访问，只能通过接口拷贝 */
static AppRuntimeState_t s_runtime;

void app_runtime_state_init(void)
{
    /* 1.加载设备地址 */
    app_dev_param_get_dev_addr(s_runtime.dev_addr);
    /* 2.加载网关地址（Flash 里的 0x11+网关 UID；13×FF = 未绑定） */
    app_dev_param_get_gw_addr(s_runtime.gw_addr);
    /* 2.加载传感器持久化配置 */
    app_dev_param_get_sensor_cfg(&s_runtime.sensor_cfg);
    /* 3.网关时间初始化为0，掉电丢失 */
    memset(&s_runtime.gateway_time, 0U, sizeof(RuntimeGatewayTime_t));
}


void app_runtime_state_get(AppRuntimeState_t *out)
{
    if(out != NULL)
    {
        memcpy(out, &s_runtime, sizeof(AppRuntimeState_t));
    }
}

void app_runtime_state_set_gateway_time(uint8_t year, uint8_t month, uint8_t day, uint8_t week, uint8_t hour, uint8_t minute)
{
    s_runtime.gateway_time.year   = year;
    s_runtime.gateway_time.month  = month;
    s_runtime.gateway_time.day    = day;
    s_runtime.gateway_time.week   = week;
    s_runtime.gateway_time.hour   = hour;
    s_runtime.gateway_time.minute = minute;
    s_runtime.gateway_time.second = 0U; /*网关不下发秒，固定0*/
}

void app_runtime_state_set_sensor_cfg(const DevSensorConfig_t *cfg)
{
    if(cfg == NULL)
    {
        return;
    }
    /* 只更新RAM：STOP低功耗不会丢RAM。
     * Flash 落盘由 main.c 睡前调 app_dev_param_set_sensor_cfg() 负责
     * （避开收包窗口做擦写，且那边内部有去重） */
    memcpy(&s_runtime.sensor_cfg, cfg, sizeof(DevSensorConfig_t));
}

void app_runtime_state_set_gw_addr(const uint8_t gw_addr[PROTO_ADDR_LEN])
{
    if(gw_addr == NULL)
    {
        return;
    }
    memcpy(s_runtime.gw_addr, gw_addr, PROTO_ADDR_LEN);
}

