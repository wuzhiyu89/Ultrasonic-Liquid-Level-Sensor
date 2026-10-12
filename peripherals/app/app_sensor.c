#include "app_sensor.h"
#include "level_uart.h"
#include "app_power.h"
#include "app_time.h"
#include "app_lora.h"
#include "app_runtime_state.h"
#include "crc16_modbus.h"
#include "log.h"
#include <string.h>

/* 上报帧总长 = 13 + SENSOR_DATA_LEN = 13 + 7 = 20 字节
 *
 * 帧结构(索引):
 *   [0]AA [1]55 [2]ver [3..5]gw [6..8]dev [9]func [10]dlen=7 [11..17]数据 [18][19]CRC
 *   数据域 7 字节: 液位高 | 液位低 | 电量% | 月 | 日 | 时 | 分
 *
 * ⚠️ 曾经误判这里"少发一个字节": 其实是网关协议层把恒定开销写成了 14(正确是 13),
 *    导致网关按 21 字节校验、和 20 字节的帧对不上。设备端这边一直是对的, 不用改。
 * 这里的算法是: 帧头部分 13 字节(含 2 字节 CRC) + 数据 7 字节。
 * 注意 13 是"帧头 11 字节 + CRC 2 字节", 不是 11 —— 别把 CRC 漏掉或重复算。 */
#define SENSOR_REPORT_FRAME_MAX_LEN     20U
/* 协议常量 */
#define SENSOR_FRAME_HEAD0      0xAAU
#define SENSOR_FRAME_HEAD1      0x55U
#define SENSOR_PROTO_VER        0x01U
#define SENSOR_FC_LEVEL_REPORT  0x02U
#define SENSOR_DATA_LEN         0x07U
/* 数据域以外的固定开销: 帧头部分11(AA55+ver+gw3+dev3+func+dlen) + CRC2 = 13 */
#define SENSOR_FRAME_OVERHEAD   13U

bool app_sensor_build_level_report(uint16_t level_mm, uint8_t *out_buf, uint16_t *out_len)
{
    uint8_t frame_buf[SENSOR_REPORT_FRAME_MAX_LEN];
    uint16_t frame_idx = 0U;
    AppRuntimeState_t rt_state;
    uint8_t bat_percent;
    bsp_rtc_calendar_t cal;         /* 上报用的北京时间日历（app_time_get_bj_calendar 填） */

    if((out_buf == NULL) || (out_len == NULL))
    {
        return false;
    }

    /* 运行期自检: 缓冲区要放得下 "帧头部分 + CRC" + 数据域。
     * 这里 13 已经含 CRC 2 字节, 所以别再额外加 2, 否则会误报"缓冲区太小"。
     * 目的是以后有人改长度改错了能直接返回 false, 而不是越界踩内存。 */
    if (SENSOR_REPORT_FRAME_MAX_LEN < (SENSOR_FRAME_OVERHEAD + SENSOR_DATA_LEN))
    {
        log_info("[sensor] !! frame buffer too small: %u < %u\r\n",
                 (unsigned)SENSOR_REPORT_FRAME_MAX_LEN,
                 (unsigned)(SENSOR_FRAME_OVERHEAD + SENSOR_DATA_LEN));
        return false;
    }

    /* 简单校验：液位无效直接返回，不发送 */
    if(level_mm == 0xFFFFU)
    {
        return false;
    }

    /* 1.读取运行时状态：设备地址、网关地址、gateway_time(网关下发YY.MM.DD.WW.HH.MM) */
    app_runtime_state_get(&rt_state);

    /* 2.读取电池电量 */
    bat_percent = app_power_get_battery_percent();

    /* ========= 组装协议帧 ========= */
    /* 帧头 */
    frame_buf[frame_idx++] = SENSOR_FRAME_HEAD0;
    frame_buf[frame_idx++] = SENSOR_FRAME_HEAD1;
    /* 协议版本 */
    frame_buf[frame_idx++] = SENSOR_PROTO_VER;
    /* 网关地址3字节 */
    memcpy(&frame_buf[frame_idx], rt_state.gw_addr, 3U); frame_idx +=3U;
    /* 设备地址3字节 */
    memcpy(&frame_buf[frame_idx], rt_state.dev_addr, 3U); frame_idx +=3U;
    /* 功能码 */
    frame_buf[frame_idx++] = SENSOR_FC_LEVEL_REPORT;
    /* 数据长度 */
    frame_buf[frame_idx++] = SENSOR_DATA_LEN;

    /*
     * 【7字节数据域，保持总长度不变】
     * offset0‑1: uint16_t 液位(大端)
     * offset2:   uint8_t 电池百分比
     * offset3:   uint8_t 月份 MM
     * offset4:   uint8_t 日期 DD
     * offset5:   uint8_t 小时 HH
     * offset6:   uint8_t 分钟 MM
     */
    frame_buf[frame_idx++] = (uint8_t)( (level_mm >> 8) & 0xFFU );
    frame_buf[frame_idx++] = (uint8_t)( level_mm & 0xFFU );
    frame_buf[frame_idx++] = bat_percent;

    /* 时间字段用【北京时间】—— 协议口径就是北京时间（网关 app/protocol/lora_business.c
     * 对这 4 个字节的注释是 "← 北京时间"，而且网关取不到 4G 时间时会直接采用设备报的值）。
     * RTC 内部跑的是 UTC，所以这里必须走 app_time_get_bj_calendar() 做 +8h；
     * ⚠️ 2026-10-09 修的 bug：原先是直接填 RTC 日历(=UTC)，上报出去整整差 8 小时。
     * RTC 还没校时则退回网关最近下发的原始北京时间（通常全 0）。 */
    if(app_time_get_bj_calendar(&cal) == 0U)
    {
        frame_buf[frame_idx++] = cal.month;
        frame_buf[frame_idx++] = cal.day;
        frame_buf[frame_idx++] = cal.hour;
        frame_buf[frame_idx++] = cal.minute;
    }
    else
    {
        frame_buf[frame_idx++] = rt_state.gateway_time.month;
        frame_buf[frame_idx++] = rt_state.gateway_time.day;
        frame_buf[frame_idx++] = rt_state.gateway_time.hour;
        frame_buf[frame_idx++] = rt_state.gateway_time.minute;
    }

    /* CRC16 Modbus：CRC覆盖从【版本字节】到【数据域末尾】；帧头不参与CRC */
    uint16_t crc = crc16_modbus(&frame_buf[2], frame_idx - 2U);
    frame_buf[frame_idx++] = (uint8_t)(crc &0xFFU);        /* CRC低字节 */
    frame_buf[frame_idx++] = (uint8_t)( (crc >>8) &0xFFU );/* CRC高字节 */

    memcpy(out_buf, frame_buf, frame_idx);
    *out_len = frame_idx;
    return true;
}

bool app_sensor_level_report_send_ex(uint16_t level_mm)
{
    uint8_t  frame_buf[SENSOR_REPORT_FRAME_MAX_LEN];
    uint16_t frame_len = 0U;

    if(!app_sensor_build_level_report(level_mm, frame_buf, &frame_len))
    {
        return false;
    }

    /* 发送：发完等ACK，没ACK按 LORA_SEND_RETRY 重发 */
    return LoraApp_SendWaitAck(frame_buf, (uint8_t)frame_len, LORA_SEND_RETRY);
}

bool app_sensor_level_report_send(void)
{
    /* 生产路径：液位来自液位传感器(没接/读不到时 Level_Get_Val() 返回 0xFFFF，
     * build 里会判无效直接不发) */
    return app_sensor_level_report_send_ex(Level_Get_Val());
}
