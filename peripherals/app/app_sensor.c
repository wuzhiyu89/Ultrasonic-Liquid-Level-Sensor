#include "app_sensor.h"
#include "level_uart.h"
#include "app_power.h"
#include "app_time.h"
#include "app_lora.h"
#include "app_runtime_state.h"
#include "protocol_util.h"      /* PROTO_ADDR_LEN = 13（0x21/0x11 + UID） */
#include "crc16_modbus.h"
#include "log.h"
#include <string.h>

/* 上报帧总长 = 固定头 31 + 数据 7 + CRC 2 = 40 字节
 *   [0]AA [1]55 [2]ver [3..15]gw(13) [16..28]dev(13) [29]func=0x02 [30]dlen=7
 *   [31..37]液位高|液位低|电量%|月|日|时|分  [38][39]CRC
 * ⚠️ SENSOR_FRAME_OVERHEAD 已含 2 字节 CRC，别再额外加 2，否则会误报"缓冲区太小"。 */
#define SENSOR_REPORT_FRAME_MAX_LEN     40U
/* 协议常量 */
#define SENSOR_FRAME_HEAD0      0xAAU
#define SENSOR_FRAME_HEAD1      0x55U
#define SENSOR_PROTO_VER        0x01U
#define SENSOR_FC_LEVEL_REPORT  0x02U
#define SENSOR_DATA_LEN         0x07U
/* 数据域以外的固定开销：固定头31(AA55+ver+gw13+dev13+func+dlen) + CRC2 = 33 */
#define SENSOR_FRAME_OVERHEAD   33U

/* 液位换算：模块读数 -> 最终液位（测量模式 / 安装高度 / 校准偏移）
 *   ① LEVEL：模块读数就是液位（出厂默认）
 *   ② DIST ：模块读数是探头到水面的距离 -> 液位 = 安装高度(cm)×10 − 距离(mm)
 *   ③ 校准偏移叠加到结果上（可负），结果按 0 饱和
 * 返回值约定与 Level_Measure() 一致：0xFFFF = 无效。
 * ⚠️ 上限取 65534：0xFFFF 是"无效"哨兵，不能被真实液位占用。
 */
uint16_t app_sensor_convert_level(uint16_t raw_mm)
{
    AppRuntimeState_t rt;
    int32_t           v;

    if (raw_mm == 0xFFFFU)
    {
        return 0xFFFFU;                 /* 读数无效：原样透传，让上层按无效处理 */
    }

    app_runtime_state_get(&rt);

    if ((rt.sensor_cfg.measure_mode == DEV_MEASURE_MODE_DIST) &&
        (rt.sensor_cfg.install_height_cm != 0U))
    {
        /* ⚠️ 距离模式里 raw=0 不是"水面贴着探头"，而是【没有测到回波】：
         *    app_read_level() 在整轮采样全废时会兜底返回"上次有效值"，
         *    而首次上电那个兜底值就是 0。要是照公式换算就成了
         *    "液位 = 安装高度 - 0 = 满池"，凭空报一个满量程出去。
         *    模块最小量程也有几十厘米，真实距离不会是 0，所以这里直接按无效处理。 */
        if (raw_mm == 0U)
        {
            return 0U;
        }
        v = ((int32_t)rt.sensor_cfg.install_height_cm * 10) - (int32_t)raw_mm;
    }
    else
    {
        /* LEVEL 模式，或 DIST 但安装高度为 0（0x01 收到时就已归一化，这里再兜一次） */
        v = (int32_t)raw_mm;
    }

    v += (int32_t)rt.sensor_cfg.calib_offset_mm;    /* 有符号：负值 = 向下修正 */

    if (v < 0)
    {
        v = 0;                          /* 距离大于安装高度（水面低于探头/池底反射）-> 按 0 报 */
    }
    if (v > 65534)
    {
        v = 65534;                      /* 别占 0xFFFF（无效哨兵） */
    }
    return (uint16_t)v;
}


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

    /* 运行期自检：缓冲区要放得下 固定头+CRC(33) + 数据域(7)，改错长度直接返回 false */
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
    /* 网关地址13字节（0x11 + 网关 UID） */
    memcpy(&frame_buf[frame_idx], rt_state.gw_addr, PROTO_ADDR_LEN); frame_idx += PROTO_ADDR_LEN;
    /* 设备地址13字节（0x21 + 本机 UID） */
    memcpy(&frame_buf[frame_idx], rt_state.dev_addr, PROTO_ADDR_LEN); frame_idx += PROTO_ADDR_LEN;
    /* 功能码 */
    frame_buf[frame_idx++] = SENSOR_FC_LEVEL_REPORT;
    /* 数据长度 */
    frame_buf[frame_idx++] = SENSOR_DATA_LEN;

    /* 7 字节数据域：液位(大端) | 电量% | 月 | 日 | 时 | 分 */
    frame_buf[frame_idx++] = (uint8_t)( (level_mm >> 8) & 0xFFU );
    frame_buf[frame_idx++] = (uint8_t)( level_mm & 0xFFU );
    frame_buf[frame_idx++] = bat_percent;

    /* 时间字段用【北京时间】（协议口径，网关取不到 4G 时间时会直接采用设备报的值）。
     * RTC 内部跑 UTC，所以必须走 app_time_get_bj_calendar() 做 +8h；
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


