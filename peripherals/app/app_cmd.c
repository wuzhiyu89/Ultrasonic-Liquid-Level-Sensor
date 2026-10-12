#include "app_cmd.h"
#include "app_runtime_state.h"
#include "app_dev_param.h"
#include "crc16_modbus.h"
#include <string.h>
#include "app_time.h"
#include "protocol_util.h"
#include "log.h"
#include "delay.h"      /* delay_ms(): 收发模式切换等待用 */

/* 双槽命令队列，缓解网关连续下发两条命令时第二条被丢弃的问题 */
#define APP_CMD_QUEUE_SIZE  2U
static AppCmdMsg_t g_appCmdQueue[APP_CMD_QUEUE_SIZE];
static uint8_t     g_cmd_count = 0U;

void app_cmd_push_msg(const uint8_t *buf, uint16_t len)
{
    if ((buf == NULL) || (len == 0U) || (len > APP_CMD_BUF_MAX))
    {
        return;
    }
    /* 双槽全满则丢弃，依靠网关重传补偿 */
    if (g_cmd_count >= APP_CMD_QUEUE_SIZE)
    {
        return;
    }
    memcpy(g_appCmdQueue[g_cmd_count].rx_buf, buf, len);
    g_appCmdQueue[g_cmd_count].rx_len = len;
    g_appCmdQueue[g_cmd_count].ready = true;
    g_cmd_count++;
}

bool app_cmd_pop_msg(AppCmdMsg_t *outMsg)
{
    if (outMsg == NULL)
    {
        return false;
    }
    if (g_cmd_count == 0U)
    {
        return false;
    }
    memcpy(outMsg, &g_appCmdQueue[0], sizeof(AppCmdMsg_t));
    /* 前移剩余消息 */
    if (g_cmd_count > 1U)
    {
        memcpy(&g_appCmdQueue[0], &g_appCmdQueue[1], sizeof(AppCmdMsg_t));
    }
    g_cmd_count--;
    return true;
}

static bool app_cmd_build_ack_frame(uint8_t *ack_buf,
                                    uint16_t ack_buf_max,
                                    uint16_t *ack_len,
                                    uint8_t func_code,
                                    const uint8_t gw_addr[3],
                                    const uint8_t dev_addr[3],
                                    const uint8_t *data_ptr,
                                    uint8_t data_len)
{
    uint16_t frame_body_len = 2U + 1U + 3U + 3U + 1U + 1U + data_len;
    uint16_t total_len = frame_body_len + 2U;
    if (total_len > ack_buf_max)
    {
        *ack_len = 0U;
        return false;
    }
    uint8_t *p = ack_buf;
    *p++ = PROTO_HEADER_0;
    *p++ = PROTO_HEADER_1;
    *p++ = PROTO_VER_V1;
    memcpy(p, gw_addr, 3U);
    p += 3U;
    memcpy(p, dev_addr, 3U);
    p += 3U;
    *p++ = func_code;
    *p++ = data_len;
    if ((data_ptr != NULL) && (data_len > 0U))
    {
        memcpy(p, data_ptr, data_len);
        p += data_len;
    }
    crc16_modbus_append(&ack_buf[2], frame_body_len - 2U);
    *ack_len = total_len;

    /* 把要回给网关的应答帧原样打出来(台架对帧用) */
    {
        uint16_t i;
        log_info("[tx] ack func=0x%02X len=%u:", (unsigned)func_code, (unsigned)total_len);
        for (i = 0U; i < total_len; i++)
        {
            log_info(" %02X", (unsigned)ack_buf[i]);
        }
        log_info("\r\n");
    }
    return true;
}

/**
 * @brief 把 0x01 命令的 13 字节载荷解析成 CmdSetSensorParamData_t
 * @note  逐字节手工拼装, 不依赖结构体内存布局/对齐/主机字节序;
 *        多字节字段按协议规定为小端。
 */
static void app_cmd_parse_set_sensor_param(const uint8_t *data, CmdSetSensorParamData_t *out)
{
    out->detect_interval_s     = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
    out->report_interval_s     = (uint16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
    out->same_level_report_en  = data[4];
    out->level_alarm_thresh_mm = (uint16_t)((uint16_t)data[5] | ((uint16_t)data[6] << 8));
    out->year                  = data[7];
    out->month                 = data[8];
    out->day                   = data[9];
    out->week                  = data[10];
    out->hour                  = data[11];
    out->minute                = data[12];
}

/**
 * @brief 由【上报间隔】推导检测间隔（规则见 app_cmd.h【检测间隔的推导规则】）
 * @param report_s 上报间隔(秒)
 * @return 检测间隔(秒)，已经夹到 [APP_CMD_DETECT_INTERVAL_MIN_S, APP_CMD_DETECT_INTERVAL_MAX_S]
 * @note  云端只下发上报间隔，载荷里的检测间隔字段不可信（网关填的默认值），
 *        所以设备端一律自己算：detect = report / APP_CMD_DETECT_FROM_REPORT_DIV，
 *        再夹到 5 分钟 ~ 2 小时。
 */
#if (APP_CMD_DETECT_DERIVE_FROM_REPORT)
static uint16_t app_cmd_detect_from_report_s(uint32_t report_s)
{
    uint32_t detect_s = report_s / APP_CMD_DETECT_FROM_REPORT_DIV;

    if (detect_s < APP_CMD_DETECT_INTERVAL_MIN_S)
    {
        detect_s = APP_CMD_DETECT_INTERVAL_MIN_S;       /* 5 分钟下限 */
    }
    if (detect_s > APP_CMD_DETECT_INTERVAL_MAX_S)
    {
        detect_s = APP_CMD_DETECT_INTERVAL_MAX_S;       /* 2 小时上限 */
    }
    return (uint16_t)detect_s;
}
#endif

/**
 * @brief 【只允许主循环调用，禁止中断上下文！内部会执行Flash擦写】
 */
AppCmdRet_t app_cmd_process(const uint8_t *rx_buf,
                            uint16_t rx_len,
                            uint8_t *ack_buf,
                            uint16_t ack_buf_max,
                            uint16_t *ack_len)
{
    *ack_len = 0U;
    if (rx_buf == NULL || ack_buf == NULL)
    {
        return CMD_RET_ERR_PARAM;
    }
    const uint16_t FRAME_MIN_LEN = PROTO_FRAME_MIN_LEN;  /* 13 = 帧头2+版本1+地址6+功能码1+数据长度1+CRC2 */    
    if (rx_len < FRAME_MIN_LEN)
    {
        return CMD_RET_ERR_DATALEN;
    }
    if ((rx_buf[0] != PROTO_HEADER_0) || (rx_buf[1] != PROTO_HEADER_1))
    {
        return CMD_RET_ERR_HEADER;
    }
    uint8_t proto_ver = rx_buf[2];
    if (proto_ver != PROTO_VER_V1)
    {
        return CMD_RET_ERR_VER;
    }
    const ProtoDownFrame_t *p_frame = (const ProtoDownFrame_t *)rx_buf;
    uint8_t data_len = p_frame->data_len;
    uint16_t frame_without_crc_len = 2U + 1U + 3U + 3U + 1U + 1U + data_len;
    if ((frame_without_crc_len + 2U) != rx_len)
    {
        return CMD_RET_ERR_DATALEN;
    }
    if (crc16_modbus_check(&rx_buf[2], rx_len - 2U) == false)
    {
        /* 【排查用】把"期望的CRC"打出来，直接和网关发的帧尾对比：
         *   期望 = crc16_modbus(版本字节..数据域末尾)  <- 帧头 AA 55 不参与
         *   CRC16/Modbus: init=0xFFFF, poly=0xA001(反射), 低字节在前
         * 例：AA 55 01 11 00 01 21 00 01 80 00 | 期望 4A 76 */
        uint16_t calc = crc16_modbus(&rx_buf[2], rx_len - 2U);
        log_info("[cmd] CRC error: expect %02X %02X, got %02X %02X (frame_len=%u)\r\n",
                 (unsigned)(calc & 0xFFU), (unsigned)((calc >> 8U) & 0xFFU),
                 (unsigned)rx_buf[rx_len - 2U], (unsigned)rx_buf[rx_len - 1U],
                 (unsigned)rx_len);
        return CMD_RET_ERR_CRC;
    }
    /*===== 获取运行时本机地址，用于帧地址匹配 =====*/
    AppRuntimeState_t rt;
    app_runtime_state_get(&rt);
    /* 防御：未初始化有效设备地址直接丢弃包 */
    if (rt.dev_addr[0] == 0 && rt.dev_addr[1] == 0 && rt.dev_addr[2] == 0)
    {
        *ack_len = 0U;
        return CMD_RET_ERR_DST_ADDR;
    }
    if (memcmp(p_frame->dev_addr, rt.dev_addr, 3U) != 0U)
    {
        *ack_len = 0U;
        return CMD_RET_ERR_DST_ADDR;
    }

    /* 通过 CRC + 地址校验, 把整帧原样打出来(台架对帧用) */
    {
        uint16_t i;
        log_info("[rx] frame func=0x%02X dlen=%u len=%u:",
                 (unsigned)p_frame->func_code, (unsigned)data_len, (unsigned)rx_len);
        for (i = 0U; i < rx_len; i++)
        {
            log_info(" %02X", (unsigned)rx_buf[i]);
        }
        log_info("\r\n");
    }

    /* 这里【不】再单独等一段: 回 ACK 的等待由发送路径统一负责
     * (LoraApp_SendSync -> LoraApp_Send 内部等 LORA_TX_MODE_DELAY_MS=1s 再切发射),
     * 在这里再加一段会叠加成 2s, 容易超出网关的应答窗口。
     * 也就是: 收到消息 -> 解析组帧 -> 发送路径里等 1s -> 切 TX 发 ACK。 */

    ProtoFuncCode_t func_code = (ProtoFuncCode_t)p_frame->func_code;
    AppCmdRet_t ret_code = CMD_RET_OK;
    switch (func_code)
    {
    case CMD_FUNC_SET_SENSOR_PARAM:
    {
        /* 载荷必须正好是协议约定的 13 字节(方案A: 0-1检测/2-3上报/4相同上报/
         * 5-6阈值/7-12时间), 长度不符直接回 0x81, 避免按错位字节解析出垃圾参数 */
        if (data_len != APP_CMD_SET_SENSOR_PARAM_LEN)
        {
            ret_code = CMD_RET_ERR_DATALEN;
            break;
        }
        CmdSetSensorParamData_t rx_param;
        app_cmd_parse_set_sensor_param(p_frame->data, &rx_param);

        /* 载荷单位就是【秒】(2026-10-05 起，与网关口径一致，两边都不换算)；
         * 这里仍走 APP_CMD_INTERVAL_TO_S 只是为了保留"万一要回到分钟制"的单点开关，
         * 默认档位下它就是恒等映射，见 app_cmd.h【单位开关】 */
        uint32_t raw_detect_s = APP_CMD_INTERVAL_TO_S(rx_param.detect_interval_s);
        uint32_t report_s = APP_CMD_INTERVAL_TO_S(rx_param.report_interval_s);
        uint32_t detect_s;

        /* 【检测间隔不再取载荷里的值】云端只下发上报间隔，0x01 里那个检测间隔是
         * 网关拿默认值填的、不可信 -> 一律自己按 上报间隔/2 推导，再夹到
         * [5 分钟, 2 小时]，规则见 app_cmd.h【检测间隔的推导规则】 */
#if (APP_CMD_DETECT_DERIVE_FROM_REPORT)
        detect_s = (uint32_t)app_cmd_detect_from_report_s(report_s);
#else
        detect_s = raw_detect_s;
#endif

        log_info("[cmd] 0x01 set_sensor: report=%lus same=%u thresh=%umm\r\n",
                 (unsigned long)report_s,
                 (unsigned)rx_param.same_level_report_en,
                 (unsigned)rx_param.level_alarm_thresh_mm);
#if (APP_CMD_DETECT_DERIVE_FROM_REPORT)
        log_info("[cmd] 0x01 payload detect=%lus IGNORED -> detect=report/%u=%lus (clamp %u~%us)\r\n",
                 (unsigned long)raw_detect_s,
                 (unsigned)APP_CMD_DETECT_FROM_REPORT_DIV,
                 (unsigned long)detect_s,
                 (unsigned)APP_CMD_DETECT_INTERVAL_MIN_S,
                 (unsigned)APP_CMD_DETECT_INTERVAL_MAX_S);
#else
        log_info("[cmd] 0x01 payload detect=%lus (used as-is)\r\n",
                 (unsigned long)raw_detect_s);
#endif
        log_info("[cmd] 0x01 time=%02u-%02u-%02u w%u %02u:%02u\r\n",
                 (unsigned)rx_param.year, (unsigned)rx_param.month,
                 (unsigned)rx_param.day, (unsigned)rx_param.week,
                 (unsigned)rx_param.hour, (unsigned)rx_param.minute);

        /* 检测周期不能为 0/过小: 它是 STOP 的唤醒周期, 太小设备会一直在醒。
         * 【秒制】下这里就是直接比较秒数(存储字段是 uint16 秒)。
         * 上报间隔 = 0 直接拒: 它是推导检测间隔的唯一来源, 0 推导不出有意义的值,
         * 而且会污染后面"按上报间隔调度"的逻辑；回 0x81 让网关/平台知道下发的值不对。
         * ⚠️ 16 位字段上限 65535 秒 ≈ 18.2 小时。 */
        if ((report_s == 0U) ||
            (detect_s < APP_CMD_DETECT_INTERVAL_MIN_S) ||
            (detect_s > APP_CMD_DETECT_INTERVAL_MAX_S) ||
            (report_s > APP_CMD_SLEEP_INTERVAL_MAX_S))
        {
            log_info("[cmd] 0x01 rejected: interval out of range (report=%lus detect=%lus)\r\n",
                     (unsigned long)report_s, (unsigned long)detect_s);
            ret_code = CMD_RET_ERR_PARAM;
            break;
        }
        /* 年份只支持 2000~2099, 月份必须在 1~12 */
        if ((rx_param.year > 99U) ||
            (rx_param.month == 0U) || (rx_param.month > 12U))
        {
            ret_code = CMD_RET_ERR_PARAM;
            break;
        }

        /* 【重要】保存网关下发原始北京时间到RAM，用于LoRa上报，不做时区转换 */
        app_runtime_state_set_gateway_time(rx_param.year,
                                           rx_param.month,
                                           rx_param.day,
                                           rx_param.week,
                                           rx_param.hour,
                                           rx_param.minute);

        /* 组装北京时间日历 */
        bsp_rtc_calendar_t bj_cal;
        bj_cal.year = 2000U + rx_param.year;
        bj_cal.month = rx_param.month;
        bj_cal.day = rx_param.day;
        bj_cal.week = rx_param.week;
        bj_cal.hour = rx_param.hour;
        bj_cal.minute = rx_param.minute;
        bj_cal.second = 0U;

        uint32_t bj_ts = app_time_calendar_to_unix(&bj_cal);
        if (bj_ts == 0U)
        {
            // 日历非法
            ret_code = CMD_RET_ERR_PARAM;
            break;
        }
        // 北京时间转UTC，防止无符号下溢
        // （偏移量只认 app_time.h 的 APP_TIME_BJ_OFFSET_S，别在这里另写一遍 8*3600）
        if (bj_ts < APP_TIME_BJ_OFFSET_S)
        {
            ret_code = CMD_RET_ERR_PARAM;
            break;
        }
        uint32_t utc_ts = bj_ts - APP_TIME_BJ_OFFSET_S;

        if (app_time_set_unix_ts(utc_ts) != 0U)
        {
            ret_code = CMD_RET_ERR_PARAM;
            break;
        }
        log_info("[cmd] 0x01 rtc updated, unix_ts=%lu\r\n", (unsigned long)utc_ts);
        /* 时间不写Flash：STOP低功耗不会丢RAM，掉电后等网关重新校时即可 */

        /* 组装传感器配置，只更新RAM（不写Flash，避免频繁擦写） */
        DevSensorConfig_t new_cfg;
        new_cfg.detect_interval_s = (uint16_t)detect_s;
        new_cfg.report_interval_s = (uint16_t)report_s;
        new_cfg.same_level_report_en = rx_param.same_level_report_en;
        new_cfg.level_alarm_thresh_mm = rx_param.level_alarm_thresh_mm;
        /* 注：下发帧里的“探针阻值”字段已按需求从协议删除, 不再解析 */

        app_runtime_state_set_sensor_cfg(&new_cfg);
        ret_code = CMD_RET_OK;
    }
    break;

    case CMD_FUNC_UNBIND:
    {
        /* 0x05 解绑：载荷必须为空 —— 协议里 0x05 没有载荷，多字节直接判长度错。
         * 这里【只组 0x80 ACK 返回】，真正的"清网关地址 + 进 STOP 等下次上电"
         * 由 main.c 的 app_handle_rx_frame() 在【发完 ACK 之后】做。
         * 顺序约定：先回 ACK(毫秒级) -> 再清 Flash gw_addr=FF FF FF ->
         *            关 RTC 唤醒 + 关射频 -> STOP 等下次上电(不回配对)。
         * 注：清 Flash 不能放这里——ACK 还没发 gw_addr 就被清，与协议要求的
         *     "先回 ACK 再解绑"顺序相反；且 app_cmd_process() 只负责"解析+组应答"，
         *     副作用(清Flash/STOP)归 main.c，保持职责分离。 */
        if (data_len != APP_CMD_UNBIND_DLEN)
        {
            log_info("[cmd] 0x05 unbind: bad dlen=%u (expect 0)\r\n", (unsigned)data_len);
            ret_code = CMD_RET_ERR_DATALEN;
            break;
        }
        log_info("[cmd] 0x05 unbind: ACK built, main will clear gw_addr + STOP after send\r\n");
        ret_code = CMD_RET_OK;
    }
    break;

    case CMD_FUNC_PAIR_BIND:
        /* 0x04 是【设备发出去的】配对请求（目的地址 FF FF FF），网关不会下发给设备。
         * 万一收到，说明方向用反了，回 0x81 让对端知道不支持。 */
        log_info("[cmd] 0x04 pair_bind is uplink-only (device->gateway)\r\n");
        ret_code = CMD_RET_ERR_FUNCCODE;
        break;

    case CMD_FUNC_SENSOR_REPORT:
        ret_code = CMD_RET_ERR_FUNCCODE;
        break;
    case CMD_FUNC_SET_ALARM:
        ret_code = CMD_RET_OK;
        break;
    case CMD_FUNC_ACK_OK:
    case CMD_FUNC_ACK_ERR_RETRY:
        /* 收到的是网关对本机上报的应答：不用再回应答，否则会互相ACK停不下来 */
        *ack_len = 0U;
        return CMD_RET_OK;
    default:
        ret_code = CMD_RET_ERR_FUNCCODE;
        break;
    }
    uint8_t ack_func = (ret_code == CMD_RET_OK) ? CMD_FUNC_ACK_OK : CMD_FUNC_ACK_ERR_RETRY;
    /* 应答帧填充本机设备地址，从运行时状态获取 */
    bool build_ok = app_cmd_build_ack_frame(ack_buf, ack_buf_max, ack_len,
                                            ack_func,
                                            p_frame->gw_addr,
                                            rt.dev_addr,
                                            NULL, 0U);
    if (build_ok == false)
    {
        *ack_len = 0U;
        return CMD_RET_ERR_PARAM;
    }
    return ret_code;
}
