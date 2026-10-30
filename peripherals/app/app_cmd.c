#include "app_cmd.h"
#include "app_runtime_state.h"
#include "app_dev_param.h"
#include "crc16_modbus.h"
#include <string.h>
#include "app_time.h"
#include "protocol_util.h"
#include "log.h"
#include "delay.h"      /* delay_ms(): 收发模式切换等待用 */

bool app_cmd_build_ack_frame(uint8_t *ack_buf,
                            uint16_t ack_buf_max,
                            uint16_t *ack_len,
                            uint8_t func_code,
                            const uint8_t gw_addr[PROTO_ADDR_LEN],
                            const uint8_t dev_addr[PROTO_ADDR_LEN],
                            const uint8_t *data_ptr,
                            uint8_t data_len)
{
    uint16_t frame_body_len = PROTO_FIXED_HEAD_LEN + data_len;
    uint16_t total_len = frame_body_len + PROTO_CRC_LEN;
    if (total_len > ack_buf_max)
    {
        *ack_len = 0U;
        return false;
    }
    uint8_t *p = ack_buf;
    *p++ = PROTO_HEADER_0;
    *p++ = PROTO_HEADER_1;
    *p++ = PROTO_VER_V1;
    memcpy(p, gw_addr, PROTO_ADDR_LEN);         /* 13 字节：0x11 + 网关 UID */
    p += PROTO_ADDR_LEN;
    memcpy(p, dev_addr, PROTO_ADDR_LEN);        /* 13 字节：0x21 + 本机 UID */
    p += PROTO_ADDR_LEN;
    *p++ = func_code;
    *p++ = data_len;
    if ((data_ptr != NULL) && (data_len > 0U))
    {
        memcpy(p, data_ptr, data_len);
        p += data_len;
    }
    crc16_modbus_append(&ack_buf[2], frame_body_len - 2U);
    *ack_len = total_len;

    /* 把要回给网关的应答帧逐字节打出来 */
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
 * @brief 把 0x01 命令的 18 字节载荷解析成 CmdSetSensorParamData_t
 * @note  逐字节手工拼装，不依赖结构体布局/对齐/主机字节序；多字节字段按小端解析。
 *        上报间隔原样镜像 3 字节，拼成 uint32 交给 app_cmd_param_report_s()，
 *        这样结构体长度始终等于载荷长度，编译期断言才拦得住"改字段忘改长度"。
 */
static void app_cmd_parse_set_sensor_param(const uint8_t *data, CmdSetSensorParamData_t *out)
{
    out->report_interval_b[0]  = data[0];
    out->report_interval_b[1]  = data[1];
    out->report_interval_b[2]  = data[2];
    out->same_level_report_en  = data[3];
    out->level_alarm_thresh_mm = (uint16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));
    out->measure_mode          = data[6];
    out->install_height_cm     = (uint16_t)((uint16_t)data[7] | ((uint16_t)data[8] << 8));
    /* 校准偏移必须按【有符号】解析：0xFF EC = -20mm。
     * 先拼成 uint16 再强转 int16 —— 这是 C 里"按二进制原样解释成负数"的标准写法，
     * 不能写成 (int16_t)(data[9] | (data[10] << 8))，那样在某些编译器上会先做
     * 有符号提升，0xEC80 这种值的行为就不确定了。 */
    out->calib_offset_mm       = (int16_t)((uint16_t)((uint16_t)data[9] |
                                                     ((uint16_t)data[10] << 8)));
    out->low_battery_pct       = data[11];
    out->year                  = data[12];
    out->month                 = data[13];
    out->day                   = data[14];
    out->week                  = data[15];
    out->hour                  = data[16];
    out->minute                = data[17];
}

/**
 * @brief 由上报间隔推导检测间隔（规则见 app_cmd.h【检测间隔的推导规则】）
 * @param report_s 上报间隔(秒)
 * @return 检测间隔(秒)：ceil(report/3)，超过 2h 夹到 2h 上限
 * @note  全工程唯一推导入口：收 0x01 时打日志、main.c 睡前算睡眠周期都用它。
 *        返回值最大 7200，uint16 足够。
 */
uint16_t app_cmd_detect_from_report_s(uint32_t report_s)
{
    uint32_t detect_s = (report_s + (APP_CMD_DETECT_FROM_REPORT_DIV - 1U)) /
                        APP_CMD_DETECT_FROM_REPORT_DIV;     /* 向上取整 */

    if (detect_s > APP_CMD_DETECT_INTERVAL_MAX_S)
    {
        detect_s = APP_CMD_DETECT_INTERVAL_MAX_S;           /* 2 小时上限：周期内检测 >3 次 */
    }

    if (detect_s == 0U)
    {
        detect_s = APP_CMD_DETECT_INTERVAL_MAX_S;           /* 兜底（report=0 已在上游拒绝） */
    }
    return (uint16_t)detect_s;
}

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
    const uint16_t FRAME_MIN_LEN = PROTO_FRAME_MIN_LEN;  /* 33 = 帧头2+版本1+网关13+设备13+功能码1+数据长度1+CRC2 */
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
    uint16_t frame_without_crc_len = PROTO_FIXED_HEAD_LEN + data_len;
    if ((frame_without_crc_len + PROTO_CRC_LEN) != rx_len)
    {
        return CMD_RET_ERR_DATALEN;
    }
    if (crc16_modbus_check(&rx_buf[2], rx_len - 2U) == false)
    {
        /* 【排查用】把"期望的 CRC"打出来直接和帧尾对比：
         * 期望 = crc16_modbus(版本字节..数据域末尾)，帧头 AA 55 不参与；
         * CRC16/Modbus: init=0xFFFF, poly=0xA001(反射)，低字节在前 */
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
    /* 防御：本机设备地址没读到（UID 全 0）时直接丢弃包 */
    if (proto_addr_is_all_zero(rt.dev_addr))
    {
        *ack_len = 0U;
        return CMD_RET_ERR_DST_ADDR;
    }
    /* 目的地址：本机 或 广播(13×FF) 都收 —— 有些网关的 ACK 用广播地址回，
     * 只放行精确匹配会把它们当成"不是给本机的"丢掉 */
    if ((memcmp(p_frame->dev_addr, rt.dev_addr, PROTO_ADDR_LEN) != 0U) &&
        (!proto_addr_is_broadcast(p_frame->dev_addr)))
    {
        *ack_len = 0U;
        return CMD_RET_ERR_DST_ADDR;
    }

    /* CRC + 地址校验通过，逐字节打出入帧 */
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

    /* 回 ACK 的等待统一由发送路径负责（LoraApp_Send 内部等 LORA_TX_MODE_DELAY_MS），
     * 这里不再另加延时，否则会叠加成 1s 超出网关应答窗口。 */

    ProtoFuncCode_t func_code = (ProtoFuncCode_t)p_frame->func_code;
    AppCmdRet_t ret_code = CMD_RET_OK;
    switch (func_code)
    {
    case CMD_FUNC_SET_SENSOR_PARAM:
    {
        /* 载荷必须正好 18 字节（布局见 app_cmd.h），长度不符直接回 0x81，
         * 避免按错位字节解析出垃圾参数 */
        if (data_len != APP_CMD_SET_SENSOR_PARAM_LEN)
        {
            log_info("[cmd] 0x01 rejected: payload len %u != %u\r\n",
                     (unsigned)data_len, (unsigned)APP_CMD_SET_SENSOR_PARAM_LEN);
            ret_code = CMD_RET_ERR_DATALEN;
            break;
        }
        CmdSetSensorParamData_t rx_param;
        app_cmd_parse_set_sensor_param(p_frame->data, &rx_param);

        /* 载荷单位就是秒，与网关一致，两边都不换算。
         * ⚠️ report_s 必须是 uint32：uint24 满量程 16777215，用 uint16 接会静默截断。 */
        uint32_t report_s = app_cmd_param_report_s(&rx_param);
        uint32_t detect_s;

        /* 【只认上报间隔，先校验它】云端是唯一的节奏来源，这里也是唯一把关点。
         * 越界一律回 0x81，不静默夹取 —— 让云端界面能看见自己配错了。 */
        if ((report_s < APP_CMD_REPORT_INTERVAL_MIN_S) ||
            (report_s > APP_CMD_REPORT_INTERVAL_MAX_S))
        {
            log_info("[cmd] 0x01 rejected: report interval %lus out of [%u, %lu]\r\n",
                     (unsigned long)report_s,
                     (unsigned)APP_CMD_REPORT_INTERVAL_MIN_S,
                     (unsigned long)APP_CMD_REPORT_INTERVAL_MAX_S);
            ret_code = CMD_RET_ERR_PARAM;
            break;
        }

        /* 检测间隔一律现算，不落 Flash 也不进运行时状态（派生量，存下来只会和 report 打架） */
        detect_s = (uint32_t)app_cmd_detect_from_report_s(report_s);

        /* 【测量模式归一化】协议只定义 0=液位 / 1=距离，>1 按距离处理。
         * 距离模式没有安装高度就没法换算（会算成负数 -> 恒 0，表现为"永远空池"），
         * 所以缺高度时退回液位模式并打告警，不把坏配置带进主流程。 */
        uint8_t measure_mode = (rx_param.measure_mode != 0U) ? DEV_MEASURE_MODE_DIST
                                                             : DEV_MEASURE_MODE_LEVEL;
        if ((measure_mode == DEV_MEASURE_MODE_DIST) && (rx_param.install_height_cm == 0U))
        {
            log_info("[cmd] 0x01 !! measure_mode=1 but install_height=0 -> fallback to LEVEL mode\r\n");
            measure_mode = DEV_MEASURE_MODE_LEVEL;
        }

        /* 【校时字段全 0 = 这次不校时】网关取不到 4G 时间时会先填 0。
         * 这不是"非法参数"，必须放行，否则 4G 刚断开那一刻下发的整条命令都会被拒。 */
        uint8_t time_zero = (((rx_param.year == 0U) && (rx_param.month == 0U) &&
                              (rx_param.day == 0U) && (rx_param.week == 0U) &&
                              (rx_param.hour == 0U) && (rx_param.minute == 0U))) ? 1U : 0U;

        log_info("[cmd] 0x01 set_sensor: report=%lus -> detect=%lus (ceil(report/%u), cap %us)\r\n",
                 (unsigned long)report_s, (unsigned long)detect_s,
                 (unsigned)APP_CMD_DETECT_FROM_REPORT_DIV,
                 (unsigned)APP_CMD_DETECT_INTERVAL_MAX_S);
        log_info("[cmd] 0x01 same=%u thresh=%umm mode=%u(%s) height=%ucm offset=%dmm lowbat=%u%%\r\n",
                 (unsigned)rx_param.same_level_report_en,
                 (unsigned)rx_param.level_alarm_thresh_mm,
                 (unsigned)measure_mode,
                 (measure_mode == DEV_MEASURE_MODE_DIST) ? "DIST" : "LEVEL",
                 (unsigned)rx_param.install_height_cm,
                 (int)rx_param.calib_offset_mm,
                 (unsigned)rx_param.low_battery_pct);
        if (time_zero != 0U)
        {
            log_info("[cmd] 0x01 time=ALL-ZERO -> skip RTC sync (params still applied)\r\n");
        }
        else
        {
            log_info("[cmd] 0x01 time=%02u-%02u-%02u w%u %02u:%02u\r\n",
                     (unsigned)rx_param.year, (unsigned)rx_param.month,
                     (unsigned)rx_param.day, (unsigned)rx_param.week,
                     (unsigned)rx_param.hour, (unsigned)rx_param.minute);
        }

        /* 【校时字段校验】年份只支持 2000~2099，月/日/时/分必须落在合法区间，
         * 时间全 0 时跳过整段（见上面 time_zero）。
         * ⚠️ day=0 必须拒：time_convert_calendar_to_unix() 里 (day-1U) 是无符号运算，
         *    0-1 会回绕成 0xFFFFFFFF，时间戳直接飞到 2106 年，上报周期全乱。
         * week：0 = 网关没填（下面按日期反算），1~7 = 周一~周日，>7 直接拒。 */
        if ((time_zero == 0U) &&
            ((rx_param.year > 99U) ||
             (rx_param.month == 0U) || (rx_param.month > 12U) ||
             (rx_param.day == 0U) || (rx_param.day > 31U) ||
             (rx_param.week > 7U) ||
             (rx_param.hour > 23U) || (rx_param.minute > 59U)))
        {
            log_info("[cmd] 0x01 rejected: bad time %02u-%02u-%02u w%u %02u:%02u\r\n",
                     (unsigned)rx_param.year, (unsigned)rx_param.month,
                     (unsigned)rx_param.day, (unsigned)rx_param.week,
                     (unsigned)rx_param.hour, (unsigned)rx_param.minute);
            ret_code = CMD_RET_ERR_PARAM;
            break;
        }

        /* 只更新 RAM（不写 Flash，避免在收包窗口里擦写），落盘由 main.c 睡前统一做。
         * 放在校时之前：参数和校时是两件独立的事，校时失败不该让参数白下发。 */
        DevSensorConfig_t new_cfg;
        new_cfg.report_interval_s     = report_s;       /* uint24 -> uint32，不做窄化（不截断） */
        new_cfg.level_alarm_thresh_mm = rx_param.level_alarm_thresh_mm;
        new_cfg.calib_offset_mm       = rx_param.calib_offset_mm;   /* 有符号，可为负 */
        new_cfg.install_height_cm     = rx_param.install_height_cm;
        new_cfg.measure_mode          = measure_mode;   /* 已归一化，见上面 */
        new_cfg.same_level_report_en  = rx_param.same_level_report_en;
        new_cfg.low_battery_pct       = rx_param.low_battery_pct;
        new_cfg.rsv0                  = 0U;
        /* 注：检测间隔【不存】—— 它是 report 的派生量，由 app_cmd_detect_from_report_s() 现算 */
        app_runtime_state_set_sensor_cfg(&new_cfg);

        /* ---- 校时（时间字段全 0 时整段跳过，参数已经生效） ---- */
        if (time_zero == 0U)
        {
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
            /* week：0 = 网关没填 -> 先占位 1，等下面算出时间戳后按日期反算（日历转时间戳不看 week） */
            bj_cal.week = (rx_param.week == 0U) ? 1U : rx_param.week;
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
            if (rx_param.week == 0U)
            {
                /* 网关没填星期：按日期反算（1=周一..7=周日）。1970-01-01 是周四，
                 * 所以 (天数 + 3) % 7 + 1；RTC 的 week 只是给人看的，不影响计时。 */
                bj_cal.week = (uint8_t)((((bj_ts / 86400U) + 3U) % 7U) + 1U);
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
            /* 这里不立即写 Flash：RTC 时间由 main.c 定期落盘到独立时间戳页
             * （app_time_save_to_flash -> app_dev_param_set_persist_unix_ts），
             * 冷启动靠 app_time_init 从该页恢复 RTC，不必等网关重新校时。 */
        }

        ret_code = CMD_RET_OK;
    }
    break;

    case CMD_FUNC_UNBIND:
    {
        /* 0x05 解绑：载荷必须为空。这里【只组 0x80 ACK】，
         * "清网关地址 + 复位"由 main.c 在发完 ACK 之后做（副作用归调用方，保持职责分离）。 */
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

    case CMD_FUNC_SET_FIELD:
    {
        /* 0x06 SET_FIELD：写入 10 字节设备字段（ASCII），掉电不丢。
         * 只校验长度；写入成功回 0x80，Flash 擦写失败回 0x81。
         * 【幂等】网关 5s 内最多重发 3 条，每次都重写 Flash、都回 0x80。 */
        uint8_t di;

        if (data_len != APP_DEV_FIELD_LEN)
        {
            log_info("[cmd] 0x06 set_field: bad dlen=%u (expect %u)\r\n",
                     (unsigned)data_len, (unsigned)APP_DEV_FIELD_LEN);
            ret_code = CMD_RET_ERR_DATALEN;
            break;
        }
        log_info("[cmd] 0x06 set_field:");
        for (di = 0U; di < APP_DEV_FIELD_LEN; di++)
        {
            log_info(" %02X", (unsigned)p_frame->data[di]);   /* hex 打印，不依赖 %c */
        }
        log_info("\r\n");
        ret_code = app_dev_param_set_field(p_frame->data) ? CMD_RET_OK : CMD_RET_ERR_FLASH;
    }
    break;

    case CMD_FUNC_GET_FIELD:
        /* 0x07 GET_FIELD：不组普通 ACK，返回专门状态让 main.c 走
         * "发 0x08 -> 开 5s 窗口等 0x80 -> 没等到再发一次（最多 2 次）"的流程 */
        if (data_len != 0U)
        {
            log_info("[cmd] 0x07 get_field: bad dlen=%u (expect 0)\r\n", (unsigned)data_len);
            ret_code = CMD_RET_ERR_DATALEN;
            break;
        }
        log_info("[cmd] 0x07 get_field -> main will send 0x08 (gw uid from this frame)\r\n");
        *ack_len = 0U;
        return CMD_RET_FIELD_REQ;

    case CMD_FUNC_FIELD_RESP:
        /* 0x08 是【设备发出去的】字段应答，网关不会下发；收到说明方向用反了 */
        log_info("[cmd] 0x08 field_resp is uplink-only (device->gateway)\r\n");
        ret_code = CMD_RET_ERR_FUNCCODE;
        break;

    case CMD_FUNC_PAIR_BIND:
        /* 0x04 是【设备发出去的】配对请求（目的地址 13×FF），网关不会下发给设备。
         * 万一收到，说明方向用反了，回 0x81 让对端知道不支持。 */
        log_info("[cmd] 0x04 pair_bind is uplink-only (device->gateway)\r\n");
        ret_code = CMD_RET_ERR_FUNCCODE;
        break;

    case CMD_FUNC_SENSOR_REPORT:
        /* 0x02 是【设备发出去的】上报帧，网关不会下发 */
        ret_code = CMD_RET_ERR_FUNCCODE;
        break;
    case CMD_FUNC_SET_ALARM:
        /* 0x03 设置告警阈值：【本固件没有实现】，明确回 0x81。
         * 不能回 OK —— 那种"假成功"比报错难查得多（平台显示已下发、设备没改）。
         * 液位告警阈值请走 0x01 的 level_alarm_thresh_mm（off4-5）。 */
        log_info("[cmd] 0x03 set_alarm NOT implemented -> 0x81 (use 0x01 alarm_thresh)\r\n");
        ret_code = CMD_RET_ERR_FUNCCODE;
        break;
    case CMD_FUNC_ACK_OK:
    case CMD_FUNC_ACK_ERR_RETRY:
        /* 收到的是网关对本机上报的应答：不用再回应答，否则会互相ACK停不下来 */
        *ack_len = 0U;
        return CMD_RET_OK;
    default:
        /* 未知功能码：把"功能码 + 载荷"打出来，否则现场不知道网关发了什么 */
        {
            uint8_t ui;

            log_info("[cmd] !! unsupported func=0x%02X dlen=%u:", (unsigned)func_code,
                     (unsigned)data_len);
            for (ui = 0U; ui < data_len; ui++)
            {
                log_info(" %02X", (unsigned)p_frame->data[ui]);
            }
            log_info("\r\n");
        }
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
