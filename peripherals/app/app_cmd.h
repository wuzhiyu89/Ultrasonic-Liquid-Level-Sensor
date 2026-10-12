/**
 * @file app_cmd.h
 * @brief 网关下行命令解析模块，只做帧解析，不保存业务状态
 * @note 只处理网关下发帧；上报帧0x02由app_sensor模块处理；禁止在中断调用
 */
#ifndef __APP_CMD_H__
#define __APP_CMD_H__
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

#define PROTO_HEADER_0          0xAAU
#define PROTO_HEADER_1          0x55U
#define PROTO_VER_V1            0x01U

#define APP_CMD_BUF_MAX         128U

/* ==================== 0x01 设置传感器参数 载荷布局 ====================
 * 网上载荷固定 13 字节, 与网关 app/protocol 侧完全一致(main.c 命令表):
 *   off0-1  detect_interval        小端 uint16  检测间隔，单位【秒】
 *          ⚠️ 2026-10-07 起【这个字段不再使用】：云端只下发上报间隔，
 *             网关那侧是拿默认值填的这一格（协议不好改、不转发平台值），
 *             所以设备端自己按上报间隔推导检测间隔，见下面【检测间隔的推导规则】。
 *   off2-3  report_interval        小端 uint16  上报间隔，单位【秒】（唯一可信的间隔）
 *   off4    same_level_report_en   1B           相同液位是否上报 0/1
 *   off5-6  level_alarm_thresh_mm  小端 uint16  液位告警阈值(mm)
 *   off7    year   (2000+)    off8  month
 *   off9    day               off10 week
 *   off11   hour              off12 minute
 * 注: 原结构体里的 probe_wakeup_res_ohm(探针阻值) 已按需求删除, 载荷从 15B 收窄到 13B。
 *     探针唤醒阻值以后要下发的话, 需重新约定(建议 data_len 扩到 15 并补 off13-14)。
 * 注: 设备内部【统一用秒】存(app_runtime_state 的 detect_interval_s/report_interval_s),
 *     载荷单位【也是秒】(2026-10-05 起, 与网关口径一致), 所以两边一致、不做换算;
 *     老的"分钟制"保留在下面【单位开关】里, 只在网关也回到分钟转发时才用。
 */
#define APP_CMD_SET_SENSOR_PARAM_LEN   13U

/* ==================== 【检测间隔的推导规则（2026-10-07）】 ====================
 * 背景（需求原话）："云端下发的只有上报时间间隔了，所以我们 lora 传来的检测
 *   时间间隔不可信（协议不好改，所以网关那边用的默认值）"。处理办法：
 *
 *       检测间隔 = 上报间隔 / APP_CMD_DETECT_FROM_REPORT_DIV        (= /2)
 *       再夹到 [APP_CMD_DETECT_INTERVAL_MIN_S, APP_CMD_DETECT_INTERVAL_MAX_S]
 *              = [300s(5 分钟), 7200s(2 小时)]
 *
 *   例：上报 1200s(20 分钟) -> 检测 600s(10 分钟)
 *       上报  600s(10 分钟) -> 检测 300s(5 分钟)
 *       上报  300s( 5 分钟) -> 检测 300s(被下限顶住, 不是 150s)
 *       上报 2 天(172800s)  -> 检测 7200s(被 2 小时上限顶住)
 *
 *   载荷里的 detect 字段仍然解析、仍然打印（日志里标 IGNORED + 打印推导结果），
 *   只当台架对帧参考，绝不写进运行时状态。
 *
 *   ⚠️ 生效范围：只作用于"收到 0x01 之后写进运行时状态的检测间隔"。
 *      出厂默认（还没收到云端配置时的 app_dev_param.c 30s/30s）不受影响 ——
 *      台架上电还没配云端也能 30s 一轮，方便调试。
 *      要让出厂默认也走这条规则，把 app_dev_param.c 的 report_interval_s
 *      改成 600U（推导出来正好是 300s）即可。
 *   ⚠️ 两个间隔都是 uint16 秒：上限 65535s，所以 2h 的上限在存储上没问题。
 *   ⚠️ 上报间隔 = 0 视为非法下发（回 0x81），不拿 0 去推导检测间隔。
 * ============================================================================ */
#define APP_CMD_DETECT_DERIVE_FROM_REPORT   1       /* 1 = 用 上报/2 推导（当前）；0 = 回到用载荷字段 */
#define APP_CMD_DETECT_FROM_REPORT_DIV      2U      /* 检测 = 上报 / 这个除数 */
#define APP_CMD_DETECT_INTERVAL_MIN_S       300U    /* 检测间隔下限 300s = 5 分钟 */
#define APP_CMD_DETECT_INTERVAL_MAX_S       7200U   /* 检测间隔上限 7200s = 2 小时 */

/* ==================== 【单位开关】两个间隔字段的单位 ====================
 * ★ 2026-10-05 起：协议字段的单位就是【秒】，两边都不做任何换算 ★
 *   依据（网关侧原文）：app/protocol/lora_business.c 的"用户口径改版"——
 *     平台下发的配置本来就是秒(reportPeriodSeconds, 5~3600s)，网关【原样填】进 0x01,
 *     并明确写了"协议字段=秒, 谁都不要换算; 任何一侧漏改 => 时间差 60 倍"。
 *   所以设备端也必须按秒解析、按秒直接用（内部本来就是秒：休眠周期、上报都按秒）。
 *
 *   APP_CMD_INTERVAL_UNIT_SECONDS = 1（【当前默认】载荷就是秒）
 *   APP_CMD_INTERVAL_UNIT_SECONDS = 0（2026-09 的旧"分钟制"：负载当分钟，内部 ×60）
 *       只有在【网关也回到"分钟转发"】时才允许改成 0。
 *
 * ⚠️ 两边必须成对，否则时间差 60 倍：
 *     网关发秒   + 设备按分钟解析 => 平台配 20 分钟(1200s) -> 设备睡 1200 分钟(20 小时)
 *     网关发分钟 + 设备按秒解析   => 平台配 20 分钟        -> 设备只睡 20 秒
 * ⚠️ 目前只有 0x01 带间隔字段（0x04 已改成配对绑定，不带间隔）。
 * ⚠️ 设备本地默认休眠周期（app_dev_param.c 的 detect_interval_s = 30 秒）是
 *    【秒】，属于本地参数，不受这个开关影响。
 */
#ifndef APP_CMD_INTERVAL_UNIT_SECONDS
#define APP_CMD_INTERVAL_UNIT_SECONDS   1
#endif

/* 载荷里的间隔值 -> 内部秒数 */
#if (APP_CMD_INTERVAL_UNIT_SECONDS)
#define APP_CMD_INTERVAL_TO_S(v)    ((uint32_t)(v))
#define APP_CMD_S_TO_INTERVAL(s)    ((uint16_t)(s))
#else
#define APP_CMD_INTERVAL_TO_S(v)    ((uint32_t)(v) * 60U)
#define APP_CMD_S_TO_INTERVAL(s)    ((uint16_t)((uint32_t)(s) / 60U))
#endif

/* 【网关侧 0x01 示例帧的 CRC 更正记录】
 *   网关工程(app/protocol/lora_business.c)里长期写着这一帧的 CRC 是 38 1B：
 *     AA 55 01 11 00 01 21 00 01 01 07 05 00 0A 00 00 DC 05 | 38 1B   ← 错
 *   正确值是 98 E0：
 *     AA 55 01 11 00 01 21 00 01 01 07 05 00 0A 00 00 DC 05 | 98 E0   ← 对
 *   （crc16_modbus(01 11 00 01 21 00 01 01 07 05 00 0A 00 00 DC 05) = 0xE098）
 *   0x01 载荷(13B, 小端): 检测间隔 5 / 上报间隔 10 / 相同液位上报 0 /
 *                        告警阈值 1500mm / 时间 后面 6 字节。
 *   ⚠️ 单位是【秒】(2026-10-05 起)：这一帧 = 检测 5 秒 / 上报 10 秒；
 *      在旧的"分钟制"下它才表示 5 分钟 / 10 分钟。
 */

/* ==================== 0x04 从机配对/绑定、0x05 从机解绑 ====================
 * 【0x04 配对绑定】方向：设备 -> 网关（上行）
 *   data_len = 0，载荷为空，设备自己的地址就在帧头的 dev_addr[3] 里。
 *   目的地址 gw_addr 填 FF FF FF（= 任意网关，谁收到谁应答）。
 *   设备行为（见 app_pair.c）：上电发现 Flash 里的网关地址无效(FF FF FF)
 *     就进配对模式，每 5s 发一帧 0x04；收到网关应答(0x80)后把应答帧头里的
 *     网关地址写进 Flash，下次上电直接进主程序。
 *
 * 【0x05 解绑】方向：网关 -> 设备（下行），data_len = 0
 *   设备行为（【2026-10-07 改版】见 main.c 的 app_handle_rx_frame()）：
 *     ① 回 0x80 ACK【两次】，两次之间隔 5s（APP_UNBIND_ACK_COUNT / _GAP_MS）
 *        —— 网关发完下行要切到接收，单次 ACK 可能落在它的切换盲区里
 *     ② 两次 ACK 都发完【之后】才把 Flash 里的网关地址写回 FF FF FF（顺序铁律：
 *        地址清掉就再也组不出应答帧了）
 *     ③ 关 RTC 周期唤醒 + 关探针唤醒 + 射频进 Sleep，然后反复进 STOP
 *        —— 本次运行内【不回配对、不发 0x04】，只有复位/重新上电才能退出；
 *           下次上电读到 gw_addr 无效，才自动进配对流程。
 *
 * 【台架对帧样例】CRC=Modbus(低字节在前)，覆盖范围：版本字节(01)到数据域末尾，
 * 帧头 AA 55 不参与。工具自检 crc16_modbus("123456789") == 0x4B37。
 *   设备(21 00 01) 发配对请求   AA 55 01 FF FF FF 21 00 01 04 00 BC 63
 *   网关(11 00 01) 回配对应答   AA 55 01 11 00 01 21 00 01 80 00 4A 76
 *                              （设备就是从这一帧的 [3..5] 取出 11 00 01 存进 Flash）
 *   网关(11 00 01) 下发解绑     AA 55 01 11 00 01 21 00 01 05 00 28 E6
 *   设备回解绑应答              AA 55 01 11 00 01 21 00 01 80 00 4A 76
 *                              （这一帧会【连发两次】，两次间隔 5s）
 *   换个设备(21 00 02) 发配对   AA 55 01 FF FF FF 21 00 02 04 00 4C 63
 *
 * 【配对模式收帧的判定条件（app_pair.c）】
 *   帧头 AA 55 + 版本 01 + 长度自洽 + CRC 通过
 *   目的 dev_addr == 本机设备地址 或 FF FF FF
 *   帧头 gw_addr 必须是有效地址（FF FF FF / 00 00 00 不接受，会一直重试）
 *   功能码 = 0x80(ACK_OK) 或 0x04(有的网关按原功能码回) -> 判定配对成功
 *   功能码 = 0x81(ACK_ERR) -> 网关拒绝，继续每 5s 重发
 *
 * 注意：0x04 没有"设置休眠周期"含义了（那是早期临时约定，已删除）。
 *       休眠/上报周期一律由 0x01 的 检测间隔/上报间隔 决定。
 */
#define APP_CMD_PAIR_DLEN               0U      /* 0x04 配对帧载荷长度(空) */
#define APP_CMD_UNBIND_DLEN             0U      /* 0x05 解绑帧载荷长度(空) */

/* 【已废弃】早期临时加的"0x04 设置休眠周期"指令已删除：
 * 0x04 现在是从机配对绑定，间隔类参数只走 0x01。 */
/* 休眠周期上限(s)：受 uint16 存储限制，最大 65535s(≈18.2h)。 */
#define APP_CMD_SLEEP_INTERVAL_MAX_S    65535U

/* 注意: 这里【不再】单独加"回 ACK 前的等待"。
 * 收到下行帧 -> 解析 -> LoraApp_SendSync() -> LoraApp_Send() 内部已经统一
 * 等 app_lora.h 的 LORA_TX_MODE_DELAY_MS(1000ms) 才切到发射。
 * 那 1s 就是"收到消息到发出应答"之间的全部等待; 在这里再加一段会和它叠加成
 * 2s(之前 APP_CMD_RX_SETTLE_MS 100ms + LORA_TX_MODE_DELAY_MS 100ms 就是这个毛病),
 * 容易超出网关的应答窗口。要调等待时间只改 LORA_TX_MODE_DELAY_MS 一处。 */

/* 命令消息缓存，从中断投递，主循环取出执行 */
typedef struct
{
    bool        ready;                     // true：存在待处理命令帧
    uint8_t     rx_buf[APP_CMD_BUF_MAX];
    uint16_t    rx_len;
}AppCmdMsg_t;

typedef enum
{
    CMD_FUNC_SET_SENSOR_PARAM   = 0x01U,
    CMD_FUNC_SENSOR_REPORT      = 0x02U,
    CMD_FUNC_SET_ALARM          = 0x03U,
    CMD_FUNC_PAIR_BIND          = 0x04U,   /* 配对绑定：设备->网关(目的FF FF FF)，载荷见上面 0x04 */
    CMD_FUNC_UNBIND             = 0x05U,   /* 解绑：网关->设备，载荷为空 */
    CMD_FUNC_ACK_OK             = 0x80U,
    CMD_FUNC_ACK_ERR_RETRY      = 0x81U,
}ProtoFuncCode_t;

typedef enum
{
    CMD_RET_OK              = 0U,
    CMD_RET_ERR_CRC         = 1U,
    CMD_RET_ERR_HEADER      = 2U,
    CMD_RET_ERR_VER         = 3U,
    CMD_RET_ERR_DST_ADDR    = 4U,
    CMD_RET_ERR_FUNCCODE    = 5U,
    CMD_RET_ERR_DATALEN     = 6U,
    CMD_RET_ERR_PARAM       = 7U,
    CMD_RET_ERR_FLASH       = 8U,
}AppCmdRet_t;

#pragma pack(1)
typedef struct
{
    uint16_t    detect_interval_s;      /* off0-1  小端 */
    uint16_t    report_interval_s;      /* off2-3  小端 */
    uint8_t     same_level_report_en;   /* off4 */
    uint16_t    level_alarm_thresh_mm;  /* off5-6  小端 */
    uint8_t     year;                   /* off7  (2000+) */
    uint8_t     month;                  /* off8  */
    uint8_t     day;                    /* off9  */
    uint8_t     week;                   /* off10 */
    uint8_t     hour;                   /* off11 */
    uint8_t     minute;                 /* off12 */
}CmdSetSensorParamData_t;
#pragma pack()

#pragma pack(1)
typedef struct
{
    uint8_t     header[2];
    uint8_t     proto_ver;
    uint8_t     gw_addr[3];
    uint8_t     dev_addr[3];
    uint8_t     func_code;
    uint8_t     data_len;
    uint8_t     data[];
}ProtoDownFrame_t;
#pragma pack()

/**
 * @brief 中断上下文调用：投递收到的LoRa下行命令帧到缓存
 * @param buf 接收数据指针
 * @param len 接收长度
 * @note 若已有未处理消息，直接丢弃新包，依靠网关重传补偿
 */
void app_cmd_push_msg(const uint8_t *buf, uint16_t len);

/**
 * @brief 主循环调用：取出待处理命令帧
 * @param outMsg 输出消息结构体
 * @retval true 存在待处理消息；false无消息
 */
bool app_cmd_pop_msg(AppCmdMsg_t *outMsg);

/**
 * @brief 命令帧解析处理，【只允许在主循环上下文调用！禁止中断内调用】
 */
AppCmdRet_t app_cmd_process(const uint8_t *rx_buf,
                            uint16_t rx_len,
                            uint8_t *ack_buf,
                            uint16_t ack_buf_max,
                            uint16_t *ack_len);

#ifdef __cplusplus
}
#endif
#endif
