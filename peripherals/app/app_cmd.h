/**
 * @file app_cmd.h
 * @brief 网关下行命令解析模块，只做帧解析与组应答帧，不保存业务状态
 * @note 只处理网关下发帧；上报帧 0x02 由 app_sensor 模块处理；禁止在中断里调用
 */
#ifndef __APP_CMD_H__
#define __APP_CMD_H__

#include <stdint.h>
#include <stdbool.h>
#include "protocol_util.h"      /* 13 字节地址格式(0x21/0x11 + UID) + 帧内偏移宏 */

#ifdef __cplusplus
extern "C" {
#endif

#define PROTO_HEADER_0          0xAAU
#define PROTO_HEADER_1          0x55U
#define PROTO_VER_V1            0x01U

#define APP_CMD_BUF_MAX         128U

/* ==================== 0x01 设置传感器参数：载荷固定 18 字节 ====================
 * 与网关 N32H482 侧完全一致（网关 app/links/lora/lora_downlink.c:
 * LORA_SENSOR_PARAM_LEN = 18 / TIME_OFF = 12）。
 *
 *   off0-2   report_interval        小端 uint24  上报间隔，单位【秒】（唯一权威的间隔）
 *   off3     same_level_report_en   1B           相同液位是否上报（网关恒填 1，设备不解析）
 *   off4-5   level_alarm_thresh_mm  小端 uint16  液位告警阈值(mm)，0 = 关闭告警
 *   off6     measure_mode           1B           0 = 液位模式，1 = 距离模式
 *   off7-8   install_height_cm      小端 uint16  安装高度(cm)，仅距离模式参与换算
 *   off9-10  calib_offset_mm        小端 int16   校准偏移(mm)，【有符号】
 *   off11    low_battery_pct        1B           低电量阈值(%)，0 = 禁用（判定在网关侧）
 *   off12-17 year/month/day/week/hour/minute      北京时间，week 1 = 周一
 *
 * 语义约定：
 *   ① 检测间隔不在载荷里 —— 一律由上报间隔推导（见下面【检测间隔的推导规则】）。
 *   ② 「相同液位是否上报」设备端不解析：现行行为就是"上报点到了就发，液位一样也发"，
 *      与网关恒填 1 等价，两端都不要依赖它。
 *   ③ 校时字段全 0 = "这次不校时"（网关取不到 4G 时间时会先填 0）：其余字段照常生效、
 *      仍回 0x80。绝不能把 month=0 当非法参数拒掉整条命令。
 *   ④ 载荷单位就是秒（与网关一致），两边都不做换算。
 *   ⑤ 长度不符（任何非 18 字节）直接回 0x81(ACK_ERR_RETRY)，绝不按错位字节解析。
 */
#define APP_CMD_SET_SENSOR_PARAM_LEN   18U

/* ==================== 检测间隔的推导规则 ====================
 * 节奏全部听云端：云端只下发上报间隔，设备自己算检测间隔，规则只有一条
 * "一个上报周期检测 3 次"（第 3 次就是开上报窗口那一次）。
 *
 *   d = ceil(report / APP_CMD_DETECT_FROM_REPORT_DIV)      = ceil(report/3)
 *   若 d > APP_CMD_DETECT_INTERVAL_MAX_S(7200s) -> d = 7200s（本周期内检测更多次，越限更早暴露）
 *   没有下限：云端配 3 分钟上报就是 1 分钟检测。唯一兜底是上报间隔本身的合法区间。
 *
 *   例：180s -> d=60s（3 次）；1800s -> d=600s（3 次）；12h -> 夹到 7200s（6 次）
 *       7 天 -> 夹到 7200s（84 次）；194 天（满量程）-> 夹到 7200s（2330 次）
 *
 * 为什么向上取整：sleep = min(d, 距上报点剩余) 会把最后一觉对齐到上报点，
 *   取整后 3*d >= report，正好 3 次检测落在周期内；向下取整会多出第 4 次苏醒。
 *
 * ⚠️ d 是算出来的，不落 Flash：设备侧只有 report_interval_s 一个持久化间隔字段，
 *    检测间隔每次用时现算，不存在"Flash 里的 detect 和 report 对不上"。
 */
#define APP_CMD_DETECT_FROM_REPORT_DIV      3U      /* 检测 = 上报 / 这个除数 */
#define APP_CMD_DETECT_INTERVAL_MAX_S       7200U   /* 检测间隔上限 2 小时 */

/* 上报间隔的合法区间：0x01 里越界直接回 0x81（不静默夹取，让云端看得见配错了）
 *   下限 60s：一轮唤醒（POR 1s + 探针 + 收发窗口）就要十几秒，更短等于连续工作。
 *   上限 16777215s：3 字节满量程（≈194 天）。它不是睡眠上限 —— 单次 STOP 最多
 *             65535s，更长的间隔会拆成多段睡，每段醒来判水/巡检，到点才真发。 */
#define APP_CMD_REPORT_INTERVAL_MIN_S       60U
#define APP_CMD_REPORT_INTERVAL_MAX_S       16777215UL

/* ==================== 0x01 示例帧（载荷 18B） ====================
 * 占位 UID：设备 = 01 02 .. 0C，网关 = 21 22 .. 2C。
 * 内容：上报 90s(0x00005A 小端 5A 00 00) / 相同液位 1 / 阈值 500mm(0x01F4) /
 *       距离模式(1) / 安装高度 5cm(0x0005) / 校准偏移 +20mm(0x0014) / 低电量 0 /
 *       时间 2026-10-22 周四 14:30。整帧 51 字节（时间字段仅示例）。
 *     AA 55 01
 *     11 21 22 23 24 25 26 27 28 29 2A 2B 2C      <- 网关 0x11 + UID(12B)
 *     21 01 02 03 04 05 06 07 08 09 0A 0B 0C      <- 设备 0x21 + UID(12B)
 *     01 12                                        <- func=0x01 / data_len=18
 *     5A 00 00 | 01 | F4 01 | 01 | 05 00 | 14 00 | 00           <- 载荷前 12B
 *     1A 0A 16 04 0E 1E                                          <- 时间 6B
 *     91 0E                                        <- CRC16/Modbus，低字节在前
 * CRC 覆盖"版本字节到载荷末尾"（AA 55 不参与）；自检 crc16_modbus("123456789") == 0x4B37。
 */

/* ==================== 0x04 配对绑定 / 0x05 解绑 ====================
 * 【0x04】设备 -> 网关，dlen=0，本机地址在帧头 dev_addr 里（0x21 + 96 位 UID），
 *   gw_addr 填 13×FF = 任意网关。
 *   上电发现 Flash 里网关地址无效就进配对（app_pair.c）：每 5s 发一帧 0x04，
 *   收到网关应答(0x80)后把应答帧头里的网关地址(0x11 + 网关 UID)写进 Flash。
 *
 * 【0x05】网关 -> 设备，dlen=0。设备行为（main.c 的 app_handle_rx_frame）：
 *   ① 回 0x80 两次，间隔 5s —— 网关发完下行要切接收，单次 ACK 可能落在它的盲区里
 *   ② 两次 ACK 都发完【之后】才把 Flash 里的网关地址写回 13×FF（顺序铁律）
 *   ③ 关 RTC 唤醒 + 探针唤醒 + 射频进 Sleep，复位后走出厂流程
 *
 * 【配对帧样例】CRC 覆盖版本字节(01)到数据域末尾，帧头 AA 55 不参与：
 *   设备(0x21+UID) 发配对请求，33B：
 *     AA 55 01 | FF×13 | 21 01 02 03 04 05 06 07 08 09 0A 0B 0C | 04 | 00 | C8 46
 *   网关(0x11+UID) 回配对应答，33B：
 *     AA 55 01 | 11 21 22 23 24 25 26 27 28 29 2A 2B 2C | 21 .. 0C | 80 | 00 | C7 29
 *   网关下发解绑，33B：同上网关帧，功能码 05、CRC = A5 B9
 *   设备回解绑应答：与"网关回配对应答"逐字节相同（0x80，连发两次）
 *
 * 【配对模式收帧判定（app_pair.c）】帧头 + 版本 + 长度自洽 + CRC 通过；
 *   目的 dev_addr = 本机或 13×FF；帧头 gw_addr 必须是有效网关地址；
 *   功能码 0x80 或 0x04 = 配对成功，0x81 = 网关拒绝继续重发。
 *
 * 帧长：固定头 31 字节；无载荷帧(0x04/0x05/0x80) 33 字节；0x02 上报帧 40 字节。
 *
 * 【0x06 SET_FIELD / 0x07 GET_FIELD / 0x08 FIELD_RESP】设备字段 = 10 字节 ASCII，
 * 掉电不丢（Flash 参数页，出厂默认 "YEWEI00000"）。
 *   - 0x06（网关->设备，dlen=10）：写入 Flash 后回 0x80；dlen 不对或写失败回 0x81。
 *         【幂等】网关 5s 内最多重发 3 条，每次都重写、每次都要回 0x80。
 *   - 0x07（网关->设备，dlen=0）：读字段请求。设备立刻发 0x08，再开 5s 窗口等网关 0x80；
 *         没等到就再发一次 0x08 + 再等 5s；最多 2 次，之后回主流程。
 *   - 0x08（设备->网关，dlen=10）：gw_uid 取 0x07 帧头、dev_uid 用本机。
 *   示例（真实 UID：设备 21 36 06 11 30 32 39 36 31 31 01 7C 42，
 *              网关 11 36 08 11 50 41 39 43 32 30 09 2D 2C）：
 *     0x06 (43B)  AA 55 01 <gw13> 21 36 .. 7C 42 06 0A 41 42 43 44 45 31 32 33 34 35 49 0A
 *     0x07 (33B)  ... 07 00 4F A5
 *     0x08 (43B)  ... 08 0A 59 45 57 45 49 30 30 30 30 30 18 DA（"YEWEI00000"）
 */
#define APP_CMD_PAIR_DLEN               0U      /* 0x04 配对帧载荷长度(空) */
#define APP_CMD_UNBIND_DLEN             0U      /* 0x05 解绑帧载荷长度(空) */

/* 单次 STOP 的睡眠上限(s)：RTC 唤醒计数器只有 16 位，最多 65535s(≈18.2h)。
 * ⚠️ 它管的是"一觉睡多久"，不是"上报间隔能配多长"（见 APP_CMD_REPORT_INTERVAL_MAX_S）。 */
#define APP_CMD_SLEEP_INTERVAL_MAX_S    65535U

/* 注意：回 ACK 前不再单独加等待 —— LoraApp_Send() 内部已统一等
 * LORA_TX_MODE_DELAY_MS(500ms) 才切到发射，在这里再加一段会叠加成 1s，
 * 容易超出网关的应答窗口。要调等待只改 LORA_TX_MODE_DELAY_MS 一处。 */

typedef enum
{
    CMD_FUNC_SET_SENSOR_PARAM   = 0x01U,
    CMD_FUNC_SENSOR_REPORT      = 0x02U,
    CMD_FUNC_SET_ALARM          = 0x03U,
    CMD_FUNC_PAIR_BIND          = 0x04U,   /* 配对绑定：设备->网关(目的 13×FF) */
    CMD_FUNC_UNBIND             = 0x05U,   /* 解绑：网关->设备，载荷为空 */
    CMD_FUNC_SET_FIELD          = 0x06U,   /* 网关->设备：写入 10 字节设备字段，回 0x80/0x81 */
    CMD_FUNC_GET_FIELD          = 0x07U,   /* 网关->设备：读字段请求，设备回 0x08 */
    CMD_FUNC_FIELD_RESP         = 0x08U,   /* 设备->网关：字段应答(dlen=10)，设备主动发 */
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
    CMD_RET_FIELD_REQ       = 9U,   /* 收到 0x07：不走普通 ACK，由 main.c 走 0x08 流程 */
}AppCmdRet_t;

/* 0x01 载荷解析结果（逐字节填，不依赖对齐）
 * 注：上报间隔用 uint8_t[3] 原样镜像 uint24 而不是 uint32 —— 用 uint32 会多占 1 字节，
 *     结构体就不再是载荷镜像，下面的编译期断言也就失去意义。要秒数请调
 *     app_cmd_param_report_s()。
 * 注：字段顺序【就是载荷顺序】，pack(1) 下 sizeof 必须 == 18。 */
#pragma pack(1)
typedef struct
{
    uint8_t     report_interval_b[3];   /* off0-2  小端 uint24（秒） */
    uint8_t     same_level_report_en;   /* off3    设备不解析（行为恒等于"相同也报"） */
    uint16_t    level_alarm_thresh_mm;  /* off4-5  小端，0 = 关闭告警 */
    uint8_t     measure_mode;           /* off6    0=液位 1=距离（>1 按 1 处理） */
    uint16_t    install_height_cm;      /* off7-8  小端，厘米 */
    int16_t     calib_offset_mm;        /* off9-10 小端，【有符号】 */
    uint8_t     low_battery_pct;        /* off11   %，0 = 禁用（判定在网关侧） */
    uint8_t     year;                   /* off12  (2000+) */
    uint8_t     month;                  /* off13  */
    uint8_t     day;                    /* off14  */
    uint8_t     week;                   /* off15  1=周一 .. 7=周日，0=网关没填 */
    uint8_t     hour;                   /* off16  */
    uint8_t     minute;                 /* off17  */
}CmdSetSensorParamData_t;
#pragma pack()

/* 编译期断言：结构体必须和协议载荷等长，改字段忘改长度会直接编译失败 */
typedef char app_cmd_param_size_check[
    (sizeof(CmdSetSensorParamData_t) == APP_CMD_SET_SENSOR_PARAM_LEN) ? 1 : -1];

/**
 * @brief 把解析结果里的 uint24 上报间隔拼成秒数
 * @param p 0x01 解析结果
 * @return 上报间隔(秒)，0 ~ 16777215
 */
static inline uint32_t app_cmd_param_report_s(const CmdSetSensorParamData_t *p)
{
    return (uint32_t)p->report_interval_b[0]
         | ((uint32_t)p->report_interval_b[1] << 8)
         | ((uint32_t)p->report_interval_b[2] << 16);
}

#pragma pack(1)
typedef struct
{
    uint8_t     header[2];
    uint8_t     proto_ver;
    uint8_t     gw_addr[PROTO_ADDR_LEN];    /* off3..15 ：0x11 + 网关 UID（13 字节） */
    uint8_t     dev_addr[PROTO_ADDR_LEN];   /* off16..28：0x21 + 设备 UID（13 字节） */
    uint8_t     func_code;                  /* off29 */
    uint8_t     data_len;                   /* off30 */
    uint8_t     data[];                     /* off31..  */
}ProtoDownFrame_t;
#pragma pack()

/**
 * @brief 命令帧解析处理，【只允许在主循环上下文调用！禁止中断内调用】
 */
AppCmdRet_t app_cmd_process(const uint8_t *rx_buf,
                            uint16_t rx_len,
                            uint8_t *ack_buf,
                            uint16_t ack_buf_max,
                            uint16_t *ack_len);

/**
 * @brief 组一帧上行/应答帧：AA 55 | ver | gw(13) | dev(13) | func | dlen | data | CRC
 * @param ack_buf     输出缓冲
 * @param ack_buf_max 输出缓冲容量
 * @param ack_len     [out]整帧长度
 * @param func_code   功能码（0x80/0x81 应答，或 0x08 字段应答）
 * @param gw_addr     帧头网关地址（13B）
 * @param dev_addr    帧头设备地址（13B）
 * @param data_ptr    载荷（可为 NULL）
 * @param data_len    载荷长度
 * @retval true 成功；false 缓冲不够
 * @note  main.c 发 0x08 FIELD_RESP 也调它（网关 UID 取自 0x07 帧头）
 */
bool app_cmd_build_ack_frame(uint8_t *ack_buf,
                             uint16_t ack_buf_max,
                             uint16_t *ack_len,
                             uint8_t func_code,
                             const uint8_t gw_addr[PROTO_ADDR_LEN],
                             const uint8_t dev_addr[PROTO_ADDR_LEN],
                             const uint8_t *data_ptr,
                             uint8_t data_len);

/**
 * @brief 由上报间隔推导检测间隔（全工程唯一入口，规则见本文件【检测间隔的推导规则】）
 * @param report_s 上报间隔(秒)，有效范围 [60, 16777215]（0x01 里已校验）
 * @retval 检测间隔(秒)：ceil(report/3)，超过 7200s 时夹到 7200s
 * @note  返回值不超过 7200，用 uint16 返回安全；检测间隔不落 Flash（派生量）
 */
uint16_t app_cmd_detect_from_report_s(uint32_t report_s);

#ifdef __cplusplus
}
#endif
#endif
