/**
 * @file app_dev_param.h
 * @brief 设备持久化参数：网关地址(配对写入/解绑清除) + 传感器配置 + 设备字段，存 Flash 掉电不丢
 *
 * 【地址模型】
 *   设备地址 = 0x21 + 96 位 UID（13 字节），由芯片 UID 每次上电现算，不落 Flash，
 *   也没有产线写址接口（见 app_dev_param_build_dev_addr()）。
 *   Flash 里只存 13 字节网关地址 = 0x11 + 网关 UID；13×FF = 未绑定。
 *
 * 【配对/绑定与 Flash 的关系】
 *   出厂/解绑后：Flash 网关地址 = 13×FF。
 *   上电：gw_addr 无效（前缀不是 0x11，或全 FF/全 00）-> 进配对模式（app_pair.c），
 *         每 5s 发一帧 0x04（目的 13×FF），收到 0x80 后把帧头网关地址写进 Flash。
 *   解绑：0x05 -> 先回 2 次 ACK(间隔 5s) -> 网关地址写回 13×FF -> 关唤醒/关射频 -> 复位。
 */
#ifndef __APP_DEV_PARAM_H__
#define __APP_DEV_PARAM_H__
#include <stdint.h>
#include <stdbool.h>
#include "protocol_util.h"
#ifdef __cplusplus
extern "C" {
#endif

/* ========= Flash参数存储配置：落在 n32g031.sct 预留的参数区里，避开代码段 =========
 * 参数区 = 0x0800F800 ~ 0x0800FFFF（2KB，见 MDK-ARM/n32g031.sct 的 LR_PARAM_STORAGE，
 * 代码只用到 0x0800F7FF）；该地址正好是芯片页的起点 —— N32G031 页大小 512B=0x200
 * （见厂商 Flash 例程的 FLASH_PAGE_SIZE），store/erase 都按这一页走。 */
#define DEV_PARAM_FLASH_PAGE_ADDR    0x0800F800U
/* 魔术字，用来判断Flash参数是否有效 */
#define DEV_PARAM_MAGIC              0xA55AA55AU

/* ============ 时间戳单独一页 ============
 * 时间戳每小时写一次，如果和网关地址/设备字段/传感器配置挤在同一页，
 * 每次都要"擦整页 -> 整页重写"，擦写中掉电会把绑定关系一起抹掉
 * （现场表现："设备突然变回未绑定，要重新配对"）。
 * 所以时间戳搬到紧邻的第二页，各写各的：写坏最多丢时间（网关下次 0x01 会补）。
 * 参数区 0x0800F800~0x0800FFFF 共 2KB = 4 个 512B 页：
 *   F800 参数页（本结构体） / FA00 时间戳页 / FC00、FE00 预留。 */
#define DEV_PARAM_TS_PAGE_ADDR       0x0800FA00U
/* 时间戳页的魔术字（和参数页各认各的） */
#define DEV_PARAM_TS_MAGIC           0x5A5AC33CU
/* 参数格式版本，写在 DevParamFlash_t.reserved[0]。
 * 改了 DevParamFlash_t 布局或出厂默认值时必须 +1：上电发现版本不一致，
 * 就把传感器配置 / 设备字段刷成新的出厂默认值再整页写回（网关地址保留，
 * 不清绑定关系）。不靠"结构体大小变没变"判断 —— 大小不变但字段挪位的情况一样有。 */
#define DEV_PARAM_CFG_VER            1U

/* 测量模式（0x01 的 off6）：决定"超声模块读数怎么变成液位"
 *   本机超声模块（AJ-SR04M / DYP 类）串口帧输出的是【探头到液面的距离】，所以：
 *     LEVEL：模块读数直接当液位（模块内部已换算，或探头朝上装）
 *     DIST ：液位 = 安装高度(cm)×10 - 距离(mm)
 *   两种之外的取值按 DIST（app_cmd.c 归一化：非 0 一律按 DIST，网关侧 `!=0 ? 1 : 0`）。 */
#define DEV_MEASURE_MODE_LEVEL       0U
#define DEV_MEASURE_MODE_DIST        1U

/**
 * @brief 传感器持久化配置参数【运行时可改，保存在Flash里，断电不丢失】
 * @note  收到 0x01 时先经 app_runtime_state 只更新 RAM（避免在收包窗口里擦写）；
 *        Flash 落盘由 main.c 睡前调 app_dev_param_set_sensor_cfg() 完成
 *        （该函数内部有去重，配置没变就不擦写）
 */
typedef struct
{
    uint32_t    report_interval_s;      /* 上报间隔，单位秒（0x01 off0-2 uint24，合法 [60,16777215]）
                                         * 这是【唯一持久化的间隔字段】：检测间隔 = ceil(report/3)
                                         * 由它现算，不存 Flash。 */
    uint16_t    level_alarm_thresh_mm;  /* 液位告警阈值 mm；0 = 关闭告警 */
    int16_t     calib_offset_mm;        /* 校准偏移 mm（有符号），叠加到换算后的液位，
                                         * 结果按 [0, 65534] 饱和（见 app_sensor_convert_level） */
    uint16_t    install_height_cm;      /* 安装高度 cm（探头到池底），仅 DIST 模式参与换算，换算时 ×10 */
    uint8_t     measure_mode;           /* 测量模式：DEV_MEASURE_MODE_LEVEL / _DIST */
    uint8_t     same_level_report_en;   /* 相同液位是否上报：设备只存不用（行为恒等于"一样也报"） */
    uint8_t     low_battery_pct;        /* 低电量阈值 %：只存不用，判定在网关侧 */
    uint8_t     rsv0;                   /* 对齐填充，保证 sizeof 是 4 的倍数 */
}DevSensorConfig_t;

/* 设备字段长度（0x06/0x08 的 data_len，固定 10 字节 ASCII） */
#define APP_DEV_FIELD_LEN            10U

/**
 * @brief 存在 Flash 参数页里的参数结构体（4 字节对齐）
 * @note  设备地址不在结构里（由 UID 现算）；gw_addr 出厂/解绑后为 13×FF。
 */
typedef struct
{
    uint32_t magic;                     /* 魔术字，校验是否有效 */
    uint8_t gw_addr[PROTO_ADDR_LEN];    /* 13 字节网关地址(0x11+网关UID)：13×FF = 未绑定 */
    uint8_t rsv1[2];                    /* 占位：让 sensor_cfg 落在 4 字节对齐偏移（不足 1 字节由编译器补） */
    DevSensorConfig_t sensor_cfg;       /* 传感器业务参数（运行可改） */
    uint8_t reserved[4];                /* reserved[0]=DEV_PARAM_CFG_VER，其余预留 */
    uint8_t field[APP_DEV_FIELD_LEN];   /* 设备字段 10 字节 ASCII（0x06 写 / 0x08 回） */
}DevParamFlash_t;

/* 编译期断言：Flash写入要求4字节对齐，打包编译时提前报错 */
typedef char dev_param_align_check[(sizeof(DevParamFlash_t) % 4U == 0U) ? 1 : -1];

/**
 * @brief 初始化设备参数，上电调用：读取flash，失败加载出厂默认
 */
void app_dev_param_init(void);

/**
 * @brief 获取本机设备地址（由 96 位 UID 现算，不是从 Flash 读的）
 * @param out_addr [out]输出13字节地址缓存：0x21 + UID(12B)
 */
void app_dev_param_get_dev_addr(uint8_t out_addr[PROTO_ADDR_LEN]);

/**
 * @brief 获取网关地址
 * @param out_gw_addr [out]输出13字节网关地址缓存（13×FF 表示未绑定）
 */
void app_dev_param_get_gw_addr(uint8_t out_gw_addr[PROTO_ADDR_LEN]);

/**
 * @brief【配对绑定用，运行时可调】写入网关地址并落Flash
 * @param gw_addr_in 13字节网关地址（必须是有效地址：前缀 0x11 且非全 FF/全 00）
 * @return true成功 false参数非法或Flash操作失败
 * @note  配对模式收到网关应答后调用一次；内部会擦写整页Flash，
 *        只能在主循环上下文调用，禁止中断里调。
 */
bool app_dev_param_set_gw_addr(const uint8_t gw_addr_in[PROTO_ADDR_LEN]);

/**
 * @brief【解绑用，运行时可调】把网关地址写成 13×FF（未绑定）并落Flash
 * @return true成功 false Flash操作失败
 * @note  收到网关下发的 0x05 解绑指令、并且【两次 ACK 都发完之后】调用
 *        （顺序见 main.c 的 app_handle_rx_frame()）；调用后设备关唤醒/关射频进 STOP，
 *        本次运行不再配对，下次上电读到未绑定才进配对流程。
 *        同样只能在主循环上下文调用。
 */
bool app_dev_param_clear_gw_addr(void);

/* ==================== 设备字段（10 字节 ASCII，掉电不丢） ====================
 * 默认 "YEWEI00000"；网关用 0x06 SET_FIELD 写入（只校验长度 10），
 * 用 0x07 GET_FIELD 读取时设备回 0x08（载荷就是这 10 字节原文）。
 * 用途：交付前由网关写入甲方要求的设备编号/字段。 */

/**
 * @brief 获取设备字段（从 RAM 缓存，10 字节，不保证有结束符）
 * @param out_field [out]10 字节输出缓冲
 */
void app_dev_param_get_field(uint8_t out_field[APP_DEV_FIELD_LEN]);

/**
 * @brief 写入设备字段并落 Flash（0x06 SET_FIELD 用）
 * @param in_field 10 字节 ASCII（调用方已校验长度）
 * @return true 成功；false Flash 擦写失败
 * @note  【幂等要求】即使内容和当前完全相同，也会重新擦写 Flash 并返回 true。
 *        因为网关会在 5 秒内最多重发 3 条 0x06，每次都要求再回 0x80，
 *        所以这里【故意不做"值没变就跳过写"的去重】。
 */
bool app_dev_param_set_field(const uint8_t in_field[APP_DEV_FIELD_LEN]);

/**
 * @brief 获取持久化传感器配置(从RAM缓存读取)
 * @param out_cfg 输出传感器配置
 */
void app_dev_param_get_sensor_cfg(DevSensorConfig_t *out_cfg);

/**
 * @brief 设置并持久化传感器配置，写入Flash【运行时可改】
 * @param in_cfg 输入传感器配置
 * @return true成功 false flash操作失败
 */
bool app_dev_param_set_sensor_cfg(const DevSensorConfig_t *in_cfg);

/**
 * @brief 获取Flash保存的持久化unix时间戳（从 RAM 缓存，上电时已从时间戳页读出）
 * @return 0=无有效时间；非0为unix时间戳
 */
uint32_t app_dev_param_get_persist_unix_ts(void);

/**
 * @brief 设置并持久化保存unix时间戳到【独立的时间戳页】0x0800FA00
 * @param ts unix时间戳，0代表清除有效时间
 * @return true成功 false flash失败
 * @note  只擦写时间戳那一页，不再动参数页 —— 写中掉电也不会丢网关地址/设备字段
 */
bool app_dev_param_set_persist_unix_ts(uint32_t ts);

/* 产线只需烧固件，不需要写地址：设备地址由 UID 现算（无处可写），
 * 网关地址只允许配对(0x04)/解绑(0x05)两条运行时路径改写。 */

#ifdef __cplusplus
}
#endif
#endif
