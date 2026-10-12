/**
 * @file app_dev_param.h
 * @brief 设备持久化参数：设备地址(烧录固化)、网关地址(配对写入/解绑清除)；传感器配置等，保存到Flash，断电不丢失
 *
 * 【配对/绑定 与 Flash 的关系（2026-09 新增）】
 *   出厂：Flash 里【只有本机设备地址】，网关地址 = FF FF FF（未绑定）。
 *   上电：app_runtime_state 里 gw_addr 无效(FF FF FF / 00 00 00) -> 进配对模式
 *         （见 app_pair.c）：每 5s 发一帧 0x04 配对请求（目的地址 FF FF FF），
 *         收到网关应答(0x80)后把应答帧里的网关地址写进 Flash -> 下次上电直接进主程序。
 *   解绑：网关下发 0x05 -> 设备先回 2 次 ACK(间隔 5s) -> app_dev_param_clear_gw_addr()
 *         把网关地址写回 FF FF FF -> 关唤醒/关射频 -> STOP 等下次上电
 *         （【不是】立刻回配对模式：本次运行不再发 0x04，下次上电读到未绑定才进配对）。
 *   设备地址 dev_addr 仍然只允许产线烧录，运行期不改。
 */
#ifndef __APP_DEV_PARAM_H__
#define __APP_DEV_PARAM_H__
#include <stdint.h>
#include <stdbool.h>
#include "protocol_util.h"
#ifdef __cplusplus
extern "C" {
#endif

/* ========= Flash参数存储配置，务必选择芯片最后一页，避开代码段 ========= */
#define DEV_PARAM_FLASH_PAGE_ADDR    0x0800F800U
/* 魔术字，用来判断Flash参数是否有效 */
#define DEV_PARAM_MAGIC              0xA55AA55AU
/* 参数格式版本，写在 DevParamFlash_t.reserved[0]。
 * 固件改了"出厂默认参数"后必须把这个版本号 +1：
 * 上电发现 Flash 里的版本和固件不一致时，保留产线固化的设备地址，
 * 只把传感器配置刷成新的出厂默认值再写回（否则老板子永远用着旧默认值，
 * 比如"休眠周期默认 30s"这种改动在老板子上不生效）。
 *   V1 = 初始版本（reserved[0] = 0）
 *   V2 = 休眠周期默认 30s（detect_interval_s: 10 -> 30）
 *   V3 = 新增配对绑定功能：迁移时把网关地址清成 FF FF FF（未绑定），
 *        升级完上电直接进配对模式（见下面 DEV_PARAM_MIGRATE_CLEAR_GW）
 *
 *   ⚠️ 2026-10-05：协议 0x01 载荷里"检测间隔/上报间隔"的单位由【分钟】改成【秒】
 *      （与网关 app/protocol/lora_business.c 的"用户口径改版"对齐，两边都不换算）。
 *      这【不涉及 Flash 格式】：Flash 里存的本来就是秒，所以【不升版本号、老板子不用迁移】；
 *      受影响的只有"从网关收到载荷那一刻"的解析，见 app_cmd.h 的 APP_INTERVAL_UNIT 开关。
 */
#define DEV_PARAM_CFG_VER            3U

/* 【迁移开关】参数版本不一致时，要不要把网关地址清成 FF FF FF(未绑定)：
 *   1 = 清（【当前】联调：升级固件后上电立刻进配对模式，方便验证 5s 配对帧）
 *   0 = 保留 Flash 里已配对的网关地址（【量产升级固件时改成 0】，
 *       否则一次固件升级会把现场所有已绑定设备的绑定关系清掉，全都要重新配对）
 * 注意：不管填 0 还是 1，"设备地址"永远保留，只动网关地址。 */
#ifndef DEV_PARAM_MIGRATE_CLEAR_GW
#define DEV_PARAM_MIGRATE_CLEAR_GW   1
#endif

/**
 * @brief 传感器持久化配置参数【网关下发后只在RAM里用，不写Flash】
 * @note  STOP低功耗不会丢RAM，所以网关参数不需要落Flash；
 *        频繁擦写Flash会缩短寿命，除非确认掉电也必须保住，才调用
 *        app_dev_param_set_sensor_cfg() 落盘
 */
typedef struct
{
    uint16_t     detect_interval_s;      /* 检测参数时间间隔，单位【秒】
                                          * 收到 0x01 后由 report_interval_s/2 推导(夹到 5min~2h)，
                                          * 不再采用载荷里的检测间隔，见 app_cmd.h【检测间隔的推导规则】 */
    uint16_t    report_interval_s;      /* 上报时间间隔，单位【秒】(同上) */
    uint8_t     same_level_report_en;   /* 相同液位是否上报 0关闭 1开启 */
    uint16_t    level_alarm_thresh_mm;  /* 液位告警阈值 mm */
}DevSensorConfig_t;

/**
 * @brief 存储在Flash里的参数结构体，4字节对齐！
 * NOTE：dev_addr 为产线烧录固化，运行态禁止修改；
 *       gw_addr 出厂为 FF FF FF，由配对(0x04)/解绑(0x05)在运行期改写。
 */
typedef struct
{
    uint32_t magic;                     /* 魔术字，校验是否有效 4字节 */
    uint8_t dev_addr[PROTO_ADDR_LEN];   /*【烧录固化】3字节设备地址 */
    uint8_t gw_addr[PROTO_ADDR_LEN];    /* 3字节网关地址：FF FF FF = 未绑定(需要配对) */
    uint8_t rsv1[2];                    /* 补齐4字节对齐 */
    /* 【新增】持久化RTC校时时间，网关校时之后保存，上电app_time读取恢复硬件RTC */
    uint32_t persist_unix_ts;           /* 保存网关校时的unix时间戳；0代表无有效校时 */
    /* 传感器业务持久化参数【运行可改】 */
    DevSensorConfig_t sensor_cfg;
    uint8_t reserved[4];                /* reserved[0]=DEV_PARAM_CFG_VER，其余预留 */
}DevParamFlash_t;

/* 编译期断言：Flash写入要求4字节对齐，打包编译时提前报错 */
typedef char dev_param_align_check[(sizeof(DevParamFlash_t) % 4U == 0U) ? 1 : -1];

/**
 * @brief 初始化设备参数，上电调用：读取flash，失败加载出厂默认
 */
void app_dev_param_init(void);

/**
 * @brief 获取本机设备地址
 * @param out_addr [out]输出3字节地址缓存
 */
void app_dev_param_get_dev_addr(uint8_t out_addr[PROTO_ADDR_LEN]);

/**
 * @brief 获取网关地址
 * @param out_gw_addr [out]输出3字节网关地址缓存（FF FF FF 表示未绑定）
 */
void app_dev_param_get_gw_addr(uint8_t out_gw_addr[PROTO_ADDR_LEN]);

/**
 * @brief【配对绑定用，运行时可调】写入网关地址并落Flash
 * @param gw_addr_in 3字节网关地址（必须是有效地址，FF FF FF 会被拒绝）
 * @return true成功 false参数非法或Flash操作失败
 * @note  配对模式收到网关应答后调用一次；内部会擦写整页Flash，
 *        只能在主循环上下文调用，禁止中断里调。
 */
bool app_dev_param_set_gw_addr(const uint8_t gw_addr_in[PROTO_ADDR_LEN]);

/**
 * @brief【解绑用，运行时可调】把网关地址写成 FF FF FF 并落Flash
 * @return true成功 false Flash操作失败
 * @note  收到网关下发的 0x05 解绑指令、并且【两次 ACK 都发完之后】调用
 *        （顺序见 main.c 的 app_handle_rx_frame()）；调用后设备关唤醒/关射频进 STOP，
 *        本次运行不再配对，下次上电读到未绑定才进配对流程。
 *        同样只能在主循环上下文调用。
 */
bool app_dev_param_clear_gw_addr(void);

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
 * @brief 获取Flash保存的持久化unix时间戳
 * @return 0=无有效时间；非0为unix时间戳
 */
uint32_t app_dev_param_get_persist_unix_ts(void);

/**
 * @brief 设置并持久化保存unix时间戳到Flash【运行时可改】
 * @param ts unix时间戳，0代表清除有效时间
 * @return true成功 false flash失败
 */
bool app_dev_param_set_persist_unix_ts(uint32_t ts);

/*==================== 产线烧录专用接口 ====================*/
/**
 * @brief【仅产线烧录调用！应用业务禁止调用】烧录写入设备地址+网关地址，擦写整页Flash，只在烧录脚本/烧录工程调用一次
 * @param dev_addr_in 3字节设备地址
 * @param gw_addr_in  3字节网关地址（出厂烧录请填 FF FF FF = 未绑定，让设备自己去配对）
 * @return true成功
 * @note 此函数只在烧录阶段执行一次；设备地址运行态永不修改，
 *       网关地址运行态只能由配对(0x04)/解绑(0x05)两条路径改写
 *       （app_dev_param_set_gw_addr / app_dev_param_clear_gw_addr）
 */
bool app_dev_param_prog_line_write_addr(const uint8_t dev_addr_in[PROTO_ADDR_LEN], const uint8_t gw_addr_in[PROTO_ADDR_LEN]);

#ifdef __cplusplus
}
#endif
#endif
