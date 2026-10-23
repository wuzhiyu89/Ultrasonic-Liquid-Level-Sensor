/**
 * @file app_dev_param.c
 * @brief 设备持久化参数实现
 */
#include "app_dev_param.h"
#include "storage.h"
#include "n32g031.h"            /* UID_BASE / UID_LENGTH：96 位芯片唯一号 */
#include "log.h"
#include <string.h>

/* RAM 缓存：程序运行全读这个，不反复读 Flash */
static DevParamFlash_t s_dev_param;
/* 时间戳页的 RAM 缓存（参数页结构里不再保留这个字段） */
static uint32_t s_persist_unix_ts = 0U;

/* 出厂默认网关地址 = 13×FF = 未绑定：设备上电发现网关地址无效就进配对模式
 * （app_pair.c，每 5s 发一帧 0x04 配对请求），配对成功后才会被改写。 */
static const uint8_t g_default_gw_addr[PROTO_ADDR_LEN] =
    {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU,
     0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};

/* ============ 设备地址：0x21 + 96 位 UID（每次上电现算，不写 Flash） ============
 * N32G031 的 UID 是出厂固化的 96 位唯一号：UID_BASE = 0x1FFFF4FC、12 字节
 * （见 firmware/CMSIS/device/n32g031.h；厂商 GetUID() 也是逐字节这么读，
 *  但那个文件没加进编译组，这里按基地址直接读）。
 * ⚠️ 读出来全 0/全 FF（读失败）时按原样使用，只打醒目日志：
 *    真芯片几乎不会这样，一旦出现就可能两颗设备撞地址，必须现场能看出来。 */
static uint8_t s_dev_addr[PROTO_ADDR_LEN];

static void app_dev_param_build_dev_addr(void)
{
    uint8_t uid[PROTO_UID_LEN];
    uint8_t i;
    uint8_t all_ff = 1U;
    uint8_t all_00 = 1U;

    for (i = 0U; i < PROTO_UID_LEN; i++)
    {
        uid[i] = *(volatile uint8_t *)(UID_BASE + (uint32_t)i);     /* 物理顺序直读 */
        if (uid[i] != 0xFFU)
        {
            all_ff = 0U;
        }
        if (uid[i] != 0x00U)
        {
            all_00 = 0U;
        }
    }

    s_dev_addr[0] = PROTO_ADDR_PREFIX_DEVICE;                       /* 0x21 */
    memcpy(&s_dev_addr[PROTO_ADDR_PREFIX_LEN], uid, PROTO_UID_LEN);

    if (all_ff != 0U)
    {
        log_info("[dev] !! UID reads all 0xFF (read failed?): dev_addr may collide\r\n");
    }
    else if (all_00 != 0U)
    {
        log_info("[dev] !! UID reads all 0x00 (read failed?): dev_addr may collide\r\n");
    }
}

/* 传感器出厂默认（Flash 损坏/第一次上电，也是"还没收到云端 0x01"时的节奏）：
 * 上报 30min -> 检测 10min（= 1800/3，一个周期检测 3 次） */
static const DevSensorConfig_t g_default_sensor_cfg =
    {
        .report_interval_s = 1800U,    /* 上报 30min；检测间隔不存，现算 600s */
        .level_alarm_thresh_mm = 500U, /* 液位告警阈值 500mm：超了立即上报 */
        .calib_offset_mm = 0,          /* 没标定过就不动读数 */
        .install_height_cm = 0U,       /* 安装高度未配置 */
        .measure_mode = DEV_MEASURE_MODE_LEVEL,   /* 出厂按液位模式（读数直接当液位） */
        .same_level_report_en = 0U,    /* 只存不用 */
        .low_battery_pct = 0U,         /* 0 = 禁用（判定在网关侧） */
        .rsv0 = 0U,
};

/* 设备字段出厂默认："YEWEI00000"（正好 10 字节 ASCII，无结束符） */
static const uint8_t g_default_field[APP_DEV_FIELD_LEN] =
    { 'Y', 'E', 'W', 'E', 'I', '0', '0', '0', '0', '0' };

/* 字段合法性：10 个字节必须都是可打印 ASCII(0x20~0x7E)，
 * 擦除态 0xFF 判为非法 -> 用默认值 */
static bool app_dev_param_field_ok(const uint8_t *f)
{
    uint8_t i;

    for (i = 0U; i < APP_DEV_FIELD_LEN; i++)
    {
        if ((f[i] < 0x20U) || (f[i] > 0x7EU))
        {
            return false;
        }
    }
    return true;
}

/* 读参数页（DEV_PARAM_FLASH_PAGE_ADDR）：网关地址 / 设备字段 / 传感器配置
 * 时间戳在另一页，见 app_dev_param_init() */
static void app_dev_param_load_params(void)
{
    DevParamFlash_t flash_buf;

    /* 设备地址与 Flash 无关：每次上电从 UID 现算（0x21 + 96 位 UID） */
    app_dev_param_build_dev_addr();

    /* 读取flash页数据 */
    if (!storage_flash_read(DEV_PARAM_FLASH_PAGE_ADDR, (uint8_t *)&flash_buf, sizeof(DevParamFlash_t)))
    {
        goto load_default;
    }
    /* 判断魔术字是否合法 */
    if (flash_buf.magic != DEV_PARAM_MAGIC)
    {
        goto load_default;
    }
    /* 【版本不一致】整页重写成出厂默认：传感器配置/设备字段回默认，
     * 网关地址【保留】—— 升级固件不该把现场已绑定设备的绑定关系清掉。 */
    if (flash_buf.reserved[0] != (uint8_t)DEV_PARAM_CFG_VER)
    {
        memcpy(&s_dev_param, &flash_buf, sizeof(DevParamFlash_t));
        memcpy(&s_dev_param.sensor_cfg, &g_default_sensor_cfg, sizeof(DevSensorConfig_t));
        memcpy(s_dev_param.field, g_default_field, APP_DEV_FIELD_LEN);
        memset(s_dev_param.reserved, 0U, sizeof(s_dev_param.reserved));
        s_dev_param.reserved[0] = (uint8_t)DEV_PARAM_CFG_VER;
        (void)storage_flash_erase_page(DEV_PARAM_FLASH_PAGE_ADDR);
        (void)storage_flash_write(DEV_PARAM_FLASH_PAGE_ADDR, (uint8_t *)&s_dev_param, sizeof(DevParamFlash_t));
        return;
    }
    /* flash参数有效，拷贝到ram缓存 */
    memcpy(&s_dev_param, &flash_buf, sizeof(DevParamFlash_t));
    /* 设备字段校验不过（擦除态/脏数据）就回默认值，
     * 下一次收到网关 0x06 SET_FIELD 时会写进真实字段 */
    if (!app_dev_param_field_ok(s_dev_param.field))
    {
        memcpy(s_dev_param.field, g_default_field, APP_DEV_FIELD_LEN);
        log_info("[dev] field invalid in flash -> use default\r\n");
    }
    return;
load_default:
    /* 使用出厂默认 */
    memset(&s_dev_param, 0U, sizeof(s_dev_param));
    s_dev_param.magic = DEV_PARAM_MAGIC;
    memcpy(s_dev_param.gw_addr, g_default_gw_addr, PROTO_ADDR_LEN);
    memcpy(s_dev_param.field, g_default_field, APP_DEV_FIELD_LEN);
    memcpy(&s_dev_param.sensor_cfg, &g_default_sensor_cfg, sizeof(DevSensorConfig_t));
    s_dev_param.reserved[0] = (uint8_t)DEV_PARAM_CFG_VER;
    /* 默认参数落一次 Flash：保证"未绑定"标记(13×FF)断电不丢 */
    (void)storage_flash_erase_page(DEV_PARAM_FLASH_PAGE_ADDR);
    (void)storage_flash_write(DEV_PARAM_FLASH_PAGE_ADDR, (uint8_t *)&s_dev_param, sizeof(DevParamFlash_t));
}

void app_dev_param_get_dev_addr(uint8_t out_addr[PROTO_ADDR_LEN])
{
    memcpy(out_addr, s_dev_addr, PROTO_ADDR_LEN);       /* UID 现算的 0x21+UID */
}

void app_dev_param_get_gw_addr(uint8_t out_gw_addr[PROTO_ADDR_LEN])
{
    memcpy(out_gw_addr, s_dev_param.gw_addr, PROTO_ADDR_LEN);
}

/* 把 RAM 缓存整体写回 Flash：擦一页 -> 写 -> （成功即已在缓存里）
 * @retval true 成功；false 擦除或写入失败
 * @note  只能在主循环上下文调用（内部会擦写 Flash，中断里调会拖死时序） */
static bool app_dev_param_flush(void)
{
    s_dev_param.magic = DEV_PARAM_MAGIC;        /* 防呆：保证魔术字有效 */
    if (!storage_flash_erase_page(DEV_PARAM_FLASH_PAGE_ADDR))
    {
        return false;
    }
    if (!storage_flash_write(DEV_PARAM_FLASH_PAGE_ADDR, (uint8_t *)&s_dev_param, sizeof(DevParamFlash_t)))
    {
        return false;
    }
    return true;
}

bool app_dev_param_set_gw_addr(const uint8_t gw_addr_in[PROTO_ADDR_LEN])
{
    if (gw_addr_in == NULL)
    {
        return false;
    }
    /* 13×FF（未绑定）/ 全 00 / 前缀不是 0x11 都不是有效网关地址：
     * 配对时网关回了这种地址就拒绝写入，否则设备会把自己写成"未绑定"然后一直配对不上。 */
    if (!proto_addr_is_valid_gw(gw_addr_in))
    {
        return false;
    }
    /* 去重：同一个网关重复配对不重复擦写 Flash（延长 Flash 寿命） */
    if (proto_addr_equal(s_dev_param.gw_addr, gw_addr_in))
    {
        return true;
    }
    memcpy(s_dev_param.gw_addr, gw_addr_in, PROTO_ADDR_LEN);
    return app_dev_param_flush();
}

bool app_dev_param_clear_gw_addr(void)
{
    uint8_t unbound[PROTO_ADDR_LEN];

    proto_addr_set_unbound(unbound);
    if (proto_addr_equal(s_dev_param.gw_addr, unbound))
    {
        return true;                            /* 本来就是未绑定，不用再擦一次 */
    }
    memcpy(s_dev_param.gw_addr, unbound, PROTO_ADDR_LEN);
    return app_dev_param_flush();
}

/* ==================== 设备字段（10 字节 ASCII，掉电不丢） ==================== */

void app_dev_param_get_field(uint8_t out_field[APP_DEV_FIELD_LEN])
{
    if (out_field != NULL)
    {
        memcpy(out_field, s_dev_param.field, APP_DEV_FIELD_LEN);
    }
}

bool app_dev_param_set_field(const uint8_t in_field[APP_DEV_FIELD_LEN])
{
    if (in_field == NULL)
    {
        return false;
    }
    if (memcmp(s_dev_param.field, in_field, APP_DEV_FIELD_LEN) == 0)
    {
        /* 【幂等】值没变也照样重写 Flash：网关 5s 内最多重发 3 条 0x06，每次都要求回 0x80，
         * 按需求"仍要重新写"，所以不做去重 */
        log_info("[dev] field unchanged, rewrite flash anyway (idempotent)\r\n");
    }
    memcpy(s_dev_param.field, in_field, APP_DEV_FIELD_LEN);
    return app_dev_param_flush();
}

void app_dev_param_get_sensor_cfg(DevSensorConfig_t *out_cfg)
{
    if (out_cfg != NULL)
    {
        memcpy(out_cfg, &s_dev_param.sensor_cfg, sizeof(DevSensorConfig_t));
    }
}

bool app_dev_param_set_sensor_cfg(const DevSensorConfig_t *in_cfg)
{
    if (in_cfg == NULL)
    {
        return false;
    }
    /* 去重：新配置与RAM中当前配置相同则跳过Flash写入 */
    if (memcmp(&s_dev_param.sensor_cfg, in_cfg, sizeof(DevSensorConfig_t)) == 0)
    {
        return true;
    }
    DevParamFlash_t write_buf;
    memcpy(&write_buf, &s_dev_param, sizeof(DevParamFlash_t));
    write_buf.magic = DEV_PARAM_MAGIC;
    memcpy(&write_buf.sensor_cfg, in_cfg, sizeof(DevSensorConfig_t));
    if (!storage_flash_erase_page(DEV_PARAM_FLASH_PAGE_ADDR))
    {
        return false;
    }
    if (!storage_flash_write(DEV_PARAM_FLASH_PAGE_ADDR, (uint8_t *)&write_buf, sizeof(DevParamFlash_t)))
    {
        return false;
    }
    memcpy(&s_dev_param, &write_buf, sizeof(DevParamFlash_t));
    return true;
}

/* ==================== 时间戳：单独一页（0x0800FA00） ====================
 * 这一页每小时改一次，参数页几年才改一次 —— 挤在一页会让每次写时间戳都擦掉绑定关系。
 * 记录 12 字节（3 个字，满足 4 字节对齐）：magic 认页、chk 防"擦除态/半写"被当成有效时间。 */
typedef struct
{
    uint32_t magic;         /* DEV_PARAM_TS_MAGIC */
    uint32_t unix_ts;       /* UTC 秒 */
    uint32_t chk;           /* magic ^ unix_ts ^ DEV_PARAM_TS_CHK_SALT */
} DevTimeFlash_t;

#define DEV_PARAM_TS_CHK_SALT   0xA5A5A5A5U
typedef char dev_time_size_check[(sizeof(DevTimeFlash_t) % 4U == 0U) ? 1 : -1];

static uint32_t app_dev_param_ts_chk(uint32_t magic, uint32_t ts)
{
    return (uint32_t)(magic ^ ts ^ DEV_PARAM_TS_CHK_SALT);
}

/* 上电把时间戳页读进 RAM 缓存（读失败/校验不过 = 0 = 没有可信时间） */
static void app_dev_param_load_ts(void)
{
    DevTimeFlash_t rec;

    s_persist_unix_ts = 0U;
    if (!storage_flash_read(DEV_PARAM_TS_PAGE_ADDR, (uint8_t *)&rec, (uint16_t)sizeof(rec)))
    {
        return;
    }
    if (rec.magic != DEV_PARAM_TS_MAGIC)
    {
        return;                                 /* 从没写过（擦除态）/ 被别的数据占了 */
    }
    if (rec.chk != app_dev_param_ts_chk(rec.magic, rec.unix_ts))
    {
        log_info("[dev] ts page checksum bad -> ignore stored time\r\n");
        return;
    }
    s_persist_unix_ts = rec.unix_ts;
}

uint32_t app_dev_param_get_persist_unix_ts(void)
{
    return s_persist_unix_ts;                   /* RAM 缓存，init 时从时间戳页读入 */
}

bool app_dev_param_set_persist_unix_ts(uint32_t ts)
{
    DevTimeFlash_t rec;

    rec.magic   = DEV_PARAM_TS_MAGIC;
    rec.unix_ts = ts;
    rec.chk     = app_dev_param_ts_chk(rec.magic, ts);

    if (!storage_flash_erase_page(DEV_PARAM_TS_PAGE_ADDR))
    {
        return false;
    }
    if (!storage_flash_write(DEV_PARAM_TS_PAGE_ADDR, (uint8_t *)&rec, (uint16_t)sizeof(rec)))
    {
        return false;
    }
    s_persist_unix_ts = ts;
    return true;
}

/* 上电初始化：先读参数页，再读时间戳页（两页各管各的）
 * 必须在 app_time_init() 之前完成 —— 冷启动要靠时间戳恢复 RTC。 */
void app_dev_param_init(void)
{
    app_dev_param_load_params();
    app_dev_param_load_ts();
}
