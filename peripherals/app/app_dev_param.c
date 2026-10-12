/**
 * @file app_dev_param.c
 * @brief 设备持久化参数实现
 */
#include "app_dev_param.h"
#include "storage.h"
#include <string.h>

/* RAM内存缓存，程序运行全部读这个，不反复读Flash */
static DevParamFlash_t s_dev_param;

/* 出厂默认设备地址、网关地址，flash无效时使用，产线烧录会覆盖。
 * 网关地址出厂 = FF FF FF = 未绑定：设备上电发现网关地址无效就进配对模式
 * （app_pair.c，每 5s 发一帧 0x04 配对请求），配对成功后才会被改写。 */
static const uint8_t g_default_dev_addr[PROTO_ADDR_LEN] = {0x21U, 0x00U, 0x01U}; // 设备地址
static const uint8_t g_default_gw_addr[PROTO_ADDR_LEN] = {0xFFU, 0xFFU, 0xFFU};  // 网关地址(未绑定)

/* 传感器出厂默认参数，Flash损坏/第一次上电加载 */
static const DevSensorConfig_t g_default_sensor_cfg =
    {
        .detect_interval_s = 30U,      /* 休眠周期 30s：收到 0x01 之前用（台架方便）；
                                        * 收到 0x01 后 detect 改成 report/2(夹 5min~2h) */
        .report_interval_s = 30U,      /* 上报间隔 30s（出厂默认；云端下发后覆盖，检测间隔由它推导） */
        .same_level_report_en = 0U,    /* 相同液位是否上报（当前业务：周期到就发，一样也发） */
        .level_alarm_thresh_mm = 500U, /* 液位告警阈值 500mm：超了立即上报 */
};

void app_dev_param_init(void)
{
    DevParamFlash_t flash_buf;
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
    /* 【参数格式迁移】Flash里是旧版本的参数：产线固化的设备地址必须保留，
     * 只把传感器配置刷成新的出厂默认值，再写回并把版本号更新掉（只做一次）。
     * V2->V3 额外动作：把网关地址清成 FF FF FF(未绑定)，让升级完的设备上电进配对模式。
     * ⚠️ 量产升级固件时要把 DEV_PARAM_MIGRATE_CLEAR_GW 改成 0，
     *    否则现场所有已绑定设备的绑定关系都会被这次升级清掉。 */
    if (flash_buf.reserved[0] != (uint8_t)DEV_PARAM_CFG_VER)
    {
        memcpy(&s_dev_param, &flash_buf, sizeof(DevParamFlash_t));
        memcpy(&s_dev_param.sensor_cfg, &g_default_sensor_cfg, sizeof(DevSensorConfig_t));
#if (DEV_PARAM_MIGRATE_CLEAR_GW)
        memcpy(s_dev_param.gw_addr, g_default_gw_addr, PROTO_ADDR_LEN);
#endif
        memset(s_dev_param.reserved, 0U, sizeof(s_dev_param.reserved));
        s_dev_param.reserved[0] = (uint8_t)DEV_PARAM_CFG_VER;
        (void)storage_flash_erase_page(DEV_PARAM_FLASH_PAGE_ADDR);
        (void)storage_flash_write(DEV_PARAM_FLASH_PAGE_ADDR, (uint8_t *)&s_dev_param, sizeof(DevParamFlash_t));
        return;
    }
    /* flash参数有效，拷贝到ram缓存 */
    memcpy(&s_dev_param, &flash_buf, sizeof(DevParamFlash_t));
    return;
load_default:
    /* 使用出厂默认 */
    memset(&s_dev_param, 0U, sizeof(s_dev_param));
    s_dev_param.magic = DEV_PARAM_MAGIC;
    memcpy(s_dev_param.dev_addr, g_default_dev_addr, PROTO_ADDR_LEN);
    memcpy(s_dev_param.gw_addr, g_default_gw_addr, PROTO_ADDR_LEN);
    memcpy(&s_dev_param.sensor_cfg, &g_default_sensor_cfg, sizeof(DevSensorConfig_t));
    s_dev_param.reserved[0] = (uint8_t)DEV_PARAM_CFG_VER;
    /* 默认参数落一次Flash：保证设备地址(和"未绑定"标记 FF FF FF)断电不丢（只会写这一次） */
    (void)storage_flash_erase_page(DEV_PARAM_FLASH_PAGE_ADDR);
    (void)storage_flash_write(DEV_PARAM_FLASH_PAGE_ADDR, (uint8_t *)&s_dev_param, sizeof(DevParamFlash_t));
}

void app_dev_param_get_dev_addr(uint8_t out_addr[PROTO_ADDR_LEN])
{
    memcpy(out_addr, s_dev_param.dev_addr, PROTO_ADDR_LEN);
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
    /* FF FF FF / 00 00 00 不是有效网关地址：配对时网关回了这种地址就拒绝写入，
     * 否则设备会把自己写成"未绑定"然后一直配对不上。 */
    if (!proto_addr_is_valid(gw_addr_in))
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

uint32_t app_dev_param_get_persist_unix_ts(void)
{
    return s_dev_param.persist_unix_ts;
}

bool app_dev_param_set_persist_unix_ts(uint32_t ts)
{
    DevParamFlash_t write_buf;
    memcpy(&write_buf, &s_dev_param, sizeof(DevParamFlash_t));
    write_buf.magic = DEV_PARAM_MAGIC;
    write_buf.persist_unix_ts = ts;
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

/*==================== 产线烧录专用实现 ====================*/
bool app_dev_param_prog_line_write_addr(const uint8_t dev_addr_in[PROTO_ADDR_LEN], const uint8_t gw_addr_in[PROTO_ADDR_LEN])
{
    DevParamFlash_t write_buf;

    /* Flash里还没有有效参数时，先装载出厂默认，避免把传感器配置写成全0 */
    if (s_dev_param.magic != DEV_PARAM_MAGIC)
    {
        app_dev_param_init();
    }

    memcpy(&write_buf, &s_dev_param, sizeof(DevParamFlash_t));
    write_buf.magic = DEV_PARAM_MAGIC;
    /* 烧录固化设备、网关地址 */
    memcpy(write_buf.dev_addr, dev_addr_in, PROTO_ADDR_LEN);
    memcpy(write_buf.gw_addr, gw_addr_in, PROTO_ADDR_LEN);

    if (!storage_flash_erase_page(DEV_PARAM_FLASH_PAGE_ADDR))
    {
        return false;
    }
    if (!storage_flash_write(DEV_PARAM_FLASH_PAGE_ADDR, (uint8_t *)&write_buf, sizeof(DevParamFlash_t)))
    {
        return false;
    }
    /* 更新RAM缓存 */
    memcpy(&s_dev_param, &write_buf, sizeof(DevParamFlash_t));
    return true;
}
