#include "water_cu.h"
#include "bsp_adc.h"
#include "bsp_lowpower.h"    /* BSP_EXTI_WakeupPinConfig()：PB1 上升沿唤醒 */
#include "delay.h"
#include "n32g031_gpio.h"
#include "n32g031_rcc.h"
#include "misc.h"            /* NVIC_Init() */
#include "log.h"
#include <stddef.h>      /* NULL */

/*
 * ============================ 探针测量（电压法） ============================
 * 接线（本板实物）：PA7 ─[110k]─ 电极1 ─(空气/水)─ 电极2 ─[110k]─ PB1(ADC_CH_9)
 *                   PB1 ─[1M]─ GND
 *   - PA7：探针激励脚，推挽输出高；PB1：检测脚，只做 ADC 采样。
 *   - PB1 只允许"输入（带内部上下拉）"或"模拟"，永不做推挽输出，
 *     否则 PA7/PB1 电平相反时只有 220k 串阻限流。
 *   - 电极回路是 RC（双电层电容 + 探针线电容，τ 实测 0.1~3s），
 *     所以每轮先放电 150ms，再自适应充电（见 probe_charge()）。
 *   - 判水：raw ≥ WATER_DETECT_WET_RAW_THRESHOLD(800) 或 PB1 数字电平为高；
 *     PROBE_ROUNDS 轮投票，≥2 轮为湿即判湿。
 * =========================================================================
 */
#define PROBE_PORT          GPIOA
#define PROBE_DRIVE_PIN     GPIO_PIN_7      /* PA7 激励（配 PB1 做 ADC） */
#define PROBE_ADC_PIN       GPIO_PIN_1      /* PB1 ADC_CH_9，仍在 GPIOB */
#define PROBE_ADC_PORT      GPIOB

/* 可调参数（PROBE_SETTLE_US / PROBE_SAMPLES / PROBE_STRONG_DRIVE / PROBE_DEBUG）
 * 都集中在 water_cu.h，方便统一修改 */

/* 相位里把激励脚(PA7)配成的模式 */
#if (PROBE_STRONG_DRIVE != 0)
#define DRV_MODE_HIGH       3U      /* 推挽输出高 */
#define DRV_MODE_LOW        4U      /* 推挽输出低 */
#define DRV_NAME_HIGH       "pp-high"
#define DRV_NAME_LOW        "pp-low"
#else
#define DRV_MODE_HIGH       1U      /* 输入+内部上拉(弱高) */
#define DRV_MODE_LOW        2U      /* 输入+内部下拉(弱低) */
#define DRV_NAME_HIGH       "weak-high"
#define DRV_NAME_LOW        "weak-low"
#endif

/* PB1 模式：0=模拟(ADC)  1=输入+内部上拉  2=输入+内部下拉  3=输入+无上下拉(靠板上外部下拉) */
static void pb1_set_mode(uint8_t mode)
{
    GPIO_InitType gpio;

    GPIO_InitStruct(&gpio);
    gpio.Pin            = PROBE_ADC_PIN;
    gpio.GPIO_Speed     = GPIO_SPEED_LOW;
    gpio.GPIO_Current   = GPIO_DC_LOW;
    gpio.GPIO_Alternate = GPIO_NO_AF;

    if (mode == 0U)
    {
        gpio.GPIO_Mode = GPIO_MODE_ANALOG;
        gpio.GPIO_Pull = GPIO_NO_PULL;
    }
    else if (mode == 3U)
    {
        /* 数字输入 + 无上下拉：给"板上有外部下拉"的场合用。
         * ⚠️ 不能用模拟模式，否则施密特触发器关闭，EXTI 收不到边沿。 */
        gpio.GPIO_Mode = GPIO_MODE_INPUT;
        gpio.GPIO_Pull = GPIO_NO_PULL;
    }
    else
    {
        gpio.GPIO_Mode = GPIO_MODE_INPUT;
        gpio.GPIO_Pull = (mode == 1U) ? GPIO_PULL_UP : GPIO_PULL_DOWN;
    }
    GPIO_InitPeripheral(PROBE_ADC_PORT, &gpio);     /* PB1 在 GPIOB */
}

/* 激励脚(PA7)模式：0=高阻输入  1=输入+内部上拉(弱高)  2=输入+内部下拉(弱低)
 *                   3=推挽输出高  4=推挽输出低（需 PROBE_STRONG_DRIVE=1） */
static void probe_drive_set_mode(uint8_t mode)
{
    GPIO_InitType gpio;

    /* 先设好输出电平再切成输出，避免切换瞬间输出错误电平 */
    if ((mode == 3U) || (mode == 1U))
    {
        GPIO_SetBits(PROBE_PORT, PROBE_DRIVE_PIN);
    }
    else
    {
        GPIO_ResetBits(PROBE_PORT, PROBE_DRIVE_PIN);
    }

    GPIO_InitStruct(&gpio);
    gpio.Pin            = PROBE_DRIVE_PIN;
    gpio.GPIO_Speed     = GPIO_SPEED_HIGH;
    gpio.GPIO_Current   = GPIO_DC_HIGH;
    gpio.GPIO_Alternate = GPIO_NO_AF;

    switch (mode)
    {
        case 1U:    /* 弱高：输入 + 内部上拉 */
            gpio.GPIO_Mode = GPIO_MODE_INPUT;
            gpio.GPIO_Pull = GPIO_PULL_UP;
            break;
        case 2U:    /* 弱低：输入 + 内部下拉 */
            gpio.GPIO_Mode = GPIO_MODE_INPUT;
            gpio.GPIO_Pull = GPIO_PULL_DOWN;
            break;
        case 3U:    /* 强高：推挽输出（默认不用） */
            gpio.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
            gpio.GPIO_Pull = GPIO_NO_PULL;
            break;
        case 4U:    /* 强低：推挽输出（默认不用） */
            gpio.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
            gpio.GPIO_Pull = GPIO_NO_PULL;
            break;
        default:    /* 高阻 */
            gpio.GPIO_Mode = GPIO_MODE_INPUT;
            gpio.GPIO_Pull = GPIO_NO_PULL;
            break;
    }
    GPIO_InitPeripheral(PROBE_PORT, &gpio);
}

/* 最近一次采样时 PB1 的数字电平（趁采样配置还在时读）。
 * ADC 门限(0.65V) 低于 EXTI 数字门限(1.65V)，中间那段"ADC 说没水、引脚仍是高"
 * 的区间靠它兜住，否则会睡死。 */
static uint8_t s_last_lvl = 0U;

/* 最近一次采样的 ADC 原始值与当时 PB1 的负载模式（只读，供日志/排查）。
 * raw 是区分"硬件没接对"和"门限太高"的唯一证据，别再删。 */
static uint16_t s_last_raw = 0U;

/* 第一个采样点(加激励后 100ms 的值)与投票结果，排查用：
 *   first≈0、raw≈0、用时走满上限 -> 回路真断；
 *   first 小、raw 几千          -> 正常泡水（充得慢但能充满）；
 *   first 就已很大               -> 电极短接（τ≈0）。 */
static uint16_t s_last_raw_fast = 0U;
static uint8_t  s_last_votes    = 0U;
static uint16_t s_last_charge_ms = 0U;   /* 最后一轮实际充电用了多少毫秒（诊断 τ 用） */

/* 读一次 ADC 平均值：ADC 本体初始化/反初始化都在这里，PB1 的负载配置保持不动 */
static uint16_t probe_adc_avg(void)
{
    uint32_t sum = 0U;
    uint8_t  i;

    BSP_ADC_LevelProbe_AdcInit();
    for (i = 0U; i < PROBE_SAMPLES; i++)
    {
        sum += BSP_ADC_LevelProbe_ReadRaw();
    }
    /* 趁 PA7 仍在激励、PB1 仍带负载时记一次数字电平；
     * 撤掉配置后引脚回高阻，读到的就是悬空电平了。 */
    s_last_lvl = (GPIO_ReadInputDataBit(PROBE_ADC_PORT, PROBE_ADC_PIN) != 0U) ? 1U : 0U;
    BSP_ADC_LevelProbe_DeInit();

    return (uint16_t)(sum / PROBE_SAMPLES);
}

/**
 * @brief 把节点电荷放掉（每轮测量前）：PA7 推挽低 + PB1 下拉(≈40k)∥1M ≈ 38.5k，150ms 放到 2% 以下
 * @note  PB1 仍只是"输入 + 下拉"，永不是输出；反极性脉冲兼作电荷平衡，抵消部分极化。
 */
static void probe_discharge(uint16_t ms)
{
    probe_drive_set_mode(0U);
    pb1_set_mode(0U);
    delay_us(50U);

    pb1_set_mode(2U);           /* PB1 = 输入 + 内部下拉(≈40k)：泄放负载 */
    probe_drive_set_mode(4U);   /* PA7 = 推挽输出低：把节点电荷抽走 */
    delay_ms(ms);

    probe_drive_set_mode(0U);
    pb1_set_mode(0U);
}

/**
 * @brief 加激励后"充到稳态或超时"，返回最后一个采样点（判水就用它）；out_first/out_ms 可为 NULL
 * @note  τ 实测 0.1~3s：每 PROBE_CHARGE_STEP_MS 采一点，连续 PROBE_STABLE_STEPS 次变化 ≤ PROBE_STABLE_DELTA 即停。
 */
static uint16_t probe_charge(uint8_t pb1_mode, uint8_t drv_mode, uint16_t max_ms,
                             uint16_t *out_first, uint16_t *out_ms)
{
    uint16_t v          = 0U;
    uint16_t prev       = 0U;
    uint16_t first      = 0U;
    uint16_t t          = 0U;
    uint8_t  stable     = 0U;
    uint8_t  got_first  = 0U;

    /* 1. 两个脚先都回高阻，避免切换过程中对顶 */
    probe_drive_set_mode(0U);
    pb1_set_mode(0U);
    delay_us(50U);

    /* 2. 先挂负载(PB1 上/下拉)，再加激励 —— 顺序不能反 */
    pb1_set_mode(pb1_mode);
    probe_drive_set_mode(drv_mode);

    /* 3. 边充边采，直到稳态或超时；采样期间 PB1 必须保持"输入+上下拉"当负载，
     *    配成模拟等于拆掉负载，节点充上电就再也不泄放。 */
    while (t < max_ms)
    {
        delay_ms(PROBE_CHARGE_STEP_MS);
        t = (uint16_t)(t + PROBE_CHARGE_STEP_MS);
        v = probe_adc_avg();

        if (got_first == 0U)
        {
            first     = v;
            prev      = v;
            got_first = 1U;
            continue;
        }

        /* 变化量 ≤ PROBE_STABLE_DELTA 就累加"稳定计数"，否则归零重数 */
        if (((v > prev) ? (uint16_t)(v - prev) : (uint16_t)(prev - v)) <= PROBE_STABLE_DELTA)
        {
            stable++;
            if (stable >= PROBE_STABLE_STEPS)
            {
                break;              /* 已到稳态，提前结束（好水/短接通常 0.3s 内） */
            }
        }
        else
        {
            stable = 0U;
        }
        prev = v;
    }

    /* 4. 收尾：激励脚回高阻、PB1回高阻，避免长期往水里灌电流 */
    probe_drive_set_mode(0U);
    pb1_set_mode(0U);

    if (out_first != NULL)
    {
        *out_first = first;
    }
    if (out_ms != NULL)
    {
        *out_ms = t;
    }
    return v;
}

/* 最近一次采样的 ADC 原始值(0~4095)与 PB1 数字电平，供上层日志/排查读取 */
uint16_t WaterDetect_LastRaw(void)
{
    return s_last_raw;
}

uint16_t WaterDetect_LastRawFast(void)
{
    return s_last_raw_fast;
}

uint8_t WaterDetect_LastWetVotes(void)
{
    return s_last_votes;
}

uint16_t WaterDetect_LastChargeMs(void)
{
    return s_last_charge_ms;
}

uint8_t WaterDetect_LastLvl(void)
{
    return s_last_lvl;
}

/**
 * @brief 水位检测初始化：两个脚都置高阻，不驱动
 */
void WaterDetect_Init(void)
{
    /* 激励脚 PA7 在 GPIOA，检测脚 PB1 在 GPIOB，两个时钟都要开 */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB, ENABLE);

    probe_drive_set_mode(0U);   /* PA7(激励) 高阻 */
    pb1_set_mode(0U);           /* PB1 模拟(高阻) */

    /*log_info("[probe] init: DRV(PA7)=HiZ(%s drive), PB1=analog(ADC_CH9), settle=%uus samples=%u\r\n",
#if (PROBE_STRONG_DRIVE != 0)
             "strong/pp",
#else
             "weak/pull",
#endif
             (unsigned)PROBE_SETTLE_US, (unsigned)PROBE_SAMPLES);*/
}

/* ---- 【电压法】判水：PA7 推挽高当激励，PB1 带泄放负载，水把回路接通电压才升起来 ---- */

/**
 * @brief 【电压法】读一次探针电压(ADC原始值)，单轮、不投票（出厂配对轮询用，主流程用 Scan()）
 * @note  ⚠️ PB1 的负载须与睡前武装一致（PROBE_WAKE_EXT_PULLDOWN），内部 40k 与外部 1M 混用会失配。
 */
uint16_t WaterDetect_ReadVoltageRaw(void)
{
    uint16_t first = 0U;
    uint16_t used_ms = 0U;
    uint16_t slow;

    /* 先放电到 0V 起步：不放电测到的是节点残留电荷，不是水阻。 */
    probe_discharge(PROBE_DISCHARGE_MS);

    /* PA7 = 推挽输出高(激励)；PB1 = 按本板的负载配置 */
#if (PROBE_WAKE_EXT_PULLDOWN)
    slow = probe_charge(3U, 3U, PROBE_CHARGE_MAX_MS, &first, &used_ms);
#else
    slow = probe_charge(2U, 3U, PROBE_CHARGE_MAX_MS, &first, &used_ms);
#endif
    s_last_raw      = slow;
    s_last_raw_fast = first;
    s_last_charge_ms = used_ms;
    return slow;
}

/* 判水状态记录（供 WaterDetect_WakeArm 决定要不要武装等水唤醒）：
 *   s_scanned  : 做过判水没有（没做过绝不武装，免得白驱动电极）
 *   s_last_wet : 最近一次判水结果（已湿不武装，避免直流长期加在湿电极上） */
static uint8_t s_last_wet = 0U;
static uint8_t s_scanned  = 0U;
/* 是否无条件武装水唤醒（出厂等水流程经 WaterDetect_WakeAlwaysArm 设置）：
 *   0 = 只在"上次判无水"时武装（业务模式）
 *   1 = 无条件武装（出厂未绑定，有水就醒） */
static uint8_t s_arm_always = 0U;
/* EXTI 触发边沿：0 = 只上升沿（判水模式，"水来"才醒）
 *               1 = 双边沿（探针常开模式，水来/水退都触发） */
static uint8_t s_edge_both = 0U;

/* 一次判水 = PROBE_ROUNDS 轮（放电 -> 自适应充电采数），≥PROBE_WET_MIN_VOTES 轮为湿即判湿。 */
WaterDetectStatus_t WaterDetect_Scan(void)
{
    uint16_t raw;
    uint16_t first;
    uint16_t used_ms;
    uint16_t best_raw   = 0U;
    uint16_t best_first = 0U;
    uint16_t best_ms    = 0U;
    uint8_t  votes      = 0U;
    uint8_t  i;
    WaterDetectStatus_t st;

    /* 【多轮投票】单轮读数同时受"上次残留电荷""工频干扰""电极极化"影响，
     * 会出现同一块水里时而判湿时而判干；每轮之间先放电，轮与轮才是独立样本。
     * 代价：3 轮 ≈ 1.5~4s（短接/好水提前稳定，只有真开路才走满上限），每次唤醒只做一次。 */
    s_last_lvl = 0U;

    for (i = 0U; i < PROBE_ROUNDS; i++)
    {
        probe_discharge(PROBE_DISCHARGE_MS);

#if (PROBE_WAKE_EXT_PULLDOWN)
        raw = probe_charge(3U, 3U, PROBE_CHARGE_MAX_MS, &first, &used_ms);
#else
        raw = probe_charge(2U, 3U, PROBE_CHARGE_MAX_MS, &first, &used_ms);
#endif

        /* 单轮判据（任一成立即算有水）：
         *   ① 稳态读数 ≥ WATER_DETECT_WET_RAW_THRESHOLD(800 ≈ 0.65V) —— 主判据
         *   ② PB1 数字电平为高（施密特门限 ≈ 0.5VDD ≈ 1.65V）—— 兜底
         * ⚠️ 两个门限之间有 0.65~1.65V 落差，只看 ADC 会把"引脚仍为高"误判成无水，
         *    随后按无水挂【上升沿】中断，而引脚本就是高 —— 再也不会有上升沿，永久睡死。 */
        if ((raw >= WATER_DETECT_WET_RAW_THRESHOLD) || (s_last_lvl != 0U))
        {
            votes++;
        }

        /* 记录"最好"的一轮（最大值）：日志报的就是这轮的 first/raw/用时，
         * 判断"回路断没断"看 first 是否也小、用时是否走满上限。 */
        if (raw > best_raw)
        {
            best_raw   = raw;
            best_first = first;
            best_ms    = used_ms;
        }
    }

    s_last_raw       = best_raw;
    s_last_raw_fast  = best_first;
    s_last_charge_ms = best_ms;
    s_last_votes     = votes;

    /* 多数票为湿即判湿；另外"最后一轮引脚仍是高"也算湿（防睡死，见上）。 */
    if ((votes >= PROBE_WET_MIN_VOTES) || (s_last_lvl != 0U))
    {
        st = WATER_STATUS_HAS_WATER;
    }
    else
    {
        st = WATER_STATUS_NO_WATER;
    }

    /* 记下结果，睡前据此决定要不要武装（见 WaterDetect_WakeArm） */
    s_last_wet = (st == WATER_STATUS_HAS_WATER) ? 1U : 0U;
    s_scanned  = 1U;

    return st;
}

/* ==========================================================================
 *                      【有水自动唤醒】睡眠期间保持电压法接法
 *   PA7 = 推挽输出高，PB1 = 输入+下拉 + EXTI 上升沿：干态无回路 0 电流，
 *   触水 -> PB1 升高 -> 上升沿叫醒 STOP。数字门限约 0.5VDD(1.65V)，比判水的 0.65V 高得多，
 *   分压不够就叫不醒（门限表见 water_cu.h）。EXTI 线 1 属 EXTI0_1_IRQn，与 PA0 共向量不冲突。
 * ⚠️ 直流长期加在湿电极上会极化/腐蚀并耗电，故只在"上次判无水"时武装。
 * ========================================================================== */
static volatile uint8_t s_wake_flag = 0U;   /* EXTI 中断置位，主循环读后清零 */

/**
 * @brief 【中断里调】PB1 上升沿 -> 只置标志，别在中断里跑 ADC/打印
 */
void WaterDetect_WakeIsrHandler(void)
{
    s_wake_flag = 1U;
}

/**
 * @brief 取"是不是被水叫醒的"标志（读后清零）
 * @retval 1 = 本次唤醒来自 PB1 上升沿(探针碰到水)
 */
uint8_t WaterDetect_TakeWakeFlag(void)
{
    uint8_t f = s_wake_flag;

    s_wake_flag = 0U;
    return f;
}

/**
 * @brief 设置"水唤醒是否无条件武装"（出厂等水流程用）
 * @param enable 1 = 无条件武装（有水就醒）；0 = 带保护判断（上次判无水才武装）
 */
void WaterDetect_WakeAlwaysArm(uint8_t enable)
{
    s_arm_always = (enable != 0U) ? 1U : 0U;
}

/**
 * @brief 让探针一直工作：PA7 常驻 3.3V、PB1 常驻数字输入检测 + EXTI 双边沿
 * @note  采样函数会把两脚释放，采样后要再调一次本函数恢复
 */
void WaterDetect_ProbeAlwaysOn(void)
{
    s_edge_both = 1U;                   /* 双边沿：电压"变化"就触发 */

    /* 配 EXTI(双边沿) + PB1 映射到 EXTI 线 1（这个 BSP 函数会把 PB1 先配成"输入+上拉"） */
    BSP_EXTI_WakeupPinConfig(PROBE_ADC_PORT, PROBE_ADC_PIN, EXTI_Trigger_Rising_Falling);

    /* PB1 的负载：靠板上外部下拉时用"无上下拉"，否则用内部下拉 */
#if (PROBE_WAKE_EXT_PULLDOWN)
    pb1_set_mode(3U);                   /* 数字输入 + 无上下拉（靠板上下拉电阻） */
#else
    pb1_set_mode(2U);                   /* 数字输入 + 内部下拉(≈40k) */
#endif

    probe_drive_set_mode(3U);           /* PA7 = 推挽输出高：一直 3.3V 激励 */

    EXTI_ClrITPendBit(EXTI_LINE1);
    s_wake_flag = 0U;

    {
        NVIC_InitType nvic;

        nvic.NVIC_IRQChannel                   = EXTI0_1_IRQn;
        nvic.NVIC_IRQChannelPriority           = 3U;
        nvic.NVIC_IRQChannelCmd                = ENABLE;
        NVIC_Init(&nvic);
    }
}

/**
 * @brief 读当前 PB1 的数字电平（不改动引脚配置）
 */
uint8_t WaterDetect_ProbeLevel(void)
{
    return (GPIO_ReadInputDataBit(PROBE_ADC_PORT, PROBE_ADC_PIN) != 0U) ? 1U : 0U;
}

/**
 * @brief 睡前"会不会真的武装等水唤醒"（纯推算，不改任何东西）；返回 1=会武装，0=会撤防
 * @note  日志与动作都走这一个函数，口径才一致，不会出现"日志写 off、实际已挂 EXTI"。
 */
uint8_t WaterDetect_WouldArm(void)
{
#if (PROBE_WAKE_ON_WATER)
    if (s_arm_always != 0U)
    {
        return 1U;                  /* 无条件武装（出厂等水） */
    }
    /* 必须"判过水"且"上次是无水"才武装（已湿时不武装，避免直流长期加在电极上） */
    return ((s_scanned != 0U) && (s_last_wet == 0U)) ? 1U : 0U;
#else
    return 0U;
#endif
}

/**
 * @brief 睡前武装/撤防"等水唤醒"；arm=1 请求武装，返回 1=真的武装了，0=没武装
 * @note  须在 BSP_GPIO_SetAllAnalog() 之后、关 GPIO 时钟之前调用；是否真武装由 WouldArm() 定。
 */
uint8_t WaterDetect_WakeArm(uint8_t arm)
{
#if (PROBE_WAKE_ON_WATER)
    /* arm=0 撤防；其余交由 WaterDetect_WouldArm() 判定（和日志用同一个口径） */
    if ((arm == 0U) || (WaterDetect_WouldArm() == 0U))
    {
        /* 撤防：关 EXTI、激励回高阻、PB1 回高阻 —— 最省电，也不往电极灌电流 */
        EXTI->IMASK &= ~(uint32_t)EXTI_LINE1;
        EXTI->EMASK &= ~(uint32_t)EXTI_LINE1;
        EXTI_ClrITPendBit(EXTI_LINE1);
        probe_drive_set_mode(0U);
        pb1_set_mode(0U);
        s_wake_flag = 0U;
        return 0U;
    }

    /* 1) 先配 EXTI（这个 BSP 函数会把 PB1 配成"输入+内部上拉"，下面再覆盖成下拉/无拉）；
     *    边沿：业务模式只上升沿；"探针一直工作"模式下用双边沿（水来/水退都触发） */
    BSP_EXTI_WakeupPinConfig(PROBE_ADC_PORT, PROBE_ADC_PIN,
                             (s_edge_both != 0U) ? EXTI_Trigger_Rising_Falling
                                                 : EXTI_Trigger_Rising);

    /* 2) PB1 的负载：有外部 1MΩ 下拉时用"输入+无上下拉"，否则用内部下拉 */
#if (PROBE_WAKE_EXT_PULLDOWN)
    pb1_set_mode(3U);                   /* 数字输入 + 无上下拉（靠板上 1MΩ） */
#else
    pb1_set_mode(2U);                   /* 数字输入 + 内部下拉(≈40k) */
#endif

    /* 3) PA7 = 推挽输出高（激励）。用 push-pull 而不是"弱高"：
     *    弱高的激励源自己就是 40k，会和水阻、下拉三者分压，湿态电压上不去。 */
    probe_drive_set_mode(3U);

    EXTI_ClrITPendBit(EXTI_LINE1);
    s_wake_flag = 0U;

    /* 4) NVIC 使能 EXTI0_1（PB1 = 线 1）。STOP 下 EXTI 能把 MCU 叫醒。 */
    {
        NVIC_InitType nvic;

        nvic.NVIC_IRQChannel                   = EXTI0_1_IRQn;
        nvic.NVIC_IRQChannelPriority           = 3U;
        nvic.NVIC_IRQChannelCmd                = ENABLE;
        NVIC_Init(&nvic);
    }
    return 1U;
#else
    (void)arm;
    return 0U;
#endif
}
