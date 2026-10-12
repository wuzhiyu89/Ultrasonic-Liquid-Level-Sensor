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
 * ============================ 探针测量（方案C） ============================
 * 【2026-09-20 更正：激励脚是 PA7，不是 PB7！PB7 是外设电源使能脚】
 * 接线（两端各串10k）：
 *     PA7 ──[10k]── 电极1 ──(空气/水)── 电极2 ──[10k]── PB1(ADC_CH_9)
 *
 * 为什么必须这么做：
 *   PB1是高阻ADC输入，回路上没有任何泄放阻抗时，"干燥"和"有水"都测不出区别
 *   （干燥=悬空乱跳；有水=节点直接跟随激励脚电平），所以要用 MCU 内部的
 *   上拉/下拉(约30~50k)当"负载电阻"，水阻才能变成可测的电压分压。
 *
 * 测量周期（两个相位极性相反，差分抵消极化/漂移）：
 *   相位H：PB1=输入+内部下拉(负载)，PA7=输出高，等 T，PB1切模拟 -> 采样 adc_H
 *   相位L：PB1=输入+内部上拉(负载)，PA7=输出低，等 T，PB1切模拟 -> 采样 adc_L
 *   metric = adc_H + (4095 - adc_L)     干燥≈0，越湿越大
 *
 * ！！铁律！！ PB1 只在 "输入(带上/下拉)" 与 "模拟" 之间切换，永远不要配成输出，
 *            否则 PA7/PB1 电平相反时中间只有20k会灌大电流。
 *
 * ！！历史坑！！ 以前这里把激励脚写成了 PB7。而 PB7 是外设电源使能脚，
 *            于是"驱动激励"实际是在反复切外设/整板电源 -> 电源被拉低、
 *            RST_CAUSE=POR/BOR 约3秒复位一次，SWD 都连不上。
 *            现在改对了：PA7 才是电极激励。
 * =========================================================================
 */
#define PROBE_PORT          GPIOA
#define PROBE_DRIVE_PIN     GPIO_PIN_7      /* PA7 激励（配 PB1 做 ADC） */
#define PROBE_ADC_PIN       GPIO_PIN_1      /* PB1 ADC_CH_9，仍在 GPIOB */
#define PROBE_ADC_PORT      GPIOB

/* 标定用的可调参数（PROBE_SETTLE_US / PROBE_SAMPLES / PROBE_STRONG_DRIVE /
 * PROBE_DEBUG）都放在 water_cu.h 里，方便集中修改 */

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

/* PB1 模式：0=模拟(ADC)  1=输入+内部上拉  2=输入+内部下拉 */
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
        /* 数字输入 + 无上下拉：配"板上有外部下拉电阻"的场合用。
         * ⚠️ 不能用 mode 0(模拟)：模拟模式会关掉施密特触发器，EXTI 就收不到边沿了。 */
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

/**
 * @brief 按"原始模式"做一次采样（标定/排查用，能任意组合 PB1 与 PA7 的模式）
 * @param pb1_mode 0=模拟(无负载) 1=输入+内部上拉 2=输入+内部下拉
 * @param drv_mode 0=高阻 1=弱高 2=弱低 3=推挽高 4=推挽低
 * @retval 该组合下的 ADC 平均值
 * @note  【关键】采样期间 PB1 必须保持"输入 + 内部上/下拉"当负载。
 *        以前这里调 BSP_ADC_LevelProbe_Init()，它会把 PB1 切成"模拟 + 无上下拉"，
 *        等于把负载电阻拆掉：节点一旦被充电就再也不泄放，只会保持上一相位的电荷，
 *        于是干态/湿态都读到 ~VDD（就是"探针一直是高电平"的原因）。
 *        模拟模式下 PB1 的漏电时间常数约 1~3 秒，所以 nodrive 那列会看到
 *        "从 1600 慢慢掉到 0"的慢漂移 —— 这也从侧面证明节点是悬空的。
 */
static uint16_t probe_raw(uint8_t pb1_mode, uint8_t drv_mode)
{
    uint32_t sum = 0U;
    uint8_t  i;

    /* 1. 两个脚先都回高阻，避免切换过程中对顶 */
    probe_drive_set_mode(0U);
    pb1_set_mode(0U);
    delay_us(50U);

    /* 2. 先挂负载(PB1 内部上/下拉)，再加激励 —— 顺序不能反 */
    pb1_set_mode(pb1_mode);
    probe_drive_set_mode(drv_mode);

    /* 3. 等节点电压稳定 */
    delay_us(PROBE_SETTLE_US);

    /* 4. 只初始化 ADC 本体，PB1 的上下拉/输入模式保持不动（负载还在） */
    BSP_ADC_LevelProbe_AdcInit();
    for (i = 0U; i < PROBE_SAMPLES; i++)
    {
        sum += BSP_ADC_LevelProbe_ReadRaw();
    }
    BSP_ADC_LevelProbe_DeInit();

    /* 5. 收尾：激励脚回高阻、PB1回高阻，避免长期往水里灌电流 */
    probe_drive_set_mode(0U);
    pb1_set_mode(0U);

    return (uint16_t)(sum / PROBE_SAMPLES);
}

/**
 * @brief 做一次单相位采样
 * @param pb1_pull   1=内部上拉 2=内部下拉
 * @param drive_high 1=激励脚(PA7)给高 0=给低
 * @retval 该相位的ADC平均值
 */
static uint16_t probe_sample_phase(uint8_t pb1_pull, uint8_t drive_high)
{
    return probe_raw(pb1_pull, (drive_high != 0U) ? DRV_MODE_HIGH : DRV_MODE_LOW);
}

/**
 * @brief 【标定/排查用】一次给出 6 个原始组合的采样值，用来判断"上下拉到底有没有生效"
 * @param out 长度 >= 6 的数组，依次为：
 *   [0] PA7高阻 + PB1上拉      -> 必须≈4095（证明内部上拉生效、ADC 能读到 VDD）
 *   [1] PA7高阻 + PB1下拉      -> 干态必须≈0  （证明内部下拉生效）
 *   [2] PA7推挽高 + PB1下拉     -> 正向分压（有水下明显升高）
 *   [3] PA7推挽低 + PB1上拉     -> 反向分压（有水下明显降低）
 *   [4] PA7弱高   + PB1下拉     -> 弱驱动正向（原 H）
 *   [5] PA7弱低   + PB1上拉     -> 弱驱动反向（原 L）
 * @note 判读：
 *   若 [0] 不是≈4095 或 [1] 不是≈0（PA7 都没驱动时），说明内部上下拉没生效/
 *   板上还有别的器件拉这个网络 —— 那是硬件问题，软件再怎么调都测不准。
 *   若 [0]/[1] 正常而 [2]~[5] 恒为 VDD，说明负载在采样时被拆掉了（见 probe_raw 注释）。
 */
void WaterDetect_ReadMatrix(uint16_t *out)
{
    if (out == NULL)
    {
        return;
    }

    out[0] = probe_raw(1U, 0U);
    out[1] = probe_raw(2U, 0U);
    out[2] = probe_raw(2U, 3U);
    out[3] = probe_raw(1U, 4U);
    out[4] = probe_raw(2U, DRV_MODE_HIGH);
    out[5] = probe_raw(1U, DRV_MODE_LOW);
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

/**
 * @brief 双向(方案C)测量一次
 * @param adc_h [out] 正向( 激励脚PA7=H, PB1内部下拉 )采样值，可传NULL
 * @param adc_l [out] 反向( 激励脚PA7=L, PB1内部上拉 )采样值，可传NULL
 * @retval metric = adc_h + (4095 - adc_l)，干燥≈0，越湿越大
 */
uint16_t WaterDetect_ReadBiPolar(uint16_t *adc_h, uint16_t *adc_l)
{
    uint16_t h;
    uint16_t l;

    h = probe_sample_phase(2U, 1U);     /* PB1下拉(负载) + 激励脚给高 */
    l = probe_sample_phase(1U, 2U);     /* PB1上拉(负载) + 激励脚给低 */

    if (adc_h != NULL)
    {
        *adc_h = h;
    }
    if (adc_l != NULL)
    {
        *adc_l = l;
    }

    return (uint16_t)((uint32_t)h + (4095U - (uint32_t)l));
}

/**
 * @brief 只读一次探针ADC(PB1)，不驱动PB7（硬件排查用）
 */
uint16_t WaterDetect_ReadAdcOnly(void)
{
    uint16_t raw;

    BSP_ADC_LevelProbe_Init();
    raw = BSP_ADC_LevelProbe_ReadRaw();
    BSP_ADC_LevelProbe_DeInit();

    return raw;
}

/**
 * @brief 【数字方式】判水：PA7 输出高激励，PB1 配"输入+内部下拉"，直接读电平
 * @retval 1=PB1 读到高(有水) 0=低(无水)
 * @note  判水条件 ≈ 水阻 < 下拉电阻：
 *          内部下拉 ≈40k(容差±50%) → 只能测到 Rw < ~20kΩ 的水；
 *          换成外部 100k/1M 下拉则 Rw < 80k/980k，且阈值确定。
 *        干态没有回路 → 0 电流；湿态才有电流（1M 下拉约 3µA）。
 *        ⚠️ 绝不能把 PB1 配成推挽输出，否则和 PA7 对顶。
 */
uint8_t WaterDetect_ReadDigital(void)
{
    uint8_t hi;

    pb1_set_mode(2U);                   /* PB1 = 输入 + 内部下拉（干态给确定低电平） */
    probe_drive_set_mode(3U);           /* PA7 = 推挽输出高（激励） */
    delay_us(PROBE_SETTLE_US);          /* 等节点稳定（电极有容性，可加大） */

    hi = (GPIO_ReadInputDataBit(PROBE_ADC_PORT, PROBE_ADC_PIN) != 0U) ? 1U : 0U;

    probe_drive_set_mode(0U);           /* 收尾：PA7 回高阻 */
    pb1_set_mode(0U);                   /*       PB1 回高阻 */

    return hi;
}

/* ==========================================================================
 *                          【电压法】判水（当前采用）
 * --------------------------------------------------------------------------
 * 和万用表量水一个道理：
 *   PA7 = 推挽输出高     —— 表笔搭 VCC（激励）
 *   PB1 = 输入+内部下拉  —— 测量点的泄放负载(≈40k)
 *   空气：回路断开 -> PB1 被下拉钳到 0V，读数≈0
 *   触水：回路接通 -> PB1 出现明显电压，读数升高
 * 单次测量就够（不用正反两相），空气确定是 0、不再悬空乱跳。
 * ========================================================================== */

/**
 * @brief 【电压法】读一次探针电压(ADC原始值)
 */
uint16_t WaterDetect_ReadVoltageRaw(void)
{
    /* PB1=输入+内部下拉(负载/泄放，采样期间必须保持)  PA7=推挽输出高(激励)
     * ⚠️ 不能用"弱高(内部上拉)驱动"：那样激励源自己就是 40k，
     *    和水阻、下拉三者串起来分压，触水电压也上不去。这里要的是"表笔接 VCC"。 */
    return probe_raw(2U, 3U);
}

uint16_t WaterDetect_ReadVoltageMv(uint16_t vdd_mv)
{
    return (uint16_t)(((uint32_t)WaterDetect_ReadVoltageRaw() * (uint32_t)vdd_mv) / 4095U);
}

uint16_t WaterDetect_ReadBaseRaw(void)
{
    /* PA7=高阻(不激励) + PB1=输入+内部下拉：空气/触水都该≈0 */
    return probe_raw(2U, 0U);
}

/* 判水状态记录（供文件后面的 WaterDetect_WakeArm 决定要不要"武装等水唤醒"）：
 *   s_scanned  : 做过判水没有（没做过 -> 绝不武装，免得低功耗台架里白驱动电极）
 *   s_last_wet : 最近一次判水结果（已湿 -> 不武装：没必要，且避免直流长期加在湿电极上） */
static uint8_t s_last_wet = 0U;
static uint8_t s_scanned  = 0U;

WaterDetectStatus_t WaterDetect_Scan(void)
{
    uint16_t raw = WaterDetect_ReadVoltageRaw();
    WaterDetectStatus_t st;

    /* 触水 -> PB1 电压升高 -> 判为有水；空气 -> ≈0 -> 无水 */
    if (raw >= WATER_DETECT_WET_RAW_THRESHOLD)
    {
        st = WATER_STATUS_HAS_WATER;
    }
    else
    {
        st = WATER_STATUS_NO_WATER;
    }

    /* 记下最近一次判水结果：睡前用它决定要不要"武装等水唤醒"（见 WaterDetect_WakeArm）。
     * 没做过判水（s_scanned=0）就绝不武装 —— 免得在低功耗台架里白驱动电极。 */
    s_last_wet = (st == WATER_STATUS_HAS_WATER) ? 1U : 0U;
    s_scanned  = 1U;

    return st;
}

/* ==========================================================================
 *                      【有水自动唤醒】（2026-10-07 新增）
 * --------------------------------------------------------------------------
 * 思路：把"电压法判水"的接法【整个保持在睡眠期间】：
 *     PA7 = 推挽输出高（激励，"表笔接 VCC"）
 *     PB1 = 输入 + 下拉（泄放负载） + EXTI 上升沿中断
 *   干态：没有回路 -> PB1 被下拉到 0V -> 没有边沿、**0 电流** ✅
 *   触水：回路接通 -> PB1 电压升高 -> 上升沿 -> 把 STOP 里的 MCU 立刻叫醒 ✅
 *   （EXTI 线 1 属于 EXTI0_1_IRQn，和 PA0 共向量；PA0 是 SPI1_SCK，没配 EXTI，不冲突）
 *
 * ★★ 能不能叫醒，全看分压：湿态 PB1 = VDD·Rpd/(Rpd + 10k + Rw + 10k) ★★
 *   电极两端各串 10k（见文件头接线图），Rw = 水的等效电阻。
 *   数字输入门限约 0.5·VDD ≈ 1.65V（注意：判水用的 ADC 阈值只有 0.65V，门槛低得多）：
 *     内部下拉 40k  -> 需要 Rw < ~20kΩ  才叫得醒
 *     外部下拉 1M   -> 需要 Rw < ~980kΩ（湿态电流只有 ~3µA）✅ 推荐
 *   按现场读数反推：判水刚好在 800counts(0.65V) 附近 => Rw ≈ 100kΩ，
 *   所以【内部下拉这一档大概率叫不醒】—— 这时必须给 PB1 加一颗外部 1MΩ 下拉，
 *   并把 PROBE_WAKE_EXT_PULLDOWN 改成 1（同时固件不能用内部下拉，否则 40k 会把 1M 淹掉）。
 *
 * ⚠️ 腐蚀/功耗：激励是直流，若一直加在湿电极上会极化/腐蚀、还一直吃电流。
 *    所以只在【上一次判定为"无水"】时才武装（等水出现）；
 *    一旦判定有水，就撤防、退回 RTC 定时唤醒 —— 见 WaterDetect_WakeArm()。
 * ========================================================================== */
static volatile uint8_t s_wake_flag = 0U;   /* EXTI 中断置位，主循环读后清零 */
static uint8_t          s_wake_veto = 0U;   /* 一次性否决：置 1 后 WakeArm 永远撤防 */

/**
 * @brief 【一次性】永久否决"等水唤醒"（解绑后走死路时用）
 * @note  解绑后设备只剩"等下次上电"，不再需要被水叫醒；这里顺手撤防，
 *        免得设备长时间不插拔时电极还被直流激着（撤防后 PA7/PB1 回高阻）。
 *        置位后本函数不会被取消，复位才清。
 */
void WaterDetect_WakeDisable(void)
{
    s_wake_veto = 1U;
    (void)WaterDetect_WakeArm(0U);
}

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
 * @brief 睡前武装/撤防"等水唤醒"
 * @param arm 1 = 请求武装（内部还会再看"是否判过水 + 上次是不是无水"）
 * @retval 1 = 真的武装了；0 = 没武装（已湿/没判过水/功能关闭）
 * @note  必须在 BSP_GPIO_SetAllAnalog() 之后、关 GPIO 时钟之前调用。
 */
uint8_t WaterDetect_WakeArm(uint8_t arm)
{
#if (PROBE_WAKE_ON_WATER)
    if ((arm == 0U) || (s_wake_veto != 0U) || (s_scanned == 0U) || (s_last_wet != 0U))
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

    /* 1) 先配 EXTI（这个 BSP 函数会把 PB1 配成"输入+内部上拉"，下面再覆盖成下拉/无拉） */
    BSP_EXTI_WakeupPinConfig(PROBE_ADC_PORT, PROBE_ADC_PIN, EXTI_Trigger_Rising);

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
