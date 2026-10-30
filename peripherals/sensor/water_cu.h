#ifndef __WATER_CU_H__
#define __WATER_CU_H__

#include "n32g031.h"

/* ==================== 探针（水位检测） ====================
 * 接线（本板实物）：PA7 ─[110k]─ 电极1 ─(空气/水)─ 电极2 ─[110k]─ PB1(ADC_CH_9)
 *                   PB1 ─[1M]─ GND
 *   PA7 = 激励脚（配 PB1 做 ADC，做 GPIO 即可）；PB1 = 检测脚(ADC_CH_9)。
 *   ⚠️ PB7 是外设(LoRa/液位模块)电源使能，不是探针脚。
 * 电压法：PA7 推挽输出高（表笔接 VCC），PB1 输入 + 外部 1M 下拉当泄放负载；
 *   空气 -> 回路断开 -> raw≈0；触水 -> raw 明显升高；短接 raw≈3350；
 *   本板实测水中(Rw≈100k) raw≈3100，门限 800 很安全。
 * ⚠️ 门限与唤醒分压都是按 110k×2 + 1M 定的，别按 10k 反推。
 * 【铁律】PB1 只在"输入（带上/下拉）"与"模拟"之间切换，永不配置成推挽输出。
 * ========================================================= */
#ifndef WATER_DETECT_WET_RAW_THRESHOLD
/* 判"有水"的 ADC 门槛（12 位，4095 = VDD）。
 * 取 800(≈0.65V)：给"潮湿但没真正泡到水"留余量，干态离 0 也有很大噪声裕量。 */
#define WATER_DETECT_WET_RAW_THRESHOLD  800U
#endif

/* ==================== 自适应充电（"充到稳态或超时"） ====================
 * 电极泡在水里是 RC：双电层电容 + 探针线电容，串上 220k 后 τ 实测 0.3~3s；
 * 固定等 200ms 只充到百分之几，于是"泡在水里判干、短接却正常"。
 * 改法：每 PROBE_CHARGE_STEP_MS 采一点，连续 PROBE_STABLE_STEPS 次变化都小于
 * PROBE_STABLE_DELTA 就认为稳定、提前结束；最多等 PROBE_CHARGE_MAX_MS。
 * ==================================================================== */
#ifndef PROBE_CHARGE_MAX_MS
#define PROBE_CHARGE_MAX_MS   1200U     /* 单轮上限：高阻水 1.2s 也够充满，开路则走满 */
#endif
#ifndef PROBE_CHARGE_STEP_MS
#define PROBE_CHARGE_STEP_MS  100U      /* 每 100ms 采一个点，够分辨 τ 0.1~3s 的充电曲线 */
#endif
#ifndef PROBE_STABLE_DELTA
#define PROBE_STABLE_DELTA    60U       /* 相邻两点变化 ≤60 counts(≈48mV) 视为不再上升 */
#endif
#ifndef PROBE_STABLE_STEPS
#define PROBE_STABLE_STEPS    2U        /* 连续 2 次都"不再上升"才算稳态，避开单点噪声 */
#endif

/* 每一轮测量前先把节点电荷放掉多少毫秒（PA7 推挽低 + PB1 内部下拉 40k）。
 * 取 150ms：放电电阻 40k∥外部 1M ≈ 38.5k，µF 级节点也能放到 2% 以下，
 * 保证每轮都从接近 0V 起步（不放会读到上一轮残留电荷，结论自相矛盾）。 */
#ifndef PROBE_DISCHARGE_MS
#define PROBE_DISCHARGE_MS    150U
#endif
/* 判水做几轮、至少几轮判"有水"才算有水（多轮投票）。
 * 3 轮里 2 轮为湿 -> 判湿：宁可误判湿（多测一次超声波），也不要误判干（可能永久睡死）。 */
#ifndef PROBE_ROUNDS
#define PROBE_ROUNDS          3U
#endif
#ifndef PROBE_WET_MIN_VOTES
#define PROBE_WET_MIN_VOTES   2U
#endif
/* 每个相位的采样次数(取平均，抗 50Hz 干扰) */
#ifndef PROBE_SAMPLES
#define PROBE_SAMPLES       4U
#endif
/* ==================== 【有水自动唤醒】开关 ====================
 * 1（默认）= 睡前若"上次判无水"，就把 PA7 推挽输出高、PB1 输入+下拉、开 EXTI 上升沿；
 *            水碰到电极即叫醒 STOP 里的 MCU（日志 wakeup (WATER)），干态 0 电流。
 * 0        = 关掉，只按 RTC 定时唤醒。
 * 注意：能否叫醒由分压决定（湿态 PB1 = VDD·Rpd/(Rpd + 220k + Rw)），数字门限约 0.3~0.7VDD：
 *   内部下拉 40k -> 短接也只 0.51V：【测不出水、也叫不醒】；
 *   外部下拉 110k-> 短接 1.10V：能判水但过不了数字门限；
 *   外部下拉 1M  -> 短接 2.71V，Rw ≲ 200k~780k 都可叫醒，湿态仅 ~3µA（本板采用）。
 * ============================================================= */
#ifndef PROBE_WAKE_ON_WATER
#define PROBE_WAKE_ON_WATER     1
#endif

/* 唤醒时 PB1 的负载：
 *   0 = MCU 内部下拉(≈40k) —— 只在没加外部下拉且串阻很小时可用
 *   1 = "输入 + 无上下拉"，靠板上外部下拉
 *       【本板：110k×2 + 外部 1MΩ -> 取 1】⚠️ 没焊外部下拉会悬空乱唤醒 */
#ifndef PROBE_WAKE_EXT_PULLDOWN
#define PROBE_WAKE_EXT_PULLDOWN 1
#endif

/* ==================== 【水唤醒：要不要做保护判断】 ====================
 * 启动时设置一次：
 *   不调用（默认）= 只有"判过水且上次判无水"才武装（防电极长期加直流）
 *   WaterDetect_WakeAlwaysArm(1) = 无条件武装，有水就唤醒（出厂未绑定等水）
 * ========================================================================= */

/**
 * @brief 设置"水唤醒是否无条件武装"(enable=1 无条件，0 = 带保护判断)；出厂流程启动时调一次
 */
void WaterDetect_WakeAlwaysArm(uint8_t enable);

/* ==================== 【探针一直工作】 ====================
 * PA7 常驻 3.3V、PB1 常驻数字输入 + EXTI 双边沿，任何时候都在检测电平变化。
 * ⚠️ 采样函数内部会把两脚释放，所以每次采样后要再调一次 WaterDetect_ProbeAlwaysOn()。
 * ======================================================== */

/**
 * @brief 让探针一直工作：PA7 = 推挽输出高(3.3V)、PB1 = 数字输入 + EXTI 双边沿
 * @note  出厂配对流程用；调用后即使不采样，PB1 也一直在检测电平变化
 */
void WaterDetect_ProbeAlwaysOn(void);

/**
 * @brief 读当前 PB1 的数字电平（不改动引脚配置）
 * @retval 1 = 高(有水)，0 = 低(无水)
 */
uint8_t WaterDetect_ProbeLevel(void);

/**
 * @brief 最近一次判水采样的 ADC 原始值(0~4095)；只给日志/排查用，干态应≈0，门限 800
 * @note  ⚠️ "泡在水里却判 DRY"先看它：raw≈0 -> PA7 没驱动或回路真断；0<raw<800 -> 只是没过门限。
 */
uint16_t WaterDetect_LastRaw(void);

/**
 * @brief 最近一次判水的【第一个采样点】(加激励后 100ms 的值，0~4095)
 * @note  首点≈0 且稳态≈0、用时走满 -> 回路真断；首点小、稳态大 -> 正常泡水；首点近满量程 -> 短接。
 */
uint16_t WaterDetect_LastRawFast(void);

/**
 * @brief 最近一次判水中"最好那一轮"实际充电用了多少毫秒（直接反映回路 τ）
 * @note  短接/极好接触 200~300ms；自来水/正常泡水 400~900ms；走满上限则回路阻抗很高。
 */
uint16_t WaterDetect_LastChargeMs(void);

/**
 * @brief 最近一次判水时"几轮投票里判为有水的轮数"
 * @note  PROBE_ROUNDS 轮里 >= PROBE_WET_MIN_VOTES 轮为湿才判湿（见 water_cu.c 的 Scan）。
 */
uint8_t WaterDetect_LastWetVotes(void);

/**
 * @brief 最近一次判水采样时 PB1 的数字电平(0/1)
 * @note  和 EXTI 唤醒用的是同一个施密特门限，是"睡下去还能不能叫醒"的依据。
 */
uint8_t WaterDetect_LastLvl(void);

typedef enum
{
    WATER_STATUS_NO_WATER  = 0,   // 无水
    WATER_STATUS_HAS_WATER = 1,   // 有水
}WaterDetectStatus_t;

/**
 * @brief 水位检测初始化（激励脚 PA7 / 检测脚 PB1 都置高阻，不驱动）
 */
void WaterDetect_Init(void);

/**
 * @brief 【电压法】读一次探针电压(ADC原始值 0~4095)：PA7 推挽高激励，PB1 靠板上 1M 下拉当负载
 * @note  空气≈0，触水明显升高；判水就用它与 WATER_DETECT_WET_RAW_THRESHOLD 比较。
 */
uint16_t WaterDetect_ReadVoltageRaw(void);

/**
 * @brief 执行一次水位检测（电压法：raw >= WATER_DETECT_WET_RAW_THRESHOLD 判为有水）
 * @retval WATER_STATUS_NO_WATER / WATER_STATUS_HAS_WATER；结果会记下来供 WakeArm() 用
 */
WaterDetectStatus_t WaterDetect_Scan(void);

/* ==================== 【有水自动唤醒】（见上面的开关说明） ==================== */

/**
 * @brief 睡前武装/撤防"等水唤醒"；arm=1 请求武装（内部再看"判过水没有 + 上次是否无水"）
 * @note  返回 1=真的武装了，0=没武装；须在 BSP_GPIO_SetAllAnalog() 之后、关 GPIO 时钟之前调。
 */
uint8_t WaterDetect_WakeArm(uint8_t arm);

/**
 * @brief 用当前状态推一遍"睡前会不会真的武装等水唤醒"（不改任何东西）；1=会武装，0=会撤防
 * @note  main.c 的 probe-wake 日志必须用它，别拿 dry 状态猜，否则日志与实际行为不一致。
 */
uint8_t WaterDetect_WouldArm(void);

/**
 * @brief 取"是不是被水叫醒的"标志（读后清零）
 * @retval 1 = 本次唤醒来自 PB1 上升沿（探针碰到水）
 */
uint8_t WaterDetect_TakeWakeFlag(void);

/**
 * @brief 【中断里调】PB1 上升沿处理：只置标志，不做耗时动作
 */
void WaterDetect_WakeIsrHandler(void);

#endif // __WATER_CU_H__
