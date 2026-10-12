#ifndef __WATER_CU_H__
#define __WATER_CU_H__

#include "n32g031.h"

/* ==========================================================================
 *                        探针(水位检测) 进度记录  2026-xx-xx 暂停
 * ==========================================================================
 *
 * 【接线】两端各串 10k：
 *     PA7 ──[10k]── 电极1 ──(空气/水)── 电极2 ──[10k]── PB1(ADC_CH_9)
 *   - PA7 = 激励/放电脚（配 PB1 做 ADC；不需要 ADC 复用，做 GPIO 即可）
 *   - PB1 = ADC 检测脚（ADC_CH_9）
 *   - ⚠️【2026-09-20 更正】PB7 不是探针脚，PB7 是外设(LoRa/液位模块)电源使能！
 *     本文档早期版本把 PA7/PB7 写反了，导致固件拿"外设电源"的 API 去驱动探针、
 *     又拿探针的去驱动电源脚 —— 后者正是"推挽驱动就把电源拉低、POR/BOR 反复复位"
 *     的真正原因（那是在反复切外设/整板供电，见 bsp_gpio.c）。现已改回：
 *     激励=PA7(GPIOA)，电源使能=PB7(GPIOB)。
 *
 * 【本轮试过的方案与结论】
 *   方案1  单次采样：PB7拉低300us放电 -> 拉高 -> 200us -> 读PB1
 *          结论：不可用。PB1是高阻ADC输入，回路上没有任何泄放阻抗时，
 *                干燥=悬空乱跳(0~4095)，有水=节点直接跟随PB7电平，测不出水阻。
 *   方案2  正反向差分：PB7推挽输出高/低各采一次再取差值
 *          结论：不可用。**推挽驱动PB7会把电源拉低，RST_CAUSE=POR/BOR，
 *                约3秒复位一次，连SWD烧录都会失败**。
 *                注意：原理图上串了10k，理论上限流0.33mA，正常电源不该被拉垮，
 *                所以怀疑 PB7 网络上还挂着别的器件(对顶)，或供电本身很弱。
 *   方案3  方案C(当前实现)：PB1用"内部上拉/下拉"当负载 + PB7双向激励 + 差分
 *          相位H：PB1=输入+内部下拉(负载)，PB7给高，等 PROBE_SETTLE_US，
 *                 PB1切模拟 -> 采样 adc_H
 *          相位L：PB1=输入+内部上拉(负载)，PB7给低，等 PROBE_SETTLE_US，
 *                 PB1切模拟 -> 采样 adc_L
 *          metric = adc_H + (4095 - adc_L)     干燥≈0，越湿越大
 *          结论：**仍未跑通，暂停**。实测仍会 POR/BOR 复位；偶尔测到
 *                adc_H≈296~303 / adc_L≈520~692 (metric 3706~3871，误判为WATER)；
 *                接杜邦线后干扰很大(50Hz/悬空耦合)。
 *   方案3b 弱驱动：PB7 只用内部上拉/下拉当激励源(几十uA)，见 PROBE_STRONG_DRIVE=0
 *          结论：对"复位"和"干扰"都没有明显改善。
 *
 * 【未解决 / 下次继续时的检查清单】
 *   1) 断电用万用表量：
 *      - PB7↔GND、PB1↔GND 电阻（应≈10k或开路；接近0Ω=板上短路）
 *      - 两探针短接时 PB7↔PB1 电阻（应≈20k，两颗10k串联）
 *        -> 若几乎0Ω，说明那两颗10k没串上，实际接线和设想不一致
 *      - 对照原理图确认 PB7 是否还接到别的器件
 *   2) 复位到底是不是电源跌落：看测试日志里的 [probe] vdd=xxxxmV
 *      （跌下去=供电弱；一直稳却复位=其它复位源）
 *   3) 干扰治理：探针线改屏蔽线/双绞线，PB1节点加 100nF 到 GND，
 *      采样次数加大(如16~32次平均)，必要时硬件RC低通
 *
 * 【推荐的下一个硬件改动（改动很小，最省事）】
 *   PB1 节点对 GND 加一颗 1MΩ 下拉(+可选100nF)：
 *     干燥：节点被1M拉到0V -> ADC≈0 稳定，不再悬空
 *     有水：水阻与1M分压 -> ADC明显升高
 *   这样只读ADC、甚至不用驱动PB7就能判水，是最保守可靠的方案(方案A)。
 *
 * 【代码里的可调开关】
 *   PROBE_STRONG_DRIVE  0=弱驱动(内部上/下拉，默认)  1=PB7推挽强驱动(危险)
 *   PROBE_SETTLE_US     每个相位稳定时间(默认200us)
 *   PROBE_SAMPLES       每相位采样次数(默认4,抗工频干扰可加到16~32)
 *   PROBE_DEBUG         1=打印每个相位明细(默认1)
 *   WATER_DETECT_METRIC_THRESHOLD  判水阈值，目前还是占位值(2500)，未标定
 *
 * 【铁律】PB1 只在 "输入(带内部上/下拉)" 和 "模拟" 之间切换，
 *         永不配置成推挽输出！否则 PB7/PB1 电平相反时中间只有20k会灌大电流。
 *
 * 【当前测试用法】main.c 里每300ms打印 adc_H/adc_L/metric/vdd，可直接标定。
 *   标定方法：干燥 / 纸巾沾水半湿 / 浸水 三组 metric，取中间值填阈值。
 *
 * 【根因结论：为什么以前 H/L 恒为高电平（2026-09-20 定位）】
 *   采样那一步以前调的是 BSP_ADC_LevelProbe_Init()，它会顺手把 PB1 配成
 *   "模拟 + 无上下拉" —— 等于在每个相位采样前把唯一的负载电阻拆掉了。
 *   于是节点既不放也不充，只是"记住"上一个相位留下的电荷（漏电时间常数
 *   约 1~3 秒），所以无论干燥还是有水、无论 PA7 给高还是给低，读回来都是
 *   ~VDD（H≈3850 L≈3950）。日志里 nodrive 从 1600 慢慢掉到 0，正是悬空节点
 *   被极小的漏电流缓慢放电的表现 —— 反过来证明负载当时是断开的。
 *   修法：采样时必须保持 PB1 = "输入 + 内部上拉/下拉"，只初始化 ADC 本体
 *   （改用 BSP_ADC_LevelProbe_AdcInit()）。
 *
 *   另外：旧日志后半段 dig 由 1 变 0、H 从 3850 掉到 3600，是探针离开水面
 *   造成的，但 H 依然是 3600 —— 干态也读到高电平，正说明读数与水无关。
 *
 * 注：本文件注释建议用 UTF-8 打开保存（Keil: Edit -> Configuration -> Encoding
 *     -> UTF-8），否则中文注释会被按ANSI保存成乱码。
 * ========================================================================== */

/* 湿度判据阈值：!未标定!
 * 干燥≈0；越湿越大；先按"干燥"和"浸水"两组实测值取中间填这里 */
#define WATER_DETECT_METRIC_THRESHOLD   2500U

/* ==========================================================================
 * 【当前采用】电压法判水（"和万用表量水一个道理"）
 * --------------------------------------------------------------------------
 * 接线不变：PA7 ──[10k]── 电极1 ──(空气/水)── 电极2 ──[10k]── PB1
 * 做法：
 *   PA7 = 推挽输出高  —— 相当于把表笔搭到 VCC（激励）
 *   PB1 = 输入 + 内部下拉(≈40k) —— 相当于给测量点一个泄放负载
 *   空气：回路断开，PB1 被内部下拉钳到 0V   -> 读数≈0mV（不再悬空乱跳）
 *   触水：水把回路接通，PB1 上出现明显电压   -> 读数明显升高
 * 判据：PB1 电压 >= WATER_DETECT_WET_RAW_THRESHOLD(ADC原始值) 即为有水。
 * 优点：比"双向差分 metric"直白，空气确定是 0，单次测量就够，省电省时间。
 * 代价：阈值由内部下拉(≈40k，容差±50%)决定，等效"水阻 < ~20kΩ"；
 *       水阻偏大（薄水膜/纯净水）会判不出 —— 那时要么减小串联电阻，
 *       要么在 PB1 对 GND 加一颗 100k/1M 的外部下拉（阈值立刻变确定值，
 *       并且 PB1 的 EXTI 还能做"一有水就唤醒"）。
 * ⚠️ PA7 只在采样那几百 µs 内被驱动，测完立刻释放，不能常开。
 * ========================================================================== */
#ifndef WATER_DETECT_WET_RAW_THRESHOLD
#define WATER_DETECT_WET_RAW_THRESHOLD  800U    /* ≈0.65V@3.3V；按实测"空气/触水"改 */
#endif

/* ==================== 探针标定用的可调参数（都集中在这里改） ==================== */
/* 每个相位的稳定时间(us)：电极有双层电容+极化，200us 很可能还没稳。
 * 标定时依次试 200 / 1000 / 5000 / 20000，看 metric 是否变稳、是否不再随时间漂 */
#ifndef PROBE_SETTLE_US
#define PROBE_SETTLE_US     200U
#endif
/* 每个相位的采样次数(取平均，抗 50Hz 干扰) */
#ifndef PROBE_SAMPLES
#define PROBE_SAMPLES       4U
#endif
/* 打印每个相位细节 */
#ifndef PROBE_DEBUG
#define PROBE_DEBUG         1
#endif
/* 激励方式：
 *   0 = 弱驱动：PA7 只用"内部上拉/下拉"当激励源，电流几十 uA
 *   1 = 强驱动：PA7 推挽输出高/低，10k+10k 限流约 0.33mA
 * ⚠️ 历史记录里"强驱动会把电源拉低、POR/BOR 复位"是【引脚搞反时期】的现象
 *    （那时驱动的其实是 PB7 = 外设电源脚）。现在驱动的是 PA7（真正的电极激励脚），
 *    所以强驱动可以放心试。 */
#ifndef PROBE_STRONG_DRIVE
#define PROBE_STRONG_DRIVE  0
#endif

/* ==================== 【有水自动唤醒】开关（2026-10-07 新增） ====================
 * 1（默认）= 允许"睡着等水来"：睡前若【上一次判水是无水】，就把
 *            PA7 推挽输出高、PB1 输入+下拉、并开 EXTI 上升沿中断；
 *            水一碰到电极就把 STOP 里的 MCU 立刻叫醒（日志打 wakeup (WATER)）。
 *            干态没有回路 -> 0 电流；只有湿了才有电流。
 * 0        = 关掉，只按 RTC 定时唤醒（老行为）。
 *
 * ❗能不能叫醒由分压决定（湿态 PB1 = VDD·Rpd/(Rpd+10k+Rw+10k)，电极两端各串 10k）：
 *     数字输入门限约 0.5·VDD ≈ 1.65V（判水用的 ADC 阈值只有 0.65V，门槛低得多）
 *       内部下拉 40k  -> 需要水阻 Rw < ~20kΩ  才叫得醒
 *       外部下拉 1M   -> 需要 Rw < ~980kΩ（湿态电流仅 ~3µA）✅ 推荐
 *   按现场读数反推 Rw ≈ 100kΩ => 【默认这档大概率叫不醒】。
 *   要让"有水自动唤醒"真正可靠：给 PB1 加一颗【外部 1MΩ 下拉】，
 *   然后把 PROBE_WAKE_EXT_PULLDOWN 改成 1。
 * ============================================================================ */
#ifndef PROBE_WAKE_ON_WATER
#define PROBE_WAKE_ON_WATER     1
#endif

/* 唤醒时 PB1 的负载：
 *   0（默认）= 用 MCU 内部下拉(≈40k)：只在【没加外部电阻】时用，Rw 要 < ~20kΩ
 *   1        = 用"输入 + 无上下拉"，靠板上【外部 1MΩ 下拉】：Rw < ~980kΩ 都行
 *              ⚠️ 配 1 时必须真的焊了外部下拉，否则 PB1 悬空会乱唤醒 */
#ifndef PROBE_WAKE_EXT_PULLDOWN
#define PROBE_WAKE_EXT_PULLDOWN 0
#endif

typedef enum
{
    WATER_STATUS_NO_WATER  = 0,   // 无水
    WATER_STATUS_HAS_WATER = 1,   // 有水
}WaterDetectStatus_t;

/**
 * @brief 水位检测初始化（PB7/PB1 都置高阻，不驱动）
 */
void WaterDetect_Init(void);

/**
 * @brief 双向(方案C)测量一次：内部上/下拉当负载 + PB7双向激励
 * @param adc_h [out] 正向(PB1内部下拉 + PB7给高)采样值，可传NULL
 * @param adc_l [out] 反向(PB1内部上拉 + PB7给低)采样值，可传NULL
 * @retval metric = adc_h + (4095 - adc_l)，干燥≈0，越湿越大
 */
uint16_t WaterDetect_ReadBiPolar(uint16_t *adc_h, uint16_t *adc_l);

/**
 * @brief 只读一次探针ADC(PB1)，完全不驱动PB7（硬件排查用，最安全）
 * @retval 0~4095
 */
uint16_t WaterDetect_ReadAdcOnly(void);

/**
 * @brief 【数字方式】判水：PA7 输出高激励，PB1 配"输入+内部下拉"，直接读电平
 * @retval 1=PB1 读到高(有水)  0=低(无水)
 * @note  业务上只需要"有水/无水"布尔量时用这个，不用 ADC、不用算 metric。
 *        阈值固定约 0.5×VDD，用内部下拉(≈40k，容差±50%)时等效"水阻 < ~20kΩ"，
 *        只能粗判；把 PB1 换成外部下拉(100k/1M)阈值就变确定值、量程也更宽，
 *        而且外部下拉 + PB1 的 EXTI 边沿还能做"有水即唤醒"。
 */
uint8_t WaterDetect_ReadDigital(void);

/**
 * @brief 【标定/排查用】一次给出 6 个原始组合的采样值
 * @param out 长度 >= 6 的数组：
 *   [0] PA7高阻+PB1上拉(应≈4095)  [1] PA7高阻+PB1下拉(干态应≈0)
 *   [2] PA7推挽高+PB1下拉         [3] PA7推挽低+PB1上拉
 *   [4] PA7弱高+PB1下拉(原H)      [5] PA7弱低+PB1上拉(原L)
 */
void WaterDetect_ReadMatrix(uint16_t *out);

/**
 * @brief 【电压法】读一次探针电压(ADC原始值 0~4095)
 * @note  PA7=推挽输出高(激励) + PB1=输入+内部下拉(负载/泄放)
 *        空气≈0；触水明显升高。判水就用这个值跟阈值比。
 */
uint16_t WaterDetect_ReadVoltageRaw(void);

/**
 * @brief 【电压法】读一次探针电压并换算成 mV（打印/标定用）
 * @param vdd_mv 当前供电电压(mV)，由 app_power_get_battery_mv() 得到
 */
uint16_t WaterDetect_ReadVoltageMv(uint16_t vdd_mv);

/**
 * @brief 【电压法】读"不激励时的底噪"：PA7高阻 + PB1内部下拉
 * @retval ADC原始值；空气和触水都应该≈0（没有激励源，回路通不通都测不到电压）
 * @note  这个值用来确认节点没有悬空/没有别的器件在拉它 —— 是电压法的健康自检。
 */
uint16_t WaterDetect_ReadBaseRaw(void);

/**
 * @brief 执行一次水位检测（电压法 raw >= WATER_DETECT_WET_RAW_THRESHOLD 判为有水）
 * @retval WATER_STATUS_NO_WATER / WATER_STATUS_HAS_WATER
 * @note  内部会记下这次结果，供 WaterDetect_WakeArm() 决定要不要"武装等水唤醒"。
 */
WaterDetectStatus_t WaterDetect_Scan(void);

/* ==================== 【有水自动唤醒】（2026-10-07 新增，见上面的开关说明） ==================== */

/**
 * @brief 睡前武装/撤防"等水唤醒"
 * @param arm 1 = 请求武装（内部还会再看"判过水没有 + 上次是不是无水"）
 * @retval 1 = 真的武装了；0 = 没武装（已湿 / 没判过水 / 开关关闭）
 * @note  必须在 BSP_GPIO_SetAllAnalog() 之后、关 GPIO 时钟之前调用；
 *        武装后 PA7 会一直输出高、PB1 保持输入+下拉并等上升沿 —— 干态 0 电流。
 */
uint8_t WaterDetect_WakeArm(uint8_t arm);

/**
 * @brief 取"是不是被水叫醒的"标志（读后清零）
 * @retval 1 = 本次唤醒来自 PB1 上升沿（探针碰到水）
 */
uint8_t WaterDetect_TakeWakeFlag(void);

/**
 * @brief 【中断里调】PB1 上升沿处理：只置标志，不做耗时动作
 */
void WaterDetect_WakeIsrHandler(void);

/**
 * @brief 【一次性】永久否决"等水唤醒"（解绑后走死路时用；复位才清）
 * @note  调用后 WaterDetect_WakeArm() 一律撤防，探针两脚回高阻、不再驱动电极。
 */
void WaterDetect_WakeDisable(void);

#endif // __WATER_CU_H__
