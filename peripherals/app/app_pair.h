/**
 * @file app_pair.h
 * @brief 从机配对/绑定模式：网关地址未绑定(FF FF FF)时运行
 *
 * 用途（需求 2026-09）：
 *   出厂时 Flash 里只有本机设备地址，网关地址 = FF FF FF。
 *   每次上电判断网关地址是否有效：
 *     有效   -> 直接进主程序（app_main_loop）
 *     无效   -> 进本模块的配对模式：
 *                每 5s 发一帧 0x04 配对请求（目的地址 FF FF FF），
 *                帧头里带上自己的设备地址，形如
 *                  AA 55 01 FF FF FF 21 00 01 04 00 BC 63
 *                收到网关应答(0x80)后，把应答帧头里的网关地址写进 Flash，
 *                然后【留在配对模式里一直回 ACK】，下次上电才进正常流程
 *                （见下面"绑定成功之后的行为"）。
 *                ⚠️ 绑定的地址取自应答帧的 gw_addr[3..5]（网关自己的地址），
 *                   所以网关回包时必须把"自己的地址"填在帧头网关字段里；
 *                   如果它把请求里的 FF FF FF 原样回填，设备取不到地址会一直重试。
 *   运行期收到 0x05 解绑 -> 先回 2 次 ACK(间隔 5s)，再把 Flash 网关地址写回 FF FF FF
 *     -> 关唤醒/关射频进 STOP，【本次运行不再进本模块】（只有复位/重新上电后，
 *        主循环读到未绑定才会再进这里发 0x04 配对）。
 *
 * 【绑定成功之后的行为】（2026-09-28 需求）
 *   网关回应 ack -> 网关地址写进 Flash/RAM -> 【不返回、不进正常流程】，
 *   留在配对模式里对收到的每一帧有效下行一直回 0x80 ACK；
 *   直到【下次上电】因为 Flash 里已有有效网关地址，才直接进正常流程。
 *   两个开关（都在 app_pair.c 顶部）：
 *     APP_PAIR_STAY_AFTER_BOUND         默认 1 = 留在配对模式；0 = 老行为(立刻进正常流程)
 *     APP_PAIR_KEEP_REQUEST_AFTER_BOUND 默认 1 = 绑定后仍每 5s 发一次 0x04（可自愈）
 *                                                0 = 绑定后不再发，只一直收 + 回 ACK
 *
 * 注意：
 *   - app_pair_run() 只在【主循环上下文】调用（内部会擦写 Flash、开射频收发）；
 *   - 【一直绑不上就不返回】，这是设计要求（未配对设备不应该继续跑上报业务）。
 *   - 测试阶段不考虑低功耗：醒着一直"发一帧 + 收满 5s 间隔"，不进 STOP。
 *     以后要做低功耗，把接收窗口收短、剩下时间换成 RTC 唤醒进 STOP 即可。
 */
#ifndef __APP_PAIR_H__
#define __APP_PAIR_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 进入配对模式（阻塞），配对成功后把网关地址【写进 Flash】
 * @note  正式流程（MAIN_MODE_PRODUCTION）未绑定时调这个：
 *        绑定成功后返回；一直收不到应答就不返回。
 *        返回后调用方应重新读运行时状态（网关地址已更新）。
 */
void app_pair_run(void);

/**
 * @brief 进入配对模式（阻塞），配对成功后【只改 RAM，不写 Flash】
 * @note  台架配对测试（MAIN_MODE_PAIR_TEST）用：配对成功后返回，调用方一般就停住；
 *        因为没落 Flash，下次上电网关地址又是 FF FF FF，会重新配对。
 */
void app_pair_run_ramonly(void);

#ifdef __cplusplus
}
#endif
#endif /* __APP_PAIR_H__ */
