/**
 * @file app_pair.h
 * @brief 从机配对/绑定：网关地址未绑定（13×FF）时运行
 *
 * 外层流程在 main.c 的 app_factory_run()：未绑定设备先睡在"等水唤醒"状态，
 * 短接探针把它叫醒、判到有水之后才进本模块配对。
 *
 * 一轮配对：
 *   每 5s 发一帧 0x04（目的 13×FF = 任意网关，帧头带本机 13 字节设备地址），形如：
 *     AA 55 01 | FF×13 | 21 01 02 03 04 05 06 07 08 09 0A 0B 0C | 04 00 | C8 46
 *   收到网关应答(0x80)后把应答帧头里的网关地址写进 Flash。
 *   ⚠️ 网关回包必须把"自己的地址"填在帧头网关字段；原样回填 13×FF 的话设备取不到地址，
 *      会一直重试。
 *   最多等 APP_PAIR_WAIT_ANSWER_MS(60s)，超时返回 false 让调用方回去睡。
 *
 * 绑定成功后：持续发 10s 的 0x80 ACK（每 2s 一帧，期间收到下行也回 ACK），
 *   然后返回 true，由调用方复位；复位后 Flash 里已有有效网关地址 -> 直接进主流程。
 *
 * 注意：只在主循环上下文调用（内部擦写 Flash、开射频收发），且是阻塞的。
 */
#ifndef __APP_PAIR_H__
#define __APP_PAIR_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 配对一轮（阻塞）：发 0x04 等网关应答，最多等 APP_PAIR_WAIT_ANSWER_MS(默认 60s)
 * @retval true  = 绑定成功（网关地址已写 Flash），并已发完 10s 的 0x80 ACK -> 调用方应复位
 *         false = 60s 内没等到应答 -> 调用方回去睡（等下次水唤醒/短接）
 */
bool app_pair_run(void);

#ifdef __cplusplus
}
#endif
#endif /* __APP_PAIR_H__ */
