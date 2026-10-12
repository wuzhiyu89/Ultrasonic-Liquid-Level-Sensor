#ifndef __LOG_H__
#define __LOG_H__

/* 日志串口：USART1，TX = PB6(AF4)，115200 8N1
 * ⚠️ 液位模块的接收(模块TX -> PA10 = USART1_RX, AF4)也在 USART1 上，
 *    两者共享同一个波特率：读液位时临时切到模块波特率，读完切回本值再打印。
 *    所以 log_init() 里 Mode 必须是 TX|RX，否则会把接收方向关掉。 */
#ifndef LOG_BAUD
#define LOG_BAUD    115200U
#endif

#ifndef LOG_ENABLE
#define LOG_ENABLE 1
#endif

#if LOG_ENABLE

#include <stdio.h>

#define LOG_NONE    0
#define LOG_ERROR   10
#define LOG_WARNING 20
#define LOG_INFO    30
#define LOG_DEBUG   40

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_DEBUG
#endif

#if LOG_LEVEL >= LOG_INFO
#define log_info(...) printf(__VA_ARGS__)
#else
#define log_info(...)
#endif

#if LOG_LEVEL >= LOG_ERROR
#define log_error(...) printf(__VA_ARGS__)
#else
#define log_error(...)
#endif

#if LOG_LEVEL >= LOG_WARNING
#define log_warning(...) printf(__VA_ARGS__)
#else
#define log_warning(...)
#endif

#if LOG_LEVEL >= LOG_DEBUG
#define log_debug(...) printf(__VA_ARGS__)
#else
#define log_debug(...)
#endif

void log_init(void);

#else /* !LOG_ENABLE */

#define log_info(...)
#define log_warning(...)
#define log_error(...)
#define log_debug(...)
#define log_init()

#endif

#define log_func() log_debug("call %s\r\n", __FUNCTION__)

#endif /* __LOG_H__ */
