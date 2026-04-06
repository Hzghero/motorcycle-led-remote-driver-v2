#ifndef APP_DEBUG_H
#define APP_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>

/**
 * @brief 串口调试打印（默认使用 USART1 的 huart1，阻塞发送）
 */
void DebugPrint(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* APP_DEBUG_H */

