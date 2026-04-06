#ifndef APP_LED_H
#define APP_LED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief LED 指示：触发一次“快闪”（非阻塞）
 * @param now_ms 当前时间（HAL_GetTick）
 */
void AppLed_Flash(uint32_t now_ms);

/**
 * @brief 1ms Tick，用于自动熄灭
 * @param now_ms 当前时间（HAL_GetTick）
 */
void AppLed_Tick1ms(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_LED_H */

