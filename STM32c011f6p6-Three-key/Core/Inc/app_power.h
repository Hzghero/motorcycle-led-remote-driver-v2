#ifndef APP_POWER_H
#define APP_POWER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_key.h"

/**
 * @brief 低功耗模块初始化
 */
void AppPower_Init(void);

/**
 * @brief 记录一次按键唤醒事件（在 EXTI 回调中调用）
 * @param pin 触发唤醒的 GPIO_Pin（例如 GPIO_PIN_0）
 */
void AppPower_OnKeyWake(uint16_t pin);

/**
 * @brief 主循环中调用：空闲时进入 STOP（由按键 EXTI 唤醒）
 * @param now_ms 当前时间（HAL_GetTick）
 * @param idle_hint 1=当前无业务要做（可睡眠）；0=仍忙（不睡）
 */
void AppPower_Poll(uint32_t now_ms, int idle_hint);

/**
 * @brief 获取并清除上次唤醒引脚（0 表示无）
 */
uint16_t AppPower_TakeLastWakePin(void);

/**
 * @brief 主循环取出按键事件后调用：若有待打印的 STOP 唤醒日志，则输出一行（含键名；若与本次事件同一键则含 evt）
 */
void AppPower_OnKeyEventForWakeLog(const AppKeyEvent *evt);

#ifdef __cplusplus
}
#endif

#endif /* APP_POWER_H */

