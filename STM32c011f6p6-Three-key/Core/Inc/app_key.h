#ifndef APP_KEY_H
#define APP_KEY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32c0xx_hal.h"

/**
 * @brief 按键ID
 */
typedef enum
{
  APP_KEY_ID_L = 0,
  APP_KEY_ID_M = 1,
  APP_KEY_ID_R = 2,
  APP_KEY_ID_MAX
} AppKeyId;

/**
 * @brief 按键事件类型
 */
typedef enum
{
  APP_KEY_EVT_SINGLE_CLICK = 0,
  APP_KEY_EVT_DOUBLE_CLICK = 1,
  APP_KEY_EVT_LONG_PRESS_1S = 2,
} AppKeyEventType;

/**
 * @brief 按键事件
 */
typedef struct
{
  AppKeyId id;
  AppKeyEventType type;
  uint32_t t_ms;          /* 事件产生时刻（HAL_GetTick） */
  uint16_t down_ms;       /* 本次按下持续时长（可用于调试/验证阈值） */
  uint16_t gap_ms;        /* 双击间隔（仅双击有效） */
} AppKeyEvent;

/**
 * @brief 初始化按键模块（绑定端口/引脚与参数）
 */
void AppKey_Init(void);

/**
 * @brief 1ms Tick，推进消抖与状态机（建议在主循环中每ms调用一次）
 * @param now_ms 当前毫秒时间（建议传 HAL_GetTick()）
 */
void AppKey_Tick1ms(uint32_t now_ms);

/**
 * @brief 取出一个按键事件（无事件返回 0）
 * @return 1：取到事件；0：无事件
 */
int AppKey_PopEvent(AppKeyEvent *evt);

/**
 * @brief 获取按键名称（L/M/R）
 */
const char *AppKey_Name(AppKeyId id);

/**
 * @brief GPIO_Pin（如 GPIO_PIN_0）转按键 ID；不匹配则返回 APP_KEY_ID_MAX
 */
AppKeyId AppKey_IdFromPin(uint16_t gpio_pin);

/**
 * @brief 引脚号转名称（用于唤醒日志）
 */
const char *AppKey_NameFromPin(uint16_t gpio_pin);

/**
 * @brief 事件类型短字符串（SINGLE/DOUBLE/LONG1S）
 */
const char *AppKey_EventTypeStr(AppKeyEventType t);

/**
 * @brief 是否允许 MCU 进入 STOP：按键计时依赖 SysTick，睡眠期间 tick 不走会卡死长按/双击窗口；
 *        任一键仍按下或处于双击等待窗内时返回 0。
 */
int AppKey_CanEnterStop(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_KEY_H */

