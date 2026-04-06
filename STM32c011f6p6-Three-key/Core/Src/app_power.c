#include "app_power.h"
#include "main.h"
#include "app_debug.h"

#include "stm32c0xx_hal.h"

#define APP_POWER_IDLE_TO_STOP_MS   (120u)

static volatile uint8_t s_wakeup_flag = 0;
static volatile uint16_t s_last_wake_pin = 0;
static uint32_t s_last_activity_ms = 0;

/* STOP 唤醒后待打印一行 PWR 日志（等首个按键事件合并 evt，或进睡前回退打印） */
static uint8_t s_wake_log_pending = 0;
static uint32_t s_wake_dur_ms = 0;
static uint32_t s_enter_stop_ms = 0;

static void AppPower_FlushWakeLogNoEvent(void)
{
  if (!s_wake_log_pending) {
    return;
  }
  DebugPrint("[PWR] WAKE key=%s dur=%lums\r\n",
             AppKey_NameFromPin(s_last_wake_pin),
             (unsigned long)s_wake_dur_ms);
  s_wake_log_pending = 0u;
}

void AppPower_Init(void)
{
  s_wakeup_flag = 0;
  s_last_wake_pin = 0;
  s_last_activity_ms = HAL_GetTick();
  s_wake_log_pending = 0u;
  s_wake_dur_ms = 0u;
}

void AppPower_OnKeyWake(uint16_t pin)
{
  s_wakeup_flag = 1u;
  s_last_wake_pin = pin;
}

uint16_t AppPower_TakeLastWakePin(void)
{
  uint16_t p = s_last_wake_pin;
  s_last_wake_pin = 0;
  return p;
}

extern void SystemClock_Config(void);

void AppPower_OnKeyEventForWakeLog(const AppKeyEvent *evt)
{
  if (evt == NULL || !s_wake_log_pending) {
    return;
  }

  if (AppKey_IdFromPin(s_last_wake_pin) == evt->id) {
    DebugPrint("[PWR] WAKE key=%s evt=%s dur=%lums\r\n",
               AppKey_Name(evt->id),
               AppKey_EventTypeStr(evt->type),
               (unsigned long)s_wake_dur_ms);
  } else {
    DebugPrint("[PWR] WAKE key=%s dur=%lums\r\n",
               AppKey_NameFromPin(s_last_wake_pin),
               (unsigned long)s_wake_dur_ms);
  }
  s_wake_log_pending = 0u;
}

void AppPower_Poll(uint32_t now_ms, int idle_hint)
{
  if (!idle_hint) {
    s_last_activity_ms = now_ms;
    return;
  }

  if (s_wakeup_flag) {
    s_wakeup_flag = 0u;
    s_last_activity_ms = now_ms;
    return;
  }

  if ((uint32_t)(now_ms - s_last_activity_ms) < APP_POWER_IDLE_TO_STOP_MS) {
    return;
  }

  /* 长按/双击窗口依赖 HAL_GetTick；STOP 期间 SysTick 停表，不能睡 */
  if (!AppKey_CanEnterStop()) {
    s_last_activity_ms = now_ms;
    return;
  }

  /* 若上次 STOP 唤醒后尚未打出日志（无按键事件就再次空闲），补打一行 */
  AppPower_FlushWakeLogNoEvent();

  __HAL_GPIO_EXTI_CLEAR_IT(M_KEY_Pin);
  __HAL_GPIO_EXTI_CLEAR_IT(R_KEY_Pin);
  __HAL_GPIO_EXTI_CLEAR_IT(L_KEY_Pin);

  DebugPrint("[PWR] ->STOP\r\n");

  s_enter_stop_ms = HAL_GetTick();
  HAL_SuspendTick();
  HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
  HAL_ResumeTick();

  SystemClock_Config();
  MX_USART1_UART_Init();

  {
    uint32_t wake_ms = HAL_GetTick();
    uint32_t sleep_dur = wake_ms - s_enter_stop_ms;
    s_wake_dur_ms = sleep_dur;
    s_wake_log_pending = 1u;
    s_last_activity_ms = wake_ms;
  }
}
