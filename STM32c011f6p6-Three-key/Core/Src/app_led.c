#include "app_led.h"
#include "main.h"

#define APP_LED_FLASH_MS   (40u)

static uint8_t s_led_on = 0;
static uint32_t s_led_off_at = 0;

void AppLed_Flash(uint32_t now_ms)
{
  /* 触发快闪：点亮并刷新熄灭时间 */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  s_led_on = 1u;
  s_led_off_at = now_ms + APP_LED_FLASH_MS;
}

void AppLed_Tick1ms(uint32_t now_ms)
{
  if (!s_led_on) return;
  if ((int32_t)(now_ms - s_led_off_at) >= 0) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    s_led_on = 0u;
  }
}

