#include "app_power.h"
#include "main.h"

#include "stm32c0xx_hal.h"

/* 说明：
 * HAL 的 EXTI 回调是 weak 的，放在任意编译单元实现即可。
 * 这里专门集中处理“按键唤醒标记”，避免把逻辑塞进中断里。
 */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == M_KEY_Pin || GPIO_Pin == R_KEY_Pin || GPIO_Pin == L_KEY_Pin) {
    AppPower_OnKeyWake(GPIO_Pin);
  }
}

