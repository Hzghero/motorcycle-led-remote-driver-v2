#include "app_debug.h"
#include "main.h"

#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

/* 兼容某些工具链/库配置下未暴露 vsnprintf 声明的情况 */
#ifndef __VSNPRINTF_DECLARED
int vsnprintf(char *s, size_t n, const char *format, va_list arg);
#endif

void DebugPrint(const char *fmt, ...)
{
  char buf[128];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (n <= 0) return;

  size_t len = strlen(buf);
  HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 100);
}

