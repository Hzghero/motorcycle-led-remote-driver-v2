# 版本变更索引

| 版本号 | 日期 | 修改要点 | 关联文件 |
| --- | --- | --- | --- |
| v1.0.0-ThreeKey | 2026-03-31 | 增加三按键单击/200ms双击/长按1s识别，串口打印事件，LED有效事件快闪 | Core/Inc/main.h, Core/Src/main.c, Core/Inc/app_key.h, Core/Src/app_key.c, Core/Inc/app_debug.h, Core/Src/app_debug.c, Core/Inc/app_led.h, Core/Src/app_led.c, SPECIFICATION.md |
| v1.0.1-ThreeKey | 2026-03-31 | 按键消抖改为10ms；双击改为“第二次按下即触发”（双击优先），单击为双击窗口超时确认 | Core/Inc/main.h, Core/Src/app_key.c, SPECIFICATION.md |
| v1.0.2-ThreeKey | 2026-03-31 | 修复：双击触发后第二次按下不松手时不再误触发长按（直到松手） | Core/Inc/main.h, Core/Src/app_key.c, SPECIFICATION.md, KEY_APP_MANUAL.md |
| v1.1.0-ThreeKeyStopWake | 2026-03-31 | 增加 STOP 低功耗与按键 EXTI 上升沿唤醒（三键均可唤醒） | Core/Src/main.c, Core/Src/stm32c0xx_it.c, Core/Inc/stm32c0xx_it.h, Core/Src/app_power.c, Core/Inc/app_power.h, Core/Src/app_wakeup.c, MDK-ARM/STM32c011f6p6-Three-key.uvprojx |
