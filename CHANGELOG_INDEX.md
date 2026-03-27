# CHANGELOG_INDEX

> 项目修改索引总表（按版本记录）。

| 版本号 | 日期 | 修改要点 | 关联文件 |
| --- | --- | --- | --- |
| v2.8.3-DeviceId-Flash | 2026-03-27 | 推进“对码流程骨架”：`device_id` 从常量改为运行态并落地 Flash 持久化（上电加载/异常回退/回写）；TX/RX 日志输出改为动态 ID。 | `reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`reference STM32c011f6p6-xl2400t/Core/Inc/main.h`、`docs/03-单向通信方案正式版_v1.0_2026-03-27.md` |
| v2.8.2-Link-Observe | 2026-03-27 | 强化单向通信可观测性：临时包加入 `seq/checksum/tag`；补充 `TX/RX` 统计、超时/跳号/坏包日志；形成《单向通信方案正式版 v1.0》并明确排除旧项目日夜/充放电逻辑。 | `reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`docs/02-开发进展_2026-03-27.md`、`docs/03-单向通信方案正式版_v1.0_2026-03-27.md` |
| v2.8.1-BuildFix | 2026-03-27 | 修复 `main.c` 编译错误：补充 `Light_Tick` 前置声明；调整 `g_light_mode/g_light_bright` 声明位置，消除未声明与静态声明冲突。 | `reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`reference STM32c011f6p6-xl2400t/Core/Inc/main.h`、`docs/02-开发进展_2026-03-27.md` |
| v2.8.0-ADC-Scan-Mode | 2026-03-26 | 引入 ADC 双通道扫描调试与夜间调试窗口日志；上电打印固件版本。 | `reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`reference STM32c011f6p6-xl2400t/Core/Inc/main.h` |

## 硬件映射备忘（XL2400T 接收端）

| 模块 | 信号 | MCU 引脚 |
| --- | --- | --- |
| `XL2400T`（接收端） | `RF_CSN` | `PA4` |
| `XL2400T`（接收端） | `RF_SCK` | `PA5` |
| `XL2400T`（接收端） | `RF_DATA` | `PA7` |

> 补充：`PA3` 已从原 `BOOST_EN` 改为 `TIM1_CH4`（`LED_DRV2_Pin`）作为第 2 路 PWM 输出。