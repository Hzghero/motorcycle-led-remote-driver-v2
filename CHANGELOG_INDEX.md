# CHANGELOG_INDEX

> 项目修改索引总表（按版本记录）。

| 版本号 | 日期 | 修改要点 | 关联文件 |
| --- | --- | --- | --- |
| v2.9.9-TX-BootWarm-Burst | 2026-03-29 | TX 发包与按键策略收敛：按键事件短突发发送、上电保温窗口+首包增强、空闲RF睡眠；按键先收敛单击+长按，双击暂缓。 | `reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`reference STM32c011f6p6-xl2400t/Core/Inc/main.h`、`docs/05-会话历史摘要_2026-03-28.md` |
| v2.9.5-Doc-KeysOff-PwmSpec | 2026-03-28 | 仅文档修订：补充“熄灯后单击右键直入特效1”、特效快闪=2Hz，明确20kHz PWM/恒高规则；清单同步更新待实现项。 | `docs/01-功能说明书.md`、`docs/04-对齐功能说明书_执行清单_2026-03-28.md`、`CHANGELOG_INDEX.md` |
| v2.9.1-Checklist-AlignSpec | 2026-03-28 | 新增《对齐功能说明书执行清单》：用于后续逐项勾选完成并避免遗漏；索引总表新增快捷入口。 | `docs/04-对齐功能说明书_执行清单_2026-03-28.md`、`CHANGELOG_INDEX.md` |
| v2.8.4-Pairing-Formal | 2026-03-27 | 正式对码流程上线：RX 长按进入对码模式，捕获首个合法 `device_id` 并写入 Flash，支持超时退出；详见《单向通信方案正式版》P2。 | `reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`reference STM32c011f6p6-xl2400t/Core/Inc/main.h`、`docs/03-单向通信方案正式版_v1.0_2026-03-27.md` |
| v2.9.0-DevId16 | 2026-03-27 | 设备 ID 升级到 16-bit：RF 负载 8→9 字节（ID 低/高字节）；Flash 持久化结构同步升级并接入 MCU UID 派生默认 ID；日志/对码入口改为 16-bit 显示。详见《单向通信方案正式版 v1.0》P3 小节。 | `reference STM32c011f6p6-xl2400t/Core/Inc/XL2400T.h`、`reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`reference STM32c011f6p6-xl2400t/Core/Inc/main.h`、`docs/03-单向通信方案正式版_v1.0_2026-03-27.md` |
| v2.8.3-DeviceId-Flash | 2026-03-27 | 推进“对码流程骨架”：`device_id` 从常量改为运行态并落地 Flash 持久化（上电加载/异常回退/回写）；TX/RX 日志输出改为动态 ID。 | `reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`reference STM32c011f6p6-xl2400t/Core/Inc/main.h`、`docs/03-单向通信方案正式版_v1.0_2026-03-27.md` |
| v2.8.2-Link-Observe | 2026-03-27 | 强化单向通信可观测性：临时包加入 `seq/checksum/tag`；补充 `TX/RX` 统计、超时/跳号/坏包日志；形成《单向通信方案正式版 v1.0》并明确排除旧项目日夜/充放电逻辑。 | `reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`docs/02-开发进展_2026-03-27.md`、`docs/03-单向通信方案正式版_v1.0_2026-03-27.md` |
| v2.8.1-BuildFix | 2026-03-27 | 修复 `main.c` 编译错误：补充 `Light_Tick` 前置声明；调整 `g_light_mode/g_light_bright` 声明位置，消除未声明与静态声明冲突。 | `reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`reference STM32c011f6p6-xl2400t/Core/Inc/main.h`、`docs/02-开发进展_2026-03-27.md` |
| v2.8.0-ADC-Scan-Mode | 2026-03-26 | 引入 ADC 双通道扫描调试与夜间调试窗口日志；上电打印固件版本。 | `reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`reference STM32c011f6p6-xl2400t/Core/Inc/main.h` |

## 快速指引

- 对齐功能说明书执行清单（持续更新、完成后打勾）：`docs/04-对齐功能说明书_执行清单_2026-03-28.md`

## 硬件映射备忘（XL2400T 接收端）

| 模块 | 信号 | MCU 引脚 |
| --- | --- | --- |
| `XL2400T`（接收端） | `RF_CSN` | `PA4` |
| `XL2400T`（接收端） | `RF_SCK` | `PA5` |
| `XL2400T`（接收端） | `RF_DATA` | `PA7` |

> 补充：`PA3` 已从原 `BOOST_EN` 改为 `TIM1_CH4`（`LED_DRV2_Pin`）作为第 2 路 PWM 输出。
