# CHANGELOG_INDEX

> 项目修改索引总表（按版本记录）。

| 版本号 | 日期 | 修改要点 | 关联文件 |
| --- | --- | --- | --- |
| v2.10.17-LPNoSleepWhilePress | 2026-04-02 | 低功耗长按稳定性增强：新增“按键按下期间禁止进入低功耗”策略，并提高唤醒保护/按键保持窗口（1.0s/1.4s），避免长按过程中被 STOP 打断导致漏触发。 | `reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`reference STM32c011f6p6-xl2400t/Core/Inc/main.h`、`CHANGELOG_INDEX.md` |
| v2.10.16-LPWakeGuard-KeyHold | 2026-04-02 | 低功耗抗漏按优化（方向A）：新增“唤醒保护窗口 + 按键活动保持窗口”，防止 TX 从 STOP 唤醒后快速回睡切碎短按/双击，提升遥控触发稳定性。 | `reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`reference STM32c011f6p6-xl2400t/Core/Inc/main.h`、`CHANGELOG_INDEX.md` |
| v2.10.15-LPWakeGuard-KeyCapture | 2026-04-02 | 修复低功耗后 TX 业务恢复：在空闲低功耗唤醒后，若进入业务繁忙态则先强制恢复 RF 到 TX 配置（`RF_Link_ConfigTx`），避免“已唤醒但未发包”导致遥控失效。 | `reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`reference STM32c011f6p6-xl2400t/Core/Inc/main.h`、`CHANGELOG_INDEX.md` |
| v2.10.14-RoleFlowUnify-LP | 2026-04-02 | 统一主流程：业务收发不再依赖 APP_RF_LINK_TEST 分支，TX/RX 在正常模式下也执行完整收发；TX 空闲时继续进入低功耗；新增启动配置横幅 `[CFG] role=... link_test=... lowpower=...` 避免固件身份混淆。 | `reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`reference STM32c011f6p6-xl2400t/Core/Inc/main.h`、`CHANGELOG_INDEX.md` |
| v2.10.13-CfgBanner-MacroCentral | 2026-04-02 | 将关键宏收敛到 main.c 头部“宏配置总区”（角色/链路测试/低功耗/诊断），并在启动时新增 `[CFG] role=... link_test=... lowpower=...` 统一配置打印，避免烧录后模式混淆。 | `reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`reference STM32c011f6p6-xl2400t/Core/Inc/main.h`、`CHANGELOG_INDEX.md` |
| v2.10.14-RoleFlowUnify-LP | 2026-04-02 | 统一主流程：业务收发不再依赖 APP_RF_LINK_TEST 分支，TX/RX 在正常模式下也执行完整收发；TX 空闲时继续进入低功耗；新增启动配置横幅 `[CFG] role=... link_test=... lowpower=...` 避免固件身份混淆。 | `reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`reference STM32c011f6p6-xl2400t/Core/Inc/main.h`、`CHANGELOG_INDEX.md` |
| v2.10.17-LPNoSleepWhilePress | 2026-04-02 | 低功耗长按稳定性增强：新增“按键按下期间禁止进入低功耗”策略，并提高唤醒保护/按键保持窗口（1.0s/1.4s），避免长按过程中被 STOP 打断导致漏触发。 | `reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`reference STM32c011f6p6-xl2400t/Core/Inc/main.h`、`CHANGELOG_INDEX.md` |
| v2.10.16-LPWakeGuard-KeyHold | 2026-04-02 | 低功耗抗漏按优化（方向A）：新增“唤醒保护窗口 + 按键活动保持窗口”，防止 TX 从 STOP 唤醒后快速回睡切碎短按/双击，提升遥控触发稳定性。 | `reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`reference STM32c011f6p6-xl2400t/Core/Inc/main.h`、`CHANGELOG_INDEX.md` |
| v2.10.12-CleanupLegacySolar-IdleStop | 2026-04-02 | 清理太阳能项目残留：移除白天/夜间与充电相关变量及分支，TX 低功耗触发改为“业务空闲进入 STOP/SLEEP”，不再依赖昼夜状态机；保留 TX/RX 宏切换与对码链路。 | `reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`reference STM32c011f6p6-xl2400t/Core/Inc/main.h`、`CHANGELOG_INDEX.md` |
| v2.10.11-TxStopDiag-RfPinWakeTrace | 2026-04-02 | 增加低功耗诊断模式：进入低功耗前打印 RF 三线电平（CSN/SCK/DATA）并记录进入计数，唤醒后打印 PWR 唤醒标志（WUFI/WUF1/WUF2）与唤醒计数，用于确认 STOP 驻留情况与定位高功耗来源。 | `reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`reference STM32c011f6p6-xl2400t/Core/Inc/main.h`、`CHANGELOG_INDEX.md` |
| v2.10.10-TxStopLowPower-Step2 | 2026-04-02 | TX 低功耗第二步：针对 STOP 电流仍偏高问题，进入 STOP 前暂停 SysTick 与 TIM14 中断，唤醒后恢复时钟、恢复 Tick 与 TIM14，再恢复串口，减少高频唤醒造成的电流抬升。 | `reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`reference STM32c011f6p6-xl2400t/Core/Inc/main.h`、`CHANGELOG_INDEX.md` |
| v2.10.9-TxStopLowPower-Step1 | 2026-04-02 | TX 低功耗第一步：新增 SLEEP/STOP 可切换策略，默认启用 STOP；进入低功耗前无条件下发 RF 睡眠并固定 RF 三线静态电平（CSN高/SCK低/DATA低），唤醒后统一恢复时钟与串口。 | `reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`reference STM32c011f6p6-xl2400t/Core/Inc/main.h`、`CHANGELOG_INDEX.md` |
| v2.10.8-KeyLatencyFix-RfTxPoll-UartBudget | 2026-04-02 | 按键响应卡顿优化：RF 发送后等待改为“可切换 固定延时/快速轮询”策略（默认快速轮询），并为串口命令轮询加入“单轮字节预算+回显开关”，降低主循环阻塞导致的按键漏触发风险。 | `reference STM32c011f6p6-xl2400t/Core/Src/XL2400T.c`、`reference STM32c011f6p6-xl2400t/Core/Inc/XL2400T.h`、`reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`reference STM32c011f6p6-xl2400t/Core/Inc/main.h`、`docs/05-会话历史摘要_2026-04-01_全局PWM与MOS重构.md`、`CHANGELOG_INDEX.md` |
| v2.10.7-TxKeyDouble200-RfChGuard | 2026-04-01 | TX 三键恢复双击识别并将双击窗口收敛到 200ms（与功能说明书一致），三键双击均接入熄灯触发链路；同时新增 RF 频道强约束说明，明确 TX=76/RX=75 必须错开，避免同频道导致收包异常。 | `reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`reference STM32c011f6p6-xl2400t/Core/Inc/main.h`、`docs/KEY_APP_MANUAL.md`、`CHANGELOG_INDEX.md` |
| v2.10.6-GlobalPwmMos-IORefactor | 2026-04-01 | 完成双灯头控制重构：PA2 统一为 PWM_GLOBAL，PA3/PA6/PA8/PA10 作为四路 MOS 通道；RX 灯效改为“全局 PWM + 通道位图”；新增 PWM 强制高/低电平调试开关。台架实测：4 路 MOS 与 PWM 均符合预期。 | `reference STM32c011f6p6-xl2400t/Core/Src/main.c`、`reference STM32c011f6p6-xl2400t/Core/Inc/main.h`、`docs/01-功能说明书.md`、`docs/04-双灯头全局PWM与MOS硬开关功能说明_v1.0_2026-04-01.md`、`docs/06-台架测试记录模板_双灯头全局PWM与MOS_v2.10.6.md`、`docs/05-会话历史摘要_2026-03-28.md` |
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

> 2026-04-01 补充（双灯头全局PWM + MOS 架构）：
> `PA2` = `PWM_GLOBAL`（TIM1_CH3，全局亮度/模拟调光）；
> `PA3` = `MOS_L_HI`；`PA6` = `MOS_L_LO`；`PA8` = `MOS_R_HI`；
> `PA10` = `MOS_R_LO`；`PB6` = `LED`（状态指示）。
