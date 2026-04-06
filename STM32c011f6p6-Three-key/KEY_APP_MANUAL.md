# 三按键（单击/双击/长按）+ STOP 唤醒 — 开发规格书与复刻手册

适用工程：`STM32c011f6p6-Three-key`  
MCU：**STM32C011F6P6**  
框架：**STM32 HAL**（`stm32c0xx_hal`）

**本文目标**：把硬件连接、软件模块、主循环顺序、低功耗与串口约定写清楚，使 **任何人按本手册即可在同等硬件上复刻当前程序行为**（含 STOP 睡眠与 EXTI 唤醒）。

> 配套：`SPECIFICATION.md` 为短规格摘要；版本字符串以 `Core/Inc/main.h` 中 **`FW_VERSION`** 为准（当前示例：`v1.1.0-ThreeKeyStopWake`）。

---

## 1. 功能目标

| 类别 | 说明 |
| --- | --- |
| **按键** | 3 路：L / M / R |
| **事件** | 单击（SINGLE）、双击（DOUBLE，双击优先）、长按 1s（LONG1S，到点立即触发） |
| **LED** | 任意有效按键事件后 **快闪 40ms**（非阻塞） |
| **串口** | `USART1` **115200 8N1**，可区分 **按键 + 事件类型**；上电打印固件版本 |
| **低功耗** | 空闲 **≥120ms** 可进入 **STOP**；三键 **上升沿 EXTI** 唤醒 MCU |
| **唤醒日志** | 从 STOP 唤醒后，在 **首个按键事件** 或 **再次入睡前** 打印精简 `[PWR]` 行（含键名；与唤醒键一致时含 `evt`） |

**重要分工**：

- **按键识别**完全依赖 **1ms 轮询 + 软件消抖 + 状态机**（读 GPIO 电平）。  
- **EXTI 仅用于**：从 STOP 唤醒 CPU + 记录“哪根线唤醒”；**不在 EXTI 里做 `DebugPrint`**（避免唤醒瞬间 SYSCLK 未切回 HSE 导致 UART 乱码，且避免与主线程 `HAL_UART_Transmit` 重入）。

---

## 2. 硬件映射（与 `main.h` 引脚名一致）

| 名称 | 端口/引脚 | GPIO 模式 | 上拉/下拉 | 逻辑 |
| --- | --- | --- | --- | --- |
| **L_KEY** | **PB7** | **GPIO_MODE_IT_RISING**（EXTI） | 下拉 | 按下 = 高(1)，松开 = 低(0) |
| **M_KEY** | **PA0** | **GPIO_MODE_IT_RISING**（EXTI） | 下拉 | 同上 |
| **R_KEY** | **PA1** | **GPIO_MODE_IT_RISING**（EXTI） | 下拉 | 同上 |
| **LED** | **PB6** | 推挽输出 | 无 | 高 = 亮 |
| **USART1_TX** | **PA11**（重映射） | 复用 AF1 | - | 见下文 SYSCFG |
| **USART1_RX** | **PA12**（重映射） | 复用 AF1 | - | 见下文 SYSCFG |

**USART1 引脚说明**（与 `stm32c0xx_hal_msp.c` 一致）：

- Cube/HAL 注释中常见写法为 `PA9 [PA11] -> TX`、`PA10 [PA12] -> RX`。  
- 本工程在 **`HAL_MspInit`** 中调用：  
  `HAL_SYSCFG_EnableRemap(SYSCFG_REMAP_PA11);`  
  `HAL_SYSCFG_EnableRemap(SYSCFG_REMAP_PA12);`  
  实际接线应使用 **PA11 = TX、PA12 = RX**（若未重映射则对应 PA9/PA10，**与本工程不符**）。

**NVIC（STOP 唤醒必需）**：

- **EXTI0_1_IRQn**：处理 **PA0、PA1**（M、R）。  
- **EXTI4_15_IRQn**：处理 **PB7**（L）。  
- 在 `MX_GPIO_Init` 末尾对以上 IRQ **使能**（优先级可按项目调整，当前为 0）。

---

## 3. 时钟与 STOP 恢复（复刻时必须一致）

- **系统时钟**：`main.c` 中 `SystemClock_Config()` 使用 **HSE** 作为 **SYSCLK**（与 Cube 生成配置一致）。  
- **从 STOP 唤醒后**：HAL 从 WFI 返回时，时钟处于芯片默认恢复状态；**必须再次调用**：
  1. `SystemClock_Config();`
  2. `MX_USART1_UART_Init();`  
  否则 **UART 波特率与当前 APB 时钟不匹配**，串口会出现乱码。

实现位置：`Core/Src/app_power.c` 中 `HAL_PWR_EnterSTOPMode` 返回之后。

---

## 4. 时间参数（与源码宏一致）

| 含义 | 源码位置 | 宏 / 常量 | 当前值 |
| --- | --- | --- | --- |
| 消抖 | `app_key.c` | `APP_KEY_DEBOUNCE_MS` | **20ms**（连续稳定 20ms 才承认按下/松开边沿） |
| 双击窗口 | `app_key.c` | `APP_KEY_DOUBLE_MS` | **200ms** |
| 长按阈值 | `app_key.c` | `APP_KEY_LONG_MS` | **1000ms** |
| LED 快闪 | `app_led.c` | `APP_LED_FLASH_MS` | **40ms** |
| 空闲进 STOP | `app_power.c` | `APP_POWER_IDLE_TO_STOP_MS` | **120ms** |

**调参**：修改上述宏后重新编译即可；若发布固件，请同步更新 `main.h` 的 **`FW_VERSION`** 与 `CHANGELOG_INDEX.md`（见 `PROJECT_RULES.md`）。

---

## 5. 事件定义（对外 API）

头文件：`Core/Inc/app_key.h`

- **`AppKeyId`**：`APP_KEY_ID_L` / `M` / `R`  
- **`AppKeyEventType`**：`SINGLE_CLICK` / `DOUBLE_CLICK` / `LONG_PRESS_1S`  
- **`AppKeyEvent`** 字段：`id`、`type`、`t_ms`（`HAL_GetTick()`）、`down_ms`、`gap_ms`（双击间隔）

**辅助函数**（供电源/日志使用）：

- `AppKey_Name(AppKeyId id)` → `"L"` / `"M"` / `"R"`  
- `AppKey_IdFromPin(uint16_t gpio_pin)`、`AppKey_NameFromPin(uint16_t gpio_pin)`  
- `AppKey_EventTypeStr(AppKeyEventType t)` → `"SINGLE"` / `"DOUBLE"` / `"LONG1S"`  
- **`int AppKey_CanEnterStop(void)`**：返回 **0** 表示 **当前不允许进 STOP**（见第 10 节）

---

## 6. 状态机规则（方案 B，与实现一致）

### 6.1 判定表

| 场景 | 条件 | 行为 |
| --- | --- | --- |
| **长按** | 稳定按住 ≥ 1000ms | 立即产生 LONG1S（与双击窗口无关） |
| **双击** | 第 1 次稳定松开后 **≤200ms** 内出现第 2 次稳定按下 | 在 **第 2 次按下被确认时** 立即产生 DOUBLE |
| **单击** | 第 1 次稳定松开后 **>200ms** 仍无第二次按下 | 窗口到期时产生 SINGLE（时间戳为第一次松开时刻相关逻辑） |

### 6.2 特殊规则：双击后长按无效（直到松手）

- 已触发 **DOUBLE** 后，若用户 **第二次按下仍不松手** 继续按住超过 1s，**不得**再产生 **LONG1S**。  
- 实现：变量 **`suppress_next_release`**（源码命名），在双击触发后置位，直到本次 **稳定松开** 清掉；长按检测需满足 `!suppress_next_release`。

### 6.3 内部变量（与 `app_key.c` 对应）

- `logic`：`KEY_ST_IDLE` / `KEY_ST_WAIT_SECOND`（等待第二次点击窗口）  
- `t_press_ms`：当前稳定按下时刻  
- `t_first_rel_ms`：第一次稳定松开时刻（双击窗口起点）  
- `long_fired`：长按是否已在本轮按下中产生过  
- `suppress_next_release`：双击已触发后的抑制标志  

消抖与边沿处理见第 7、8 节。

---

## 7. 软件模块与文件清单（复刻时按此组织）

| 路径 | 职责 |
| --- | --- |
| `Core/Inc/app_key.h`、`Core/Src/app_key.c` | 按键采样、消抖、状态机、事件队列 |
| `Core/Inc/app_led.h`、`Core/Src/app_led.c` | LED 非阻塞快闪 |
| `Core/Inc/app_debug.h`、`Core/Src/app_debug.c` | `DebugPrint()` → `HAL_UART_Transmit(&huart1, …)` |
| `Core/Inc/app_power.h`、`Core/Src/app_power.c` | 空闲进 STOP、唤醒后恢复时钟/UART、PWR 日志、`AppKey_CanEnterStop` 门禁 |
| `Core/Src/app_wakeup.c` | 实现 **`HAL_GPIO_EXTI_Rising_Callback`**：仅调用 `AppPower_OnKeyWake(GPIO_Pin)` |
| `Core/Src/main.c` | `main` 初始化顺序、主循环、`MX_GPIO_Init`、`MX_USART1_UART_Init`、`SystemClock_Config` |
| `Core/Src/stm32c0xx_it.c` | `EXTI0_1_IRQHandler` / `EXTI4_15_IRQHandler` → `HAL_GPIO_EXTI_IRQHandler` |
| `Core/Src/stm32c0xx_hal_msp.c` | USART1 GPIO、**PA11/PA12 重映射**、时钟 |

**注意**：STM32C0 HAL 使用 **`HAL_GPIO_EXTI_Rising_Callback`**（在 `HAL_GPIO_EXTI_IRQHandler` 内调用）。**不要**依赖已移除的 `HAL_GPIO_EXTI_Callback`（本工程未使用）。

---

## 8. 主循环顺序（必须严格一致）

位置：`main.c` 的 `while(1)`。

```text
1. now = HAL_GetTick()
2. 若 now 与上次不同（每 1ms 一次）：
     AppKey_Tick1ms(now)
     AppLed_Tick1ms(now)
3. did_work = 0
4. while (AppKey_PopEvent(&evt))：
     did_work = 1
     AppPower_OnKeyEventForWakeLog(&evt)   // 若有待输出的 STOP 唤醒日志，在此行合并打印
     按 evt.type 打印 [KEY] ...
     AppLed_Flash(now)
5. AppPower_Poll(now, did_work ? 0 : 1)
     idle_hint = 1 表示“本圈无按键业务”，允许考虑睡眠
     idle_hint = 0 表示“本圈有事件”，刷新活动时间
```

**`AppPower_Poll` 要点摘要**：

- `idle_hint==0`：更新 `s_last_activity_ms`，直接返回。  
- 若刚置位过唤醒标志 `s_wakeup_flag`：清标志并刷新活动时间，**本圈不进 STOP**。  
- 若距离上次活动 **<120ms**：返回。  
- 调用 **`AppKey_CanEnterStop()`**，若为 0：刷新 `s_last_activity_ms` 并返回（**禁止进 STOP**）。  
- 否则：必要时补打未消费的唤醒日志 → 清 EXTI pending → 打印 `[PWR] ->STOP` → `HAL_SuspendTick` → `HAL_PWR_EnterSTOPMode` → `HAL_ResumeTick` → `SystemClock_Config` → `MX_USART1_UART_Init` → 记录唤醒时长并置 **`s_wake_log_pending`**。

---

## 9. 为何需要 `AppKey_CanEnterStop()`（长按/双击与 STOP）

进入 STOP 前会 **`HAL_SuspendTick()`**，睡眠期间 **`HAL_GetTick()` 基本不增长**。  
而单击窗口、双击窗口、长按计时 **全部依赖 `HAL_GetTick()` 的连续递增**。

若在 **仍按住键** 或 **仍处于双击等待窗**（`KEY_ST_WAIT_SECOND`）时进入 STOP：

- 长按 **永远计不满 1s**；  
- 双击窗口 **停滞**；  
- 仅上升沿唤醒时，**按住不放可能没有新边沿**，行为异常。

**因此**：`AppKey_CanEnterStop()` 在以下任一成立时返回 **0**：

- 任一键 **`Key_ReadRaw` 为按下**（高电平）；  
- 任一键 `logic == KEY_ST_WAIT_SECOND`。

---

## 10. 低功耗与唤醒日志（`app_power.c`）

### 10.1 EXTI 回调（`app_wakeup.c`）

```c
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == M_KEY_Pin || GPIO_Pin == R_KEY_Pin || GPIO_Pin == L_KEY_Pin) {
    AppPower_OnKeyWake(GPIO_Pin);
  }
}
```

`AppPower_OnKeyWake` **仅**：`s_wakeup_flag = 1`，`s_last_wake_pin = pin`。**禁止在此处 `DebugPrint`。**

### 10.2 串口打印格式（固定前缀，便于脚本）

| 时机 | 示例格式 |
| --- | --- |
| 上电 | `[FW] v1.1.0-ThreeKeyStopWake`（以 `FW_VERSION` 为准） |
| 即将进 STOP | `[PWR] ->STOP` |
| 从 STOP 唤醒后，**首个** `PopEvent` 且与唤醒键一致 | `[PWR] WAKE key=L evt=SINGLE dur=<ms>`（`evt` 为 SINGLE/DOUBLE/LONG1S） |
| 从 STOP 唤醒后，首个事件 **非** 唤醒键 | 先 `[PWR] WAKE key=<唤醒键> dur=<ms>`，再照常 `[KEY] ...` |
| 唤醒后一直无按键事件，再次空闲满 120ms 将要睡 | 入睡前补打 `[PWR] WAKE key=<唤醒键> dur=<ms>`（无 `evt`） |

**`dur`**：近似为 **唤醒时刻 tick − 进入 STOP 前记录的 tick**（用于观察睡眠时长）。

---

## 11. 消抖实现（稳定计数法）

每个按键在 `app_key.c` 中维护：

- `raw_level` / `last_raw_level`  
- `stable_level`  
- `stable_cnt_ms`  

规则：

- raw 与上一拍相同 → `stable_cnt_ms++`  
- raw 变化 → 计数清零  
- 当 `stable_cnt_ms >= APP_KEY_DEBOUNCE_MS` 且 raw ≠ stable → 更新 `stable_level`，调用 `Key_OnStableEdge`  

**前提**：`AppKey_Tick1ms` **每毫秒**调用一次（由主循环用 `HAL_GetTick` 变化触发）。

---

## 12. 状态机伪代码（与代码变量名对齐）

### 12.1 稳定按下（stable 0→1）

- 记录 `t_press_ms = now`，`long_fired = 0`  
- 若 `logic == WAIT_SECOND`：  
  - `gap = now - t_first_rel_ms`  
  - `gap <= 200ms` → 发 DOUBLE，`suppress_next_release = 1`，`logic = IDLE`  
  - 否则 → 先发 SINGLE（上一击），`logic = IDLE`，本次按下作为新一轮  

### 12.2 稳定松开（stable 1→0）

- 若 `suppress_next_release`：清标志并 `logic = IDLE`，return  
- 若 `long_fired`：仅复位逻辑，return  
- 若 `logic == IDLE`：进入 `WAIT_SECOND`，`t_first_rel_ms = now`  

### 12.3 每 ms：长按

若 `stable_level==1` 且 `!long_fired` 且 `!suppress_next_release`：  
`now - t_press_ms >= 1000` → 发 LONG1S，`long_fired=1`，`logic=IDLE`

### 12.4 每 ms：单击窗口超时

若 `logic == WAIT_SECOND` 且 `now - t_first_rel_ms > 200ms` → 发 SINGLE，`logic=IDLE`

---

## 13. 串口事件行（按键）

每个事件一行（`main.c`）：

- 单击：`[KEY] <L|M|R> SINGLE t=<ms>`  
- 双击：`[KEY] <L|M|R> DOUBLE t=<ms> gap=<ms>`  
- 长按：`[KEY] <L|M|R> LONG1S t=<ms>`  

---

## 14. LED

- `AppLed_Flash(now)`：置位 LED，记录 `now + 40ms` 熄灭。  
- `AppLed_Tick1ms(now)`：到点熄灭。  

---

## 15. 复刻步骤清单（推荐顺序）

1. **硬件**：按第 2 节焊接/连接；确认 USART 使用 **PA11/PA12** 且与 **SYSCFG 重映射**一致。  
2. **CubeMX / 工程**：选择 STM32C011F6P6，配置 HSE、USART1、GPIO（三键 EXTI 上升沿 + 下拉，LED 输出）。  
3. **复制源码模块**：`app_key` / `app_led` / `app_debug` / `app_power` / `app_wakeup` 及 `main` 中 USER 段集成方式。  
4. **确认 `stm32c0xx_it.c`** 中 EXTI 向量与引脚线号一致。  
5. **确认 STOP 唤醒后** 调用 **`SystemClock_Config` + `MX_USART1_UART_Init`**。  
6. **主循环** 严格按第 8 节顺序实现。  
7. 上电验证 `[FW]`、按键三类事件、`[PWR]` 行为与 STOP 电流/唤醒。

---

## 16. 验证用例（测试）

| # | 操作 | 期望 |
| --- | --- | --- |
| 1 | 单击：按下—松开，等待 >200ms | 输出 `SINGLE`（相对松开最多约 200ms+ 延迟属正常） |
| 2 | 双击：第 1 次松开后 200ms 内第 2 次按下 | 第 2 次按下确认后输出 `DOUBLE` |
| 3 | 长按：按住 ≥1s | 约 1s 时输出 `LONG1S` |
| 4 | 双击后第 2 次按下不松手再满 1s | **仅** `DOUBLE`，**无** `LONG1S`，松手后下一轮才允许长按 |
| 5 | 空闲约 120ms 后 | 可看到 `[PWR] ->STOP`；按任意键唤醒 |
| 6 | 唤醒后 | 串口 **无** 乱码前缀；可按第 10.2 节出现 `[PWR] WAKE ...` |
| 7 | 按住不放等长按 | 全程 **不应** 在未满 1s 时因睡眠导致长按失效 |

---

## 17. 常见调参

- **易误判双击**：已用 **20ms** 消抖；仍可略增 `APP_KEY_DOUBLE_MS`（如 250~300ms），但单击确认会更慢。  
- **双击难触发**：略增 `APP_KEY_DOUBLE_MS`。  
- **仍误触**：继续略增 `APP_KEY_DEBOUNCE_MS`（如 25~30ms）。  
- **更早/更晚进 STOP**：改 `APP_POWER_IDLE_TO_STOP_MS`（注意与双击窗口、长按体验的平衡）。

---

## 18. 已知限制与约定

- 单击事件需等双击窗口结束才确认，故 **相对松开动作** 最多约有 **200ms 级** 延迟。  
- 触发 **LONG1S** 后，本次松开不再产生单击/双击。  
- **不要在 EXTI 回调里直接 `DebugPrint`**；唤醒后打印须在 **主线程** 且 **时钟与 UART 已重配** 之后。  
- 仅 **上升沿** 配置为 EXTI 唤醒时，**按住无新边沿** 期间依赖 **不再进 STOP**（`AppKey_CanEnterStop`）才能保证长按与双击窗口正确。

---

*文档版本：与固件 `FW_VERSION` 对齐维护；修改固件行为时请同步更新本节与 `CHANGELOG_INDEX.md`。*
