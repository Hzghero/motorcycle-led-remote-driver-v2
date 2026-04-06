# XL2400T + STM32C011 Standby 低功耗：心得与易踩坑清单

> 目标：把“能复现、能解释、能验证”的关键结论固化下来，避免后续重复踩坑。  
> 场景：STM32C011F6P6（3.3V 供电）+ XL2400T（同 3.3V 供电，3 线软件 SPI：CSN/SCK/DATA）。

---

## 1. 结论速览（最重要的 3 条）

1. **`RF_Link_Sleep()` 本身有效，但不等于 Standby 后仍保持睡眠。**  
   实测：进入 Standby 前的 WFI 等待窗口内，XL2400T 可进入极低电流；但一旦 MCU 进入 Standby，RF 电流可能立刻回升（被“线状态变化/悬空”唤醒）。

2. **进入 Standby 后，GPIO 不一定保持“推挽输出态”。**  
   若 RF 的 CSN/SCK/DATA 在 Standby 期间变为高阻或电平漂移，XL2400T 可能被异常边沿/毛刺唤醒，出现“Standby 反而更耗电”的假象。

3. **解决关键：用 PWR 的 Standby Pull-up / Pull-down 保持 RF 三线电平。**  
   - CSN：上拉保持高（片选无效）  
   - SCK/DATA：下拉保持低（避免时钟/数据毛刺）  
   - 并启用 APC：`HAL_PWREx_EnablePullUpPullDownConfig()`

---

## 2. 经典复现场景（强烈推荐作为验收用例）

### 2.1 现象（“很经典”）

- 欠压触发后先让 RF 睡眠，并在进入 Standby 前等待 \(N\) 秒（WFI）用于观察电流
- **在等待窗口内电流接近 0/极低**（说明 RF 已睡）
- **但进入 Standby 后电流立刻上升**（说明 RF 被唤醒/未保持睡眠）

### 2.2 排查要点（按优先级）

- **先确认 RF 睡眠指令是否真正写入**：建议在下发睡眠后回读 `CFG_TOP`，打印确认（不要只靠“软件状态变量”判断）。
- **再确认 Standby 后引脚是否保持**：如果 Standby 后电流上升，优先怀疑引脚保持问题，而不是 RTC/Standby 本身。

---

## 3. 代码侧关键策略（本项目落地）

### 3.1 RF 睡眠动作要“无条件”

- 不要过度依赖 `g_rf_sleeping` 之类的软件状态，**低功耗入口处直接无条件下发 `RF_Link_Sleep()`** 更稳妥。
- 同时把软件 SPI 三线设为静态电平：CSN=High、SCK=Low、DATA=Low。

### 3.2 Standby 期间用 PWR 维持 IO 上拉/下拉（核心）

- 通过 HAL PWREx API 在 Standby 期间维持：
  - `PA4(RF_CSN)` 上拉
  - `PA5(RF_SCK)` 下拉
  - `PA7(RF_DATA)` 下拉
- 开启 APC 使配置在 Standby 生效：`HAL_PWREx_EnablePullUpPullDownConfig()`

---

## 4. 与 Standby 退出机制相关的坑（补充）

- **Standby 唤醒=复位启动**：唤醒后一定会重新跑初始化逻辑，别指望 RAM 里状态还在。
- **Standby 期间 RTC 会停机**：不要依赖 `RTC Alarm A` 做“定时唤醒”；Standby exit 主要依赖 `WKUP` 引脚事件（或 `NRST`/`IWDG` 复位、`LSE` 故障 CSS 等）。
  正确做法：需要周期性唤醒就用 `STOP + RTC Alarm`；需要更低静态功耗才用 `STANDBY + WKUP`。

---

## 5. 快速验收清单（你可以直接照着测）

- **功能**：
  - 欠压触发 → 打印进入低功耗路径
  - Standby → `WKUP1(PA0)`（太阳能电压上升）唤醒复位 → 再次打印启动信息
- **电流**：
  - RF 睡眠等待窗口内：电流应显著下降（接近极低）
  - `enter STANDBY now` 后：电流应保持在极低（例如 µA 级），不应“突然回升”

---

## 6. 实验数据（Proof 对照测量）

你在同一工况下、测量的是 **XL2400T 模块电流**，并且通过改变 `UV_RF_PROOF_MODE` 得到三组关键结果：

1. **9 秒窗口（wait RF sleep settle ... 之后、尚未进入 Standby）**
   - `UV_RF_PROOF_MODE=1`（`HAL_Delay` 忙等）：约 `190 uA`
   - `UV_RF_PROOF_MODE=2`（`WFI(SLEEP)`）：约 `160 uA`

2. **9 秒结束点（紧接下一条 `RTC arm ...` 之前/之后）**
   - `UV_RF_PROOF_MODE=1`：约 `190 uA`
   - `UV_RF_PROOF_MODE=2`：约 `160 uA`

3. **进入 Standby（打印 `enter STANDBY now` 之后）**
   - `UV_RF_PROOF_MODE=1/2`：两种模式都约 `1.6 uA`

**结论**：浅睡眠阶段仍存在残余偏置/IO 耦合，使 RF 电流仍在百 uA 量级；而“真正完成 Standby 进入 + Standby 期间 RF 三线电平被 PWR 强制保持”之后，RF 才进入最终极低电流态。

---

## 7. 今日补充（STOP 回归验证）

你最新日志显示：
- 上电后正常初始化 RF、同步逻辑正常；
- 欠压后进入 `STOP` 分支：`[UV] enter STOP ...`；
- 每个周期都有 `RTC arm currSec=... nextSec=...`；
- 每次都能 `flashPD ON -> flashPD OFF -> wake from STOP` 循环。

这说明当前 `STOP + RTC + Flash power-down` 主链路是通的。  
即使“静态电流仍高于 Standby 方案”，从工程可用性与可唤醒稳定性角度看，已经达到可持续迭代的稳定基线。

