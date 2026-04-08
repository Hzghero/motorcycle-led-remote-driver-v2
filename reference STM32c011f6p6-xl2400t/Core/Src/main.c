/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "XL2400T.h"
#include "rf_xl2400.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32c0xx_hal_flash.h"
#include "stm32c0xx_hal_flash_ex.h"

/* 中文注释：
   - XL2400T.h 与 rf_xl2400.h 提供 2.4GHz RF 模块初始化、发送、接收接口
   - 采用 900ms 周期同步协议：0-450ms 发送同步包，450-900ms 接收同步包
   - LED 指示：PB6 (LED_Pin) 同步事件指示 + PA2 (PWM_GLOBAL) 全局 PWM 驱动
*/
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ============================ 宏配置总区（集中管理） ============================
 * 说明：
 * 1) 先看“角色与构建模式”决定当前固件身份（TX/RX + 是否链路测试）；
 * 2) 再看“低功耗与调试”决定运行策略；
 * 3) 其余为协议与时序参数。 */

/* [A] 角色与构建模式（最关键）
 * - 正常发射端固件：APP_RF_LINK_TEST=0, APP_ROLE_TX_ONLY=1, APP_ROLE_RX_ONLY=0
 * - 正常接收端固件：APP_RF_LINK_TEST=0, APP_ROLE_TX_ONLY=0, APP_ROLE_RX_ONLY=1 */
#define APP_RF_LINK_TEST            1U  /* 1=开启链路诊断附加打印，0=关闭附加打印（不影响业务主流程） */
#define APP_ROLE_RX_ONLY            0U  /* 1=接收端固件 */
#define APP_ROLE_TX_ONLY            1U  /* 1=发射端固件 */

/* [A-2] RX 三线输入复用映射（仅 RX 角色生效）
 * 说明：同一工程内复用 TX 三键 IO 口给 RX 车辆输入，靠角色宏区分避免冲突。 */
#define RX_ACC_IN_PORT              L_KEY_GPIO_Port
#define RX_ACC_IN_PIN               L_KEY_Pin
#define RX_CAR_HI_IN_PORT           M_KEY_GPIO_Port
#define RX_CAR_HI_IN_PIN            M_KEY_Pin
#define RX_HORN_IN_PORT             R_KEY_GPIO_Port
#define RX_HORN_IN_PIN              R_KEY_Pin

/* RX 三线输入极性切换：
 * - NORMAL: 高电平有效（保持当前行为）
 * - INVERTED: 低电平有效（整体反向） */
#define RX_WIRE_ACTIVE_POLARITY_NORMAL    0U
#define RX_WIRE_ACTIVE_POLARITY_INVERTED  1U
#define RX_WIRE_ACTIVE_POLARITY           RX_WIRE_ACTIVE_POLARITY_INVERTED 

/* 三线防抖时间（单位ms）
 * 当前定标：50ms（用于提升车辆线束抖动/干扰场景下的稳定性）。
 * 若需更灵敏响应可回调到 20ms，但应重新做台架防抖与误触发验证。 */
#define RX_WIRE_DEBOUNCE_MS               50U

/* 黄线首次生效门限：首次需“双击”后才放开黄线控制（单位ms） */
#define RX_HI_FIRST_DBL_MS                600U

#if (RX_WIRE_ACTIVE_POLARITY == RX_WIRE_ACTIVE_POLARITY_INVERTED)
#define RX_IN_ACTIVE_LEVEL          GPIO_PIN_RESET
#define RX_WIRE_POLARITY_NAME       "INVERTED"
#else
#define RX_IN_ACTIVE_LEVEL          GPIO_PIN_SET
#define RX_WIRE_POLARITY_NAME       "NORMAL"
#endif

/* [B] 低功耗与诊断 */
#define TX_LOWPOWER_MODE_SLEEP      0U
#define TX_LOWPOWER_MODE_STOP       1U
#define TX_LOWPOWER_MODE_STANDBY    2U
#define TX_LOWPOWER_MODE            TX_LOWPOWER_MODE_STANDBY /* TX低功耗模式：SLEEP/STOP/STANDBY（当前测试STANDBY） */
#define LOWPOWER_DIAG_ENABLE        1U  /* 1=低功耗诊断打印，0=关闭诊断打印 */

/* [C] RF 基本参数
 * RF 频道强约束：TX=76 / RX=75 必须错开一档，严禁同频道。 */
#define RF_TX_CHANNEL               76U /* XL2400T TX 频道 76 (2476 MHz) */
#define RF_RX_CHANNEL               75U /* XL2400T RX 频道 75 (2475 MHz) */

/* [D] 旧同步/时序协议参数（历史保留）
 * 说明：以下 SYNC_* 为旧“太阳能同步闪灯”模块遗留参数。
 * 当前三键遥控 + RX三线仲裁主流程不依赖该模块，保留仅用于历史回溯与对比调试。
 * 如后续确认不再需要，再整体移除该模块（函数+宏）而不是只删单个宏。 */
#define SYNC_CYCLE_MS               900U    /* 旧同步周期 900ms */
#define SYNC_LED_ON_MS              100U    /* 旧同步LED亮灯时间 100ms */
#define SYNC_TX_TIME_MS             450U    /* 旧同步TX发送时间：所有节点固定 450ms */
#define SYNC_TX_DELAY_MS            6U      /* 旧同步传输延迟补偿 */
#define SYNC_PKT_SIZE               4U      /* 旧同步包：AA 55 + 2字节相位 */

/* 仅允许启用一个角色 */
#if (APP_ROLE_RX_ONLY + APP_ROLE_TX_ONLY) != 1
#error "APP_ROLE_RX_ONLY / APP_ROLE_TX_ONLY must enable exactly one role"
#endif

/* 临时灯光控制包（先测收发/解析，不上真正业务字段复杂度） */
/* 协议（8字节负载）：
 * byte0=0xAA, byte1=0x55  : 帧头
 * byte2=mode(1~7)        : 灯光模式号
 * byte3=bright(0~2)     : 亮度档位（0=30%, 1=70%, 2=100%）
 * byte4..byte7=0        : 预留（先不做 ID/校验）
 */
#define TLIGHT_PKT_HEADER0 0xAA
#define TLIGHT_PKT_HEADER1 0x55
#define TLIGHT_MODE_OFF     0U
#define TLIGHT_MODE_MIN     1U
#define TLIGHT_MODE_MAX     7U

#define TLIGHT_BRIGHT_30    0U
#define TLIGHT_BRIGHT_70    1U
#define TLIGHT_BRIGHT_100   2U

/* 临时协议扩展：加入按键来源/触发类型与序号，便于联调观察 */
#define TLIGHT_PKT_KEYTRIG_IDX    4U  /* bit7..6=key, bit5..4=trig, bit3..0=seq4 */
#define TLIGHT_PKT_DEV_ID_L_IDX   5U
#define TLIGHT_PKT_DEV_ID_H_IDX   6U
#define TLIGHT_PKT_CHKSUM_IDX     7U
#define TLIGHT_PKT_TAG_IDX        8U
#define TLIGHT_PKT_TAG_CONST      0x5AU

#define TLIGHT_KEY_NONE           3U
#define TLIGHT_KEY_L              0U
#define TLIGHT_KEY_M              1U
#define TLIGHT_KEY_R              2U

#define TLIGHT_TRIG_NONE          0U
#define TLIGHT_TRIG_SINGLE        1U
#define TLIGHT_TRIG_DOUBLE        2U
#define TLIGHT_TRIG_LONG          3U

/* 对码专用包标记（方案1）：仅在 TX 进入对码模式时发送；RX 对码窗口内仅接受该标记触发配对。 */
#define TLIGHT_PAIR_KEY_MARK      TLIGHT_KEY_NONE
#define TLIGHT_PAIR_TRIG_MARK     TLIGHT_TRIG_LONG

/* P1：设备ID（单向链路唯一性）
 * - TX 发送自己的设备ID
 * - RX 仅接收与本机已配对ID一致的数据包
 * 当前 8 字节包先使用 8-bit ID；后续若扩展 16-bit，可在 v1.1 增长包长度。 */
#define TLIGHT_DEVICE_ID_DEFAULT   0x2A2AU
#define TLIGHT_DEVICE_ID_INVALID   0x0000U

/* 对码流程（正式版） */
#define PAIRING_MODE_TIMEOUT_MS 5000U

/* 设备ID持久化（Flash 最后一页）
 * 说明：这是“对码流程骨架”实现。量产时建议在链接脚本明确预留参数页，避免与程序区冲突。 */
#define DEVCFG_MAGIC0              0xD1U
#define DEVCFG_MAGIC1              0xA5U
#define DEVCFG_VERSION             0x03U
#define DEVCFG_FLASH_ADDR          (FLASH_BASE + FLASH_SIZE - FLASH_PAGE_SIZE)
#define DEVCFG_FLASH_PAGE          (FLASH_PAGE_NB - 1U)

#define TLIGHT_MODE_SWITCH_INTERVAL_MS 2000U
#define LINK_RX_TIMEOUT_MS           1200U  /* 超过 1.2s 未收到有效包，判定链路超时 */
#define LINK_STATS_LOG_INTERVAL_MS   3000U  /* 每 3s 打印一次统计，避免刷屏 */
#define UART_CMD_BUF_SIZE            48U
#define UART_CMD_POLL_MAX_BYTES      8U   /* 每轮主循环最多处理字节数，防止串口命令饿死按键扫描 */
#define UART_CMD_ECHO_ENABLE         0U   /* 1=回显输入字符；0=不回显（降低阻塞） */

/* TX 低功耗策略已在“宏配置总区 [B]”集中定义 */

/* TX 发包策略：按键事件触发短突发；对码窗口内才周期发
 * 可靠性调优（STANDBY 场景）：
 * - 事件突发帧数增加到 8，提升“至少命中一帧”概率
 * - 保持 40ms 帧间隔，降低对原时序与空口占用的冲击
 * - 新增事件后活跃保持窗口，避免“刚发完就回睡”导致 RX 偶发漏收 */
#define LINK_TX_EVENT_BURST_COUNT    8U
#define LINK_TX_BURST_INTERVAL_MS    40U
#define LINK_TX_PAIRING_INTERVAL_MS  120U
#define RF_TX_WAKEUP_SETTLE_MS       2U
#define RF_TX_BOOT_WARM_MS           1500U
#define LINK_TX_BOOT_BURST_COUNT     6U
#define TX_POST_EVENT_HOLD_MS        400U

/* TX 低功耗抗漏按参数（方向A）：
 * - 唤醒保护窗口：唤醒后至少保持一段时间不回睡，给按键去抖/识别留时间
 * - 按键活动保持：检测到任一按键按下后，再保持一段时间活跃，避免短按被切碎 */
#define TX_LP_WAKE_GUARD_MS          3000U /* 折中增强：唤醒后至少保持3s活跃，降低快速回睡与频繁Flash写入 */
#define TX_LP_KEY_ACTIVE_HOLD_MS     1100U /* 微调：按键活动保持1.1s，覆盖双击/长按又减少常亮耗电 */
#define TX_STUCK_KEY_FORCE_SLEEP_MS 10000U /* 卡键保护：任一键持续按下10s后强制进低功耗 */

/* ===================== TX 状态持久化宏配置表（量产可调） =====================
 * 使用说明（仅改这里即可完成车型调参）：
 * 1) TX_PERSIST_ENABLE
 *    - 1：启用状态持久化（推荐量产）
 *    - 0：关闭持久化（仅临时调试）
 *
 * 2) TX_STATE_SAVE_DEFER_MS（延迟合并）
 *    - 状态变化后，至少静默这么久再尝试保存
 *    - 值越大，写入越少；但掉电时丢“最新瞬态”的概率略升
 *
 * 3) TX_STATE_SAVE_MIN_INTERVAL_MS（最小写间隔）
 *    - 仅限制“后台延迟提交”的频率
 *    - 不限制“进 STANDBY 前兜底提交”（兜底优先，防止丢最终状态）
 *
 * 4) TX_STATE_SAVE_FORCE_BEFORE_STANDBY
 *    - 1：进入 STANDBY 前若 dirty 则强制保存一次（推荐）
 *    - 0：不兜底，寿命更优但有丢最终状态风险
 *
 * 5) TX_STATE_SAVE_COUNTER_LOG_ENABLE
 *    - 1：上电打印累计写入计数，便于寿命评估
 *    - 0：关闭该日志 */
#define TX_PERSIST_ENABLE                    1U
#define TX_STATE_SAVE_DEFER_MS            2000U
#define TX_STATE_SAVE_MIN_INTERVAL_MS    30000U
#define TX_STATE_SAVE_FORCE_BEFORE_STANDBY  1U
#define TX_STATE_SAVE_COUNTER_LOG_ENABLE     1U

/* 简化可见指示（用 LED_Pin，不做真实 LM3409 PWM 还原） */
#define LED_FLASH_PERIOD_MS  250U   /* 闪烁周期 */
#define LED_BREATH_PERIOD_MS 800U   /* 呼吸周期（简化为慢速占空） */

/* RX 模式 4~7 灯效可调参数（统一放宏，便于快速调效果）
 * 说明：
 * - 频率(Hz) ≈ 1000 / (2 * 半周期ms)
 * - 呼吸总周期(ms) = 从暗到亮再到暗的完整时间
 */
#define FX4_FLASH_HALF_PERIOD_MS     100U   /* 模式4：双远光同步闪 半周期，默认 2Hz */
#define FX5_ALT_HALF_PERIOD_MS       260U   /* 模式5：左右交替闪 半周期，留意这里通过修改定时器时间基解决了。 */
#define FX6_BREATH_PERIOD_MS        4000U   /* 模式6：呼吸灯总周期，默认 2s */
#define FX7_WHEEL_HALF_PERIOD_MS     100U   /* 模式7：左右轮闪 半周期，默认 2Hz 轮闪 */

/* 蓝线(HORN)独立爆闪参数（不复用右键模式链） */
#define HORN_FLASH_HALF_PERIOD_MS     50U   /* 50ms亮 / 50ms灭 */

#define DEBUG_ADC_VERBOSE  0   /* 调试：1=打印 ADC 采样值，完成后可改为 0 精简 */
#define DEBUG_SYNC_VERBOSE 1   /* 调试：1=打印详细同步信息，0=只打印关键事件 */
#define DEBUG_LED_VERBOSE  1   /* 调试：1=打印LED状态，0=不打印 */
#define DEBUG_PERIODIC     0   /* 调试：1=启用定期打印（影响功耗），0=禁用 */
#define UART_LOG_ENABLE    1   /* 串口日志总开关：1=开启打印，0=关闭打印（先用于排查波形） */
#define RX_LOCAL_EFFECT_TEST  0U  /* RX本地灯效测试：1=忽略RF命令并按本地脚本切模式 */
#define RX_LOCAL_STEP_MS      8000U /* 本地测试每个模式驻留时间 */
#define RX_LOCAL_TEST_BRIGHT  TLIGHT_BRIGHT_70 /* 本地测试亮度：70% 便于观察PWM相位锁定 */

/* PWM_GLOBAL 强制电平测试开关：
 * 0 = 按正常模式输出 PWM 占空比
 * 1 = 强制低电平（0%）
 * 2 = 强制高电平（100%） */
#define PWM_GLOBAL_FORCE_LEVEL  0U

/* 按键对码（L=PB6, R=PA2，内部下拉，按下为高） */
#define KEY_ACTIVE_LEVEL        GPIO_PIN_SET
#define KEY_DEBOUNCE_MS         20U
#define KEY_DEBOUNCE_MS_WKUP4_OFF 5U    /* WKUP4+OFF 专用：临时缩短右键防抖，加快超快双击命中 */
#define KEY_LONG_MS             2000U
#define KEY_COMBO_SIMULT_MS     150U   /* L/R 组合键“同时按下”窗口：超窗按单键处理 */

/* RX 对码结果反馈（通过近光灯） */
#define RX_PAIR_SUCCESS_HOLD_MS       1000U
#define RX_PAIR_FAIL_FLASH_HALF_MS    120U
#define RX_PAIR_FAIL_FLASH_TOGGLE_CNT 6U

/* TX 三键熄灯/点亮规则（v0.3.3） */
#define KEY_TX_LONG_MS          1000U
#define KEY_TX_DBL_MS           200U
#define KEY_TX_DBL_MS_WKUP4_OFF 500U   /* 仅 WKUP4+OFF 场景扩展双击窗口（验证用） */
#define KEY_TX_WKUP4_FAST_EDGE_DEBOUNCE_MS 8U /* WKUP4+OFF 专用：第二击下降沿快速捕获最小间隔（抗抖） */
#define KEY_TX_TAP_MAX_MS       320U
#define KEY_TX_DOUBLE_ENABLE    1U
#define TX_KEY_LED_ON_MS        180U

/* TX 右键本地调试（积木化排查）
 * 1: 仅测试右键 single/double/long 的识别与 LED 反馈，不走业务发送
 * 0: 关闭调试，恢复正常业务逻辑 */
#define TX_KEY_R_DEBUG_ONLY     0U
#define RDBG_LED_STEP_MS        500U
#define RDBG_LED_DBL_HOLD_MS    2000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static uint8_t RF_TX_Buf[RF_PACKET_SIZE]   = {0};
static uint8_t RF_RX_Buf[RF_PACKET_SIZE]   = {0};

static uint32_t g_cycle = 0;
static uint16_t g_phase_ms = 0;
static uint32_t g_last_tick_ms = 0;
static uint32_t g_last_tx_cycle = (uint32_t)-1;

static uint8_t  g_rf_mode = 0;
static uint8_t  g_led_state = 0;
static uint32_t g_led_on_tick = 0;

static uint8_t  g_rf_sleeping = 0;
static uint32_t g_adc_display_tick = 0;  /* ADC显示计时器 */
static uint32_t g_lp_enter_cnt = 0U;     /* 低功耗进入计数（诊断） */
static uint32_t g_lp_wake_cnt = 0U;      /* 低功耗唤醒计数（诊断） */
static uint32_t g_tx_lp_wake_tick = 0U;  /* TX 低功耗唤醒时刻+1（0=未生效），用于唤醒保护窗口 */
static uint32_t g_tx_key_active_until_tick = 0U; /* TX 按键活动保持截止时刻 */
static uint8_t  g_boot_lp_wufi = 0U;      /* 本次启动早期捕获的 WUFI 标志 */
static uint8_t  g_boot_lp_wuf1 = 0U;      /* 本次启动早期捕获的 WUF1 标志 */
static uint8_t  g_boot_lp_wuf2 = 0U;      /* 本次启动早期捕获的 WUF2 标志 */
static uint8_t  g_boot_lp_wuf3 = 0U;      /* 本次启动早期捕获的 WUF3 标志 */
static uint8_t  g_boot_lp_wuf4 = 0U;      /* 本次启动早期捕获的 WUF4 标志 */
static uint8_t  g_boot_lp_sbf = 0U;       /* 本次启动早期捕获的 SBF 标志 */
static uint8_t  g_tx_boot_wkup_pending_key = TLIGHT_KEY_NONE; /* STANDBY 唤醒补发单击：L/M/R（作为双击第1击注入状态机） */
static uint8_t  g_tx_r_wkup_off_guard = 0U;            /* WKUP4+OFF 保护态 */
static uint8_t  g_tx_r_wkup_wait_release = 0U;         /* WKUP4+OFF：等待右键先释放，再把唤醒首击注入为click#1 */
static uint32_t g_tx_wkup4_tick = 0U;                  /* WKUP4 捕获时刻（诊断） */
static uint32_t g_tx_r_click1_inject_tick = 0U;        /* R click#1 注入时刻（诊断） */
static uint32_t g_tx_r_wkup_click1_anchor_tick = 0U;   /* WKUP4+OFF 专用：双击第1击锚点时刻（前移到 t_wkup） */
static uint8_t  g_tx_r_wkup_ext_dbl_active = 0U;       /* WKUP4+OFF 专用扩展双击窗口激活标记 */
static volatile uint8_t  g_tx_r_wkup_fast_edge_armed = 0U;   /* WKUP4+OFF 专用：快速二击边沿捕获开关 */
static volatile uint8_t  g_tx_r_wkup_fast_edge_hit = 0U;     /* WKUP4+OFF 专用：ISR 已捕获到二击边沿 */
static volatile uint32_t g_tx_r_wkup_fast_edge_tick = 0U;    /* WKUP4+OFF 专用：二击边沿捕获时刻 */
static volatile uint32_t g_tx_r_wkup_fast_edge_last_tick = 0U;/* WKUP4+OFF 专用：最近一次边沿时刻（抗抖） */
static volatile uint32_t g_tx_r_wkup_exti_rise_tick = 0U;    /* 诊断：R 键 EXTI 上升沿时刻 */
static volatile uint32_t g_tx_r_wkup_exti_fall_tick = 0U;    /* 诊断：R 键 EXTI 下降沿时刻 */
static volatile uint8_t  g_tx_r_wkup_exti_rise_seen = 0U;    /* 诊断：本轮窗口内是否见到上升沿 */
static volatile uint8_t  g_tx_r_wkup_exti_fall_seen = 0U;    /* 诊断：本轮窗口内是否见到下降沿 */
static uint8_t  g_tx_r_wkup_pending_single = 0U;       /* 兼容保留：右键待提交单击 */
static uint32_t g_tx_r_wkup_pending_deadline = 0U;     /* 兼容保留：右键待提交单击截止时刻 */
static uint32_t g_tx_post_event_hold_until_tick = 0U;  /* 事件后活跃保持截止时刻，防止发完即回睡 */
static uint8_t  g_tx_state_dirty = 0U;                 /* TX 业务状态脏标记：1=待提交到Flash */
static uint32_t g_tx_state_dirty_since_tick = 0U;      /* 首次变脏时刻，用于延迟合并提交 */
static uint32_t g_tx_state_last_save_tick = 0U;        /* 最近一次成功提交时刻，用于最小间隔节流 */
static uint16_t g_tx_state_save_count = 0U;            /* TX 状态累计写入次数（上电日志可见） */

/* 链路调试统计（用于串口观测通信质量） */
static uint8_t  g_link_tx_seq = 0;
static uint8_t  g_link_last_rx_seq = 0;
static uint8_t  g_tx_last_key = TLIGHT_KEY_NONE;
static uint8_t  g_tx_last_trig = TLIGHT_TRIG_NONE;
static uint8_t  g_link_has_last_seq = 0;
static uint32_t g_link_tx_ok = 0;
static uint32_t g_link_tx_fail = 0;
static uint32_t g_link_rx_ok = 0;
static uint32_t g_link_rx_parse_err = 0;
static uint32_t g_link_rx_dev_mismatch = 0;
static uint32_t g_link_rx_dup = 0;
static uint32_t g_link_rx_jump = 0;
static uint8_t  g_link_timeout_flag = 0;
static uint32_t g_link_last_rx_tick = 0;
static uint32_t g_link_last_stats_tick = 0;
static uint32_t g_link_last_log_tx_ok = 0;
static uint32_t g_link_last_log_tx_fail = 0;
static uint8_t  g_link_last_log_tx_awake = 0xFFU;

/* 设备ID运行态（来自 Flash，16-bit） */
static uint16_t g_dev_id_local = TLIGHT_DEVICE_ID_DEFAULT;
static uint16_t g_dev_id_paired = TLIGHT_DEVICE_ID_DEFAULT;

/* 串口命令缓存（P1 临时对码入口） */
static char g_uart_cmd_buf[UART_CMD_BUF_SIZE] = {0};
static uint8_t g_uart_cmd_len = 0;

/* 按键状态（L/R 组合键进入对码） */
static uint8_t  g_pair_last_raw = 0U;
static uint8_t  g_pair_stable = 0U;
static uint8_t  g_pair_long_done = 0U;
static uint32_t g_pair_raw_change_tick = 0;
static uint32_t g_pair_press_start_tick = 0;
static uint32_t g_pair_l_down_tick = 0U;
static uint32_t g_pair_r_down_tick = 0U;
static uint8_t  g_pair_combo_armed = 0U;
static uint8_t  g_pair_guard_log_state = 0U;

static uint32_t g_tx_anykey_down_tick = 0U;
static uint8_t  g_tx_force_sleep_latched = 0U;

/* 正式对码流程状态 */
static uint8_t  g_pairing_mode = 0U;
static uint32_t g_pairing_enter_tick = 0U;
static uint32_t g_pair_led_tick = 0U;

typedef enum {
  PAIR_FB_NONE = 0,
  PAIR_FB_SUCCESS,
  PAIR_FB_FAIL
} PairFeedbackState_t;

typedef enum {
  RX_PAIR_FB_NONE = 0,
  RX_PAIR_FB_SUCCESS_HOLD,
  RX_PAIR_FB_FAIL_FLASH
} RxPairFeedbackState_t;

static PairFeedbackState_t g_pair_fb_state = PAIR_FB_NONE;
static uint32_t g_pair_fb_tick = 0U;
static uint8_t g_pair_fb_step = 0U;

static RxPairFeedbackState_t g_rx_pair_fb_state = RX_PAIR_FB_NONE;
static uint32_t g_rx_pair_fb_tick = 0U;
static uint8_t g_rx_pair_fb_step = 0U;

/* TX 三键事件（由按键状态机在主循环生成） */
static volatile uint8_t g_tx_evt_l = 0U;
static volatile uint8_t g_tx_evt_m = 0U;
static volatile uint8_t g_tx_evt_r = 0U;

/* TX 按键识别状态（单击/双击/长按） */
typedef struct {
  uint8_t  stable_level;
  uint8_t  last_raw;
  uint32_t raw_change_tick;
  uint32_t press_start_tick;
  uint32_t last_press_duration_ms;
  uint8_t  long_reported;
  uint8_t  click_pending;
  uint32_t first_click_tick;
  uint8_t  suppress_release_after_double;
} TxKeyState_t;

static TxKeyState_t g_txk_l = {0};
static TxKeyState_t g_txk_m = {0};
static TxKeyState_t g_txk_r = {0};

/* TX 业务态（按功能说明书语义）
 * 上电默认 OFF：确保首次右键单击走“OFF->特效1(模式4)”入口。 */
static uint8_t g_tx_mode = TLIGHT_MODE_OFF;
static uint8_t g_tx_bright = TLIGHT_BRIGHT_30;
static uint8_t g_tx_basic_mode = TLIGHT_MODE_MIN;  /* 1..3 */
static uint8_t g_tx_effect_mode = 4U;              /* 4..7 */
static uint8_t g_tx_in_effect = 0U;
static uint8_t g_tx_is_off = 1U;
static uint8_t g_tx_left_all_on_armed = 0U;
static uint8_t g_tx_need_send = 0U;
static uint8_t g_tx_burst_left = 0U;
static uint32_t g_tx_last_send_tick = 0U;
static uint8_t g_tx_rf_awake = 1U;
static uint32_t g_tx_rf_wakeup_tick = 0U;
static uint8_t g_tx_boot_warm_done = 0U;
static uint32_t g_tx_boot_warm_until = 0U;
static uint8_t g_tx_first_event_done = 0U;

#if APP_ROLE_TX_ONLY
static uint8_t g_rdbg_evt_single = 0U;
static uint8_t g_rdbg_evt_double = 0U;
static uint8_t g_rdbg_evt_long = 0U;
static uint32_t g_rdbg_single_cnt = 0U;
static uint32_t g_rdbg_double_cnt = 0U;
static uint32_t g_rdbg_long_cnt = 0U;

typedef enum {
  RDBG_LED_IDLE = 0,
  RDBG_LED_SINGLE_ON,
  RDBG_LED_LONG_ON,
  RDBG_LED_LONG_OFF,
  RDBG_LED_DBL_HOLD
} RdbgLedState_t;

static RdbgLedState_t g_rdbg_led_state = RDBG_LED_IDLE;
static uint32_t g_rdbg_led_mark = 0U;
static uint8_t g_rdbg_led_long_remain = 0U;
#endif

/* 双路 PWM 灯效状态（RX-only） */
static uint8_t g_light_mode = TLIGHT_MODE_OFF;
static uint8_t g_light_bright = TLIGHT_BRIGHT_30;

/* RX 三线输入状态（ACC/原车远光/喇叭） */
static uint8_t g_rx_acc_last = 0U;
static uint8_t g_rx_hi_last = 0U;
static uint8_t g_rx_horn_last = 0U;
static uint8_t g_rx_acc_raw_last = 0U;
static uint8_t g_rx_hi_raw_last = 0U;
static uint8_t g_rx_horn_raw_last = 0U;
static uint32_t g_rx_acc_raw_change_tick = 0U;
static uint32_t g_rx_hi_raw_change_tick = 0U;
static uint32_t g_rx_horn_raw_change_tick = 0U;
static uint8_t g_rx_horn_active = 0U;
static uint8_t g_rx_pre_horn_mode = TLIGHT_MODE_OFF;
static uint8_t g_rx_pre_horn_bright = TLIGHT_BRIGHT_30;
static uint8_t g_rx_remote_pending_valid = 0U;
static uint8_t g_rx_remote_pending_mode = TLIGHT_MODE_OFF;
static uint8_t g_rx_remote_pending_bright = TLIGHT_BRIGHT_30;

/* 黄线首次“双触发”解锁：未解锁前忽略黄线边沿；在窗口内发生两次边沿后解锁 */
static uint8_t g_rx_hi_gate_unlocked = 0U;
static uint8_t g_rx_hi_edge_cnt = 0U;
static uint32_t g_rx_hi_first_edge_tick = 0U;

/* 相位锁定节拍（1ms） */
static volatile uint16_t g_phase_1ms = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
static void DebugPrint(const char *s);
static void DebugPrintHex(const uint8_t *buf, uint8_t len);
static void DebugPrintDec(uint16_t val);
static void SyncTime_Update(void);
static void SyncLamp_Update(void);
static void BuildSyncPacket(uint8_t *pkt);
static uint16_t ParseSyncPacket(const uint8_t *pkt);
static void BuildTempLightPacket(uint8_t *pkt, uint8_t mode, uint8_t bright, uint8_t key, uint8_t trig);
static uint8_t ParseTempLightPacket(const uint8_t *pkt, uint8_t *mode, uint8_t *bright, uint8_t *key, uint8_t *trig);
static uint8_t TempLight_CalcChecksum(const uint8_t *pkt);
static uint8_t DevId_CalcCrc8(const uint8_t *buf, uint8_t len);
static uint16_t DevId_FromUid(void);
static void DevId_LoadFromFlash(void);
static uint8_t DevId_SaveToFlash(uint16_t local_id, uint16_t paired_id);
static uint8_t TxState_SaveToFlash(void);
static void TxState_MarkDirty(uint32_t now);
static void TxState_SaveWorker(uint32_t now);
static void UartCmd_Poll(void);
static void UartCmd_HandleLine(const char *line);
static void KeyPairing_Poll(uint32_t now);
static void LinkStats_Log(uint32_t now);
static void Sync_AdjustFromPacket(uint16_t rx_phase_ms);
static void Sync_MainLoop(void);
static void Light_Tick(uint32_t now);
static void PwmGlobal_Set(uint16_t pulse);
static void Channel_ApplyMask(uint8_t mask);
static void PairingLed_Update(uint32_t now);
static void PairingLed_Feedback(uint8_t success);
static void RxWireInput_Poll(uint32_t now);
static uint8_t TxKey_ReadRaw(uint16_t pin, GPIO_TypeDef *port);
static void TxKeys_Process(uint32_t now, uint8_t tick_1ms);
static void TxKeyR_DebugLedTask(uint32_t now);
static uint16_t GetPhase1ms(uint16_t period_ms);
static void EnterTxLowPower(void);
static void ExitTxLowPower(void);
static void LowPowerDiag_LogRfPins(void);
static void LowPowerDiag_LogWakeFlags(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct {
  uint8_t magic0;
  uint8_t magic1;
  uint8_t version;
  uint8_t local_id_l;
  uint8_t local_id_h;
  uint8_t paired_id_l;
  uint8_t paired_id_h;
  uint8_t tx_mode;
  uint8_t tx_bright;
  uint8_t tx_flags;   /* bit0: is_off, bit1: in_effect */
  uint8_t tx_basic_mode;
  uint8_t tx_effect_mode;
  uint8_t tx_save_cnt_l; /* 状态持久化累计写入计数（低字节） */
  uint8_t tx_save_cnt_h; /* 状态持久化累计写入计数（高字节） */
  uint8_t crc8;
  uint8_t tail;
} DevCfg_t;

static uint8_t DevId_CalcCrc8(const uint8_t *buf, uint8_t len)
{
  uint8_t x = 0;
  uint8_t i = 0;
  for (i = 0; i < len; i++) {
    x ^= buf[i];
  }
  return x;
}

static uint16_t DevId_FromUid(void)
{
  const uint32_t *uid = (const uint32_t *)UID_BASE;
  uint32_t mix = uid[0] ^ uid[1] ^ uid[2];
  mix ^= (mix >> 16);
  uint16_t id = (uint16_t)(mix & 0xFFFFU);
  if (id == TLIGHT_DEVICE_ID_INVALID) {
    id = TLIGHT_DEVICE_ID_DEFAULT;
  }
  return id;
}

static void DevId_LoadFromFlash(void)
{
  const DevCfg_t *cfg = (const DevCfg_t *)DEVCFG_FLASH_ADDR;
  uint8_t calc = DevId_CalcCrc8((const uint8_t *)cfg, 14U);
  uint16_t local_id = (uint16_t)((cfg->local_id_h << 8) | cfg->local_id_l);
  uint16_t paired_id = (uint16_t)((cfg->paired_id_h << 8) | cfg->paired_id_l);

  if ((cfg->magic0 == DEVCFG_MAGIC0) &&
      (cfg->magic1 == DEVCFG_MAGIC1) &&
      (cfg->version == DEVCFG_VERSION) &&
      (cfg->tail == TLIGHT_PKT_TAG_CONST) &&
      (local_id != TLIGHT_DEVICE_ID_INVALID) &&
      (paired_id != TLIGHT_DEVICE_ID_INVALID) &&
      (cfg->tx_mode <= TLIGHT_MODE_MAX) &&
      (cfg->tx_bright <= TLIGHT_BRIGHT_100) &&
      (cfg->tx_basic_mode >= TLIGHT_MODE_MIN) &&
      (cfg->tx_basic_mode <= 3U) &&
      (cfg->tx_effect_mode >= 4U) &&
      (cfg->tx_effect_mode <= TLIGHT_MODE_MAX) &&
      (cfg->crc8 == calc)) {
    g_dev_id_local = local_id;
    g_dev_id_paired = paired_id;
    g_tx_mode = cfg->tx_mode;
    g_tx_bright = cfg->tx_bright;
    g_tx_is_off = (uint8_t)(cfg->tx_flags & 0x01U);
    g_tx_in_effect = (uint8_t)((cfg->tx_flags >> 1) & 0x01U);
    g_tx_basic_mode = cfg->tx_basic_mode;
    g_tx_effect_mode = cfg->tx_effect_mode;
    g_tx_state_save_count = (uint16_t)((cfg->tx_save_cnt_h << 8) | cfg->tx_save_cnt_l);
    g_tx_left_all_on_armed = 0U;
    return;
  }

  /* Flash 未初始化或损坏：使用 UID 派生默认ID并回写一次 */
  g_dev_id_local = DevId_FromUid();
  g_dev_id_paired = g_dev_id_local;
  g_tx_mode = TLIGHT_MODE_OFF;
  g_tx_bright = TLIGHT_BRIGHT_30;
  g_tx_basic_mode = TLIGHT_MODE_MIN;
  g_tx_effect_mode = 4U;
  g_tx_in_effect = 0U;
  g_tx_is_off = 1U;
  g_tx_left_all_on_armed = 0U;
  g_tx_state_save_count = 0U;
  (void)DevId_SaveToFlash(g_dev_id_local, g_dev_id_paired);
}

static uint8_t DevId_SaveToFlash(uint16_t local_id, uint16_t paired_id)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t page_error = 0;
  DevCfg_t cfg;
  uint64_t row0 = 0;
  uint64_t row1 = 0;

  if ((local_id == TLIGHT_DEVICE_ID_INVALID) || (paired_id == TLIGHT_DEVICE_ID_INVALID)) {
    return 0;
  }

  memset(&cfg, 0, sizeof(cfg));
  cfg.magic0 = DEVCFG_MAGIC0;
  cfg.magic1 = DEVCFG_MAGIC1;
  cfg.version = DEVCFG_VERSION;
  cfg.local_id_l = (uint8_t)(local_id & 0xFFU);
  cfg.local_id_h = (uint8_t)(local_id >> 8);
  cfg.paired_id_l = (uint8_t)(paired_id & 0xFFU);
  cfg.paired_id_h = (uint8_t)(paired_id >> 8);
  cfg.tx_mode = g_tx_mode;
  cfg.tx_bright = g_tx_bright;
  cfg.tx_flags = (uint8_t)((g_tx_is_off & 0x01U) | ((g_tx_in_effect & 0x01U) << 1));
  cfg.tx_basic_mode = g_tx_basic_mode;
  cfg.tx_effect_mode = g_tx_effect_mode;
  cfg.tx_save_cnt_l = (uint8_t)(g_tx_state_save_count & 0xFFU);
  cfg.tx_save_cnt_h = (uint8_t)(g_tx_state_save_count >> 8);
  cfg.crc8 = DevId_CalcCrc8((const uint8_t *)&cfg, 14U);
  cfg.tail = TLIGHT_PKT_TAG_CONST;

  memcpy(&row0, &cfg, sizeof(row0));
  memcpy(&row1, ((const uint8_t *)&cfg) + sizeof(row0), sizeof(row1));

  HAL_FLASH_Unlock();

  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Page = DEVCFG_FLASH_PAGE;
  erase.NbPages = 1;
  if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) {
    HAL_FLASH_Lock();
    return 0;
  }

  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, DEVCFG_FLASH_ADDR, row0) != HAL_OK) {
    HAL_FLASH_Lock();
    return 0;
  }

  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, DEVCFG_FLASH_ADDR + 8U, row1) != HAL_OK) {
    HAL_FLASH_Lock();
    return 0;
  }

  HAL_FLASH_Lock();

  g_dev_id_local = local_id;
  g_dev_id_paired = paired_id;
  return 1;
}

/* TX 状态持久化：把当前灯态写入 Flash，供 STANDBY 唤醒后恢复上次业务状态。 */
static uint8_t TxState_SaveToFlash(void)
{
#if APP_ROLE_TX_ONLY
  if (TX_PERSIST_ENABLE == 0U) {
    return 1U;
  }

  if (g_tx_state_save_count != 0xFFFFU) {
    g_tx_state_save_count++;
  }

  if (DevId_SaveToFlash(g_dev_id_local, g_dev_id_paired) == 0U) {
    if (g_tx_state_save_count != 0U) {
      g_tx_state_save_count--;
    }
    DebugPrint("[TX-STATE] flash save fail\r\n");
    return 0U;
  }
  g_tx_state_last_save_tick = HAL_GetTick();
  return 1U;
#else
  return 0U;
#endif
}

/* 标记状态变更：由按键业务触发，交给后台节流保存，避免每次按键都写Flash。 */
static void TxState_MarkDirty(uint32_t now)
{
#if APP_ROLE_TX_ONLY
  if (g_tx_state_dirty == 0U) {
    g_tx_state_dirty = 1U;
    g_tx_state_dirty_since_tick = now;
  }
#else
  (void)now;
#endif
}

/* TX 状态保存后台任务：
 * - 先做延迟合并提交（减少写入次数）
 * - 满足最小间隔后再落盘
 * - 在进入 STANDBY 前由主流程强制兜底提交（不在这里做） */
static void TxState_SaveWorker(uint32_t now)
{
#if APP_ROLE_TX_ONLY
  if (g_tx_state_dirty == 0U) {
    return;
  }

  if ((now - g_tx_state_dirty_since_tick) < TX_STATE_SAVE_DEFER_MS) {
    return;
  }

  if ((g_tx_state_last_save_tick != 0U) && ((now - g_tx_state_last_save_tick) < TX_STATE_SAVE_MIN_INTERVAL_MS)) {
    return;
  }

  if (TxState_SaveToFlash() != 0U) {
    g_tx_state_dirty = 0U;
    DebugPrint("[TX-STATE] save commit(defer)\r\n");
  }
#else
  (void)now;
#endif
}

static void UartCmd_HandleLine(const char *line)
{
  unsigned long v = 0;

  if ((line[0] == '\0') || (line[0] == '\r') || (line[0] == '\n')) {
    return;
  }

  if ((strcmp(line, "HELP") == 0) || (strcmp(line, "help") == 0)) {
    DebugPrint("[CMD] HELP/ID?/PAIR?/ID <1..65535>/PAIR <1..65535>\r\n");
    return;
  }

  if ((strcmp(line, "ID?") == 0) || (strcmp(line, "id?") == 0)) {
    DebugPrint("[CMD] ID=");
    DebugPrintHex((const uint8_t *)&g_dev_id_local, 2);
    DebugPrint("\r\n");
    return;
  }

  if ((strcmp(line, "PAIR?") == 0) || (strcmp(line, "pair?") == 0)) {
    DebugPrint("[CMD] PAIRED=");
    DebugPrintHex((const uint8_t *)&g_dev_id_paired, 2);
    DebugPrint("\r\n");
    return;
  }

  if ((strncmp(line, "ID ", 3) == 0) || (strncmp(line, "id ", 3) == 0)) {
    v = strtoul(&line[3], NULL, 10);
    if ((v >= 1UL) && (v <= 65535UL)) {
      if (DevId_SaveToFlash((uint16_t)v, g_dev_id_paired)) {
        DebugPrint("[CMD] ID saved: ");
        DebugPrintHex((const uint8_t *)&g_dev_id_local, 2);
        DebugPrint("\r\n");
      } else {
        DebugPrint("[CMD-ERR] ID save fail\r\n");
      }
    } else {
      DebugPrint("[CMD-ERR] ID range 1..65535\r\n");
    }
    return;
  }

  if ((strncmp(line, "PAIR ", 5) == 0) || (strncmp(line, "pair ", 5) == 0)) {
    v = strtoul(&line[5], NULL, 10);
    if ((v >= 1UL) && (v <= 65535UL)) {
      if (DevId_SaveToFlash(g_dev_id_local, (uint16_t)v)) {
        DebugPrint("[CMD] PAIRED saved: ");
        DebugPrintHex((const uint8_t *)&g_dev_id_paired, 2);
        DebugPrint("\r\n");
      } else {
        DebugPrint("[CMD-ERR] PAIRED save fail\r\n");
      }
    } else {
      DebugPrint("[CMD-ERR] PAIR range 1..65535\r\n");
    }
    return;
  }

  DebugPrint("[CMD-ERR] Unknown. use HELP\r\n");
}

static void UartCmd_Poll(void)
{
  uint8_t ch = 0;
  uint8_t processed = 0U;

  while ((processed < UART_CMD_POLL_MAX_BYTES) && (HAL_UART_Receive(&huart1, &ch, 1, 0) == HAL_OK)) {
#if UART_CMD_ECHO_ENABLE
    /* 回显，便于确认命令确实被固件接收到了 */
    HAL_UART_Transmit(&huart1, &ch, 1, 20);
#endif

    if ((ch == '\r') || (ch == '\n')) {
      DebugPrint("\r\n");
      g_uart_cmd_buf[g_uart_cmd_len] = '\0';
      UartCmd_HandleLine(g_uart_cmd_buf);
      g_uart_cmd_len = 0;
      processed++;
      continue;
    }

    if ((ch >= 32U) && (ch <= 126U)) {
      if (g_uart_cmd_len < (UART_CMD_BUF_SIZE - 1U)) {
        g_uart_cmd_buf[g_uart_cmd_len++] = (char)ch;
      } else {
        g_uart_cmd_len = 0;
        DebugPrint("[CMD-ERR] too long\r\n");
      }
    }

    processed++;
  }
}

static void KeyPairing_Poll(uint32_t now)
{
  uint8_t l_raw = (HAL_GPIO_ReadPin(L_KEY_GPIO_Port, L_KEY_Pin) == KEY_ACTIVE_LEVEL) ? 1U : 0U;
  uint8_t r_raw = (HAL_GPIO_ReadPin(R_KEY_GPIO_Port, R_KEY_Pin) == KEY_ACTIVE_LEVEL) ? 1U : 0U;
  uint8_t raw = (l_raw && r_raw) ? 1U : 0U;

  /* 退出对码超时（TX/RX 通用） */
  if ((g_pairing_mode != 0U) && ((now - g_pairing_enter_tick) >= PAIRING_MODE_TIMEOUT_MS)) {
    g_pairing_mode = 0U;
    g_pairing_enter_tick = 0U;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    DebugPrint("[PAIR] timeout\r\n");
#if APP_ROLE_TX_ONLY
    /* TX 对码超时指示：长亮 500ms（反馈期间独占 LED） */
    PairingLed_Feedback(0U);
#endif
  }

#if APP_ROLE_TX_ONLY
  /* 记录 L/R 各自按下时刻，用于“同时按下窗口”判定 */
  if ((l_raw != 0U) && (g_pair_l_down_tick == 0U)) {
    g_pair_l_down_tick = now;
  }
  if ((l_raw == 0U) && (g_pair_l_down_tick != 0U)) {
    g_pair_l_down_tick = 0U;
  }

  if ((r_raw != 0U) && (g_pair_r_down_tick == 0U)) {
    g_pair_r_down_tick = now;
  }
  if ((r_raw == 0U) && (g_pair_r_down_tick != 0U)) {
    g_pair_r_down_tick = 0U;
  }

  if ((l_raw != 0U) && (r_raw != 0U) && (g_pair_l_down_tick != 0U) && (g_pair_r_down_tick != 0U)) {
    uint32_t dt = (g_pair_l_down_tick >= g_pair_r_down_tick) ?
                  (g_pair_l_down_tick - g_pair_r_down_tick) :
                  (g_pair_r_down_tick - g_pair_l_down_tick);
    g_pair_combo_armed = (dt <= KEY_COMBO_SIMULT_MS) ? 1U : 0U;
  } else {
    g_pair_combo_armed = 0U;
  }

  raw = ((l_raw != 0U) && (r_raw != 0U) && (g_pair_combo_armed != 0U)) ? 1U : 0U;

  if ((g_pairing_mode == 0U) && (raw == 0U)) {
    return;
  }

  if (raw != g_pair_last_raw) {
    g_pair_last_raw = raw;
    g_pair_raw_change_tick = now;
  }

  if ((now - g_pair_raw_change_tick) < KEY_DEBOUNCE_MS) {
    return;
  }

  if (raw != g_pair_stable) {
    g_pair_stable = raw;

    DebugPrint("[KEY] L=");
    DebugPrintDec((uint16_t)l_raw);
    DebugPrint(" R=");
    DebugPrintDec((uint16_t)r_raw);
    DebugPrint(" LR=");
    DebugPrintDec((uint16_t)raw);
    DebugPrint("\r\n");

    if (g_pair_stable == 1U) {
      g_pair_press_start_tick = now;
      g_pair_long_done = 0U;
      DebugPrint("[PAIR] LR down\r\n");
    } else {
      g_pair_press_start_tick = 0U;
      g_pair_long_done = 0U;
      DebugPrint("[PAIR] LR up\r\n");
    }
  }

  if ((g_pair_stable == 1U) && (g_pair_long_done == 0U) && (g_pair_press_start_tick != 0U)) {
    uint32_t held = now - g_pair_press_start_tick;

    if (held >= KEY_LONG_MS) {
      g_pairing_mode = 1U;
      g_pairing_enter_tick = now;
      g_pair_led_tick = now;

      /* 进入对码时清空三键状态，避免“进入前已累计的单/双/长按事件”在后续被误执行。 */
      memset(&g_txk_l, 0, sizeof(g_txk_l));
      memset(&g_txk_m, 0, sizeof(g_txk_m));
      memset(&g_txk_r, 0, sizeof(g_txk_r));
      g_tx_evt_l = 0U;
      g_tx_evt_m = 0U;
      g_tx_evt_r = 0U;

      DebugPrint("[PAIR] enter (tx)\r\n");
      g_pair_long_done = 1U;
      g_tx_need_send = 1U;
    }
  }
#else
  (void)l_raw;
  (void)r_raw;
  (void)raw;
#endif
}

/* TX 进入低功耗前的统一收敛：
 * 1) RF 无条件下发睡眠
 * 2) 三线固定为 CSN=高、SCK=低、DATA=低，避免悬空/毛刺导致模块额外耗电
 * 3) 关闭串口，减少静态功耗 */
static void LowPowerDiag_LogRfPins(void)
{
#if LOWPOWER_DIAG_ENABLE
  uint8_t csn = (HAL_GPIO_ReadPin(RF_CSN_GPIO_Port, RF_CSN_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  uint8_t sck = (HAL_GPIO_ReadPin(RF_SCK_GPIO_Port, RF_SCK_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  uint8_t dat = (HAL_GPIO_ReadPin(RF_DATA_GPIO_Port, RF_DATA_Pin) == GPIO_PIN_SET) ? 1U : 0U;

  DebugPrint("[LP-RF-PIN] CSN=");
  DebugPrintDec(csn);
  DebugPrint(" SCK=");
  DebugPrintDec(sck);
  DebugPrint(" DAT=");
  DebugPrintDec(dat);
  DebugPrint("\r\n");
#endif
}

static void LowPowerDiag_LogWakeFlags(void)
{
#if LOWPOWER_DIAG_ENABLE
  uint8_t wufi = (__HAL_PWR_GET_FLAG(PWR_FLAG_WUFI) != 0U) ? 1U : 0U;
  uint8_t wuf1 = (__HAL_PWR_GET_FLAG(PWR_FLAG_WUF1) != 0U) ? 1U : 0U;
  uint8_t wuf2 = (__HAL_PWR_GET_FLAG(PWR_FLAG_WUF2) != 0U) ? 1U : 0U;
#if defined(PWR_FLAG_WUF3)
  uint8_t wuf3 = (__HAL_PWR_GET_FLAG(PWR_FLAG_WUF3) != 0U) ? 1U : 0U;
#else
  uint8_t wuf3 = 0U;
#endif
#if defined(PWR_FLAG_WUF4)
  uint8_t wuf4 = (__HAL_PWR_GET_FLAG(PWR_FLAG_WUF4) != 0U) ? 1U : 0U;
#else
  uint8_t wuf4 = 0U;
#endif
#if defined(PWR_FLAG_SBF)
  uint8_t sbf  = (__HAL_PWR_GET_FLAG(PWR_FLAG_SBF)  != 0U) ? 1U : 0U;
#else
  uint8_t sbf  = 0U;
#endif

  DebugPrint("[LP-WAKE] cnt=");
  DebugPrintDec((uint16_t)(g_lp_wake_cnt & 0xFFFFU));
  DebugPrint(" WUFI=");
  DebugPrintDec(wufi);
  DebugPrint(" WUF1=");
  DebugPrintDec(wuf1);
  DebugPrint(" WUF2=");
  DebugPrintDec(wuf2);
  DebugPrint(" WUF3=");
  DebugPrintDec(wuf3);
  DebugPrint(" WUF4=");
  DebugPrintDec(wuf4);
  DebugPrint(" SBF=");
  DebugPrintDec(sbf);
  DebugPrint("\r\n");
#endif
}

static void EnterTxLowPower(void)
{
  /* 诊断打印必须在串口关闭前执行 */
#if LOWPOWER_DIAG_ENABLE
  g_lp_enter_cnt++;
  DebugPrint("[LP-ENTER] cnt=");
  DebugPrintDec((uint16_t)(g_lp_enter_cnt & 0xFFFFU));
  DebugPrint("\r\n");
#endif

  RF_Link_Sleep();
  g_rf_sleeping = 1U;

  HAL_GPIO_WritePin(RF_CSN_GPIO_Port, RF_CSN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RF_SCK_GPIO_Port, RF_SCK_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RF_DATA_GPIO_Port, RF_DATA_Pin, GPIO_PIN_RESET);

#if LOWPOWER_DIAG_ENABLE
  LowPowerDiag_LogRfPins();
#endif

  /* 低功耗前关闭串口，减少外设静态功耗 */
  HAL_UART_DeInit(&huart1);

#if (TX_LOWPOWER_MODE == TX_LOWPOWER_MODE_STOP)
  /* STOP 前暂停 1ms Tick 与 TIM14，避免高频周期唤醒导致电流居高不下 */
  HAL_TIM_Base_Stop_IT(&htim14);
  HAL_SuspendTick();
#elif (TX_LOWPOWER_MODE == TX_LOWPOWER_MODE_STANDBY)
  /* STANDBY 前停掉 1ms Tick 与 TIM14，避免无意义中断活动 */
  HAL_TIM_Base_Stop_IT(&htim14);
  HAL_SuspendTick();

  /* 说明：WKUP 引脚在 PWR 视图不支持直接配置上下拉。
   * 进入 STANDBY 前通过 PWREx 固定关键引脚电平，避免待机期间悬空导致额外静态功耗：
   * - PA0  (WKUP1, M_KEY) 下拉：保持空闲低电平，仅在外部拉高时唤醒
   * - PA2  (WKUP4, R_KEY) 下拉：新右键引脚（Cube 无法直接配上下拉，这里手动兜底）
   * - PB6  (WKUP3, L_KEY) 下拉：新左键引脚（Cube 无法直接配上下拉，这里手动兜底）
   * - PA4  (RF_CSN)       上拉：保证 RF 片选在待机期间维持高电平（模块睡眠态）
   * - PA5/PA7             下拉：避免时钟/数据脚悬空抖动 */
  HAL_PWREx_DisableGPIOPullUp(PWR_GPIO_A, PWR_GPIO_BIT_0 | PWR_GPIO_BIT_2 | PWR_GPIO_BIT_5 | PWR_GPIO_BIT_7);
  HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_A, PWR_GPIO_BIT_0 | PWR_GPIO_BIT_2 | PWR_GPIO_BIT_5 | PWR_GPIO_BIT_7);
  HAL_PWREx_DisableGPIOPullUp(PWR_GPIO_B, PWR_GPIO_BIT_6);
  HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_B, PWR_GPIO_BIT_6);
  HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_A, PWR_GPIO_BIT_4);
  HAL_PWREx_DisableGPIOPullDown(PWR_GPIO_A, PWR_GPIO_BIT_4);
  HAL_PWREx_EnablePullUpPullDownConfig();

#if LOWPOWER_DIAG_ENABLE
  DebugPrint("[LP-PWR-PULL] PA0=PD PA2=PD PB6=PD PA4=PU PA5=PD PA7=PD\r\n");
#endif

  /* 清理历史唤醒标志，避免“刚进就醒” */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);

  /* 使能 WKUP1/3/4，保持“高电平唤醒”与当前按键高有效一致 */
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1_HIGH);
#if defined(PWR_WAKEUP_PIN3_HIGH)
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN3_HIGH);
#endif
#if defined(PWR_WAKEUP_PIN4_HIGH)
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN4_HIGH);
#endif
#endif
}

/* TX 退出低功耗后的统一恢复：
 * STOP 唤醒后需要先恢复系统时钟，再恢复 Tick/TIM14，最后初始化串口。
 * SLEEP 唤醒仅需恢复串口。 */
static void ExitTxLowPower(void)
{
#if (TX_LOWPOWER_MODE == TX_LOWPOWER_MODE_STOP)
  SystemClock_Config();
  HAL_ResumeTick();
  HAL_TIM_Base_Start_IT(&htim14);
#endif
  MX_USART1_UART_Init();

  g_tx_lp_wake_tick = HAL_GetTick() + 1U;
#if LOWPOWER_DIAG_ENABLE
  DebugPrint("[TX-LP] wake_guard_ms=");
  DebugPrintDec((uint16_t)TX_LP_WAKE_GUARD_MS);
  DebugPrint("\r\n");
#endif

#if LOWPOWER_DIAG_ENABLE
  g_lp_wake_cnt++;
  LowPowerDiag_LogWakeFlags();
#endif

#if APP_ROLE_TX_ONLY
  g_tx_lp_wake_tick = HAL_GetTick() + 1U;
#endif
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
#if APP_ROLE_TX_ONLY
  /* TX 启动早期先读取唤醒来源，再清理标志，避免证据被提前清掉 */
  g_boot_lp_wufi = (__HAL_PWR_GET_FLAG(PWR_FLAG_WUFI) != 0U) ? 1U : 0U;
  g_boot_lp_wuf1 = (__HAL_PWR_GET_FLAG(PWR_FLAG_WUF1) != 0U) ? 1U : 0U;
  g_boot_lp_wuf2 = (__HAL_PWR_GET_FLAG(PWR_FLAG_WUF2) != 0U) ? 1U : 0U;
#if defined(PWR_FLAG_WUF3)
  g_boot_lp_wuf3 = (__HAL_PWR_GET_FLAG(PWR_FLAG_WUF3) != 0U) ? 1U : 0U;
#else
  g_boot_lp_wuf3 = 0U;
#endif
#if defined(PWR_FLAG_WUF4)
  g_boot_lp_wuf4 = (__HAL_PWR_GET_FLAG(PWR_FLAG_WUF4) != 0U) ? 1U : 0U;
#else
  g_boot_lp_wuf4 = 0U;
#endif
#if defined(PWR_FLAG_SBF)
  g_boot_lp_sbf  = (__HAL_PWR_GET_FLAG(PWR_FLAG_SBF)  != 0U) ? 1U : 0U;
#else
  g_boot_lp_sbf  = 0U;
#endif

  /* 读取后再清标志，防止影响后续进入低功耗 */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);
#if defined(PWR_FLAG_SBF)
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SBF);
#endif
#endif

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  /* 2026-04-06 阶段摘要：本文件已完成三键唤醒重映射、WUF1/3/4 注入与 PWM 通道迁移；
   * 后续优化项见 docs/09-阶段成果总结_2026-04-06.md。 */
  HAL_TIM_Base_Start_IT(&htim14);
  DevId_LoadFromFlash();

#if APP_ROLE_TX_ONLY
  /* STANDBY 唤醒会走重启路径：这里统一拉起“唤醒活跃保护”起点，
   * 保证 TX_LP_WAKE_GUARD_MS 在主循环首轮就能生效。 */
  g_tx_lp_wake_tick = HAL_GetTick() + 1U;
#if LOWPOWER_DIAG_ENABLE
  DebugPrint("[TX-LP] wake_guard_ms=");
  DebugPrintDec((uint16_t)TX_LP_WAKE_GUARD_MS);
  DebugPrint("\r\n");
#endif
#endif
  DebugPrint("[FW] " FW_VERSION "\r\n");

  DebugPrint("[CFG] role=");
#if APP_ROLE_TX_ONLY
  DebugPrint("TX");
#else
  DebugPrint("RX");
#endif
  DebugPrint(" tx_only=");
#if APP_ROLE_TX_ONLY
  DebugPrint("1");
#else
  DebugPrint("0");
#endif
  DebugPrint(" rx_only=");
#if APP_ROLE_RX_ONLY
  DebugPrint("1");
#else
  DebugPrint("0");
#endif
  DebugPrint(" link_test=");
#if APP_RF_LINK_TEST
  DebugPrint("1");
#else
  DebugPrint("0");
#endif
  DebugPrint(" lowpower=");
#if (TX_LOWPOWER_MODE == TX_LOWPOWER_MODE_STOP)
  DebugPrint("STOP");
#elif (TX_LOWPOWER_MODE == TX_LOWPOWER_MODE_STANDBY)
  DebugPrint("STANDBY");
#else
  DebugPrint("SLEEP");
#endif
  DebugPrint(" rx_debounce_ms=");
  DebugPrintDec((uint16_t)RX_WIRE_DEBOUNCE_MS);
  DebugPrint(" rx_polarity=");
  DebugPrint(RX_WIRE_POLARITY_NAME);
  DebugPrint(" wuf_prio=1>3>4");
  DebugPrint(" horn_pending=last-wins");
  DebugPrint("\r\n");

  DebugPrint("[DEV] local=");
  DebugPrintHex((const uint8_t *)&g_dev_id_local, 2);
  DebugPrint(" paired=");
  DebugPrintHex((const uint8_t *)&g_dev_id_paired, 2);
  DebugPrint("\r\n");
  DebugPrint("[CMD] type HELP\r\n");

#if APP_ROLE_TX_ONLY
  DebugPrint("[BOOT-LP] WUFI=");
  DebugPrintDec(g_boot_lp_wufi);
  DebugPrint(" WUF1=");
  DebugPrintDec(g_boot_lp_wuf1);
  DebugPrint(" WUF2=");
  DebugPrintDec(g_boot_lp_wuf2);
  DebugPrint(" WUF3=");
  DebugPrintDec(g_boot_lp_wuf3);
  DebugPrint(" WUF4=");
  DebugPrintDec(g_boot_lp_wuf4);
  DebugPrint(" SBF=");
  DebugPrintDec(g_boot_lp_sbf);
  DebugPrint("\r\n");

#if TX_STATE_SAVE_COUNTER_LOG_ENABLE
  DebugPrint("[TX-STATE] save_count=");
  DebugPrintDec(g_tx_state_save_count);
  DebugPrint("\r\n");
#endif

  /* STANDBY 三键唤醒补偿（WKUP1/3/4）：
   * - WKUP1(PA0)=中键 M
   * - WKUP3(PB6)=左键 L
   * - WKUP4(PA2)=右键 R
   * 唤醒后默认挂起“对应单击补发”，按键扫描阶段再做按住过滤。 */
  g_tx_boot_wkup_pending_key = TLIGHT_KEY_NONE;
  g_tx_r_wkup_off_guard = 0U;
  g_tx_r_wkup_wait_release = 0U;
  g_tx_wkup4_tick = 0U;
  g_tx_r_click1_inject_tick = 0U;
  g_tx_r_wkup_click1_anchor_tick = 0U;
  g_tx_r_wkup_ext_dbl_active = 0U;
  g_tx_r_wkup_fast_edge_armed = 0U;
  g_tx_r_wkup_fast_edge_hit = 0U;
  g_tx_r_wkup_fast_edge_tick = 0U;
  g_tx_r_wkup_fast_edge_last_tick = 0U;
  g_tx_r_wkup_exti_rise_tick = 0U;
  g_tx_r_wkup_exti_fall_tick = 0U;
  g_tx_r_wkup_exti_rise_seen = 0U;
  g_tx_r_wkup_exti_fall_seen = 0U;
  g_tx_r_wkup_pending_single = 0U;
  g_tx_r_wkup_pending_deadline = 0U;
  if (g_boot_lp_wuf1 != 0U) {
    g_tx_boot_wkup_pending_key = TLIGHT_KEY_M;
    DebugPrint("[LP-WKUP] WKUP1 wake detected, pending M-key inject\r\n");
  } else if (g_boot_lp_wuf3 != 0U) {
    g_tx_boot_wkup_pending_key = TLIGHT_KEY_L;
    DebugPrint("[LP-WKUP] WKUP3 wake detected, pending L-key inject\r\n");
  } else if (g_boot_lp_wuf4 != 0U) {
    g_tx_boot_wkup_pending_key = TLIGHT_KEY_R;
    g_tx_wkup4_tick = HAL_GetTick();
    if (g_tx_is_off != 0U) {
      /* WKUP4+OFF 专用：将双击第一击锚点前移到 t_wkup，
       * 避免“等待释放后再注入”导致快双击第二击落在窗口外。 */
      g_tx_r_wkup_off_guard = 1U;
      g_tx_r_wkup_wait_release = 1U;
      g_tx_r_wkup_click1_anchor_tick = g_tx_wkup4_tick;
      g_txk_r.click_pending = 1U;
      g_txk_r.first_click_tick = g_tx_r_wkup_click1_anchor_tick;
      g_tx_r_wkup_ext_dbl_active = 1U;
      g_tx_r_wkup_fast_edge_armed = 1U;
      g_tx_r_wkup_fast_edge_hit = 0U;
      g_tx_r_wkup_fast_edge_tick = 0U;
      g_tx_r_wkup_fast_edge_last_tick = 0U;
      DebugPrint("[LP-WKUP] WKUP4+OFF: anchor click#1 at t_wkup, wait release\r\n");
    }
    DebugPrint("[LP-WKUP] WKUP4 wake detected, pending R-key inject\r\n");
  }
#endif

#if APP_ROLE_TX_ONLY
  /* TX 上电自检：LED 快闪 1 次 */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  HAL_Delay(70);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
#endif

  /* PA3 已切换为 MOS_L_HI（GPIO 输出），不再作为 PWM 通道 */
  HAL_Delay(100);

  /* 单路全局 PWM 启动（PA1=TIM1_CH2） */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  /* 使能 CCR 预装载：比较值在更新事件统一生效 */
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_2);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  Channel_ApplyMask(0U);



#if APP_ROLE_TX_ONLY && TX_KEY_R_DEBUG_ONLY
  DebugPrint("RF Bypass (R-key debug only)\r\n");
#else
    /* RF 模块初始化 */
  DebugPrint("RF Init...\r\n");
  RF_Link_Init();
  DebugPrint("RF Init OK\r\n");

#if APP_ROLE_RX_ONLY
  /* RX 角色：上电进入接收模式 */
  DebugPrint("RF Config RX...\r\n");
  RF_Link_ConfigRx(RF_RX_CHANNEL);
  g_rf_mode = 0;
#elif APP_ROLE_TX_ONLY
  /* TX 角色：上电进入发送模式并保温一段时间，再按空闲策略休眠 */
  DebugPrint("RF Config TX...\r\n");
  RF_Link_ConfigTx(RF_TX_CHANNEL);
  g_rf_mode = 1;
  g_tx_rf_awake = 1U;
  g_tx_rf_wakeup_tick = HAL_GetTick();
  g_tx_boot_warm_done = 0U;
  g_tx_boot_warm_until = HAL_GetTick() + RF_TX_BOOT_WARM_MS;
  g_tx_first_event_done = 0U;
  DebugPrint("[RF] TX warm@boot\r\n");
#endif
  DebugPrint("RF Ready\r\n");
#endif
  
  /* 调试：确认TX固定时间 */
  DebugPrint("[SYNC] TX fixed at ");
  DebugPrintDec(SYNC_TX_TIME_MS);
  DebugPrint("ms, Window: ");
  DebugPrintDec(SYNC_TX_TIME_MS);
  DebugPrint("- ");
  DebugPrintDec(SYNC_TX_TIME_MS + 50U);
  DebugPrint("ms\r\n");





  /* 初始化同步数据 */
  g_cycle = 0;
  g_phase_ms = 0;
  g_last_tick_ms = HAL_GetTick();
  g_last_tx_cycle = (uint32_t)-1;

  g_led_state = 0;
  g_led_on_tick = 0;

  g_rf_sleeping = 0;

#if APP_ROLE_RX_ONLY
  /* RX 上电后自动进入 5 秒对码窗口（保持旧版行为）。 */
  g_pairing_mode = 1U;
  g_pairing_enter_tick = HAL_GetTick();
  DebugPrint("[PAIR] auto enter (rx, 5s window)\r\n");

  /* RX 三线输入初值采样（用于遥控命令与三线优先级仲裁） */
  g_rx_acc_last = (HAL_GPIO_ReadPin(RX_ACC_IN_PORT, RX_ACC_IN_PIN) == RX_IN_ACTIVE_LEVEL) ? 1U : 0U;
  g_rx_hi_last = (HAL_GPIO_ReadPin(RX_CAR_HI_IN_PORT, RX_CAR_HI_IN_PIN) == RX_IN_ACTIVE_LEVEL) ? 1U : 0U;
  g_rx_horn_last = (HAL_GPIO_ReadPin(RX_HORN_IN_PORT, RX_HORN_IN_PIN) == RX_IN_ACTIVE_LEVEL) ? 1U : 0U;
  g_rx_acc_raw_last = g_rx_acc_last;
  g_rx_hi_raw_last = g_rx_hi_last;
  g_rx_horn_raw_last = g_rx_horn_last;
  g_rx_acc_raw_change_tick = HAL_GetTick();
  g_rx_hi_raw_change_tick = g_rx_acc_raw_change_tick;
  g_rx_horn_raw_change_tick = g_rx_acc_raw_change_tick;
  g_rx_horn_active = g_rx_horn_last;
#endif

#if DEBUG_ADC_VERBOSE
  DebugPrint("[ADC] disabled in this project branch\r\n");
#endif

      /* 看门狗：暂时禁用，后续再解决 */
  // MX_IWDG_Init();
  // 
  // /* 等待看门狗计数器超过窗口值（3072），确保第一次喂狗在窗口内 */
  // HAL_Delay(100);  /* 100ms 足够让计数器超过 3072（32.7s * 3072/4095 ≈ 24.5s 的 0.4%） */
  // 
  // /* 第一次喂狗，确保在窗口内 */
  // HAL_IWDG_Refresh(&hiwdg);
  // DebugPrint("[IWDG] First refresh OK\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
  {
    /* 积木8：喂狗 - 暂时禁用 */
    // HAL_IWDG_Refresh(&hiwdg);

    /* P1：串口命令入口（临时对码） */
    UartCmd_Poll();

      /* 收发业务主流程（TX/RX 角色按宏切换）；
       * APP_RF_LINK_TEST 仅用于控制附加调试打印，不再决定是否执行该主流程。 */
      static uint32_t s_last_tick_ms = 0U;
      uint32_t now = HAL_GetTick();
      uint8_t tick_1ms = 0U;
      if (now != s_last_tick_ms) {
        s_last_tick_ms = now;
        tick_1ms = 1U;
      }

      /* P2：按键对码入口（L+R 长按） */
      KeyPairing_Poll(now);

      /* 对码窗口提示（TX/RX 都可用，且优先级高于普通提示） */
      PairingLed_Update(now);

#if APP_ROLE_TX_ONLY
      /* 三键语义：按参考工程改为严格 1ms 节拍驱动按键扫描 */
      TxKeys_Process(now, tick_1ms);

#if TX_KEY_R_DEBUG_ONLY
      /* 右键本地调试模式：仅做按键识别与 LED 反馈，不发送业务包 */
      TxKeyR_DebugLedTask(now);
#else
      if ((g_tx_boot_warm_done == 0U) && ((int32_t)(now - g_tx_boot_warm_until) >= 0)) {
        g_tx_boot_warm_done = 1U;
        if ((g_pairing_mode == 0U) && (g_tx_need_send == 0U) && (g_tx_burst_left == 0U)) {
          RF_Link_Sleep();
          g_tx_rf_awake = 0U;
          DebugPrint("[RF] TX sleep@boot_end\r\n");
        }
      }

      /* 事件触发发包：空闲睡眠，事件唤醒后短突发发送 */
      if ((g_pairing_mode != 0U) || (g_tx_burst_left > 0U) || (g_tx_need_send != 0U)) {
        uint32_t tx_itv = (g_pairing_mode != 0U) ? LINK_TX_PAIRING_INTERVAL_MS : LINK_TX_BURST_INTERVAL_MS;

        if (g_tx_rf_awake == 0U) {
          RF_Link_ConfigTx(RF_TX_CHANNEL);
          g_tx_rf_awake = 1U;
          g_tx_rf_wakeup_tick = now;
          DebugPrint("[RF] wake->tx\r\n");
        }

        if ((now - g_tx_rf_wakeup_tick) >= RF_TX_WAKEUP_SETTLE_MS) {
          if ((now - g_tx_last_send_tick) >= tx_itv) {
            g_tx_last_send_tick = now;

            {
              uint8_t tx_key = g_tx_last_key;
              uint8_t tx_trig = g_tx_last_trig;

              if (g_pairing_mode != 0U) {
                tx_key = TLIGHT_PAIR_KEY_MARK;
                tx_trig = TLIGHT_PAIR_TRIG_MARK;
              }

              BuildTempLightPacket(RF_TX_Buf, g_tx_mode, g_tx_bright, tx_key, tx_trig);
            }
            if (RF_Link_Send(RF_TX_Buf, RF_PACKET_SIZE) == 0) {
              g_link_tx_ok++;
            } else {
              g_link_tx_fail++;
              DebugPrint("[TX-ERR] send fail\r\n");
            }

            g_tx_need_send = 0U;
            if (g_tx_burst_left > 0U) {
              g_tx_burst_left--;
            }
          }
        }
      } else {
        if (g_tx_rf_awake != 0U) {
          RF_Link_Sleep();
          g_tx_rf_awake = 0U;
          DebugPrint("[RF] sleep\r\n");
        }
      }

      /* LED 定时熄灭（对码期间不熄灭/不改灯效） */
      if ((g_led_state == 1) && (g_pairing_mode == 0U)) {
        if ((now - g_led_on_tick) >= TX_KEY_LED_ON_MS) {
          HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
          g_led_state = 0;
        }
      }
#endif

#elif APP_ROLE_RX_ONLY
#if RX_LOCAL_EFFECT_TEST
      static uint32_t rx_local_mark = 0U;
      static uint8_t rx_local_idx = 0U;
      static const uint8_t rx_local_modes[4] = {4U, 5U, 6U, 7U};

      if ((now - rx_local_mark) >= RX_LOCAL_STEP_MS) {
        rx_local_mark = now;
        g_light_mode = rx_local_modes[rx_local_idx];
        g_light_bright = RX_LOCAL_TEST_BRIGHT;
        rx_local_idx++;
        if (rx_local_idx >= 4U) {
          rx_local_idx = 0U;
        }
        DebugPrint("[RX-LOCAL] mode=");
        DebugPrintDec((uint16_t)g_light_mode);
        DebugPrint(" bright=");
        DebugPrintDec((uint16_t)g_light_bright);
        DebugPrint("\r\n");
      }
#else
      /* RX-only：轮询接收 */
      uint8_t rx_len = 0;
      static uint8_t cur_mode = TLIGHT_MODE_MIN;
      static uint8_t cur_bright = TLIGHT_BRIGHT_30;
      static uint8_t cur_key = TLIGHT_KEY_NONE;
      static uint8_t cur_trig = TLIGHT_TRIG_NONE;
      static uint8_t last_print_mode = 0xFF;
      static uint8_t last_print_bright = 0xFF;
      static uint8_t last_print_key = 0xFF;
      static uint8_t last_print_trig = 0xFF;

      if (RF_Link_PollReceive(RF_RX_Buf, &rx_len) == 1) {
        uint8_t mode = 0;
        uint8_t bright = 0;

        /* 正式对码流程：进入对码模式后，仅接受“对码专用标记包”触发配对。 */
        if (g_pairing_mode != 0U) {
          if ((RF_RX_Buf[0] == TLIGHT_PKT_HEADER0) &&
              (RF_RX_Buf[1] == TLIGHT_PKT_HEADER1) &&
              (RF_RX_Buf[TLIGHT_PKT_TAG_IDX] == TLIGHT_PKT_TAG_CONST) &&
              (RF_RX_Buf[TLIGHT_PKT_CHKSUM_IDX] == TempLight_CalcChecksum(RF_RX_Buf)) &&
              (RF_RX_Buf[2] >= TLIGHT_MODE_OFF) && (RF_RX_Buf[2] <= TLIGHT_MODE_MAX) &&
              (RF_RX_Buf[3] <= TLIGHT_BRIGHT_100)) {
            uint8_t key_raw = (uint8_t)((RF_RX_Buf[TLIGHT_PKT_KEYTRIG_IDX] >> 6) & 0x03U);
            uint8_t trig_raw = (uint8_t)((RF_RX_Buf[TLIGHT_PKT_KEYTRIG_IDX] >> 4) & 0x03U);

            if ((key_raw == TLIGHT_PAIR_KEY_MARK) && (trig_raw == TLIGHT_PAIR_TRIG_MARK)) {
              uint16_t new_pair_id = (uint16_t)((RF_RX_Buf[TLIGHT_PKT_DEV_ID_H_IDX] << 8) | RF_RX_Buf[TLIGHT_PKT_DEV_ID_L_IDX]);
              if (new_pair_id != TLIGHT_DEVICE_ID_INVALID) {
                /* 窗口内收到“当前已配对遥控器”的对码标记包：退出窗口，回归正常业务。 */
                if (new_pair_id == g_dev_id_paired) {
                  g_pairing_mode = 0U;
                  g_pairing_enter_tick = 0U;
                  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
                  DebugPrint("[PAIR] matched known id, resume normal\r\n");
                } else if (DevId_SaveToFlash(g_dev_id_local, new_pair_id)) {
                  g_pairing_mode = 0U;
                  g_pairing_enter_tick = 0U;
                  DebugPrint("[PAIR] success id=");
                  DebugPrintHex((const uint8_t *)&new_pair_id, 2);
                  DebugPrint("\r\n");
                  PairingLed_Feedback(1U);
                  g_rx_pair_fb_state = RX_PAIR_FB_SUCCESS_HOLD;
                  g_rx_pair_fb_tick = now;
                  g_rx_pair_fb_step = 0U;
                } else {
                  DebugPrint("[PAIR-ERR] flash save fail\r\n");
                  PairingLed_Feedback(0U);
                  g_rx_pair_fb_state = RX_PAIR_FB_FAIL_FLASH;
                  g_rx_pair_fb_tick = now;
                  g_rx_pair_fb_step = 0U;
                }
              }
            }
          }
        }

        if (ParseTempLightPacket(RF_RX_Buf, &mode, &bright, &cur_key, &cur_trig)) {
          uint8_t seq = (uint8_t)(RF_RX_Buf[TLIGHT_PKT_KEYTRIG_IDX] & 0x0FU);

          /* 中文说明：
           * RX 收到合法遥控包后，先更新链路统计；
           * 真正是否立即改灯态，要受“ACC/喇叭强制态”仲裁约束。 */

          g_link_rx_ok++;
          g_link_last_rx_tick = now;
          g_link_timeout_flag = 0;

          if (g_link_has_last_seq) {
            uint8_t delta = (uint8_t)((seq - g_link_last_rx_seq) & 0x0FU);
            if (delta == 0U) {
              g_link_rx_dup++;
            } else if (delta > 1U) {
              g_link_rx_jump++;
              DebugPrint("[RX-WARN] seq_jump prev=");
              DebugPrintDec((uint16_t)g_link_last_rx_seq);
              DebugPrint(" now=");
              DebugPrintDec((uint16_t)seq);
              DebugPrint("\r\n");
            }
          }
          g_link_last_rx_seq = seq;
          g_link_has_last_seq = 1;

          cur_mode = mode;
          cur_bright = bright;

          /* P4 遥控命令融合规则：
           * - ACC=0：忽略遥控输出变更；
           * - HORN激活：缓存为pending，待喇叭释放后优先恢复；
           * - 对码反馈独占窗口：仅统计链路，不立刻改灯态；
           * - 其余情况：立即应用遥控灯态。 */
          if (g_rx_pair_fb_state != RX_PAIR_FB_NONE) {
            /* 对码反馈期间：冻结业务灯态应用，避免覆盖成功/失败反馈。 */
          } else if (g_rx_acc_last == 0U) {
            /* ACC 关闭，遥控不改当前输出 */
          } else if (g_rx_horn_active != 0U) {
            g_rx_remote_pending_valid = 1U;
            g_rx_remote_pending_mode = cur_mode;
            g_rx_remote_pending_bright = cur_bright;
          } else {
            g_light_mode = cur_mode;
            g_light_bright = cur_bright;
          }

          if(cur_mode != last_print_mode || cur_bright != last_print_bright || cur_key != last_print_key || cur_trig != last_print_trig) {
            DebugPrint("[RX] mode=");
            DebugPrintDec((uint16_t)cur_mode);
            DebugPrint(" bright=");
            DebugPrintDec((uint16_t)cur_bright);
            DebugPrint(" key=");
            DebugPrintDec((uint16_t)cur_key);
            DebugPrint(" trig=");
            DebugPrintDec((uint16_t)cur_trig);
            DebugPrint(" seq=");
            DebugPrintDec((uint16_t)seq);
            DebugPrint(" (30/70/100)\r\n");
            last_print_mode = cur_mode;
            last_print_bright = cur_bright;
            last_print_key = cur_key;
            last_print_trig = cur_trig;
          }
        } else {
          if ((RF_RX_Buf[0] == TLIGHT_PKT_HEADER0) &&
              (RF_RX_Buf[1] == TLIGHT_PKT_HEADER1) &&
              (RF_RX_Buf[TLIGHT_PKT_TAG_IDX] == TLIGHT_PKT_TAG_CONST) &&
              (RF_RX_Buf[TLIGHT_PKT_CHKSUM_IDX] == TempLight_CalcChecksum(RF_RX_Buf)) &&
              (((uint16_t)((RF_RX_Buf[TLIGHT_PKT_DEV_ID_H_IDX] << 8) | RF_RX_Buf[TLIGHT_PKT_DEV_ID_L_IDX])) != g_dev_id_paired)) {
            g_link_rx_dev_mismatch++;
          } else {
            uint8_t hdr_ok = (uint8_t)((RF_RX_Buf[0] == TLIGHT_PKT_HEADER0) && (RF_RX_Buf[1] == TLIGHT_PKT_HEADER1));
            uint8_t mode_ok = (uint8_t)((RF_RX_Buf[2] >= TLIGHT_MODE_OFF) && (RF_RX_Buf[2] <= TLIGHT_MODE_MAX));
            uint8_t bright_ok = (uint8_t)(RF_RX_Buf[3] <= TLIGHT_BRIGHT_100);
            uint8_t key_ok = (uint8_t)(((RF_RX_Buf[TLIGHT_PKT_KEYTRIG_IDX] >> 6) & 0x03U) <= TLIGHT_KEY_NONE);
            uint8_t trig_ok = (uint8_t)(((RF_RX_Buf[TLIGHT_PKT_KEYTRIG_IDX] >> 4) & 0x03U) <= TLIGHT_TRIG_LONG);
            uint8_t tag_ok = (uint8_t)(RF_RX_Buf[TLIGHT_PKT_TAG_IDX] == TLIGHT_PKT_TAG_CONST);
            uint8_t chk_ok = (uint8_t)(RF_RX_Buf[TLIGHT_PKT_CHKSUM_IDX] == TempLight_CalcChecksum(RF_RX_Buf));

            g_link_rx_parse_err++;
            DebugPrint("[RX-ERR] bad packet h/m/b/k/t/g/c=");
            DebugPrintDec((uint16_t)hdr_ok);
            DebugPrint("/");
            DebugPrintDec((uint16_t)mode_ok);
            DebugPrint("/");
            DebugPrintDec((uint16_t)bright_ok);
            DebugPrint("/");
            DebugPrintDec((uint16_t)key_ok);
            DebugPrint("/");
            DebugPrintDec((uint16_t)trig_ok);
            DebugPrint("/");
            DebugPrintDec((uint16_t)tag_ok);
            DebugPrint("/");
            DebugPrintDec((uint16_t)chk_ok);
            DebugPrint("\r\n");
          }
        }
      }

      if ((g_link_last_rx_tick != 0U) && ((now - g_link_last_rx_tick) >= LINK_RX_TIMEOUT_MS) && (g_link_timeout_flag == 0U)) {
        g_link_timeout_flag = 1;
        DebugPrint("[RX-WARN] timeout > ");
        DebugPrintDec((uint16_t)LINK_RX_TIMEOUT_MS);
        DebugPrint("ms\r\n");
      }
#endif

      /* RX 三线输入业务（ACC/原车远光/喇叭） */
      RxWireInput_Poll(now);

      /* 驱动两路 PWM 输出（PA2=CH3, PA3=CH4） */
      Light_Tick(now);
#endif

#if APP_ROLE_TX_ONLY && TX_KEY_R_DEBUG_ONLY
      /* 右键本地调试模式：避免RF统计刷屏 */
#else
      /* 周期统计，避免日志刷屏 */
      LinkStats_Log(now);
#endif

#if APP_ROLE_TX_ONLY
    /* TX 角色：当业务空闲时进入低功耗，不再依赖历史太阳能“昼夜判定”逻辑 */
    {
      uint32_t now = HAL_GetTick();
      uint8_t tx_busy = 0U;

      if ((g_pairing_mode != 0U) || (g_tx_need_send != 0U) || (g_tx_burst_left > 0U)) {
        tx_busy = 1U;
      }

      /* 后台节流保存任务：活跃期间择机提交状态，避免每次按键直接写Flash。 */
      TxState_SaveWorker(now);
      if ((g_tx_rf_awake != 0U) && ((now - g_tx_last_send_tick) < LINK_TX_BURST_INTERVAL_MS)) {
        tx_busy = 1U;
      }
      /* 三键唤醒重映射后，PA2/PB6 在硬件端无内部上下拉：
       * 这里软件层继续使用唤醒保护窗口，避免唤醒后过快回睡。 */
      if ((g_tx_lp_wake_tick != 0U) && ((now - (g_tx_lp_wake_tick - 1U)) < TX_LP_WAKE_GUARD_MS)) {
        tx_busy = 1U;
      }
      if ((int32_t)(g_tx_post_event_hold_until_tick - now) > 0) {
        tx_busy = 1U;
      }

      {
        uint8_t l_now = TxKey_ReadRaw(L_KEY_Pin, L_KEY_GPIO_Port);
        uint8_t m_now = TxKey_ReadRaw(M_KEY_Pin, M_KEY_GPIO_Port);
        uint8_t r_now = TxKey_ReadRaw(R_KEY_Pin, R_KEY_GPIO_Port);

        /* 卡键保护：任一键连续按下超过阈值后，强制进低功耗直到全部释放。 */
        if ((l_now != 0U) || (m_now != 0U) || (r_now != 0U)) {
          if (g_tx_anykey_down_tick == 0U) {
            g_tx_anykey_down_tick = now;
          }

          if ((now - g_tx_anykey_down_tick) >= TX_STUCK_KEY_FORCE_SLEEP_MS) {
            if (g_tx_force_sleep_latched == 0U) {
              g_tx_force_sleep_latched = 1U;
              DebugPrint("[TX-LP] force_sleep_by_stuck_key\r\n");
            }
          }
        } else {
          if (g_tx_anykey_down_tick != 0U) {
            /* 全部按键释放边沿：只在这一刻刷新一次活动保持窗口。 */
            g_tx_key_active_until_tick = now + TX_LP_KEY_ACTIVE_HOLD_MS;
          }
          g_tx_anykey_down_tick = 0U;
          if (g_tx_force_sleep_latched != 0U) {
            g_tx_force_sleep_latched = 0U;
            g_tx_lp_wake_tick = now + 1U;
            DebugPrint("[TX-LP] stuck_key_released\r\n");
          }
        }

        if (g_tx_force_sleep_latched == 0U) {
          /* 正常策略：按键活动期间保持活跃，防止漏按。 */
          if ((l_now != 0U) || (m_now != 0U) || (r_now != 0U)) {
            tx_busy = 1U;
            g_tx_key_active_until_tick = now + TX_LP_KEY_ACTIVE_HOLD_MS;
          }

          if ((int32_t)(g_tx_key_active_until_tick - now) > 0) {
            tx_busy = 1U;
          }
        }
      }

      if (tx_busy != 0U) {
        /* 业务繁忙：若刚从低功耗唤醒，先恢复 RF 到 TX，再继续按键/发包 */
        if (g_rf_sleeping != 0U) {
          g_rf_sleeping = 0U;
          if (g_tx_rf_awake == 0U) {
            RF_Link_ConfigTx(RF_TX_CHANNEL);
            g_tx_rf_awake = 1U;
            g_tx_rf_wakeup_tick = now;
#if APP_RF_LINK_TEST
            DebugPrint("[RF] wake->tx (idle_exit)\r\n");
#endif
          }
        }
        HAL_Delay(1);
      } else {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

        /* 进入 STANDBY 前兜底保存：若仍有脏状态，立即提交，避免丢最后一次状态。 */
        if (g_tx_state_dirty != 0U) {
          if (TxState_SaveToFlash() != 0U) {
            g_tx_state_dirty = 0U;
            DebugPrint("[TX-STATE] save commit(pre-standby)\r\n");
          }
        }

        if(!g_rf_sleeping) {
          EnterTxLowPower();
        }

#if (TX_LOWPOWER_MODE == TX_LOWPOWER_MODE_STOP)
        HAL_PWREx_EnableFlashPowerDown(PWR_FLASHPD_STOP);
        HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
        HAL_PWREx_DisableFlashPowerDown(PWR_FLASHPD_STOP);
        ExitTxLowPower();
#elif (TX_LOWPOWER_MODE == TX_LOWPOWER_MODE_STANDBY)
        HAL_PWR_EnterSTANDBYMode();
        /* 理论上不会执行到这里；若异常返回，做一次兜底恢复 */
        ExitTxLowPower();
#else
        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
        ExitTxLowPower();
#endif
      }
    }
#else
    /* RX 角色：保持主循环常规调度 */
    HAL_Delay(1);
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 115;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 23;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOS_L_HI_Pin|RF_CSN_Pin|RF_SCK_Pin|MOS_L_LO_Pin
                          |RF_DATA_Pin|MOS_R_HI_Pin|MOS_R_LO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOS_L_HI_Pin MOS_L_LO_Pin MOS_R_HI_Pin MOS_R_LO_Pin */
  GPIO_InitStruct.Pin = MOS_L_HI_Pin|MOS_L_LO_Pin|MOS_R_HI_Pin|MOS_R_LO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_CSN_Pin RF_SCK_Pin RF_DATA_Pin */
  GPIO_InitStruct.Pin = RF_CSN_Pin|RF_SCK_Pin|RF_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
#if APP_ROLE_RX_ONLY
  /* RX 三线输入（白/黄/蓝）在运行态明确配置为输入，并按极性选择内部上下拉：
   * NORMAL(高有效)  -> 空闲低，使用下拉
   * INVERTED(低有效)-> 空闲高，使用上拉 */
  GPIO_InitStruct.Pin = L_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
#if (RX_WIRE_ACTIVE_POLARITY == RX_WIRE_ACTIVE_POLARITY_INVERTED)
  GPIO_InitStruct.Pull = GPIO_PULLUP;
#else
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
#endif
  HAL_GPIO_Init(L_KEY_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = M_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
#if (RX_WIRE_ACTIVE_POLARITY == RX_WIRE_ACTIVE_POLARITY_INVERTED)
  GPIO_InitStruct.Pull = GPIO_PULLUP;
#else
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
#endif
  HAL_GPIO_Init(M_KEY_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = R_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
#if (RX_WIRE_ACTIVE_POLARITY == RX_WIRE_ACTIVE_POLARITY_INVERTED)
  GPIO_InitStruct.Pull = GPIO_PULLUP;
#else
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
#endif
  HAL_GPIO_Init(R_KEY_GPIO_Port, &GPIO_InitStruct);
#endif

#if APP_ROLE_TX_ONLY
  /* TX 右键输入在运行态明确配置为中断输入（双边沿）：
   * - 唤醒仍由 WKUP4 负责；
   * - 运行态通过 EXTI 下降沿快速捕获 WKUP4+OFF 场景第二击，减少边界漏判。 */
  GPIO_InitStruct.Pin = R_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(R_KEY_GPIO_Port, &GPIO_InitStruct);
#endif

  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* ===== 双路 PWM 灯效输出（RX-only）===== */

static uint16_t Pwm_GetArrPlus1(void)
{
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
  if (arr > 0xFFFFU) arr = 0xFFFFU;
  return (uint16_t)(arr + 1U);
}

static uint16_t Bright_ToPulse(uint8_t bright)
{
  uint16_t arrp1 = Pwm_GetArrPlus1();
  uint32_t pct = 30U;
  if (bright == TLIGHT_BRIGHT_70) pct = 70U;
  else if (bright == TLIGHT_BRIGHT_100) pct = 100U;
  return (uint16_t)((arrp1 * pct) / 100U);
}

/* 四路灯通道位图定义（仅用于 MOS 导通选择） */
#define CH_L_HI_MASK  (1U << 0)
#define CH_L_LO_MASK  (1U << 1)
#define CH_R_HI_MASK  (1U << 2)
#define CH_R_LO_MASK  (1U << 3)

/* 设置全局亮度 PWM（PA1/TIM1_CH2）：
 * - 正常模式：按 pulse 输出占空比
 * - 调试模式：可强制 0% 或 100% 验证 MOS 通道逻辑 */
static void PwmGlobal_Set(uint16_t pulse)
{
  uint16_t arrp1 = Pwm_GetArrPlus1();
  if (pulse > arrp1) pulse = arrp1;

#if (PWM_GLOBAL_FORCE_LEVEL == 1U)
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
#elif (PWM_GLOBAL_FORCE_LEVEL == 2U)
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, arrp1);
#else
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse);
#endif
}

/* 应用四路 MOS 通道掩码：
 * - 1 表示导通该通道
 * - 0 表示关断该通道
 * 说明：MOS 仅用于通道选择，不做亮度调制。 */
static void Channel_ApplyMask(uint8_t mask)
{
  HAL_GPIO_WritePin(MOS_L_HI_GPIO_Port, MOS_L_HI_Pin, (mask & CH_L_HI_MASK) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOS_L_LO_GPIO_Port, MOS_L_LO_Pin, (mask & CH_L_LO_MASK) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOS_R_HI_GPIO_Port, MOS_R_HI_Pin, (mask & CH_R_HI_MASK) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOS_R_LO_GPIO_Port, MOS_R_LO_Pin, (mask & CH_R_LO_MASK) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void PairingLed_Update(uint32_t now)
{
  /* 对码反馈优先级最高，且必须非阻塞，避免影响按键扫描 */
  if (g_pair_fb_state == PAIR_FB_SUCCESS) {
    g_led_state = 0;
    if ((now - g_pair_fb_tick) >= 120U) {
      g_pair_fb_tick = now;
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      g_pair_fb_step++;
      if (g_pair_fb_step >= 6U) {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        g_pair_fb_state = PAIR_FB_NONE;
        g_pair_fb_step = 0U;
      }
    }
    return;
  }

  if (g_pair_fb_state == PAIR_FB_FAIL) {
    g_led_state = 0;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    if ((now - g_pair_fb_tick) >= 1000U) {
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      g_pair_fb_state = PAIR_FB_NONE;
    }
    return;
  }

  if (g_pairing_mode == 0U) {
    return;
  }

  /* 对码窗口期间：LED 由对码逻辑独占，普通短亮提示不允许抢占 */
  g_led_state = 0;

  if ((now - g_pair_led_tick) >= 200U) {
    g_pair_led_tick = now;
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }
}

static void PairingLed_Feedback(uint8_t success)
{
  /* 反馈期间关闭普通短亮状态，防止抢占 */
  g_led_state = 0;
  g_pair_fb_tick = HAL_GetTick();
  g_pair_fb_step = 0U;

  if (success != 0U) {
    g_pair_fb_state = PAIR_FB_SUCCESS;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  } else {
    g_pair_fb_state = PAIR_FB_FAIL;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  }
}

/* RX 三线输入处理（冲突优先级状态表）
 * P0: ACC=0 -> 强制OFF（最高优先级）
 * P1: ACC=1 且 HORN=1 -> 强制爆闪（模式4）
 * P2: HORN 下降沿释放 -> 回到黄线状态（HI=1远光/HI=0近光）
 * P3: HORN=0 时，黄线边沿控制近/远光（上升=远光，下降=近光）
 *     其中“黄线首次生效”需先在窗口内出现两次边沿（双触发）后才解锁；
 *     解锁后按边沿正常生效。该规则兼容正/负逻辑极性配置。
 * P4: 遥控命令仅在未触发 P0/P1 强制态时生效（已落地）。 */
static void RxWireInput_Poll(uint32_t now)
{
#if APP_ROLE_RX_ONLY
  uint8_t acc_raw  = (HAL_GPIO_ReadPin(RX_ACC_IN_PORT, RX_ACC_IN_PIN) == RX_IN_ACTIVE_LEVEL) ? 1U : 0U;
  uint8_t hi_raw   = (HAL_GPIO_ReadPin(RX_CAR_HI_IN_PORT, RX_CAR_HI_IN_PIN) == RX_IN_ACTIVE_LEVEL) ? 1U : 0U;
  uint8_t horn_raw = (HAL_GPIO_ReadPin(RX_HORN_IN_PORT, RX_HORN_IN_PIN) == RX_IN_ACTIVE_LEVEL) ? 1U : 0U;
  uint8_t acc = g_rx_acc_last;
  uint8_t hi = g_rx_hi_last;
  uint8_t horn = g_rx_horn_last;

  if (acc_raw != g_rx_acc_raw_last) {
    g_rx_acc_raw_last = acc_raw;
    g_rx_acc_raw_change_tick = now;
  }
  if ((now - g_rx_acc_raw_change_tick) >= RX_WIRE_DEBOUNCE_MS) {
    acc = acc_raw;
  }

  if (hi_raw != g_rx_hi_raw_last) {
    g_rx_hi_raw_last = hi_raw;
    g_rx_hi_raw_change_tick = now;
  }
  if ((now - g_rx_hi_raw_change_tick) >= RX_WIRE_DEBOUNCE_MS) {
    hi = hi_raw;
  }

  if (horn_raw != g_rx_horn_raw_last) {
    g_rx_horn_raw_last = horn_raw;
    g_rx_horn_raw_change_tick = now;
  }
  if ((now - g_rx_horn_raw_change_tick) >= RX_WIRE_DEBOUNCE_MS) {
    horn = horn_raw;
  }

  if (acc == 0U) {
    g_rx_horn_active = 0U;
    g_light_mode = TLIGHT_MODE_OFF;

    /* ACC 关闭时，重置黄线首次双触发门限；下次ACC有效后重新判定首次双触发 */
    g_rx_hi_gate_unlocked = 0U;
    g_rx_hi_edge_cnt = 0U;
    g_rx_hi_first_edge_tick = 0U;

    g_rx_acc_last = acc;
    g_rx_hi_last = hi;
    g_rx_horn_last = horn;
    return;
  }

  if ((horn != 0U) && (g_rx_horn_last == 0U)) {
    /* 喇叭上升沿：先快照当前状态，再进入爆闪覆盖态 */
    g_rx_pre_horn_mode = g_light_mode;
    g_rx_pre_horn_bright = g_light_bright;
    g_rx_remote_pending_valid = 0U;

    g_rx_horn_active = 1U;
    g_light_mode = 4U; /* 仅作状态兼容标记；实际爆闪由 HORN 独立 50/50ms 逻辑驱动 */
    g_light_bright = TLIGHT_BRIGHT_100;
  }
  if ((horn == 0U) && (g_rx_horn_last != 0U)) {
    g_rx_horn_active = 0U;

    /* 喇叭释放优先级：
     * 1) HORN期间若有遥控pending，优先应用pending
     * 2) 否则恢复到HORN触发前快照状态 */
    if (g_rx_remote_pending_valid != 0U) {
      g_light_mode = g_rx_remote_pending_mode;
      g_light_bright = g_rx_remote_pending_bright;
      g_rx_remote_pending_valid = 0U;
    } else {
      g_light_mode = g_rx_pre_horn_mode;
      g_light_bright = g_rx_pre_horn_bright;
    }
  }

  if (g_rx_horn_active == 0U) {
    uint8_t hi_edge = (uint8_t)((hi != g_rx_hi_last) ? 1U : 0U);

    if (g_rx_hi_gate_unlocked == 0U) {
      if (hi_edge != 0U) {
        if (g_rx_hi_edge_cnt == 0U) {
          g_rx_hi_edge_cnt = 1U;
          g_rx_hi_first_edge_tick = now;
        } else {
          if ((now - g_rx_hi_first_edge_tick) <= RX_HI_FIRST_DBL_MS) {
            g_rx_hi_gate_unlocked = 1U;
            g_rx_hi_edge_cnt = 0U;
            g_rx_hi_first_edge_tick = 0U;
          } else {
            /* 超窗：以当前边沿作为新的第一次 */
            g_rx_hi_edge_cnt = 1U;
            g_rx_hi_first_edge_tick = now;
          }
        }
      }
    }

    if (g_rx_hi_gate_unlocked != 0U) {
      if ((hi != 0U) && (g_rx_hi_last == 0U)) {
        g_light_mode = 2U; /* 黄线上升沿：远光 */
      }
      if ((hi == 0U) && (g_rx_hi_last != 0U)) {
        g_light_mode = 1U; /* 黄线下降沿：近光 */
      }
    }
  }

  g_rx_acc_last = acc;
  g_rx_hi_last = hi;
  g_rx_horn_last = horn;
#else
  (void)now;
#endif
}

static uint8_t TxKey_ReadRaw(uint16_t pin, GPIO_TypeDef *port)
{
  return (HAL_GPIO_ReadPin(port, pin) == KEY_ACTIVE_LEVEL) ? 1U : 0U;
}

static uint8_t TxKey_Update(TxKeyState_t *ks, uint8_t raw, uint32_t now, uint8_t tick_1ms, uint8_t *evt_single, uint8_t *evt_double, uint8_t *evt_long)
{
  uint8_t changed = 0U;
  uint32_t debounce_ms = KEY_DEBOUNCE_MS;
  *evt_single = 0U;
  *evt_double = 0U;
  *evt_long = 0U;

  /* WKUP4+OFF 专用：在扩展双击窗口激活期间，将右键防抖缩短到 5ms，
   * 提前确认 second-press 的稳定按下，减少“按得很快被判 single”概率。 */
  if ((ks == &g_txk_r) && (g_tx_r_wkup_ext_dbl_active != 0U)) {
    debounce_ms = KEY_DEBOUNCE_MS_WKUP4_OFF;
  }

  /* 对齐参考工程：按键识别仅在 1ms 节拍中推进，避免主循环阻塞造成采样漂移 */
  if (tick_1ms == 0U) {
    return 0U;
  }

  if (raw != ks->last_raw) {
    ks->last_raw = raw;
    ks->raw_change_tick = now;
  }

  if ((now - ks->raw_change_tick) >= debounce_ms) {
    if (ks->stable_level != raw) {
      ks->stable_level = raw;
      changed = 1U;

      if (ks->stable_level != 0U) {
        /* 稳定按下 */
        ks->press_start_tick = now;
        ks->long_reported = 0U;

#if KEY_TX_DOUBLE_ENABLE
        if (ks->click_pending != 0U) {
          uint32_t gap = now - ks->first_click_tick;
          if (gap <= (ks == &g_txk_r && g_tx_r_wkup_ext_dbl_active ? KEY_TX_DBL_MS_WKUP4_OFF : KEY_TX_DBL_MS)) {
            *evt_double = 1U;
            ks->click_pending = 0U;
            ks->suppress_release_after_double = 1U;
            if ((ks == &g_txk_r) && (g_tx_r_wkup_ext_dbl_active != 0U)) {
              DebugPrint("[LP-WKUP] R ext dbl hit gap_ms=");
              DebugPrintDec((uint16_t)(gap & 0xFFFFU));
              DebugPrint(" win_ms=");
              DebugPrintDec((uint16_t)KEY_TX_DBL_MS_WKUP4_OFF);
              DebugPrint("\r\n");
              g_tx_r_wkup_ext_dbl_active = 0U;
              g_tx_r_wkup_fast_edge_armed = 0U;
              g_tx_r_wkup_fast_edge_hit = 0U;
            }
          } else {
            /* 第二次按下超窗：上一击补发单击，本次按下作为新一轮 */
            *evt_single = 1U;
            ks->click_pending = 0U;
          }
        }
#endif
      } else {
        /* 稳定松开 */
        uint32_t press_dur = 0U;
        if (ks->press_start_tick != 0U) {
          press_dur = now - ks->press_start_tick;
        }
        ks->last_press_duration_ms = press_dur;

        if (ks->suppress_release_after_double != 0U) {
          ks->suppress_release_after_double = 0U;
          ks->press_start_tick = 0U;
          return changed;
        }

        if (ks->long_reported == 0U) {
#if KEY_TX_DOUBLE_ENABLE
          ks->click_pending = 1U;
          ks->first_click_tick = now;
#else
          *evt_single = 1U;
#endif
        }

        ks->press_start_tick = 0U;
      }
    }
  }

  if ((ks->stable_level != 0U) && (ks->long_reported == 0U) && (ks->press_start_tick != 0U)) {
    if ((now - ks->press_start_tick) >= KEY_TX_LONG_MS) {
      ks->long_reported = 1U;
      ks->click_pending = 0U;
      ks->suppress_release_after_double = 0U;
      *evt_long = 1U;
    }
  }

#if KEY_TX_DOUBLE_ENABLE
  if (ks == &g_txk_r) {
    if ((ks->click_pending != 0U) && ((now - ks->first_click_tick) > (g_tx_r_wkup_ext_dbl_active ? KEY_TX_DBL_MS_WKUP4_OFF : KEY_TX_DBL_MS))) {
      ks->click_pending = 0U;
      *evt_single = 1U;
      if (g_tx_r_wkup_ext_dbl_active != 0U) {
        DebugPrint("[LP-WKUP] R ext dbl timeout_ms=");
        DebugPrintDec((uint16_t)KEY_TX_DBL_MS_WKUP4_OFF);
        DebugPrint(" gap_ms=");
        DebugPrintDec((uint16_t)((now - ks->first_click_tick) & 0xFFFFU));
        DebugPrint(" rise_seen=");
        DebugPrintDec(g_tx_r_wkup_exti_rise_seen);
        DebugPrint(" rise_gap=");
        if (g_tx_r_wkup_exti_rise_seen != 0U) {
          DebugPrintDec((uint16_t)((g_tx_r_wkup_exti_rise_tick - ks->first_click_tick) & 0xFFFFU));
        } else {
          DebugPrint("NA");
        }
        DebugPrint(" fall_seen=");
        DebugPrintDec(g_tx_r_wkup_exti_fall_seen);
        DebugPrint(" fall_gap=");
        if (g_tx_r_wkup_exti_fall_seen != 0U) {
          DebugPrintDec((uint16_t)((g_tx_r_wkup_exti_fall_tick - ks->first_click_tick) & 0xFFFFU));
        } else {
          DebugPrint("NA");
        }
        DebugPrint("\r\n");
        g_tx_r_wkup_ext_dbl_active = 0U;
        g_tx_r_wkup_fast_edge_armed = 0U;
        g_tx_r_wkup_fast_edge_hit = 0U;
      }
    }
  } else {
    if ((ks->click_pending != 0U) && ((now - ks->first_click_tick) > KEY_TX_DBL_MS)) {
      ks->click_pending = 0U;
      *evt_single = 1U;
    }
  }
#endif

  return changed;
}

static uint16_t GetPhase1ms(uint16_t period_ms)
{
  uint16_t phase = g_phase_1ms;
  if (period_ms == 0U) return 0U;
  return (uint16_t)(phase % period_ms);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM14) {
    g_phase_1ms++;
  }
}

static void Tx_ScheduleBurst(void)
{
  g_tx_need_send = 1U;
  if (g_tx_first_event_done == 0U) {
    g_tx_burst_left = LINK_TX_BOOT_BURST_COUNT;
    g_tx_first_event_done = 1U;
  } else {
    g_tx_burst_left = LINK_TX_EVENT_BURST_COUNT;
  }
}

static void Tx_DoOff(const char *key_name, const char *trig)
{
  g_tx_is_off = 1U;
  g_tx_in_effect = 0U;
  g_tx_mode = 0U;
  g_tx_left_all_on_armed = 0U;

  DebugPrint("[TX-OFF] key=");
  DebugPrint(key_name);
  DebugPrint(" trig=");
  DebugPrint(trig);
  DebugPrint("\r\n");
}

static void Tx_OnFromOff_BySingle(uint8_t key)
{
  g_tx_is_off = 0U;
  g_tx_last_key = key;
  g_tx_last_trig = TLIGHT_TRIG_SINGLE;

  if (key == TLIGHT_KEY_R) { /* 右键 */
    g_tx_in_effect = 1U;
    g_tx_effect_mode = 4U;
    g_tx_mode = g_tx_effect_mode;
    DebugPrint("[TX-ON] key=R trig=single to=effect1\r\n");
  } else {
    g_tx_in_effect = 0U;
    g_tx_basic_mode = 1U;
    g_tx_mode = g_tx_basic_mode;
    if (key == TLIGHT_KEY_L) {
      DebugPrint("[TX-ON] key=L trig=single to=basic1\r\n");
    } else {
      DebugPrint("[TX-ON] key=M trig=single to=basic1\r\n");
    }
  }
}

static void Tx_HandleSingle(uint8_t key)
{
  if (g_tx_is_off != 0U) {
    Tx_OnFromOff_BySingle(key);
    return;
  }

  g_tx_last_key = key;
  g_tx_last_trig = TLIGHT_TRIG_SINGLE;

  if (key == TLIGHT_KEY_L) { /* 左键 */
    if (g_tx_in_effect != 0U) {
      g_tx_in_effect = 0U;
      g_tx_basic_mode = 1U;
      g_tx_mode = g_tx_basic_mode;
    } else {
      g_tx_basic_mode++;
      if (g_tx_basic_mode > 2U) g_tx_basic_mode = 1U;
      g_tx_mode = g_tx_basic_mode;
    }
  } else if (key == TLIGHT_KEY_M) { /* 中键 */
    g_tx_bright++;
    if (g_tx_bright > TLIGHT_BRIGHT_100) g_tx_bright = TLIGHT_BRIGHT_30;
  } else { /* 右键 */
    g_tx_in_effect = 1U;
    g_tx_effect_mode++;
    if (g_tx_effect_mode > 7U) g_tx_effect_mode = 4U;
    g_tx_mode = g_tx_effect_mode;
  }
}

/* TX 双击/长按入口：
 * - 左键：仅做“近光(1) <-> 同亮(3)”循环，不熄灯（OFF时可拉起到basic1）
 * - 右键：用于熄灯；全灭状态下再次触发无效
 * - 中键：双击/长按等效单击（不熄灯） */
static void Tx_HandleOffTrigger(uint8_t key, const char *trig)
{
  uint8_t trig_code = TLIGHT_TRIG_NONE;

  if ((trig[0] == 'd') || (trig[0] == 'D')) {
    trig_code = TLIGHT_TRIG_DOUBLE;
  } else {
    trig_code = TLIGHT_TRIG_LONG;
  }

  /* 左键：双击/长按只做 近光 <-> 同亮 循环，不参与熄灯 */
  if (key == TLIGHT_KEY_L) {
    if (g_tx_is_off != 0U) {
      g_tx_is_off = 0U;
      g_tx_in_effect = 0U;
      g_tx_basic_mode = 1U;
      g_tx_mode = 1U;
      g_tx_last_key = TLIGHT_KEY_L;
      g_tx_last_trig = trig_code;
      DebugPrint("[TX-KEY] key=L trig=");
      DebugPrint(trig);
      DebugPrint(" action=off_to_basic1\r\n");
      return;
    }

    g_tx_in_effect = 0U;
    if (g_tx_mode == 3U) {
      g_tx_basic_mode = 1U;
      g_tx_mode = 1U;
      DebugPrint("[TX-KEY] key=L trig=");
      DebugPrint(trig);
      DebugPrint(" action=all_on_to_basic1\r\n");
    } else {
      g_tx_basic_mode = 3U;
      g_tx_mode = 3U;
      DebugPrint("[TX-KEY] key=L trig=");
      DebugPrint(trig);
      DebugPrint(" action=basic1_to_all_on\r\n");
    }
    g_tx_last_key = TLIGHT_KEY_L;
    g_tx_last_trig = trig_code;
    return;
  }

  /* 中键：双击/长按等效单击（亮度循环，不熄灯） */
  if (key == TLIGHT_KEY_M) {
    Tx_HandleSingle(TLIGHT_KEY_M);
    DebugPrint("[TX-KEY] key=M trig=");
    DebugPrint(trig);
    DebugPrint(" action=as_single\r\n");
    return;
  }

  /* 右键：双击/长按=全灭；但全灭状态下再次触发无效 */
  if (key == TLIGHT_KEY_R) {
    if (g_tx_is_off != 0U) {
      DebugPrint("[TX-KEY] key=R trig=");
      DebugPrint(trig);
      DebugPrint(" action=off_ignore\r\n");
      return;
    }

    g_tx_last_key = key;
    g_tx_last_trig = trig_code;
    Tx_DoOff("R", trig);
    return;
  }
}

static void TxKeys_Process(uint32_t now, uint8_t tick_1ms)
{
#if APP_ROLE_TX_ONLY
  uint8_t changed = 0U;
  uint8_t s = 0U, d = 0U, l = 0U;
  uint8_t l_raw_now = (HAL_GPIO_ReadPin(L_KEY_GPIO_Port, L_KEY_Pin) == KEY_ACTIVE_LEVEL) ? 1U : 0U;
  uint8_t r_raw_now = (HAL_GPIO_ReadPin(R_KEY_GPIO_Port, R_KEY_Pin) == KEY_ACTIVE_LEVEL) ? 1U : 0U;

  /* L+R 组合键对码候选/对码中：屏蔽单键业务识别，避免先触发业务再进入对码。
   * 仅在“当前物理上仍同时按下且组合窗口有效”时屏蔽，避免组合键抬起后因状态滞留导致业务假死。 */
  {
    uint8_t guard_on = 0U;
    if (g_pairing_mode != 0U) {
      guard_on = 1U;
    } else if ((l_raw_now != 0U) && (r_raw_now != 0U) && (g_pair_combo_armed != 0U) && (g_pair_long_done == 0U)) {
      guard_on = 1U;
    }

    if (guard_on != g_pair_guard_log_state) {
      g_pair_guard_log_state = guard_on;
      if (guard_on != 0U) {
        DebugPrint("[PAIR-GUARD] on\r\n");
      } else {
        DebugPrint("[PAIR-GUARD] off\r\n");
      }
    }

    if (guard_on != 0U) {
      return;
    }
  }

#if TX_KEY_R_DEBUG_ONLY
  (void)TxKey_Update(&g_txk_r, TxKey_ReadRaw(R_KEY_Pin, R_KEY_GPIO_Port), now, tick_1ms, &s, &d, &l);
  if (s) {
    g_rdbg_evt_single = 1U;
    g_rdbg_single_cnt++;
    DebugPrint("[RDBG] single cnt=");
    DebugPrintDec((uint16_t)(g_rdbg_single_cnt & 0xFFFFU));
    DebugPrint(" dur=");
    DebugPrintDec((uint16_t)g_txk_r.last_press_duration_ms);
    DebugPrint("ms\r\n");
  }
  if (d) {
    g_rdbg_evt_double = 1U;
    g_rdbg_double_cnt++;
    DebugPrint("[RDBG] double cnt=");
    DebugPrintDec((uint16_t)(g_rdbg_double_cnt & 0xFFFFU));
    DebugPrint("\r\n");
  }
  if (l) {
    g_rdbg_evt_long = 1U;
    g_rdbg_long_cnt++;
    DebugPrint("[RDBG] long cnt=");
    DebugPrintDec((uint16_t)(g_rdbg_long_cnt & 0xFFFFU));
    DebugPrint(" hold>=");
    DebugPrintDec((uint16_t)KEY_TX_LONG_MS);
    DebugPrint("ms\r\n");
  }
#else
  (void)TxKey_Update(&g_txk_l, TxKey_ReadRaw(L_KEY_Pin, L_KEY_GPIO_Port), now, tick_1ms, &s, &d, &l);
  if (s) { g_tx_evt_l = 1U; }
  if (d) { Tx_HandleOffTrigger(0U, "double"); changed = 1U; }
  if (l) { Tx_HandleOffTrigger(0U, "long"); changed = 1U; }

  (void)TxKey_Update(&g_txk_m, TxKey_ReadRaw(M_KEY_Pin, M_KEY_GPIO_Port), now, tick_1ms, &s, &d, &l);
  if (s) { g_tx_evt_m = 1U; }
  if (d) { Tx_HandleOffTrigger(1U, "double"); changed = 1U; }
  if (l) { Tx_HandleOffTrigger(1U, "long"); changed = 1U; }

  (void)TxKey_Update(&g_txk_r, TxKey_ReadRaw(R_KEY_Pin, R_KEY_GPIO_Port), now, tick_1ms, &s, &d, &l);
  if (s) { g_tx_evt_r = 1U; }
  if (d) {
    if (g_tx_r_wkup_pending_single != 0U) {
      g_tx_r_wkup_pending_single = 0U;
      g_tx_r_wkup_pending_deadline = 0U;
      DebugPrint("[LP-WKUP] R pending single canceled by double\r\n");
    }
    if (g_tx_r_click1_inject_tick != 0U) {
      DebugPrint("[LP-WKUP] R double gap_ms=");
      DebugPrintDec((uint16_t)((now - g_tx_r_click1_inject_tick) & 0xFFFFU));
      DebugPrint(" (from click#1 inject)\r\n");
    }
    g_tx_r_wkup_off_guard = 0U;
    Tx_HandleOffTrigger(2U, "double");
    changed = 1U;
  }
  if (l) { Tx_HandleOffTrigger(2U, "long"); changed = 1U; }

  if (g_tx_evt_l != 0U) {
    g_tx_evt_l = 0U;
    Tx_HandleSingle(0U);
    changed = 1U;
  }

  if (g_tx_evt_m != 0U) {
    g_tx_evt_m = 0U;
    Tx_HandleSingle(1U);
    changed = 1U;
  }

  if (g_tx_evt_r != 0U) {
    g_tx_evt_r = 0U;
    if (g_tx_r_wkup_off_guard != 0U) {
      DebugPrint("[LP-WKUP] R single swallowed in OFF wake guard\r\n");
    } else {
      Tx_HandleSingle(2U);
      changed = 1U;
    }
  }

  /* 低功耗唤醒补偿（三键，说明书一致）：
   * 仅在对应按键已释放时，将唤醒键作为“双击第1击”注入按键状态机。
   * 这样用户后续第2击即可命中双击，无需额外第3击。 */
  if (g_tx_boot_wkup_pending_key != TLIGHT_KEY_NONE) {
    uint8_t wake_key = g_tx_boot_wkup_pending_key;
    uint8_t key_now = 0U;
    TxKeyState_t *ks = &g_txk_r;

    g_tx_boot_wkup_pending_key = TLIGHT_KEY_NONE;
    if (wake_key == TLIGHT_KEY_L) {
      key_now = TxKey_ReadRaw(L_KEY_Pin, L_KEY_GPIO_Port);
      ks = &g_txk_l;
    } else if (wake_key == TLIGHT_KEY_M) {
      key_now = TxKey_ReadRaw(M_KEY_Pin, M_KEY_GPIO_Port);
      ks = &g_txk_m;
    } else {
      key_now = TxKey_ReadRaw(R_KEY_Pin, R_KEY_GPIO_Port);
      ks = &g_txk_r;
    }

    if ((wake_key == TLIGHT_KEY_R) && (g_tx_r_wkup_off_guard != 0U) && (g_tx_r_wkup_wait_release != 0U)) {
      /* WKUP4+OFF 专用：
       * - 若当前仍按下(key_now=1)，等待释放后再注入 click#1；
       * - 若已释放(key_now=0)，可立即注入 click#1。 */
      if (key_now != 0U) {
        DebugPrint("[LP-WKUP] R hold_keep, wait release for click#1\r\n");
      } else {
        /* 仅做“释放确认”，不重置 first_click_tick：
         * 双击窗口锚点已前移到 t_wkup。 */
        g_txk_r.click_pending = 1U;
        if (g_tx_r_wkup_click1_anchor_tick != 0U) {
          g_txk_r.first_click_tick = g_tx_r_wkup_click1_anchor_tick;
        }
        g_tx_r_click1_inject_tick = now;
        g_tx_r_wkup_ext_dbl_active = 1U;
        g_tx_r_wkup_fast_edge_armed = 1U;
        /* 注意：此处不清空 fast-edge 已捕获标记。
         * 若“超快双击”的 second-press 早于 release-confirm，ISR 可能已置位 hit，
         * 清零会导致后续被误判为超时 single。 */
        g_tx_r_wkup_wait_release = 0U;
        g_tx_r_wkup_off_guard = 0U;
        DebugPrint("[LP-WKUP] R already released, confirm click#1 dt_wkup_ms=");
        DebugPrintDec((uint16_t)((now - g_tx_wkup4_tick) & 0xFFFFU));
        DebugPrint("\r\n");
      }
    } else {
      if (key_now == 0U) {
        ks->click_pending = 1U;
        ks->first_click_tick = now;
        DebugPrint("[LP-WKUP] inject as click#1 key=");
        if (wake_key == TLIGHT_KEY_L) DebugPrint("L");
        else if (wake_key == TLIGHT_KEY_M) DebugPrint("M");
        else DebugPrint("R");
        DebugPrint(" dbl_ms=");
        DebugPrintDec((uint16_t)KEY_TX_DBL_MS);
        DebugPrint("\r\n");
      } else {
        DebugPrint("[LP-WKUP] hold_keep, skip inject\r\n");
      }
    }
  }

  /* WKUP4+OFF：检测到右键首次释放后，仅做“释放确认”；
   * 双击 click#1 锚点已在 t_wkup 建立，不再重置 first_click_tick。 */
  if ((g_tx_r_wkup_off_guard != 0U) && (g_tx_r_wkup_wait_release != 0U)) {
    uint8_t r_now = TxKey_ReadRaw(R_KEY_Pin, R_KEY_GPIO_Port);
    if (r_now == 0U) {
      g_txk_r.click_pending = 1U;
      if (g_tx_r_wkup_click1_anchor_tick != 0U) {
        g_txk_r.first_click_tick = g_tx_r_wkup_click1_anchor_tick;
      }
      g_tx_r_click1_inject_tick = now;
      g_tx_r_wkup_ext_dbl_active = 1U;
      g_tx_r_wkup_fast_edge_armed = 1U;
      /* 注意：此处不清空 fast-edge 已捕获标记。
       * 若“超快双击”的 second-press 早于 release-confirm，ISR 可能已置位 hit，
       * 清零会导致后续被误判为超时 single。 */
      g_tx_r_wkup_wait_release = 0U;
      g_tx_r_wkup_off_guard = 0U;
      DebugPrint("[LP-WKUP] R released, confirm click#1 dt_wkup_ms=");
      DebugPrintDec((uint16_t)((now - g_tx_wkup4_tick) & 0xFFFFU));
      DebugPrint("\r\n");
    }
  }

  /* WKUP4+OFF 专用：若 ISR 已捕获二击下降沿，则直接按双击命中处理，
   * 不等待主循环防抖稳定，降低边界场景下 second hit 漏判概率。 */
  if ((g_tx_r_wkup_ext_dbl_active != 0U) && (g_tx_r_wkup_fast_edge_hit != 0U) && (g_txk_r.click_pending != 0U)) {
    uint32_t edge_tick = g_tx_r_wkup_fast_edge_tick;
    uint32_t gap = edge_tick - g_txk_r.first_click_tick;
    if (gap <= KEY_TX_DBL_MS_WKUP4_OFF) {
      g_tx_r_wkup_fast_edge_hit = 0U;
      g_tx_r_wkup_fast_edge_armed = 0U;
      g_tx_r_wkup_ext_dbl_active = 0U;
      g_txk_r.click_pending = 0U;
      g_txk_r.suppress_release_after_double = 1U;

      DebugPrint("[LP-WKUP] R fast-edge dbl hit gap_ms=");
      DebugPrintDec((uint16_t)(gap & 0xFFFFU));
      DebugPrint(" win_ms=");
      DebugPrintDec((uint16_t)KEY_TX_DBL_MS_WKUP4_OFF);
      DebugPrint("\r\n");

      g_tx_r_wkup_off_guard = 0U;
      Tx_HandleOffTrigger(2U, "double");
      changed = 1U;
    } else {
      /* 超窗后不在此处抢提交，交由现有超时分支统一落 single，避免语义分叉。 */
      g_tx_r_wkup_fast_edge_hit = 0U;
      g_tx_r_wkup_fast_edge_armed = 0U;
    }
  }

  if (changed != 0U) {
    Tx_ScheduleBurst();
    /* 事件后保持一段活跃窗口，避免 STANDBY 场景下“发完立刻回睡”造成 RX 偶发漏收。 */
    g_tx_post_event_hold_until_tick = now + TX_POST_EVENT_HOLD_MS;
    /* 标记状态变更：由后台节流任务统一提交，降低Flash写入频率。 */
    TxState_MarkDirty(now);

    if (g_pairing_mode == 0U) {
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      g_led_state = 1;
      g_led_on_tick = HAL_GetTick();
    }

    DebugPrint("[TX-KEY] mode=");
    DebugPrintDec((uint16_t)g_tx_mode);
    DebugPrint(" bright=");
    DebugPrintDec((uint16_t)g_tx_bright);
    DebugPrint(" key=");
    DebugPrintDec((uint16_t)g_tx_last_key);
    DebugPrint(" trig=");
    DebugPrintDec((uint16_t)g_tx_last_trig);
    DebugPrint(" off=");
    DebugPrintDec((uint16_t)g_tx_is_off);
    DebugPrint(" dev=");
    DebugPrintHex((const uint8_t *)&g_dev_id_local, 2);
    DebugPrint("\r\n");
  }
#endif
#else
  (void)now;
  (void)tick_1ms;
  /* RX 构建保持空实现 */
#endif
}

/* 非阻塞灯效状态机：只依赖 HAL_GetTick() */
static void TxKeyR_DebugLedTask(uint32_t now)
{
#if APP_ROLE_TX_ONLY && TX_KEY_R_DEBUG_ONLY
  if (g_rdbg_evt_double != 0U) {
    g_rdbg_evt_double = 0U;
    g_rdbg_evt_single = 0U;
    g_rdbg_led_state = RDBG_LED_DBL_HOLD;
    g_rdbg_led_mark = now;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    DebugPrint("[RDBG-LED] DOUBLE hold_on\r\n");
    return;
  }

  if (g_rdbg_evt_long != 0U) {
    g_rdbg_evt_long = 0U;
    g_rdbg_evt_single = 0U;
    g_rdbg_led_state = RDBG_LED_LONG_ON;
    g_rdbg_led_mark = now;
    g_rdbg_led_long_remain = 3U;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    DebugPrint("[RDBG-LED] LONG blink3_start\r\n");
    return;
  }

  if (g_rdbg_evt_single != 0U) {
    g_rdbg_evt_single = 0U;
    g_rdbg_led_state = RDBG_LED_SINGLE_ON;
    g_rdbg_led_mark = now;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    DebugPrint("[RDBG-LED] SINGLE blink1_start\r\n");
    return;
  }

  if (g_rdbg_led_state == RDBG_LED_SINGLE_ON) {
    if ((now - g_rdbg_led_mark) >= RDBG_LED_STEP_MS) {
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      g_rdbg_led_state = RDBG_LED_IDLE;
      DebugPrint("[RDBG-LED] SINGLE done\r\n");
    }
    return;
  }

  if (g_rdbg_led_state == RDBG_LED_DBL_HOLD) {
    if ((now - g_rdbg_led_mark) >= RDBG_LED_DBL_HOLD_MS) {
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      g_rdbg_led_state = RDBG_LED_IDLE;
    }
    return;
  }

  if (g_rdbg_led_state == RDBG_LED_LONG_ON) {
    if ((now - g_rdbg_led_mark) >= RDBG_LED_STEP_MS) {
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      g_rdbg_led_mark = now;
      g_rdbg_led_state = RDBG_LED_LONG_OFF;
      /* pulse_off log disabled to reduce UART blocking */
    }
    return;
  }

  if (g_rdbg_led_state == RDBG_LED_LONG_OFF) {
    if ((now - g_rdbg_led_mark) >= RDBG_LED_STEP_MS) {
      if (g_rdbg_led_long_remain > 1U) {
        g_rdbg_led_long_remain--;
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        g_rdbg_led_mark = now;
        g_rdbg_led_state = RDBG_LED_LONG_ON;
        /* pulse_on log disabled to reduce UART blocking */
      } else {
        g_rdbg_led_long_remain = 0U;
        g_rdbg_led_state = RDBG_LED_IDLE;
        /* long done log disabled to reduce UART blocking */
      }
    }
    return;
  }

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
#else
  (void)now;
#endif
}

/* 非阻塞灯效状态机：基于 TIM14 1ms 节拍相位锁定 */
static void Light_Tick(uint32_t now)
{
  const uint16_t base = Bright_ToPulse(g_light_bright);

  /* 对齐说明书参数（统一从宏读取，便于快速调参） */
  const uint16_t fast_ms = FX4_FLASH_HALF_PERIOD_MS;    /* mode4 */
  const uint16_t alt_ms  = FX5_ALT_HALF_PERIOD_MS;      /* mode5 */
  const uint16_t wheel_ms = FX7_WHEEL_HALF_PERIOD_MS;   /* mode7（旧参数保留，不再用于输出节拍） */
  const uint16_t breath_ms = FX6_BREATH_PERIOD_MS;      /* mode6 */

  uint8_t mask = 0U;
  uint16_t pulse = 0U;

#if APP_ROLE_RX_ONLY
  /* 蓝线(HORN)独立爆闪：固定 50ms 亮 / 50ms 灭，不复用右键模式语义 */
  if (g_rx_horn_active != 0U) {
    uint16_t phase = GetPhase1ms((uint16_t)(HORN_FLASH_HALF_PERIOD_MS * 2U));
    mask = (uint8_t)(CH_L_HI_MASK | CH_R_HI_MASK);
    pulse = (phase < HORN_FLASH_HALF_PERIOD_MS) ? base : 0U;
    Channel_ApplyMask(mask);
    PwmGlobal_Set(pulse);
    return;
  }

  /* RX 对码结果反馈：仅短时覆盖近光输出，结束后自动回归原业务态。 */
  if (g_rx_pair_fb_state == RX_PAIR_FB_SUCCESS_HOLD) {
    mask = (uint8_t)(CH_L_LO_MASK | CH_R_LO_MASK);
    pulse = base;
    Channel_ApplyMask(mask);
    PwmGlobal_Set(pulse);
    if ((now - g_rx_pair_fb_tick) >= RX_PAIR_SUCCESS_HOLD_MS) {
      g_rx_pair_fb_state = RX_PAIR_FB_NONE;
      g_rx_pair_fb_step = 0U;
    }
    return;
  }

  if (g_rx_pair_fb_state == RX_PAIR_FB_FAIL_FLASH) {
    uint8_t on_phase = (uint8_t)((g_rx_pair_fb_step & 0x01U) == 0U);
    mask = (uint8_t)(CH_L_LO_MASK | CH_R_LO_MASK);
    pulse = (on_phase != 0U) ? base : 0U;
    Channel_ApplyMask(mask);
    PwmGlobal_Set(pulse);

    if ((now - g_rx_pair_fb_tick) >= RX_PAIR_FAIL_FLASH_HALF_MS) {
      g_rx_pair_fb_tick = now;
      g_rx_pair_fb_step++;
      if (g_rx_pair_fb_step >= RX_PAIR_FAIL_FLASH_TOGGLE_CNT) {
        g_rx_pair_fb_state = RX_PAIR_FB_NONE;
        g_rx_pair_fb_step = 0U;
      }
    }
    return;
  }
#endif

  (void)now;

  switch (g_light_mode) {
    case 1: /* 近光常亮 */
      mask = (uint8_t)(CH_L_LO_MASK | CH_R_LO_MASK);
      pulse = base;
      break;

    case 2: /* 远光常亮 */
      mask = (uint8_t)(CH_L_HI_MASK | CH_R_HI_MASK);
      pulse = base;
      break;

    case 3: /* 远近同亮 */
      mask = (uint8_t)(CH_L_HI_MASK | CH_L_LO_MASK | CH_R_HI_MASK | CH_R_LO_MASK);
      pulse = base;
      break;

    case 4: /* 远光同步快闪 */
    {
      uint16_t phase = GetPhase1ms((uint16_t)(fast_ms * 2U));
      mask = (uint8_t)(CH_L_HI_MASK | CH_R_HI_MASK);
      pulse = (phase < fast_ms) ? base : 0U;
      break;
    }

    case 5: /* 远近交替闪：A=双近光，B=双远光（左右同相） */
    {
      uint16_t phase = GetPhase1ms((uint16_t)(alt_ms * 2U));
      if (phase < alt_ms) {
        mask = (uint8_t)(CH_L_LO_MASK | CH_R_LO_MASK); /* 双近光 */
      } else {
        mask = (uint8_t)(CH_L_HI_MASK | CH_R_HI_MASK); /* 双远光 */
      }
      pulse = base;
      break;
    }

    case 6: /* 近光呼吸（三角波，相位锁定） */
    {
      uint16_t phase = GetPhase1ms(breath_ms);
      uint16_t half = (uint16_t)(breath_ms / 2U);
      uint16_t tri = (phase < half) ? phase : (uint16_t)(breath_ms - phase);
      mask = (uint8_t)(CH_L_LO_MASK | CH_R_LO_MASK);
      pulse = (uint16_t)(((uint32_t)base * (uint32_t)tri) / (uint32_t)half);
      break;
    }

    case 7: /* 远光左右轮闪（双脉冲）：L(100亮130灭100亮330灭) -> R(100亮130灭100亮330灭) */
    {
      uint16_t phase = GetPhase1ms(1320U);
      if (phase < 660U) {
        /* 左组阶段 */
        if ((phase < 100U) || ((phase >= 230U) && (phase < 330U))) {
          mask = CH_L_HI_MASK;
          pulse = base;
        } else {
          mask = 0U;
          pulse = 0U;
        }
      } else {
        /* 右组阶段 */
        uint16_t rphase = (uint16_t)(phase - 660U);
        if ((rphase < 100U) || ((rphase >= 230U) && (rphase < 330U))) {
          mask = CH_R_HI_MASK;
          pulse = base;
        } else {
          mask = 0U;
          pulse = 0U;
        }
      }
      break;
    }

    default:
      mask = 0U;
      pulse = 0U;
      break;
  }

  Channel_ApplyMask(mask);
  PwmGlobal_Set(pulse);
}

static void DebugPrint(const char *s)
{
#if UART_LOG_ENABLE
  HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), 100);
#else
  (void)s;
#endif
}

static void DebugPrintHex(const uint8_t *buf, uint8_t len)
{
#if UART_LOG_ENABLE
  char hex[3];
  uint8_t i;
  for(i = 0; i < len; i++) {
    sprintf(hex, "%02X", buf[i]);
    HAL_UART_Transmit(&huart1, (uint8_t*)hex, 2, 100);
  }
#else
  (void)buf;
  (void)len;
#endif
}

static void DebugPrintDec(uint16_t val)
{
#if UART_LOG_ENABLE
  char buf[6];
  sprintf(buf, "%u", val);
  HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
#else
  (void)val;
#endif
}


/**
 * @brief  更新本地时钟相位基准，积木5：周期边界时启动 LED PWM
 *         使用 HAL_GetTick 的时间差，滚动更新 g_phase_ms
 */
static void SyncTime_Update(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t delta = now - g_last_tick_ms;
  if(delta == 0) return;
  g_last_tick_ms = now;

  uint32_t total = g_phase_ms + delta;
  uint32_t new_cycles = total / SYNC_CYCLE_MS;
  g_cycle += new_cycles;
  g_phase_ms = (uint16_t)(total % SYNC_CYCLE_MS);

        /* 周期边界：跨越 0ms 时立即点亮 LED（PWM 124kHz 60%），积木4 */
  if(new_cycles > 0) {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    g_led_on_tick = now;
    g_led_state = 1;
    #if DEBUG_SYNC_VERBOSE
    DebugPrint("[CYCLE] new_cycle=");
    DebugPrintDec(g_cycle);
    DebugPrint(", phase_reset=0ms\r\n");
#endif
#if DEBUG_LED_VERBOSE
    DebugPrint("[LED] CYCLE @0\r\n");
#endif
  }
}

/**
 * @brief  同步闪灯：时间戳保证至少亮 100ms 后熄灭，积木4/5
 *         PWM_GLOBAL(PA2) 用 TIM1_CH3 PWM，PB6 状态指示
 */
static void SyncLamp_Update(void)
{
  uint32_t now = HAL_GetTick();

  if(g_led_state == 1) {
    if((now - g_led_on_tick) >= SYNC_LED_ON_MS) {
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      g_led_state = 0;
      #if DEBUG_LED_VERBOSE
      DebugPrint("[LED] OFF\r\n");
#endif
    }
  }
  /* 注释：移除补亮逻辑，LED只在周期边界点亮 */
  /* 这样确保每个周期只点亮一次，持续100ms */
}

/* 构造同步包，积木8：传输延迟补偿 6ms */
static void BuildSyncPacket(uint8_t *pkt)
{
  uint16_t compensated = g_phase_ms + SYNC_TX_DELAY_MS;
  if(compensated >= SYNC_CYCLE_MS) compensated -= SYNC_CYCLE_MS;

  pkt[0] = 0xAA;
  pkt[1] = 0x55;
  pkt[2] = (uint8_t)(compensated >> 8);
  pkt[3] = (uint8_t)(compensated & 0xFF);
}

static uint16_t ParseSyncPacket(const uint8_t *pkt)
{
  if(pkt[0] != 0xAA || pkt[1] != 0x55) return 0xFFFF;
  return (uint16_t)((pkt[2] << 8) | pkt[3]);
}

/* 临时灯光控制包构造：先占位业务字段（后面替换成你们真正协议） */
static uint8_t TempLight_CalcChecksum(const uint8_t *pkt)
{
  uint8_t x = 0;
  x ^= pkt[0];
  x ^= pkt[1];
  x ^= pkt[2];
  x ^= pkt[3];
  x ^= pkt[TLIGHT_PKT_KEYTRIG_IDX];
  x ^= pkt[TLIGHT_PKT_DEV_ID_L_IDX];
  x ^= pkt[TLIGHT_PKT_DEV_ID_H_IDX];
  x ^= pkt[TLIGHT_PKT_TAG_IDX];
  return x;
}

static void BuildTempLightPacket(uint8_t *pkt, uint8_t mode, uint8_t bright, uint8_t key, uint8_t trig)
{
  uint8_t seq4 = (uint8_t)(g_link_tx_seq & 0x0FU);
  if (key > TLIGHT_KEY_NONE) key = TLIGHT_KEY_NONE;
  if (trig > TLIGHT_TRIG_LONG) trig = TLIGHT_TRIG_NONE;

  pkt[0] = TLIGHT_PKT_HEADER0;
  pkt[1] = TLIGHT_PKT_HEADER1;
  pkt[2] = mode;
  pkt[3] = bright;
  pkt[TLIGHT_PKT_KEYTRIG_IDX] = (uint8_t)(((key & 0x03U) << 6) | ((trig & 0x03U) << 4) | seq4);
  pkt[TLIGHT_PKT_DEV_ID_L_IDX] = (uint8_t)(g_dev_id_local & 0xFFU);
  pkt[TLIGHT_PKT_DEV_ID_H_IDX] = (uint8_t)(g_dev_id_local >> 8);
  pkt[TLIGHT_PKT_TAG_IDX] = TLIGHT_PKT_TAG_CONST;
  pkt[TLIGHT_PKT_CHKSUM_IDX] = TempLight_CalcChecksum(pkt);

  g_link_tx_seq++;
}

/* 解析临时灯光控制包：返回 1=合法，否则 0 */
static uint8_t ParseTempLightPacket(const uint8_t *pkt, uint8_t *mode, uint8_t *bright, uint8_t *key, uint8_t *trig)
{
  uint16_t pkt_id = (uint16_t)((pkt[TLIGHT_PKT_DEV_ID_H_IDX] << 8) | pkt[TLIGHT_PKT_DEV_ID_L_IDX]);
  uint8_t key_raw = (uint8_t)((pkt[TLIGHT_PKT_KEYTRIG_IDX] >> 6) & 0x03U);
  uint8_t trig_raw = (uint8_t)((pkt[TLIGHT_PKT_KEYTRIG_IDX] >> 4) & 0x03U);

  if(pkt[0] != TLIGHT_PKT_HEADER0 || pkt[1] != TLIGHT_PKT_HEADER1) return 0;
  if(pkt[2] < TLIGHT_MODE_OFF || pkt[2] > TLIGHT_MODE_MAX) return 0;
  if(pkt[3] > TLIGHT_BRIGHT_100) return 0;
  if(key_raw > TLIGHT_KEY_NONE) return 0;
  if(trig_raw > TLIGHT_TRIG_LONG) return 0;
  if(pkt_id != g_dev_id_paired) return 0;
  if(pkt[TLIGHT_PKT_TAG_IDX] != TLIGHT_PKT_TAG_CONST) return 0;
  if(pkt[TLIGHT_PKT_CHKSUM_IDX] != TempLight_CalcChecksum(pkt)) return 0;

  *mode = pkt[2];
  *bright = pkt[3];
  *key = key_raw;
  *trig = trig_raw;
  return 1;
}

static void LinkStats_Log(uint32_t now)
{
  if ((now - g_link_last_stats_tick) < LINK_STATS_LOG_INTERVAL_MS) {
    return;
  }
  g_link_last_stats_tick = now;

#if APP_ROLE_TX_ONLY
  if ((g_link_tx_ok == 0U) && (g_link_tx_fail == 0U) && (g_pairing_mode == 0U)) {
    return;
  }

  if ((g_link_tx_ok == g_link_last_log_tx_ok) &&
      (g_link_tx_fail == g_link_last_log_tx_fail) &&
      (g_tx_rf_awake == g_link_last_log_tx_awake) &&
      (g_pairing_mode == 0U)) {
    return;
  }

  g_link_last_log_tx_ok = g_link_tx_ok;
  g_link_last_log_tx_fail = g_link_tx_fail;
  g_link_last_log_tx_awake = g_tx_rf_awake;

  DebugPrint("[TX-STAT] ok=");
  DebugPrintDec((uint16_t)(g_link_tx_ok & 0xFFFFU));
  DebugPrint(" fail=");
  DebugPrintDec((uint16_t)(g_link_tx_fail & 0xFFFFU));
  DebugPrint(" seq=");
  DebugPrintDec((uint16_t)g_link_tx_seq);
  DebugPrint(" awake=");
  DebugPrintDec((uint16_t)g_tx_rf_awake);
  DebugPrint(" dev=");
  DebugPrintHex((const uint8_t *)&g_dev_id_local, 2);
  DebugPrint("\r\n");
#elif APP_ROLE_RX_ONLY
  DebugPrint("[RX-STAT] ok=");
  DebugPrintDec((uint16_t)(g_link_rx_ok & 0xFFFFU));
  DebugPrint(" parse_err=");
  DebugPrintDec((uint16_t)(g_link_rx_parse_err & 0xFFFFU));
  DebugPrint(" dev_miss=");
  DebugPrintDec((uint16_t)(g_link_rx_dev_mismatch & 0xFFFFU));
  DebugPrint(" dup=");
  DebugPrintDec((uint16_t)(g_link_rx_dup & 0xFFFFU));
  DebugPrint(" jump=");
  DebugPrintDec((uint16_t)(g_link_rx_jump & 0xFFFFU));
  DebugPrint("\r\n");
#endif
}

static void Sync_AdjustFromPacket(uint16_t rx_phase_ms)
{
  int16_t delta = (int16_t)rx_phase_ms - (int16_t)g_phase_ms;
  if(delta > (int16_t)(SYNC_CYCLE_MS / 2)) delta -= SYNC_CYCLE_MS;
  if(delta < -(int16_t)(SYNC_CYCLE_MS / 2)) delta += SYNC_CYCLE_MS;
  
  #if DEBUG_SYNC_VERBOSE
  /* 调试：显示调整前的详细信息 */
  DebugPrint("[ADJ] rx=");
  DebugPrintDec(rx_phase_ms);
  DebugPrint(" local=");
  DebugPrintDec(g_phase_ms);
  DebugPrint(" delta=");
  if(delta >= 0) {
    DebugPrint("+");
    DebugPrintDec((uint16_t)delta);
  } else {
    DebugPrint("-");
    DebugPrintDec((uint16_t)(-delta));
  }
  DebugPrint(" adjust=");
  DebugPrintDec((uint16_t)(delta / 4));
  DebugPrint("ms\r\n");
#endif

  g_phase_ms = (uint16_t)((int16_t)g_phase_ms + delta / 4);
  if(g_phase_ms >= SYNC_CYCLE_MS) g_phase_ms -= SYNC_CYCLE_MS;
}

/**
 * @brief  同步主状态机：TX@450ms固定时间发送，450-900ms全程RX侦听
 *         相邻频道策略（TX@76, RX@75）避免自干扰和碰撞
 */
static void Sync_MainLoop(void)
{
  /* 更新本地时间基准（增量式） */
  SyncTime_Update();
  /* 驱动 LED 指示灯（100ms 保持亮，然后熄灭） */
  SyncLamp_Update();

  /* 固定 TX 时间段：所有节点同一时刻发送，确保相位对齐 */
  if (g_cycle != g_last_tx_cycle && g_phase_ms >= SYNC_TX_TIME_MS && g_phase_ms < (SYNC_TX_TIME_MS + 50U)) {
    /* 切换到 TX 模式，频道 76 */
    if (g_rf_mode != 1) {
      RF_Link_ConfigTx(RF_TX_CHANNEL);
      g_rf_mode = 1;
    }

        /* 构造并发送同步包 */
    BuildSyncPacket(RF_TX_Buf);
        if (RF_Link_Send(RF_TX_Buf, SYNC_PKT_SIZE) == 0) {
#if DEBUG_SYNC_VERBOSE
      DebugPrint("[TX] phase=");
      DebugPrintDec(g_phase_ms);
      DebugPrint("ms, target=");
      DebugPrintDec(SYNC_TX_TIME_MS);
      DebugPrint("ms, diff=");
      if(g_phase_ms >= SYNC_TX_TIME_MS) {
        DebugPrint("+");
        DebugPrintDec(g_phase_ms - SYNC_TX_TIME_MS);
      } else {
        DebugPrint("-");
        DebugPrintDec(SYNC_TX_TIME_MS - g_phase_ms);
      }
      DebugPrint("ms\r\n");
#else
      DebugPrint("[TX]\r\n");
#endif
    }
    g_last_tx_cycle = g_cycle;

            /* 注释：TX事件不再点亮LED，只做调试输出 */
    /* HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); */
    /* HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); */
    /* g_led_on_tick = HAL_GetTick(); */
    /* g_led_state = 1; */
    /* LED调试：TX事件（仅调试，不影响LED） */
    DebugPrint("[TX]\r\n");

    /* 发送完立刻切回 RX 模式，频道 75，全程侦听 */
    RF_Link_ConfigRx(RF_RX_CHANNEL);
    g_rf_mode = 0;
  }

  /* RX 模式下轮询接收 - 全周期执行（不睡眠，连续侦听） */
  if (g_rf_mode == 0) {
    uint8_t rx_len = 0;
    if (RF_Link_PollReceive(RF_RX_Buf, &rx_len) == 1) {
      uint16_t rx_phase_ms = ParseSyncPacket(RF_RX_Buf);

                  if (rx_phase_ms < SYNC_CYCLE_MS) {
#if DEBUG_SYNC_VERBOSE
        DebugPrint("RX: ph=");
        DebugPrintDec(rx_phase_ms);
        DebugPrint(" local=");
        DebugPrintDec(g_phase_ms);
        
        /* 计算相位差异 */
        int16_t phase_diff = (int16_t)rx_phase_ms - (int16_t)g_phase_ms;
        if(phase_diff > (int16_t)(SYNC_CYCLE_MS / 2)) phase_diff -= SYNC_CYCLE_MS;
        if(phase_diff < -(int16_t)(SYNC_CYCLE_MS / 2)) phase_diff += SYNC_CYCLE_MS;
        
        DebugPrint(" diff=");
        if(phase_diff >= 0) {
          DebugPrint("+");
          DebugPrintDec((uint16_t)phase_diff);
        } else {
          DebugPrint("-");
          DebugPrintDec((uint16_t)(-phase_diff));
        }
        
        DebugPrint("[RX]");
        
        DebugPrint(" adj=");
        DebugPrintDec(g_phase_ms);
        DebugPrint("\r\n");
#else
        DebugPrint("RX\r\n");
#endif
        
        /* 相位调整 */
        Sync_AdjustFromPacket(rx_phase_ms);
      }
    }
  }
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
#if APP_ROLE_TX_ONLY
  if (GPIO_Pin == R_KEY_Pin) {
    uint32_t now = HAL_GetTick();
    g_tx_r_wkup_exti_rise_tick = now;
    g_tx_r_wkup_exti_rise_seen = 1U;

    /* WKUP4+OFF 专用：按下沿（上升沿）才是 second-press 候选，
     * 先在 ISR 侧快速置位，主循环再统一判语义。 */
    if ((g_tx_r_wkup_fast_edge_armed != 0U) && (g_tx_r_wkup_ext_dbl_active != 0U)) {
      if ((now - g_tx_r_wkup_fast_edge_last_tick) >= KEY_TX_WKUP4_FAST_EDGE_DEBOUNCE_MS) {
        g_tx_r_wkup_fast_edge_last_tick = now;
        g_tx_r_wkup_fast_edge_tick = now;
        g_tx_r_wkup_fast_edge_hit = 1U;
      }
    }
  }
#else
  (void)GPIO_Pin;
#endif
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
#if APP_ROLE_TX_ONLY
  if (GPIO_Pin == R_KEY_Pin) {
    g_tx_r_wkup_exti_fall_tick = HAL_GetTick();
    g_tx_r_wkup_exti_fall_seen = 1U;
  }
#else
  (void)GPIO_Pin;
#endif
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
