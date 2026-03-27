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
#include "stm32c0xx_hal_flash.h"
#include "stm32c0xx_hal_flash_ex.h"

/* 中文注释：
   - XL2400T.h 与 rf_xl2400.h 提供 2.4GHz RF 模块初始化、发送、接收接口
   - 采用 900ms 周期同步协议：0-450ms 发送同步包，450-900ms 接收同步包
   - LED 指示：PB6 (LED_Pin) 同步事件指示 + PA2 (LED_DRV_Pin) PWM 驱动
*/
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SYNC_CYCLE_MS       900U    /* 同步周期 900ms */
#define SYNC_LED_ON_MS      100U    /* LED 亮灯时间 100ms */
#define SYNC_TX_TIME_MS     450U    /* TX 发送时间：所有节点固定 450ms */
#define SYNC_TX_DELAY_MS    6U      /* 传输延迟补偿 */
#define SYNC_PKT_SIZE       4U      /* AA 55 + 2字节相位 */
#define RF_TX_CHANNEL       76U     /* XL2400T TX 频道 76 (2476 MHz) */
#define RF_RX_CHANNEL       75U     /* XL2400T RX 频道 75 (2475 MHz) - 相邻频道避免自干扰 */

/* RF 链路测试模式（先简后繁）：
 * - RX-only：循环 PollReceive，收到后打印/点亮 LED
 * - TX-only：循环周期性 Send 同步包（AA55 + 相位）
 * 你需要把角色宏切成不同编译即可得到 TX/RX 两个程序。
 */
#define APP_RF_LINK_TEST  1
#define APP_ROLE_RX_ONLY  0
#define APP_ROLE_TX_ONLY  1

/* 仅允许启用一个角色 */
#if (APP_ROLE_RX_ONLY + APP_ROLE_TX_ONLY) != 1
#error "APP_ROLE_RX_ONLY / APP_ROLE_TX_ONLY must enable exactly one role"
#endif

#define LINK_TX_INTERVAL_MS  200U

/* 临时灯光控制包（先测收发/解析，不上真正业务字段复杂度） */
/* 协议（8字节负载）：
 * byte0=0xAA, byte1=0x55  : 帧头
 * byte2=mode(1~7)        : 灯光模式号
 * byte3=bright(0~2)     : 亮度档位（0=30%, 1=70%, 2=100%）
 * byte4..byte7=0        : 预留（先不做 ID/校验）
 */
#define TLIGHT_PKT_HEADER0 0xAA
#define TLIGHT_PKT_HEADER1 0x55
#define TLIGHT_MODE_MIN     1U
#define TLIGHT_MODE_MAX     7U

#define TLIGHT_BRIGHT_30    0U
#define TLIGHT_BRIGHT_70    1U
#define TLIGHT_BRIGHT_100   2U

/* 临时协议扩展：加入序号与简易校验，便于链路质量统计 */
#define TLIGHT_PKT_SEQ_IDX      4U
#define TLIGHT_PKT_DEV_ID_IDX   5U
#define TLIGHT_PKT_CHKSUM_IDX   6U
#define TLIGHT_PKT_TAG_IDX      7U
#define TLIGHT_PKT_TAG_CONST    0x5AU

/* P1：设备ID（单向链路唯一性）
 * - TX 发送自己的设备ID
 * - RX 仅接收与本机已配对ID一致的数据包
 * 当前 8 字节包先使用 8-bit ID；后续若扩展 16-bit，可在 v1.1 增长包长度。 */
#define TLIGHT_DEVICE_ID_DEFAULT   0x2AU
#define TLIGHT_DEVICE_ID_INVALID   0x00U

/* 设备ID持久化（Flash 最后一页）
 * 说明：这是“对码流程骨架”实现。量产时建议在链接脚本明确预留参数页，避免与程序区冲突。 */
#define DEVCFG_MAGIC0              0xD1U
#define DEVCFG_MAGIC1              0xA5U
#define DEVCFG_VERSION             0x01U
#define DEVCFG_FLASH_ADDR          (FLASH_BASE + FLASH_SIZE - FLASH_PAGE_SIZE)
#define DEVCFG_FLASH_PAGE          (FLASH_PAGE_NB - 1U)

#define TLIGHT_MODE_SWITCH_INTERVAL_MS 2000U
#define LINK_RX_TIMEOUT_MS           1200U  /* 超过 1.2s 未收到有效包，判定链路超时 */
#define LINK_STATS_LOG_INTERVAL_MS   3000U  /* 每 3s 打印一次统计，避免刷屏 */

/* 简化可见指示（用 LED_Pin，不做真实 LM3409 PWM 还原） */
#define LED_FLASH_PERIOD_MS  250U   /* 闪烁周期 */
#define LED_BREATH_PERIOD_MS 800U   /* 呼吸周期（简化为慢速占空） */

#define SOLAR_ADC_NIGHT_THRESHOLD   360U
#define SOLAR_ADC_DAY_THRESHOLD    496U
#define DAYNIGHT_SAMPLE_INTERVAL_MS 1000U
#define DAYNIGHT_HOLD_MS            3000U

#define BATT_ADC_OVERCHARGE_RAW     1924U
#define BATT_ADC_REENABLE_RAW      1676U
#define CHARGE_SAMPLE_INTERVAL_MS  3000U

#define DEBUG_ADC_VERBOSE  1   /* 调试：1=打印 ADC 采样值，完成后可改为 0 精简 */
#define DEBUG_SYNC_VERBOSE 1   /* 调试：1=打印详细同步信息，0=只打印关键事件 */
#define DEBUG_LED_VERBOSE  1   /* 调试：1=打印LED状态，0=不打印 */
#define DEBUG_PERIODIC     0   /* 调试：1=启用定期打印（影响功耗），0=禁用 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

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

static uint8_t  g_is_night = 1;
static uint32_t g_last_daynight_tick = 0;
static uint32_t g_daynight_hold_until_tick = 0;

static uint32_t g_last_charge_tick = 0;

static uint8_t  g_rf_sleeping = 0;
static uint8_t  g_rf_sleeping_night = 0;
static uint32_t g_adc_display_tick = 0;  /* ADC显示计时器 */

/* 链路调试统计（用于串口观测通信质量） */
static uint8_t  g_link_tx_seq = 0;
static uint8_t  g_link_last_rx_seq = 0;
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

/* 设备ID运行态（来自 Flash） */
static uint8_t g_dev_id_local = TLIGHT_DEVICE_ID_DEFAULT;
static uint8_t g_dev_id_paired = TLIGHT_DEVICE_ID_DEFAULT;

/* 双路 PWM 灯效状态（RX-only） */
static uint8_t g_light_mode = TLIGHT_MODE_MIN;
static uint8_t g_light_bright = TLIGHT_BRIGHT_30;

/* 夜间初期双通道采样调试 */
static uint32_t g_night_start_tick = 0;     /* 夜间开始时间 */
static uint8_t  g_night_debug_window = 0;   /* 是否在调试窗口内（夜间开始后6秒） */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void DebugPrint(const char *s);
static void DebugPrintHex(const uint8_t *buf, uint8_t len);
static void DebugPrintDec(uint16_t val);
static void SyncTime_Update(void);
static void SyncLamp_Update(void);
static void BuildSyncPacket(uint8_t *pkt);
static uint16_t ParseSyncPacket(const uint8_t *pkt);
static void BuildTempLightPacket(uint8_t *pkt, uint8_t mode, uint8_t bright);
static uint8_t ParseTempLightPacket(const uint8_t *pkt, uint8_t *mode, uint8_t *bright);
static uint8_t TempLight_CalcChecksum(const uint8_t *pkt);
static uint8_t DevId_CalcCrc8(const uint8_t *buf, uint8_t len);
static void DevId_LoadFromFlash(void);
static uint8_t DevId_SaveToFlash(uint8_t local_id, uint8_t paired_id);
static void LinkStats_Log(uint32_t now);
static void Sync_AdjustFromPacket(uint16_t rx_phase_ms);
static void Sync_MainLoop(void);
static void DayNight_Update(void);
static void Charge_Update(void);
static void Light_Tick(uint32_t now);
static uint32_t Read_ADC1_Channel(uint32_t channel);  /* 指定通道读 ADC1 一次，PA0=ch0 太阳能，PA1=ch1 电池 */
static void Test_ADC_Channels(void);                   /* ADC通道切换测试函数 */
static uint8_t Read_ADC1_DualChannel(uint32_t* solar_raw, uint32_t* batt_raw);  /* 扫描模式读取双通道 */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct {
  uint8_t magic0;
  uint8_t magic1;
  uint8_t version;
  uint8_t local_id;
  uint8_t paired_id;
  uint8_t reserved;
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

static void DevId_LoadFromFlash(void)
{
  const DevCfg_t *cfg = (const DevCfg_t *)DEVCFG_FLASH_ADDR;
  uint8_t calc = DevId_CalcCrc8((const uint8_t *)cfg, 6U);

  if ((cfg->magic0 == DEVCFG_MAGIC0) &&
      (cfg->magic1 == DEVCFG_MAGIC1) &&
      (cfg->version == DEVCFG_VERSION) &&
      (cfg->tail == TLIGHT_PKT_TAG_CONST) &&
      (cfg->local_id != TLIGHT_DEVICE_ID_INVALID) &&
      (cfg->paired_id != TLIGHT_DEVICE_ID_INVALID) &&
      (cfg->crc8 == calc)) {
    g_dev_id_local = cfg->local_id;
    g_dev_id_paired = cfg->paired_id;
    return;
  }

  /* Flash 未初始化或损坏：使用默认ID并回写一次 */
  g_dev_id_local = TLIGHT_DEVICE_ID_DEFAULT;
  g_dev_id_paired = TLIGHT_DEVICE_ID_DEFAULT;
  (void)DevId_SaveToFlash(g_dev_id_local, g_dev_id_paired);
}

static uint8_t DevId_SaveToFlash(uint8_t local_id, uint8_t paired_id)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t page_error = 0;
  DevCfg_t cfg;
  uint64_t row = 0;

  if ((local_id == TLIGHT_DEVICE_ID_INVALID) || (paired_id == TLIGHT_DEVICE_ID_INVALID)) {
    return 0;
  }

  cfg.magic0 = DEVCFG_MAGIC0;
  cfg.magic1 = DEVCFG_MAGIC1;
  cfg.version = DEVCFG_VERSION;
  cfg.local_id = local_id;
  cfg.paired_id = paired_id;
  cfg.reserved = 0x00U;
  cfg.crc8 = DevId_CalcCrc8((const uint8_t *)&cfg, 6U);
  cfg.tail = TLIGHT_PKT_TAG_CONST;

  memcpy(&row, &cfg, sizeof(row));

  HAL_FLASH_Unlock();

  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Page = DEVCFG_FLASH_PAGE;
  erase.NbPages = 1;
  if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) {
    HAL_FLASH_Lock();
    return 0;
  }

  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, DEVCFG_FLASH_ADDR, row) != HAL_OK) {
    HAL_FLASH_Lock();
    return 0;
  }

  HAL_FLASH_Lock();

  g_dev_id_local = local_id;
  g_dev_id_paired = paired_id;
  return 1;
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  DevId_LoadFromFlash();
  DebugPrint("[FW] " FW_VERSION "\r\n");
  DebugPrint("[DEV] local=");
  DebugPrintDec((uint16_t)g_dev_id_local);
  DebugPrint(" paired=");
  DebugPrintDec((uint16_t)g_dev_id_paired);
  DebugPrint("\r\n");

  /* 原 PA3=BOOST_EN 已改为 PWM2（TIM1_CH4 / LED_DRV2_Pin），不再控制升压使能 */
  HAL_Delay(100);

  /* 两路 PWM 启动（PA2=TIM1_CH3, PA3=TIM1_CH4） */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);



    /* RF 模块初始化 */
  DebugPrint("RF Init...\r\n");
  RF_Link_Init();
  DebugPrint("RF Init OK\r\n");

#if APP_RF_LINK_TEST && APP_ROLE_RX_ONLY
  /* 链路测试：RX-only */
  DebugPrint("RF Config RX...\r\n");
  RF_Link_ConfigRx(RF_RX_CHANNEL);
  g_rf_mode = 0;
#elif APP_RF_LINK_TEST && APP_ROLE_TX_ONLY
  /* 链路测试：TX-only */
  DebugPrint("RF Config TX...\r\n");
  RF_Link_ConfigTx(RF_TX_CHANNEL);
  g_rf_mode = 1;
#else
  /* 默认进入 RX 模式，监听频道 75（保持原 demo 行为） */
  DebugPrint("RF Config RX...\r\n");
  RF_Link_ConfigRx(RF_RX_CHANNEL);
  g_rf_mode = 0;
#endif
  DebugPrint("RF Ready\r\n");
  
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

  g_is_night = 1;
  g_last_daynight_tick = HAL_GetTick();
  g_daynight_hold_until_tick = 0;  /* 0=未在确认中，非0=满足条件后需持续到该 tick 才翻转 */

  g_last_charge_tick = HAL_GetTick();

  g_rf_sleeping = 0;
  g_rf_sleeping_night = 0;

#if DEBUG_ADC_VERBOSE
  /* 积木1 调试：上电读取双通道 ADC 验证 */
  uint32_t s0 = Read_ADC1_Channel(ADC_CHANNEL_0);
  uint32_t s1 = Read_ADC1_Channel(ADC_CHANNEL_1);
  DebugPrint("[ADC] ch0=");
  DebugPrintDec((uint16_t)s0);
  DebugPrint(" ch1=");
  DebugPrintDec((uint16_t)s1);
  DebugPrint("\r\n");
  
  /* 运行ADC通道切换测试 */
  Test_ADC_Channels();
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
    static uint8_t last_is_night = 1;  /* 积木6/7：日夜切换时处理 RF 与低功耗 */
    /* 积木8：喂狗 - 暂时禁用 */
    // HAL_IWDG_Refresh(&hiwdg);

#if APP_RF_LINK_TEST
      /* 直接做收发链路测试，跳过后续日夜/同步/低功耗逻辑 */
      uint32_t now = HAL_GetTick();

#if APP_ROLE_TX_ONLY
      static uint32_t last_tx_ms = 0;

      static uint8_t cur_mode = TLIGHT_MODE_MIN;
      static uint8_t cur_bright = TLIGHT_BRIGHT_30;
      static uint32_t last_mode_ms = 0;

      /* 周期性发送临时灯光控制包（先测链路/解析，后面替换成你们正式业务协议） */
      if ((now - last_tx_ms) >= LINK_TX_INTERVAL_MS) {
        last_tx_ms = now;

        BuildTempLightPacket(RF_TX_Buf, cur_mode, cur_bright);

        if (RF_Link_Send(RF_TX_Buf, RF_PACKET_SIZE) == 0) {
          g_link_tx_ok++;
          /* 发送成功用 LED 指示一下（持续短亮），串口只在“mode变化”时打印 */
          HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
          g_led_state = 1;
          g_led_on_tick = now;
        } else {
          g_link_tx_fail++;
          DebugPrint("[TX-ERR] send fail\r\n");
        }
      }

      /* mode/brightness 每 2 秒切换一次 */
      if(last_mode_ms == 0) {
        last_mode_ms = now;
      } else if((now - last_mode_ms) >= TLIGHT_MODE_SWITCH_INTERVAL_MS) {
        last_mode_ms = now;

        cur_mode++;
        if(cur_mode > TLIGHT_MODE_MAX) cur_mode = TLIGHT_MODE_MIN;

        cur_bright++;
        if(cur_bright > TLIGHT_BRIGHT_100) cur_bright = TLIGHT_BRIGHT_30;

        DebugPrint("[TX] mode=");
        DebugPrintDec((uint16_t)cur_mode);
        DebugPrint(" bright=");
        DebugPrintDec((uint16_t)cur_bright);
        DebugPrint(" seq=");
        DebugPrintDec((uint16_t)((uint8_t)(g_link_tx_seq - 1U)));
        DebugPrint(" dev=");
        DebugPrintDec((uint16_t)g_dev_id_local);
        DebugPrint(" (30/70/100)\r\n");
      }

      /* LED 定时熄灭 */
      if (g_led_state == 1) {
        if ((now - g_led_on_tick) >= SYNC_LED_ON_MS) {
          HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
          g_led_state = 0;
        }
      }

#elif APP_ROLE_RX_ONLY
      /* RX-only：轮询接收 */
      uint8_t rx_len = 0;
      static uint8_t cur_mode = TLIGHT_MODE_MIN;
      static uint8_t cur_bright = TLIGHT_BRIGHT_30;
      static uint8_t last_print_mode = 0xFF;
      static uint8_t last_print_bright = 0xFF;

      if (RF_Link_PollReceive(RF_RX_Buf, &rx_len) == 1) {
        uint8_t mode = 0;
        uint8_t bright = 0;
        if (ParseTempLightPacket(RF_RX_Buf, &mode, &bright)) {
          uint8_t seq = RF_RX_Buf[TLIGHT_PKT_SEQ_IDX];

          g_link_rx_ok++;
          g_link_last_rx_tick = now;
          g_link_timeout_flag = 0;

          if (g_link_has_last_seq) {
            uint8_t delta = (uint8_t)(seq - g_link_last_rx_seq);
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
          g_light_mode = cur_mode;
          g_light_bright = cur_bright;

          if(cur_mode != last_print_mode || cur_bright != last_print_bright) {
            DebugPrint("[RX] mode=");
            DebugPrintDec((uint16_t)cur_mode);
            DebugPrint(" bright=");
            DebugPrintDec((uint16_t)cur_bright);
            DebugPrint(" seq=");
            DebugPrintDec((uint16_t)seq);
            DebugPrint(" (30/70/100)\r\n");
            last_print_mode = cur_mode;
            last_print_bright = cur_bright;
          }
        } else {
          if ((RF_RX_Buf[0] == TLIGHT_PKT_HEADER0) &&
              (RF_RX_Buf[1] == TLIGHT_PKT_HEADER1) &&
              (RF_RX_Buf[TLIGHT_PKT_TAG_IDX] == TLIGHT_PKT_TAG_CONST) &&
              (RF_RX_Buf[TLIGHT_PKT_CHKSUM_IDX] == TempLight_CalcChecksum(RF_RX_Buf)) &&
              (RF_RX_Buf[TLIGHT_PKT_DEV_ID_IDX] != g_dev_id_paired)) {
            g_link_rx_dev_mismatch++;
          } else {
            g_link_rx_parse_err++;
            DebugPrint("[RX-ERR] bad packet\r\n");
          }
        }
      }

      if ((g_link_last_rx_tick != 0U) && ((now - g_link_last_rx_tick) >= LINK_RX_TIMEOUT_MS) && (g_link_timeout_flag == 0U)) {
        g_link_timeout_flag = 1;
        DebugPrint("[RX-WARN] timeout > ");
        DebugPrintDec((uint16_t)LINK_RX_TIMEOUT_MS);
        DebugPrint("ms\r\n");
      }
      
      /* 驱动两路 PWM 输出（PA2=CH3, PA3=CH4） */
      Light_Tick(now);
#endif

      /* 周期统计，避免日志刷屏 */
      LinkStats_Log(now);

      /* 继续下一轮测试 */
      continue;
#endif /* APP_RF_LINK_TEST */

    #if DEBUG_PERIODIC
    /* 定期显示双通道ADC值（每10秒一次）- 影响功耗，调试完成后禁用 */
    uint32_t now = HAL_GetTick();
    if((now - g_adc_display_tick) >= 10000U) {
      g_adc_display_tick = now;
      
            uint32_t solar_raw, batt_raw;
      if(Read_ADC1_DualChannel(&solar_raw, &batt_raw) == 0) {
        DebugPrint("[ADC-DUAL] solar=");
        DebugPrintDec((uint16_t)solar_raw);
        DebugPrint("(");
        
        /* 计算太阳能电压 */
        uint32_t solar_mv = (solar_raw * 3300) / 4095;
        DebugPrintDec((uint16_t)(solar_mv / 1000));
        DebugPrint(".");
        DebugPrintDec((uint16_t)((solar_mv % 1000) / 100));
        DebugPrintDec((uint16_t)((solar_mv % 100) / 10));
        DebugPrint("V) batt=");
        
        DebugPrintDec((uint16_t)batt_raw);
        DebugPrint("(");
        
        /* 计算电池电压 */
        uint32_t batt_mv = (batt_raw * 3300) / 4095;
        DebugPrintDec((uint16_t)(batt_mv / 1000));
        DebugPrint(".");
        DebugPrintDec((uint16_t)((batt_mv % 1000) / 100));
        DebugPrintDec((uint16_t)((batt_mv % 100) / 10));
        DebugPrint("V)\r\n");
      }
    }
#endif

    DayNight_Update();
    Charge_Update();

    /* 日夜切换：进入夜间唤醒 RF，进入白天 RF 睡眠 */
    {
      static uint8_t last_is_night = 1;
      if(last_is_night != g_is_night) {
        if(g_is_night) {
          RF_Link_ConfigRx(RF_RX_CHANNEL);
          g_rf_mode = 0;
          g_rf_sleeping = 0;
        } else {
          RF_Link_Sleep();
          g_rf_sleeping = 1;
        }
        last_is_night = g_is_night;
      }
    }

    if(g_is_night) {
      /* 夜间：同步+闪灯 */
      Sync_MainLoop();
      SyncLamp_Update();
      HAL_Delay(1);
    } else {
      /* 白天：关灯，不跑同步 */
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

      if(!g_rf_sleeping) {
        RF_Link_Sleep();
        g_rf_sleeping = 1;
      }

            /* 积木7：白天 CPU 进入 Sleep(WFI)，靠 SysTick 唤醒 */
      /* 进入睡眠前禁用串口以降低功耗 */
      HAL_UART_DeInit(&huart1);
      HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
      /* 唤醒后重新初始化串口 */
      MX_USART1_UART_Init();
    }
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  HAL_GPIO_WritePin(GPIOA, RF_CSN_Pin|RF_SCK_Pin|DIV_MOS_Pin|RF_DATA_Pin
                          |CHG_MOS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : KEY_IN_Pin */
  GPIO_InitStruct.Pin = KEY_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_CSN_Pin RF_SCK_Pin DIV_MOS_Pin RF_DATA_Pin
                           CHG_MOS_Pin */
  GPIO_InitStruct.Pin = RF_CSN_Pin|RF_SCK_Pin|DIV_MOS_Pin|RF_DATA_Pin
                          |CHG_MOS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

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

static void Pwm_Set(uint16_t ch3_pulse, uint16_t ch4_pulse)
{
  uint16_t arrp1 = Pwm_GetArrPlus1();
  if (ch3_pulse > arrp1) ch3_pulse = arrp1;
  if (ch4_pulse > arrp1) ch4_pulse = arrp1;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ch3_pulse);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ch4_pulse);
}

/* 非阻塞灯效状态机：只依赖 HAL_GetTick() */
static void Light_Tick(uint32_t now)
{
  const uint16_t base = Bright_ToPulse(g_light_bright);

  /* 让不同模式有不同节奏，便于示波器/肉眼区分 */
  const uint32_t fast_ms = 100U;  /* mode4: 10Hz */
  const uint32_t alt_ms  = 200U;  /* mode5: 2.5Hz 每边 200ms */
  const uint32_t wheel_ms = 120U; /* mode7: 比 mode5 更快一点 */
  const uint32_t breath_ms = 2000U; /* mode6: 2s 一个呼吸周期 */

  static uint32_t t_mark = 0;
  static uint8_t flip = 0;

  switch (g_light_mode) {
    case 1: /* 近光：CH4 */
      Pwm_Set(0, base);
      break;
    case 2: /* 远光：CH3 */
      Pwm_Set(base, 0);
      break;
    case 3: /* 远近同亮 */
      Pwm_Set(base, base);
      break;
    case 4: /* 远光同步快闪 */
      if ((now - t_mark) >= fast_ms) { t_mark = now; flip = !flip; }
      Pwm_Set(flip ? base : 0, 0);
      break;
    case 5: /* 远近交替闪 */
      if ((now - t_mark) >= alt_ms) { t_mark = now; flip = !flip; }
      Pwm_Set(flip ? base : 0, flip ? 0 : base);
      break;
    case 6: /* 近光呼吸（三角波） */
    {
      uint32_t phase = now % breath_ms; /* 0..1999 */
      uint32_t tri = (phase < (breath_ms / 2U)) ? phase : (breath_ms - phase);
      /* tri: 0..1000..0，映射到 0..base */
      uint16_t pulse = (uint16_t)((base * tri) / (breath_ms / 2U));
      Pwm_Set(0, pulse);
      break;
    }
    case 7: /* 左右轮闪：两路轮流，节奏更快 */
      if ((now - t_mark) >= wheel_ms) { t_mark = now; flip = !flip; }
      Pwm_Set(flip ? base : 0, flip ? 0 : base);
      break;
    default:
      Pwm_Set(0, 0);
      break;
  }
}

static void DebugPrint(const char *s)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), 100);
}

static void DebugPrintHex(const uint8_t *buf, uint8_t len)
{
  char hex[3];
  uint8_t i;
  for(i = 0; i < len; i++) {
    sprintf(hex, "%02X", buf[i]);
    HAL_UART_Transmit(&huart1, (uint8_t*)hex, 2, 100);
  }
}

static void DebugPrintDec(uint16_t val)
{
  char buf[6];
  sprintf(buf, "%u", val);
  HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
}

/* ADC通道切换测试函数 */
static void Test_ADC_Channels(void)
{
  DebugPrint("[ADC-TEST] Starting channel switching test...\r\n");
  
    /* 测试1：连续读取同一通道多次 */
  DebugPrint("[ADC-TEST] Reading CH0 3 times:\r\n");
  {
    int i;
    for(i = 0; i < 3; i++) {
      uint32_t val = Read_ADC1_Channel(ADC_CHANNEL_0);
      if(val != 0xFFFFU) {
        DebugPrint("  CH0[");
        DebugPrintDec(i);
        DebugPrint("]=");
        DebugPrintDec((uint16_t)val);
        DebugPrint("\r\n");
      }
      HAL_Delay(100);
    }
  }
  
    /* 测试2：使用扫描模式读取双通道 */
  DebugPrint("[ADC-TEST] Reading dual channels (scan mode):\r\n");
  uint32_t ch0_val, ch1_val;
  if(Read_ADC1_DualChannel(&ch0_val, &ch1_val) == 0) {
    DebugPrint("  CH0=");
    DebugPrintDec((uint16_t)ch0_val);
    DebugPrint(" CH1=");
    DebugPrintDec((uint16_t)ch1_val);
    DebugPrint(" Diff=");
    if(ch0_val > ch1_val) {
      DebugPrintDec((uint16_t)(ch0_val - ch1_val));
    } else {
      DebugPrintDec((uint16_t)(ch1_val - ch0_val));
    }
    DebugPrint("\r\n");
  } else {
    DebugPrint("  Dual channel read failed\r\n");
  }
  
    /* 测试3：交替读取 */
  DebugPrint("[ADC-TEST] Alternating CH0/CH1 3 times:\r\n");
  {
    int i;
    for(i = 0; i < 3; i++) {
      uint32_t val0 = Read_ADC1_Channel(ADC_CHANNEL_0);
      HAL_Delay(50);
      uint32_t val1 = Read_ADC1_Channel(ADC_CHANNEL_1);
      HAL_Delay(50);
      
      if(val0 != 0xFFFFU && val1 != 0xFFFFU) {
        DebugPrint("  Iter[");
        DebugPrintDec(i);
        DebugPrint("]: CH0=");
        DebugPrintDec((uint16_t)val0);
        DebugPrint(" CH1=");
        DebugPrintDec((uint16_t)val1);
        DebugPrint("\r\n");
      }
    }
  }
  
  DebugPrint("[ADC-TEST] Test completed\r\n");
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
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
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
 *         LED_DRV(PA2) 用 TIM1_CH3 PWM，PB6 状态指示
 */
static void SyncLamp_Update(void)
{
  uint32_t now = HAL_GetTick();

  if(g_led_state == 1) {
    if((now - g_led_on_tick) >= SYNC_LED_ON_MS) {
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
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
  x ^= pkt[TLIGHT_PKT_SEQ_IDX];
  x ^= pkt[TLIGHT_PKT_DEV_ID_IDX];
  x ^= pkt[TLIGHT_PKT_TAG_IDX];
  return x;
}

static void BuildTempLightPacket(uint8_t *pkt, uint8_t mode, uint8_t bright)
{
  pkt[0] = TLIGHT_PKT_HEADER0;
  pkt[1] = TLIGHT_PKT_HEADER1;
  pkt[2] = mode;
  pkt[3] = bright;
  pkt[TLIGHT_PKT_SEQ_IDX] = g_link_tx_seq++;
  pkt[TLIGHT_PKT_DEV_ID_IDX] = g_dev_id_local;
  pkt[TLIGHT_PKT_TAG_IDX] = TLIGHT_PKT_TAG_CONST;
  pkt[TLIGHT_PKT_CHKSUM_IDX] = TempLight_CalcChecksum(pkt);
}

/* 解析临时灯光控制包：返回 1=合法，否则 0 */
static uint8_t ParseTempLightPacket(const uint8_t *pkt, uint8_t *mode, uint8_t *bright)
{
  if(pkt[0] != TLIGHT_PKT_HEADER0 || pkt[1] != TLIGHT_PKT_HEADER1) return 0;
  if(pkt[2] < TLIGHT_MODE_MIN || pkt[2] > TLIGHT_MODE_MAX) return 0;
  if(pkt[3] > TLIGHT_BRIGHT_100) return 0;
  if(pkt[TLIGHT_PKT_DEV_ID_IDX] != g_dev_id_paired) return 0;
  if(pkt[TLIGHT_PKT_TAG_IDX] != TLIGHT_PKT_TAG_CONST) return 0;
  if(pkt[TLIGHT_PKT_CHKSUM_IDX] != TempLight_CalcChecksum(pkt)) return 0;
  *mode = pkt[2];
  *bright = pkt[3];
  return 1;
}

static void LinkStats_Log(uint32_t now)
{
  if ((now - g_link_last_stats_tick) < LINK_STATS_LOG_INTERVAL_MS) {
    return;
  }
  g_link_last_stats_tick = now;

#if APP_ROLE_TX_ONLY
  DebugPrint("[TX-STAT] ok=");
  DebugPrintDec((uint16_t)(g_link_tx_ok & 0xFFFFU));
  DebugPrint(" fail=");
  DebugPrintDec((uint16_t)(g_link_tx_fail & 0xFFFFU));
  DebugPrint(" seq=");
  DebugPrintDec((uint16_t)g_link_tx_seq);
  DebugPrint(" dev=");
  DebugPrintDec((uint16_t)g_dev_id_local);
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
    /* HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); */
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

/**
 * @brief  指定通道读 ADC1 一次，返回 raw（失败返回 0xFFFF）
 *         PA0=ch0 太阳能板电压，PA1=ch1 电池电压
 *         积木1：STM32C0 需每次切换通道后再启动转换
 */
static uint32_t Read_ADC1_Channel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  
  /* 方案1：彻底停止并重新初始化ADC */
  HAL_ADC_Stop(&hadc1);
  HAL_ADC_DeInit(&hadc1);
  
  /* 重新初始化ADC */
  MX_ADC1_Init();
  
    /* 配置指定通道 */
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;                   /* STM32C0使用ADC_REGULAR_RANK_1 */
  /* 增加采样时间：从1.5周期增加到7.5周期，确保信号稳定 */
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    return 0xFFFFU;
  }
  
  /* 通道切换后增加短暂延时，让ADC输入稳定 */
  HAL_Delay(2);
  
  /* 清除所有ADC标志 */
  __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC | ADC_FLAG_OVR | ADC_FLAG_EOS | ADC_FLAG_EOSMP);
  
  /* 启动ADC */
  if (HAL_ADC_Start(&hadc1) != HAL_OK) {
    return 0xFFFFU;
  }
  
  /* 等待转换完成 */
  if (HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK) {
    HAL_ADC_Stop(&hadc1);
    return 0xFFFFU;
  }
  
  uint32_t raw = (uint32_t)HAL_ADC_GetValue(&hadc1);
  
  /* 停止ADC */
  HAL_ADC_Stop(&hadc1);
  
  return raw;
}

/**
 * @brief  扫描模式读取双通道ADC值（太阳能和电池）
 * @param  solar_raw: 太阳能电压ADC原始值（PA0/CH0）
 * @param  batt_raw: 电池电压ADC原始值（PA1/CH1）
 * @retval 0=成功，1=失败
 * 说明：使用ADC扫描模式，一次转换读取两个通道
 */
static uint8_t Read_ADC1_DualChannel(uint32_t* solar_raw, uint32_t* batt_raw)
{
  uint32_t adc_values[2] = {0};
  
  /* 确保ADC已停止 */
  HAL_ADC_Stop(&hadc1);
  
  /* 清除所有ADC标志 */
  __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC | ADC_FLAG_OVR | ADC_FLAG_EOS | ADC_FLAG_EOSMP);
  
  /* 启动ADC转换 */
  if (HAL_ADC_Start(&hadc1) != HAL_OK) {
    return 1;
  }
  
    /* 等待序列转换完成 */
  if (HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK) {
    HAL_ADC_Stop(&hadc1);
    return 1;
  }
  
  /* 读取两个通道的值 */
  /* 注意：在扫描模式下，HAL_ADC_GetValue()会返回当前转换的值 */
  /* 我们需要读取两次来获取两个通道的值 */
  adc_values[0] = HAL_ADC_GetValue(&hadc1);  /* 第一个转换：CH0 */
  
  /* 对于STM32C0，在扫描模式下，需要再次调用GetValue获取第二个通道 */
  /* 或者使用DMA/中断方式，这里我们简单处理 */
  adc_values[1] = HAL_ADC_GetValue(&hadc1);  /* 第二个转换：CH1 */
  
  /* 停止ADC */
  HAL_ADC_Stop(&hadc1);
  
  /* 返回结果 */
  *solar_raw = adc_values[0];
  *batt_raw = adc_values[1];
  
  return 0;
}

/* 日/夜检测：读 PA0 太阳能板电压，滞回 + 持续 3s 确认后翻转 (规格书 §5)，积木2 */
static void DayNight_Update(void)
{
  uint32_t now = HAL_GetTick();
  if((now - g_last_daynight_tick) < DAYNIGHT_SAMPLE_INTERVAL_MS) {
    return;
  }
  g_last_daynight_tick = now;

  uint32_t raw = Read_ADC1_Channel(ADC_CHANNEL_0);
  if(raw == 0xFFFFU) {
    return;
  }
  uint16_t adc_val = (uint16_t)raw;
  uint8_t prev = g_is_night;

    if(adc_val < SOLAR_ADC_NIGHT_THRESHOLD) {
    /* 条件满足「夜间」：需持续 3s 才从白天切到夜间 */
    if(g_is_night) {
      g_daynight_hold_until_tick = 0;
    } else {
      if(g_daynight_hold_until_tick == 0) {
        g_daynight_hold_until_tick = now + DAYNIGHT_HOLD_MS;
      } else if(now >= g_daynight_hold_until_tick) {
        /* 检测到从白天切换到夜间，启动调试窗口 */
        uint8_t prev_night = g_is_night;
        g_is_night = 1;
        g_daynight_hold_until_tick = 0;
        
        /* 记录夜间开始时间，启动6秒调试窗口 */
        if(prev_night == 0) {  /* 从白天切换到夜间 */
          g_night_start_tick = now;
          g_night_debug_window = 1;
          DebugPrint("[NIGHT] Debug window started (6s)\r\n");
        }
      }
    }
  } else if(adc_val > SOLAR_ADC_DAY_THRESHOLD) {
    /* 条件满足「白天」：需持续 3s 才从夜间切到白天 */
    if(!g_is_night) {
      g_daynight_hold_until_tick = 0;
    } else {
      if(g_daynight_hold_until_tick == 0) {
        g_daynight_hold_until_tick = now + DAYNIGHT_HOLD_MS;
      } else if(now >= g_daynight_hold_until_tick) {
        g_is_night = 0;
        g_daynight_hold_until_tick = 0;
        
        /* 切换到白天，关闭调试窗口 */
        g_night_debug_window = 0;
      }
    }
  } else {
    /* 滞回中间区：取消翻转确认 */
    g_daynight_hold_until_tick = 0;
  }

#if DEBUG_ADC_VERBOSE
  if(prev != g_is_night) {
    DebugPrint(g_is_night ? "->Night\r\n" : "->Day\r\n");
  }
  
  /* 检查是否在夜间调试窗口内（夜间开始后6秒） */
  if(g_is_night && g_night_debug_window) {
    uint32_t now = HAL_GetTick();
        if(now - g_night_start_tick <= 6000U) {  /* 6秒窗口 */
      /* 在调试窗口内：使用扫描模式同时采样两个通道 */
      uint32_t solar_raw, batt_raw;
      if(Read_ADC1_DualChannel(&solar_raw, &batt_raw) == 0) {
        DebugPrint("[NIGHT-DBG] solar=");
        DebugPrintDec((uint16_t)solar_raw);
        DebugPrint("(");
        
        uint32_t solar_mv = (solar_raw * 3300) / 4095;
        DebugPrintDec((uint16_t)(solar_mv / 1000));
        DebugPrint(".");
        DebugPrintDec((uint16_t)((solar_mv % 1000) / 100));
        DebugPrintDec((uint16_t)((solar_mv % 100) / 10));
        
        DebugPrint("V) batt=");
        DebugPrintDec((uint16_t)batt_raw);
        DebugPrint("(");
        
        uint32_t batt_mv = (batt_raw * 3300) / 4095;
        DebugPrintDec((uint16_t)(batt_mv / 1000));
        DebugPrint(".");
        DebugPrintDec((uint16_t)((batt_mv % 1000) / 100));
        DebugPrintDec((uint16_t)((batt_mv % 100) / 10));
        
        DebugPrint("V)\r\n");
      }
    } else {
      /* 6秒窗口结束 */
      g_night_debug_window = 0;
      DebugPrint("[NIGHT] Debug window ended\r\n");
    }
  } else {
    /* 不在调试窗口：正常单通道采样显示 */
    static uint8_t dn_cnt = 0;
    if(++dn_cnt >= 5) {
      dn_cnt = 0;
      DebugPrint("[ADC] solar=");
      DebugPrintDec(adc_val);
      DebugPrint("(");
      
      uint32_t voltage_mv = (adc_val * 3300) / 4095;
      DebugPrintDec((uint16_t)(voltage_mv / 1000));
      DebugPrint(".");
      DebugPrintDec((uint16_t)((voltage_mv % 1000) / 100));
      DebugPrintDec((uint16_t)((voltage_mv % 100) / 10));
      
      DebugPrint("V) night=");
      DebugPrint(g_is_night ? "1" : "0");
      if(g_daynight_hold_until_tick != 0) DebugPrint(" hold");
      DebugPrint("\r\n");
    }
  }
#else
  if(prev != g_is_night) {
    DebugPrint(g_is_night ? "->Night\r\n" : "->Day\r\n");
  }
#endif
}

/* 充电控制：读 PA1 电池电压，过充 1.55V 关断、1.35V 滞回重开，积木3：太阳能预判+过充二次确认 */
static void Charge_Update(void)
{
  uint32_t now = HAL_GetTick();
  if((now - g_last_charge_tick) < CHARGE_SAMPLE_INTERVAL_MS) {
    return;
  }
  g_last_charge_tick = now;

  /* 仅在“有太阳能输入”时检查过充，夜间放电时跳过以减少 ADC 次数 */
  uint32_t solar_raw = Read_ADC1_Channel(ADC_CHANNEL_0);
  if(solar_raw == 0xFFFFU) return;
  if(solar_raw <= SOLAR_ADC_DAY_THRESHOLD) {
    return;  /* 太阳能电压不高，认为未在充电，跳过过充判断 */
  }

  uint32_t raw = Read_ADC1_Channel(ADC_CHANNEL_1);
  if(raw == 0xFFFFU) return;
  uint16_t adc_val = (uint16_t)raw;

  /* 过充二次确认：连续两次采样都 >= 阈值才关断，避免误触发 */
  static uint8_t overcharge_pending = 0;

  if(adc_val >= BATT_ADC_OVERCHARGE_RAW) {
    if(!overcharge_pending) {
      overcharge_pending = 1;
#if DEBUG_ADC_VERBOSE
      DebugPrint("[CHG] batt=");
      DebugPrintDec(adc_val);
      DebugPrint("(");
      
      /* 计算电池电压值（假设3.3V参考电压，12位ADC） */
      uint32_t batt_voltage_mv = (adc_val * 3300) / 4095;
      DebugPrintDec((uint16_t)(batt_voltage_mv / 1000));  /* 整数部分 */
      DebugPrint(".");
      DebugPrintDec((uint16_t)((batt_voltage_mv % 1000) / 100));  /* 小数第一位 */
      DebugPrintDec((uint16_t)((batt_voltage_mv % 100) / 10));    /* 小数第二位 */
      
      DebugPrint("V) pending\r\n");
#endif
      return;
    }
    overcharge_pending = 0;
    HAL_GPIO_WritePin(CHG_MOS_GPIO_Port, CHG_MOS_Pin, GPIO_PIN_SET);  /* 过充：高电平关断 */
#if DEBUG_ADC_VERBOSE
    DebugPrint("[CHG] overcharge OFF batt=");
    DebugPrintDec(adc_val);
    DebugPrint("(");
    
    /* 计算电池电压值 */
    uint32_t batt_voltage_mv = (adc_val * 3300) / 4095;
    DebugPrintDec((uint16_t)(batt_voltage_mv / 1000));
    DebugPrint(".");
    DebugPrintDec((uint16_t)((batt_voltage_mv % 1000) / 100));
    DebugPrintDec((uint16_t)((batt_voltage_mv % 100) / 10));
    
    DebugPrint("V)\r\n");
#endif
  } else if(adc_val <= BATT_ADC_REENABLE_RAW) {
    overcharge_pending = 0;
    HAL_GPIO_WritePin(CHG_MOS_GPIO_Port, CHG_MOS_Pin, GPIO_PIN_RESET);  /* 滞回：重新开启充电 */
#if DEBUG_ADC_VERBOSE
    DebugPrint("[CHG] reenable ON batt=");
    DebugPrintDec(adc_val);
    DebugPrint("(");
    
    /* 计算电池电压值 */
    uint32_t batt_voltage_mv = (adc_val * 3300) / 4095;
    DebugPrintDec((uint16_t)(batt_voltage_mv / 1000));
    DebugPrint(".");
    DebugPrintDec((uint16_t)((batt_voltage_mv % 1000) / 100));
    DebugPrintDec((uint16_t)((batt_voltage_mv % 100) / 10));
    
    DebugPrint("V)\r\n");
#endif
  }
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
