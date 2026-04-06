#include "app_key.h"
#include "main.h"

/* ============ 可调参数（按项目需求固定） ============ */
#define APP_KEY_DEBOUNCE_MS     (20u)     /* 消抖：连续稳定 20ms，减轻短按被判成双击 */
#define APP_KEY_LONG_MS         (1000u)   /* 长按阈值：1s */
#define APP_KEY_DOUBLE_MS       (200u)    /* 双击窗口：200ms（你已确认） */

#define APP_KEY_EVT_QUEUE_SIZE  (16u)     /* 事件队列深度：够用即可 */

typedef enum
{
  KEY_ST_IDLE = 0,
  KEY_ST_WAIT_SECOND,
} KeyLogicState;

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
  AppKeyId id;

  /* 采样与消抖 */
  uint8_t raw_level;         /* 当前采样电平（0/1） */
  uint8_t stable_level;      /* 消抖后的稳定电平（0/1） */
  uint8_t last_raw_level;
  uint16_t stable_cnt_ms;    /* 连续稳定计数 */

  /* 逻辑状态 */
  KeyLogicState logic;
  uint8_t long_fired;
  uint8_t suppress_next_release; /* 双击在“第二次按下”已触发：抑制下一次松开进入单击逻辑 */
  uint32_t t_press_ms;       /* 稳定按下时刻 */
  uint32_t t_first_rel_ms;   /* 第一次松开时刻（用于单击/双击判定） */
  uint16_t last_down_ms;     /* 最近一次按下时长（松开时更新） */
} KeyCtx;

static KeyCtx s_keys[APP_KEY_ID_MAX];

static AppKeyEvent s_evt_q[APP_KEY_EVT_QUEUE_SIZE];
static volatile uint8_t s_evt_w = 0;
static volatile uint8_t s_evt_r = 0;

static uint8_t Key_ReadRaw(const KeyCtx *k)
{
  GPIO_PinState st = HAL_GPIO_ReadPin(k->port, k->pin);
  return (st == GPIO_PIN_SET) ? 1u : 0u;
}

static void Evt_Push(AppKeyId id, AppKeyEventType type, uint32_t now_ms, uint16_t down_ms, uint16_t gap_ms)
{
  uint8_t next = (uint8_t)((s_evt_w + 1u) % APP_KEY_EVT_QUEUE_SIZE);
  if (next == s_evt_r) {
    /* 队列满：丢弃最旧，保证新事件进入（调试场景更友好） */
    s_evt_r = (uint8_t)((s_evt_r + 1u) % APP_KEY_EVT_QUEUE_SIZE);
  }

  s_evt_q[s_evt_w].id = id;
  s_evt_q[s_evt_w].type = type;
  s_evt_q[s_evt_w].t_ms = now_ms;
  s_evt_q[s_evt_w].down_ms = down_ms;
  s_evt_q[s_evt_w].gap_ms = gap_ms;
  s_evt_w = next;
}

static void Key_OnStableEdge(KeyCtx *k, uint8_t new_stable_level, uint32_t now_ms)
{
  /* new_stable_level: 1=稳定按下, 0=稳定松开 */
  if (new_stable_level) {
    k->t_press_ms = now_ms;
    k->long_fired = 0u;
    /* 方案B（双击优先）：等待窗口内捕获到“第二次按下”就立刻触发双击 */
    if (k->logic == KEY_ST_WAIT_SECOND) {
      if (now_ms >= k->t_first_rel_ms) {
        uint32_t g = now_ms - k->t_first_rel_ms;
        if (g <= APP_KEY_DOUBLE_MS) {
          Evt_Push(k->id, APP_KEY_EVT_DOUBLE_CLICK, now_ms, 0, (uint16_t)g);
          k->logic = KEY_ST_IDLE;
          k->suppress_next_release = 1u;
        } else {
          /* 超出双击窗口：上一击确定为单击，本次按下当作新的开始 */
          Evt_Push(k->id, APP_KEY_EVT_SINGLE_CLICK, k->t_first_rel_ms, 0, 0);
          k->logic = KEY_ST_IDLE;
        }
      } else {
        /* tick 回绕极少见：直接复位逻辑 */
        k->logic = KEY_ST_IDLE;
      }
    }
    return;
  }

  /* 稳定松开 */
  if (k->suppress_next_release) {
    /* 双击已触发：忽略这次松开，避免把它当成新的一击 */
    k->suppress_next_release = 0u;
    k->logic = KEY_ST_IDLE;
    return;
  }

  if (now_ms >= k->t_press_ms) {
    uint32_t d = now_ms - k->t_press_ms;
    if (d > 0xFFFFu) d = 0xFFFFu;
    k->last_down_ms = (uint16_t)d;
  } else {
    k->last_down_ms = 0;
  }

  if (k->long_fired) {
    /* 长按已触发：松开不再产生单击/双击 */
    k->logic = KEY_ST_IDLE;
    return;
  }

  if (k->logic == KEY_ST_IDLE) {
    k->logic = KEY_ST_WAIT_SECOND;
    k->t_first_rel_ms = now_ms;
    return;
  }

  /* 方案B：第二次松开不用于判定双击（双击已在第二次按下判定） */
}

void AppKey_Init(void)
{
  s_keys[APP_KEY_ID_L].port = L_KEY_GPIO_Port;
  s_keys[APP_KEY_ID_L].pin = L_KEY_Pin;
  s_keys[APP_KEY_ID_L].id = APP_KEY_ID_L;

  s_keys[APP_KEY_ID_M].port = M_KEY_GPIO_Port;
  s_keys[APP_KEY_ID_M].pin = M_KEY_Pin;
  s_keys[APP_KEY_ID_M].id = APP_KEY_ID_M;

  s_keys[APP_KEY_ID_R].port = R_KEY_GPIO_Port;
  s_keys[APP_KEY_ID_R].pin = R_KEY_Pin;
  s_keys[APP_KEY_ID_R].id = APP_KEY_ID_R;

  int i;
  for (i = 0; i < (int)APP_KEY_ID_MAX; i++) {
    KeyCtx *k = &s_keys[i];
    uint8_t lv = Key_ReadRaw(k);
    k->raw_level = lv;
    k->last_raw_level = lv;
    k->stable_level = lv;
    k->stable_cnt_ms = 0;
    k->logic = KEY_ST_IDLE;
    k->long_fired = 0;
    k->suppress_next_release = 0;
    k->t_press_ms = 0;
    k->t_first_rel_ms = 0;
    k->last_down_ms = 0;
  }

  s_evt_w = 0;
  s_evt_r = 0;
}

void AppKey_Tick1ms(uint32_t now_ms)
{
  int i;
  for (i = 0; i < (int)APP_KEY_ID_MAX; i++) {
    KeyCtx *k = &s_keys[i];
    uint8_t lv = Key_ReadRaw(k);
    k->raw_level = lv;

    if (lv == k->last_raw_level) {
      if (k->stable_cnt_ms < 0xFFFFu) {
        k->stable_cnt_ms++;
      }
    } else {
      k->stable_cnt_ms = 0;
      k->last_raw_level = lv;
    }

    /* 达到消抖要求后，承认稳定变化 */
    if (k->stable_level != lv && k->stable_cnt_ms >= APP_KEY_DEBOUNCE_MS) {
      k->stable_level = lv;
      Key_OnStableEdge(k, lv, now_ms);
    }

    /* 长按判定：按住满 1s 立即触发一次
     * 规则补丁：双击在第二次按下已触发后（未松手前），本次按住不再触发长按
     */
    if (k->stable_level && !k->long_fired && !k->suppress_next_release) {
      if (now_ms >= k->t_press_ms) {
        uint32_t d = now_ms - k->t_press_ms;
        if (d >= APP_KEY_LONG_MS) {
          k->long_fired = 1u;
          Evt_Push(k->id, APP_KEY_EVT_LONG_PRESS_1S, now_ms, (uint16_t)APP_KEY_LONG_MS, 0);
          k->logic = KEY_ST_IDLE;
        }
      }
    }

    /* 单击超时：等待第二次点击窗口过期 -> 产生单击 */
    if (k->logic == KEY_ST_WAIT_SECOND) {
      if (now_ms >= k->t_first_rel_ms) {
        uint32_t g = now_ms - k->t_first_rel_ms;
        if (g > APP_KEY_DOUBLE_MS) {
          Evt_Push(k->id, APP_KEY_EVT_SINGLE_CLICK, k->t_first_rel_ms, 0, 0);
          k->logic = KEY_ST_IDLE;
        }
      } else {
        /* tick 回绕极少见：直接复位逻辑 */
        k->logic = KEY_ST_IDLE;
      }
    }
  }
}

int AppKey_PopEvent(AppKeyEvent *evt)
{
  if (evt == NULL) return 0;
  if (s_evt_r == s_evt_w) return 0;

  *evt = s_evt_q[s_evt_r];
  s_evt_r = (uint8_t)((s_evt_r + 1u) % APP_KEY_EVT_QUEUE_SIZE);
  return 1;
}

const char *AppKey_Name(AppKeyId id)
{
  switch (id) {
    case APP_KEY_ID_L: return "L";
    case APP_KEY_ID_M: return "M";
    case APP_KEY_ID_R: return "R";
    default: return "?";
  }
}

AppKeyId AppKey_IdFromPin(uint16_t gpio_pin)
{
  if (gpio_pin == L_KEY_Pin) return APP_KEY_ID_L;
  if (gpio_pin == M_KEY_Pin) return APP_KEY_ID_M;
  if (gpio_pin == R_KEY_Pin) return APP_KEY_ID_R;
  return APP_KEY_ID_MAX;
}

const char *AppKey_NameFromPin(uint16_t gpio_pin)
{
  AppKeyId id = AppKey_IdFromPin(gpio_pin);
  if (id >= APP_KEY_ID_MAX) {
    return "?";
  }
  return AppKey_Name(id);
}

const char *AppKey_EventTypeStr(AppKeyEventType t)
{
  switch (t) {
    case APP_KEY_EVT_SINGLE_CLICK: return "SINGLE";
    case APP_KEY_EVT_DOUBLE_CLICK: return "DOUBLE";
    case APP_KEY_EVT_LONG_PRESS_1S: return "LONG1S";
    default: return "?";
  }
}

int AppKey_CanEnterStop(void)
{
  int i;
  for (i = 0; i < (int)APP_KEY_ID_MAX; i++) {
    const KeyCtx *k = &s_keys[i];
    if (Key_ReadRaw(k)) {
      return 0;
    }
    if (k->logic == KEY_ST_WAIT_SECOND) {
      return 0;
    }
  }
  return 1;
}

