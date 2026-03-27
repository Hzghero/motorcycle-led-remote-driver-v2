#include "rf_xl2400.h"
#include "XL2400T.h"

/* 内部发送/接收缓冲区，长度与 Demo 保持一致 */
static uint8_t rf_tx_buf[RF_PACKET_SIZE];
static uint8_t rf_rx_buf[RF_PACKET_SIZE];

void RF_Link_Init(void)
{
  /* 底层 3 线 SPI + XL2400T 参数初始化 */
  XL2400T_SPI_Init();
  XL2400T_Init();
}

void RF_Link_ConfigTx(uint8_t channel)
{
  RF_Tx_Mode();
  RF_Set_Chn(channel);
}

void RF_Link_ConfigRx(uint8_t channel)
{
  RF_Rx_Mode();
  RF_Set_Chn(channel);
}

void RF_Link_Sleep(void)
{
  RF_Sleep();
}

int RF_Link_Send(const uint8_t *buf, uint8_t len)
{
  uint8_t i;
  if (buf == NULL) return -1;

  /* 复制并按需截断/填充到固定长度 8 字节 */
  for (i = 0; i < RF_PACKET_SIZE; i++) {
    if (i < len) {
      rf_tx_buf[i] = buf[i];
    } else {
      rf_tx_buf[i] = 0;
    }
  }

  /* 调用 Demo 的发送函数，返回 0x20 表示 TX_DS 成功 */
  return (RF_TX_Data(rf_tx_buf) == 0x20) ? 0 : -2;
}

int RF_Link_PollReceive(uint8_t *buf, uint8_t *len)
{
  if (buf == NULL || len == NULL) return -1;

  if (RF_RX_Data(rf_rx_buf) == 1) {
    uint8_t i;
    for (i = 0; i < RF_PACKET_SIZE; i++) {
      buf[i] = rf_rx_buf[i];
    }
    *len = RF_PACKET_SIZE;
    return 1;
  }

  return 0;
}