#include "XL2400T.h"

/* GPIO init for 3-wire software SPI */
static void RF_SPI_GPIO_Init(void)
{
  RF_SPI_DATA_CLK_ENABLE;
  RF_SPI_SCK_CLK_ENABLE;
  RF_SPI_CSN_CLK_ENABLE;

  GPIO_InitTypeDef GPIO_InitStruct;

  /* DATA as output */
  GPIO_InitStruct.Pin   = RF_SPI_DATA_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RF_SPI_DATA_PORT, &GPIO_InitStruct);

  /* SCK as output */
  GPIO_InitStruct.Pin   = RF_SPI_SCK_PIN;
  HAL_GPIO_Init(RF_SPI_SCK_PORT, &GPIO_InitStruct);

  /* CSN as output */
  GPIO_InitStruct.Pin   = RF_SPI_CSN_PIN;
  HAL_GPIO_Init(RF_SPI_CSN_PORT, &GPIO_InitStruct);
}

static void RF_SPI_Data_OutPut(void)
{
  RF_SPI_DATA_CLK_ENABLE;

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin   = RF_SPI_DATA_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RF_SPI_DATA_PORT, &GPIO_InitStruct);
}

static void RF_SPI_Data_InPut(void)
{
  RF_SPI_DATA_CLK_ENABLE;

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin   = RF_SPI_DATA_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(RF_SPI_DATA_PORT, &GPIO_InitStruct);
}

void XL2400T_SPI_Init(void)
{
  RF_SPI_GPIO_Init();
  CSN_High();
  SCK_Low();
}

/* Simple delay helpers (from vendor demo) */
void RF_DelayUs(unsigned char Time)
{
  volatile unsigned char a;
  while (Time--) {
    for (a = 0; a < 50; a++) {
      __NOP();
    }
  }
}

void RF_DelayMs(unsigned char Time)
{
  while (Time--) {
    RF_DelayUs(200);
  }
}

/* Software SPI byte transfer */
static void RF_SPI_Write_Byte(unsigned char buff)
{
  unsigned char i;

  RF_SPI_Data_OutPut();

  for (i = 0; i < 8; i++) {
    SCK_Low();
    if (buff & 0x80) {
      SPI_DATA_High();
    } else {
      SPI_DATA_Low();
    }
    buff <<= 1;
    SCK_High();
  }
  SPI_DATA_High();
  SCK_Low();
}

static unsigned char RF_SPI_Read_Byte(void)
{
  unsigned char buff = 0;
  unsigned char i;

  RF_SPI_Data_InPut();

  for (i = 0; i < 8; i++) {
    SCK_Low();
    buff <<= 1;
    SCK_High();
    if (Read_DATA()) {
      buff |= 0x01;
    }
  }
  SCK_Low();
  return buff;
}

void RF_SPI_Write_Reg(unsigned char RF_Reg, unsigned char W_Data)
{
  CSN_Low();
  RF_SPI_Write_Byte(RF_Reg);
  RF_SPI_Write_Byte(W_Data);
  CSN_High();
}

unsigned char RF_SPI_Read_Reg(unsigned char RF_Reg)
{
  unsigned char rTemp = 0;
  CSN_Low();
  RF_SPI_Write_Byte(RF_Reg);
  rTemp = RF_SPI_Read_Byte();
  CSN_High();
  return rTemp;
}

void RF_Write_Buff(unsigned char RF_Reg, unsigned char *pBuff, unsigned char Len)
{
  unsigned char i;
  CSN_Low();
  RF_SPI_Write_Byte(RF_Reg);
  for (i = 0; i < Len; i++) {
    RF_SPI_Write_Byte(pBuff[i]);
  }
  CSN_High();
}

void RF_Read_Buff(unsigned char RF_Reg, unsigned char *pBuff, unsigned char Len)
{
  unsigned char i;
  CSN_Low();
  RF_SPI_Write_Byte(RF_Reg);
  for (i = 0; i < Len; i++) {
    pBuff[i] = RF_SPI_Read_Byte();
  }
  CSN_High();
}

/* RF helpers (from vendor demo) */
void RF_CE_High(void)
{
  RF_SPI_Write_Reg(W_REGISTER + CFG_TOP, 0xEF);
}

void RF_CE_Low(void)
{
  RF_SPI_Write_Reg(W_REGISTER + CFG_TOP, 0xEE);
}

void RF_Refresh_State(void)
{
  RF_SPI_Write_Reg(W_REGISTER + RF_STATUS, 0x70);
  RF_SPI_Write_Reg(FLUSH_TX, CMD_NOP);
  RF_SPI_Write_Reg(FLUSH_RX, CMD_NOP);
}

void RF_Reset(void)
{
  RF_SPI_Write_Reg(W_REGISTER + CFG_TOP, 0xEA);
  RF_DelayUs(200);
  RF_SPI_Write_Reg(W_REGISTER + CFG_TOP, 0xEE);
  RF_DelayMs(1);
}

void RF_Set_Chn(unsigned char Chn)
{
  unsigned temp = 0;
  temp = RF_SPI_Read_Reg(R_REGISTER + EN_AA);
  temp &= ~(1 << 6);
  RF_SPI_Write_Reg(W_REGISTER + EN_AA, temp);

  RF_SPI_Write_Reg(W_REGISTER + RF_CH, Chn + 0x60);

  temp = RF_SPI_Read_Reg(R_REGISTER + EN_AA);
  temp |= (1 << 6);
  RF_SPI_Write_Reg(W_REGISTER + EN_AA, temp);
}

void RF_Set_Address(unsigned char *AddrBuff)
{
  RF_Write_Buff(W_REGISTER + TX_ADDR, AddrBuff, 5);
  RF_Write_Buff(W_REGISTER + RX_ADDR_P0, AddrBuff, 5);
}

void RF_Set_Power(unsigned char Power)
{
  unsigned char Power_Buff[3] = {0};
  RF_Read_Buff(R_REGISTER + RF_SETUP, Power_Buff, 2);
  Power_Buff[1] = Power;
  RF_Write_Buff(W_REGISTER + RF_SETUP, Power_Buff, 2);
}

void RF_Sleep(void)
{
  RF_SPI_Write_Reg(W_REGISTER + CFG_TOP, 0x00);
}

void RF_WakeUp(void)
{
  /* 从睡眠模式唤醒，重新设置基本配置 */
  RF_SPI_Write_Reg(W_REGISTER + CFG_TOP, 0xEE);
}

void RF_Tx_Mode(void)
{
  unsigned char Mode_Buff[3] = {0};
  Mode_Buff[0] = 0xEE;
  Mode_Buff[1] = 0x80;
  RF_Write_Buff(W_REGISTER + CFG_TOP, Mode_Buff, 2);
  RF_Refresh_State();
  RF_DelayMs(1);
}

void RF_Rx_Mode(void)
{
  unsigned char Mode_Buff[3] = {0};
  Mode_Buff[0] = 0xEE;
  Mode_Buff[1] = 0xC0;
  RF_Write_Buff(W_REGISTER + CFG_TOP, Mode_Buff, 2);
  RF_Refresh_State();
  RF_CE_High();
  RF_DelayMs(1);
}

static unsigned char RF_Test_Adrress[5] = {0xCC,0xCC,0xCC,0xCC,0xCC};

void XL2400T_Init(void)
{
  /* Common config */
  RF_SPI_Write_Reg(W_REGISTER + CFG_TOP, 0x02);
  RF_DelayMs(2);
  RF_SPI_Write_Reg(W_REGISTER + CFG_TOP, 0x3E);
  RF_DelayMs(2);

  /* Analog config */
  unsigned char gRfBuffer[8] = {0};
  RF_Read_Buff(PGA_SETTING, gRfBuffer, 5);
  gRfBuffer[5] = ((gRfBuffer[5] & 0xFF) | 0x6D);
  RF_DelayMs(1);
  gRfBuffer[0] = 0x44;
  gRfBuffer[1] = 0x3E;
  gRfBuffer[2] = 0x38;
  gRfBuffer[3] = 0x32;
  gRfBuffer[4] = 0x2A;
  RF_Write_Buff(W_REGISTER + PGA_SETTING, gRfBuffer, 5);
  RF_DelayMs(1);

  /* Address length & address */
  RF_SPI_Write_Reg(W_REGISTER + SETUP_AW, 0xAF);
  RF_Set_Address(RF_Test_Adrress);

  /* Data rate: 1Mbps (v1.8.0 优化，比 250Kbps 快 4 倍) */
  RF_SPI_Write_Reg(W_REGISTER + RF_SETUP, C_DR_1M);

  /* RX params */
  RF_SPI_Write_Reg(W_REGISTER + EN_RXADDR, 0x01);
  RF_SPI_Write_Reg(W_REGISTER + RX_PW_PX, RF_PACKET_SIZE);
  RF_SPI_Write_Reg(W_REGISTER + EN_AA, 0x00);
  RF_SPI_Write_Reg(W_REGISTER + DYNPD, 0x00);
  RF_SPI_Write_Reg(W_REGISTER + FEATURE, 0x18);

  /* TX params */
  RF_SPI_Write_Reg(W_REGISTER + SETUP_RETR, 0x33);
  RF_Set_Power(RF_TX_Power);
}

unsigned char RF_TX_Data(unsigned char* tx_buff)
{
  RF_Refresh_State();

  RF_Write_Buff(W_TX_PLOAD, tx_buff, RF_PACKET_SIZE);

  RF_CE_High();
  RF_DelayUs(100);
  RF_CE_Low();

#if RF_TX_USE_FIXED_DELAY_MODE
  /* 兼容旧逻辑：固定等待，简单但会阻塞主循环 */
  RF_DelayMs((unsigned char)RF_TX_POST_CE_FIXED_DELAY_MS);
#else
  /* 快速模式：轮询 TX_DS/MAX_RT，减少阻塞时间 */
  {
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < RF_TX_STATUS_WAIT_TIMEOUT_MS) {
      unsigned char st = RF_SPI_Read_Reg(R_REGISTER + RF_STATUS);
      if (st & (TX_DS | MAX_RT)) {
        break;
      }
    }
  }
#endif

  if (RF_SPI_Read_Reg(R_REGISTER + RF_STATUS) & TX_DS) {
    RF_Refresh_State();
    return 0x20;
  } else {
    RF_Refresh_State();
    return 0;
  }
}

unsigned char RF_RX_Data(unsigned char* rx_buff)
{
  unsigned char RfPlwid = 0;
  if (RF_SPI_Read_Reg(RF_STATUS) & RX_DR) {
    RF_CE_Low();
    RfPlwid = RF_SPI_Read_Reg(R_RX_PL_WID);
    RF_Read_Buff(R_RX_PLOAD, rx_buff, RfPlwid);
    RF_Refresh_State();
    RF_CE_High();
    return 1;
  } else {
    return 0;
  }
}