#ifndef __XL2400T_H
#define __XL2400T_H

/**
 * XL2400T 3-wire software SPI driver (port from vendor demo)
 * Pins (mapped to current board):
 *   CSN  -> PA4 (RF_CSN_Pin)
 *   SCK  -> PA5 (RF_SCK_Pin)
 *   DATA -> PA7 (RF_DATA_Pin)
 */

#include "main.h"

/* DATA/SCK/CSN pins and ports */
#define  RF_SPI_DATA_PORT        RF_DATA_GPIO_Port
#define  RF_SPI_DATA_PIN         RF_DATA_Pin

#define  RF_SPI_SCK_PORT         RF_SCK_GPIO_Port
#define  RF_SPI_SCK_PIN          RF_SCK_Pin

#define  RF_SPI_CSN_PORT         RF_CSN_GPIO_Port
#define  RF_SPI_CSN_PIN          RF_CSN_Pin

/* GPIO clocks (all on GPIOA in this project) */
#define  RF_SPI_DATA_CLK_ENABLE  __HAL_RCC_GPIOA_CLK_ENABLE()
#define  RF_SPI_SCK_CLK_ENABLE   __HAL_RCC_GPIOA_CLK_ENABLE()
#define  RF_SPI_CSN_CLK_ENABLE   __HAL_RCC_GPIOA_CLK_ENABLE()

/* Basic pin operations */
#define CSN_Low()    HAL_GPIO_WritePin(RF_SPI_CSN_PORT, RF_SPI_CSN_PIN, GPIO_PIN_RESET)
#define CSN_High()   HAL_GPIO_WritePin(RF_SPI_CSN_PORT, RF_SPI_CSN_PIN, GPIO_PIN_SET)

#define SCK_High()   HAL_GPIO_WritePin(RF_SPI_SCK_PORT, RF_SPI_SCK_PIN, GPIO_PIN_SET)
#define SCK_Low()    HAL_GPIO_WritePin(RF_SPI_SCK_PORT, RF_SPI_SCK_PIN, GPIO_PIN_RESET)

#define SPI_DATA_High()  HAL_GPIO_WritePin(RF_SPI_DATA_PORT, RF_SPI_DATA_PIN, GPIO_PIN_SET)
#define SPI_DATA_Low()   HAL_GPIO_WritePin(RF_SPI_DATA_PORT, RF_SPI_DATA_PIN, GPIO_PIN_RESET)

#define Read_DATA()      HAL_GPIO_ReadPin(RF_SPI_DATA_PORT, RF_SPI_DATA_PIN)

/* SPI helper APIs */
void XL2400T_SPI_Init(void);
unsigned char RF_SPI_Read_Reg(unsigned char RF_Reg);
void RF_SPI_Write_Reg(unsigned char RF_Reg, unsigned char W_Data);
void RF_Read_Buff(unsigned char RF_Reg , unsigned char *pBuff , unsigned char Len);
void RF_Write_Buff(unsigned char RF_Reg , unsigned char *pBuff , unsigned char Len);

/* Register addresses */
#define CFG_TOP             (0x00)
#define EN_AA               (0x01)
#define EN_RXADDR           (0x02)
#define SETUP_AW            (0x03)
#define SETUP_RETR          (0x04)
#define RF_CH               (0x05)
#define RF_SETUP            (0x06)
#define RF_STATUS           (0x07)
#define OBSERVE_TX          (0x08)
#define RSSI                (0x09)
#define RX_ADDR_P0          (0x0A)
#define RX_ADDR_P1          (0x0B)
#define RX_ADDR_P2TOP5      (0x0C)
#define BER_RESULT          (0x0D)
#define AGC_SETTING         (0x0E)
#define PGA_SETTING         (0x0F)
#define TX_ADDR             (0x10)
#define RX_PW_PX            (0x11)
#define ANALOG_CFG0         (0x12)
#define ANALOG_CFG1         (0x13)
#define ANALOG_CFG2         (0x14)
#define ANALOG_CFG3         (0x15)
#define STATUS_FIFO         (0x17)
#define RSSIREC             (0x18)
#define TXPROC_CFG          (0x19)
#define RXPROC_CFG          (0x1A)
#define DYNPD               (0x1C)
#define FEATURE             (0x1D)
#define RAMP_CFG            (0x1E)

/* SPI commands */
#define R_REGISTER          0x00
#define W_REGISTER          0x20
#define R_RX_PLOAD          0x61
#define W_TX_PLOAD          0xA0
#define FLUSH_TX            0xE1
#define FLUSH_RX            0xE2
#define R_RX_PL_WID         0x60
#define W_ACK_PLOAD         0xA8
#define W_TX_PLOAD_NOACK    0xB0
#define CMD_NOP             0xFF

/* Status flags */
#define RX_DR               (0x40)
#define TX_DS               (0x20)
#define MAX_RT              (0x10)

/* Data rates */
#define C_DR_1M             0x02
#define C_DR_250K           0x22

/* TX power table (XL2400T) */
#define C_RF13dBm           36
#define C_RF12dBm           30
#define C_RF11dBm           24
#define C_RF10dBm           19
#define C_RF9dBm            16
#define C_RF8dBm            14
#define C_RF7dBm            12
#define C_RF5dBm            9
#define C_RF3dBm            8
#define C_RF0dBm            6
#define C_RF_4dBm           4
#define C_RF_9dBm           2
#define C_RF_10dBm          1

/* High-level RF helpers */
void RF_Reset(void);
void RF_Sleep(void);
void RF_WakeUp(void);  /* 添加唤醒函数 */
void RF_Rx_Mode(void);
void RF_Tx_Mode(void);
void RF_Set_Chn(unsigned char Chn);
void RF_Set_Power(unsigned char Power);
void RF_Refresh_State(void);
void XL2400T_Init(void);
unsigned char RF_TX_Data(unsigned char* tx_buff);
unsigned char RF_RX_Data(unsigned char* rx_buff);

/* User configuration */
#define RF_TX_Power         C_RF13dBm   /* 最大功率 13dBm，增强传输距离 */
#define RF_BAUD             C_DR_250K
#define RF_PACKET_SIZE      8

#endif /* __XL2400T_H */