#ifndef __RF_XL2400_H__
#define __RF_XL2400_H__

/**
 * 通用 XL2400 无线接口（基于 XL2400T Demo 封装）
 *
 * 设计目标：
 * - 上层只关心"初始化 / 设通道 / 发包 / 收包"
 * - 底层可以是当前 XL2400T 软件 SPI，也可以未来切换为硬件 SPI 或其他芯片
 *
 * 约定：
 * - 当前实现使用固定长度 8 字节负载（RF_PACKET_SIZE）
 * - 发送长度小于 8 字节时自动零填充；大于 8 字节时只发送前 8 字节
 */

#include "stdint.h"

/* 角色（目前仅用于上层语义标识） */
#define RF_ROLE_TX   1
#define RF_ROLE_RX   0

/* 初始化 RF（SPI + XL2400T 寄存器） */
void RF_Link_Init(void);

/* 配置为 TX/RX 模式并设置通道（channel 为 XL2400T 的信道号 0~80） */
void RF_Link_ConfigTx(uint8_t channel);
void RF_Link_ConfigRx(uint8_t channel);

/* 低功耗：让 RF 进入睡眠/待机（底层写 CFG_TOP=0x00） */
void RF_Link_Sleep(void);

/**
 * 发送一个固定长度数据包（最多 8 字节）。
 *
 * @param buf  待发送数据缓冲区
 * @param len  待发送数据长度（<=8），超过会被截断
 * @return 0 表示发送成功，负值表示失败
 */
int RF_Link_Send(const uint8_t *buf, uint8_t len);

/**
 * 轮询接收（非阻塞）。
 *
 * @param buf  用于存放接收到的数据（至少 8 字节）
 * @param len  实际接收到的长度（当前实现恒为 8）
 * @return 1 表示收到新数据并已写入 buf，0 表示暂无数据
 */
int RF_Link_PollReceive(uint8_t *buf, uint8_t *len);

#endif /* __RF_XL2400_H__ */