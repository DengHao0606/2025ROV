#ifndef __RS485_PROCESS_H
#define __RS485_PROCESS_H

#include <stdint.h>
#include <string.h>
#include "usart.h"

#define USARTx                          UART7                // 串口端口
#define USARTx_CLK_ENABLE()             __HAL_RCC_UART8_CLK_ENABLE()
#define GPIO_USARTx_TX_GPIO_PORT         GPIOE                 // TX引脚 GPIOA9
#define GPIO_USARTx_TX_GPIO_PIN          GPIO_PIN_7
#define GPIO_USARTx_RX_GPIO_PORT         GPIOE                 // RX引脚 GPIOA10
#define GPIO_USARTx_RX_GPIO_PIN          GPIO_PIN_8

// 协议定义
#define FRAME_HEAD_REQUEST1             0x12                // 请求帧头1
#define FRAME_HEAD_REQUEST2             0x4C                // 请求帧头2
#define FRAME_HEAD_RESPONSE1            0x05                // 响应帧头1
#define FRAME_HEAD_RESPONSE2            0x1C                // 响应帧头2

// 命令ID定义
#define CMD_PING                        0x01                // 检测命令
#define CMD_SET_ANGLE                   0x08                // 角度控制命令
#define CMD_GET_ANGLE                   0x0A                // 角度读取命令
#define CMD_CHANGE_ID                   0x04                // ID修改命令

#define MAX_DATA_LEN                    32                  // 最大数据包长度

extern uint8_t tx_buf[12];

void servo_set_angle(uint8_t servo_id, float angle, uint16_t time_ms);
uint8_t calculate_checksum(uint8_t *data, uint8_t length);


#endif
