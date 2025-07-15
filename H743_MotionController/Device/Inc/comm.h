#ifndef __COMM_H
#define __COMM_H

#include "autocontrol.h"
#include "usart.h"

#define UART_RX_STATE_START 0
#define UART_RX_STATE_READY 2
#define UART_RX_STATE_DEAL 1

typedef struct {
    uint8_t          buf[256];      // 接收数据缓存数组
    volatile uint8_t rx_len;        // 接收一帧数据的长度
    volatile uint8_t recv_end_flag; // 一帧数据接收完成标志
    int              cnt;
} RecBuf;

typedef struct {
    uint8_t imustate;
    uint8_t dvlstate;

    CoordinateVector pos;
    CoordinateVector spd;
} IMU;

extern uint8_t dvlstate;
extern IMU imu;

extern RecBuf uart1rec;
extern RecBuf uart4rec;
extern RecBuf uart7rec;
extern RecBuf uart8rec;

void CommInit(void);
void dvl_shutdown(UART_HandleTypeDef *huart);
void dvl_startup(UART_HandleTypeDef *huart);
void imu_setmode(UART_HandleTypeDef *huart);

#endif