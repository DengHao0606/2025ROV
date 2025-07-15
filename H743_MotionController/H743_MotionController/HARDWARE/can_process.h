#ifndef __CAN_PROCESS_H
#define __CAN_PROCESS_H

#include "fdcan.h"

typedef struct {
    FDCAN_RxHeaderTypeDef RxHeader;   // 用来保存接收到的数据帧头部信息
    uint8_t               RxData[64]; // 用来保存接收数据端数据
} CanRx;


void FDCAN1_Config(void);
void Canfd1Transmit64(uint8_t id, uint8_t *can1_txbuf);


#ifdef USING_ROTATINGSPEED
void FDCAN2_Config(void);
void Canfd2Transmit8(uint8_t id, uint8_t *can2_txbuf);
#endif


#endif