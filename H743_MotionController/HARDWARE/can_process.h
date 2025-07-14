#ifndef __CAN_PROCESS_H
#define __CAN_PROCESS_H

#include "fdcan.h"

typedef struct {
    FDCAN_RxHeaderTypeDef RxHeader;   // 用来保存接收到的数据帧头部信息
    uint8_t               RxData[64]; // 用来保存接收数据端数据
} CanRx;


void FDCAN1_Config(void);
void Canfd1Transmit64(uint8_t id, uint8_t *can1_txbuf);
void FDCAN2_Config(void);
void Canfd2Transmit8(uint8_t id, uint8_t *can2_txbuf);
void ParseCanData(uint8_t *data);
uint32_t build_can_id(uint8_t motor_id, uint8_t data_type, uint8_t priority);
HAL_StatusTypeDef send_control_command(FDCAN_HandleTypeDef *hfdcan, 
                                        uint8_t motor_id, 
                                        uint8_t command_type, 
                                        float value, 
                                        uint8_t priority);
HAL_StatusTypeDef set_position(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float angle_deg, uint8_t priority);
HAL_StatusTypeDef set_speed(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float speed_dps, uint8_t priority);
HAL_StatusTypeDef set_current(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float current_A, uint8_t priority);
HAL_StatusTypeDef emergency_stop(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, uint8_t priority);

#ifdef USING_ROTATINGSPEED
void FDCAN2_Config(void);
void Canfd2Transmit8(uint8_t id, uint8_t *can2_txbuf);
#endif


#endif