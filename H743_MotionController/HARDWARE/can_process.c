#include "can_process.h"
#include "fdcan.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

CanRx canfd1_rx = {0};
CanRx canfd2_rx = {0};

extern float received_depth;
extern float received_temp; 
extern uint8_t can_rx_data;

// 构建 CAN ID
uint32_t build_can_id(uint8_t motor_id, uint8_t data_type, uint8_t priority) 
{
    uint32_t servo_id = ((uint32_t)0x00 << 24) | 
                            ((uint32_t)motor_id << 16) | 
                            ((uint32_t)0x41 << 8) | 
                            (uint32_t)priority;
    return servo_id;  
}

// 发送控制指令
HAL_StatusTypeDef send_control_command(FDCAN_HandleTypeDef *hfdcan, 
                                        uint8_t motor_id, 
                                        uint8_t command_type, 
                                        float value, 
                                        uint8_t priority) 
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8] = {0};
    uint32_t float_bytes;
    
    // 构建小端模式ID
    TxHeader.Identifier = build_can_id(motor_id, 0x41, priority);
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    
    // 填充数据段（正确顺序：Data0=指令类型，Data1~4=float参数）
    TxData[0] = command_type;  // 指令类型放在Data0
    memcpy(&float_bytes, &value, sizeof(float));  // 读取float的32位值

    TxData[1] = (float_bytes >> 0) & 0xFF;
    TxData[2] = (float_bytes >> 8) & 0xFF;
    TxData[3] = (float_bytes >> 16) & 0xFF;
    TxData[4] = (float_bytes >> 24) & 0xFF;
    
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData);
}

// 封装的控制函数
HAL_StatusTypeDef set_position(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float angle_deg, uint8_t priority) 
{
    return send_control_command(hfdcan, motor_id, 0x00, angle_deg, priority);
}

HAL_StatusTypeDef set_speed(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float speed_dps, uint8_t priority) 
{
    return send_control_command(hfdcan, motor_id, 0x01, speed_dps, priority);
}

HAL_StatusTypeDef set_current(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float current_A, uint8_t priority) 
{
    return send_control_command(hfdcan, motor_id, 0x02, current_A, priority);
}

HAL_StatusTypeDef emergency_stop(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, uint8_t priority)
{
    return send_control_command(hfdcan, motor_id, 0x03, 0.0f, priority);
}

void FDCAN1_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    // 标准段滤波器初始化0x000~0x0FF
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex  = 1;
    sFilterConfig.FilterType   = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1    = 0x00000000;
    sFilterConfig.FilterID2    = 0x00000000;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

    // 标准段滤波器初始化0x200~0x2FF
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex  = 2;
    sFilterConfig.FilterType   = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1    = 0x00000000;
    sFilterConfig.FilterID2    = 0x00000000;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

    // 以下为启动中断
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // 使能FIFO0数据接收中断

    // 以下为启动CAN外设
    HAL_FDCAN_Start(&hfdcan1);
}

void FDCAN2_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    // 标准段初始化（文档未使用标准帧，可配置为接收所有扩展帧）
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex  = 0;
    sFilterConfig.FilterType   = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1    = 0x00000000;
    sFilterConfig.FilterID2    = 0x00000000;
    HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);

    // 扩展段初始化（接收所有扩展ID）
    sFilterConfig.IdType       = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterIndex  = 1;
    sFilterConfig.FilterType   = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1    = 0x00000000;  // 掩码全0表示接收所有ID
    sFilterConfig.FilterID2    = 0x00000000;
    HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

    // 以下为启动中断
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // 使能FIFO0数据接收中断
    // 以下为启动CAN外设
    HAL_FDCAN_Start(&hfdcan2);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan == &hfdcan1) 
    {
        /*来自G4V的深度计数据*/
        if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
        {
            FDCAN_RxHeaderTypeDef RxHeader; // 用来保存接收到的数据帧头部信号
            uint8_t RxData[64];             // 用来保存接收数据端数据
            
            if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) // 从接收队列中读取数据
            {
                if(RxHeader.Identifier == 0x005) 
                {
                    ParseCanData(RxData);
                    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
                }
            }
        }
        /*来自can舵机的数据*/
        if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) 
        {
            FDCAN_RxHeaderTypeDef RxHeader;
            uint8_t RxData[8] = {0};
            
            if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) 
            {
                // 解析小端模式ID（直接按字段位置提取）
                uint8_t motor_id = (RxHeader.Identifier >> 16) & 0xFF;    // ID23~16
                uint8_t data_type = (RxHeader.Identifier >> 8) & 0xFF;     // ID15~8
                uint8_t priority = RxHeader.Identifier & 0xFF;             // ID7~0
                
                // 处理数据段（RxData[0]为指令类型，RxData[1~4]为参数）
                uint8_t cmd_type = RxData[0];
                float param_value;
                memcpy(&param_value, RxData + 1, sizeof(float));  // 提取参数
                
                // 处理不同类型的消息
                switch(data_type) 
                {
                    case 0x00: // 位置反馈
                        // 处理位置：param_value为当前角度（°）
                        break;
                    case 0x01: // 速度反馈
                        // 处理速度：param_value为当前转速（°/s）
                        break;
                    case 0x02: // 电流反馈
                        // 处理电流：param_value为当前电流（A）
                        break;
                    default:
                        break;
                }
            }
        }
    }
}

void ParseCanData(uint8_t *data)
{
    // 解析温度（前4字节）
    uint32_t temp_bytes = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    received_temp = *(float*)&temp_bytes;
    received_temp = temp_bytes / 100.0f;

    // 解析深度（后4字节）
    uint32_t depth_bytes = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
    received_depth = *(float*)&depth_bytes;
    received_depth = depth_bytes / 100.0f;
    // printf("Temperature: %.2f 'C\r\n", received_temp);

}