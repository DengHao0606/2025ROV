#include "RS485_process.h"

uint8_t tx_buf[12] = {0};

uint8_t calculate_checksum(uint8_t *data, uint8_t length) 
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < length; i++) 
    {
        sum += data[i];
    }
    return (uint8_t)(sum % 256);  // 对256取余
}

void servo_set_angle(uint8_t servo_id, float angle, uint16_t time_ms) 
{
    tx_buf[0] = FRAME_HEAD_REQUEST1;
    tx_buf[1] = FRAME_HEAD_REQUEST2;
    tx_buf[2] = CMD_SET_ANGLE;
    tx_buf[3] = 0x07;              // 内容长度：1(ID) + 2(角度) + 2(时间) + 2(功率) = 7
    
    tx_buf[4] = servo_id;
    
    // 处理角度（小端字节序，角度×10）
    int32_t angle_scaled = (int32_t)(angle * 10.0f);
    if (angle >= 0) 
    {
        tx_buf[5] = (uint8_t)(angle_scaled & 0xFF);
        tx_buf[6] = (uint8_t)((angle_scaled >> 8) & 0xFF);
    } 
    else 
    {
        // 对于负数，直接使用补码表示
        tx_buf[5] = (uint8_t)(angle_scaled & 0xFF);
        tx_buf[6] = (uint8_t)((angle_scaled >> 8) & 0xFF);
    }
    
    // 处理时间间隔（小端字节序，单位ms）
    tx_buf[7] = (uint8_t)(time_ms & 0xFF);
    tx_buf[8] = (uint8_t)((time_ms >> 8) & 0xFF);
    
    // 执行功率设为0（2字节）
    tx_buf[9] = 0x00;
    tx_buf[10] = 0x00;
    
    tx_buf[11] = calculate_checksum(tx_buf, 11);  // 计算前11字节校验和
    
    HAL_UART_Transmit_IT(&huart7, tx_buf, sizeof(tx_buf));
}