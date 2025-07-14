#include "wt_usart.h"
#include "comm.h"
uint8_t rx_state = 1;

Attitude attitude;

char string_buf[128];

extern uint8_t angle[8];
/**
 * @brief 计算给定数据的 CRC-16 校验码（CRC-16/CCITT-FALSE）
 *
 * @param data 数据指针
 * @param length 数据长度（字节数）
 * @return uint16_t 生成的 CRC-16 校验码
 */
uint16_t crc16(const void *data, size_t length) {
    static uint16_t crc_table[256];     // CRC 查找表
    static int table_initialized = 0;    // 表是否已初始化

    // 生成 CRC 查找表（只在第一次调用时生成）
    if (!table_initialized) {
        for (int i = 0; i < 256; i++) {
            uint16_t crc = i;

            for (int j = 0; j < 8; j++) {
                if (crc & 0x8000)
                    crc = (crc << 1) ^ 0x1021;  // CRC-16/CCITT-FALSE 多项式
                else
                    crc = crc << 1;
            }

            crc_table[i] = crc;
        }

        table_initialized = 1;  // 标记表格已初始化
    }

    // 初始化 CRC 值
    uint16_t crc = 0xFFFF;

    // 将输入数据转换为字节指针
    const uint8_t *p = data;

    // 计算 CRC
    for (size_t i = 0; i < length; i++) {
        uint8_t index = (crc >> 8) ^ p[i];
        crc = (crc << 8) ^ crc_table[index];
    } 

    // 返回最终的 CRC-16 校验码（无需异或）
    return crc;
}

void attitude_solve(uint8_t* buf, uint8_t rx_st)
{
	switch(rx_state)
		{
			case 0://加速度
				attitude.ax = ((short)buf[3]<<8 | buf[4]) / 32768.0f * 16.0f *9.81f;
				attitude.ay = ((short)buf[5]<<8 | buf[6]) / 32768.0f * 16.0f *9.81f;
				attitude.az = ((short)buf[7]<<8 | buf[8]) / 32768.0f * 16.0f *9.81f;
				break;
			case 1: //角度
				attitude.pitch = ((short)buf[3]<<8 | buf[4]) / 32768.0f * 180.0f;
				attitude.roll = ((short)buf[5]<<8 | buf[6]) / 32768.0f * 180.0f;
				attitude.yaw = ((short)buf[7]<<8 | buf[8]) / 32768.0f * 180.0f;
				break;
		}
}

void WIT_UART_Transmit()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_UART_Transmit_IT(&huart8, angle, 8);
    HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     if (huart->Instance == USART1) 
//     {
// 		if(uart1rec.buf[uart1rec.cnt-2] == 0x50 && uart1rec.buf[uart1rec.cnt-1] == 0x03 && uart1rec.buf[uart1rec.cnt] == 0x06)
// 		{
// 			uart1rec.cnt = 2;
// 			uart1rec.buf[0] = 0x50;
// 			uart1rec.buf[1] = 0x03;
// 			uart1rec.buf[2] = 0x06;
// 		}
// 		if(uart1rec.cnt == 10)
// 		{
// 			uint16_t crc;
// 			crc = crc16(uart1rec.buf, 9);
// 			//if(((crc >> 8 & 0xFF) == uart1rec.buf[9]) && ((crc & 0xFF) == uart1rec.buf[10]))
// 			if(1)
// 			{
// 				attitude_solve(uart1rec.buf, rx_state);
// 				sprintf(string_buf,"yaw: %0.3f, pitch: %0.3f, roll: %0.3f, ax:: %0.3f, ay: %0.3f, az: %0.3f.", attitude.yaw, attitude.pitch, attitude.roll, attitude.ax, attitude.ay, attitude.az);
// 				HAL_UART_Transmit(&huart1, (uint8_t *)string_buf, strlen(string_buf), 100);
// 				uart1rec.cnt = 201;
// 			}
// 		}
// 		if(uart1rec.cnt > 200) uart1rec.cnt = 0;
// 		else uart1rec.cnt++;	
//     }
// 	HAL_UART_Receive_IT(&huart1,uart1rec.buf + uart1rec.cnt,1);
// }




