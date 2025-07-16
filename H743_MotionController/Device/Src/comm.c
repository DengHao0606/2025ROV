#include "comm.h"
#include "usart.h"
#include "main.h"
#include "motor.h"
#include "json_process.h"
#include "wt_usart.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

RecBuf uart1rec = {0};
RecBuf uart4rec = {0};
RecBuf uart7rec = {0};
RecBuf uart8rec = {0};

extern uint8_t rx_state;
extern Attitude attitude;
extern char string_buf[128];
uint8_t dvlstate = 0;

IMU imu;

extern int threadmonitor_uart1;
extern int threadmonitor_uart4;
extern int threadmonitor_uart7;
extern int threadmonitor_uart8;

extern float checkeddepth;

extern CoordinateSystems target;


/*
 * 函数名: CommInit
 * 描述  : 串口通信初始化
 * 输入  : /
 * 输出  : /
 * 备注  : /
 */
void CommInit(void)
{
    uart1rec.cnt = 0;
    uart7rec.cnt = 0;
    uart8rec.cnt = 0;

    // __HAL_UART_CLEAR_OREFLAG(&huart4);
    // huart4.RxState = HAL_UART_STATE_READY;
    // huart4.Lock    = HAL_UNLOCKED;

    // __HAL_UART_CLEAR_OREFLAG(&huart7);
    // huart7.RxState = HAL_UART_STATE_READY;
    // huart7.Lock    = HAL_UNLOCKED;

    HAL_UART_Receive_IT(&huart1, uart1rec.buf, 1);
    HAL_UART_Receive_IT(&huart7, uart7rec.buf, 1);
    HAL_UART_Receive_IT(&huart8, uart8rec.buf, 1);

    HAL_Delay(100); // 稍作延迟防止无法进入中断
}
/*
 * 函数名: HAL_UART_RxCpltCallback
 * 描述  : 串口中断处理
 * 输入  : UART_HandleTypeDef *huart 串口地址
 * 输出  : /
 * 备注  : /
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) // 串口接收中断
{
    // 上位机摇杆指令
    if (huart == &huart1) 
    {
        threadmonitor_uart1 = 200;
        if (uart1rec.buf[uart1rec.cnt - 1] == '{' && uart1rec.buf[uart1rec.cnt] == '\"' && uart1rec.cnt > 0)
        {
            uart1rec.cnt = 1;
            uart1rec.buf[0] = '{';
            uart1rec.buf[1] = '\"';
        }
        // 检查帧尾（换行符作为结束）
        else if (uart1rec.buf[uart1rec.cnt] == '\n' && uart1rec.cnt > 0)
        {
            uart1rec.buf[uart1rec.cnt] = '\0'; // 确保字符串终止
            JSON_Process_Data((uint8_t *)uart1rec.buf);//JSON字符串处理
            uart1rec.cnt = 201; // 使缓冲计数归零
        }
        if (uart1rec.cnt >= 200)// 防止缓冲区溢出
            uart1rec.cnt = 0; 
        else
            uart1rec.cnt++;
        
        HAL_UART_Receive_IT(&huart1, uart1rec.buf + uart1rec.cnt, 1);
    }
    //姿态传感器
    // if (huart == &huart8) 
    // {
	// 	if(uart8rec.buf[uart8rec.cnt-2] == 0x50 && uart8rec.buf[uart8rec.cnt-1] == 0x03 && uart8rec.buf[uart8rec.cnt] == 0x06)
	// 	{
	// 		uart8rec.cnt = 2;
	// 		uart8rec.buf[0] = 0x50;
	// 		uart8rec.buf[1] = 0x03;
	// 		uart8rec.buf[2] = 0x06;
	// 	}
	// 	if(uart8rec.cnt == 10)
	// 	{
			// uint16_t crc;
			// crc = crc16(uart8rec.buf, 9);
			//if(((crc >> 8 & 0xFF) == uart1rec.buf[9]) && ((crc & 0xFF) == uart1rec.buf[10]))
	// 		if(1)
	// 		{
    //             rx_state = 1;
	// 			attitude_solve(uart8rec.buf, rx_state);
	// 			sprintf(string_buf,"yaw: %0.3f, pitch: %0.3f, roll: %0.3f, ax:: %0.3f, ay: %0.3f, az: %0.3f.",
    //                     attitude.yaw, attitude.pitch, attitude.roll, attitude.ax, attitude.ay, attitude.az);
	// 			HAL_UART_Transmit_IT(&huart1, (uint8_t *)string_buf, strlen(string_buf));
	// 			uart8rec.cnt = 201;
	// 		}
	// 	}
	// 	if (uart8rec.cnt > 200) 
    //         uart8rec.cnt = 0;
	// 	else 
    //         uart8rec.cnt++;
        
	//     HAL_UART_Receive_IT(&huart8,uart8rec.buf + uart8rec.cnt,1);
    // }

}