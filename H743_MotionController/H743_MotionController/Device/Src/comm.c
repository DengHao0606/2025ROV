#include "comm.h"
#include "usart.h"
#include "main.h"
#include "motor.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

RecBuf uart4rec = {0};
RecBuf uart7rec = {0};


uint8_t dvlstate = 0;

IMU imu;

extern int threadmonitor_uart4;
extern int threadmonitor_uart7;
extern float checkeddepth;

extern CoordinateSystems target;
extern uint8_t debug_data[2];

// /*
//  * 函数名: imu_setmode
//  * 描述  : 设置室内导航模式
//  * 输入  : /
//  * 输出  : /
//  * 备注  : /
//  */
// void imu_setmode(UART_HandleTypeDef *huart)
// {
//     uint8_t BUFIMUSETMODE[14] = {0xFC, 0xCF, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0xFD, 0xDF};
//     HAL_UART_Transmit(huart, BUFIMUSETMODE, 14, 10); // 设置室内导航模式
// }

// /*
//  * 函数名: dvl_shutdown
//  * 描述  : 关闭Dvl
//  * 输入  : /
//  * 输出  : /
//  * 备注  : /
//  */
// void dvl_shutdown(UART_HandleTypeDef *huart)
// {
//     uint8_t BUFDVLSHUTDOWN[14] = {0xFC, 0xCF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0xFD, 0xDF};
//     HAL_UART_Transmit(huart, BUFDVLSHUTDOWN, 14, 10); // 关闭DVL
// }

// /*
//  * 函数名: dvl_startup
//  * 描述  : 开启Dvl
//  * 输入  : /
//  * 输出  : /
//  * 备注  : /
//  */
// void dvl_startup(UART_HandleTypeDef *huart)
// {
//     uint8_t BUFDVLPOWERON[14] = {0xFC, 0xCF, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0xFD, 0xDF};
//     HAL_UART_Transmit(huart, BUFDVLPOWERON, 14, 10); // 关闭DVL
// }

/*
 * 函数名: CommInit
 * 描述  : 串口通信初始化
 * 输入  : /
 * 输出  : /
 * 备注  : /
 */
void CommInit(void)
{
    uart7rec.cnt = 0;
    uart8rec.cnt = 0;

    // __HAL_UART_CLEAR_OREFLAG(&huart4);
    // huart4.RxState = HAL_UART_STATE_READY;
    // huart4.Lock    = HAL_UNLOCKED;

    // __HAL_UART_CLEAR_OREFLAG(&huart7);
    // huart7.RxState = HAL_UART_STATE_READY;
    // huart7.Lock    = HAL_UNLOCKED;

    HAL_UART_Receive_IT(&huart8, uart4rec.buf, 1);
    HAL_UART_Receive_IT(&huart7, uart7rec.buf, 1);
    HAL_UART_Receive_IT(&huart1, uart1rec.buf, 1);
    init_485(&huart8, debug_data, 2, GPIOB, GPIO_PIN_7);

    HAL_Delay(100); // 稍作延迟防止无法进入中断
}

/*
 * 函数名: void Transmit_485_IT
 * 描述  : RS485中断发送
 * 输入  : UART_HandleTypeDef *huart, uint8_t *buf, uint16_t len, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin
 * 输出  : /
 * 备注  : /
 */
void Transmit_485_IT(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t len, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_UART_Transmit_IT(huart, buf, len);
    // HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

/*
 * 函数名: init_485
 * 描述  : RS485串口初始化
 * 输入  : UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin
 * 输出  : /
 * 备注  : /
 */
void init_485(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    HAL_UART_Receive_IT(huart, pData, Size);
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}
 
/*
 * 函数名: Check_Data
 * 描述  : 异或校验
 * 输入  : /
 * 输出  : /
 * 备注  : /
 */
// uint8_t Check_Data(uint8_t *data, uint8_t size)
// {
//     uint8_t temp, i;
//     temp = data[0];
//     for (i = 1; i < size; i++) { temp = temp ^ data[i]; }
//     return temp;
// }

/*
 * 函数名: ImuSolve
 * 描述  : Imu数据解算
 * 输入  : /
 * 输出  : /
 * 备注  : /
 */
// void ImuSolve(uint8_t *buf)
// {
//     float   imu_data[12]; // 前6位位置姿态矢量 后6位速度矢量
//     uint8_t check;

//     check = Check_Data(buf, 91);

//     if (buf[0] == 0xfa && buf[1] == 0xaf && check == buf[91]) // 校验帧头
//     {
//         // 读数据缓冲
//         memcpy(&(imu_data[3]), buf + 2, 12); // 姿态
//         memcpy(&(imu_data[0]), buf + 54, 8); // x y
//         memcpy(&(imu_data[2]), buf + 80, 4); // Z
//         imu.imustate = buf[66];              // 导航状态
//         imu.dvlstate = buf[67];
//         // memcpy(&imu_data[6], buf + 26, 12);  // 速度
//         // memcpy(&(imu.spd.ry), buf + 68, 4);  // 横滚速度
//         // memcpy(&(imu.spd.rx), buf + 72, 4);  // 俯仰速度
//         // memcpy(&(imu.spd.rz), buf + 76, 4);  // 转向速度

//         // 读取IMU数据
//         imu.pos.y  = imu_data[0];
//         imu.pos.x  = -imu_data[1];
//         //imu.pos.z  = imu_data[2];
//         imu.pos.z = checkeddepth;
//         imu.pos.ry = imu_data[3];
//         imu.pos.rx = imu_data[4];
//         imu.pos.rz = imu_data[5];
//         // if (imu_data[0] > -10 && imu_data[0] < 10) imu.pos.y = imu_data[0];
//         // if (imu_data[1] > -10 && imu_data[1] < 10) imu.pos.x = -imu_data[1];
//         // if (imu_data[2] > 0 && imu_data[2] < 5) imu.pos.z = imu_data[2];
//         // if (imu_data[3] > -180 && imu_data[3] < 180) imu.pos.ry = imu_data[3];
//         // if (imu_data[4] > -180 && imu_data[4] < 180) imu.pos.rx = imu_data[4];
//         // if (imu_data[5] > -180 && imu_data[5] < 180) imu.pos.rz = imu_data[5];

//         // imu.spd.x = -imu_data[6];
//         // imu.spd.y = imu_data[7];
//         // imu.spd.z = imu_data[8];
//     }

//     // led
//     if (led_uart7)
//         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
//     else
//         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
//     led_uart7 = !led_uart7;
// }

/*
 * 函数名: CmdSolve
 * 描述  : 下行命令解算
 * 输入  : /
 * 输出  : /
 * 备注  : /
 */
// void CmdSolve(uint8_t *buf)
// {
//     switch (buf[2])
//     {
//         case 0x01: // 开关指定轴上的PID控制
//             memcpy(&(robot_controller.state), buf + 3, 6);
//             break;
//         case 0x02: // 推力设置
//             memcpy(openloop_thrust, buf + 3, 24);
//             break;
//         case 0x03: // 目标位置下行
//             // 移动至指定位置
//             if (buf[3] == 0x00)
//             {
//                 memcpy(&(robot.target_inworld.vector), buf + 4, 24);
//                 robot.target_inworld.extract(&(robot.target_inworld));
//             }
//             // 移动相对距离
//             else if (buf[3] == 0x01)
//             {
//                 memcpy(&(robot.target_inbase.vector), buf + 4, 24);
//                 robot.target_inbase.extract(&(robot.target_inbase));
//                 robot.base2world(&robot);
//             }
//             // 目标值步进
//             else if (buf[3] == 0x02)
//             {
//                 memcpy(&(target.target_inbase.vector), buf + 4, 24);                         // 接收步进信息
//                 target.target_inbase.extract(&(target.target_inbase));                       // 更新齐次矩阵
//                 memcpy(&(target.base.vector), &(robot.target_inworld.vector), 24);           // 读取目标起始位置
//                 target.base.extract(&(target.base));                                         // 更新齐次矩阵
//                 target.base2world(&target);                                                  // 步进与起始位置叠加
//                 memcpy(&(robot.target_inworld.vector), &(target.target_inworld.vector), 24); // 写入目标位置
//                 robot.target_inworld.extract(&(robot.target_inworld));                       // 更新齐次矩阵
//             }
//             break;
//         case 0x04: // IMU状态设置
//             dvlstate = buf[3];
//             break;
//         case 0x05: // PID参数
//             switch (buf[3])
//             {
//                 case 0x00:
//                     memcpy(&(robot_controller.x.kp), buf + 4, 4);
//                     memcpy(&(robot_controller.x.ki), buf + 8, 4);
//                     memcpy(&(robot_controller.x.kd), buf + 12, 4);
//                     memcpy(&(robot_controller.x.i_limit), buf + 16, 4);
//                     memcpy(&(robot_controller.x.output_limit), buf + 20, 4);
//                     break;
//                 case 0x01:
//                     memcpy(&(robot_controller.y.kp), buf + 4, 4);
//                     memcpy(&(robot_controller.y.ki), buf + 8, 4);
//                     memcpy(&(robot_controller.y.kd), buf + 12, 4);
//                     memcpy(&(robot_controller.y.i_limit), buf + 16, 4);
//                     memcpy(&(robot_controller.y.output_limit), buf + 20, 4);
//                     break;
//                 case 0x02:
//                     memcpy(&(robot_controller.z.kp), buf + 4, 4);
//                     memcpy(&(robot_controller.z.ki), buf + 8, 4);
//                     memcpy(&(robot_controller.z.kd), buf + 12, 4);
//                     memcpy(&(robot_controller.z.i_limit), buf + 16, 4);
//                     memcpy(&(robot_controller.z.output_limit), buf + 20, 4);
//                     break;
//                 case 0x03:
//                     memcpy(&(robot_controller.rx.kp), buf + 4, 4);
                    // memcpy(&(robot_controller.rx.ki), buf + 8, 4);
//                     memcpy(&(robot_controller.rx.kd), buf + 12, 4);
//                     memcpy(&(robot_controller.rx.i_limit), buf + 16, 4);
//                     memcpy(&(robot_controller.rx.output_limit), buf + 20, 4);
//                     break;
//                 case 0x04:
//                     memcpy(&(robot_controller.ry.kp), buf + 4, 4);
//                     memcpy(&(robot_controller.ry.ki), buf + 8, 4);
//                     memcpy(&(robot_controller.ry.kd), buf + 12, 4);
//                     memcpy(&(robot_controller.ry.i_limit), buf + 16, 4);
//                     memcpy(&(robot_controller.ry.output_limit), buf + 20, 4);
//                     break;
//                 case 0x05:
//                     memcpy(&(robot_controller.rz.kp), buf + 4, 4);
//                     memcpy(&(robot_controller.rz.ki), buf + 8, 4);
//                     memcpy(&(robot_controller.rz.kd), buf + 12, 4);
//                     memcpy(&(robot_controller.rz.i_limit), buf + 16, 4);
//                     memcpy(&(robot_controller.rz.output_limit), buf + 20, 4);
//                     break;
//                 default: break;
//             }
//             break;

//         case 0x07: // 推力曲线
//             memcpy(&(thrustcurve[buf[3]].pwm[0]), buf + 4, 4);
//             memcpy(&(thrustcurve[buf[3]].pwm[1]), buf + 8, 4);
//             memcpy(&(thrustcurve[buf[3]].pwm[2]), buf + 12, 4);
//             memcpy(&(thrustcurve[buf[3]].pwm[3]), buf + 16, 4);

//             memcpy(&(thrustcurve[buf[3]].thrust[0]), buf + 20, 4);
//             memcpy(&(thrustcurve[buf[3]].thrust[1]), buf + 24, 4);
//             memcpy(&(thrustcurve[buf[3]].thrust[2]), buf + 28, 4);
//             memcpy(&(thrustcurve[buf[3]].thrust[3]), buf + 32, 4);
//             break;

//         default: break;
//     }

//     // led
//     if (led_uart4)
//         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
//     else
//         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
//     led_uart4 = !led_uart4;
// }

/*
 * 函数名: HAL_UART_RxCpltCallback
 * 描述  : 串口中断处理
 * 输入  : UART_HandleTypeDef *huart 串口地址
 * 输出  : /
 * 备注  : /
 */
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) // 串口接收中断
// {
    // 串口接收控制指令
//     if (huart == &huart4)
//     {
//         threadmonitor_uart4 = 200;
//         if (uart4rec.buf[uart4rec.cnt - 1] == 0xfa && uart4rec.buf[uart4rec.cnt] == 0xaf && uart4rec.cnt > 0)
//         {
//             uart4rec.cnt    = 1;
//             uart4rec.buf[0] = 0xfa;
//             uart4rec.buf[1] = 0xaf;
//         }
//         if (uart4rec.buf[uart4rec.cnt - 1] == 0xfb && uart4rec.buf[uart4rec.cnt] == 0xbf && uart4rec.cnt > 0)
//         {
//             CmdSolve(uart4rec.buf);
//             uart4rec.cnt = 201; // 使缓冲计数归零
//         }
//         if (uart4rec.cnt > 200) // 防止缓冲溢出
//             uart4rec.cnt = 0;
//         else
//             uart4rec.cnt++;

//         HAL_UART_Receive_IT(&huart4, uart4rec.buf + uart4rec.cnt, 1);
//     }

     // 惯性导航系统
//     if (huart == &huart7)
//     {
//         threadmonitor_uart7 = 100;
//         if (uart7rec.buf[uart7rec.cnt - 1] == 0xfa && uart7rec.buf[uart7rec.cnt] == 0xaf && uart7rec.cnt > 0)
//         {
//             uart7rec.cnt    = 1;
//             uart7rec.buf[0] = 0xfa;
//             uart7rec.buf[1] = 0xaf;
//         }
//         if (uart7rec.buf[uart7rec.cnt - 1] == 0xfb && uart7rec.buf[uart7rec.cnt] == 0xbf && uart7rec.cnt > 5)
//         {
//             ImuSolve(uart7rec.buf);
//             uart7rec.cnt = 201; // 使缓冲计数归零
//         }
//         if (uart7rec.cnt > 200) // 防止缓冲溢出
//             uart7rec.cnt = 0;
//         else
//             uart7rec.cnt++;

//         __HAL_UART_CLEAR_OREFLAG(&huart7);
//         huart7.RxState = HAL_UART_STATE_READY;
//         huart7.Lock    = HAL_UNLOCKED;

//         HAL_UART_Receive_IT(&huart7, uart7rec.buf + uart7rec.cnt, 1);
//     }
// }
