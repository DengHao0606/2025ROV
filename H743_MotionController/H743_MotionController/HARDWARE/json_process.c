#include "json_process.h"
#include "comm.h"

#include "main.h"
#include "usart.h"
#include "tim.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

RecBuf uart8rec = {0};
RecBuf uart1rec = {0};
float servo0angle = 0.0;
extern int threadmonitor_uart8;
int bbb=0;
JSON_Command_t command = {0};

extern uint8_t debug_data[2];
/* 私有变量 */
static uint8_t rx_buffer[1024];
static uint16_t rx_index = 0;
static uint8_t rx_char;

/* 私有函数 */
static void parse_json_data(uint8_t *json_str);
static void apply_motor_control(void);

/**
  * @brief  打印当前所有电机的推力参数
  * @param  None
  * @retval None
  */
static void print_thrust_params(void)
{
    printf("\nLatest Thrust Parameters:\n");
    printf("Motor | np_mid | np_ini | pp_ini | pp_mid | nt_end | nt_mid | pt_mid | pt_end\n");
    printf("------|--------|--------|--------|--------|--------|--------|--------|-------\n");
    
    for (int i = 0; i < 6; i++) {
        MotorParams_t *m = &command.motors[i];
        printf("M%-4d | %-6.2f | %-6.2f | %-6.2f | %-6.2f | %-6.2f | %-6.2f | %-6.2f | %-6.2f\n",
               i,
               m->np_mid, m->np_ini,
               m->pp_ini, m->pp_mid,
               m->nt_end, m->nt_mid,
               m->pt_mid, m->pt_end);
    }
    printf("\n");
}

/*
 * 函数名: JSON_Process_Init
 * 描述  : 初始化JSON处理器，启动串口接收中断，并初始化接收缓冲区
 * 输入  : 无
 * 输出  : 无
 * 备注  : 调用HAL_UART_Receive_IT函数启动串口接收中断，将接收缓冲区rx_buffer清零，并重置接收索引rx_index，
 *         同时打印初始化成功的提示信息。
 */
void JSON_Process_Init(void) 
{
    HAL_UART_Receive_IT(&huart1, &rx_char, 1);  // 启动接收中断
    memset(rx_buffer, 0, sizeof(rx_buffer));
    rx_index = 0;
    // HAL_Delay(100); // 稍作延迟防止无法进入中断
}

/*
 * 函数名: JSON_Process_Data
 * 描述  : 处理接收到的JSON数据，调用parse_json_data函数进行解析
 * 输入  : json_str - 指向接收到的JSON数据的指针
 * 输出  : 无
 * 备注  : 
 */
void JSON_Process_Data(uint8_t *json_str)
{
    parse_json_data(json_str);
}

/*
 * 函数名: HAL_UART_RxCpltCallback
 * 描述  : 串口中断处理
 * 输入  : UART_HandleTypeDef *huart 串口地址
 * 输出  : /
 * 备注  : /
 */
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
// {
//     // 串口接收控制指令
//     if (huart == &huart8) 
//     {
//         threadmonitor_uart8 = 200;
//         if (uart8rec.buf[uart8rec.cnt - 1] == '{' && uart8rec.buf[uart8rec.cnt] == '\"' && uart8rec.cnt > 0)
//         {
//             uart8rec.cnt = 1;
//             uart8rec.buf[0] = '{';
//             uart8rec.buf[1] = '\"';
//         }
//         // 检查帧尾（换行符作为结束）
//         else if (uart8rec.buf[uart8rec.cnt] == '\n' && uart8rec.cnt > 0)
//         {
//             // printf("RX JSON: %s\n", uart8rec.buf);
//             uart8rec.buf[uart8rec.cnt] = '\0'; // 确保字符串终止
//             JSON_Process_Data((uint8_t *)uart8rec.buf);
//             uart8rec.cnt = 1001; // 使缓冲计数归零
//         }
//         if (uart8rec.cnt >= 1000)// 防止缓冲区溢出
//             uart8rec.cnt = 0; 
//         else
//             uart8rec.cnt++;
        
//         HAL_UART_Receive_IT(&huart8, uart8rec.buf + uart8rec.cnt, 1);
//     }
// }
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
    // 串口接收控制指令
    if (huart == &huart1) 
    {
        threadmonitor_uart8 = 200;
        if (uart1rec.buf[uart1rec.cnt - 1] == '{' && uart1rec.buf[uart1rec.cnt] == '\"' && uart1rec.cnt > 0)
        {
            uart1rec.cnt = 1;
            uart1rec.buf[0] = '{';
            uart1rec.buf[1] = '\"';
        }
        // 检查帧尾（换行符作为结束）
        else if (uart1rec.buf[uart1rec.cnt] == '\n' && uart1rec.cnt > 0)
        {
            // printf("RX JSON: %s\n", uart8rec.buf);
            uart1rec.buf[uart1rec.cnt] = '\0'; // 确保字符串终止
            JSON_Process_Data((uint8_t *)uart1rec.buf);
            uart1rec.cnt = 1001; // 使缓冲计数归零
        }
        if (uart1rec.cnt >= 1000)// 防止缓冲区溢出
            uart1rec.cnt = 0; 
        else
            uart1rec.cnt++;
        
        HAL_UART_Receive_IT(&huart1, uart1rec.buf + uart1rec.cnt, 1);
    }
    else if (huart == &huart8)
    {
      Transmit_485_IT(&huart8, debug_data, 2, GPIOB, GPIO_PIN_7);
      init_485(&huart8, debug_data, 2, GPIOB, GPIO_PIN_7);
    }
}
/*
 * 函数名: parse_json_data
 * 描述  : 解析接收到的JSON数据，提取其中的字段并打印解析结果，调用时先在启动文件里修改Heap_Size
 * 输入  : json_str - 指向接收到的JSON数据的指针
 * 输出  : 无
 * 备注  :   
 */
static void parse_json_data(uint8_t *json_str)
{
  cJSON *root = cJSON_Parse((char *)json_str);
  if (!root) 
  {
      const char *error_ptr = cJSON_GetErrorPtr();
      if (error_ptr != NULL) 
      {
          printf("Error before: %s\n", error_ptr);
      }
      // printf("Error\r\n");
      return;
  }

  /* 提取各字段 */


  cJSON *x_item = cJSON_GetObjectItem(root, "x");
    if  (x_item) command.x = x_item->valuedouble;

  cJSON *y_item = cJSON_GetObjectItem(root, "y");
    if  (y_item) command.y = y_item->valuedouble;

  cJSON *z_item = cJSON_GetObjectItem(root, "z");
    if  (z_item) command.z = z_item->valuedouble;

  cJSON *yaw_item = cJSON_GetObjectItem(root, "yaw");
    if  (yaw_item) command.yaw = yaw_item->valuedouble;

  cJSON *servo0_item = cJSON_GetObjectItem(root, "servo0");
    if  (servo0_item) command.servo0 = servo0_item->valuedouble;



    openloop_thrust[0] = command.x;
    openloop_thrust[1] = command.y;
    openloop_thrust[2] = command.z;
    openloop_thrust[3] = command.roll;
    openloop_thrust[4] = command.yaw;
    openloop_thrust[5] = command.pitch;

    servo0angle = command.servo0;


  /* 不需要解析 */
  // cJSON *roll_item = cJSON_GetObjectItem(root, "roll");
  //   if  (roll_item) command.roll = roll_item->valuedouble;

  // cJSON *pitch_item = cJSON_GetObjectItem(root, "pitch");
  //   if  (pitch_item) command.pitch = pitch_item->valuedouble;

  // cJSON *servo1_item = cJSON_GetObjectItem(root, "servo1");
  //   if  (servo1_item) command.servo1 = servo1_item->valuedouble;

  // cJSON *state_item = cJSON_GetObjectItem(root, "state");
  //   if  (state_item) command.state = state_item->valueint;

/* 电机参数解析（两种格式兼容） */
    cJSON *cmd_item = cJSON_GetObjectItem(root, "cmd");
    if (cmd_item && strcmp(cmd_item->valuestring, "thrust_init") == 0)
    {
        // 格式1: {"cmd":"thrust_init", "motor":0, "np_mid":1.0...}
        int motor_num = cJSON_GetObjectItem(root, "motor")->valueint;
        parse_thrust_params(root, motor_num);
    } 
    // else 
    // {
    //     // 格式2: {"m0":{"np_mid":1.0...}, ...}
    //     for (int i = 0; i < 6; i++)
    //     {
    //         char motor_name[5];
    //         sprintf(motor_name, "m%d", i);
    //         cJSON *motor_item = cJSON_GetObjectItem(root, motor_name);
    //         if (motor_item) parse_thrust_params(motor_item, i);
    //         printf("");
    //     }
    // }
  // /* 打印解析结果 */
  // if(servo0angle > 0.5)
  // {
  //   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
  // }
  // else
  // {
  //   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
  // }
  printf("x: %.2f  ",openloop_thrust[0]);
  printf("y: %.2f  ",openloop_thrust[1]);
  printf("z: %.2f  ",openloop_thrust[2]);
  printf("yaw: %.2f  ",openloop_thrust[4]);
  printf("servo0: %.3f\r\n",servo0angle);
  bbb++;
  if (bbb == 20)
  {
      print_thrust_params();bbb=0;
    /* code */
  }
//   print_thrust_params();
  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_7);
  cJSON_Delete(root);
}

/**
  * @brief  仅提取电机推力参数（不做任何校验和额外操作）
  * @param  motor_item: cJSON对象指针
  * @param  motor_num: 电机编号(0-5)
  * @retval None
  */
static void parse_thrust_params(cJSON *motor_item, int motor_num) 
{
    MotorParams_t *motor = &command.motors[motor_num];
    motor->motor_num = motor_num;

    cJSON *item;
    if ((item = cJSON_GetObjectItem(motor_item, "np_mid"))) 
        motor->np_mid = item->valuedouble;
    
    if ((item = cJSON_GetObjectItem(motor_item, "np_ini"))) 
        motor->np_ini = item->valuedouble;
    
    if ((item = cJSON_GetObjectItem(motor_item, "pp_ini"))) 
        motor->pp_ini = item->valuedouble;
    
    if ((item = cJSON_GetObjectItem(motor_item, "pp_mid"))) 
        motor->pp_mid = item->valuedouble;
    
    if ((item = cJSON_GetObjectItem(motor_item, "nt_end"))) 
        motor->nt_end = item->valuedouble;
    
    if ((item = cJSON_GetObjectItem(motor_item, "nt_mid"))) 
        motor->nt_mid = item->valuedouble;
    
    if ((item = cJSON_GetObjectItem(motor_item, "pt_mid"))) 
        motor->pt_mid = item->valuedouble;
    
    if ((item = cJSON_GetObjectItem(motor_item, "pt_end"))) 
        motor->pt_end = item->valuedouble;
}