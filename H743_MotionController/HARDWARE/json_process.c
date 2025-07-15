#include "json_process.h"
#include "comm.h"
#include "motor.h"
#include "RS485_process.h"

#include "usart.h"
#include "tim.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

float servo0angle = 0.5;
extern int threadmonitor_uart8;
JSON_Command_t command = {0};
extern float received_depth;
extern float received_temp;
extern uint8_t can_rx_data;

static void parse_json_data(uint8_t *json_str);
static void apply_motor_control(void);

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
        //   printf("Error before: %s\n", error_ptr);
      }  
      return;
  }

  cJSON *x_item = cJSON_GetObjectItem(root, "x");
    if  (x_item) command.x = x_item->valuedouble;

  cJSON *y_item = cJSON_GetObjectItem(root, "y");
    if  (y_item) command.y = y_item->valuedouble;

  cJSON *z_item = cJSON_GetObjectItem(root, "z");
    if  (z_item) command.z = z_item->valuedouble;

  cJSON *yaw_item = cJSON_GetObjectItem(root, "yaw");
    if  (yaw_item) command.yaw = yaw_item->valuedouble;
    // 解析舵机角度
  cJSON *servo0_item = cJSON_GetObjectItem(root, "servo0");
    if (servo0_item) command.servo0 = servo0_item->valuedouble;

    openloop_thrust[0] = command.x;
    openloop_thrust[1] = command.y;
    openloop_thrust[2] = command.z;
    openloop_thrust[3] = command.roll;
    openloop_thrust[4] = command.yaw;
    openloop_thrust[5] = command.pitch;

    servo0angle = command.servo0;
    
    /* 电机参数解析 */
    cJSON *cmd_item = cJSON_GetObjectItem(root, "cmd");
    if (cmd_item && strcmp(cmd_item->valuestring, "thrust_init") == 0)
    {
        // 格式1: {"cmd":"thrust_init", "motor":0, "np_mid":1.0...}
        int motor_num = cJSON_GetObjectItem(root, "motor")->valueint;
        parse_thrust_params(root, motor_num);
        apply_thrust_params_to_curve(motor_num);  // 新增这行
    } 
    
//   HAL_UART_Transmit_IT(&huart1,(uint16_t)&servo0angle, 1);//调试用

    static uint32_t last_upload_time = 0;
    if(HAL_GetTick() - last_upload_time > 100) 
    { // 每1秒上传一次
        UploadCurrentPWMOutput();
        last_upload_time = HAL_GetTick();
    }

    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_7);
    cJSON_Delete(root);
}

/**
  * @brief  仅提取电机推力参数
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

void apply_thrust_params_to_curve(int motor_num) 
{
    if (motor_num < 0 || motor_num >= 6) return;
    
    MotorParams_t *motor = &command.motors[motor_num];
    ThrustCurve *curve = &thrustcurve[motor_num];
    
    // 映射PWM参数
    curve->pwm[0] = motor->np_mid;
    curve->pwm[1] = motor->np_ini;
    curve->pwm[2] = motor->pp_ini;
    curve->pwm[3] = motor->pp_mid;
    
    // 映射推力参数
    curve->thrust[0] = motor->nt_end;
    curve->thrust[1] = motor->nt_mid;
    curve->thrust[2] = motor->pt_mid;
    curve->thrust[3] = motor->pt_end;
}

// 将realdepth和temperature打包成json发送到上位机
void send_depth_temperature()
{
    static uint8_t send_buf[128]; // 专用发送缓冲区
    cJSON *root = cJSON_CreateObject();
    if (!root) return;

    // 使用cJSON_AddNumberToObject直接添加数字
    cJSON_AddNumberToObject(root, "depth", received_depth);
    cJSON_AddNumberToObject(root, "temperature", received_temp);

    char *json_str = cJSON_PrintUnformatted(root);
    if (json_str) {
        size_t len = strlen(json_str);
        
        // 使用专用缓冲区
        if (len + 3 < sizeof(send_buf)) {
            memcpy(send_buf, json_str, len);
            send_buf[len] = '\r';
            send_buf[len+1] = '\n';
            // 使用阻塞发送（非中断模式）
            HAL_UART_Transmit(&huart1, send_buf, len + 2, 100);
        }
        
        free(json_str);
    }
    cJSON_Delete(root);
}
