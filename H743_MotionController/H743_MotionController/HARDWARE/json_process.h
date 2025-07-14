#ifndef __JSON_PROCESS_H
#define __JSON_PROCESS_H

#include "cJSON.h"
#include "main.h"

// typedef struct {
//   uint8_t          buf[512];      // 接收数据缓存数组
//   volatile uint8_t rx_len;        // 接收一帧数据的长度
//   volatile uint8_t recv_end_flag; // 一帧数据接收完成标志
//   int              cnt;
// } RecBuf;

// 定义电机参数结构体
typedef struct {
    int motor_num;
    float np_mid;
    float np_ini;
    float pp_ini;
    float pp_mid;
    float nt_end;
    float nt_mid;
    float pt_mid;
    float pt_end;
} MotorParams_t;


/*JSON 数据中的各个键值对，用于存储解析后的相关数据*/
typedef struct {
  float x;               //  x 坐标值，浮点数类型
  float y;               //  y 坐标值，浮点数类型
  float z;               //  z 坐标值，浮点数类型
  float roll;            // 翻滚角度值，浮点数类型
  float pitch;           // 俯仰角度值，浮点数类型
  float yaw;             // 偏航角度值，浮点数类型
  float servo0;          // 舵机 0 的值，浮点数类型
  float servo1;          // 舵机 1 的值，浮点数类型
  int state;           // 模式状态 的值，浮点数类型
  MotorParams_t motors[6];
} JSON_Command_t;



void JSON_Process_Init(void);
void JSON_Process_Data(uint8_t *json_str);
static void parse_thrust_params(cJSON *motor_item, int motor_num);
#endif