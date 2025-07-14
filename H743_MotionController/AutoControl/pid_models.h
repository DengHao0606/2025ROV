#ifndef __PID_MODELS_H
#define __PID_MODELS_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// 位置式PID
typedef struct
{
    // 控制器参数
    float kp;
    float ki;
    float kd;
    float i_limit;
    float output_limit;

    // 状态参量
    float output;
    float delta;   // 差值
    float delta_i; // 差值积分
    float delta_d; // 差值微分

    float delta_l; // 上一步差值
    int delta_cnt; // 上一步差值 时间

    // 不参与运算
    // int cnt;  // 模型完成后的时间

    // 不难发现 K与delta_d等价
    // 一次函数模型中的 b 项无效
    float k, b; // 一次函数 参量

    float (*refresh)(void *, float); // 控制器更新
} PositionalPID;

void PositionalPID_Init(PositionalPID *Ppid, float kp, float ki, float kd, float i_limit, float output_limit);

#endif
