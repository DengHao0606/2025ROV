#include "pid_models.h"

/*
 * 函数名: Abs
 * 描述  : 绝对值函数
 * 输入  : 待求绝对值
 * 输出  : 绝对值
 * 备注  : /
 */
float Abs(float input)
{
    if (input < 0)
        return -input;
    else
        return input;
}

/*
 * 函数名: Limit
 * 描述  : 限幅函数
 * 输入  : float input 待限幅量
 *         float limit 限制
 * 输出  : 略
 * 备注  : /
 */
float Limit(float input, float limit)
{
    if (input < -Abs(limit))
        return -Abs(limit);
    else if (input > Abs(limit))
        return Abs(limit);
    else
        return input;
}

/*
 * 函数名: PositionalPID_Refresh
 * 描述  : 位置式PID控制器更新
 * 输入  : 差值
 * 输出  : /
 * 备注  : /
 */
float PositionalPID_Refresh(void *obj, float delta)
{
    PositionalPID *Ppid   = (PositionalPID *)obj;
    float          output = 0;
    // 不难发现 K与delta_d等价
    // 一次函数模型中的 b 项无效
    // 对传感器更新速率较低的情况进行补偿
    if (fabs(Ppid->delta_l - delta) < fabs(delta*0.01))
    {
        // 传感器频率 小于 采样频率
        Ppid->delta_cnt++;
        Ppid->delta_d = Ppid->k;
    }
    else
    {
        if (Ppid->delta_cnt == 0) { Ppid->delta_d = delta - Ppid->delta_l; }
        else { Ppid->delta_d = Ppid->k = (delta - Ppid->delta_l) / Ppid->delta_cnt; }

        Ppid->delta_l   = delta;
        Ppid->delta_cnt = 0;
    }

    Ppid->delta = delta;

    Ppid->delta_i += delta;

    if (Ppid->i_limit != 0) Ppid->delta_i = Limit(Ppid->delta_i, Ppid->i_limit); // 积分限幅

    output = Ppid->kp * Ppid->delta + Ppid->ki * Ppid->delta_i + Ppid->kd * Ppid->delta_d;

    if (Ppid->output_limit != 0) output = Limit(output, Ppid->output_limit); // 输出限幅

    return output;
}

/*
 * 函数名: PositionalPID_Init
 * 描述  : 位置式PID控制器初始化
 * 输入  : 控制器参数
 * 输出  : /
 * 备注  : /
 */
void PositionalPID_Init(PositionalPID *Ppid, float kp, float ki, float kd, float i_limit, float output_limit)
{
    // 参量初始化
    Ppid->kp           = kp;
    Ppid->ki           = ki;
    Ppid->kd           = kd;
    Ppid->i_limit      = i_limit;
    Ppid->output_limit = output_limit;

    Ppid->delta     = 0.0f;
    Ppid->delta_i   = 0.0f;
    Ppid->delta_d   = 0.0f;
    Ppid->delta_l   = 0.0f;
    Ppid->delta_cnt = 1;
    Ppid->k         = 0.0f;

    // 方法绑定
    Ppid->refresh = PositionalPID_Refresh;
}
