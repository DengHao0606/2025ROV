#include "thrust.h"

/*
 * 函数名: ThrustAllocate
 * 描述  : 推力分配
 * 输入  : askedthrust 0~5 对应 x y z rx ry rz
 *         motorthrust 电机被分配的推力 0~5 对应电机 0~5
 * 输出  : /
 * 备注  : /
 */
void ThrustAllocate(float *askedthrust, float *motorthrust)
{
    // motorthrust[0] = -0.4125f *askedthrust[0] + 0.3480f * askedthrust[2] + 2.6630f * askedthrust[4] + 1.9254f * askedthrust[5];
    // motorthrust[1] = 0.4119f *askedthrust[0] + 0.3481f * askedthrust[2] + 2.6652f * askedthrust[4] - 1.9218f * askedthrust[5];
    // motorthrust[2] = 0.1065f *askedthrust[0] + 0.5003f * askedthrust[1] + 2.4164f * askedthrust[5] - 4.7794f * askedthrust[3];
    // motorthrust[3] = -0.1065f *askedthrust[0] + 0.4997f * askedthrust[1] - 2.4164f * askedthrust[5] + 4.7794f * askedthrust[3];
    // motorthrust[4] = -0.4598f *askedthrust[0] - 0.2623f * askedthrust[2] + 2.6652f * askedthrust[4] - 1.9218f * askedthrust[5];
    // motorthrust[5] = 0.4593f *askedthrust[0] - 0.2624f * askedthrust[2] + 2.6630f * askedthrust[4] + 1.9254f * askedthrust[5];
/***
    motorthrust[0] = -0.4125f *askedthrust[0] + 0.3480f * askedthrust[2] + 2.6630f * askedthrust[4] ;
    motorthrust[1] =  0.4119f *askedthrust[0] + 0.3481f * askedthrust[2] + 2.6652f * askedthrust[4] ;
    motorthrust[2] =  0.1065f *askedthrust[0] + 0.5003f * askedthrust[1] + 2.4164f * askedthrust[5] - 4.7794f * askedthrust[3];
    motorthrust[3] = -0.1065f *askedthrust[0] + 0.4997f * askedthrust[1] - 2.4164f * askedthrust[5] + 4.7794f * askedthrust[3];
    motorthrust[4] = -0.4598f *askedthrust[0] - 0.3480f * askedthrust[2] + 2.6652f * askedthrust[4] ;
    motorthrust[5] =  0.4593f *askedthrust[0] - 0.3481f * askedthrust[2] + 2.6630f * askedthrust[4] ;
*/

    motorthrust[0] = -0.4370f * askedthrust[0]  + 0.3051f * askedthrust[1] + askedthrust[5] / 0.4501f;
    motorthrust[1] = 0.4370f * askedthrust[0] + 0.3051f * askedthrust[1] - askedthrust[5] / 0.4501f;
    motorthrust[2] = -0.5 * askedthrust[2] + askedthrust[4] / 0.212;
    motorthrust[3] = -0.5 * askedthrust[2] - askedthrust[4] / 0.212;
    motorthrust[4] = -0.4347 * askedthrust[0]  - 0.3051f * askedthrust[1] - askedthrust[5] / 0.4501f;
    motorthrust[5] = 0.4347 * askedthrust[0]  - 0.3051f * askedthrust[1] + askedthrust[5] / 0.4501f;

}

