#include "filter.h"

float MeanFilter_Refresh(void *obj, float input)
{
    MeanFilter *f = (MeanFilter *)obj;

    volatile float output = 0;

    f->list[f->length - 1] = input;
    for (int i = 1; i < f->length; i++) { f->list[i - 1] = f->list[i]; }

    for (int i = 0; i < f->length; i++) { output += f->list[i]; }


    return output / ((float)f->length);
}

void MeanFilter_Init(MeanFilter *filter, int length)
{
    if (length < 2) length = 2;
    if (length > 256) length = 256;
    filter->length = length;

    for (int i = 0; i < length; i++) { filter->list[i] = 0; }

    filter->refresh = MeanFilter_Refresh;
}

void KalmanFilter(float measured_value, float *pchecked_value)
{
    static float x_est = 0.0; // 估计值
    static float p     = 1.0; // 方差
    static float k     = 0.0; // 卡尔曼增益

    // 预测
    float x_pred = x_est;
    float p_pred = p + KalmanFilter_Q;

    // 更新
    k     = p_pred / (p_pred + KalmanFilter_R);
    x_est = x_pred + k * (measured_value - x_pred);
    p     = (1 - k) * p_pred;

    *pchecked_value = x_est;
}
