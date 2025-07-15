#ifndef __FILTER_H
#define __FILTER_H

#define KalmanFilter_Q 0.05     //过程噪声协方差
#define KalmanFilter_R 0.4      //测量噪声协方差
#define FLITER_LENGTH 10        //滑动窗口长度

typedef struct {
    int   length;
    float list[256];
    float (*refresh)(void *,float input);
} MeanFilter;

void MeanFilter_Init(MeanFilter *filter, int length);
void KalmanFilter(float measured_value, float *pchecked_value);

#endif