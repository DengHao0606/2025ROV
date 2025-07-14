#ifndef __COORDINATE_SYSTEM_H
#define __COORDINATE_SYSTEM_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifndef deg2rad
#define deg2rad 0.01745329252f
#endif

#ifndef rad2deg
#define rad2deg 57.29577951f
#endif


#ifndef pi
#define pi 3.141592654f
#endif

typedef struct {
    float x;
    float y;
    float z;
    float rx;       // 绕X的旋转角 pitch
    float ry;       // 绕Y的旋转角 roll
    float rz;       // 绕Z的旋转角 yaw
} CoordinateVector; // 一个结构体，用于描述与机器人相关的坐标系统

/*
此处仅考虑机器人的位置变换关系
矩阵的分布为逐列从上至下
映射关系如下
0  | 1  | 2  | 3
4  | 5  | 6  | 7
8  | 9  | 10 | 11
12 | 13 | 14 | 15
*/
typedef struct {
    CoordinateVector vector;       // 位置与姿态
    float            h_matrix[16]; // 齐次矩阵
    void (*extract)(void *);       // 将 位置姿态参数 导出为 齐次矩阵
    void (*r_extract)(void *);     // 将 齐次矩阵 导出为 位置姿态参数
} MotionState; // 一个结构体，用于某一坐标系中机器人的运动状态

typedef struct {
    MotionState base;           // 世界坐标系 中的 机器人 的运动状态
    MotionState target_inbase;  // 机器人坐标系 中的 目标 的运动状态
    MotionState target_inworld; // 世界坐标系 中的 目标 的运动状态

    void (*world2base)(void *); // 坐标系变换 世界 => 机器人
    void (*base2world)(void *); // 坐标系变换 机器人 => 世界
} CoordinateSystems;            // 完整的描述


void  CoordinateSystems_Init(CoordinateSystems *CSs);
float AngleCorrect(float angle);

#endif
