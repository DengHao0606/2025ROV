#include "coordinate_system.h"

/*
 * 函数名: AngleCorrect
 * 描述  : 角度矫正
 * 输入  : float  待矫正角度
 * 输出  : float  正确角度
 * 备注  : /
 */
float AngleCorrect(float angle)
{
    int cnt = 0;
    if (angle > 180)
    {
        cnt = (int)((angle + 180) / 360);
        angle -= cnt * 360;
    }
    if (angle < -180)
    {
        cnt = (int)((angle - 180) / 360);
        angle -= cnt * 360;
    }
    return angle;
}

/*
 * 函数名: matrixInverse
 * 描述  : 4*4 矩阵求逆
 * 输入  : 略
 * 输出  : /
 * 备注  : /
 */
void swap(float *a, float *b)
{
    float temp = *a;
    *a         = *b;
    *b         = temp;
}
void matrixInverse(float *input, float *result)
{
    float matrix[4][8];

    // 初始化增广矩阵
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            matrix[i][j]     = input[i * 4 + j];
            matrix[i][j + 4] = (i == j) ? 1.0f : 0.0f;
        }
    }

    // 消元过程
    for (int i = 0; i < 4; i++)
    {
        // 如果当前行的对角线元素为0，则需要进行行交换
        if (matrix[i][i] == 0)
        {
            int row = i + 1;
            while (row < 4 && matrix[row][i] == 0) { row++; }

            // 如果找到了非零元素的行，则进行交换
            if (row < 4)
            {
                for (int j = 0; j < 8; j++) { swap(&matrix[i][j], &matrix[row][j]); }
            }
        }

        // 将对角线元素调整为1
        float scale = matrix[i][i];
        for (int j = 0; j < 8; j++) { matrix[i][j] /= scale; }

        // 消元
        for (int j = 0; j < 4; j++)
        {
            if (j != i)
            {
                float factor = matrix[j][i];
                for (int k = 0; k < 8; k++) { matrix[j][k] -= factor * matrix[i][k]; }
            }
        }
    }

    // 提取逆矩阵部分
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++) { result[i * 4 + j] = matrix[i][j + 4]; }
    }
}


/*
 * 函数名: multiplyMatrix
 * 描述  : 4*4 矩阵乘法
 * 输入  : 略
 * 输出  : /
 * 备注  : /
 */
void multiplyMatrix(float *m1, float *m2, float *result)
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            result[i * 4 + j] = 0.0;
            for (int k = 0; k < 4; k++) { result[i * 4 + j] += m1[i * 4 + k] * m2[k * 4 + j]; }
        }
    }
}

/*
 * 函数名: Target_CS_world2base
 * 描述  :
 * 对象CoordinateSystems的子方法，用于将目标的绝对坐标系中的运动状态参量转换为相对坐标系中的运动状态参量
 * 输入  : /
 * 输出  : /
 * 备注  : 仅计算位置、姿态的变换
 */
void Target_CS_world2base(void *obj)
{
    CoordinateSystems *CSs      = (CoordinateSystems *)obj;
    float              base[16] = {0};

    matrixInverse(CSs->base.h_matrix, base);
    multiplyMatrix(base, CSs->target_inworld.h_matrix, CSs->target_inbase.h_matrix);

    // 解算出欧拉角
    CSs->target_inbase.r_extract(&(CSs->target_inbase));
}

/*
 * 函数名: Target_CS_base2world
 * 描述  :
 * 对象CoordinateSystems的子方法，用于将目标的相对坐标系中的运动状态参量转换为绝对坐标系中的运动状态参量
 * 输入  : /
 * 输出  : /
 * 备注  : 仅计算位置、姿态的变换
 */
void Target_CS_base2world(void *obj)
{
    CoordinateSystems *CSs = (CoordinateSystems *)obj;

    // 矩阵相乘    base * target_inbase = target_inworld
    multiplyMatrix(CSs->base.h_matrix, CSs->target_inbase.h_matrix, CSs->target_inworld.h_matrix);

    // 解算出欧拉角
    CSs->target_inworld.r_extract(&(CSs->target_inworld));
}

/*
 * 函数名: MotionState_Extract
 * 描述  : 将 位置姿态参数 导出为 齐次矩阵
 * 输入  : /
 * 输出  : /
 * 备注  : /
 */
void MotionState_Extract(void *obj)
{
    MotionState *Ms = (MotionState *)obj;

    float Sx = sin(Ms->vector.rx * deg2rad);
    float Cx = cos(Ms->vector.rx * deg2rad);
    float Sy = sin(Ms->vector.ry * deg2rad);
    float Cy = cos(Ms->vector.ry * deg2rad);
    float Sz = sin(Ms->vector.rz * deg2rad);
    float Cz = cos(Ms->vector.rz * deg2rad);

    // 矩阵计算
    Ms->h_matrix[0] = Cy * Cz;
    Ms->h_matrix[1] = Sx * Sy * Cz - Cx * Sz;
    Ms->h_matrix[2] = Cx * Sy * Cz + Sx * Sz;
    Ms->h_matrix[3] = Ms->vector.x;

    Ms->h_matrix[4] = Cy * Sz;
    Ms->h_matrix[5] = Sx * Sy * Sz + Cx * Cz;
    Ms->h_matrix[6] = Cx * Sy * Sz - Sx * Cz;
    Ms->h_matrix[7] = Ms->vector.y;

    Ms->h_matrix[8]  = -Sy;
    Ms->h_matrix[9]  = Sx * Cy;
    Ms->h_matrix[10] = Cx * Cy;
    Ms->h_matrix[11] = Ms->vector.z;

    Ms->h_matrix[12] = 0.0f;
    Ms->h_matrix[13] = 0.0f;
    Ms->h_matrix[14] = 0.0f;
    Ms->h_matrix[15] = 1.0f;
}

/*
 * 函数名: MotionState_ReExtract
 * 描述  : 将 齐次矩阵 导出为 位置姿态参数
 * 输入  : /
 * 输出  : /
 * 备注  : /
 */
void MotionState_ReExtract(void *obj)
{
    MotionState *Ms = (MotionState *)obj;

    Ms->vector.x = Ms->h_matrix[3];
    Ms->vector.y = Ms->h_matrix[7];
    Ms->vector.z = Ms->h_matrix[11];

    float rx1, rx2;
    float ry1, ry2;

    if (Ms->h_matrix[8] != 1.0f || Ms->h_matrix[8] != -1.0f) // 若越界，放弃更新
    {
        ry1 = -asin(Ms->h_matrix[8]);
        ry2 = pi - ry1;

        rx1 = atan2(Ms->h_matrix[9] / cos(ry1), Ms->h_matrix[10] / cos(ry1));
        rx2 = atan2(Ms->h_matrix[9] / cos(ry2), Ms->h_matrix[10] / cos(ry2));

        if (ry1 < pi / 2 && ry1 > -pi / 2) Ms->vector.ry = ry1 * rad2deg;
        if (ry2 < pi / 2 && ry2 > -pi / 2) Ms->vector.ry = ry2 * rad2deg;

        if (rx1 < pi / 2 && rx1 > -pi / 2) Ms->vector.rx = rx1 * rad2deg;
        if (rx2 < pi / 2 && rx2 > -pi / 2) Ms->vector.rx = rx2 * rad2deg;

        // if (rz1 < pi && rz1 > -pi)
        Ms->vector.rz = atan2(Ms->h_matrix[4] / cos(Ms->vector.ry * deg2rad), Ms->h_matrix[0] / cos(Ms->vector.ry * deg2rad)) * rad2deg;
        // if (rz2 < pi && rz2 > -pi)
        //     Ms->vector.rz = rz2 * rad2deg;
    }
}

/*
 * 函数名: Motionstate_Init
 * 描述  : 机器人的运动状态描述量初始化
 * 输入  : MotionState * input
 * 输出  : /
 * 备注  : 赋值函数
 */
void Motionstate_Init(MotionState *input)
{
    input->vector.x  = 0.0f;
    input->vector.y  = 0.0f;
    input->vector.z  = 0.0f;
    input->vector.rx = 0.0f;
    input->vector.ry = 0.0f;
    input->vector.rz = 0.0f;

    input->extract   = MotionState_Extract;
    input->r_extract = MotionState_ReExtract;

    input->extract(input);
}

/*
 * 函数名: CoordinateSystems_Init
 * 描述  : 将机器人坐标描述初始化
 * 输入  : /
 * 输出  : CoordinateSystems_Init 完整的坐标系系统 对象
 * 备注  : /
 */
void CoordinateSystems_Init(CoordinateSystems *CSs)
{
    // 成员变量初始化
    Motionstate_Init(&(CSs->base));
    Motionstate_Init(&(CSs->target_inbase));
    Motionstate_Init(&(CSs->target_inworld));

    // 绑定方法
    CSs->world2base = Target_CS_world2base;
    CSs->base2world = Target_CS_base2world;
}
