#include "can_process.h"
#include "fdcan.h"
#include <string.h>
#include <stdio.h>

CanRx canfd1_rx = {0};

/*
 * name     :FDCAN1_Config
 * discript :FDCAN1初始�??
 * input    :
 * output   :
 * remark   :
 */
void FDCAN1_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    // 标准段初始化
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex  = 1;
    sFilterConfig.FilterType   = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1    = 0x00000000;
    sFilterConfig.FilterID2    = 0x00000FFF;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

    // 扩展段初始化
    sFilterConfig.IdType       = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterIndex  = 0;
    sFilterConfig.FilterType   = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1    = 0x10000000;
    sFilterConfig.FilterID2    = 0x1FFFFFFF;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

    // 以下为启动中断
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // 使能FIFO0数据接收中断

    // 以下为启动CAN外设
    HAL_FDCAN_Start(&hfdcan1);
}

/*
 * name     :HAL_FDCAN_RxFifo0Callback
 * discript :FDCAN中断回调函数
 * input    :FDCAN_HandleTypeDef *hfdcan    传入中断的fdcan
 *           uint32_t RxFifo0ITs            中断标志
 * output   :
 * remark   :
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan == &hfdcan1) // 判断是hfdcan1的中断
    {
        if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) // 判断是FIFO0_NEW_MESSAGE回调
        {
            FDCAN_RxHeaderTypeDef RxHeader;   // 用来保存接收到的数据帧头部信号
            uint8_t               RxData[64]; // 用来保存接收数据端数据
            uint8_t               cmd = 0;
            uint32_t              id  = 0;

            if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) // 从接收队列中读取数据帧
            {
                cmd = (uint8_t)(RxHeader.Identifier & 0x0ff);
                id  = RxHeader.Identifier - (uint32_t)cmd;
                printf("%c\r\n", RxData[60]);
                HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_6);
            }
        }
    }
}
