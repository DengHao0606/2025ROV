#include "can_process.h"

#include "fdcan.h"
#include <string.h>

CanRx canfd1_rx = {0};

/*
 * name     :FDCAN1_Config
 * discript :FDCAN1初始化
 * input    :
 * output   :
 * remark   :
 */
void FDCAN1_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    // 标准段滤波器初始化0x000~0x0FF
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex  = 1;
    sFilterConfig.FilterType   = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1    = 0x00000000;
    sFilterConfig.FilterID2    = 0x000000FF;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

    // 标准段滤波器初始化0x200~0x2FF
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex  = 2;
    sFilterConfig.FilterType   = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1    = 0x00000200;
    sFilterConfig.FilterID2    = 0x000002FF;
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
            FDCAN_RxHeaderTypeDef RxHeader; // 用来保存接收到的数据帧头部信号
            uint8_t RxData[64];             // 用来保存接收数据端数据

            if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) // 从接收队列中读取数据
            {
                HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
    //             FDCAN_TxHeaderTypeDef TxHeader = {0}; // 用来保存发送数据帧头部信息
    //             uint8_t TxData[64];                   // 用来保存发送数据帧数据

    //             TxHeader.Identifier  = RxHeader.Identifier + 3;
    //             TxHeader.IdType      = RxHeader.IdType;                      // 标准-FDCAN_STANDARD_ID; 扩展-FDCAN_EXTENDED_ID
    //             TxHeader.TxFrameType = RxHeader.RxFrameType;                 // 数据帧-FDCAN_DATA_FRAME; 远程帧-FDCAN_REMOTE_FRAME
    //             TxHeader.DataLength  = RxHeader.DataLength;                  // FDCAN_DLC_BYTES_xx
    //                                                                          // xx = 0 1 2 3 4 5 6 7 8 12 16 20 24 32 48 64
    //             TxHeader.ErrorStateIndicator = RxHeader.ErrorStateIndicator; // FDCAN_ESI_ACTIVE FDCAN_ESI_PASSIVE
    //             TxHeader.BitRateSwitch       = RxHeader.BitRateSwitch;       // 波特率不可变-FDCAN_BRS_OFF; 波特率可变-FDCAN_BRS_ON
    //             TxHeader.FDFormat            = RxHeader.FDFormat;            // 经典CAN-FDCAN_CLASSIC_CAN; CANFD-FDCAN_FD_CAN
    //             // TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    //             // TxHeader.MessageMarker = 0;

    //             for (int i = 0; i < 64; i++) { TxData[i] = RxData[i]; }

    //             while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0)
    //                 ; // 等待有发送邮箱可用

    //             HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData); // 发送数据帧
            }
        }
    }
}

/*
 * name     :Canfd1Transmit64
 * discript :CANFD1 发送函数
 * input    :uint8_t id             CAN ID
 *           uint8_t * can1_txbuf   待发送数据
 * output   :
 * remark   :发送数据长度64位
 */
void Canfd1Transmit64(uint8_t id, uint8_t *can1_txbuf)
{
    FDCAN_TxHeaderTypeDef txheader1;

    txheader1.Identifier          = id;                // CAN ID
    txheader1.IdType              = FDCAN_STANDARD_ID; // 标准帧
    txheader1.TxFrameType         = FDCAN_DATA_FRAME;
    txheader1.DataLength          = FDCAN_DLC_BYTES_64;
    txheader1.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txheader1.BitRateSwitch       = FDCAN_BRS_ON;
    txheader1.FDFormat            = FDCAN_FD_CAN; // CANFD
    txheader1.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txheader1.MessageMarker       = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txheader1, can1_txbuf);
}