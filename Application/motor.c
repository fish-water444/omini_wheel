#include "motor.h"

Motor_Feedback_t motor_fb[8];
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

void CAN_User_Init(void)//can初始化
{
    CAN_FilterTypeDef can_filter_st;

    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow  = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow  = 0x0000;

    if (HAL_CAN_ConfigFilter(&hcan1, &can_filter_st) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
}

void C620_SendCurrents(CAN_HandleTypeDef *hcan, int16_t currents[4])//向 C620 电调发送电流控制指令
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;

    tx_header.StdId = 0x200;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    for (int i = 0; i < 4; i++)
    {
        tx_data[2 * i]     = (currents[i] >> 8) & 0xFF;
        tx_data[2 * i + 1] =  currents[i]       & 0xFF;
    }

    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0);

    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//CAN 接收中断回调，处理 C620 发回的反馈数据
{
    if (hcan->Instance == CAN1)
    {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];

        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

        if (rx_header.StdId >= 0x201 && rx_header.StdId <= 0x208)
        {
            uint8_t index = rx_header.StdId - 0x201;

            motor_fb[index].angle         = (rx_data[0] << 8) | rx_data[1];
            motor_fb[index].speed_rpm     = (rx_data[2] << 8) | rx_data[3];
            motor_fb[index].torque_current = (rx_data[4] << 8) | rx_data[5];
            motor_fb[index].temperature   = rx_data[6];
        }
    }
}
