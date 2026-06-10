/**
 ******************************************************************************
 * @file    bsp_CAN.c
 * @author  Wang Hongxi
 * @version V1.1.0
 * @date    2021/3/30
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "bsp_CAN.h"
#include "bsp_dwt.h"
#include "detect_task.h"

uint8_t tempBuff[24] = {0};
uint8_t enable_navigation;
uint32_t Bsp_Can_Count = 0;
float dt_cap0727 = 0.0f; // 单向电容反馈的时间间隔
// uint8_t XData[4];
// uint8_t YData[4];

/**
 * @Func		CAN_Device_Init
 * @Brief
 * @Param		CAN_HandleTypeDef* hcan
 * @Retval		None
 * @Date       2019/11/4
 **/
// void CAN_Device_Init(CAN_HandleTypeDef *_hcan)
// {
//     // ��ʼ��CAN������Ϊ������״̬ ��Ϊ�������� �����
//     CAN_FilterTypeDef can_filter_st;
//     can_filter_st.FilterActivation = ENABLE;
//     can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
//     can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
//     can_filter_st.FilterIdHigh = 0x0000;
//     can_filter_st.FilterIdLow = 0x0000;
//     can_filter_st.FilterMaskIdHigh = 0x0000;
//     can_filter_st.FilterMaskIdLow = 0x0000;
//     can_filter_st.FilterBank = 0;
//     can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

//     // CAN2��CAN1��FilterBank��ͬ
//     if (_hcan == &hcan2)
//     {
//         can_filter_st.SlaveStartFilterBank = 14;
//         can_filter_st.FilterBank = 14;
//     }

//     // CAN ��������ʼ��
//     HAL_CAN_ConfigFilter(_hcan, &can_filter_st);
//     // ����CAN
//     HAL_CAN_Start(_hcan);
//     // ����֪ͨ
//     HAL_CAN_ActivateNotification(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
// }
void CAN_Device_Init(void)
{
	// ��ʼ��CAN������Ϊ������״̬ ��Ϊ�������� �����
	CAN_FilterTypeDef can_filter_st;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_st.FilterIdHigh = 0x0000;
	can_filter_st.FilterIdLow = 0x0000;
	can_filter_st.FilterMaskIdHigh = 0x0000;
	can_filter_st.FilterMaskIdLow = 0x0000;
	can_filter_st.FilterBank = 0;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	// CAN ��������ʼ��
	while (HAL_CAN_ConfigFilter(&hcan1, &can_filter_st) != HAL_OK)
	{
	}
	// ����CAN
	while (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
	}
	// ����֪ͨ
	while (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
	}

	can_filter_st.SlaveStartFilterBank = 14;
	can_filter_st.FilterBank = 14;
	// CAN ��������ʼ��
	while (HAL_CAN_ConfigFilter(&hcan2, &can_filter_st) != HAL_OK)
	{
	}
	// ����CAN
	while (HAL_CAN_Start(&hcan2) != HAL_OK)
	{
	}
	// ����֪ͨ
	while (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
	}
}

/**
 * @Func	    void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* _hcan)
 * @Brief      CAN���߽��ջص����� ���ڽ��յ������
 * @Param	    CAN_HandleTypeDef* _hcan
 * @Retval	    None
 * @Date       2019/11/4
 **/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *_hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	static uint8_t RC_Data_Buf[16];
	static uint8_t Follow_Data_Buf[8];
	static uint8_t MiniPC_CtrlFrame_Buf[8];
	static uint8_t Gimbal_Buf[8];
	static int16_t CAP_vol, CAP0727_vol;

	HAL_CAN_GetRxMessage(_hcan, CAN_RX_FIFO0, &rx_header, rx_data);

	if (_hcan == &hcan1)
	{
		if (rx_header.StdId >= 0x201 && rx_header.StdId <= 0x204)
		{
			if (Chassis.ChassisMotor[rx_header.StdId - 0x201].msg_cnt++ <= 50)
			{
				get_moto_offset(&Chassis.ChassisMotor[rx_header.StdId - 0x201], rx_data);
			}
			else
			{
				get_moto_info(&Chassis.ChassisMotor[rx_header.StdId - 0x201], rx_data);
			}
			switch (rx_header.StdId)
			{
			case 0x201:
			{
				Detect_Hook(CHASSIS_MOTOR1_TOE);
				break;
			}
			case 0x202:
			{
				Detect_Hook(CHASSIS_MOTOR2_TOE);
				break;
			}
			case 0x203:
			{
				Detect_Hook(CHASSIS_MOTOR3_TOE);
				break;
			}
			case 0x204:
			{
				Detect_Hook(CHASSIS_MOTOR4_TOE);
				break;
			}
			}
		}
		switch (rx_header.StdId)
		{
		case CAN_CAP_INFO_ID:
			CAP_vol = rx_data[0];
			Cap.Voltage = (float)CAP_vol / 5.0f;
			Cap.Power = (float)((rx_data[1] << 8 | rx_data[2]) / 10.0f);
			Cap.PowerIn = (float)((rx_data[3] << 8 | rx_data[4]) / 10.0f);
			Cap.Cap_Normol_Chassis_Flag = rx_data[5];
			Detect_Hook(CAP_TOE);
			break;
		case CAN_CAP0727_INFO_ID:
			CAP0727_vol = rx_data[0];
			Cap0727.Voltage = (float)CAP0727_vol / 5.0f;
			Cap0727.Power = (float)((rx_data[1] << 8 | rx_data[2]) / 10.0f);
			dt_cap0727 = DWT_GetDeltaT(&Bsp_Can_Count);
			Detect_Hook(CAP0727_TOE);
			break;
		}
	}
	if (_hcan == &hcan2)
	{
		switch (rx_header.StdId)
		{
		case CAN_RC_DATA_Frame_0:
			RC_Data_Buf[0] = rx_data[0];
			RC_Data_Buf[1] = rx_data[1];
			RC_Data_Buf[2] = rx_data[2];
			RC_Data_Buf[3] = rx_data[3];
			RC_Data_Buf[4] = rx_data[4];
			RC_Data_Buf[5] = rx_data[5];
			RC_Data_Buf[6] = rx_data[6];
			RC_Data_Buf[7] = rx_data[7];
			break;
		case CAN_RC_DATA_Frame_1:
			RC_Data_Buf[8] = rx_data[0];
			RC_Data_Buf[9] = rx_data[1];
			RC_Data_Buf[10] = rx_data[2];
			RC_Data_Buf[11] = rx_data[3];
			RC_Data_Buf[12] = rx_data[4];
			RC_Data_Buf[13] = rx_data[5];
			RC_Data_Buf[14] = rx_data[6];
			RC_Data_Buf[15] = rx_data[7];
			Callback_RC_Handle(&remote_control, RC_Data_Buf);
			break;
		case YAW_MOTOR_ID:
			Detect_Hook(GIMBAL_YAW_MOTOR_TOE);
			if (rx_data[6] != 0 && rx_data[7] != 0)
				get_RMD_info(&Gimbal.YawMotor, rx_data);
			break;
		case 0x250:
			tempBuff[0] = rx_data[0]; // NFX
			tempBuff[1] = rx_data[1]; // NFX
			tempBuff[2] = rx_data[2]; // NFY
			tempBuff[3] = rx_data[3]; // NFY
			tempBuff[4] = rx_data[4]; // TVX
			tempBuff[5] = rx_data[5]; // TVX
			tempBuff[6] = rx_data[6]; // TVY
			tempBuff[7] = rx_data[7]; // TVY

			Chassis.nowFaceX = ((int16_t)((tempBuff[1] << 8) | tempBuff[0])) / 1000.0f;
			Chassis.nowFaceY = ((int16_t)((tempBuff[3] << 8) | tempBuff[2])) / 1000.0f;
			Chassis.TargetVelocityXtemp = ((int16_t)((tempBuff[5] << 8) | tempBuff[4])) / 1000.0f;
			Chassis.TargetVelocityYtemp = ((int16_t)((tempBuff[7] << 8) | tempBuff[6])) / 1000.0f;
			break;
		case 0x251:
			tempBuff[8] = rx_data[0];  // NFX
			tempBuff[9] = rx_data[1];  // NFX
			tempBuff[10] = rx_data[2]; // NFY
			tempBuff[11] = rx_data[3]; // NFY
			tempBuff[12] = rx_data[4]; // TVX
			tempBuff[13] = rx_data[5]; // TVX
			tempBuff[14] = rx_data[6]; // TVY
			tempBuff[15] = rx_data[7]; // TVY
			Chassis.decision_info.TgtStatus = tempBuff[8];
			Chassis.decision_info.do_spin = tempBuff[9];
			Chassis.decision_info.open_cap = tempBuff[10];
			break;
		case CAN_GIMBAL_Info_ID:
			Gimbal_Buf[0] = rx_data[0];
			Gimbal_Buf[1] = rx_data[1];
			Gimbal_Buf[2] = rx_data[2];
			Gimbal_Data.MiniPC_state = Gimbal_Buf[0];
			Gimbal_Data.slope = Gimbal_Buf[1];
			Gimbal_Data.isrollover = Gimbal_Buf[2];
			break;
		}
	}
}

// ͨ��CAN���߷���ң������Ϣ ��������δʹ��
void Send_RC_Data(CAN_HandleTypeDef *_hcan, uint8_t *rc_data)
{
	static CAN_TxHeaderTypeDef TX_MSG;
	static uint8_t CAN_Send_Data[8];
	uint32_t send_mail_box;

	TX_MSG.StdId = CAN_RC_DATA_Frame_0;
	TX_MSG.IDE = CAN_ID_STD;
	TX_MSG.RTR = CAN_RTR_DATA;
	TX_MSG.DLC = 0x08;
	CAN_Send_Data[0] = rc_data[0];
	CAN_Send_Data[1] = rc_data[1];
	CAN_Send_Data[2] = rc_data[2];
	CAN_Send_Data[3] = rc_data[3];
	CAN_Send_Data[4] = rc_data[4];
	CAN_Send_Data[5] = rc_data[5];
	CAN_Send_Data[6] = rc_data[6];
	CAN_Send_Data[7] = rc_data[7];

	while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
	{
	}
	while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
	{
	}
	/* Check Tx Mailbox 1 status */
	if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX0;
	}
	/* Check Tx Mailbox 1 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX2;
	}
	HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);

	TX_MSG.StdId = CAN_RC_DATA_Frame_1;
	CAN_Send_Data[0] = rc_data[8];
	CAN_Send_Data[1] = rc_data[9];
	CAN_Send_Data[2] = rc_data[10];
	CAN_Send_Data[3] = rc_data[11];
	CAN_Send_Data[4] = rc_data[12];
	CAN_Send_Data[5] = rc_data[13];
	CAN_Send_Data[6] = rc_data[14];
	CAN_Send_Data[7] = rc_data[15];

	while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
	{
	}
	/* Check Tx Mailbox 1 status */
	if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX0;
	}
	/* Check Tx Mailbox 1 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX2;
	}
	HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void Send_VTM_Data(CAN_HandleTypeDef *_hcan, uint8_t *vtm_data)
{
	static CAN_TxHeaderTypeDef TX_MSG;
	static uint8_t CAN_Send_Data[8];
	uint32_t send_mail_box;

	TX_MSG.StdId = CAN_VTM_DATA_Frame_0;
	TX_MSG.IDE = CAN_ID_STD;
	TX_MSG.RTR = CAN_RTR_DATA;
	TX_MSG.DLC = 0x08;
	CAN_Send_Data[0] = vtm_data[0];
	CAN_Send_Data[1] = vtm_data[1];
	CAN_Send_Data[2] = vtm_data[2];
	CAN_Send_Data[3] = vtm_data[3];
	CAN_Send_Data[4] = vtm_data[4];
	CAN_Send_Data[5] = vtm_data[5];
	CAN_Send_Data[6] = vtm_data[6];
	CAN_Send_Data[7] = vtm_data[7];

	while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
	{
	}
	while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
	{
	}
	/* Check Tx Mailbox 1 status */
	if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX0;
	}
	/* Check Tx Mailbox 1 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX2;
	}
	HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);

	TX_MSG.StdId = CAN_VTM_DATA_Frame_1;
	CAN_Send_Data[0] = vtm_data[8];
	CAN_Send_Data[1] = vtm_data[9];
	CAN_Send_Data[2] = vtm_data[10];
	CAN_Send_Data[3] = vtm_data[11];
	CAN_Send_Data[4] = 0;
	CAN_Send_Data[5] = 0;
	CAN_Send_Data[6] = 0;
	CAN_Send_Data[7] = 0;

	while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
	{
	}
	while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
	{
	}
	/* Check Tx Mailbox 1 status */
	if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX0;
	}
	/* Check Tx Mailbox 1 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX2;
	}
	HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void Send_Power_Data(CAN_HandleTypeDef *_hcan, uint16_t Chassis_power_limit, uint16_t Chassis_power_buffer)
{
	if (Chassis_power_limit >= 10240)
		Chassis_power_limit /= 256;
	if (Chassis_power_limit >= 200)
		Chassis_power_limit /= 5;
	static CAN_TxHeaderTypeDef TX_MSG;
	static uint8_t CAN_Send_Data[8];
	uint32_t send_mail_box;

	TX_MSG.StdId = CAN_POWER_INFO_ID;
	TX_MSG.IDE = CAN_ID_STD;
	TX_MSG.RTR = CAN_RTR_DATA;
	TX_MSG.DLC = 0x08;
	CAN_Send_Data[0] = Chassis_power_limit >> 8;
	CAN_Send_Data[1] = Chassis_power_limit;
	CAN_Send_Data[2] = Chassis_power_buffer >> 8;
	CAN_Send_Data[3] = Chassis_power_buffer;
	CAN_Send_Data[4] = 0;
	CAN_Send_Data[5] = 0;
	CAN_Send_Data[6] = 0;
	CAN_Send_Data[7] = 0;

	while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
	{
	}
	while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0)
	{
	}
	/* Check Tx Mailbox 0 status */
	if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX0;
	}
	/* Check Tx Mailbox 1 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX2;
	}

	HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void Send_Reset_Command(CAN_HandleTypeDef *_hcan)
{
	static CAN_TxHeaderTypeDef TX_MSG;
	static uint8_t CAN_Send_Data[8];
	uint32_t send_mail_box;

	TX_MSG.StdId = CAN_SYSTEM_RESET_CMD;
	TX_MSG.IDE = CAN_ID_STD;
	TX_MSG.RTR = CAN_RTR_DATA;
	TX_MSG.DLC = 0x08;

	while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
	{
	}
	while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
	{
	}
	/* Check Tx Mailbox 1 status */
	if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX0;
	}
	/* Check Tx Mailbox 1 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX2;
	}
	HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void SendAerialData(CAN_HandleTypeDef *_hcan, float *X, float *Y, uint8_t *KeyBoard)
{
	static CAN_TxHeaderTypeDef TX_MSG;
	static uint8_t CAN_Send_Data[8];
	uint32_t send_mail_box;
	static uint8_t XData[4];
	static uint8_t YData[4];

	float2u8array(X, XData, TRUE);
	float2u8array(Y, YData, TRUE);

	TX_MSG.StdId = CAN_AERIAL_DATA_1;
	TX_MSG.IDE = CAN_ID_STD;
	TX_MSG.RTR = CAN_RTR_DATA;
	TX_MSG.DLC = 0x08;
	CAN_Send_Data[0] = XData[0];
	CAN_Send_Data[1] = XData[1];
	CAN_Send_Data[2] = XData[2];
	CAN_Send_Data[3] = XData[3];
	CAN_Send_Data[4] = YData[0];
	CAN_Send_Data[5] = YData[1];
	CAN_Send_Data[6] = YData[2];
	CAN_Send_Data[7] = YData[3];

	while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
	{
	}
	while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0)
	{
	}
	/* Check Tx Mailbox 1 status */
	if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX0;
	}
	/* Check Tx Mailbox 1 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX2;
	}
	HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);

	TX_MSG.StdId = CAN_AERIAL_DATA_2;
	TX_MSG.IDE = CAN_ID_STD;
	TX_MSG.RTR = CAN_RTR_DATA;
	TX_MSG.DLC = 0x08;
	CAN_Send_Data[0] = map_command.cmd_keyboard;
	CAN_Send_Data[1] = 0;
	CAN_Send_Data[2] = 0;
	CAN_Send_Data[3] = 0;
	CAN_Send_Data[4] = 0;
	CAN_Send_Data[5] = 0;
	CAN_Send_Data[6] = 0;
	CAN_Send_Data[7] = 77;

	while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
	{
	}
	while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
	{
	}
	/* Check Tx Mailbox 1 status */
	if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX0;
	}
	/* Check Tx Mailbox 1 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX2;
	}
	HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void Send_Robot_Info(CAN_HandleTypeDef *_hcan, uint8_t ID, uint16_t heatLimit, uint16_t heat1, uint16_t bulletSpeed, uint16_t cooling_value,
					 uint16_t UWBPosX, uint8_t IsEnemyInvincible, uint16_t UWBPosY, uint8_t gameStatus, uint8_t Inposflag)
{
	static CAN_TxHeaderTypeDef TX_MSG;
	static uint8_t CAN_Send_Data[8];
	uint32_t send_mail_box;

	TX_MSG.StdId = 0x133;
	TX_MSG.IDE = CAN_ID_STD;
	TX_MSG.RTR = CAN_RTR_DATA;
	TX_MSG.DLC = 0x08;
	CAN_Send_Data[0] = ID;
	CAN_Send_Data[1] = (heatLimit >> 8) & 0xFF;
	CAN_Send_Data[2] = heatLimit & 0xFF;
	CAN_Send_Data[3] = (heat1 >> 8) & 0xFF;
	CAN_Send_Data[4] = heat1 & 0xFF;
	CAN_Send_Data[5] = (bulletSpeed >> 8) & 0xFF;
	CAN_Send_Data[6] = bulletSpeed & 0xFF;

	while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
	{
	}
	while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
	{
	}
	/* Check Tx Mailbox 1 status */
	if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX0;
	}
	/* Check Tx Mailbox 1 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX2;
	}
	HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);

	TX_MSG.StdId = 0x134;
	TX_MSG.IDE = CAN_ID_STD;
	TX_MSG.RTR = CAN_RTR_DATA;
	TX_MSG.DLC = 0x08;

	if (Chassis.Mode == Follow_Mode)
		IsEnemyInvincible = 0;

	CAN_Send_Data[1] = (UWBPosX >> 8) & 0xFF;
	CAN_Send_Data[2] = UWBPosX & 0xFF;
	CAN_Send_Data[3] = IsEnemyInvincible;
	CAN_Send_Data[4] = (UWBPosY >> 8) & 0xFF;
	CAN_Send_Data[5] = UWBPosY & 0xFF;
	CAN_Send_Data[6] = Inposflag;
	CAN_Send_Data[7] = gameStatus;

	while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
	{
	}
	while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
	{
	}
	/* Check Tx Mailbox 1 status */
	if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX0;
	}
	/* Check Tx Mailbox 1 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX2;
	}
	HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void Send_Nav_Data_RMUC(CAN_HandleTypeDef *_hcan)
{
	static CAN_TxHeaderTypeDef TX_MSG;
	static uint8_t CAN_Send_Data[8];
	uint32_t send_mail_box;
	static uint16_t TargetX = 0, TargetY = 0;

	TargetX = (uint16_t)(map_command.target_position_x * 1000);
	TargetY = (uint16_t)(map_command.target_position_y * 1000);

	TX_MSG.StdId = 0x503;
	TX_MSG.IDE = CAN_ID_STD;
	TX_MSG.RTR = CAN_RTR_DATA;
	TX_MSG.DLC = 0x08;
	CAN_Send_Data[0] = Chassis.status;
	CAN_Send_Data[1] = Chassis.decision_info.allowToAttackOutpost;
	CAN_Send_Data[2] = (TargetX & 0xFF00) >> 8;
	CAN_Send_Data[3] = (TargetX & 0x00FF);
	CAN_Send_Data[4] = (TargetY & 0xFF00) >> 8;
	CAN_Send_Data[5] = (TargetY & 0x00FF);
	CAN_Send_Data[6] = Chassis.decision_info.isEnemyPostAlive;
	CAN_Send_Data[7] = 77;

	while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
	{
	}
	while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
	{
	}
	/* Check Tx Mailbox 1 status */
	if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX0;
	}
	/* Check Tx Mailbox 1 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX2;
	}
	HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void float2u8array(float *FloatData, uint8_t *u8Array, bool Key)
{
	static uint8_t TempU8Arr[4];

	*(float *)TempU8Arr = *FloatData;
	if (Key == TRUE)
	{
		u8Array[0] = TempU8Arr[0];
		u8Array[1] = TempU8Arr[1];
		u8Array[2] = TempU8Arr[2];
		u8Array[3] = TempU8Arr[3];
	}
	else
	{
		u8Array[3] = TempU8Arr[0];
		u8Array[2] = TempU8Arr[1];
		u8Array[1] = TempU8Arr[2];
		u8Array[0] = TempU8Arr[3];
	}
}