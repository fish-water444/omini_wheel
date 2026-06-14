/**
 * @file    bsp_CAN.c
 * @author  Wang Hongxi
 * @author  Lu Yurui
 * @version V2.0
 * @date    2026/06/02
 *
 * @brief   CAN通信初始化、接收解析与业务协议发送
 *
 * @note
 * V1.0 (2021/03/30) Wang Hongxi
 * Initial version.
 *
 * V2.0 (2026/06/02) Lu Yurui
 * Refactored and upgraded CAN communication framework.
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_CAN.h"
#include "VTM_info.h"
#include "bsp_dwt.h"
#include "user_lib.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "detect_task.h"
#include "remote_control.h"
#include "cap.h"
#include "judgement_info.h"

/* Private macros ------------------------------------------------------------*/

// CAN发送邮箱等待超时计数
#define CAN_TX_WAIT_MAX 10000U

/* Private variables ---------------------------------------------------------*/

CAN_ErrorInfo_t CAN1_ErrorInfo = {0};
CAN_ErrorInfo_t CAN2_ErrorInfo = {0};

uint8_t tempBuff[24] = {0};
uint8_t enable_navigation;
uint32_t Bsp_Can_Count = 0;
float dt_cap0727 = 0.0f; // 单向电容反馈的时间间隔
uint8_t tgtStatus;
uint8_t set_posture_mode = 0, debug_posture = 3;

/* Private function declarations ---------------------------------------------*/

/**
 * @brief 配置CAN接收过滤器
 *
 * @param _hcan CAN编号
 * @param filter_bank 滤波器编号
 * @return HAL_StatusTypeDef 执行状态
 */
static HAL_StatusTypeDef CAN_Config_Filter(CAN_HandleTypeDef *_hcan, uint32_t filter_bank)
{
	CAN_FilterTypeDef can_filter_st = {0};

	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_st.FilterIdHigh = 0x0000;
	can_filter_st.FilterIdLow = 0x0000;
	can_filter_st.FilterMaskIdHigh = 0x0000;
	can_filter_st.FilterMaskIdLow = 0x0000;
	can_filter_st.FilterBank = filter_bank;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	can_filter_st.SlaveStartFilterBank = 14;

	return HAL_CAN_ConfigFilter(_hcan, &can_filter_st);
}

/**
 * @brief CAN收发错误信息收集
 *
 * @param _hcan CAN编号
 * @return CAN_ErrorInfo_t* 错误信息指针
 */
static CAN_ErrorInfo_t *CAN_Get_ErrorInfo(CAN_HandleTypeDef *_hcan)
{
	if (_hcan == &hcan1)
		return &CAN1_ErrorInfo;
	if (_hcan == &hcan2)
		return &CAN2_ErrorInfo;
	return NULL;
}

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 发送标准数据帧
 *
 * @param _hcan CAN编号
 * @param std_id 标准帧ID
 * @param data 被发送的数据指针
 * @param len 数据长度
 * @return HAL_StatusTypeDef 执行状态
 */
HAL_StatusTypeDef CAN_Send_Data(CAN_HandleTypeDef *_hcan, uint16_t std_id, uint8_t *data, uint8_t len)
{
	CAN_TxHeaderTypeDef tx_header = {0};
	uint32_t send_mail_box = 0;
	uint32_t count = 0;
	HAL_StatusTypeDef ret;
	CAN_ErrorInfo_t *err = CAN_Get_ErrorInfo(_hcan);

	if (_hcan == NULL || data == NULL || len > 8U)
		return HAL_ERROR;

	while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
	{
		if (++count > CAN_TX_WAIT_MAX)
		{
			if (err != NULL)
				err->tx_timeout++;
			return HAL_TIMEOUT;
		}
	}

	count = 0;
	while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0U)
	{
		if (++count > CAN_TX_WAIT_MAX)
		{
			if (err != NULL)
			{
				err->tx_timeout++;
				err->last_error = HAL_CAN_GetError(_hcan);
			}

			HAL_CAN_AbortTxRequest(_hcan,
								   CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
			return HAL_TIMEOUT;
		}
	}

	tx_header.StdId = std_id;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = len;

	ret = HAL_CAN_AddTxMessage(_hcan, &tx_header, data, &send_mail_box);
	if (ret != HAL_OK && err != NULL)
	{
		err->tx_error++;
		err->last_error = HAL_CAN_GetError(_hcan);
	}

	return ret;
}

/**
 * @brief 初始化CAN总线
 *
 */
void CAN_Device_Init(void)
{
	if (CAN_Config_Filter(&hcan1, 0) != HAL_OK)
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

	if (CAN_Config_Filter(&hcan2, 14) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_CAN_Start(&hcan2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief HAL库CAN接收FIFO0中断
 *
 * @param hcan CAN编号
 */
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
		case 0x155:
			tempBuff[0] = rx_data[0];
			tempBuff[1] = rx_data[1];
			tempBuff[2] = rx_data[2];
			tempBuff[3] = rx_data[3];
			tempBuff[4] = rx_data[4];
			tempBuff[5] = rx_data[5];
			tempBuff[6] = rx_data[6];
			tempBuff[7] = rx_data[7];
			tgtStatus = tempBuff[0];
			decision_info.allow_to_lock = tempBuff[1];
			set_posture_mode = tempBuff[6];
			debug_posture = tempBuff[7];
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
			decision_info.tgtStatus = tempBuff[8];
			decision_info.reached_target = tempBuff[9];
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

/**
 * @brief 发送DT7遥控器数据
 *
 * @param _hcan CAN编号
 * @param rc_data 遥控器数据指针
 */
void Send_RC_Data(CAN_HandleTypeDef *_hcan, uint8_t *rc_data)
{
	uint8_t can_send_data[8];

	if (rc_data == NULL)
		return;

	for (uint8_t i = 0; i < 8U; i++)
		can_send_data[i] = rc_data[i];
	CAN_Send_Data(_hcan, CAN_RC_DATA_Frame_0, can_send_data, 8);

	for (uint8_t i = 0; i < 8U; i++)
		can_send_data[i] = rc_data[i + 8U];
	CAN_Send_Data(_hcan, CAN_RC_DATA_Frame_1, can_send_data, 8);
}

/**
 * @brief 图传链路数据发送
 *
 * @param _hcan CAN编号
 * @param vtm_data 图传数据指针
 */
void Send_VTM_Data(CAN_HandleTypeDef *_hcan, uint8_t *vtm_data)
{
	uint8_t can_send_data[8] = {0};

	if (vtm_data == NULL)
		return;

	for (uint8_t i = 0; i < 8U; i++)
		can_send_data[i] = vtm_data[i];
	CAN_Send_Data(_hcan, CAN_VTM_DATA_Frame_0, can_send_data, 8);

	can_send_data[0] = vtm_data[8];
	can_send_data[1] = vtm_data[9];
	can_send_data[2] = vtm_data[10];
	can_send_data[3] = vtm_data[11];
	can_send_data[4] = 0;
	can_send_data[5] = 0;
	can_send_data[6] = 0;
	can_send_data[7] = 0;
	CAN_Send_Data(_hcan, CAN_VTM_DATA_Frame_1, can_send_data, 8);
}

/**
 * @brief 发送功率控制数据
 *
 * @param _hcan CAN编号
 * @param Chassis_power_limit 底盘功率上限
 * @param Chassis_power_buffer 底盘缓冲能量
 */
void Send_Power_Data(CAN_HandleTypeDef *_hcan, uint16_t Chassis_power_limit, uint16_t Chassis_power_buffer)
{
	if (Chassis_power_limit >= 10240)
		Chassis_power_limit /= 256;
	if (Chassis_power_limit >= 200)
		Chassis_power_limit /= 5;
	uint8_t can_send_data[8] = {0};

	can_send_data[0] = Chassis_power_limit >> 8;
	can_send_data[1] = Chassis_power_limit;
	can_send_data[2] = Chassis_power_buffer >> 8;
	can_send_data[3] = Chassis_power_buffer;

	CAN_Send_Data(_hcan, CAN_POWER_INFO_ID, can_send_data, 8);
}

/**
 * @brief 发送机器人状态信息
 *
 * @param _hcan CAN编号
 * @param ID 机器人ID
 * @param heatLimit 热量上限
 * @param heat 当前热量
 * @param bulletSpeed 弹速
 * @param cooling_value 冷却值
 * @param UWBPosX UWB X坐标
 * @param IsEnemyInvincible 敌方无敌状态
 * @param UWBPosY UWB Y坐标
 * @param gameStatus 比赛状态
 * @param Inposflag 到点标志(已改为由导航下发，待删除)
 */
void Send_Robot_Info(CAN_HandleTypeDef *_hcan, uint8_t ID, uint16_t heatLimit, uint16_t heat, uint16_t bulletSpeed, uint16_t cooling_value,
					 uint16_t UWBPosX, uint8_t IsEnemyInvincible, uint16_t UWBPosY, uint8_t gameStatus, uint8_t Inposflag)
{
	uint8_t can_send_data[8] = {0};

	can_send_data[0] = ID;
	can_send_data[1] = (heatLimit >> 8) & 0xFF;
	can_send_data[2] = heatLimit & 0xFF;
	can_send_data[3] = (heat >> 8) & 0xFF;
	can_send_data[4] = heat & 0xFF;
	can_send_data[5] = (bulletSpeed >> 8) & 0xFF;
	can_send_data[6] = bulletSpeed & 0xFF;
	can_send_data[7] = (uint8_t)cooling_value;
	CAN_Send_Data(_hcan, 0x133, can_send_data, 8);

	if (Chassis.Mode == Follow_Mode)
		IsEnemyInvincible = 0;

	can_send_data[0] = 0;
	can_send_data[1] = (UWBPosX >> 8) & 0xFF;
	can_send_data[2] = UWBPosX & 0xFF;
	can_send_data[3] = IsEnemyInvincible;
	can_send_data[4] = (UWBPosY >> 8) & 0xFF;
	can_send_data[5] = UWBPosY & 0xFF;
	can_send_data[6] = Inposflag;
	can_send_data[7] = gameStatus;
	CAN_Send_Data(_hcan, 0x134, can_send_data, 8);
}

/**
 * @brief 发送决策信息
 *
 * @param _hcan CAN编号
 */
void Send_Decision_Info(CAN_HandleTypeDef *_hcan)
{
	static uint16_t cruise_begin = 0, cruise_end = 0, cruise_speed = 0, move_ratio = 0;
	uint8_t can_send_data[8] = {0};

	cruise_begin = float_to_uint16(decision_info.cruise_begin, -180.0f, 180.0f, 16);
	cruise_end = float_to_uint16(decision_info.cruise_end, -180.0f, 180.0f, 16);
	cruise_speed = float_to_uint16(decision_info.cruise_speed, 0.0f, 1.0f, 16);
	move_ratio = float_to_uint16(decision_info.move_speed_ratio, 0.0f, 1.0f, 16);

	can_send_data[0] = (cruise_begin & 0xFF00) >> 8;
	can_send_data[1] = cruise_begin & 0x00FF;
	can_send_data[2] = (cruise_end & 0xFF00) >> 8;
	can_send_data[3] = cruise_end & 0x00FF;
	can_send_data[4] = (move_ratio & 0xFF00) >> 8;
	can_send_data[5] = move_ratio & 0x00FF;
	can_send_data[6] = (cruise_speed & 0xFF00) >> 8;
	can_send_data[7] = cruise_speed & 0x00FF;
	CAN_Send_Data(_hcan, 0x135, can_send_data, 8);

	static uint16_t stage_remain_time, robot_HP;
	stage_remain_time = (uint16_t)(game_status.stage_remain_time);
	robot_HP = (uint16_t)(robot_state.current_HP);

	can_send_data[0] = (stage_remain_time & 0xFF00) >> 8;
	can_send_data[1] = stage_remain_time & 0x00FF;
	can_send_data[2] = (robot_HP & 0xFF00) >> 8;
	can_send_data[3] = robot_HP & 0x00FF;
	can_send_data[4] = decision_info.sentryPosture;
	can_send_data[5] = decision_info.sentryPresentPose;
	can_send_data[6] = decision_info.attack_outpost;
	can_send_data[7] = 0;
	CAN_Send_Data(_hcan, 0x136, can_send_data, 8);
}

/**
 * @brief 发送导航所需数据
 *
 * @param _hcan CAN编号
 */
void Send_Nav_Data_RMUC(CAN_HandleTypeDef *_hcan)
{
	static uint16_t TargetX = 0, TargetY = 0;
	uint8_t can_send_data[8] = {0};

	TargetX = (uint16_t)(map_command.target_position_x * 1000);
	TargetY = (uint16_t)(map_command.target_position_y * 1000);

	can_send_data[0] = Chassis.status;
	can_send_data[1] = 0;
	can_send_data[2] = (TargetX & 0xFF00) >> 8;
	can_send_data[3] = (TargetX & 0x00FF);
	can_send_data[4] = (TargetY & 0xFF00) >> 8;
	can_send_data[5] = (TargetY & 0x00FF);
	can_send_data[6] = decision_info.isEnemyPostAlive;
	can_send_data[7] = 77;

	CAN_Send_Data(_hcan, 0x503, can_send_data, 8);
}
