/**
 ******************************************************************************
 * @file	 bsp_CAN.h
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2020/2/4
 * @brief CAN通信初始化、接收解析与业务协议发送
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _BSP_CAN_H
#define _BSP_CAN_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include "can.h"

/* Exported macros -----------------------------------------------------------*/

#define CAN_RC_DATA_Frame_0 0x131
#define CAN_RC_DATA_Frame_1 0x132

#define CAN_POWER_INFO_ID 0x143
#define CAN_CAP_INFO_ID 0x151
#define CAN_CAP0727_INFO_ID 0x150

#define CAN_VTM_DATA_Frame_0 0x430
#define CAN_VTM_DATA_Frame_1 0x432

#define CAN_SYSTEM_RESET_CMD 0xAAA

#define CAN_PowerContol_ID 0x301

#define CAN_AERIAL_DATA_1 0x501
#define CAN_AERIAL_DATA_2 0x502

/* Exported types ------------------------------------------------------------*/

typedef struct
{
	volatile uint32_t tx_timeout;
	volatile uint32_t tx_error;
	volatile uint32_t bus_off;
	volatile uint32_t last_error;
} CAN_ErrorInfo_t;

/* Exported variables ---------------------------------------------------------*/

extern CAN_ErrorInfo_t CAN1_ErrorInfo;
extern CAN_ErrorInfo_t CAN2_ErrorInfo;

extern uint8_t enable_navigation;
extern float dt_cap0727;
extern uint8_t set_posture_mode, debug_posture;
extern uint8_t tgtStatus;

/* Exported function declarations ---------------------------------------------*/

// void CAN_Device_Init(CAN_HandleTypeDef *_hcan);

/**
 * @brief 初始化CAN总线
 *
 */
void CAN_Device_Init(void);

/**
 * @brief 发送标准数据帧
 *
 * @param _hcan CAN编号
 * @param std_id 标准帧ID
 * @param data 被发送的数据指针
 * @param len 数据长度
 * @return HAL_StatusTypeDef 执行状态
 */
HAL_StatusTypeDef CAN_Send_Data(CAN_HandleTypeDef *_hcan, uint16_t std_id, uint8_t *data, uint8_t len);

/**
 * @brief 发送遥控器数据
 *
 * @param _hcan CAN编号
 * @param rc_data 遥控器数据指针
 */
void Send_RC_Data(CAN_HandleTypeDef *_hcan, uint8_t *rc_data);

/**
 * @brief 图传链路数据发送
 *
 * @param _hcan CAN编号
 * @param vtm_data 图传数据指针
 */
void Send_VTM_Data(CAN_HandleTypeDef *_hcan, uint8_t *vtm_data);

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
					 uint16_t UWBPosX, uint8_t IsEnemyInvincible, uint16_t UWBPosY, uint8_t gameStatus, uint8_t Inposflag);

/**
 * @brief 发送决策信息
 *
 * @param _hcan CAN编号
 */
void Send_Decision_Info(CAN_HandleTypeDef *_hcan);

/**
 * @brief 发送导航所需数据
 *
 * @param _hcan CAN编号
 */
void Send_Nav_Data_RMUC(CAN_HandleTypeDef *_hcan);

/**
 * @brief 发送功率控制数据
 *
 * @param _hcan CAN编号
 * @param Chassis_power_limit 底盘功率上限
 * @param Chassis_power_buffer 底盘缓冲能量
 */
void Send_Power_Data(CAN_HandleTypeDef *_hcan, uint16_t Chassis_power_limit, uint16_t Chassis_power_buffer);

#endif
