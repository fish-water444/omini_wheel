/**
 ******************************************************************************
 * @file	 bsp_CAN.h
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2020/2/4
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _BSP_CAN_H
#define _BSP_CAN_H

#include "includes.h"
#include "power_manager.h"

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

extern uint8_t enable_navigation;
extern float dt_cap0727;

// void CAN_Device_Init(CAN_HandleTypeDef *_hcan);
void CAN_Device_Init(void);

void Send_RC_Data(CAN_HandleTypeDef *_hcan, uint8_t *rc_data);
void Send_VTM_Data(CAN_HandleTypeDef *_hcan, uint8_t *vtm_data);
void Send_Reset_Command(CAN_HandleTypeDef *_hcan);
void Send_Robot_Info(CAN_HandleTypeDef *_hcan, uint8_t ID, uint16_t heatLimit, uint16_t heat1, uint16_t bulletSpeed, uint16_t cooling_value,
					 uint16_t UWBPosX, uint8_t IsEnemyInvincible, uint16_t UWBPosY, uint8_t gameStatus, uint8_t Inposflag);
void Send_JudgeRxData(CAN_HandleTypeDef *_hcan, uint8_t *data);
void SendAerialData(CAN_HandleTypeDef *_hcan, float *X, float *Y, uint8_t *KeyBoard);
void Send_Nav_Data_RMUC(CAN_HandleTypeDef *_hcan);
void Send_Power_Data(CAN_HandleTypeDef *_hcan, uint16_t Chassis_power_limit, uint16_t Chassis_power_buffer);
void float2u8array(float *FloatData, uint8_t *u8Array, bool Key); // 浮点数转u8数组，Key为高低位变换

#endif
