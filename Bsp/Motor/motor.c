#include "motor.h"

void get_moto_info(Motor_t *ptr, uint8_t *aData)
{
    const float lpf = 2 * PI * 10 * 0.001f / (1 + 2 * PI * 10 * 0.001f);

    if (ptr->Direction != NEGATIVE)
    {
        ptr->RawAngle     = (uint16_t)(aData[0] << 8 | aData[1]);
        ptr->Velocity_RPM = (int16_t)(aData[2] << 8 | aData[3]);
    }
    else
    {
        ptr->RawAngle= 8191 - (uint16_t)(aData[0] << 8 | aData[1]);//  转子机械角度
        ptr->Velocity_RPM = -(int16_t)(aData[2] << 8 | aData[3]);
    }

    if (ptr->firstRun == 0)
    {
        ptr->firstRun = 1;
        ptr->last_Velocity_RPM = (float)ptr->Velocity_RPM;
    }

    if (ptr->ReductionRatio > 1e-6f)
    ptr->OutputVel_RadPS = (ptr->Velocity_RPM * 2*PI/60.0f) / ptr->ReductionRatio;

    ptr->Current = (int16_t)(aData[4] << 8 | aData[5]) * 20.0f / 16384.0f;

    ptr->Torque = TORQUE_CONSTANT  / REDUCTION_RATIO * ptr->Current;

    ptr->Velocity_RPM = lpf * (float)ptr->Velocity_RPM + (1.0f - lpf) * ptr->last_Velocity_RPM;

    if (ptr->RawAngle - ptr->last_angle > 4096)
        ptr->round_cnt--;
    else if (ptr->RawAngle - ptr->last_angle < -4096)
        ptr->round_cnt++;  

    ptr->Angle = loop_float_constrain(ptr->RawAngle - ptr->zero_offset, -4095, 4096);//
    ptr->AngleInDegree = ptr->Angle * 0.0439507f;//360° / 8192
    ptr->total_angle = ptr->round_cnt * 8192 + ptr->RawAngle - ptr->offset_angle;
    ptr->last_angle = ptr->RawAngle;
    ptr->last_Velocity_RPM = (float)ptr->Velocity_RPM;
}

void get_moto_offset(Motor_t *ptr, uint8_t *aData)
{
    ptr->RawAngle=(uint16_t)(aData[0] << 8 | aData[1]);
    ptr->offset_angle = ptr->RawAngle;
    ptr->last_angle   = ptr->RawAngle;
}


float Motor_Speed_Calculate(Motor_t *motor, float velocity, float target_speed)
{
    PID_Calculate(&motor->PID_Velocity, velocity, target_speed);
    motor->Output = motor->PID_Velocity.Output;
    motor->Output = float_constrain(motor->Output, -motor->Max_Out, motor->Max_Out);

    return motor->Output;
}

HAL_StatusTypeDef Send_Motor_Current_1_4(CAN_HandleTypeDef *_hcan,
                                          int16_t c1, int16_t c2,
                                          int16_t c3, int16_t c4)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t data[8];
    uint32_t mailbox;
    uint32_t cnt;

    TX_MSG.StdId = CAN_Transmit_1_4_ID;
    TX_MSG.IDE   = CAN_ID_STD;
    TX_MSG.RTR   = CAN_RTR_DATA;
    TX_MSG.DLC   = 0x08;

    data[0]=(c1>>8)&0xff;
    data[1]=c1&0xff;
    data[2]=(c2>>8)&0xff;
    data[3]=c2&0xff;
    data[4]=(c3>>8)&0xff;
    data[5]=c3&0xff;
    data[6]=(c4>>8)&0xff;
    data[7]=c4&0xff;

    cnt = 0;
    while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
    {
        if (++cnt > 10000) 
        break;
    }
    cnt = 0;
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0)
    {
        if (++cnt > 10000) 
        break;
    }

    if((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U) 
        mailbox = CAN_TX_MAILBOX0;
    else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U) 
        mailbox = CAN_TX_MAILBOX1;
    else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U) 
        mailbox = CAN_TX_MAILBOX2;
    else 
        return HAL_ERROR;

    return HAL_CAN_AddTxMessage(_hcan, &TX_MSG, data, &mailbox);
    
}