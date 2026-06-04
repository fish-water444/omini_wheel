#ifndef MOTOR_H
#define MOTOR_H

#include "controller.h"
#include "can.h"

#define ENCODERCOEF 0.000095874//2pi/65536编码器值→弧度 
#define TORQUE_CONSTANT 0.3f//扭矩系数 (N·m/A) 
#define REDUCTION_RATIO 19.203f//减速比  3591/187

#define CAN_Transmit_1_4_ID  0x200 
#define CAN_Receive_1_ID  0x201  
#define CAN_Receive_2_ID  0x202   
#define CAN_Receive_3_ID  0x203   
#define CAN_Receive_4_ID  0x204 

#define NEGATIVE  1 

typedef struct motor_t
{
    uint16_t RawAngle;       // 原始编码器 (0~65535)
    uint16_t last_angle;     // 上次编码器值
    int32_t  round_cnt;      // 圈数
    int32_t  total_angle;    // 累计角度
    uint32_t offset_angle;   // 上电初始偏移
    uint16_t zero_offset;    // 零点偏移

    float Angle;             // 圈内角度
    float AngleInDegree;     // 角度(度)

    float Velocity_RPM;      // 转速(RPM, 滤波后)
    float Velocity_Rad;      // 角速度(rad/s)
    float OutputVel_RadPS;   // 输出端转速(rad/s)

    float Current;           // 电流(A)
    float Torque;            // 扭矩(N·m)
    uint8_t Temperature;     // 温度(°C)

    
    float ReductionRatio;    // 减速比
    uint8_t Direction;       // 0=正向, NEGATIVE=反向
    float last_Velocity_RPM;
    uint8_t firstRun;
    int32_t msg_cnt;

    float Output;            // 电流输出值
    float Max_Out;           // 输出限幅

    PID_t PID_Velocity;      // 速度环PID

} Motor_t;

float Motor_Speed_Calculate(Motor_t *motor, float velocity, float target_speed);
void get_moto_info(Motor_t *ptr, uint8_t *aData);
void get_moto_offset(Motor_t *ptr, uint8_t *aData);
HAL_StatusTypeDef Send_Motor_Current_1_4(CAN_HandleTypeDef *_hcan,
                                          int16_t c1, int16_t c2,
                                          int16_t c3, int16_t c4);

#endif
