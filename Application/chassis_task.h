#ifndef _CHASSIS_TASK_H
#define _CHASSIS_TASK_H

#include "motor.h"

#define SQRT2 1.4142135f// √2的精确值
#define WHEEL_OPPOSITE 0.2675f//轮子到中间的距离
#define WHEEL_RADIUS 76.00f//轮子半径mm
#define VELOCITY_RATIO 3//速度缩放比

#define CHASSIS_VX_MAX 2.64f//前后速度限幅
#define CHASSIS_VY_MAX 2.64f//左右
#define CHASSIS_VR_MAX 2.64f//角速度
#define MOTOR_MAX_RPM  8000//电机

enum
{
    Follow_Mode   = 0 ,//
    Spinning_Mode = 1 ,//自传
    Silence_Mode  = 2 ,//停转
};

enum
{
    
  FR = 0, // 前右
  FL = 1, // 前左
  HL = 2,// 后左
  HR = 3,// 后右
};

typedef struct _Chassis_t
{
    float Vx;
    float Vy;
    float Vr;

    float Vxtransfer,Vytransfer;

    float V1;
    float V2;
    float V3;
    float V4;

    uint8_t Mode;
    float VelocityRatio;       
    float WheelRadius;         
    float WheelReductionRatio;

    Motor_t ChassisMotor[4]; 

}chassis_t;

extern chassis_t Chassis;


void Chassis_Init(void);
void Chassis_Control(void);
void Chassis_Kinematics_Inverse(void);
void Send_Chassis_Current(void);
float Max(float a,float b,float c,float d);

#endif