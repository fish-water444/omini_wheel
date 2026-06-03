#include "chassis_task.h"
#include "remote_control.h"
#include "includes.h"

Chassis_t Chassis = {0};
static uint32_t dwt_cnt = 0;
static float dt = 0;

void Chassis_Init()
{
    Chassis.WheelRadius=WHEEL_RADIUS;
    Chassis.VelocityRatio=VELOCITY_RATIO;
    Chassis.Mode=Follow_Mode;

    for (uint8_t i=0;i<4;i++)
    {
         PID_Init(&Chassis.ChassisMotor[i].PID_Velocity,
                 16384, 16384, 0,
                 50, 0.1, 0,
                 500, 100, 0.005, 0, 1,
                 Integral_Limit | OutputFilter | ChassisLpf);
        Chassis.ChassisMotor[i].Max_Out = 16384.0f;
    }
}

void Chassis_Control()
{
    dt = DWT_GetDeltaT(&dwt_cnt);

    float target_Vx=remote_control.ch3 * 0.004f;//0.004,0.1系数，经验调参
    float target_Vy=remote_control.ch4 * 0.004f;
    Chassis.Vr=remote_control.ch1 * 0.1f;

    float rc=0.01f;
    float temp;

    if(fabs(target_Vx)<1e-3f)
    {
        if (target_Vx * Chassis.Vx < 0)
        target_Vx=0;
        temp=(target_Vx-Chassis.Vx)/(dt + rc);
        if (temp>2000)
        temp=2000;
        if (temp<-2000)
        temp=-2000;
        Chassis.Vx+=temp*dt;
    }
    else 
    {
        Chassis.Vx+=((target_Vx-Chassis.Vx)/(dt + rc))*dt;
    }

    if(fabs(target_Vy)<1e-3f)
    {
        if (target_Vy * Chassis.Vy < 0)
        target_Vy=0;
        temp=(target_Vy-Chassis.Vy)/(dt + rc);
        if (temp>2000)
        temp=2000;
        if (temp<-2000)
        temp=-2000;
        Chassis.Vy+=temp*dt;
    }
    else 
    {
        Chassis.Vy+=((target_Vy-Chassis.Vy)/(dt + rc))*dt;
    }


    Chassis.Vx = float_constrain(Chassis.Vx, -CHASSIS_VX_MAX, CHASSIS_VX_MAX);
    Chassis.Vy = float_constrain(Chassis.Vy, -CHASSIS_VY_MAX, CHASSIS_VY_MAX);
    Chassis.Vr = float_constrain(Chassis.Vr, -CHASSIS_VR_MAX, CHASSIS_VR_MAX);

    switch(Chassis.Mode)
    {
        case Follow_Mode:
        Chassis.Vxtransfer=Chassis.Vx;
        Chassis.Vytransfer=Chassis.Vy;
        break;
        case Spinning_Mode:
        Chassis.Vxtransfer=0.0f;
        Chassis.Vytransfer=0.0f;
        break;
        case Silence_Mode:
        Chassis.Vxtransfer=0.0f;
        Chassis.Vytransfer=0.0f;
        Chassis.Vr=0.0f;
        break;
    }

    Chassis_Kinematics_Inverse();

    float Max_v=fabsf(Chassis.V1);
    if (fabsf(Chassis.V2)>Max_v)
        Max_v=fabsf(Chassis.V2);
    if (fabsf(Chassis.V3)>Max_v)
        Max_v=fabsf(Chassis.V3);
    if (fabsf(Chassis.V4)>Max_v)
        Max_v=fabs(Chassis.V4);

    float ratio=Max_v/MOTOR_MAX_RPM;
    if (ratio>1.0f)
        {
            Chassis.V1/=ratio;
            Chassis.V2/=ratio;
            Chassis.V3/=ratio;
            Chassis.V4/=ratio;
        }


    for (uint8_t i = 0; i < 4; i++)
        {
            float actual = Chassis.ChassisMotor[i].Velocity_RPM / Chassis.WheelReductionRatio;
            float target = 0;
            switch (i) {
            case FR: target = Chassis.V1; break;
            case FL: target = Chassis.V2; break;
            case HL: target = Chassis.V3; break;
            case HR: target = Chassis.V4; break;
            }
            Motor_Speed_Calculate(&Chassis.ChassisMotor[i], actual, target);

            if (!isnormal(Chassis.ChassisMotor[i].Output) && Chassis.ChassisMotor[i].Output != 0.0f)
                Chassis.ChassisMotor[i].Output = 0.0f;
        }

    Send_Chassis_Current();
}

void Chassis_Kinematics_Inverse()//?
{
        float vf=WHEEL_RADIUS/(WHEEL_RADIUS*SQRT2);
        float rf=WHEEL_OPPOSITE/WHEEL_RADIUS;

        Chassis.V1=(-Chassis.Vxtransfer-Chassis.Vytransfer)*vf+Chassis.Vr*rf;//FR
        Chassis.V2=(Chassis.Vxtransfer-Chassis.Vytransfer)*vf+Chassis.Vr*rf;//FL
        Chassis.V3=(Chassis.Vxtransfer+Chassis.Vytransfer)*vf+Chassis.Vr*rf;//HL
        Chassis.V4=(-Chassis.Vxtransfer+Chassis.Vytransfer)*vf+Chassis.Vr*rf;//HR
}

void Send_Chassis_Current()
{
        Send_Motor_Current_1_4(&hcan1,
        (int16_t)Chassis.ChassisMotor[FR].Output,
        (int16_t)Chassis.ChassisMotor[FL].Output,
        (int16_t)Chassis.ChassisMotor[HL].Output,
        (int16_t)Chassis.ChassisMotor[HR].Output);
}
