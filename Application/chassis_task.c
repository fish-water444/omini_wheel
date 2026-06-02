#include "chassis_task.h"
#include "remote_control.h"

Chassis_t Chassis = {0};
static uint32_t dwt_cnt = 0;
static float dt = 0;

void Chassis_Init()
{
    Chassis.WheelRadius=WHEEL_RADIUS/1000.0f;
    Chassis.VelocityRatio=VELOCITY_RATIO;
    Chassis.Mode=Follow_Mode;

    for (uint8_t i=0;i<3;i++)
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
    Chassis.vr=remote_control.ch1 * 0.1f;

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
        Chassis+=temp*dt;
    }
    else 
    {
        Chassis+=((target_Vx-Chassis.Vx)/(dt + rc))*dt;
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
        Chassis+=temp*dt;
    }
    else 
    {
        Chassis+=((target_Vy-Chassis.Vy)/(dt + rc))*dt;
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

    float Max_v=Max(Chassis.V1,Chassis.V2,Chassis.V3,Chassis.V4);
    float ratio=Chassis.V1/MOTOR_MAX_RPM;

    Chassis.V1/=ratio;
    Chassis.V2/=ratio;
    Chassis.V3/=ratio;
    Chassis.V4/=ratio;

}

void Chassis_Kinematics_Inverse()//?
{
        float vf=WHEEL_RADIUS/(WHEEL_RADIUS*SQRT2);
        float rf=WHEEL_OPPOSITE/WHEEL_RADIUS;

        Chassis.V1=(-Chassis.Vxtransfer-Chassis.Vytransfer)*vf+Chassis.Vr*rf;//FR
        Chassis.V2=(Chassis.Vxtransfer-Chassis.Vytransfer)*vf+Chassis.Vr*rf;//FL
        Chassis.V3=(Chassis.Vxtransfer+Chassis.Vytransfer)*vf+Chassis.Vr*rf;//HL
        Chassis.V1=(-Chassis.Vxtransfer+Chassis.Vytransfer)*vf+Chassis.Vr*rf;//HR
}

float Max(float a,float b,float c,float d)
{
    float max[4]={a,b,c,d};
    float max_v=max[0];
    for (int i=1;i<4;i++)
    {
        if (max[i]>max_v)
            max_v=max[i];
    }
    return max_v;
}

