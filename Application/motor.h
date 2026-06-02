#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

typedef struct {
    uint16_t angle;
    int16_t  speed_rpm;
    int16_t  torque_current;
    uint8_t  temperature;
} Motor_t;

extern Motor_t motor_fb[8];

void CAN_User_Init(void);

void C620_SendCurrents(CAN_HandleTypeDef *hcan, int16_t currents[4]);

#endif
