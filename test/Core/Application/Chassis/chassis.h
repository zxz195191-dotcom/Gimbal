#pragma once
#include "headfile.h"

extern void chassis_Init(void);
extern void Chassis_Kinematics_Resolve(float vx, float vy, float vw, int16_t target[Motor_Count]);
extern int16_t Clamp_Target_Rpm(int value);