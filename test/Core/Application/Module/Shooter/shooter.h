#pragma once
#include "headfile.h"

/*
 * Viewed from the agreed motor output-shaft side, +1 means the left friction
 * motor uses positive feedback direction for forward firing. The right motor
 * is always assigned the opposite sign.
 */
#ifndef FRICTION_FORWARD_SIGN
#define FRICTION_FORWARD_SIGN  1
#endif
#ifndef FEEDER_FORWARD_SIGN
#define FEEDER_FORWARD_SIGN    1
#endif
#define FEEDER_MAX_RPM         12000.0f

_Static_assert((FRICTION_FORWARD_SIGN == 1) ||
               (FRICTION_FORWARD_SIGN == -1),
               "FRICTION_FORWARD_SIGN must be +1 or -1");
_Static_assert((FEEDER_FORWARD_SIGN == 1) ||
               (FEEDER_FORWARD_SIGN == -1),
               "FEEDER_FORWARD_SIGN must be +1 or -1");

extern volatile float g_friction_target_percent;
extern volatile float g_friction_target_rpm;
extern volatile float g_feeder_target_percent;
extern volatile float g_feeder_target_rpm;

void Shooter_Init(int16_t target[Motor_Count]);
uint8_t Shooter_Set_Friction_Percent(int16_t target[Motor_Count],
                                     float percent);
uint8_t Shooter_Set_Friction_Rpm(int16_t target[Motor_Count], float rpm);
uint8_t Shooter_Set_Feeder_Percent(int16_t target[Motor_Count],
                                   float percent);
uint8_t Shooter_Set_Feeder_Rpm(int16_t target[Motor_Count], float rpm);
uint8_t Shooter_Set_All(int16_t target[Motor_Count],
                        float friction_percent, float feeder_percent);
void Shooter_Stop(int16_t target[Motor_Count]);
