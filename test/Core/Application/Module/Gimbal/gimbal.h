
#pragma once
#include "headfile.h"

typedef enum {
  YAW_TEST_DISABLED = 0,
  YAW_TEST_CLEARING,
  YAW_TEST_WAIT_FEEDBACK,
  YAW_TEST_VERIFY_FEEDBACK,
  YAW_TEST_ACTIVE,
  YAW_TEST_FAULT
} Yaw_Test_State_e;

typedef enum {
  YAW_FAULT_NONE = 0,
  YAW_FAULT_CAN_INIT,
  YAW_FAULT_ENABLE_TIMEOUT,
  YAW_FAULT_FEEDBACK_TIMEOUT,
  YAW_FAULT_MOTOR_ERROR,
  YAW_FAULT_OVER_TEMPERATURE,
  YAW_FAULT_TX
} Yaw_Fault_e;

typedef struct {
  Yaw_Test_State_e state;
  Yaw_Fault_e fault;
  float origin_rad;
  /* Continuous position in the coordinate frame whose zero is captured at boot. */
  float measured_relative_rad;
  float measured_velocity_rad_s;
  /* Absolute setpoint in that same boot-zero-relative coordinate frame. */
  float target_relative_rad;
  float command_relative_rad;
  float max_speed_rad_s;
  float kp;
  float kd;
  /* Actual motor-coordinate feedforward torque sent in the latest MIT frame. */
  float applied_t_ff_nm;
  uint32_t tx_count;
  uint32_t key_step;
} Yaw_Test_t;

extern volatile Yaw_Test_t g_yaw_test;
extern volatile Yaw_Test_t g_pitch_test;

#ifndef GIMBAL_YAW_DIRECTION
#define GIMBAL_YAW_DIRECTION    1.0f
#endif
#ifndef GIMBAL_PITCH_DIRECTION
#define GIMBAL_PITCH_DIRECTION  1.0f
#endif

void Gimbal_Yaw_Test_Init(void);
void Gimbal_Yaw_Test_Update(uint32_t now_ms, GPIO_PinState key_level);
void Gimbal_Yaw_Test_Stop(void);
void Gimbal_Yaw_Enable(void);
void Gimbal_Pitch_Stop(void);
void Gimbal_Pitch_Enable(void);
void Gimbal_Enable_All(void);
void Gimbal_Stop_All(void);

void Pitch_Cal_Update(uint32_t now_ms);
void Pitch_Cal_Start(void);

uint8_t Gimbal_Yaw_Set_Target_Deg(float target_deg);
uint8_t Gimbal_Yaw_Set_Target_Rad(float target_rad);
uint8_t Gimbal_Yaw_Set_Target_Speed_Deg(float target_deg,float max_speed_deg_s);
uint8_t Gimbal_Yaw_Move_Relative_Deg(float delta_deg);
uint8_t Gimbal_Yaw_Move_Relative_Speed_Deg(float delta_deg,float max_speed_deg_s);
uint8_t Gimbal_Yaw_Set_Max_Speed_Deg(float max_speed_deg_s);
uint8_t Gimbal_Yaw_Set_Kp(float kp);
uint8_t Gimbal_Yaw_Set_Kd(float kd);
uint8_t Gimbal_Pitch_Set_Target_Deg(float target_deg);
uint8_t Gimbal_Pitch_Set_Target_Rad(float target_rad);
uint8_t Gimbal_Pitch_Set_Target_Speed_Deg(float target_deg,float max_speed_deg_s);
uint8_t Gimbal_Pitch_Move_Relative_Deg(float delta_deg);
uint8_t Gimbal_Pitch_Move_Relative_Speed_Deg(float delta_deg,float max_speed_deg_s);
uint8_t Gimbal_Pitch_Set_Max_Speed_Deg(float max_speed_deg_s);
uint8_t Gimbal_Pitch_Set_Kp(float kp);
uint8_t Gimbal_Pitch_Set_Kd(float kd);
/* 0 disables gravity feedforward; 0..1 scales the calibrated 10..35 deg table. */
uint8_t Gimbal_Pitch_Set_Gravity_FF_Scale(float scale);
