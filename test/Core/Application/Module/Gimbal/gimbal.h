
#pragma once
#include "headfile.h"
#include "pitch_continuous_cal.h"
#include "yaw_inertial.h"
#include "pitch_inertial.h"

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
  YAW_FAULT_TX,
  YAW_FAULT_IMU
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

typedef enum {
  PITCH_SCAN_IDLE = 0,
  PITCH_SCAN_POSITION_START,
  PITCH_SCAN_TURN_UP_5,
  PITCH_SCAN_UP_5,
  PITCH_SCAN_TURN_DOWN_5,
  PITCH_SCAN_DOWN_5,
  PITCH_SCAN_TURN_UP_20,
  PITCH_SCAN_UP_20,
  PITCH_SCAN_TURN_DOWN_20,
  PITCH_SCAN_DOWN_20,
  PITCH_SCAN_COMPLETE_HOLD,
  PITCH_SCAN_CANCEL_HOLD,
  PITCH_SCAN_ABORT_HOLD
} Pitch_Scan_Phase_e;

typedef enum {
  PITCH_SCAN_RESULT_NONE = 0,
  PITCH_SCAN_RESULT_COMPLETE,
  PITCH_SCAN_RESULT_CANCELLED,
  PITCH_SCAN_RESULT_REJECT_AXIS,
  PITCH_SCAN_RESULT_REJECT_FEEDBACK,
  PITCH_SCAN_RESULT_REJECT_BOUNDS,
  PITCH_SCAN_RESULT_STALL,
  PITCH_SCAN_RESULT_TORQUE,
  PITCH_SCAN_RESULT_OVERSPEED,
  PITCH_SCAN_RESULT_OUT_OF_BOUNDS,
  PITCH_SCAN_RESULT_AXIS_FAULT,
  PITCH_SCAN_RESULT_CAL_DATA
} Pitch_Scan_Result_e;

typedef struct {
  Pitch_Scan_Phase_e phase;
  Pitch_Scan_Result_e result;
  uint8_t active;
  uint8_t feedback_valid;
  uint8_t holding;
  uint32_t event_count;
  float saved_kp;
  float saved_kd;
  float min_deg;
  float max_deg;
  char start_summary[160];
  char event_summary[160];
  char end_summary[160];
} Pitch_Scan_Status_t;

/* 只按实测端点换向，不增加扫描专用保护或停止条件。 */
#define PITCH_SCAN_MIN_ACTUAL_DEG         PITCH_CONT_MIN_DEG
#define PITCH_SCAN_MAX_ACTUAL_DEG         PITCH_CONT_MAX_DEG

extern volatile Yaw_Test_t g_yaw_test;
extern volatile Yaw_Test_t g_pitch_test;
extern volatile Pitch_Scan_Status_t g_pitch_scan;

typedef struct {
  float min_deg, max_deg, speed_deg_s;
  int8_t direction;
  uint8_t active, positioning;
} Gimbal_Repeat_t;
extern volatile Gimbal_Repeat_t g_gimbal_repeat[DM_NUM];
extern volatile uint8_t g_gimbal_log_axis; /* 0=Yaw，1=Pitch */
uint8_t Gimbal_Pitch_Repeat_Start(float min_deg, float max_deg, float speed_deg_s);
uint8_t Gimbal_Yaw_Repeat_Start(float min_deg, float max_deg, float speed_deg_s);
void Gimbal_Yaw_Repeat_Cancel(void);

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
uint8_t Gimbal_Yaw_Set_Mode(Yaw_Control_Mode_t mode);
float Gimbal_Yaw_Control_Angle_Rad(void);
float Gimbal_Yaw_Control_Rate_Rad_S(void);
void Gimbal_Pitch_Stop(void);
void Gimbal_Pitch_Enable(void);
uint8_t Gimbal_Pitch_Set_Mode(Pitch_Control_Mode_t mode);
void Gimbal_Pitch_Probe(void);
void Gimbal_Enable_All(void);
void Gimbal_Stop_All(void);
uint8_t Gimbal_Pitch_Scan_Start(void);
void Gimbal_Pitch_Scan_Cancel(void);

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
/* 0 disables pitch feedforward; 0..1 scales the generated calibration table. */
uint8_t Gimbal_Pitch_Set_Gravity_FF_Scale(float scale);

uint8_t Gimbal_Yaw_Cal_Arm(void);
uint8_t Gimbal_Yaw_Cal_Start(void);
void Gimbal_Yaw_Cal_Stop(void);
uint8_t Gimbal_Yaw_Friction_Enable(uint8_t enable);
uint8_t Gimbal_Yaw_Position_Boost_Enable(uint8_t enable);
uint8_t Gimbal_Load_Integral_Enable(uint8_t axis, uint8_t enable);
void Gimbal_Load_Report(void);
void Gimbal_Load_Command_Result(const char *name, uint8_t ok);
uint8_t Gimbal_Load_Report_Line(char *buf, unsigned capacity);
void Gimbal_Load_Report_Advance(void);
uint8_t Gimbal_Load_Telemetry_Paused(void);
