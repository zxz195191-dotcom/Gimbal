#pragma once
#include "headfile.h"

typedef struct {
  uint16_t angle;
  int16_t speed_rmp;
  int16_t feedback_current;
  uint8_t temperature;
  uint8_t raw_data[8];
  uint32_t last_rx_tick;
  uint32_t rx_count;
}MotorFeedBack_t;

typedef enum {
  Friction_Left = 0,  // ID1, 0x201
  Friction_Right,     // ID2, 0x202
  Feeder,             // ID3, 0x203
  DJI_Motor_Reserved, // ID4, 0x204, always commanded to zero
  Motor_Count
}MotorID_e;

extern PID_TypeDef pid_spd[Motor_Count];
extern volatile MotorFeedBack_t motor_feedback[Motor_Count];
extern float target_speed_rpm[Motor_Count];
extern int16_t current_cmd[Motor_Count];

extern HAL_StatusTypeDef CAN_Send_Motor_Current(int16_t current1, int16_t current2, int16_t current3, int16_t current4);
extern void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
extern uint8_t Motor_Feedback_Valid(MotorID_e id, uint32_t now_us);
extern uint32_t DWT_Read();
