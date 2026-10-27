#pragma once

#include <stdint.h>

void Pitch_Cal_Start(void);
void Pitch_Cal_Cancel(void);
uint8_t Pitch_Cal_Is_Active(void);
void Pitch_Cal_Update(uint32_t now_ms);

void Pitch_FF_Reset(void);
float Pitch_FF_Update(float measured_deg,int8_t approach_sign,uint32_t elapsed_ms);
uint8_t Pitch_FF_Set_Scale(float scale);
float Pitch_FF_Get_Scale(void);


typedef struct {
  float measured_deg;
  float scale;//前馈力矩缩放系数

  int8_t approach_sign;
  uint8_t lookup_valid;

  uint8_t lower_index;//查表：下限点
  uint8_t upper_index;

  float lower_angle_deg;//区间下限角度
  float upper_angle_deg;

  float lower_torque_nm;//下限点对应的前馈力矩 (N・m)
  float upper_torque_nm;

  float interpolated_torque_nm;//线性插值算出原始力矩
  float scaled_torque_nm;//缩放后的力矩
  float applied_torque_nm;//scale、限幅、缓升降之后实际输出的前馈力矩
}Pitch_FF_Debug_t;

extern volatile Pitch_FF_Debug_t g_pitch_ff_debug;