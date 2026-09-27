//
// Created by zzx on 2026/8/14.
//
#include "headfile.h"



void PID_Init(PID_TypeDef *pid,float _p,float _i,float _d,float _max_out,float _max_intergral) {
  pid->p = _p;
  pid->i = _i;
  pid->d = _d;
  pid->max_out = _max_out;
  pid->max_intergral = _max_intergral;

  pid->intergral = 0.0f;
  pid->last_error = 0.0f;
}


void PID_Reset(PID_TypeDef *pid) {
  pid->intergral = 0.0f;
  pid->last_error = 0.0f;
}

static float value_restrict(float value,float max) {
  if (value > max) value = max;
  if (value < -max) value = -max;
  return value;
}

float PID_Calculate(PID_TypeDef *pid, float actual_rpm, float target_rpm) {
  float error = target_rpm - actual_rpm;

  float Pout = pid->p * error;
  pid->intergral += error;
  pid->intergral = value_restrict(pid->intergral,pid->max_intergral);
  float Iout = pid->i * pid->intergral;

  float Dout = pid->d * (error - pid->last_error);

  float out = Pout + Iout + Dout;

  pid->last_error = error;

  out = value_restrict(out,pid->max_out);

  return out;
}
