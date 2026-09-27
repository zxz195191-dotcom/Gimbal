//
// Created by zzx on 2026/8/14.
//
#pragma once
#include "headfile.h"

typedef struct {
  float p,i,d;
  float intergral;
  float last_error;
  float max_intergral;
  float max_out;
}PID_TypeDef;


extern void PID_Init(PID_TypeDef *pid,float _p,float _i,float _d,float _max_out,float _max_intergral);
void PID_Reset(PID_TypeDef *pid);
float PID_Calculate(PID_TypeDef *pid, float actual_rpm, float target_rpm);
