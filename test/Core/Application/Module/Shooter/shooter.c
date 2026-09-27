#include "headfile.h"
#include <math.h>

volatile float g_friction_target_percent = 0.0f;
volatile float g_friction_target_rpm = 0.0f;
volatile float g_feeder_target_percent = 0.0f;
volatile float g_feeder_target_rpm = 0.0f;

static int16_t round_signed_rpm(float rpm) {
  if (rpm > 32767.0f) rpm = 32767.0f;
  if (rpm < -32768.0f) rpm = -32768.0f;
  return (int16_t)(rpm + ((rpm >= 0.0f) ? 0.5f : -0.5f));
}

void Shooter_Init(int16_t target[Motor_Count]) {
  for (uint8_t i = 0U; i < Motor_Count; i++) {
    PID_Init(&pid_spd[i], P, I, D, MAX_Out, MAX_Integral);
    target_speed_rpm[i] = 0.0f;
    current_cmd[i] = 0;
    if (target != NULL) target[i] = 0;
  }
  g_friction_target_percent = 0.0f;
  g_friction_target_rpm = 0.0f;
  g_feeder_target_percent = 0.0f;
  g_feeder_target_rpm = 0.0f;
}

uint8_t Shooter_Set_Friction_Rpm(int16_t target[Motor_Count], float rpm) {
  if ((target == NULL) || !isfinite(rpm) ||
      (rpm < 0.0f) || (rpm > FRICTION_MAX_RPM)) {
    return 0U;
  }

  const int16_t rpm_command = round_signed_rpm(rpm);
  target[Friction_Left] =
      (int16_t)(FRICTION_FORWARD_SIGN * rpm_command);
  target[Friction_Right] =
      (int16_t)(-FRICTION_FORWARD_SIGN * rpm_command);
  target[DJI_Motor_Reserved] = 0;
  g_friction_target_rpm = rpm;
  g_friction_target_percent = 100.0f * rpm / FRICTION_MAX_RPM;
  return 1U;
}

uint8_t Shooter_Set_Friction_Percent(int16_t target[Motor_Count],
                                     float percent) {
  if ((target == NULL) || !isfinite(percent) ||
      (percent < 0.0f) || (percent > 100.0f)) {
    return 0U;
  }

  if (!Shooter_Set_Friction_Rpm(
          target, FRICTION_MAX_RPM * percent / 100.0f)) {
    return 0U;
  }
  g_friction_target_percent = percent;
  return 1U;
}

uint8_t Shooter_Set_Feeder_Rpm(int16_t target[Motor_Count], float rpm) {
  if ((target == NULL) || !isfinite(rpm) ||
      (rpm < -FEEDER_MAX_RPM) || (rpm > FEEDER_MAX_RPM)) {
    return 0U;
  }

  const int16_t rpm_command = round_signed_rpm(rpm);
  target[Feeder] = (int16_t)(FEEDER_FORWARD_SIGN * rpm_command);
  target[DJI_Motor_Reserved] = 0;
  g_feeder_target_rpm = rpm;
  g_feeder_target_percent = 100.0f * rpm / FEEDER_MAX_RPM;
  return 1U;
}

uint8_t Shooter_Set_Feeder_Percent(int16_t target[Motor_Count],
                                   float percent) {
  if ((target == NULL) || !isfinite(percent) ||
      (percent < -100.0f) || (percent > 100.0f)) {
    return 0U;
  }

  if (!Shooter_Set_Feeder_Rpm(
          target, FEEDER_MAX_RPM * percent / 100.0f)) {
    return 0U;
  }
  g_feeder_target_percent = percent;
  return 1U;
}

uint8_t Shooter_Set_All(int16_t target[Motor_Count],
                        float friction_percent, float feeder_percent) {
  if ((target == NULL) || !isfinite(friction_percent) ||
      !isfinite(feeder_percent) ||
      (friction_percent < 0.0f) || (friction_percent > 100.0f) ||
      (feeder_percent < -100.0f) || (feeder_percent > 100.0f)) {
    return 0U;
  }

  (void)Shooter_Set_Friction_Percent(target, friction_percent);
  (void)Shooter_Set_Feeder_Percent(target, feeder_percent);
  return 1U;
}

void Shooter_Stop(int16_t target[Motor_Count]) {
  if (target == NULL) return;

  for (uint8_t i = 0U; i < Motor_Count; i++) {
    target[i] = 0;
    target_speed_rpm[i] = 0.0f;
    current_cmd[i] = 0;
    PID_Reset(&pid_spd[i]);
  }
  g_friction_target_percent = 0.0f;
  g_friction_target_rpm = 0.0f;
  g_feeder_target_percent = 0.0f;
  g_feeder_target_rpm = 0.0f;
}
