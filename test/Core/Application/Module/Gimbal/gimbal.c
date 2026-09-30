#include "headfile.h"
#include <math.h>

#define DEG_TO_RAD                   0.017453292519943295f
#define RAD_TO_DEG                   57.29577951308232
#define AXIS_CONTROL_PERIOD_MS       5U
#define AXIS_ENABLE_DELAY_MS         20U
#define AXIS_ENABLE_TIMEOUT_MS       500U
#define AXIS_REQUIRED_RX_FRAMES      5U
#define AXIS_MAX_TEMPERATURE_C       80.0f
#define AXIS_DEFAULT_KP              20.0f
#define PITCH_DEFAULT_KP             20.0f
#define AXIS_DEFAULT_KD              0.3f
#define PITCH_DEFAULT_KD             2.0f
#define AXIS_DEFAULT_SPEED_RAD_S     (30.0f * DEG_TO_RAD)
#define AXIS_MIN_SPEED_RAD_S         (1.0f * DEG_TO_RAD)
#define AXIS_MAX_SPEED_RAD_S         (120.0f * DEG_TO_RAD)
#define AXIS_POSITION_SPAN_RAD       25.0f
#define AXIS_POSITION_HALF_SPAN_RAD  12.5f
/* Pitch target is relative to the first valid feedback position after boot. */
#define PITCH_MIN_TARGET_RAD         (-100.0f * DEG_TO_RAD)
#define PITCH_MAX_TARGET_RAD         (145.0f * DEG_TO_RAD)
#define GIMBAL_KEY_DEBOUNCE_MS       30U
#define GIMBAL_KEY_LONG_PRESS_MS     1000U
#define GIMBAL_CAN_HANDLE            (&hcan2)
#define PITCH_SAMPLE_COUNT 15U
#define PITCH_GRAVITY_FF_MIN_DEG       10.0f
#define PITCH_GRAVITY_FF_MAX_DEG       35.0f
#define PITCH_GRAVITY_FF_MAX_ABS_NM     3.0f
#define PITCH_GRAVITY_FF_RATE_NM_S       1.0f
#define PITCH_GRAVITY_FF_MAX_STEP_MS    20U
#define PITCH_MOVE_DETECT_DEG            0.2f

static void pitch_gravity_ff_reset(void);

static float pitch_gravity_ff_update(float measured_deg, uint32_t elapsed_ms);

typedef struct {
  volatile Yaw_Test_t *status;
  DM_Motor_ID_e feedback_id;
  uint16_t motor_id;
  float direction;
  float last_raw_rad;
  float continuous_rad;
  uint32_t last_processed_rx_count;
  uint32_t state_enter_ms;
  uint32_t rx_count_at_enable;
  uint32_t last_control_ms;
  uint32_t last_disable_ms;
  float min_target_rad;
  float max_target_rad;
  uint8_t target_limit_enabled;
  uint8_t boot_zero_valid;
} Gimbal_Axis_Runtime_t;

volatile Yaw_Test_t g_yaw_test;
volatile Yaw_Test_t g_pitch_test;

static Gimbal_Axis_Runtime_t axes[DM_NUM];
static GPIO_PinState key_raw_last;
static GPIO_PinState key_stable;
static uint32_t key_changed_ms;
static uint32_t key_pressed_ms;
static uint8_t key_long_handled;

static float clampf(float value, float min_value, float max_value) {
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

static void axis_disable(Gimbal_Axis_Runtime_t *axis) {
  (void)dm4310_disable(GIMBAL_CAN_HANDLE, axis->motor_id);
}

static void axis_update_measurement(Gimbal_Axis_Runtime_t *axis) {
  volatile Yaw_Test_t *status = axis->status;
  const volatile DM_Motor_Data_t *motor = &dm_motor[axis->feedback_id];

  if (motor->rx_count == axis->last_processed_rx_count) return;

  const float raw_rad = motor->p_int;
  float delta_rad = raw_rad - axis->last_raw_rad;
  if (delta_rad > AXIS_POSITION_HALF_SPAN_RAD) {
    delta_rad -= AXIS_POSITION_SPAN_RAD;
  } else if (delta_rad < -AXIS_POSITION_HALF_SPAN_RAD) {
    delta_rad += AXIS_POSITION_SPAN_RAD;
  }

  axis->continuous_rad += axis->direction * delta_rad;
  axis->last_raw_rad = raw_rad;
  axis->last_processed_rx_count = motor->rx_count;
  axis->boot_zero_valid = 1U;
  status->measured_relative_rad = axis->continuous_rad;
  status->measured_velocity_rad_s = axis->direction * motor->v_int;
}

static void axis_capture_boot_zero(Gimbal_Axis_Runtime_t *axis) {
  volatile Yaw_Test_t *status = axis->status;
  const volatile DM_Motor_Data_t *motor = &dm_motor[axis->feedback_id];

  status->origin_rad = motor->p_int;
  status->measured_relative_rad = 0.0f;
  status->measured_velocity_rad_s = axis->direction * motor->v_int;
  status->target_relative_rad = 0.0f;
  status->command_relative_rad = 0.0f;
  axis->last_raw_rad = motor->p_int;
  axis->continuous_rad = 0.0f;
  axis->last_processed_rx_count = motor->rx_count;
  axis->boot_zero_valid = 1U;
}

static void axis_enter_fault(Gimbal_Axis_Runtime_t *axis, Yaw_Fault_e fault) {
  axis_disable(axis);
  axis->last_disable_ms = HAL_GetTick();
  axis->status->fault = fault;
  axis->status->state = YAW_TEST_FAULT;
  axis->status->target_relative_rad = axis->status->measured_relative_rad;
  axis->status->command_relative_rad = axis->status->measured_relative_rad;
  axis->status->applied_t_ff_nm = 0.0f;
  if (axis->feedback_id == Pitch) pitch_gravity_ff_reset();
}

static void axis_stop(Gimbal_Axis_Runtime_t *axis) {
  axis_disable(axis);
  axis->status->state = YAW_TEST_DISABLED;
  axis->status->fault = YAW_FAULT_NONE;
  axis->status->target_relative_rad = axis->status->measured_relative_rad;
  axis->status->command_relative_rad = axis->status->measured_relative_rad;
  axis->status->applied_t_ff_nm = 0.0f;
  if (axis->feedback_id == Pitch) pitch_gravity_ff_reset();
}

static void axis_begin_enable(Gimbal_Axis_Runtime_t *axis, uint32_t now_ms) {
  axis->status->applied_t_ff_nm = 0.0f;
  if (axis->feedback_id == Pitch) pitch_gravity_ff_reset();
  if ((g_can2_filter_ret != HAL_OK) ||
      (g_can2_start_ret != HAL_OK) ||
      (g_can2_notify_ret != HAL_OK)) {
    axis_enter_fault(axis, YAW_FAULT_CAN_INIT);
    return;
  }

  if (dm4310_clear_error(GIMBAL_CAN_HANDLE, axis->motor_id) != HAL_OK) {
    axis_enter_fault(axis, YAW_FAULT_TX);
    return;
  }

  axis->rx_count_at_enable = dm_motor[axis->feedback_id].rx_count;
  axis->state_enter_ms = now_ms;
  axis->status->state = YAW_TEST_CLEARING;
  axis->status->fault = YAW_FAULT_NONE;
}

static int8_t pitch_approach_sign = 1;

static void axis_update(Gimbal_Axis_Runtime_t *axis, uint32_t now_ms) {
  volatile Yaw_Test_t *status = axis->status;
  const volatile DM_Motor_Data_t *motor = &dm_motor[axis->feedback_id];

  if (status->state == YAW_TEST_CLEARING) {
    if ((now_ms - axis->state_enter_ms) >= AXIS_ENABLE_DELAY_MS) {
      if (dm4310_enable(GIMBAL_CAN_HANDLE, axis->motor_id) != HAL_OK) {
        axis_enter_fault(axis, YAW_FAULT_TX);
        return;
      }
      axis->rx_count_at_enable = motor->rx_count;
      axis->last_control_ms = now_ms;
      axis->state_enter_ms = now_ms;
      status->state = YAW_TEST_WAIT_FEEDBACK;
    }
    return;
  }

  if (status->state == YAW_TEST_WAIT_FEEDBACK) {
    if (DM_Motor_Feedback_Valid(axis->feedback_id, now_ms) &&
        ((motor->rx_count - axis->rx_count_at_enable) >= 1U)) {
      if (!axis->boot_zero_valid) {
        axis_capture_boot_zero(axis);
      } else {
        axis_update_measurement(axis);
        status->target_relative_rad = status->measured_relative_rad;
        status->command_relative_rad = status->measured_relative_rad;
      }
      axis->last_control_ms = now_ms;
      axis->state_enter_ms = now_ms;
      status->state = YAW_TEST_VERIFY_FEEDBACK;
      return;
    }
    if ((now_ms - axis->last_control_ms) >= AXIS_CONTROL_PERIOD_MS) {
      axis->last_control_ms = now_ms;
      if (dm4310_send_mit(GIMBAL_CAN_HANDLE, axis->motor_id,
                          0.0f, 0.0f, 0.0f, 0.0f, 0.0f) != HAL_OK) {
        axis_enter_fault(axis, YAW_FAULT_TX);
        return;
      }
      status->tx_count++;
    }
    if ((now_ms - axis->state_enter_ms) > AXIS_ENABLE_TIMEOUT_MS) {
      axis_enter_fault(axis, YAW_FAULT_ENABLE_TIMEOUT);
    }
    return;
  }

  if (status->state == YAW_TEST_VERIFY_FEEDBACK) {
    if (!DM_Motor_Feedback_Valid(axis->feedback_id, now_ms)) {
      axis_enter_fault(axis, YAW_FAULT_FEEDBACK_TIMEOUT);
      return;
    }
    axis_update_measurement(axis);
    if ((motor->state >= 8U) ||
        (motor->t_mos >= AXIS_MAX_TEMPERATURE_C) ||
        (motor->t_rotor >= AXIS_MAX_TEMPERATURE_C)) {
      axis_enter_fault(axis, YAW_FAULT_MOTOR_ERROR);
      return;
    }
    if ((now_ms - axis->last_control_ms) >= AXIS_CONTROL_PERIOD_MS) {
      axis->last_control_ms = now_ms;
      if (dm4310_send_mit(GIMBAL_CAN_HANDLE, axis->motor_id,
                          status->origin_rad, 0.0f,
                          0.0f, 0.0f, 0.0f) != HAL_OK) {
        axis_enter_fault(axis, YAW_FAULT_TX);
        return;
      }
      status->tx_count++;
    }
    if ((motor->rx_count - axis->rx_count_at_enable) >=
        AXIS_REQUIRED_RX_FRAMES) {
      axis->last_control_ms = now_ms;
      status->state = YAW_TEST_ACTIVE;
      return;
    }
    if ((now_ms - axis->state_enter_ms) > AXIS_ENABLE_TIMEOUT_MS) {
      axis_enter_fault(axis, YAW_FAULT_ENABLE_TIMEOUT);
    }
    return;
  }

  if (status->state == YAW_TEST_FAULT) {
    if ((now_ms - axis->last_disable_ms) >= 50U) {
      axis_disable(axis);
      axis->last_disable_ms = now_ms;
    }
    return;
  }

  if (status->state != YAW_TEST_ACTIVE) return;

  if (!DM_Motor_Feedback_Valid(axis->feedback_id, now_ms)) {
    axis_enter_fault(axis, YAW_FAULT_FEEDBACK_TIMEOUT);
    return;
  }
#if 0
  axis_update(axis)
  │
  ├──【启动状态机】先别细读
  │     CLEARING
  │       ↓
  │     WAIT_FEEDBACK
  │       ↓
  │     VERIFY_FEEDBACK
  │       ↓
  │     ACTIVE
  │
  ├──【故障保护】脑内折叠
  │     反馈超时？
  │     电机报错？
  │     过温？
  │
  └──【真正控制】重点<--=============这里
    │
    ├─ axis_update_measurement()
    │      ↓
    │   measured_relative
    │
    ├─ target_relative
    │      ↓
    │   限制每周期最大移动量
    │      ↓
    │   command_relative
    │
    ├─ command - measured
    │      ↓
    │   motor_target_rad
    │
    └─ dm4310_send_mit
           │
           ├─ p_des
           ├─ v_des
           ├─ kp
           ├─ kd
           └─ t_ff
#endif

  axis_update_measurement(axis);


  ////////////////////////////////////////////保护
  if (motor->state >= 8U) {
    axis_enter_fault(axis, YAW_FAULT_MOTOR_ERROR);
    return;
  }
  if ((motor->t_mos >= AXIS_MAX_TEMPERATURE_C) ||
      (motor->t_rotor >= AXIS_MAX_TEMPERATURE_C)) {
    axis_enter_fault(axis, YAW_FAULT_OVER_TEMPERATURE);
    return;
  }
  if ((now_ms - axis->last_control_ms) < AXIS_CONTROL_PERIOD_MS) return;
  ///////////////////////////////////////////////



  const uint32_t elapsed_ms = now_ms - axis->last_control_ms;
  axis->last_control_ms = now_ms;

  const float max_step = status->max_speed_rad_s * ((float)elapsed_ms / 1000.0f);
  //一次循环允许移动的最大步进 = 最大速度 * 经过时间（单位s）

  const float command_error = status->target_relative_rad - status->command_relative_rad;
  float command_step;
  float target_velocity_rad_s = 0.0f;


  ////////////////////////限幅
  if (command_error > max_step) {
    command_step = max_step;
    target_velocity_rad_s = status->max_speed_rad_s;
  } else if (command_error < -max_step) {
    command_step = -max_step;
    target_velocity_rad_s = -status->max_speed_rad_s;
  }
  else {
    command_step = command_error;
  }
  ////////////////////////限幅

  if (axis->feedback_id == Pitch) {
    if (command_step > 0.0f) {
      pitch_approach_sign  = 1 ;
    }
    else if (command_step < 0.0f) {
      pitch_approach_sign = -1 ;
    }
  }




  status->command_relative_rad += command_step;

  /* Close the session coordinate loop from the latest measured raw position. */
  const float motor_target_rad = motor->p_int + axis->direction *
      (status->command_relative_rad - status->measured_relative_rad);

  float t_ff_nm = 0.0f;
  if (axis->feedback_id == Pitch) {
    const float logical_t_ff_nm = pitch_gravity_ff_update(
        // status->measured_relative_rad * RAD_TO_DEG, elapsed_ms);
        status->command_relative_rad * RAD_TO_DEG, elapsed_ms);

    t_ff_nm = axis->direction * logical_t_ff_nm;
  }
  status->applied_t_ff_nm = t_ff_nm;


  ///////////////////////////////保险
  if (dm4310_send_mit(GIMBAL_CAN_HANDLE, axis->motor_id,
                      motor_target_rad,
                      axis->direction * target_velocity_rad_s,
                      status->kp, status->kd, t_ff_nm) != HAL_OK) {
    axis_enter_fault(axis, YAW_FAULT_TX);
    return;
  }
  ///////////////////////////////保险


  status->tx_count++;
}





static uint8_t axis_set_max_speed(Gimbal_Axis_Runtime_t *axis,
                                  float max_speed_rad_s) {
  if ((max_speed_rad_s < AXIS_MIN_SPEED_RAD_S) ||
      (max_speed_rad_s > AXIS_MAX_SPEED_RAD_S)) {
    return 0U;
  }
  axis->status->max_speed_rad_s = max_speed_rad_s;
  return 1U;
}

static uint8_t axis_set_target(Gimbal_Axis_Runtime_t *axis,
                               float target_rad) {
  if (axis->status->state != YAW_TEST_ACTIVE) return 0U;
  if (!isfinite(target_rad)) return 0U;
  if (axis->target_limit_enabled) {
    target_rad = clampf(target_rad,
                        axis->min_target_rad,
                        axis->max_target_rad);
  }
  axis->status->target_relative_rad = target_rad;
  return 1U;
}

static uint8_t axis_set_target_speed(Gimbal_Axis_Runtime_t *axis,
                                     float target_rad,
                                     float max_speed_rad_s) {
  if (!axis_set_max_speed(axis, max_speed_rad_s)) return 0U;
  return axis_set_target(axis, target_rad);
}

static uint8_t axis_move_relative(Gimbal_Axis_Runtime_t *axis,
                                  float delta_rad) {
  if (axis->status->state != YAW_TEST_ACTIVE) return 0U;
  if (!isfinite(delta_rad)) return 0U;
  /* Public relative commands are converted once to the internal absolute
   * setpoint. They are based on the latest measured continuous position.
   * Relative moves intentionally bypass the absolute target soft limits. */
  const float target_rad =
      axis->status->measured_relative_rad + delta_rad;
  if (!isfinite(target_rad)) return 0U;
  axis->status->target_relative_rad = target_rad;
  return 1U;
}

static uint8_t axis_move_relative_speed(Gimbal_Axis_Runtime_t *axis,
                                        float delta_rad,
                                        float max_speed_rad_s) {
  if (!axis_set_max_speed(axis, max_speed_rad_s)) return 0U;
  return axis_move_relative(axis, delta_rad);
}

static uint8_t axis_set_kp(Gimbal_Axis_Runtime_t *axis, float kp) {
  if ((kp < 0.0f) ) return 0U;
    // || (kp > 20.0f)) return 0U;
  axis->status->kp = kp;
  return 1U;
}

static uint8_t axis_set_kd(Gimbal_Axis_Runtime_t *axis, float kd) {
  if ((kd < 0.0f) || (kd > 3.0f)) return 0U;
  axis->status->kd = kd;
  return 1U;
}

static void gimbal_update_key(uint32_t now_ms, GPIO_PinState raw_level) {
  if (raw_level != key_raw_last) {
    key_raw_last = raw_level;
    key_changed_ms = now_ms;
  }

  if ((raw_level != key_stable) &&
      ((now_ms - key_changed_ms) >= GIMBAL_KEY_DEBOUNCE_MS)) {
    key_stable = raw_level;
    if (key_stable == GPIO_PIN_SET) {
      key_pressed_ms = now_ms;
      key_long_handled = 0U;
    } else if (!key_long_handled) {
      const uint8_t both_idle =
          ((g_yaw_test.state == YAW_TEST_DISABLED) ||
           (g_yaw_test.state == YAW_TEST_FAULT)) &&
          ((g_pitch_test.state == YAW_TEST_DISABLED) ||
           (g_pitch_test.state == YAW_TEST_FAULT));
      if (both_idle) Gimbal_Enable_All();
      else Gimbal_Stop_All();
    }
  }

  if ((key_stable == GPIO_PIN_SET) && !key_long_handled &&
      ((now_ms - key_pressed_ms) >= GIMBAL_KEY_LONG_PRESS_MS)) {
    key_long_handled = 1U;
    Gimbal_Stop_All();
  }
}

void Gimbal_Yaw_Test_Init(void) {
  memset((void *)&g_yaw_test, 0, sizeof(g_yaw_test));
  memset((void *)&g_pitch_test, 0, sizeof(g_pitch_test));
  memset(axes, 0, sizeof(axes));

  axes[Yaw].status = &g_yaw_test;
  axes[Yaw].feedback_id = Yaw;
  axes[Yaw].motor_id = DM_YAW_CAN_ID;
  axes[Yaw].direction = GIMBAL_YAW_DIRECTION;

  axes[Pitch].status = &g_pitch_test;
  axes[Pitch].feedback_id = Pitch;
  axes[Pitch].motor_id = DM_PITCH_CAN_ID;
  axes[Pitch].direction = GIMBAL_PITCH_DIRECTION;
  axes[Pitch].min_target_rad = PITCH_MIN_TARGET_RAD;
  axes[Pitch].max_target_rad = PITCH_MAX_TARGET_RAD;
  axes[Pitch].target_limit_enabled = 1U;

  g_yaw_test.state = YAW_TEST_DISABLED;
  g_pitch_test.state = YAW_TEST_DISABLED;
  g_yaw_test.kp = AXIS_DEFAULT_KP;
  g_pitch_test.kp = PITCH_DEFAULT_KP;
  g_yaw_test.kd = AXIS_DEFAULT_KD;
  g_pitch_test.kd = PITCH_DEFAULT_KD;
  g_yaw_test.max_speed_rad_s = AXIS_DEFAULT_SPEED_RAD_S;
  g_pitch_test.max_speed_rad_s = AXIS_DEFAULT_SPEED_RAD_S;

  key_raw_last = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
  key_stable = key_raw_last;
  key_changed_ms = HAL_GetTick();
}

void Gimbal_Yaw_Test_Update(uint32_t now_ms, GPIO_PinState key_level) {
  gimbal_update_key(now_ms, key_level);
  axis_update(&axes[Yaw], now_ms);
  axis_update(&axes[Pitch], now_ms);
}

void Gimbal_Yaw_Test_Stop(void) { axis_stop(&axes[Yaw]); }
void Gimbal_Pitch_Stop(void) { axis_stop(&axes[Pitch]); }
void Gimbal_Stop_All(void) {
  axis_stop(&axes[Yaw]);
  axis_stop(&axes[Pitch]);
}

void Gimbal_Yaw_Enable(void) {
  if ((g_yaw_test.state == YAW_TEST_DISABLED) ||
      (g_yaw_test.state == YAW_TEST_FAULT)) {
    axis_begin_enable(&axes[Yaw], HAL_GetTick());
  }
}

void Gimbal_Pitch_Enable(void) {
  if ((g_pitch_test.state == YAW_TEST_DISABLED) ||
      (g_pitch_test.state == YAW_TEST_FAULT)) {
    axis_begin_enable(&axes[Pitch], HAL_GetTick());
  }
}

void Gimbal_Enable_All(void) {
  Gimbal_Yaw_Enable();
  Gimbal_Pitch_Enable();
}

uint8_t Gimbal_Yaw_Set_Target_Deg(float target_deg) {
  return Gimbal_Yaw_Set_Target_Rad(target_deg * DEG_TO_RAD);
}
uint8_t Gimbal_Yaw_Set_Target_Rad(float target_rad) {
  return axis_set_target(&axes[Yaw], target_rad);
}
uint8_t Gimbal_Yaw_Set_Target_Speed_Deg(float target_deg,
                                         float max_speed_deg_s) {
  return axis_set_target_speed(&axes[Yaw], target_deg * DEG_TO_RAD,
                               max_speed_deg_s * DEG_TO_RAD);
}
uint8_t Gimbal_Yaw_Move_Relative_Deg(float delta_deg) {
  return axis_move_relative(&axes[Yaw], delta_deg * DEG_TO_RAD);
}
uint8_t Gimbal_Yaw_Move_Relative_Speed_Deg(float delta_deg,
                                            float max_speed_deg_s) {
  return axis_move_relative_speed(&axes[Yaw], delta_deg * DEG_TO_RAD,
                                  max_speed_deg_s * DEG_TO_RAD);
}
uint8_t Gimbal_Yaw_Set_Max_Speed_Deg(float max_speed_deg_s) {
  return axis_set_max_speed(&axes[Yaw], max_speed_deg_s * DEG_TO_RAD);
}
uint8_t Gimbal_Yaw_Set_Kp(float kp) { return axis_set_kp(&axes[Yaw], kp); }
uint8_t Gimbal_Yaw_Set_Kd(float kd) { return axis_set_kd(&axes[Yaw], kd); }

uint8_t Gimbal_Pitch_Set_Target_Deg(float target_deg) {
  return Gimbal_Pitch_Set_Target_Rad(target_deg * DEG_TO_RAD);
}
uint8_t Gimbal_Pitch_Set_Target_Rad(float target_rad) {
  return axis_set_target(&axes[Pitch], target_rad);
}
uint8_t Gimbal_Pitch_Set_Target_Speed_Deg(float target_deg,
                                           float max_speed_deg_s) {
  return axis_set_target_speed(&axes[Pitch], target_deg * DEG_TO_RAD,
                               max_speed_deg_s * DEG_TO_RAD);
}
uint8_t Gimbal_Pitch_Move_Relative_Deg(float delta_deg) {
  return axis_move_relative(&axes[Pitch], delta_deg * DEG_TO_RAD);
}
uint8_t Gimbal_Pitch_Move_Relative_Speed_Deg(float delta_deg,
                                              float max_speed_deg_s) {
  return axis_move_relative_speed(&axes[Pitch], delta_deg * DEG_TO_RAD,
                                  max_speed_deg_s * DEG_TO_RAD);
}
uint8_t Gimbal_Pitch_Set_Max_Speed_Deg(float max_speed_deg_s) {
  return axis_set_max_speed(&axes[Pitch], max_speed_deg_s * DEG_TO_RAD);
}
uint8_t Gimbal_Pitch_Set_Kp(float kp) {
  return axis_set_kp(&axes[Pitch], kp);
}
uint8_t Gimbal_Pitch_Set_Kd(float kd) {
  return axis_set_kd(&axes[Pitch], kd);
}


typedef struct {
  float error_up_deg;
  float error_down_deg;

  float torque_up;
  float torque_down;

  float gravity_torque;
  float friction_torque;

}Pitch_Compensation_Result_t;


//TODO
//优化成指针
Pitch_Compensation_Result_t Pitch_Calc_Comp(float target_deg,float measured_up_deg,float measured_down_deg,float t_int_up,float t_int_down) {
  Pitch_Compensation_Result_t result = {0};//局部很有必要

  result.error_up_deg = target_deg - measured_up_deg;//可能出现的-号会很有价值（补偿过头）
  result.error_down_deg = target_deg - measured_down_deg;

  // result.torque_up = kp * result.error_up_deg * DEG_TO_RAD;
  // result.torque_down = kp * result.error_down_deg * DEG_TO_RAD;

  result.torque_up = t_int_up;
  result.torque_down = t_int_down;

  result.gravity_torque = (result.torque_up + result.torque_down) * 0.5f;
  result.friction_torque = (result.torque_up - result.torque_down) * 0.5f;

  return result;
}

#define PITCH_CAL_MIN_DEG      5.0f
#define PITCH_CAL_MAX_DEG     50.0f
#define PITCH_CAL_STEP_DEG     5.0f
#define PITCH_CAL_POINT_COUNT  10U


typedef struct
{
  float target_deg;

  float measured_up_deg;
  float measured_down_deg;

  float t_int_up;
  float t_int_down;

  uint32_t move_delay_up_ms;
  uint32_t move_delay_down_ms;
  float peak_velocity_up_deg_s;
  float peak_velocity_down_deg_s;

  Pitch_Compensation_Result_t comp;

} Pitch_Cal_Point_t;



static Pitch_Cal_Point_t pitch_points[PITCH_CAL_POINT_COUNT];

void Pitch_Cal_Init_Points(void)
{
  memset(pitch_points, 0, sizeof(pitch_points));

  for (uint8_t i = 0; i < PITCH_CAL_POINT_COUNT; i++)
  {
    pitch_points[i].target_deg = PITCH_CAL_MIN_DEG + i * PITCH_CAL_STEP_DEG;
  }
}


typedef enum
{
  PITCH_CAL_UP = 0,
  PITCH_CAL_DOWN,
  PITCH_CAL_DONE
} Pitch_Cal_Direction_t;

typedef struct
{
  uint8_t index;
  Pitch_Cal_Direction_t direction;

  uint8_t waiting;
  uint8_t sampling;
  uint32_t stable_start_ms;

  uint32_t move_command_ms;
  uint32_t move_delay_ms;
  float move_start_deg;
  float peak_velocity_deg_s;
  uint8_t move_started;

} Pitch_Cal_Runtime_t;

static Pitch_Cal_Runtime_t pitch_cal_runtime = {0};
static float pitch_gravity_ff_scale = 0.0f;
static float pitch_gravity_ff_applied_nm = 0.0f;
static volatile float pitch_friction_ff_scale = 1.0f;

void Pitch_Cal_Start(void)
{
  /* Calibration must measure PID holding torque without feedforward. */
  pitch_gravity_ff_scale = 0.0f;
  pitch_gravity_ff_reset();
  Pitch_Cal_Init_Points();

  pitch_cal_runtime.index = 0;
  pitch_cal_runtime.direction = PITCH_CAL_UP;
  pitch_cal_runtime.waiting = 0;
  pitch_cal_runtime.sampling = 0;
  pitch_cal_runtime.stable_start_ms = 0;
  pitch_cal_runtime.move_command_ms = 0U;
  pitch_cal_runtime.move_delay_ms = UINT32_MAX;
  pitch_cal_runtime.move_start_deg = 0.0f;
  pitch_cal_runtime.peak_velocity_deg_s = 0.0f;
  pitch_cal_runtime.move_started = 0U;
}



void Pitch_Cal_Save(uint8_t index,Pitch_Cal_Direction_t direction,float measured_deg,float t_int)
{
  if (index >= PITCH_CAL_POINT_COUNT)
    return;

  if (direction == PITCH_CAL_UP)
  {
    pitch_points[index].measured_up_deg = measured_deg;
    pitch_points[index].t_int_up = t_int;
  }
  else
  {
    pitch_points[index].measured_down_deg = measured_deg;
    pitch_points[index].t_int_down = t_int;
  }
}


void Pitch_Cal_Calculate_All(void)
{
  /* 5 deg and 50 deg are approach/turnaround endpoints. Only 10..45 deg
   * have valid measurements from both travel directions. */
  for (uint8_t i = 1U; i < (PITCH_CAL_POINT_COUNT - 1U); i++)
  {
    pitch_points[i].comp =
        Pitch_Calc_Comp(
            pitch_points[i].target_deg,
            pitch_points[i].measured_up_deg,
            pitch_points[i].measured_down_deg,
            pitch_points[i].t_int_up,
            pitch_points[i].t_int_down);
  }
}

static float Pitch_Compensation_FF_Lookup(float measured_deg)
{
  if ((pitch_cal_runtime.direction != PITCH_CAL_DONE) ||
      (pitch_gravity_ff_scale <= 0.0f) ||
      (measured_deg < PITCH_GRAVITY_FF_MIN_DEG) ||
      (measured_deg > PITCH_GRAVITY_FF_MAX_DEG))
  {
    return 0.0f;
  }

  const uint8_t first = 1U; /* 10 deg */
  const uint8_t last = 6U;  /* 35 deg */

  float gravity_nm;
  float friction_nm;

  if (measured_deg >= pitch_points[last].target_deg)
  {
    gravity_nm = pitch_points[last].comp.gravity_torque;
    friction_nm = pitch_points[last].comp.friction_torque;
  }
  else
    {
    uint8_t lower = first;

    while ((lower < last) && (measured_deg > pitch_points[lower + 1U].target_deg))
      {
        lower++;
      }
    const float x0 = pitch_points[lower].target_deg;
    const float ratio = (measured_deg - x0) / PITCH_CAL_STEP_DEG;
    const float g0 = pitch_points[lower].comp.gravity_torque;
    const float g1 = pitch_points[lower + 1U].comp.gravity_torque;
    const float f0 = pitch_points[lower].comp.friction_torque;
    const float f1 = pitch_points[lower + 1U].comp.friction_torque;
    gravity_nm = g0 + ratio * (g1 - g0);
    friction_nm = f0 + ratio * (f1 - f0);
  }

  const float compensation_nm = gravity_nm + (float)pitch_approach_sign * pitch_friction_ff_scale * friction_nm;

  return clampf(compensation_nm * pitch_gravity_ff_scale,
                -PITCH_GRAVITY_FF_MAX_ABS_NM,
                PITCH_GRAVITY_FF_MAX_ABS_NM);
}

static void pitch_gravity_ff_reset(void)
{
  pitch_gravity_ff_applied_nm = 0.0f;
}

static float pitch_gravity_ff_update(float measured_deg, uint32_t elapsed_ms)
{
  const float desired_nm = Pitch_Compensation_FF_Lookup(measured_deg);
  const uint32_t limited_ms = (elapsed_ms > PITCH_GRAVITY_FF_MAX_STEP_MS) ?
                              PITCH_GRAVITY_FF_MAX_STEP_MS : elapsed_ms;
  const float max_delta_nm = PITCH_GRAVITY_FF_RATE_NM_S *
                             ((float)limited_ms / 1000.0f);
  const float delta_nm = clampf(desired_nm - pitch_gravity_ff_applied_nm,
                                -max_delta_nm, max_delta_nm);
  pitch_gravity_ff_applied_nm += delta_nm;
  return pitch_gravity_ff_applied_nm;
}

uint8_t Gimbal_Pitch_Set_Gravity_FF_Scale(float scale)
{
  if (!isfinite(scale) || (scale < 0.0f) || (scale > 1.0f)) return 0U;
  if (scale == 0.0f) {
    pitch_gravity_ff_scale = 0.0f;
    pitch_gravity_ff_reset();
    g_pitch_test.applied_t_ff_nm = 0.0f;
    return 1U;
  }
  if ((scale > 0.0f) && (pitch_cal_runtime.direction != PITCH_CAL_DONE)) return 0U;
  pitch_gravity_ff_scale = scale;
  return 1U;
}

static void Pitch_Cal_Move_Diagnostics_Start(uint32_t now_ms)
{
  pitch_cal_runtime.move_command_ms = now_ms;
  pitch_cal_runtime.move_delay_ms = UINT32_MAX;
  pitch_cal_runtime.move_start_deg =
      g_pitch_test.measured_relative_rad * RAD_TO_DEG;
  pitch_cal_runtime.peak_velocity_deg_s = 0.0f;
  pitch_cal_runtime.move_started = 0U;
}

static void Pitch_Cal_Move_Diagnostics_Update(uint32_t now_ms)
{
  const float measured_deg =
      g_pitch_test.measured_relative_rad * RAD_TO_DEG;
  const float velocity_deg_s =
      fabsf(g_pitch_test.measured_velocity_rad_s) * RAD_TO_DEG;

  if (velocity_deg_s > pitch_cal_runtime.peak_velocity_deg_s) {
    pitch_cal_runtime.peak_velocity_deg_s = velocity_deg_s;
  }
  if ((pitch_cal_runtime.move_started == 0U) &&
      (fabsf(measured_deg - pitch_cal_runtime.move_start_deg) >=
       PITCH_MOVE_DETECT_DEG)) {
    pitch_cal_runtime.move_started = 1U;
    pitch_cal_runtime.move_delay_ms =
        now_ms - pitch_cal_runtime.move_command_ms;
  }
}

static void Pitch_Cal_Move_Diagnostics_Save(
    uint8_t index, Pitch_Cal_Direction_t direction)
{
  if (direction == PITCH_CAL_UP) {
    pitch_points[index].move_delay_up_ms = pitch_cal_runtime.move_delay_ms;
    pitch_points[index].peak_velocity_up_deg_s =
        pitch_cal_runtime.peak_velocity_deg_s;
  } else {
    pitch_points[index].move_delay_down_ms = pitch_cal_runtime.move_delay_ms;
    pitch_points[index].peak_velocity_down_deg_s =
        pitch_cal_runtime.peak_velocity_deg_s;
  }
}

static void Pitch_Sampler_Reset(void);
static void Pitch_Sampler_Push(void);
static void Pitch_Sampler_Get_Result(float *measured_deg,float *t_int);

typedef struct {
  uint8_t count;
  uint32_t last_rx_count;

  float measured_sum;
  float measured_min;
  float measured_max;

  float torque_sum;
  float torque_min;
  float torque_max;
}Pitch_Sampler_t;

static Pitch_Sampler_t pitch_sampler;

void Pitch_Cal_Update(uint32_t now_ms) {

  if (pitch_cal_runtime.direction == PITCH_CAL_DONE) return;

  uint8_t index = pitch_cal_runtime.index;

  if (pitch_cal_runtime.waiting == 0U) {
    if (Gimbal_Pitch_Set_Target_Deg(pitch_points[index].target_deg) != 0U)
      {
        pitch_cal_runtime.waiting = 1U;
        pitch_cal_runtime.stable_start_ms = 0U;
        Pitch_Cal_Move_Diagnostics_Start(now_ms);
      }
    return;
  }

  Pitch_Cal_Move_Diagnostics_Update(now_ms);

  /* command 还没走到 target */
  float command_error_deg =
      fabsf(
          g_pitch_test.target_relative_rad -
          g_pitch_test.command_relative_rad
      ) * RAD_TO_DEG;

  if (command_error_deg > 0.05f)
  {
    pitch_cal_runtime.stable_start_ms = 0;
    pitch_cal_runtime.sampling = 0U;
    return;
  }

  /* command 已经到了，开始等 400ms */
  if (pitch_cal_runtime.stable_start_ms == 0)
  {
    pitch_cal_runtime.stable_start_ms = now_ms;
    return;
  }

  if ((now_ms - pitch_cal_runtime.stable_start_ms) < 1000U) return;

  if (pitch_cal_runtime.sampling == 0U)
  {
    Pitch_Sampler_Reset();
    pitch_cal_runtime.sampling = 1U;
  }

  Pitch_Sampler_Push();

  if (pitch_sampler.count < PITCH_SAMPLE_COUNT) return;//采集PITCH_SAMPLE_COUNT（15）次

  float measured_deg;
  float t_int;

  Pitch_Sampler_Get_Result(&measured_deg, &t_int);

  Pitch_Cal_Save(index,pitch_cal_runtime.direction,measured_deg,t_int);
  Pitch_Cal_Move_Diagnostics_Save(index, pitch_cal_runtime.direction);

  pitch_cal_runtime.waiting = 0U;
  pitch_cal_runtime.sampling = 0U;

  /////////////////////////////////////////////状态切换
  if (pitch_cal_runtime.direction == PITCH_CAL_UP) {
    if (index < PITCH_CAL_POINT_COUNT - 1U)
    {
      pitch_cal_runtime.index++;
    }
    else {
      pitch_cal_runtime.direction = PITCH_CAL_DOWN;
      pitch_cal_runtime.index--;
    }
  }
  else {
    if (index > 0U){
      pitch_cal_runtime.index--;
    }
    else {
      pitch_cal_runtime.direction = PITCH_CAL_DONE;

      Pitch_Cal_Calculate_All( );
    }
  }
  /////////////////////////////////////////////状态切换
}



static void Pitch_Sampler_Reset(void)
{
  pitch_sampler.count = 0U;

  pitch_sampler.measured_sum = 0.0f;
  pitch_sampler.torque_sum = 0.0f;

  pitch_sampler.last_rx_count = dm_motor[Pitch].rx_count;
}

static void Pitch_Sampler_Push(void) {
  if (dm_motor[Pitch].rx_count == pitch_sampler.last_rx_count) return;

  pitch_sampler.last_rx_count = dm_motor[Pitch].rx_count;

  float measured = g_pitch_test.measured_relative_rad * RAD_TO_DEG;

  float torque = dm_motor[Pitch].t_int;

  if (pitch_sampler.count == 0U) {
    pitch_sampler.measured_min = measured;
    pitch_sampler.measured_max = measured;

    pitch_sampler.torque_min = torque;
    pitch_sampler.torque_max = torque;
  }

  pitch_sampler.measured_sum += measured;
  pitch_sampler.torque_sum += torque;

  if (measured < pitch_sampler.measured_min) pitch_sampler.measured_min = measured;
  if (measured > pitch_sampler.measured_max) pitch_sampler.measured_max = measured;

  if (torque < pitch_sampler.torque_min) pitch_sampler.torque_min = torque;
  if (torque > pitch_sampler.torque_max) pitch_sampler.torque_max = torque;

  pitch_sampler.count++;
}



static void Pitch_Sampler_Get_Result(float *measured_deg,float *t_int) {
  const float n = 1.0f / (float)(PITCH_SAMPLE_COUNT - 2U);

  *measured_deg = (pitch_sampler.measured_sum - pitch_sampler.measured_min - pitch_sampler.measured_max) * n ;

  *t_int = (pitch_sampler.torque_sum - pitch_sampler.torque_max - pitch_sampler.torque_min) * n;
}
