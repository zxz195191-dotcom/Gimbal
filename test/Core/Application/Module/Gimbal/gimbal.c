#include "headfile.h"
#include <math.h>

#define DEG_TO_RAD                   0.017453292519943295f
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
}

static void axis_stop(Gimbal_Axis_Runtime_t *axis) {
  axis_disable(axis);
  axis->status->state = YAW_TEST_DISABLED;
  axis->status->fault = YAW_FAULT_NONE;
  axis->status->target_relative_rad = axis->status->measured_relative_rad;
  axis->status->command_relative_rad = axis->status->measured_relative_rad;
}

static void axis_begin_enable(Gimbal_Axis_Runtime_t *axis, uint32_t now_ms) {
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
  ////////////////////////限幅


  } else {
    command_step = command_error;
  }
  status->command_relative_rad += command_step;

  /* Close the session coordinate loop from the latest measured raw position. */
  const float motor_target_rad = motor->p_int + axis->direction *
      (status->command_relative_rad - status->measured_relative_rad);


  ///////////////////////////////保险
  if (dm4310_send_mit(GIMBAL_CAN_HANDLE, axis->motor_id,
                      motor_target_rad,
                      axis->direction * target_velocity_rad_s,
                      status->kp, status->kd, 0.0f) != HAL_OK) {
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
  gimbal_update_key(now_ms, key_level);//按键状态变化的时刻更新一次时间 然后更新角度 但是用的不是按钮控制角度啊
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
