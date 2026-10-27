#include "headfile.h"
#include <math.h>
#include "pitch_continuous_cal.h"
#include "load_compensation.h"
#include "yaw_load_cal.h"
#include "yaw_diagnostics.h"
#include "yaw_friction_goal.h"
#include "pitch_inertial.h"

#define DEG_TO_RAD                   0.017453292519943295f
#define RAD_TO_DEG                   57.29577951308232
#define AXIS_CONTROL_PERIOD_MS       5U
/* Ordinary encoder Yaw only; engineering brake estimate, not measured inertia. */
#define YAW_REFERENCE_BRAKE_RAD_S2 (800.0f * DEG_TO_RAD)
#define YAW_REFERENCE_EXTRA_NM .35f
#define YAW_REFERENCE_TOTAL_NM 1.5f
#define AXIS_ENABLE_DELAY_MS         20U
#define AXIS_ENABLE_TIMEOUT_MS       500U
#define AXIS_REQUIRED_RX_FRAMES      5U
#define AXIS_MAX_TEMPERATURE_C       80.0f
#define AXIS_DEFAULT_KP              20.0f
#define YAW_MAX_KP                   35.0f /* Bench tuning cap; stability needs hardware validation. */
#define PITCH_DEFAULT_KP             25.0f
#define AXIS_DEFAULT_KD              0.3f
#define PITCH_DEFAULT_KD             2.0f
#define AXIS_DEFAULT_SPEED_RAD_S     (30.0f * DEG_TO_RAD)
#define AXIS_MIN_SPEED_RAD_S         (1.0f * DEG_TO_RAD)
#define AXIS_MAX_SPEED_RAD_S         (120.0f * DEG_TO_RAD)
#define AXIS_POSITION_SPAN_RAD       25.0f
#define AXIS_POSITION_HALF_SPAN_RAD  12.5f
/* Pitch target is relative to the first valid feedback position after boot. */
#define PITCH_MIN_TARGET_RAD         (-100.0f * DEG_TO_RAD)
#define PITCH_MAX_TARGET_RAD         (PITCH_SCAN_MAX_ACTUAL_DEG * DEG_TO_RAD)
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

typedef struct {
  int8_t direction; /* +1上行，-1下行 */
} Pitch_Scan_Runtime_t;

volatile Yaw_Test_t g_yaw_test;
volatile Yaw_Test_t g_pitch_test;
volatile Pitch_Scan_Status_t g_pitch_scan;
volatile Gimbal_Repeat_t g_gimbal_repeat[DM_NUM];
volatile uint8_t g_gimbal_log_axis = Pitch;

static Gimbal_Axis_Runtime_t axes[DM_NUM];
static Pitch_Scan_Runtime_t pitch_scan_runtime;
static GPIO_PinState key_raw_last;
static GPIO_PinState key_stable;
static uint32_t key_changed_ms;
static uint32_t key_pressed_ms;
static uint8_t key_long_handled;

/* New load compensation is opt-in, independent per axis. */
LoadCompState g_load_comp[DM_NUM];
const LoadCompConfig g_load_config[DM_NUM] = {
  [Yaw] = {40.0f,.40f,1.0f,1.0f,1.0f,.0174533f,.0523599f,.00174533f,300U},
  [Pitch] = {1.0f,.15f,.15f,10.0f,1.0f,.0174533f,.0523599f,.00174533f,300U}
};
static uint8_t yaw_friction_enabled;
static YawFrictionGoal yaw_friction_goal;
/* Bounded continuous error correction; torque is converted to position only
 * at the MIT output. No accumulated correction to the real/planned target. */
static uint8_t yaw_boost_enabled;
static float yaw_boost_nm, yaw_boost_target;
#define YAW_BOOST_MAX_NM .30f
#define YAW_BOOST_MAX_RAD (1.0f * DEG_TO_RAD)
static uint8_t yaw_cal_saved_ff;
static float yaw_cal_saved_speed;
static const char *yaw_cal_start_reason="none";
/* Separate the legacy physical base from the new total sent torque. */
static float pitch_physical_ff_nm;
static uint8_t pitch_base_valid;
static YawCalStatus report_snapshot;
static uint8_t report_pending, report_line;
static uint8_t yaw_probe_pending;
static uint8_t pitch_probe_pending;
static uint32_t yaw_probe_id;
static char report_command[12] = "status";
static uint8_t report_command_ok = 1U;
static char fault_notice[256];
static uint8_t fault_notice_active, fault_notice_sent, cal_abort_reported;
static uint32_t fault_notice_first_ms, fault_notice_last_ms;
static void fault_notice_begin(const char *axis, const char *source, unsigned code, const char *reason) {
  snprintf(fault_notice,sizeof(fault_notice),
      "!!! FAULT axis=%s source=%s code=%u reason=%s; STOPPED; telemetry_pause=5s\r\n",
      axis,source,code,reason);
  fault_notice_active=1; fault_notice_sent=0;
  report_pending=0;
}
uint8_t Gimbal_Load_Telemetry_Paused(void) {
  if (fault_notice_active && fault_notice_sent &&
      (uint32_t)(HAL_GetTick()-fault_notice_first_ms)>=5000U) fault_notice_active=0;
  return fault_notice_active;
}
static void load_reset(Gimbal_Axis_Runtime_t *axis) {
  LoadComp_Reset(&g_load_comp[axis->feedback_id]);
  if (axis->feedback_id == Yaw) {
    YawDiag_MarkReset();
    YawFrictionGoal_Reset(&yaw_friction_goal);
    yaw_boost_nm=0;
    yaw_boost_target=axis->status->target_relative_rad;
    /* Reset dynamic compensation, preserve the selected feedforward model. */
    YawCal_Stop(YCAL_CANCEL);
  }
}

static void pitch_scan_update(uint32_t now_ms);
static void pitch_scan_abort(Pitch_Scan_Result_e result, const char *reason);

static float clampf(float value, float min_value, float max_value) {
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

static float yaw_position_boost(float kp, float base_motor_rad, uint32_t dt_ms,
                                uint8_t holding) {
  const float target=g_yaw_test.target_relative_rad;
  const float error=g_yaw_test.command_relative_rad-g_yaw_test.measured_relative_rad;
  /* MIT velocity: [-30,30] rad/s, 12 bits. Codes 2047/2048 decode
   * to +/-30/4095 rad/s (~0.420 deg/s), not zero. Only normalize these
   * nearest-zero codes for boost gating/taper; keep raw feedback untouched. */
  const float raw_rate=g_yaw_test.measured_velocity_rad_s;
  const float rate=fabsf(raw_rate)<=(30.0f/4095.0f+1e-6f) ? 0.0f : raw_rate;
  if (target!=yaw_boost_target) {
    yaw_boost_target=target; yaw_boost_nm=0;
  }
  if (!yaw_boost_enabled || !holding || YawCal_Active() ||
      g_load_comp[Yaw].integral_enabled || !isfinite(error) || !isfinite(rate) ||
      !isfinite(kp) || kp<=0 || !isfinite(base_motor_rad) ||
      fabsf(base_motor_rad)>12.5f || dt_ms==0 || dt_ms>20U) {
    yaw_boost_nm=0;
    return 0;
  }
  const float direction=error>=0 ? 1.0f : -1.0f;
  const float abs_error=fabsf(error), speed=fabsf(rate);
  /* Project 100 ms of motion toward the target to start withdrawing early. */
  const float remaining=abs_error-fmaxf(0,direction*rate)*.10f;
  float goal=0;
  /* Speed reduces assistance temporarily; residual error can reapply it. */
  if (abs_error<=3.0f*DEG_TO_RAD &&
      (yaw_boost_nm!=0 || (abs_error>.15f*DEG_TO_RAD && speed<=.30f*DEG_TO_RAD))) {
    goal=direction*YAW_BOOST_MAX_NM *
      clampf((remaining-.10f*DEG_TO_RAD)/(.30f*DEG_TO_RAD),0,1) *
      clampf(1.0f-speed/(1.0f*DEG_TO_RAD),0,1);
  }
  const float step=(goal==0 ? 6.0f : 3.0f)*dt_ms*.001f;
  yaw_boost_nm+=clampf(goal-yaw_boost_nm,-step,step);
  /* Never continue pushing past the target; ordinary withdrawal is slewed. */
  if (yaw_boost_nm*error<=0) yaw_boost_nm=0;
  const float limit=fminf(YAW_BOOST_MAX_NM,kp*YAW_BOOST_MAX_RAD);
  yaw_boost_nm=clampf(yaw_boost_nm,-limit,limit);
  const float position_boost=yaw_boost_nm/kp;
  const float sent=clampf(base_motor_rad+GIMBAL_YAW_DIRECTION*position_boost,-12.5f,12.5f);
  return GIMBAL_YAW_DIRECTION*(sent-base_motor_rad);
}

float Gimbal_Yaw_Control_Angle_Rad(void) {
  return g_yaw_inertial.mode == YAW_MODE_INERTIAL ?
      g_yaw_inertial.relative_rad : g_yaw_test.measured_relative_rad;
}
float Gimbal_Yaw_Control_Rate_Rad_S(void) {
  return g_yaw_inertial.mode == YAW_MODE_INERTIAL ?
      g_yaw_inertial.rate_rad_s : g_yaw_test.measured_velocity_rad_s;
}
static float axis_control_angle(Gimbal_Axis_Runtime_t *axis) {
  return axis->feedback_id == Yaw ? Gimbal_Yaw_Control_Angle_Rad() :
      axis->status->measured_relative_rad;
}

static void axis_repeat_cancel(Gimbal_Axis_Runtime_t *axis) {
  if (!g_gimbal_repeat[axis->feedback_id].active) return;
  g_gimbal_repeat[axis->feedback_id].active = 0U;
  // 取消只停命令推进，保留位置误差和前馈支撑。
  axis->status->target_relative_rad = axis->status->command_relative_rad;
}

static void axis_repeat_update(Gimbal_Axis_Runtime_t *axis) {
  volatile Gimbal_Repeat_t *test = &g_gimbal_repeat[axis->feedback_id];
  if (!test->active) return;
  const float actual_deg = axis_control_angle(axis) * RAD_TO_DEG;
  if (test->positioning) {
    if ((test->direction > 0 && actual_deg >= test->min_deg) ||
        (test->direction < 0 && actual_deg <= test->min_deg)) {
      test->positioning = 0U;
      test->direction = 1;
    }
  } else if (test->direction > 0 && actual_deg >= test->max_deg) {
    test->direction = -1;
  } else if (test->direction < 0 && actual_deg <= test->min_deg) {
    test->direction = 1;
  }
  // 按实际端点换向，只改方向，不重置命令位置。
}

static void axis_disable(Gimbal_Axis_Runtime_t *axis) {
  (void)dm4310_disable(GIMBAL_CAN_HANDLE, axis->motor_id);
}

static void axis_update_measurement(Gimbal_Axis_Runtime_t *axis,
                                    const volatile DM_Motor_Data_t *motor) {
  volatile Yaw_Test_t *status = axis->status;

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

static void axis_capture_boot_zero(Gimbal_Axis_Runtime_t *axis,
                                    const volatile DM_Motor_Data_t *motor) {
  volatile Yaw_Test_t *status = axis->status;

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
  if (axis->feedback_id==Yaw) YawDiag_Stop(YD_STOP_FAULT);
  if (axis->feedback_id==Yaw && YawCal_Active()) {
    YawCal_Stop(fault==YAW_FAULT_FEEDBACK_TIMEOUT?YCAL_FEEDBACK:YCAL_MODE);
    Gimbal_Load_Report();
  }
  load_reset(axis);
  if (axis->feedback_id == Yaw) g_yaw_inertial.mode = YAW_MODE_ENCODER;
  axis_repeat_cancel(axis);
  axis_disable(axis);
  axis->last_disable_ms = HAL_GetTick();
  static const char *const reasons[]={"none","can_init","enable_timeout","feedback_lost",
      "motor_error","over_temperature","can_tx","imu_invalid"};
  fault_notice_begin(axis->feedback_id==Yaw?"Yaw":"Pitch","motor",fault,
      (unsigned)fault<sizeof(reasons)/sizeof(reasons[0])?reasons[fault]:"unknown");
  axis->status->fault = fault;
  axis->status->state = YAW_TEST_FAULT;
  axis->status->target_relative_rad = axis->status->measured_relative_rad;
  axis->status->command_relative_rad = axis->status->measured_relative_rad;
  axis->status->applied_t_ff_nm = 0.0f;
  if (axis->feedback_id == Pitch) { Pitch_Inertial_Reset(); Pitch_FF_Reset(); pitch_physical_ff_nm=0; pitch_base_valid=0; }
}

static void axis_stop(Gimbal_Axis_Runtime_t *axis) {
  if (axis->feedback_id==Yaw) YawDiag_Stop(YD_STOP_AXIS);
  load_reset(axis);
  if (axis->feedback_id == Yaw) g_yaw_inertial.mode = YAW_MODE_ENCODER;
  axis_repeat_cancel(axis);
  axis_disable(axis);
  axis->status->state = YAW_TEST_DISABLED;
  axis->status->fault = YAW_FAULT_NONE;
  axis->status->target_relative_rad = axis->status->measured_relative_rad;
  axis->status->command_relative_rad = axis->status->measured_relative_rad;
  axis->status->applied_t_ff_nm = 0.0f;
  if (axis->feedback_id == Pitch) { Pitch_Inertial_Reset(); Pitch_FF_Reset(); pitch_physical_ff_nm=0; pitch_base_valid=0; }
}

static void axis_begin_enable(Gimbal_Axis_Runtime_t *axis, uint32_t now_ms) {
  load_reset(axis);
  if (axis->feedback_id == Yaw) g_yaw_inertial.mode = YAW_MODE_ENCODER;
  axis_repeat_cancel(axis);
  axis->status->applied_t_ff_nm = 0.0f;
  if (axis->feedback_id == Pitch) { Pitch_Inertial_Reset(); Pitch_FF_Reset(); pitch_physical_ff_nm=0; pitch_base_valid=0; }
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
  DM_Motor_Data_t pitch_feedback;
  { /* 两轴都取完整CAN快照，避免中断更新到一半。 */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    pitch_feedback = dm_motor[axis->feedback_id];
    if (!primask) __enable_irq();
    motor = &pitch_feedback;
  }
  // Yaw age must refer to the same captured frame; sample time after the copy.
  const uint32_t feedback_now=axis->feedback_id==Yaw?HAL_GetTick():now_ms;
  const uint8_t feedback_valid=motor->rx_count &&
      (uint32_t)(feedback_now-motor->last_rx_ms)<=DM_FEEDBACK_TIMEOUT_MS;


  /* 零点已建立后，即使停用也更新新反馈；无新帧时保留旧值并由 valid 标记。 */
  if (axis->boot_zero_valid &&
      feedback_valid) {
    axis_update_measurement(axis, motor);
  }

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
    if (feedback_valid &&
        ((motor->rx_count - axis->rx_count_at_enable) >= 1U)) {
      if (!axis->boot_zero_valid) {
        axis_capture_boot_zero(axis, motor);
      } else {
        axis_update_measurement(axis, motor);
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
    if (!feedback_valid) {
      axis_enter_fault(axis, YAW_FAULT_FEEDBACK_TIMEOUT);
      return;
    }
    axis_update_measurement(axis, motor);
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

  if (!feedback_valid) {
    axis_enter_fault(axis, YAW_FAULT_FEEDBACK_TIMEOUT);
    return;
  }
  if (axis->feedback_id == Yaw && g_yaw_inertial.mode == YAW_MODE_INERTIAL &&
      !g_yaw_inertial.valid) {
    axis_enter_fault(axis, YAW_FAULT_IMU);
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

  axis_update_measurement(axis, motor);


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
  if (axis->feedback_id == Pitch) {
    Pitch_Cont_Sample(motor->rx_count, motor->last_rx_ms,
        status->measured_relative_rad * RAD_TO_DEG,
        status->measured_velocity_rad_s * RAD_TO_DEG, axis->direction * motor->t_int);
  }
  if (axis->feedback_id == Yaw && YawCal_Active()) {
    if (g_yaw_inertial.mode != YAW_MODE_ENCODER) {
      YawCal_Stop(YCAL_MODE); axis_stop(axis); Gimbal_Load_Report(); return;
    }
    const YawCalInput input = { now_ms, motor->last_rx_ms, motor->rx_count,
      status->measured_relative_rad * RAD_TO_DEG,
      status->measured_velocity_rad_s * RAD_TO_DEG, axis->direction * motor->t_int,
      status->command_relative_rad * RAD_TO_DEG, feedback_valid };
    const int result=YawCal_Update(&input);
    if (result<0) {
      if (g_yaw_cal.error==YCAL_SAMPLES || g_yaw_cal.error==YCAL_SIGN ||
          g_yaw_cal.error==YCAL_REPEATABILITY) {
        // A rejected trial is not a hardware fault: hold here and retain the committed model.
        status->target_relative_rad=status->command_relative_rad=status->measured_relative_rad;
        status->max_speed_rad_s=yaw_cal_saved_speed;
        LoadComp_Reset(&g_load_comp[Yaw]);
        yaw_friction_enabled=yaw_cal_saved_ff;
      } else { axis_stop(axis); }
      snprintf(report_command,sizeof(report_command),"ycal"); report_command_ok=1;
      Gimbal_Load_Report();
      if (status->state!=YAW_TEST_ACTIVE) return;
    } else {
      status->target_relative_rad=g_yaw_cal.target_deg*DEG_TO_RAD;
      status->max_speed_rad_s=g_yaw_cal.speed_deg_s*DEG_TO_RAD;
      if (g_yaw_cal.state==YCAL_DONE) {
        yaw_friction_enabled=1; status->max_speed_rad_s=yaw_cal_saved_speed;
        snprintf(report_command,sizeof(report_command),"ycal"); report_command_ok=1;
        Gimbal_Load_Report();
      }
    }
  }
  if ((now_ms - axis->last_control_ms) < AXIS_CONTROL_PERIOD_MS) return;
  ///////////////////////////////////////////////



  if (axis->feedback_id == Pitch) pitch_scan_update(now_ms);
  axis_repeat_update(axis);

  const uint32_t elapsed_ms = now_ms - axis->last_control_ms;
  axis->last_control_ms = now_ms;

  const uint8_t diag_active=axis->feedback_id==Yaw &&
      g_yaw_inertial.mode==YAW_MODE_ENCODER && YawDiag_Recording();
  YawDiagFrame diag={0};
  if (diag_active) {
    diag.drive=NAN; diag.brake_rad=NAN; diag.arrived=255;
    if (elapsed_ms==0 || elapsed_ms>20U) diag.flags|=YD_BAD_DT;
  }
  if (axis->feedback_id==Pitch && g_pitch_inertial.mode==PITCH_MODE_INERTIAL) {
    float corrected;
    if (Pitch_Inertial_Correct(HAL_GetTick(),status->measured_relative_rad,
          axis->min_target_rad,axis->max_target_rad,&corrected)) {
      status->target_relative_rad=corrected;
    } else {
      /* Discard stale IMU correction; retain encoder PD and gravity support.
       * Capture current position, require explicit operator re-entry. */
      Pitch_Inertial_Reset();
      status->target_relative_rad=status->command_relative_rad=clampf(
          status->measured_relative_rad,axis->min_target_rad,axis->max_target_rad);
      pitch_probe_pending=1;
    }
  }
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

  const float previous_command_rad = status->command_relative_rad;
  float yaw_brake_distance=0;
  uint8_t yaw_arrived=0;
  const uint8_t yaw_dynamic=axis->feedback_id==Yaw &&
      g_yaw_inertial.mode==YAW_MODE_ENCODER && !YawCal_Active() &&
      !g_gimbal_repeat[Yaw].active && !yaw_boost_enabled &&
      !g_load_comp[Yaw].integral_enabled;
  if (yaw_dynamic) {
    const float dt=elapsed_ms*.001f;
    const float actual=status->measured_relative_rad;
    const float error=status->target_relative_rad-actual;
    const float half_velocity_code=30.0f/4095.0f;
    const float raw_rate=status->measured_velocity_rad_s;
    const float rate=fabsf(raw_rate)<=half_velocity_code+1e-6f ? 0 : raw_rate;
    const float direction=error>=0 ? 1.0f : -1.0f;
    const float distance=fabsf(error);
    const float toward=fmaxf(0,direction*rate);
    /* Outside the zero bin, include half a velocity code in brake prediction. */
    const float brake_speed=toward>0 ? toward+half_velocity_code : 0;
    const float brake_distance=brake_speed*.010f+
        brake_speed*brake_speed/(2.0f*YAW_REFERENCE_BRAKE_RAD_S2);
    const uint8_t arrived=distance<=.10f*DEG_TO_RAD && brake_distance<=.05f*DEG_TO_RAD;
    yaw_brake_distance=brake_distance; yaw_arrived=arrived;
    const float travel=fmaxf(0,distance-.10f*DEG_TO_RAD);
    const float drive=arrived ? 0 : clampf((travel-brake_distance)/(.30f*DEG_TO_RAD),0,1);
    const float lead=status->kp>0 ? fminf(1.0f*DEG_TO_RAD,YAW_REFERENCE_EXTRA_NM/status->kp) : 0;
    float reference=status->target_relative_rad+direction*lead*drive;
    if (diag_active && fabsf(reference-actual)>2.0f*DEG_TO_RAD)
      diag.flags|=YD_REFERENCE_CLIP;
    reference=clampf(reference,actual-2.0f*DEG_TO_RAD,actual+2.0f*DEG_TO_RAD);
    /* Only command is advanced; target always remains the user's accumulated goal. */
    if (diag_active && fabsf(reference-previous_command_rad)>max_step)
      diag.flags|=YD_SPEED_CLIP;
    float next=previous_command_rad+clampf(reference-previous_command_rad,-max_step,max_step);
    if (diag_active && fabsf(next-actual)>2.0f*DEG_TO_RAD) diag.flags|=YD_LEAD_CLIP;
    next=clampf(next,actual-2.0f*DEG_TO_RAD,actual+2.0f*DEG_TO_RAD);
    /* Do not wrap/saturate a reference past the MIT raw-position boundary. */
    float raw_next=motor->p_int+axis->direction*(next-actual);
    if (diag_active && fabsf(raw_next)>12.4f) diag.flags|=YD_RAW_CLIP;
    raw_next=clampf(raw_next,-12.4f,12.4f);
    next=actual+axis->direction*(raw_next-motor->p_int);
    command_step=next-previous_command_rad;
    target_velocity_rad_s=direction*fminf(status->max_speed_rad_s,
        sqrtf(2.0f*YAW_REFERENCE_BRAKE_RAD_S2*travel))*drive;
    if (arrived || dt<=0 || dt>.020f ||
        (raw_next>=12.4f && axis->direction*target_velocity_rad_s>0) ||
        (raw_next<=-12.4f && axis->direction*target_velocity_rad_s<0))
      target_velocity_rad_s=0;
    if (diag_active) {
      diag.flags|=YD_DYNAMIC; diag.drive=drive;
      diag.brake_rad=brake_distance; diag.arrived=arrived;
    }
  }
  if ((axis->feedback_id == Pitch) && g_pitch_scan.active) {
    const int8_t direction = pitch_scan_runtime.direction;
    // 只按设定速度连续推进；不限制领先量，不因停滞或力矩终止。
    command_step = direction * max_step;
    target_velocity_rad_s = direction * status->max_speed_rad_s;
    status->target_relative_rad = previous_command_rad + command_step;
  }

  if (g_gimbal_repeat[axis->feedback_id].active) {
    const int8_t direction = g_gimbal_repeat[axis->feedback_id].direction;
    command_step = direction * max_step;
    target_velocity_rad_s = direction * status->max_speed_rad_s;
    status->target_relative_rad = previous_command_rad + command_step;
  }

  if (axis->feedback_id == Pitch) {
    if (command_step > 0.0f) {
      pitch_approach_sign  = 1 ;
    }
    else if (command_step < 0.0f) {
      pitch_approach_sign = -1 ;
    }
  }




  status->command_relative_rad += command_step;
  if (axis->feedback_id==Pitch && g_pitch_inertial.mode==PITCH_MODE_INERTIAL) {
    status->command_relative_rad=clampf(status->command_relative_rad,
        axis->min_target_rad,axis->max_target_rad);
    command_step=status->command_relative_rad-previous_command_rad;
    if ((status->command_relative_rad>=axis->max_target_rad && target_velocity_rad_s>0) ||
        (status->command_relative_rad<=axis->min_target_rad && target_velocity_rad_s<0))
      target_velocity_rad_s=0;
  }
  if (diag_active) diag.command_pre_rad=status->command_relative_rad;

  /* Close the session coordinate loop from the latest measured raw position. */
  float motor_target_rad = motor->p_int + axis->direction *
      (status->command_relative_rad - status->measured_relative_rad);

  if (axis->feedback_id==Yaw && YawCal_Active() &&
      (!isfinite(motor_target_rad) || fabsf(motor_target_rad)>12.4f)) {
    YawCal_Stop(YCAL_BOUNDS); axis_stop(axis); Gimbal_Load_Report(); return;
  }
  float t_ff_nm = 0.0f;
  if (axis->feedback_id == Pitch) {
    /* 连续校准收尾保持当前前馈，不突然抽走支撑；新运动时继续查表。 */
    const float logical_t_ff_nm = (g_pitch_cont.selected && pitch_base_valid && command_step == 0.0f &&
        Pitch_FF_Get_Scale() > 0.0f) ? axis->direction * pitch_physical_ff_nm : Pitch_FF_Update(
        status->measured_relative_rad * RAD_TO_DEG,
        pitch_approach_sign,
        elapsed_ms);

    t_ff_nm = axis->direction * logical_t_ff_nm;
    pitch_physical_ff_nm = t_ff_nm;
    pitch_base_valid=Pitch_FF_Get_Scale()>0;
  }


  const uint8_t is_pitch=axis->feedback_id==Pitch;
  LoadCompState *load=&g_load_comp[axis->feedback_id];
  const uint8_t load_allowed=is_pitch ? !g_pitch_scan.active && !Pitch_Cal_Is_Active() :
      g_yaw_inertial.mode==YAW_MODE_ENCODER && !YawCal_Active();
  // Off by default: leave legacy Pitch bit-for-bit on its original path.
  if (load->integral_enabled || load->integral_nm!=0 || load->friction_nm!=0 ||
      (!is_pitch && yaw_friction_enabled)) {
    const float desired_rate=yaw_dynamic ? target_velocity_rad_s :
        command_step/((float)elapsed_ms*.001f);
    float friction=0;
    if (!is_pitch && yaw_friction_enabled) {
      // Constant effective F; identified at scan speed, without a hidden speed/angle switch.
      const float f=YawCal_Friction(status->measured_relative_rad*RAD_TO_DEG);
      if (yaw_dynamic) {
        friction=YawFrictionGoal_Update(&yaw_friction_goal,
            status->target_relative_rad-status->measured_relative_rad,
            status->measured_velocity_rad_s,yaw_brake_distance,yaw_arrived,f);
      } else {
        YawFrictionGoal_Reset(&yaw_friction_goal);
        friction=f*clampf(desired_rate/(1.0f*DEG_TO_RAD),-1,1);
      }
    }
    if (diag_active) {
      diag.friction_goal=friction;
      diag.friction_before=load->friction_nm;
    }
    /* The goal is now independent of drive magnitude. Only explicit braking,
     * arrival or reversal may withdraw its stored tail; original rise slew stays. */
    if (yaw_dynamic) {
      if (load->friction_nm*friction<=0) load->friction_nm=0;
      else if (fabsf(load->friction_nm)>fabsf(friction)) load->friction_nm=friction;
    }
    if (diag_active && fabsf(load->friction_nm)<fabsf(diag.friction_before))
      diag.flags|=YD_FRICTION_CUT;
    const uint8_t holding=fabsf(status->target_relative_rad-status->command_relative_rad)<.0001f &&
                          !g_gimbal_repeat[axis->feedback_id].active;
    const float error=status->command_relative_rad-status->measured_relative_rad;
    const float pd=status->kp*error+status->kd*(target_velocity_rad_s-status->measured_velocity_rad_s);
    t_ff_nm=axis->direction*LoadComp_Update(load,&g_load_config[axis->feedback_id],
       axis->direction*t_ff_nm,friction,error,status->measured_velocity_rad_s,
       desired_rate,pd,elapsed_ms,holding,load_allowed);
  }
  if (yaw_dynamic && status->kp>0) {
    /* Bound estimated MIT PD + FF by projecting command onto the torque interval.
     * No change to Kp/Kd or t_ff; motor feedback/driver timing remains external. */
    const float other=status->kd*(target_velocity_rad_s-status->measured_velocity_rad_s)+
        axis->direction*t_ff_nm;
    const float error=status->command_relative_rad-status->measured_relative_rad;
    const float raw_lo=axis->direction*(-12.4f-motor->p_int);
    const float raw_hi=axis->direction*(12.4f-motor->p_int);
    const float lo=fmaxf(fmaxf(-2.0f*DEG_TO_RAD,fminf(raw_lo,raw_hi)),
        (-YAW_REFERENCE_TOTAL_NM-other)/status->kp);
    const float hi=fminf(fminf(2.0f*DEG_TO_RAD,fmaxf(raw_lo,raw_hi)),
        (YAW_REFERENCE_TOTAL_NM-other)/status->kp);
    /* If the existing PD cannot satisfy both bounds, use the existing disable. */
    if (!isfinite(lo) || !isfinite(hi) || lo>hi) { axis_stop(axis); return; }
    const float bounded=clampf(error,lo,hi);
    if (diag_active && bounded!=error) diag.flags|=YD_TORQUE_CLIP;
    status->command_relative_rad=status->measured_relative_rad+bounded;
    motor_target_rad=motor->p_int+axis->direction*bounded;
  }
  float send_kp = status->kp, send_kd = status->kd;
  if (axis->feedback_id == Yaw && g_yaw_inertial.mode == YAW_MODE_INERTIAL) {
    // 外部惯性PD输出力矩；驱动内置PD置零，编码器不再产生位置恢复力。
    const float torque_nm = status->kp *
        (status->command_relative_rad - g_yaw_inertial.relative_rad) +
        status->kd * (target_velocity_rad_s - g_yaw_inertial.rate_rad_s);
    if (!isfinite(torque_nm)) { axis_enter_fault(axis, YAW_FAULT_IMU); return; }
    t_ff_nm = clampf(axis->direction * torque_nm,
                    -YAW_MIT_TORQUE_MAX_NM, YAW_MIT_TORQUE_MAX_NM);
    motor_target_rad = 0.0f;
    target_velocity_rad_s = 0.0f;
    send_kp = send_kd = 0.0f;
  }
  if (axis->feedback_id==Yaw) {
    if (g_yaw_inertial.mode==YAW_MODE_ENCODER) {
      const uint8_t holding=command_step==0 &&
          fabsf(status->target_relative_rad-status->command_relative_rad)<.0001f &&
          !g_gimbal_repeat[Yaw].active;
      motor_target_rad+=axis->direction*yaw_position_boost(send_kp,motor_target_rad,elapsed_ms,holding);
    } else yaw_boost_nm=0;
  }
  status->applied_t_ff_nm = t_ff_nm;
  if (diag_active) {
    diag.time_ms=now_ms; diag.dt_ms=elapsed_ms;
    diag.rx_count=motor->rx_count;
    diag.feedback_age_ms=feedback_now-motor->last_rx_ms;
    diag.target_rad=status->target_relative_rad;
    diag.encoder_rad=status->measured_relative_rad;
    diag.rate_rad_s=status->measured_velocity_rad_s;
    diag.raw_position_rad=motor->p_int; diag.motor_rate_rad_s=motor->v_int;
    diag.command_rad=status->command_relative_rad;
    diag.p_des=motor_target_rad; diag.v_des=axis->direction*target_velocity_rad_s;
    diag.kp=send_kp; diag.kd=send_kd; diag.t_ff=t_ff_nm;
    diag.friction_actual=load->friction_nm; diag.feedback_torque=motor->t_int;
    diag.mode=(uint8_t)g_yaw_inertial.mode; diag.axis_state=(uint8_t)status->state;
    diag.ff_on=yaw_friction_enabled; diag.integral_on=load->integral_enabled;
    diag.boost_on=yaw_boost_enabled;
    if (yaw_friction_goal.correcting) diag.flags|=YD_FF_CORRECT;
    if (yaw_friction_goal.braking) diag.flags|=YD_FF_BRAKING;
    YawDiag_Prepare(&diag);
  }

  ///////////////////////////////保险
  if (dm4310_send_mit(GIMBAL_CAN_HANDLE, axis->motor_id,
                      motor_target_rad,
                      axis->direction * target_velocity_rad_s,
                      send_kp, send_kd, t_ff_nm) != HAL_OK) {
    axis_enter_fault(axis, YAW_FAULT_TX);
    return;
  }
  ///////////////////////////////保险


  status->tx_count++;
}





static uint8_t axis_set_max_speed(Gimbal_Axis_Runtime_t *axis,
                                  float max_speed_rad_s) {
  if (axis->feedback_id==Pitch && g_pitch_inertial.mode==PITCH_MODE_INERTIAL) return 0;
  if (axis->feedback_id==Yaw && YawCal_Active()) return 0U;
  if ((max_speed_rad_s < AXIS_MIN_SPEED_RAD_S) ||
      (max_speed_rad_s > AXIS_MAX_SPEED_RAD_S)) {
    return 0U;
  }
  axis_repeat_cancel(axis);
  axis->status->max_speed_rad_s = max_speed_rad_s;
  return 1U;
}

static uint8_t axis_set_target(Gimbal_Axis_Runtime_t *axis,
                               float target_rad) {
  if (axis->feedback_id==Pitch && g_pitch_inertial.mode==PITCH_MODE_INERTIAL) return 0;
  if (axis->feedback_id==Yaw && YawCal_Active()) return 0U;
  if (axis->feedback_id==Yaw && YawCal_Active()) return 0U;
  if (axis->status->state != YAW_TEST_ACTIVE) return 0U;
  if (!isfinite(target_rad)) return 0U;
  axis_repeat_cancel(axis);
  if (axis->target_limit_enabled) {
    target_rad = clampf(target_rad,
                        axis->min_target_rad,
                        axis->max_target_rad);
  }
  if (target_rad!=axis->status->target_relative_rad) {
    LoadComp_Reset(&g_load_comp[axis->feedback_id]);
    if (axis->feedback_id==Yaw) {
      YawDiag_MarkReset();
      YawFrictionGoal_Reset(&yaw_friction_goal);
    }
  }
  axis->status->target_relative_rad = target_rad;
  if (axis->feedback_id==Yaw && g_yaw_inertial.mode==YAW_MODE_ENCODER)
    YawDiag_TargetAccepted(HAL_GetTick());
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
  if (axis->feedback_id==Pitch && g_pitch_inertial.mode==PITCH_MODE_INERTIAL) return 0;
  if (axis->feedback_id==Yaw && YawCal_Active()) return 0U;
  if (axis->status->state != YAW_TEST_ACTIVE) return 0U;
  if (!isfinite(delta_rad)) return 0U;
  axis_repeat_cancel(axis);
  /* Accumulate commanded steps, including travel not yet completed. */
  float target_rad =
      axis->status->target_relative_rad + delta_rad;
  if (!isfinite(target_rad)) return 0U;
  if (axis->feedback_id == Pitch) target_rad = fminf(target_rad, PITCH_MAX_TARGET_RAD);
  if (target_rad!=axis->status->target_relative_rad) {
    LoadComp_Reset(&g_load_comp[axis->feedback_id]);
    if (axis->feedback_id==Yaw) {
      YawDiag_MarkReset();
      YawFrictionGoal_Reset(&yaw_friction_goal);
    }
  }
  axis->status->target_relative_rad = target_rad;
  if (axis->feedback_id==Yaw && g_yaw_inertial.mode==YAW_MODE_ENCODER)
    YawDiag_TargetAccepted(HAL_GetTick());
  return 1U;
}

static uint8_t axis_move_relative_speed(Gimbal_Axis_Runtime_t *axis,
                                        float delta_rad,
                                        float max_speed_rad_s) {
  if (!axis_set_max_speed(axis, max_speed_rad_s)) return 0U;
  return axis_move_relative(axis, delta_rad);
}

static uint8_t axis_set_kp(Gimbal_Axis_Runtime_t *axis, float kp) {
  if (axis->feedback_id==Yaw && YawCal_Active()) return 0U;
  if ((kp < 0.0f) ) return 0U;
    // || (kp > 20.0f)) return 0U;
  axis_repeat_cancel(axis);
  axis->status->kp = kp;
  return 1U;
}

static uint8_t axis_set_kd(Gimbal_Axis_Runtime_t *axis, float kd) {
  if (axis->feedback_id==Yaw && YawCal_Active()) return 0U;
  if ((kd < 0.0f) || (kd > 3.0f)) return 0U;
  axis_repeat_cancel(axis);
  axis->status->kd = kd;
  return 1U;
}

static const char *pitch_scan_phase_name(Pitch_Scan_Phase_e phase) {
  switch (phase) {
    case PITCH_SCAN_POSITION_START: return "就位";
    case PITCH_SCAN_TURN_UP_5: return "TURN_UP_5";
    case PITCH_SCAN_UP_5: return "5度/秒上行";
    case PITCH_SCAN_TURN_DOWN_5: return "TURN_DOWN_5";
    case PITCH_SCAN_DOWN_5: return "5度/秒下行";
    case PITCH_SCAN_TURN_UP_20: return "TURN_UP_20";
    case PITCH_SCAN_UP_20: return "20度/秒上行";
    case PITCH_SCAN_TURN_DOWN_20: return "TURN_DOWN_20";
    case PITCH_SCAN_DOWN_20: return "20度/秒下行";
    default: return "空闲";
  }
}

static void pitch_scan_hold(void) {
  /* 停止命令推进，保留 P 项支撑；下一帧速度目标为 0，由原 kd 减速。 */
  g_pitch_test.target_relative_rad = g_pitch_test.command_relative_rad;

}

static void pitch_scan_finish(Pitch_Scan_Result_e result,
                              Pitch_Scan_Phase_e final_phase,
                              const char *reason) {
  const uint32_t now_ms = HAL_GetTick();
  const float actual_deg = g_pitch_test.measured_relative_rad * RAD_TO_DEG;
  const uint8_t can_hold =
      (g_pitch_test.state == YAW_TEST_ACTIVE) &&
      DM_Motor_Feedback_Valid(Pitch, now_ms);

  if (can_hold) pitch_scan_hold();
  if (Pitch_Cont_Active()) pitch_base_valid=0;
  Pitch_Cont_Stop(result == PITCH_SCAN_RESULT_COMPLETE);
  g_pitch_scan.active = 0U;
  g_pitch_scan.feedback_valid = DM_Motor_Feedback_Valid(Pitch, now_ms);
  g_pitch_scan.holding = can_hold;
  g_pitch_scan.result = result;
  snprintf((char *)g_pitch_scan.end_summary,
           sizeof(g_pitch_scan.end_summary),
           "%s：%s，实测%.2f度；前馈%s；%s",
           pitch_scan_phase_name(g_pitch_scan.phase), reason, actual_deg,
           Pitch_FF_Get_Scale() > 0.0f ? "开" : "关",
           can_hold ? "保持" : "停用/故障，无法主动支撑");
  g_pitch_scan.phase = final_phase;
}

static void pitch_scan_abort(Pitch_Scan_Result_e result, const char *reason) {
  if (!g_pitch_scan.active) return;
  const Pitch_Scan_Phase_e phase = g_pitch_scan.phase;
  snprintf((char *)g_pitch_scan.event_summary,
           sizeof(g_pitch_scan.event_summary),
           "%s，实测%.2f度：%s",
           pitch_scan_phase_name(phase),
           g_pitch_test.measured_relative_rad * RAD_TO_DEG,
           reason);
  g_pitch_scan.event_count++;
  pitch_scan_finish(result, PITCH_SCAN_ABORT_HOLD, reason);
}

static void pitch_scan_set_motion(Pitch_Scan_Phase_e phase,
                                  int8_t direction, float speed_deg_s,
                                  uint32_t now_ms) {
  // 换向只改推进方向，不重置 command，也不固定发送 56/-1 度。
  g_pitch_scan.phase = phase;
  pitch_scan_runtime.direction = direction;
  g_pitch_test.max_speed_rad_s = speed_deg_s * DEG_TO_RAD;
  (void)now_ms;
}

static void pitch_scan_update(uint32_t now_ms) {
  g_pitch_scan.feedback_valid = DM_Motor_Feedback_Valid(Pitch, now_ms);
  if (!g_pitch_scan.active) return;

  if (!g_pitch_scan.feedback_valid ||
      (g_pitch_test.state != YAW_TEST_ACTIVE)) {
    pitch_scan_abort(PITCH_SCAN_RESULT_AXIS_FAULT,
                     "反馈失效或退出使能");
    return;
  }
  const float actual_deg = g_pitch_test.measured_relative_rad * RAD_TO_DEG;
  const int8_t direction = pitch_scan_runtime.direction;

  switch (g_pitch_scan.phase) {
    case PITCH_SCAN_POSITION_START:
      if ((direction > 0 && actual_deg >= PITCH_SCAN_MIN_ACTUAL_DEG) ||
          (direction < 0 && actual_deg <= PITCH_SCAN_MIN_ACTUAL_DEG))
        pitch_scan_set_motion(PITCH_SCAN_UP_5, 1, 5.0f, now_ms);
      break;
    case PITCH_SCAN_UP_5:
      // 实际经过配置上端才换向，先采到最后一个up节点。
      if (actual_deg >= PITCH_SCAN_MAX_ACTUAL_DEG)
        pitch_scan_set_motion(PITCH_SCAN_DOWN_5, -1, 5.0f, now_ms);
      break;
    case PITCH_SCAN_DOWN_5:
      if (actual_deg <= PITCH_SCAN_MIN_ACTUAL_DEG) {
        if (Pitch_Cont_Active()) {
          const int8_t cal_result=Pitch_Cont_Finish_Lap();
          pitch_physical_ff_nm=g_pitch_test.applied_t_ff_nm;
          pitch_base_valid=cal_result==0;
        }
        pitch_scan_finish(PITCH_SCAN_RESULT_COMPLETE, PITCH_SCAN_COMPLETE_HOLD,
                          "往返完成");
      }
      break;
    default:
      break;
  }
}

uint8_t Gimbal_Pitch_Scan_Start(void) {
  if (g_pitch_inertial.mode==PITCH_MODE_INERTIAL) return 0;
  const uint32_t now_ms = HAL_GetTick();
  const float actual_deg = g_pitch_test.measured_relative_rad * RAD_TO_DEG;

  if (g_pitch_scan.active) return 0U;
  memset((void *)&g_pitch_scan, 0, sizeof(g_pitch_scan));
  memset(&pitch_scan_runtime, 0, sizeof(pitch_scan_runtime));

  if ((g_pitch_test.state != YAW_TEST_ACTIVE) || Pitch_Cal_Is_Active()) {
    g_pitch_scan.result = PITCH_SCAN_RESULT_REJECT_AXIS;
    snprintf((char *)g_pitch_scan.end_summary,
             sizeof(g_pitch_scan.end_summary),
             "拒绝启动：Pitch未使能就绪，或校准正在运行");
    return 0U;
  }
  if (!DM_Motor_Feedback_Valid(Pitch, now_ms)) {
    g_pitch_scan.result = PITCH_SCAN_RESULT_REJECT_FEEDBACK;
    snprintf((char *)g_pitch_scan.end_summary,
             sizeof(g_pitch_scan.end_summary),
             "拒绝启动：Pitch反馈已过期");
    return 0U;
  }
  axis_repeat_cancel(&axes[Pitch]);
  g_gimbal_log_axis = Pitch;
  g_pitch_scan.active = 1U;
  g_pitch_scan.feedback_valid = 1U;
  g_pitch_scan.holding = 1U;
  g_pitch_scan.saved_kp = g_pitch_test.kp;
  g_pitch_scan.saved_kd = g_pitch_test.kd;
  g_pitch_scan.min_deg = PITCH_SCAN_MIN_ACTUAL_DEG;
  g_pitch_scan.max_deg = PITCH_SCAN_MAX_ACTUAL_DEG;
  snprintf((char *)g_pitch_scan.start_summary,
           sizeof(g_pitch_scan.start_summary),
           "pscan kp=%.2f kd=%.2f zero_raw=%.2fdeg scan=%.1f..%.1f FF=%.1f",
           g_pitch_scan.saved_kp, g_pitch_scan.saved_kd,
           g_pitch_test.origin_rad * RAD_TO_DEG,
           g_pitch_scan.min_deg, g_pitch_scan.max_deg, Pitch_FF_Get_Scale());
  snprintf((char *)g_pitch_scan.event_summary,
           sizeof(g_pitch_scan.event_summary), "暂无事件");
  snprintf((char *)g_pitch_scan.end_summary,
           sizeof(g_pitch_scan.end_summary), "扫描运行中，保持现有补偿开关");

  // 从下端直接上行；从高处重测时先以5度/秒下行到配置下端。
  g_pitch_test.target_relative_rad = g_pitch_test.command_relative_rad;
  pitch_scan_set_motion(PITCH_SCAN_POSITION_START,
      actual_deg <= PITCH_SCAN_MIN_ACTUAL_DEG ? 1 : -1, 5.0f, now_ms);
  return 1U;
}

void Gimbal_Pitch_Scan_Cancel(void) {
  axis_repeat_cancel(&axes[Pitch]);
  if (!g_pitch_scan.active) return;
  pitch_scan_finish(PITCH_SCAN_RESULT_CANCELLED,
                    PITCH_SCAN_CANCEL_HOLD,
                    "收到取消命令，停止推进");
}

static uint8_t axis_repeat_start(Gimbal_Axis_Runtime_t *axis,
                                 float min_deg, float max_deg, float speed_deg_s) {
  if (axis->feedback_id==Pitch && g_pitch_inertial.mode==PITCH_MODE_INERTIAL) return 0;
  if (axis->feedback_id==Yaw && YawCal_Active()) return 0U;
  if (axis->status->state != YAW_TEST_ACTIVE ||
      !DM_Motor_Feedback_Valid(axis->feedback_id, HAL_GetTick()) ||
      !isfinite(min_deg) || !isfinite(max_deg) || !isfinite(speed_deg_s) ||
      min_deg >= max_deg || speed_deg_s <= 0.0f) return 0U;
  if (axis->feedback_id == Pitch) {
    if (max_deg > PITCH_SCAN_MAX_ACTUAL_DEG) return 0U;
    if (Pitch_Cal_Is_Active()) return 0U;
    Gimbal_Pitch_Scan_Cancel();
  }
  volatile Gimbal_Repeat_t *test = &g_gimbal_repeat[axis->feedback_id];
  test->min_deg = min_deg;
  test->max_deg = max_deg;
  test->speed_deg_s = speed_deg_s;
  test->direction = axis_control_angle(axis) * RAD_TO_DEG <= min_deg ? 1 : -1;
  test->positioning = 1U;
  test->active = 1U;
  axis->status->max_speed_rad_s = speed_deg_s * DEG_TO_RAD;
  axis->status->target_relative_rad = axis->status->command_relative_rad;
  g_gimbal_log_axis = axis->feedback_id;
  return 1U;
}

uint8_t Gimbal_Pitch_Repeat_Start(float min_deg, float max_deg, float speed_deg_s) {
  return axis_repeat_start(&axes[Pitch], min_deg, max_deg, speed_deg_s);
}
uint8_t Gimbal_Yaw_Repeat_Start(float min_deg, float max_deg, float speed_deg_s) {
  return axis_repeat_start(&axes[Yaw], min_deg, max_deg, speed_deg_s);
}
void Gimbal_Yaw_Repeat_Cancel(void) { axis_repeat_cancel(&axes[Yaw]); }

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
  YawDiag_Init();
  yaw_probe_pending=0;
  YawFrictionGoal_Reset(&yaw_friction_goal);
  memset((void *)&g_yaw_test, 0, sizeof(g_yaw_test));
  memset((void *)&g_pitch_test, 0, sizeof(g_pitch_test));
  memset((void *)&g_pitch_scan, 0, sizeof(g_pitch_scan));
  memset(&pitch_scan_runtime, 0, sizeof(pitch_scan_runtime));
  memset(axes, 0, sizeof(axes));
  memset((void *)g_gimbal_repeat, 0, sizeof(g_gimbal_repeat));
  g_gimbal_log_axis = Pitch;
  Yaw_Inertial_Init();
  Pitch_Inertial_Init();
  pitch_probe_pending=0;
  YawCal_Init();
  Pitch_Cont_Init();
  memset(g_load_comp,0,sizeof(g_load_comp));
  yaw_boost_enabled=0; /* Dynamic reference trial: boost and integral stay off. */
  yaw_boost_nm=yaw_boost_target=0;
  yaw_friction_enabled=g_yaw_cal.ready; pitch_physical_ff_nm=0; pitch_base_valid=0; report_pending=0;
  fault_notice_active=fault_notice_sent=cal_abort_reported=0;

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
  const uint32_t reference_id = g_yaw_inertial.reference_id;
  Yaw_Inertial_Update(now_ms);
  if (g_yaw_inertial.mode == YAW_MODE_INERTIAL &&
      reference_id != g_yaw_inertial.reference_id)
    axis_enter_fault(&axes[Yaw], YAW_FAULT_IMU); // 重标定后禁止沿用旧目标。
  gimbal_update_key(now_ms, key_level);
  axis_update(&axes[Yaw], now_ms);
  axis_update(&axes[Pitch], now_ms);
  /* 同步故障退出；ACTIVE 扫描已在本帧发送前处理。 */
  g_pitch_scan.feedback_valid = DM_Motor_Feedback_Valid(Pitch, now_ms);
  if (g_pitch_scan.active && g_pitch_test.state != YAW_TEST_ACTIVE)
    pitch_scan_abort(PITCH_SCAN_RESULT_AXIS_FAULT, "轴故障，无法主动保持");
  if (g_pitch_test.state != YAW_TEST_ACTIVE) g_pitch_scan.holding = 0U;
}

void Gimbal_Yaw_Test_Stop(void) { axis_stop(&axes[Yaw]); }
void Gimbal_Pitch_Stop(void) {
  Pitch_Cal_Cancel();
  Gimbal_Pitch_Scan_Cancel();
  axis_stop(&axes[Pitch]);
  if (g_pitch_scan.phase >= PITCH_SCAN_COMPLETE_HOLD) {
    g_pitch_scan.holding = 0U;
    snprintf((char *)g_pitch_scan.end_summary,
             sizeof(g_pitch_scan.end_summary),
             "扫描已退出；轴已停用（撤力）；前馈发送值0");
  }
}
void Gimbal_Stop_All(void) {
  Pitch_Cal_Cancel();
  Gimbal_Pitch_Scan_Cancel();
  axis_stop(&axes[Yaw]);
  axis_stop(&axes[Pitch]);
  if (g_pitch_scan.phase >= PITCH_SCAN_COMPLETE_HOLD) {
    g_pitch_scan.holding = 0U;
    snprintf((char *)g_pitch_scan.end_summary,
             sizeof(g_pitch_scan.end_summary),
             "扫描已退出；轴已停用（撤力）；前馈发送值0");
  }
}

void Gimbal_Pitch_Probe(void) { pitch_probe_pending=1; }
uint8_t Gimbal_Pitch_Set_Mode(Pitch_Control_Mode_t mode) {
  pitch_probe_pending=1;
  if (mode!=PITCH_MODE_ENCODER && mode!=PITCH_MODE_INERTIAL) return 0;
  if (g_pitch_test.state!=YAW_TEST_ACTIVE ||
      !DM_Motor_Feedback_Valid(Pitch,HAL_GetTick())) {
    g_pitch_inertial.reason=PI_AXIS; return 0;
  }
  if (g_pitch_scan.active || Pitch_Cal_Is_Active() || Pitch_Cont_Active() ||
      g_gimbal_repeat[Pitch].active) {
    g_pitch_inertial.reason=PI_BUSY; return 0;
  }
  DM_Motor_Data_t feedback;
  const uint32_t primask=__get_PRIMASK();
  __disable_irq(); feedback=dm_motor[Pitch];
  if (!primask) __enable_irq();
  axis_update_measurement(&axes[Pitch],&feedback);
  const float encoder=g_pitch_test.measured_relative_rad;
  if (!isfinite(encoder) || encoder<axes[Pitch].min_target_rad ||
      encoder>axes[Pitch].max_target_rad) {
    g_pitch_inertial.reason=PI_BOUNDS; return 0;
  }
  if (mode==PITCH_MODE_INERTIAL) {
    if (!Pitch_Inertial_Capture(HAL_GetTick(),encoder)) return 0;
  } else {
    Pitch_Inertial_Reset();
    g_pitch_inertial.reason=PI_OK;
  }
  /* Preserve boot origin, gains, feedforward table/state and independent Yaw. */
  g_pitch_test.target_relative_rad=g_pitch_test.command_relative_rad=encoder;
  axes[Pitch].last_control_ms=HAL_GetTick();
  return 1;
}

uint8_t Gimbal_Yaw_Set_Mode(Yaw_Control_Mode_t mode) {
  if (YawCal_Active()) return 0U;
  if (mode != YAW_MODE_ENCODER && mode != YAW_MODE_INERTIAL) {
    g_yaw_inertial.switch_result = 3; return 0;
  }
  if (g_yaw_test.state != YAW_TEST_ACTIVE) {
    g_yaw_inertial.switch_result = 1; return 0;
  }
  Yaw_Inertial_Update(HAL_GetTick());
  if (mode == YAW_MODE_INERTIAL && !g_yaw_inertial.valid) {
    g_yaw_inertial.switch_result = 2; return 0;
  }
  YawDiag_Stop(YD_STOP_MODE);
  axis_repeat_cancel(&axes[Yaw]);
  // 切换以当前姿态接住，不把旧坐标的目标带入新模式。
  load_reset(&axes[Yaw]);
  Yaw_Inertial_Zero(g_yaw_test.measured_relative_rad);
  g_yaw_inertial.mode = mode;
  g_yaw_test.target_relative_rad = Gimbal_Yaw_Control_Angle_Rad();
  g_yaw_test.command_relative_rad = g_yaw_test.target_relative_rad;
  g_yaw_test.applied_t_ff_nm = 0;
  axes[Yaw].last_control_ms = HAL_GetTick();
  g_yaw_inertial.switch_result = 0;
  g_gimbal_log_axis = Yaw;
  return 1;
}

void Gimbal_Yaw_Enable(void) {
  g_gimbal_log_axis = Yaw;
  if ((g_yaw_test.state == YAW_TEST_DISABLED) ||
      (g_yaw_test.state == YAW_TEST_FAULT)) {
    axis_begin_enable(&axes[Yaw], HAL_GetTick());
  }
}

void Gimbal_Pitch_Enable(void) {
  g_gimbal_log_axis = Pitch;
  if ((g_pitch_test.state == YAW_TEST_DISABLED) ||
      (g_pitch_test.state == YAW_TEST_FAULT)) {
    axis_begin_enable(&axes[Pitch], HAL_GetTick());
  }
}

void Gimbal_Enable_All(void) {
  Gimbal_Yaw_Enable();
  Gimbal_Pitch_Enable();
}

uint8_t Gimbal_Yaw_Set_Target_Deg(float target_deg)
{
  return Gimbal_Yaw_Set_Target_Rad(target_deg * DEG_TO_RAD);
}

uint8_t Gimbal_Yaw_Set_Target_Rad(float target_rad)
{
  return axis_set_target(&axes[Yaw], target_rad);
}

uint8_t Gimbal_Yaw_Set_Target_Speed_Deg(float target_deg,float max_speed_deg_s)
{
  return axis_set_target_speed(&axes[Yaw], target_deg * DEG_TO_RAD,max_speed_deg_s * DEG_TO_RAD);
}

uint8_t Gimbal_Yaw_Move_Relative_Deg(float delta_deg)
{
  return axis_move_relative(&axes[Yaw], delta_deg * DEG_TO_RAD);
}

uint8_t Gimbal_Yaw_Move_Relative_Speed_Deg(float delta_deg,float max_speed_deg_s)
{
  return axis_move_relative_speed(&axes[Yaw], delta_deg * DEG_TO_RAD,max_speed_deg_s * DEG_TO_RAD);
}

uint8_t Gimbal_Yaw_Set_Max_Speed_Deg(float max_speed_deg_s)
{
  return axis_set_max_speed(&axes[Yaw], max_speed_deg_s * DEG_TO_RAD);
}

uint8_t Gimbal_Yaw_Set_Kp(float kp) {
  if (!isfinite(kp) || kp < 0.0f || kp > YAW_MAX_KP) return 0U;
  return axis_set_kp(&axes[Yaw], kp);
}
uint8_t Gimbal_Yaw_Set_Kd(float kd) { return axis_set_kd(&axes[Yaw], kd); }

static void pitch_scan_mark_manual_handoff(uint8_t command_ok) {
  if (command_ok && (g_pitch_scan.phase >= PITCH_SCAN_COMPLETE_HOLD)) {
    g_pitch_scan.holding = 0U;
    snprintf((char *)g_pitch_scan.end_summary,
             sizeof(g_pitch_scan.end_summary),
             "扫描已退出；前馈%s；手动控制已接管",
             Pitch_FF_Get_Scale() > 0.0f ? "开" : "关");
  }
}

uint8_t Gimbal_Pitch_Set_Target_Deg(float target_deg)
{
  return Gimbal_Pitch_Set_Target_Rad(target_deg * DEG_TO_RAD);
}

uint8_t Gimbal_Pitch_Set_Target_Rad(float target_rad)
{
  Gimbal_Pitch_Scan_Cancel();
  const uint8_t result = axis_set_target(&axes[Pitch], target_rad);
  pitch_scan_mark_manual_handoff(result);
  return result;
}

uint8_t Gimbal_Pitch_Set_Target_Speed_Deg(float target_deg,float max_speed_deg_s)
{
  Gimbal_Pitch_Scan_Cancel();
  const uint8_t result = axis_set_target_speed(
      &axes[Pitch], target_deg * DEG_TO_RAD, max_speed_deg_s * DEG_TO_RAD);
  pitch_scan_mark_manual_handoff(result);
  return result;
}

uint8_t Gimbal_Pitch_Move_Relative_Deg(float delta_deg)
{
  Gimbal_Pitch_Scan_Cancel();
  const uint8_t result =
      axis_move_relative(&axes[Pitch], delta_deg * DEG_TO_RAD);
  pitch_scan_mark_manual_handoff(result);
  return result;
}

uint8_t Gimbal_Pitch_Move_Relative_Speed_Deg(float delta_deg,float max_speed_deg_s)
{
  Gimbal_Pitch_Scan_Cancel();
  const uint8_t result = axis_move_relative_speed(
      &axes[Pitch], delta_deg * DEG_TO_RAD, max_speed_deg_s * DEG_TO_RAD);
  pitch_scan_mark_manual_handoff(result);
  return result;
}

uint8_t Gimbal_Pitch_Set_Max_Speed_Deg(float max_speed_deg_s)
{
  Gimbal_Pitch_Scan_Cancel();
  return axis_set_max_speed(&axes[Pitch], max_speed_deg_s * DEG_TO_RAD);
}

uint8_t Gimbal_Pitch_Set_Kp(float kp)
{
  Gimbal_Pitch_Scan_Cancel();
  return axis_set_kp(&axes[Pitch], kp);
}

uint8_t Gimbal_Pitch_Set_Kd(float kd)
{
  Gimbal_Pitch_Scan_Cancel();
  return axis_set_kd(&axes[Pitch], kd);
}


uint8_t Gimbal_Pitch_Set_Gravity_FF_Scale(float scale)
{
  Gimbal_Pitch_Scan_Cancel();
  const uint8_t result = Pitch_FF_Set_Scale(scale);
  if ((result != 0U) && (scale == 0.0f)) {
    g_pitch_test.applied_t_ff_nm = 0.0f;
  }
  if ((result != 0U) &&
      (g_pitch_scan.phase >= PITCH_SCAN_COMPLETE_HOLD)) {
    snprintf((char *)g_pitch_scan.end_summary,
             sizeof(g_pitch_scan.end_summary),
             "扫描已退出；前馈%s；保持",
             (scale == 0.0f) ? "关" : "开");
  }
  return result;
}

uint8_t Gimbal_Yaw_Cal_Arm(void) {
  yaw_cal_start_reason="none";
  DM_Motor_Data_t feedback;
  const uint32_t primask=__get_PRIMASK();
  __disable_irq(); feedback=dm_motor[Yaw];
  if (!primask) __enable_irq();
  const uint8_t valid=feedback.rx_count && (uint32_t)(HAL_GetTick()-feedback.last_rx_ms)<=DM_FEEDBACK_TIMEOUT_MS;
  if (valid && axes[Yaw].boot_zero_valid) axis_update_measurement(&axes[Yaw],&feedback);
  if (g_yaw_test.state!=YAW_TEST_ACTIVE) yaw_cal_start_reason="yaw_not_enabled_use_ye";
  else if (g_yaw_inertial.mode!=YAW_MODE_ENCODER) yaw_cal_start_reason="encoder_mode_required";
  else if (YawCal_Active() || g_gimbal_repeat[Yaw].active) yaw_cal_start_reason="motion_busy";
  else if (!valid) yaw_cal_start_reason="feedback_expired";
  // Keep both endpoints inside the existing motor protocol range.
  else if (!isfinite(g_yaw_test.measured_relative_rad) || !isfinite(feedback.p_int) ||
           fabsf(feedback.p_int)+YAW_CAL_GUARD_DEG*DEG_TO_RAD>12.4f)
    yaw_cal_start_reason="motor_range";
  else { g_yaw_cal.armed=1; return 1; }
  return 0;
}
uint8_t Gimbal_Yaw_Cal_Start(void) {
  if (!Gimbal_Yaw_Cal_Arm()) return 0;
  if (!YawCal_Start(HAL_GetTick(),g_yaw_test.measured_relative_rad*RAD_TO_DEG)) return 0;
  YawDiag_Stop(YD_STOP_MODE);
  cal_abort_reported=0;
  yaw_cal_saved_ff=yaw_friction_enabled;
  yaw_cal_saved_speed=g_yaw_test.max_speed_rad_s;
  yaw_friction_enabled=0;
  g_load_comp[Yaw].integral_enabled=0;
  LoadComp_Reset(&g_load_comp[Yaw]);
  // Begin the existing bounded approach at the measured position, not an old target.
  g_yaw_test.command_relative_rad=g_yaw_test.measured_relative_rad;
  g_yaw_test.target_relative_rad=g_yaw_cal.target_deg*DEG_TO_RAD;
  g_yaw_test.max_speed_rad_s=g_yaw_cal.speed_deg_s*DEG_TO_RAD;
  return 1;
}
void Gimbal_Yaw_Cal_Stop(void) {
  YawCal_Stop(YCAL_CANCEL);
  axis_stop(&axes[Yaw]);
  Gimbal_Load_Report();
}
uint8_t Gimbal_Yaw_Friction_Enable(uint8_t enable) {
  if (enable>1 || YawCal_Active()) return 0;
  if (enable && (!g_yaw_cal.ready || g_yaw_test.state!=YAW_TEST_ACTIVE ||
                  g_yaw_inertial.mode!=YAW_MODE_ENCODER)) return 0;
  yaw_friction_enabled=enable; return 1;
}
uint8_t Gimbal_Yaw_Position_Boost_Enable(uint8_t enable) {
  if (enable>1) return 0;
  yaw_boost_enabled=enable;
  yaw_boost_nm=0;
  yaw_boost_target=g_yaw_test.target_relative_rad;
  if (enable) {
    g_load_comp[Yaw].integral_enabled=0;
    g_load_comp[Yaw].integral_nm=0;
    g_load_comp[Yaw].dwell=0;
  }
  return 1;
}
uint8_t Gimbal_Load_Integral_Enable(uint8_t axis, uint8_t enable) {
  if (axis>=DM_NUM || enable>1 || (axis==Yaw && YawCal_Active())) return 0;
  if (enable && (axes[axis].status->state!=YAW_TEST_ACTIVE ||
      (axis==Yaw && g_yaw_inertial.mode!=YAW_MODE_ENCODER) ||
      (axis==Pitch && (g_pitch_scan.active || Pitch_Cal_Is_Active())))) return 0;
  if (axis==Yaw && enable) (void)Gimbal_Yaw_Position_Boost_Enable(0);
  g_load_comp[axis].integral_enabled=enable;
  g_load_comp[axis].dwell=0;
  return 1;
}
void Gimbal_Load_Report(void) {
  if (g_yaw_cal.state==YCAL_ABORT && g_yaw_cal.error!=YCAL_CANCEL && !cal_abort_reported) {
    static const char *const reasons[]={"none","cancel","motor_range","feedback_invalid",
        "angle_jump","speed","torque","tracking","timeout","stall",
        "insufficient_samples","not_repeatable","torque_sign","mode_or_motor_fault"};
    const unsigned code=g_yaw_cal.error;
    cal_abort_reported=1;
    char reason[80];
    snprintf(reason,sizeof(reason),"%s",code<sizeof(reasons)/sizeof(reasons[0])?reasons[code]:"unknown");
    fault_notice_begin("Yaw","ycal",code,reason);
    if (code==YCAL_SAMPLES || code==YCAL_SIGN || code==YCAL_REPEATABILITY) {
      unsigned used=(unsigned)snprintf(fault_notice,sizeof(fault_notice),
          "!!! YCAL FAILED code=%u pairs=%u/%u center=%.1f old_kept=%u HOLD; counts(+/-):",
          code,g_yaw_cal.paired,YAW_CAL_MIN_PAIRS,g_yaw_cal.center_deg,g_yaw_cal.ready);
      for(unsigned i=0;i<YAW_CAL_NODES && used<sizeof(fault_notice);++i)
        used+=(unsigned)snprintf(fault_notice+used,sizeof(fault_notice)-used," %.1f:%u/%u",
          YawCal_NodeAngle(i),g_yaw_cal.counts[0][0][i],g_yaw_cal.counts[0][1][i]);
      if(used+2<sizeof(fault_notice)) snprintf(fault_notice+used,sizeof(fault_notice)-used,"\r\n");
    }
    return;
  }
  report_snapshot=g_yaw_cal; report_pending=1; report_line=0;
}
void Gimbal_Load_Command_Result(const char *name, uint8_t ok) {
  snprintf(report_command,sizeof(report_command),"%s",name);
  report_command_ok=ok;
  Gimbal_Load_Report();
}
uint8_t Gimbal_Load_Report_Line(char *buf, unsigned capacity) {
  if (!buf || capacity<256) return 0;
  if (Gimbal_Load_Telemetry_Paused()) {
    if (fault_notice_sent>=3 || (fault_notice_sent &&
        (uint32_t)(HAL_GetTick()-fault_notice_last_ms)<1000U)) return 0;
    snprintf(buf,capacity,"%s",fault_notice);
    return 1;
  }
  if (pitch_probe_pending) {
    Pitch_Inertial_Observe(HAL_GetTick());
    snprintf(buf,capacity,
      "#PP,state=%u,fault=%u,mode=%u,valid=%u,age=%lu,reason=%u,goal=%.4f,elev=%.4f,enc=%.4f,target=%.4f,delta=%.4f,gain=%.3f,limited=%u,kp=%.2f,kd=%.2f,ff_scale=%.2f\r\n",
      (unsigned)g_pitch_test.state,(unsigned)g_pitch_test.fault,
      (unsigned)g_pitch_inertial.mode,g_pitch_inertial.valid,
      (unsigned long)g_pitch_inertial.age_ms,(unsigned)g_pitch_inertial.reason,
      (double)(g_pitch_inertial.target_elevation_rad*RAD_TO_DEG),
      (double)(g_pitch_inertial.elevation_rad*RAD_TO_DEG),
      (double)(g_pitch_test.measured_relative_rad*RAD_TO_DEG),
      (double)(g_pitch_test.target_relative_rad*RAD_TO_DEG),
      (double)(g_pitch_inertial.correction_rad*RAD_TO_DEG),
      (double)g_pitch_inertial.elevation_per_encoder_rad,g_pitch_inertial.limited,
      (double)g_pitch_test.kp,(double)g_pitch_test.kd,(double)Pitch_FF_Get_Scale());
    return 1;
  }
  if (yaw_probe_pending) {
    DM_Motor_Data_t feedback;
    const uint32_t primask=__get_PRIMASK();
    __disable_irq(); feedback=dm_motor[Yaw];
    if (!primask) __enable_irq();
    snprintf(buf,capacity,
      "#YP,id=%lu,state=%u,fault=%u,mode=%u,target=%.6f,enc=%.6f,raw=%.8f,rate=%.4f,kp=%.4f,kd=%.4f,ff=%u,i=%u,boost=%u,tff=%.6f,friction=%.6f,correct=%u,brake=%u,age=%lu,diag=%u\r\n",
      (unsigned long)yaw_probe_id,(unsigned)g_yaw_test.state,(unsigned)g_yaw_test.fault,
      (unsigned)g_yaw_inertial.mode,(double)(g_yaw_test.target_relative_rad*RAD_TO_DEG),
      (double)(g_yaw_test.measured_relative_rad*RAD_TO_DEG),(double)feedback.p_int,
      (double)(g_yaw_test.measured_velocity_rad_s*RAD_TO_DEG),
      (double)g_yaw_test.kp,(double)g_yaw_test.kd,yaw_friction_enabled,
      g_load_comp[Yaw].integral_enabled,yaw_boost_enabled,(double)g_yaw_test.applied_t_ff_nm,
      (double)g_load_comp[Yaw].friction_nm,yaw_friction_goal.correcting,yaw_friction_goal.braking,
      (unsigned long)(HAL_GetTick()-feedback.last_rx_ms),(unsigned)g_yaw_diag.state);
    return 1;
  }
  if (!report_pending) {
    if (YawDiag_ExportLine(buf,capacity)) return 1;
    return Pitch_Cont_Export_Line(buf,capacity);
  }
  if (!report_command_ok && (strcmp(report_command,"ycal")==0 || strcmp(report_command,"ycalarm")==0)) {
    snprintf(buf,capacity,"#YCAL cmd=%s ok=0 reject=%s\r\n",report_command,yaw_cal_start_reason);
    return 1;
  }
  if (report_line==0) {
    snprintf(buf,capacity,"#YCAL cmd=%s ok=%u state=%u error=%u lap=%u armed=%u ready=%u table=%u speed=%.2f center=%.2f B=%.5f F=%.5f pairs=%u/%u active_F=%.5f ff_limit=1.0 yff=%u yi=%u pi=%u\r\n",
      report_command,report_command_ok,report_snapshot.state,report_snapshot.error,report_snapshot.lap,report_snapshot.armed,
      report_snapshot.ready,report_snapshot.use_table,report_snapshot.speed_deg_s,report_snapshot.center_deg,
      report_snapshot.b_mean,report_snapshot.f_mean,report_snapshot.paired,YAW_CAL_NODES,YawCal_Friction(0),
      yaw_friction_enabled,g_load_comp[Yaw].integral_enabled,g_load_comp[Pitch].integral_enabled);
  } else if (report_line<=YAW_CAL_NODES) {
    const unsigned i=report_line-1;
    snprintf(buf,capacity,"#YNODE %.1f B=%.5f F=%.5f plus=%.5f minus=%.5f count=%u,%u\r\n",
      report_snapshot.center_deg-YAW_CAL_NODE_HALF_DEG+5.0f*i,report_snapshot.b[i],report_snapshot.f[i],
      report_snapshot.positive[0][i],report_snapshot.negative[0][i],
      report_snapshot.counts[0][0][i],report_snapshot.counts[0][1][i]);
  } else if (report_line==YAW_CAL_NODES+1 && report_snapshot.ready) {
    snprintf(buf,capacity,"static const float yaw_saved_f_nm = %.9ef;\r\n",YawCal_Friction(0));
  } else snprintf(buf,capacity,"#YCAL_END\r\n");
  return 1;
}
void Gimbal_Load_Report_Advance(void) {
  // Advance only after UART DMA accepted the line; BUSY must not consume a repeat.
  if (fault_notice_active) {
    fault_notice_last_ms=HAL_GetTick();
    if (!fault_notice_sent) fault_notice_first_ms=fault_notice_last_ms;
    ++fault_notice_sent;
    return;
  }
  if (pitch_probe_pending) { pitch_probe_pending=0; return; }
  if (yaw_probe_pending) { yaw_probe_pending=0; return; }
  if (!report_pending) {
    if (YawDiag_Exporting()) YawDiag_ExportAdvance();
    else Pitch_Cont_Export_Advance();
    return;
  }
  if ((!report_command_ok && (strcmp(report_command,"ycal")==0 || strcmp(report_command,"ycalarm")==0)) ||
      ++report_line>YAW_CAL_NODES+2) report_pending=0;
}

/* Diagnostic commands never change targets, gains, FF selection or axis state. */
uint8_t Gimbal_Yaw_Diag_Command(unsigned action) {
  if (action==0) {
    if (g_yaw_test.state!=YAW_TEST_ACTIVE || g_yaw_inertial.mode!=YAW_MODE_ENCODER ||
        YawCal_Active() || g_gimbal_repeat[Yaw].active || yaw_boost_enabled ||
        g_load_comp[Yaw].integral_enabled) return 0;
    return YawDiag_Arm();
  }
  if (action==1) { YawDiag_Stop(YD_STOP_MANUAL); return 1; }
  if (action==2) {
    if (g_yaw_test.state!=YAW_TEST_DISABLED && g_yaw_test.state!=YAW_TEST_FAULT) return 0;
    YawDiag_Stop(YD_STOP_MANUAL);
    return YawDiag_Dump();
  }
  return 0;
}

void Gimbal_Yaw_Probe_Request(uint32_t request_id) {
  yaw_probe_id=request_id; yaw_probe_pending=1;
}
