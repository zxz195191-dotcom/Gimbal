#include "pitch_calibration.h"
#include "pitch_continuous_cal.h"

#include "headfile.h"

#include <math.h>
#include <string.h>

#define RAD_TO_DEG                         57.29577951308232f

#define PITCH_CAL_RAW_MIN_DEG               5.0f
#define PITCH_CAL_STEP_DEG                  5.0f
/* 旧5度打点算法保留，最高节点随机构上端下取整，本配置采至40度。 */
#define PITCH_CAL_RAW_POINT_COUNT          ((PITCH_CONT_MAX_DEG-5)/5+1U)

#define PITCH_FF_TABLE_MIN_DEG             10.0f
#define PITCH_FF_TABLE_POINT_COUNT         ((PITCH_CONT_MAX_DEG-10)/5+1U)

/* Keep the proven runtime range until the high-end turnaround is verified. */
#define PITCH_FF_RUNTIME_MIN_DEG           10.0f
#define PITCH_FF_RUNTIME_MAX_DEG           40.0f
#define PITCH_FF_MAX_ABS_NM                 3.0f
#define PITCH_FF_RATE_NM_S                  1.0f
#define PITCH_FF_MAX_STEP_MS               20U

#define PITCH_SAMPLE_COUNT                 15U
#define PITCH_SETTLE_TIME_MS              400U
#define PITCH_COMMAND_TOLERANCE_DEG         0.05f
#define PITCH_MOVE_DETECT_DEG               0.2f

typedef struct {
  float command_deg;
  float measured_up_deg;
  float torque_up_nm;
  uint8_t up_valid;
  float measured_down_deg;
  float torque_down_nm;
  uint8_t down_valid;
  uint32_t move_delay_up_ms;
  uint32_t move_delay_down_ms;
  float peak_velocity_up_deg_s;
  float peak_velocity_down_deg_s;
} Pitch_Cal_Raw_t;

typedef struct {
  float angle_deg;
  float torque_up_nm;
  float torque_down_nm;
  float gravity_nm;
  float friction_nm;
  uint8_t valid;
} Pitch_FF_Point_t;

typedef struct {
  uint8_t valid;
  uint8_t lower_index;
  uint8_t upper_index;
  float x0_deg;
  float x1_deg;
  float torque0_nm;
  float torque1_nm;
  float ratio;
  float result_nm;
} Pitch_Cal_Interp_Debug_t;

typedef struct {
  float query_deg;
  uint8_t table_valid;
  Pitch_Cal_Interp_Debug_t up;
  Pitch_Cal_Interp_Debug_t down;
} Pitch_FF_Build_Debug_t;

typedef enum {
  PITCH_CAL_IDLE = 0, // 开机不排队，必须显式启动
  PITCH_CAL_MOVE_UP,//上移动
  PITCH_CAL_SETTLE_UP,//等待稳定
  PITCH_CAL_SAMPLE_UP,//上升采样
  PITCH_CAL_MOVE_DOWN,//下移动
  PITCH_CAL_SETTLE_DOWN,
  PITCH_CAL_SAMPLE_DOWN,
  PITCH_CAL_BUILD_TABLE,//把采集到的数据制表
  PITCH_CAL_DONE//校准结束
} Pitch_Cal_State_t;

typedef enum {
  PITCH_CAL_DIRECTION_UP = 0,
  PITCH_CAL_DIRECTION_DOWN
} Pitch_Cal_Direction_t;

typedef struct {
  Pitch_Cal_State_t state;
  uint8_t index;
  uint32_t stable_start_ms;
  uint32_t move_command_ms;
  uint32_t move_delay_ms;
  float move_start_deg;
  float peak_velocity_deg_s;
  uint8_t move_started;
} Pitch_Cal_Runtime_t;

typedef struct {
  uint8_t count;
  uint32_t last_rx_count;
  float measured_sum;
  float measured_min;
  float measured_max;
  float torque_sum;
  float torque_min;
  float torque_max;
} Pitch_Sampler_t;


/* Separate symbols make raw measurements and generated data clear in a debugger. */
static Pitch_Cal_Raw_t pitch_cal_raw[PITCH_CAL_RAW_POINT_COUNT];
static Pitch_FF_Point_t pitch_ff_table[PITCH_FF_TABLE_POINT_COUNT];
volatile Pitch_FF_Build_Debug_t
    pitch_ff_build_debug[PITCH_FF_TABLE_POINT_COUNT];
static Pitch_Cal_Runtime_t pitch_cal_runtime;
static Pitch_Sampler_t pitch_sampler;
static float pitch_ff_scale = 1.0f;
static float pitch_ff_applied_nm;
volatile Pitch_FF_Debug_t g_pitch_ff_debug = {0};


static float clampf(float value, float min_value, float max_value)
{
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

static Pitch_Cal_Direction_t Pitch_Cal_Get_Direction(void)
{
  if ((pitch_cal_runtime.state == PITCH_CAL_MOVE_UP) || (pitch_cal_runtime.state == PITCH_CAL_SETTLE_UP) || (pitch_cal_runtime.state == PITCH_CAL_SAMPLE_UP))
  {
    return PITCH_CAL_DIRECTION_UP;
  }
  return PITCH_CAL_DIRECTION_DOWN;
}

static void Pitch_Cal_Init_Data(void)
{
  memset(pitch_cal_raw, 0, sizeof(pitch_cal_raw));
  memset(pitch_ff_table, 0, sizeof(pitch_ff_table));

  for (uint8_t i = 0U; i < PITCH_CAL_RAW_POINT_COUNT; i++)
  {
    pitch_cal_raw[i].command_deg = PITCH_CAL_RAW_MIN_DEG + (float)i * PITCH_CAL_STEP_DEG;
  }
  for (uint8_t i = 0U; i < PITCH_FF_TABLE_POINT_COUNT; i++)
  {
    pitch_ff_table[i].angle_deg = PITCH_FF_TABLE_MIN_DEG + (float)i * PITCH_CAL_STEP_DEG;
  }
}

static void Pitch_Cal_Move_Diagnostics_Start(uint32_t now_ms)
{
  pitch_cal_runtime.move_command_ms = now_ms;
  pitch_cal_runtime.move_delay_ms = UINT32_MAX;
  pitch_cal_runtime.move_start_deg = g_pitch_test.measured_relative_rad * RAD_TO_DEG;
  pitch_cal_runtime.peak_velocity_deg_s = 0.0f;
  pitch_cal_runtime.move_started = 0U;
}

static void Pitch_Cal_Move_Diagnostics_Update(uint32_t now_ms)
{
  const float measured_deg = g_pitch_test.measured_relative_rad * RAD_TO_DEG;
  const float velocity_deg_s = fabsf(g_pitch_test.measured_velocity_rad_s) * RAD_TO_DEG;

  if (velocity_deg_s > pitch_cal_runtime.peak_velocity_deg_s) {
    pitch_cal_runtime.peak_velocity_deg_s = velocity_deg_s;
  }
  if ((pitch_cal_runtime.move_started == 0U) &&  (fabsf(measured_deg - pitch_cal_runtime.move_start_deg) >= PITCH_MOVE_DETECT_DEG)) {
    pitch_cal_runtime.move_started = 1U;
    pitch_cal_runtime.move_delay_ms = now_ms - pitch_cal_runtime.move_command_ms;
  }
}

static void Pitch_Cal_Move_Diagnostics_Save(
    uint8_t index, Pitch_Cal_Direction_t direction)
{
  if (direction == PITCH_CAL_DIRECTION_UP) {
    pitch_cal_raw[index].move_delay_up_ms = pitch_cal_runtime.move_delay_ms;
    pitch_cal_raw[index].peak_velocity_up_deg_s = pitch_cal_runtime.peak_velocity_deg_s;
  } else {
    pitch_cal_raw[index].move_delay_down_ms = pitch_cal_runtime.move_delay_ms;
    pitch_cal_raw[index].peak_velocity_down_deg_s = pitch_cal_runtime.peak_velocity_deg_s;
  }
}

static void Pitch_Sampler_Reset(void)
{
  memset(&pitch_sampler, 0, sizeof(pitch_sampler));
  pitch_sampler.last_rx_count = dm_motor[Pitch].rx_count;
}

static void Pitch_Sampler_Push(void)
{
  if (dm_motor[Pitch].rx_count == pitch_sampler.last_rx_count) return;
  pitch_sampler.last_rx_count = dm_motor[Pitch].rx_count;

  const float measured_deg =
      g_pitch_test.measured_relative_rad * RAD_TO_DEG;
  const float torque_nm = GIMBAL_PITCH_DIRECTION * dm_motor[Pitch].t_int;

  if (pitch_sampler.count == 0U) {
    pitch_sampler.measured_min = measured_deg;
    pitch_sampler.measured_max = measured_deg;
    pitch_sampler.torque_min = torque_nm;
    pitch_sampler.torque_max = torque_nm;
  }

  pitch_sampler.measured_sum += measured_deg;
  pitch_sampler.torque_sum += torque_nm;
  if (measured_deg < pitch_sampler.measured_min) {
    pitch_sampler.measured_min = measured_deg;
  }
  if (measured_deg > pitch_sampler.measured_max) {
    pitch_sampler.measured_max = measured_deg;
  }
  if (torque_nm < pitch_sampler.torque_min) {
    pitch_sampler.torque_min = torque_nm;
  }
  if (torque_nm > pitch_sampler.torque_max) {
    pitch_sampler.torque_max = torque_nm;
  }
  pitch_sampler.count++;
}

static void Pitch_Sampler_Get_Result(float *measured_deg, float *torque_nm)
{
  const float divisor = 1.0f / (float)(PITCH_SAMPLE_COUNT - 2U);
  *measured_deg =
      (pitch_sampler.measured_sum - pitch_sampler.measured_min -
       pitch_sampler.measured_max) * divisor;
  *torque_nm =
      (pitch_sampler.torque_sum - pitch_sampler.torque_min -
       pitch_sampler.torque_max) * divisor;
}

static void Pitch_Cal_Save(uint8_t index,Pitch_Cal_Direction_t direction,float measured_deg,float torque_nm)
{
  if (index >= PITCH_CAL_RAW_POINT_COUNT) return;
  if (direction == PITCH_CAL_DIRECTION_UP) {
    pitch_cal_raw[index].measured_up_deg = measured_deg;
    pitch_cal_raw[index].torque_up_nm = torque_nm;
    pitch_cal_raw[index].up_valid = 1U;
  } else {
    pitch_cal_raw[index].measured_down_deg = measured_deg;
    pitch_cal_raw[index].torque_down_nm = torque_nm;
    pitch_cal_raw[index].down_valid = 1U;
  }
}

static uint8_t Pitch_Cal_Interpolate_Torque(
    float query_deg,
    Pitch_Cal_Direction_t direction,
    float *torque_nm,
    volatile Pitch_Cal_Interp_Debug_t *debug)
{
  Pitch_Cal_Interp_Debug_t trace = {0};
  trace.lower_index = UINT8_MAX;
  trace.upper_index = UINT8_MAX;

  if ((torque_nm == NULL) || !isfinite(query_deg)) {
    if (debug != NULL) *debug = trace;
    return 0U;
  }

  for (uint8_t i = 0U; i < (PITCH_CAL_RAW_POINT_COUNT - 1U); i++) {
    float x0;
    float x1;
    float torque0;
    float torque1;
    uint8_t valid0;
    uint8_t valid1;

    if (direction == PITCH_CAL_DIRECTION_UP) {
      x0 = pitch_cal_raw[i].measured_up_deg;
      x1 = pitch_cal_raw[i + 1U].measured_up_deg;
      torque0 = pitch_cal_raw[i].torque_up_nm;
      torque1 = pitch_cal_raw[i + 1U].torque_up_nm;
      valid0 = pitch_cal_raw[i].up_valid;
      valid1 = pitch_cal_raw[i + 1U].up_valid;
    } else {
      x0 = pitch_cal_raw[i].measured_down_deg;
      x1 = pitch_cal_raw[i + 1U].measured_down_deg;
      torque0 = pitch_cal_raw[i].torque_down_nm;
      torque1 = pitch_cal_raw[i + 1U].torque_down_nm;
      valid0 = pitch_cal_raw[i].down_valid;
      valid1 = pitch_cal_raw[i + 1U].down_valid;
    }

    if ((valid0 == 0U) || (valid1 == 0U) || (x1 <= x0)) continue;
    if ((query_deg >= x0) && (query_deg <= x1)) {
      const float ratio = (query_deg - x0) / (x1 - x0);
      *torque_nm = torque0 + ratio * (torque1 - torque0);

      trace.valid = 1U;
      trace.lower_index = i;
      trace.upper_index = i + 1U;
      trace.x0_deg = x0;
      trace.x1_deg = x1;
      trace.torque0_nm = torque0;
      trace.torque1_nm = torque1;
      trace.ratio = ratio;
      trace.result_nm = *torque_nm;
      if (debug != NULL) *debug = trace;
      return 1U;
    }
  }

  if (debug != NULL) *debug = trace;
  return 0U;
}

static void Pitch_Cal_Build_FF_Table(void)
{
  for (uint8_t i = 0U; i < PITCH_FF_TABLE_POINT_COUNT; i++) {
    Pitch_FF_Point_t *point = &pitch_ff_table[i];
    float torque_up_nm = 0.0f;
    float torque_down_nm = 0.0f;

    pitch_ff_build_debug[i].query_deg = point->angle_deg;
    pitch_ff_build_debug[i].table_valid = 0U;

    point->valid = 0U;
    point->torque_up_nm = 0.0f;
    point->torque_down_nm = 0.0f;
    point->gravity_nm = 0.0f;
    point->friction_nm = 0.0f;

    const uint8_t up_valid = Pitch_Cal_Interpolate_Torque(
        point->angle_deg,
        PITCH_CAL_DIRECTION_UP,
        &torque_up_nm,
        &pitch_ff_build_debug[i].up);
    const uint8_t down_valid = Pitch_Cal_Interpolate_Torque(
        point->angle_deg,
        PITCH_CAL_DIRECTION_DOWN,
        &torque_down_nm,
        &pitch_ff_build_debug[i].down);
    if ((up_valid == 0U) || (down_valid == 0U)) continue;

    point->torque_up_nm = torque_up_nm;
    point->torque_down_nm = torque_down_nm;
    point->gravity_nm = (torque_up_nm + torque_down_nm) * 0.5f;
    point->friction_nm = (torque_up_nm - torque_down_nm) * 0.5f;
    point->valid = 1U;
    pitch_ff_build_debug[i].table_valid = 1U;
  }
}

static uint8_t Pitch_FF_Lookup(float measured_deg,
                               int8_t approach_sign,
                               float *torque_nm)
{
  /* 新表一旦选中，缺测不能回退到旧校准表。 */
  if (g_pitch_cont.selected) return Pitch_Cont_Lookup(measured_deg, approach_sign, torque_nm);
  if ((torque_nm == NULL) ||
      (pitch_cal_runtime.state != PITCH_CAL_DONE) ||
      (measured_deg < PITCH_FF_RUNTIME_MIN_DEG) ||
      (measured_deg > PITCH_FF_RUNTIME_MAX_DEG))
    {
        return 0U;
    }

    const float table_position =
        (measured_deg - PITCH_FF_TABLE_MIN_DEG) /
        PITCH_CAL_STEP_DEG;

    uint8_t lower = (uint8_t)table_position;

    if (lower >= (PITCH_FF_TABLE_POINT_COUNT - 1U))
    {
        lower = PITCH_FF_TABLE_POINT_COUNT - 1U;
    }

    /* 先记录：程序决定从哪个下标开始查 */
    g_pitch_ff_debug.lower_index = lower;

    const Pitch_FF_Point_t *point0 =
        &pitch_ff_table[lower];

    g_pitch_ff_debug.lower_angle_deg =
        point0->angle_deg;

    if (point0->valid == 0U)
    {
        return 0U;
    }

    const float torque0 =
        (approach_sign >= 0) ?
        point0->torque_up_nm :
        point0->torque_down_nm;

    g_pitch_ff_debug.lower_torque_nm = torque0;


    /*
     * 正好落在节点上，或者已经是最后一个节点：
     * 不需要两个点插值。
     */
    if ((lower == (PITCH_FF_TABLE_POINT_COUNT - 1U)) ||
        (measured_deg <= point0->angle_deg))
    {
        g_pitch_ff_debug.upper_index = lower;
        g_pitch_ff_debug.upper_angle_deg =
            point0->angle_deg;
        g_pitch_ff_debug.upper_torque_nm =
            torque0;

        g_pitch_ff_debug.interpolated_torque_nm =
            torque0;

        *torque_nm = torque0;

        return 1U;
    }


    /* 需要下一个节点参与插值 */
    const Pitch_FF_Point_t *point1 =
        &pitch_ff_table[lower + 1U];

    g_pitch_ff_debug.upper_index =
        lower + 1U;

    g_pitch_ff_debug.upper_angle_deg =
        point1->angle_deg;

    if (point1->valid == 0U)
    {
        return 0U;
    }

    const float torque1 =
        (approach_sign >= 0) ?
        point1->torque_up_nm :
        point1->torque_down_nm;

    g_pitch_ff_debug.upper_torque_nm =
        torque1;

    const float ratio =
        (measured_deg - point0->angle_deg) /
        PITCH_CAL_STEP_DEG;

    const float interpolated_nm =
        torque0 +
        ratio * (torque1 - torque0);

    g_pitch_ff_debug.interpolated_torque_nm =
        interpolated_nm;

    *torque_nm = interpolated_nm;

    return 1U;
}

uint8_t Pitch_Cal_Is_Active(void)
{
  return pitch_cal_runtime.state != PITCH_CAL_IDLE &&
         pitch_cal_runtime.state != PITCH_CAL_DONE;
}

void Pitch_Cal_Cancel(void)
{
  if (Pitch_Cal_Is_Active()) pitch_cal_runtime.state = PITCH_CAL_IDLE;
}

void Pitch_Cal_Start(void)
{
  if (g_pitch_test.state != YAW_TEST_ACTIVE ||
      !DM_Motor_Feedback_Valid(Pitch, HAL_GetTick())) return;
  /* 原校准保留为显式入口；若正在扫描，先结束扫描，避免两个 Pitch 目标源。 */
  Gimbal_Pitch_Scan_Cancel();
  g_pitch_cont.selected = 0U;
  (void)Pitch_FF_Set_Scale(0.0f);
  Pitch_Cal_Init_Data();
  memset(&pitch_cal_runtime, 0, sizeof(pitch_cal_runtime));
  pitch_cal_runtime.state = PITCH_CAL_MOVE_UP;
  pitch_cal_runtime.index = 0U;
  pitch_cal_runtime.move_delay_ms = UINT32_MAX;
}

void Pitch_Cal_Update(uint32_t now_ms)
{
  if (!Pitch_Cal_Is_Active()) return;
  if (g_pitch_scan.active || g_pitch_test.state != YAW_TEST_ACTIVE ||
      !DM_Motor_Feedback_Valid(Pitch, now_ms)) {
    Pitch_Cal_Cancel();
    return;
  }
  if (pitch_cal_runtime.state == PITCH_CAL_BUILD_TABLE) {
    Pitch_Cal_Build_FF_Table();
    pitch_cal_runtime.state = PITCH_CAL_DONE;
    return;
  }

  const uint8_t index = pitch_cal_runtime.index;
  const Pitch_Cal_Direction_t direction = Pitch_Cal_Get_Direction();

  if ((pitch_cal_runtime.state == PITCH_CAL_MOVE_UP) ||
      (pitch_cal_runtime.state == PITCH_CAL_MOVE_DOWN)) {
    if (Gimbal_Pitch_Set_Target_Deg(
            pitch_cal_raw[index].command_deg) == 0U) {
      return;
    }
    pitch_cal_runtime.stable_start_ms = 0U;
    Pitch_Cal_Move_Diagnostics_Start(now_ms);
    pitch_cal_runtime.state =
        (direction == PITCH_CAL_DIRECTION_UP) ?
        PITCH_CAL_SETTLE_UP : PITCH_CAL_SETTLE_DOWN;
    return;
  }

  Pitch_Cal_Move_Diagnostics_Update(now_ms);

  if ((pitch_cal_runtime.state == PITCH_CAL_SETTLE_UP) ||
      (pitch_cal_runtime.state == PITCH_CAL_SETTLE_DOWN)) {
    const float command_error_deg =
        fabsf(g_pitch_test.target_relative_rad -
              g_pitch_test.command_relative_rad) * RAD_TO_DEG;
    if (command_error_deg > PITCH_COMMAND_TOLERANCE_DEG) {
      pitch_cal_runtime.stable_start_ms = 0U;
      return;
    }
    if (pitch_cal_runtime.stable_start_ms == 0U) {
      pitch_cal_runtime.stable_start_ms = now_ms;
      return;
    }
    if ((now_ms - pitch_cal_runtime.stable_start_ms) <
        PITCH_SETTLE_TIME_MS) {
      return;
    }

    Pitch_Sampler_Reset();
    pitch_cal_runtime.state =
        (direction == PITCH_CAL_DIRECTION_UP) ?
        PITCH_CAL_SAMPLE_UP : PITCH_CAL_SAMPLE_DOWN;
    return;
  }

  Pitch_Sampler_Push();
  if (pitch_sampler.count < PITCH_SAMPLE_COUNT) return;

  float measured_deg;
  float torque_nm;
  Pitch_Sampler_Get_Result(&measured_deg, &torque_nm);
    Pitch_Cal_Save(index, direction, measured_deg, torque_nm);
  Pitch_Cal_Move_Diagnostics_Save(index, direction);

  if (direction == PITCH_CAL_DIRECTION_UP) {
    if (index < (PITCH_CAL_RAW_POINT_COUNT - 1U)) {
      pitch_cal_runtime.index++;
      pitch_cal_runtime.state = PITCH_CAL_MOVE_UP;
    } else {
      /* Current proven scan turns at 50 deg and next commands 45 deg. */
      pitch_cal_runtime.index--;
      pitch_cal_runtime.state = PITCH_CAL_MOVE_DOWN;
    }
  } else if (index > 0U) {
    pitch_cal_runtime.index--;
    pitch_cal_runtime.state = PITCH_CAL_MOVE_DOWN;
  } else {
    pitch_cal_runtime.state = PITCH_CAL_BUILD_TABLE;
  }
}

void Pitch_FF_Reset(void)
{
  pitch_ff_applied_nm = 0.0f;
}

float Pitch_FF_Update(float measured_deg,
                      int8_t approach_sign,
                      uint32_t elapsed_ms)
{
  /*
   * 每次控制周期先刷新诊断信息。
   * 不改变控制，只记录当前发生了什么。
   */
  g_pitch_ff_debug.measured_deg = measured_deg;
  g_pitch_ff_debug.scale = pitch_ff_scale;
  g_pitch_ff_debug.approach_sign = approach_sign;

  g_pitch_ff_debug.lookup_valid = 0U;

  g_pitch_ff_debug.lower_index = 0U;
  g_pitch_ff_debug.upper_index = 0U;

  g_pitch_ff_debug.lower_angle_deg = 0.0f;
  g_pitch_ff_debug.upper_angle_deg = 0.0f;

  g_pitch_ff_debug.lower_torque_nm = 0.0f;
  g_pitch_ff_debug.upper_torque_nm = 0.0f;

  g_pitch_ff_debug.interpolated_torque_nm = 0.0f;
  g_pitch_ff_debug.scaled_torque_nm = 0.0f;


  float desired_nm = 0.0f;

  const uint8_t lookup_ok =
      Pitch_FF_Lookup(
          measured_deg,
          approach_sign,
          &desired_nm);

  g_pitch_ff_debug.lookup_valid = lookup_ok;

  if (g_pitch_cont.selected) {
    // 新表按全量系数直接输出；不追加3N.m软件限幅或1N.m/s缓升限制。
    pitch_ff_applied_nm = lookup_ok ? desired_nm * pitch_ff_scale : 0.0f;
    g_pitch_ff_debug.interpolated_torque_nm = lookup_ok ? desired_nm : 0.0f;
    g_pitch_ff_debug.scaled_torque_nm = pitch_ff_applied_nm;
    g_pitch_ff_debug.applied_torque_nm = pitch_ff_applied_nm;
    return pitch_ff_applied_nm;
  }



  desired_nm =
      clampf(
          desired_nm * pitch_ff_scale,
          -PITCH_FF_MAX_ABS_NM,
          PITCH_FF_MAX_ABS_NM);

  g_pitch_ff_debug.scaled_torque_nm =
      desired_nm;


  const uint32_t limited_ms =
      (elapsed_ms > PITCH_FF_MAX_STEP_MS) ?
      PITCH_FF_MAX_STEP_MS :
      elapsed_ms;

  const float max_delta_nm =
      PITCH_FF_RATE_NM_S *
      ((float)limited_ms / 1000.0f);

  const float delta_nm =
      clampf(
          desired_nm - pitch_ff_applied_nm,
          -max_delta_nm,
          max_delta_nm);

  pitch_ff_applied_nm += delta_nm;

  g_pitch_ff_debug.applied_torque_nm =
      pitch_ff_applied_nm;

  return pitch_ff_applied_nm;
}

uint8_t Pitch_FF_Set_Scale(float scale)
{
  if (!isfinite(scale) || (scale < 0.0f) || (scale > 1.0f)) return 0U;
  if (scale == 0.0f) {
    pitch_ff_scale = 0.0f;
    Pitch_FF_Reset();
    return 1U;
  }
  if (g_pitch_cont.selected) {
    if (!g_pitch_cont.table_ready) return 0U;
  } else if (pitch_cal_runtime.state != PITCH_CAL_DONE) return 0U;
  pitch_ff_scale = scale;
  return 1U;
}

float Pitch_FF_Get_Scale(void) { return pitch_ff_scale; }
