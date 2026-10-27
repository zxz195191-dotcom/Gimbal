#pragma once
#include <stdint.h>
#define YAW_CAL_NODES 9U
#define YAW_CAL_MIN_PAIRS 3U
#define YAW_CAL_LAPS 1U
#define YAW_CAL_NODE_HALF_DEG 20.0f
#define YAW_CAL_TRAVEL_DEG 25.0f
#define YAW_CAL_GUARD_DEG 27.0f
enum { YCAL_IDLE, YCAL_SETTLE_LOW, YCAL_FORWARD, YCAL_SETTLE_HIGH,
       YCAL_REVERSE, YCAL_DONE, YCAL_ABORT };
enum { YCAL_OK, YCAL_CANCEL, YCAL_BOUNDS, YCAL_FEEDBACK, YCAL_JUMP,
       YCAL_SPEED, YCAL_TORQUE, YCAL_TRACKING, YCAL_TIMEOUT, YCAL_STALL,
       YCAL_SAMPLES, YCAL_REPEATABILITY, YCAL_SIGN, YCAL_MODE };
typedef struct {
    uint32_t now_ms, feedback_ms, rx_count;
    float angle_deg, speed_deg_s, torque_nm, command_deg;
    uint8_t valid;
} YawCalInput;
typedef struct {
    uint8_t armed, state, error, lap, ready, use_table, paired;
    float speed_deg_s, target_deg, b_mean, f_mean, max_repeat_error;
    float center_deg, model_center_deg;
    float b[YAW_CAL_NODES], f[YAW_CAL_NODES];
    float positive[YAW_CAL_LAPS][YAW_CAL_NODES];
    float negative[YAW_CAL_LAPS][YAW_CAL_NODES];
    uint8_t counts[YAW_CAL_LAPS][2][YAW_CAL_NODES];
} YawCalStatus;
extern YawCalStatus g_yaw_cal;
void YawCal_Init(void);
uint8_t YawCal_Active(void);
uint8_t YawCal_Start(uint32_t now, float angle_deg);
void YawCal_Stop(uint8_t reason);
/* Returns -1 on abort, 0 inactive/done, 1 active. Never sends motor commands. */
int YawCal_Update(const YawCalInput *in);
float YawCal_Friction(float angle_deg);
float YawCal_NodeAngle(unsigned i);

