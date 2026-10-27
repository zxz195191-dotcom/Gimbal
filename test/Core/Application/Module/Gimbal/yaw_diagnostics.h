#pragma once
#include <stdint.h>

#define YAW_DIAG_CAPACITY 1024U
enum { YD_IDLE, YD_ARMED, YD_RECORDING, YD_FROZEN, YD_DUMPING };
enum { YD_STOP_NONE, YD_STOP_FULL, YD_STOP_MANUAL, YD_STOP_AXIS,
       YD_STOP_FAULT, YD_STOP_MODE };
enum {
  YD_DYNAMIC=1U, YD_REFERENCE_CLIP=2U, YD_SPEED_CLIP=4U,
  YD_LEAD_CLIP=8U, YD_RAW_CLIP=16U, YD_TORQUE_CLIP=32U,
  YD_FRICTION_CUT=64U, YD_BAD_DT=128U, YD_PACKET_CLIP=256U,
  YD_LOAD_RESET=512U, YD_FF_CORRECT=1024U, YD_FF_BRAKING=2048U
};

/* Main context only. Joint target/encoder/command: logical rad; raw position,
 * MIT arguments/decoded packet and torque: motor frame, rad/rad/s/N.m.
 * drive/arrived are NAN/255 when the dynamic branch was not selected.
 * PD estimates use the actual packed parameters, the captured raw encoder
 * and motor rate. They are not measured driver current or shaft torque. */
typedef struct {
  uint32_t time_ms, dt_ms, rx_count, feedback_age_ms, flags;
  float target_rad, encoder_rad, rate_rad_s, raw_position_rad, motor_rate_rad_s;
  float command_pre_rad, command_rad;
  float p_des, v_des, kp, kd, t_ff;
  float drive, brake_rad, friction_goal, friction_before, friction_actual;
  float feedback_torque;
  float wire_p, wire_v, wire_kp, wire_kd, wire_ff;
  float p_est, d_est, total_est;
  uint8_t arrived, mode, axis_state, ff_on, integral_on, boost_on, tx_result;
} YawDiagFrame;

typedef struct {
  uint32_t count, capacity, stop_reason, tx_failures, max_record_cycles;
  uint8_t state;
} YawDiagStatus;
extern volatile YawDiagStatus g_yaw_diag;
extern volatile YawDiagFrame g_yaw_diag_latest;
void YawDiag_Init(void);
uint8_t YawDiag_Arm(void);
void YawDiag_TargetAccepted(uint32_t now_ms);
void YawDiag_MarkReset(void);
uint8_t YawDiag_Recording(void);
void YawDiag_Prepare(const YawDiagFrame *frame);
/* Called immediately before the actual CAN attempt, then with its result.
 * These callbacks are idle unless a prepared Yaw sample exists. */
void YawDiag_MitPacked(uint16_t motor_id, const uint8_t bytes[8]);
void YawDiag_MitResult(uint16_t motor_id, uint8_t result);
void YawDiag_Stop(uint32_t reason);
uint8_t YawDiag_Dump(void);
uint8_t YawDiag_Exporting(void);
uint8_t YawDiag_ExportLine(char *buf, unsigned capacity);
void YawDiag_ExportAdvance(void);

uint8_t Gimbal_Yaw_Diag_Command(unsigned action); /* 0 arm, 1 stop, 2 dump */

void Gimbal_Yaw_Probe_Request(uint32_t request_id);
