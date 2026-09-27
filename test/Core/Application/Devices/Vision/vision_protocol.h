#pragma once

#include <stdbool.h>
#include <stdint.h>

#define VISION_FRAME_HEAD_0          ((uint8_t)'S')
#define VISION_FRAME_HEAD_1          ((uint8_t)'P')
#define GIMBAL_TO_VISION_FRAME_SIZE  43U
#define VISION_TO_GIMBAL_FRAME_SIZE  29U
#define VISION_COMMAND_TIMEOUT_MS    100U

/*
 * UART2 is shared by the binary vision protocol. Blocking ASCII traffic must
 * stay disabled while this is 1, otherwise it corrupts the NUC byte stream.
 */
#ifndef VISION_PROTOCOL_ENABLED
#define VISION_PROTOCOL_ENABLED      0U
#endif

typedef enum {
  VISION_MODE_DISABLED = 0,
  VISION_MODE_AIM = 1,
  VISION_MODE_FIRE = 2
} VisionMode_e;

typedef struct {
  VisionMode_e mode;
  float q[4];                 /* w, x, y, z */
  float yaw_rad;              /* current angle relative to A-board boot zero */
  float yaw_vel_rad_s;
  float pitch_rad;
  float pitch_vel_rad_s;
  float bullet_speed_m_s;
  uint16_t bullet_count;
} GimbalToVisionData_t;

typedef struct {
  VisionMode_e mode;
  float target_yaw_rad;       /* absolute target in the reported A-board frame */
  float target_yaw_vel_rad_s;
  float target_yaw_acc_rad_s2;
  float target_pitch_rad;
  float target_pitch_vel_rad_s;
  float target_pitch_acc_rad_s2;
  uint32_t received_ms;
} VisionToGimbalCommand_t;

typedef struct {
  uint32_t tx_frames;
  uint32_t tx_busy_drops;
  uint32_t tx_errors;
  uint32_t rx_frames;
  uint32_t rx_crc_errors;
  uint32_t rx_format_errors;
  uint32_t uart_errors;
} VisionProtocolStatus_t;

bool Vision_Protocol_Init(void);
void Vision_Protocol_Service(void);
bool Vision_Send_Gimbal_State(const GimbalToVisionData_t *state);
bool Vision_Get_Latest_Command(VisionToGimbalCommand_t *command,
                               uint32_t now_ms);
void Vision_Get_Status(VisionProtocolStatus_t *status);
