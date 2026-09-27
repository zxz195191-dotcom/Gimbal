
#pragma once
#include "headfile.h"

#define DM_PITCH_CAN_ID        0x001U
#define DM_YAW_CAN_ID          0x002U
#define DM_YAW_MASTER_ID       0x012U
#define DM_FEEDBACK_TIMEOUT_MS 50U

/*
 * One-shot maintenance firmware mode:
 *   1: normal control is disabled; hold the A-board key for one second to
 *      change the only connected motor from ID 1 to yaw ID 2.
 *   0: run the normal gimbal/shooter application.
 * Set this back to 0 and flash again after the ID change is verified.
 */
#ifndef DM_ID_CONFIG_MODE
#define DM_ID_CONFIG_MODE      0U
#endif

#define DM_ID_CONFIG_OLD_ID    0x001U
#define DM_ID_CONFIG_NEW_ID    DM_YAW_CAN_ID
#define DM_ID_CONFIG_HOLD_MS   1000U
#define DM_ID_CONFIG_BOOT_PROBE_DELAY_MS 1000U
#define DM_ID_CONFIG_PROBE_PERIOD_MS     2000U
#define DM_ID_CONFIG_REPORT_PERIOD_MS    2000U
/* Send both one time for compatibility with official new/old SDK examples. */
#define DM_ID_CONFIG_SAVE_D3_NEW 0x00U
#define DM_ID_CONFIG_SAVE_D3_OLD 0x01U

typedef enum {
  DM_ID_CONFIG_WAIT_KEY = 0,
  DM_ID_CONFIG_RUNNING,
  DM_ID_CONFIG_SUCCESS,
  DM_ID_CONFIG_SAVED_REBOOT_REQUIRED,
  DM_ID_CONFIG_FAILED
} DM_Id_Config_State_e;

typedef enum {
  DM_ID_CONFIG_ERROR_NONE = 0,
  DM_ID_CONFIG_ERROR_CAN_INIT,
  DM_ID_CONFIG_ERROR_DISABLE_TX,
  DM_ID_CONFIG_ERROR_MST_WRITE,
  DM_ID_CONFIG_ERROR_ESC_WRITE,
  DM_ID_CONFIG_ERROR_SAVE,
  DM_ID_CONFIG_ERROR_VERIFY
} DM_Id_Config_Error_e;

typedef struct {
  DM_Id_Config_State_e state;
  DM_Id_Config_Error_e error;
  uint8_t last_opcode;
  uint8_t last_rid;
  uint16_t last_target_id;
  uint16_t last_reply_std_id;
  uint32_t last_value;
  uint32_t tx_count;
  uint32_t reply_count;
} DM_Id_Config_Status_t;

// 达妙电机反馈结构体
typedef struct {
  uint8_t  id;
  uint8_t  state;      // 电机状态
  uint16_t feedback_std_id; // 电机配置的 Master ID（反馈帧标准 ID）
  float    p_int;      // 当前位置 (rad)
  float    v_int;      // 当前速度 (rad/s)
  float    t_int;      // 当前扭矩 (N.m)
  float    t_mos;      // MOS管温度
  float    t_rotor;    // 转子温度
  uint32_t last_rx_ms;
  uint32_t rx_count;
} DM_Motor_Data_t;

typedef enum {
  Yaw = 0,
  Pitch,
  DM_NUM
}DM_Motor_ID_e;

extern volatile DM_Motor_Data_t dm_motor[DM_NUM];
extern volatile DM_Id_Config_Status_t g_dm_id_config;
int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void DM_Motor_CAN_Decode(const CAN_RxHeaderTypeDef *rx_header,
                         const uint8_t rx_data[8]);
void DM_Motor_CAN_Rx(CAN_HandleTypeDef *hcan);
uint8_t DM_Motor_Feedback_Valid(DM_Motor_ID_e id, uint32_t now_ms);
HAL_StatusTypeDef dm4310_send_mit(CAN_HandleTypeDef *hcan, uint16_t motor_id,
                                  float p_des, float v_des, float kp,
                                  float kd, float t_ff);
void DM_Motor_Id_Config_Init(void);
void DM_Motor_Id_Config_Update(uint32_t now_ms, GPIO_PinState key_level);
