#include "headfile.h"
#include <stdarg.h>
// 电机物理极限参数
#define DM_P_MIN -12.5f
#define DM_P_MAX  12.5f
#define DM_V_MIN -30.0f
#define DM_V_MAX  30.0f
#define DM_KP_MIN 0.0f
#define DM_KP_MAX 500.0f
#define DM_KD_MIN 0.0f
#define DM_KD_MAX 5.0f
#define DM_T_MIN -10.0f
#define DM_T_MAX  10.0f

#define DM_PARAM_CAN_ID          0x7FFU
#define DM_PARAM_READ            0x33U
#define DM_PARAM_WRITE           0x55U
#define DM_PARAM_SAVE            0xAAU
#define DM_PARAM_RID_MST_ID      7U
#define DM_PARAM_RID_ESC_ID      8U
#define DM_PARAM_REPLY_TIMEOUT_MS 120U
#define DM_PARAM_RETRY_COUNT     3U

typedef struct {
  uint16_t target_id;
  uint16_t reply_std_id;
  uint8_t opcode;
  uint8_t rid;
  uint32_t value;
  uint32_t sequence;
} DM_Param_Reply_t;

volatile DM_Motor_Data_t dm_motor[DM_NUM];
volatile DM_Id_Config_Status_t g_dm_id_config;
static volatile DM_Param_Reply_t g_dm_param_reply;
static uint32_t g_dm_next_probe_ms;
static uint32_t g_dm_last_report_ms;

static void dm_config_log(const char *format, ...) {
  char line[144];
  va_list args;
  int length;

  if (format == NULL) return;
  va_start(args, format);
  length = vsnprintf(line, sizeof(line), format, args);
  va_end(args);

  if (length <= 0) return;
  if ((size_t)length >= sizeof(line)) {
    length = (int)sizeof(line) - 1;
  }
  (void)HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)length, 100U);
}

static const char *dm_config_state_name(DM_Id_Config_State_e state) {
  switch (state) {
    case DM_ID_CONFIG_WAIT_KEY: return "WAIT_KEY";
    case DM_ID_CONFIG_RUNNING: return "RUNNING";
    case DM_ID_CONFIG_SUCCESS: return "SUCCESS";
    case DM_ID_CONFIG_SAVED_REBOOT_REQUIRED: return "REBOOT_REQUIRED";
    case DM_ID_CONFIG_FAILED: return "FAILED";
    default: return "UNKNOWN";
  }
}

static void dm_config_report_status(void) {
  dm_config_log(
      "[DM-ID][STATUS] %s err=%u std=%03X target=%u op=%02X rid=%u value=%lu tx=%lu rx=%lu\r\n",
      dm_config_state_name(g_dm_id_config.state),
      (unsigned)g_dm_id_config.error,
      g_dm_id_config.last_reply_std_id,
      g_dm_id_config.last_target_id,
      g_dm_id_config.last_opcode,
      g_dm_id_config.last_rid,
      (unsigned long)g_dm_id_config.last_value,
      (unsigned long)g_dm_id_config.tx_count,
      (unsigned long)g_dm_id_config.reply_count);
}

static uint32_t dm_u32_from_le(const uint8_t *data) {
  return ((uint32_t)data[0]) |
         ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) |
         ((uint32_t)data[3] << 24);
}

static void dm_u32_to_le(uint32_t value, uint8_t *data) {
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8);
  data[2] = (uint8_t)(value >> 16);
  data[3] = (uint8_t)(value >> 24);
}

static uint8_t dm_decode_param_reply(const CAN_RxHeaderTypeDef *rx_header,
                                     const uint8_t rx_data[8]) {
  const uint8_t opcode = rx_data[2];
  const uint16_t target_id =
      (uint16_t)rx_data[0] | ((uint16_t)rx_data[1] << 8);

  if ((g_dm_id_config.state != DM_ID_CONFIG_RUNNING) ||
      ((opcode != DM_PARAM_READ) && (opcode != DM_PARAM_WRITE)) ||
      ((target_id != DM_ID_CONFIG_OLD_ID) &&
       (target_id != DM_ID_CONFIG_NEW_ID))) {
    return 0U;
  }

  g_dm_param_reply.target_id = target_id;
  g_dm_param_reply.reply_std_id = rx_header->StdId;
  g_dm_param_reply.opcode = opcode;
  g_dm_param_reply.rid = rx_data[3];
  g_dm_param_reply.value = dm_u32_from_le(&rx_data[4]);
  g_dm_param_reply.sequence++;

  g_dm_id_config.last_opcode = opcode;
  g_dm_id_config.last_rid = rx_data[3];
  g_dm_id_config.last_target_id = g_dm_param_reply.target_id;
  g_dm_id_config.last_reply_std_id = rx_header->StdId;
  g_dm_id_config.last_value = g_dm_param_reply.value;
  g_dm_id_config.reply_count++;
  return 1U;
}

// 浮点 -> 整型 (发送用)
int float_to_uint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  if (x < x_min) x = x_min;
  else if (x > x_max) x = x_max;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

// 整型 -> 浮点 (接收用)
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void DM_Motor_CAN_Decode(const CAN_RxHeaderTypeDef *rx_header,
                         const uint8_t rx_data[8]) {
  if ((rx_header == NULL) || (rx_data == NULL) ||
      (rx_header->IDE != CAN_ID_STD) ||
      (rx_header->RTR != CAN_RTR_DATA) ||
      (rx_header->DLC != 8U)) {
    return;
  }

  if (dm_decode_param_reply(rx_header, rx_data)) {
    return;
  }

  {
    // 数据第0字节：低4位是电机ID，高4位是状态错误码
    uint8_t motor_id = rx_data[0] & 0x0F;

    /* Logical order differs from CAN ID order: yaw=2, pitch=1. */
    DM_Motor_ID_e index;
    if (motor_id == DM_YAW_CAN_ID) {
      index = Yaw;
    } else if (motor_id == DM_PITCH_CAN_ID) {
      index = Pitch;
    } else {
      return;
    }

    // 拼接16位和12位数据
    uint16_t p_int = (rx_data[1] << 8) | rx_data[2];
    uint16_t v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    uint16_t t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];

    // 解压成真实的物理浮点数
    dm_motor[index].id = motor_id;
    dm_motor[index].state = (rx_data[0] >> 4);
    dm_motor[index].feedback_std_id = rx_header->StdId;
    dm_motor[index].p_int = uint_to_float(p_int, DM_P_MIN, DM_P_MAX, 16);
    dm_motor[index].v_int = uint_to_float(v_int, DM_V_MIN, DM_V_MAX, 12);
    dm_motor[index].t_int = uint_to_float(t_int, DM_T_MIN, DM_T_MAX, 12);
    dm_motor[index].t_mos = (float)rx_data[6];
    dm_motor[index].t_rotor = (float)rx_data[7];
    dm_motor[index].last_rx_ms = HAL_GetTick();
    dm_motor[index].rx_count++;
  }
}

void DM_Motor_CAN_Rx(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  if ((hcan == NULL) || (hcan->Instance != CAN2)) return;
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                           &rx_header, rx_data) != HAL_OK) return;

  DM_Motor_CAN_Decode(&rx_header, rx_data);
}

uint8_t DM_Motor_Feedback_Valid(DM_Motor_ID_e id, uint32_t now_ms) {
  if ((uint32_t)id >= (uint32_t)DM_NUM) return 0U;
  return (dm_motor[id].rx_count > 0U) &&
         ((now_ms - dm_motor[id].last_rx_ms) <= DM_FEEDBACK_TIMEOUT_MS);
}

// MIT 模式发送指令
static DM_Param_Reply_t dm_param_reply_snapshot(void) {
  DM_Param_Reply_t reply;
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  reply.target_id = g_dm_param_reply.target_id;
  reply.reply_std_id = g_dm_param_reply.reply_std_id;
  reply.opcode = g_dm_param_reply.opcode;
  reply.rid = g_dm_param_reply.rid;
  reply.value = g_dm_param_reply.value;
  reply.sequence = g_dm_param_reply.sequence;
  if (primask == 0U) {
    __enable_irq();
  }
  return reply;
}

static HAL_StatusTypeDef dm_send_param_frame(uint16_t target_id,
                                              uint8_t opcode,
                                              uint8_t rid,
                                              uint32_t value) {
  uint8_t data[8] = {
      (uint8_t)target_id, (uint8_t)(target_id >> 8), opcode, rid,
      0U, 0U, 0U, 0U
  };

  dm_u32_to_le(value, &data[4]);
  for (uint8_t attempt = 0U; attempt < DM_PARAM_RETRY_COUNT; ++attempt) {
    HAL_StatusTypeDef result = Can_Send_Std(&hcan2, DM_PARAM_CAN_ID, data, 8U);
    if (result == HAL_OK) {
      g_dm_id_config.tx_count++;
      dm_config_log(
          "[DM-ID][TX] std=7FF data=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
          data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
      return HAL_OK;
    }
    HAL_Delay(2U);
  }
  dm_config_log("[DM-ID][TX-FAIL] target=%u op=%02X rid=%u\r\n",
                target_id, opcode, rid);
  return HAL_ERROR;
}

static uint8_t dm_wait_param_reply(uint32_t previous_sequence,
                                   uint8_t opcode,
                                   uint8_t rid,
                                   uint32_t value) {
  const uint32_t start_ms = HAL_GetTick();

  while ((HAL_GetTick() - start_ms) < DM_PARAM_REPLY_TIMEOUT_MS) {
    const DM_Param_Reply_t reply = dm_param_reply_snapshot();
    if (reply.sequence != previous_sequence) {
      previous_sequence = reply.sequence;
      dm_config_log(
          "[DM-ID][RX] std=%03X target=%u op=%02X rid=%u value=%lu\r\n",
          reply.reply_std_id, reply.target_id, reply.opcode, reply.rid,
          (unsigned long)reply.value);
      const uint8_t opcode_matches =
          (reply.opcode == opcode) ||
          ((opcode == DM_PARAM_WRITE) && (reply.opcode == DM_PARAM_READ));
      if (opcode_matches && (reply.rid == rid) &&
          (reply.value == value)) {
        return 1U;
      }
    }
  }
  dm_config_log("[DM-ID][RX-TIMEOUT] op=%02X rid=%u expect=%lu\r\n",
                opcode, rid, (unsigned long)value);
  return 0U;
}

static uint8_t dm_read_param(uint16_t target_id, uint8_t rid,
                             uint32_t expected_value) {
  for (uint8_t attempt = 0U; attempt < DM_PARAM_RETRY_COUNT; ++attempt) {
    const uint32_t sequence = dm_param_reply_snapshot().sequence;
    if (dm_send_param_frame(target_id, DM_PARAM_READ, rid, 0U) == HAL_OK &&
        dm_wait_param_reply(sequence, DM_PARAM_READ, rid, expected_value)) {
      return 1U;
    }
  }
  return 0U;
}

static uint8_t dm_write_param(uint16_t target_id, uint8_t rid,
                              uint32_t value) {
  for (uint8_t attempt = 0U; attempt < DM_PARAM_RETRY_COUNT; ++attempt) {
    const uint32_t sequence = dm_param_reply_snapshot().sequence;
    if (dm_send_param_frame(target_id, DM_PARAM_WRITE, rid, value) == HAL_OK &&
        dm_wait_param_reply(sequence, DM_PARAM_WRITE, rid, value)) {
      return 1U;
    }
  }
  return 0U;
}

static uint8_t dm_send_disable(uint16_t target_id) {
  for (uint8_t attempt = 0U; attempt < DM_PARAM_RETRY_COUNT; ++attempt) {
    if (dm4310_disable(&hcan2, target_id) == HAL_OK) {
      g_dm_id_config.tx_count++;
      dm_config_log(
          "[DM-ID][TX] std=%03X data=FF FF FF FF FF FF FF FD (disable)\r\n",
          target_id);
      HAL_Delay(5U);
      return 1U;
    }
    HAL_Delay(2U);
  }
  return 0U;
}

static uint8_t dm_save_params(void) {
  uint8_t sent = 0U;
  const uint8_t save_d3[] = {
      DM_ID_CONFIG_SAVE_D3_NEW, DM_ID_CONFIG_SAVE_D3_OLD
  };
  const uint16_t target_id[] = {
      DM_ID_CONFIG_OLD_ID, DM_ID_CONFIG_NEW_ID
  };

  /* Only one target address is active, so the motor sees two save variants. */
  (void)dm_send_disable(DM_ID_CONFIG_OLD_ID);
  (void)dm_send_disable(DM_ID_CONFIG_NEW_ID);

  dm_config_log("[DM-ID] SAVE compatibility: D3=00 then D3=01\r\n");
  for (uint8_t target = 0U; target < 2U; ++target) {
    for (uint8_t variant = 0U; variant < 2U; ++variant) {
      if (dm_send_param_frame(target_id[target], DM_PARAM_SAVE,
                              save_d3[variant], 0U) == HAL_OK) {
        sent = 1U;
      }
      HAL_Delay(120U);
    }
  }
  return sent;
}

static void dm_id_config_fail(DM_Id_Config_Error_e error) {
  g_dm_id_config.error = error;
  g_dm_id_config.state = DM_ID_CONFIG_FAILED;
  dm_config_log("[DM-ID][FAILED] error=%u tx=%lu rx=%lu\r\n",
                (unsigned)error,
                (unsigned long)g_dm_id_config.tx_count,
                (unsigned long)g_dm_id_config.reply_count);
}

static void dm_run_id_configuration(void) {
  g_dm_id_config.state = DM_ID_CONFIG_RUNNING;
  g_dm_id_config.error = DM_ID_CONFIG_ERROR_NONE;
  dm_config_log("[DM-ID] START: change ESC 1->2, MST -> 0x12\r\n");

  if ((g_can2_filter_ret != HAL_OK) || (g_can2_start_ret != HAL_OK) ||
      (g_can2_notify_ret != HAL_OK)) {
    dm_id_config_fail(DM_ID_CONFIG_ERROR_CAN_INIT);
    return;
  }

  if (!dm_send_disable(DM_ID_CONFIG_OLD_ID)) {
    dm_id_config_fail(DM_ID_CONFIG_ERROR_DISABLE_TX);
    return;
  }
  dm_config_log("[DM-ID] disable old ID OK\r\n");

  if (!dm_write_param(DM_ID_CONFIG_OLD_ID, DM_PARAM_RID_MST_ID,
                      DM_YAW_MASTER_ID)) {
    dm_id_config_fail(DM_ID_CONFIG_ERROR_MST_WRITE);
    return;
  }
  dm_config_log("[DM-ID] MST_ID write confirmed: 0x12\r\n");

  if (!dm_write_param(DM_ID_CONFIG_OLD_ID, DM_PARAM_RID_ESC_ID,
                      DM_ID_CONFIG_NEW_ID)) {
    /* The address may have changed even if its write reply was lost. */
    if (!dm_read_param(DM_ID_CONFIG_NEW_ID, DM_PARAM_RID_ESC_ID,
                       DM_ID_CONFIG_NEW_ID)) {
      dm_id_config_fail(DM_ID_CONFIG_ERROR_ESC_WRITE);
      return;
    }
  }
  dm_config_log("[DM-ID] ESC_ID write/read confirmed: 2\r\n");

  if (!dm_save_params()) {
    dm_id_config_fail(DM_ID_CONFIG_ERROR_SAVE);
    return;
  }

  if (dm_read_param(DM_ID_CONFIG_NEW_ID, DM_PARAM_RID_ESC_ID,
                    DM_ID_CONFIG_NEW_ID)) {
    g_dm_id_config.state = DM_ID_CONFIG_SUCCESS;
    dm_config_log(
        "[DM-ID][LIVE-OK] ID2 replied. Power-cycle motor to test Flash.\r\n");
    return;
  }

  /* Writes were acknowledged and save was sent; power-cycle before retesting. */
  g_dm_id_config.error = DM_ID_CONFIG_ERROR_VERIFY;
  g_dm_id_config.state = DM_ID_CONFIG_SAVED_REBOOT_REQUIRED;
  dm_config_log("[DM-ID][REBOOT] save sent; live ID2 read timed out.\r\n");
}

void DM_Motor_Id_Config_Init(void) {
  memset((void *)&g_dm_id_config, 0, sizeof(g_dm_id_config));
  memset((void *)&g_dm_param_reply, 0, sizeof(g_dm_param_reply));
  g_dm_next_probe_ms = DM_ID_CONFIG_BOOT_PROBE_DELAY_MS;
  g_dm_last_report_ms = 0U;
  g_dm_id_config.state = DM_ID_CONFIG_WAIT_KEY;
  dm_config_log("\r\n[DM-ID] boot, UART2=115200 8N1, maintenance mode\r\n");
  dm_config_log("[DM-ID] CAN2 filter/start/notify=%lu/%lu/%lu\r\n",
                (unsigned long)g_can2_filter_ret,
                (unsigned long)g_can2_start_ret,
                (unsigned long)g_can2_notify_ret);
}

void DM_Motor_Id_Config_Update(uint32_t now_ms, GPIO_PinState key_level) {
  static uint32_t pressed_since_ms = 0U;
  static uint8_t timing_press = 0U;

  if ((now_ms - g_dm_last_report_ms) >= DM_ID_CONFIG_REPORT_PERIOD_MS) {
    g_dm_last_report_ms = now_ms;
    dm_config_report_status();
  }

  if (g_dm_id_config.state != DM_ID_CONFIG_WAIT_KEY) {
    return;
  }

  if (key_level == GPIO_PIN_SET) {
    if (!timing_press) {
      timing_press = 1U;
      pressed_since_ms = now_ms;
    } else if ((now_ms - pressed_since_ms) >= DM_ID_CONFIG_HOLD_MS) {
      timing_press = 0U;
      dm_run_id_configuration();
    }
    return;
  } else {
    timing_press = 0U;
  }

  /* Keep probing: the motor and Bluetooth bridge can start after the MCU. */
  if ((int32_t)(now_ms - g_dm_next_probe_ms) >= 0) {
    g_dm_next_probe_ms = now_ms + DM_ID_CONFIG_PROBE_PERIOD_MS;
    g_dm_id_config.state = DM_ID_CONFIG_RUNNING;
    dm_config_log("[DM-ID] probe: read ESC_ID from target 2\r\n");
    if ((g_can2_start_ret == HAL_OK) &&
        dm_read_param(DM_ID_CONFIG_NEW_ID, DM_PARAM_RID_ESC_ID,
                      DM_ID_CONFIG_NEW_ID)) {
      g_dm_id_config.error = DM_ID_CONFIG_ERROR_NONE;
      g_dm_id_config.state = DM_ID_CONFIG_SUCCESS;
      dm_config_log("[DM-ID][PERSISTENT-OK] motor answered as ID2\r\n");
      dm_config_report_status();
    } else {
      g_dm_id_config.state = DM_ID_CONFIG_WAIT_KEY;
      dm_config_log("[DM-ID] ID2 not found; retrying, or hold key for 1 second\r\n");
    }
  }
}

HAL_StatusTypeDef dm4310_send_mit(CAN_HandleTypeDef *hcan, uint16_t motor_id, float p_des, float v_des, float kp, float kd, float t_ff) {
  uint8_t tx_data[8];

  // 1. 压缩物理量到整型
  uint16_t p_int  = float_to_uint(p_des, DM_P_MIN,  DM_P_MAX,  16);
  uint16_t v_int  = float_to_uint(v_des, DM_V_MIN,  DM_V_MAX,  12);
  uint16_t kp_int = float_to_uint(kp,    DM_KP_MIN, DM_KP_MAX, 12);
  uint16_t kd_int = float_to_uint(kd,    DM_KD_MIN, DM_KD_MAX, 12);
  uint16_t t_int  = float_to_uint(t_ff,  DM_T_MIN,  DM_T_MAX,  12);

  // 2. 拼图游戏：把 16位和12位 的数据塞进 8个8位的字节里
  tx_data[0] = (p_int >> 8);
  tx_data[1] = p_int & 0xFF;
  tx_data[2] = (v_int >> 4);
  tx_data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  tx_data[4] = kp_int & 0xFF;
  tx_data[5] = (kd_int >> 4);
  tx_data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  tx_data[7] = t_int & 0xFF;

  // 3. 压入发送邮箱
  return Can_Send_Std(hcan, motor_id, tx_data, 8);
}


