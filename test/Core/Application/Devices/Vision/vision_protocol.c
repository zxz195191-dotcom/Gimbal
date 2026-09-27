#include "headfile.h"

#include <math.h>

#define VISION_RX_DMA_BUFFER_SIZE 64U
#define GIMBAL_TO_VISION_CRC_OFFSET 41U
#define VISION_TO_GIMBAL_CRC_OFFSET 27U

static uint8_t tx_dma_buffer[GIMBAL_TO_VISION_FRAME_SIZE];
static uint8_t rx_dma_buffer[VISION_RX_DMA_BUFFER_SIZE];
static uint8_t rx_frame[VISION_TO_GIMBAL_FRAME_SIZE];

static volatile uint8_t tx_busy;
static volatile uint8_t rx_restart_pending;
static uint16_t rx_dma_position;
static uint8_t rx_frame_length;

static volatile uint8_t latest_command_valid;
static volatile VisionToGimbalCommand_t latest_command;
static volatile VisionProtocolStatus_t vision_status;

static void put_u16_le(uint8_t *dst, uint16_t value) {
  dst[0] = (uint8_t)(value & 0xFFU);
  dst[1] = (uint8_t)(value >> 8);
}

static uint16_t get_u16_le(const uint8_t *src) {
  return (uint16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8));
}

static void put_u32_le(uint8_t *dst, uint32_t value) {
  dst[0] = (uint8_t)(value & 0xFFU);
  dst[1] = (uint8_t)((value >> 8) & 0xFFU);
  dst[2] = (uint8_t)((value >> 16) & 0xFFU);
  dst[3] = (uint8_t)(value >> 24);
}

static uint32_t get_u32_le(const uint8_t *src) {
  return (uint32_t)src[0] |
         ((uint32_t)src[1] << 8) |
         ((uint32_t)src[2] << 16) |
         ((uint32_t)src[3] << 24);
}

static void put_float_le(uint8_t *dst, float value) {
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  put_u32_le(dst, bits);
}

static float get_float_le(const uint8_t *src) {
  uint32_t bits = get_u32_le(src);
  float value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

static uint8_t command_values_valid(const VisionToGimbalCommand_t *command) {
  return (command->mode <= VISION_MODE_FIRE) &&
         isfinite(command->target_yaw_rad) &&
         isfinite(command->target_yaw_vel_rad_s) &&
         isfinite(command->target_yaw_acc_rad_s2) &&
         isfinite(command->target_pitch_rad) &&
         isfinite(command->target_pitch_vel_rad_s) &&
         isfinite(command->target_pitch_acc_rad_s2);
}

static void accept_rx_frame(void) {
  const uint16_t expected_crc =
      CRC16_MCRF4XX(rx_frame, VISION_TO_GIMBAL_CRC_OFFSET);
  const uint16_t received_crc =
      get_u16_le(&rx_frame[VISION_TO_GIMBAL_CRC_OFFSET]);

  if (expected_crc != received_crc) {
    vision_status.rx_crc_errors++;
    return;
  }

  VisionToGimbalCommand_t command;
  command.mode = (VisionMode_e)rx_frame[2];
  command.target_yaw_rad = get_float_le(&rx_frame[3]);
  command.target_yaw_vel_rad_s = get_float_le(&rx_frame[7]);
  command.target_yaw_acc_rad_s2 = get_float_le(&rx_frame[11]);
  command.target_pitch_rad = get_float_le(&rx_frame[15]);
  command.target_pitch_vel_rad_s = get_float_le(&rx_frame[19]);
  command.target_pitch_acc_rad_s2 = get_float_le(&rx_frame[23]);
  command.received_ms = HAL_GetTick();

  if (!command_values_valid(&command)) {
    vision_status.rx_format_errors++;
    return;
  }

  latest_command = command;
  latest_command_valid = 1U;
  vision_status.rx_frames++;
}

static void consume_rx_byte(uint8_t byte) {
  if (rx_frame_length == 0U) {
    if (byte == VISION_FRAME_HEAD_0) {
      rx_frame[rx_frame_length++] = byte;
    }
    return;
  }

  if (rx_frame_length == 1U) {
    if (byte == VISION_FRAME_HEAD_1) {
      rx_frame[rx_frame_length++] = byte;
    } else if (byte == VISION_FRAME_HEAD_0) {
      rx_frame[0] = byte;
    } else {
      rx_frame_length = 0U;
    }
    return;
  }

  rx_frame[rx_frame_length++] = byte;
  if (rx_frame_length == VISION_TO_GIMBAL_FRAME_SIZE) {
    accept_rx_frame();
    rx_frame_length = 0U;
  }
}

static void consume_rx_block(const uint8_t *data, uint16_t length) {
  for (uint16_t i = 0U; i < length; i++) {
    consume_rx_byte(data[i]);
  }
}

static HAL_StatusTypeDef start_rx_dma(void) {
  rx_dma_position = 0U;
  rx_frame_length = 0U;

  HAL_StatusTypeDef result = HAL_UARTEx_ReceiveToIdle_DMA(
      &huart2, rx_dma_buffer, VISION_RX_DMA_BUFFER_SIZE);
  if (result == HAL_OK) {
    /* IDLE and transfer-complete events are sufficient for the ring buffer. */
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
  }
  return result;
}

bool Vision_Protocol_Init(void) {
  _Static_assert(sizeof(float) == 4U, "vision protocol requires 32-bit float");

  memset((void *)&vision_status, 0, sizeof(vision_status));
  memset((void *)&latest_command, 0, sizeof(latest_command));
  tx_busy = 0U;
  rx_restart_pending = 0U;
  latest_command_valid = 0U;

  if (!CRC16_MCRF4XX_SelfTest()) {
    return false;
  }
  return start_rx_dma() == HAL_OK;
}

void Vision_Protocol_Service(void) {
  if (!rx_restart_pending) return;

  rx_restart_pending = 0U;
  (void)HAL_UART_AbortReceive(&huart2);
  __HAL_UART_CLEAR_OREFLAG(&huart2);
  if (start_rx_dma() != HAL_OK) {
    rx_restart_pending = 1U;
  }
}

bool Vision_Send_Gimbal_State(const GimbalToVisionData_t *state) {
  if ((state == NULL) || (state->mode > VISION_MODE_FIRE)) {
    return false;
  }
  if (tx_busy) {
    vision_status.tx_busy_drops++;
    return false;
  }

  tx_dma_buffer[0] = VISION_FRAME_HEAD_0;
  tx_dma_buffer[1] = VISION_FRAME_HEAD_1;
  tx_dma_buffer[2] = (uint8_t)state->mode;
  put_float_le(&tx_dma_buffer[3], state->q[0]);
  put_float_le(&tx_dma_buffer[7], state->q[1]);
  put_float_le(&tx_dma_buffer[11], state->q[2]);
  put_float_le(&tx_dma_buffer[15], state->q[3]);
  put_float_le(&tx_dma_buffer[19], state->yaw_rad);
  put_float_le(&tx_dma_buffer[23], state->yaw_vel_rad_s);
  put_float_le(&tx_dma_buffer[27], state->pitch_rad);
  put_float_le(&tx_dma_buffer[31], state->pitch_vel_rad_s);
  put_float_le(&tx_dma_buffer[35], state->bullet_speed_m_s);
  put_u16_le(&tx_dma_buffer[39], state->bullet_count);

  const uint16_t crc =
      CRC16_MCRF4XX(tx_dma_buffer, GIMBAL_TO_VISION_CRC_OFFSET);
  put_u16_le(&tx_dma_buffer[GIMBAL_TO_VISION_CRC_OFFSET], crc);

  tx_busy = 1U;
  if (HAL_UART_Transmit_DMA(
          &huart2, tx_dma_buffer, GIMBAL_TO_VISION_FRAME_SIZE) != HAL_OK) {
    tx_busy = 0U;
    vision_status.tx_errors++;
    return false;
  }
  return true;
}

bool Vision_Get_Latest_Command(VisionToGimbalCommand_t *command,
                               uint32_t now_ms) {
  if ((command == NULL) || !latest_command_valid) return false;

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  *command = latest_command;
  if (primask == 0U) __enable_irq();

  return (now_ms - command->received_ms) <= VISION_COMMAND_TIMEOUT_MS;
}

void Vision_Get_Status(VisionProtocolStatus_t *status) {
  if (status == NULL) return;

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  *status = vision_status;
  if (primask == 0U) __enable_irq();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if ((huart != NULL) && (huart->Instance == USART2)) {
    tx_busy = 0U;
    vision_status.tx_frames++;
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
  if ((huart == NULL) || (huart->Instance != USART2) ||
      (size > VISION_RX_DMA_BUFFER_SIZE)) {
    return;
  }

  if (size > rx_dma_position) {
    consume_rx_block(&rx_dma_buffer[rx_dma_position],
                     (uint16_t)(size - rx_dma_position));
  } else if (size < rx_dma_position) {
    consume_rx_block(&rx_dma_buffer[rx_dma_position],
                     (uint16_t)(VISION_RX_DMA_BUFFER_SIZE - rx_dma_position));
    consume_rx_block(rx_dma_buffer, size);
  }

  rx_dma_position =
      (size == VISION_RX_DMA_BUFFER_SIZE) ? 0U : size;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if ((huart != NULL) && (huart->Instance == USART2)) {
    vision_status.uart_errors++;
    tx_busy = 0U;
    rx_restart_pending = 1U;
  }
}

