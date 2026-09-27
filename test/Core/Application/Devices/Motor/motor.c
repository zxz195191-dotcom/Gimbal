#include "headfile.h"

PID_TypeDef pid_spd[Motor_Count];
volatile MotorFeedBack_t motor_feedback[Motor_Count];
float target_speed_rpm[Motor_Count];
int16_t current_cmd[Motor_Count];

HAL_StatusTypeDef CAN_Send_Motor_Current(int16_t current1, int16_t current2, int16_t current3, int16_t current4) {
  CAN_TxHeaderTypeDef tx_header = {0};
  uint32_t tx_mailbox;
  uint8_t tx_data[8];

  tx_header.StdId = 0x200;       // 控制电调 1 到 4 的标准ID
  tx_header.IDE = CAN_ID_STD;    // 标准帧
  tx_header.RTR = CAN_RTR_DATA;  // 数据帧
  tx_header.DLC = 8;             // 数据长度8字节

  // 拆分高低8位发给电调（数值范围大概是 -16384 到 16384）
  tx_header.TransmitGlobalTime = DISABLE;

  tx_data[0] = current1 >> 8;
  tx_data[1] = current1;
  tx_data[2] = current2 >> 8;
  tx_data[3] = current2;
  tx_data[4] = current3 >> 8;
  tx_data[5] = current3;
  tx_data[6] = current4 >> 8;
  tx_data[7] = current4;

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
    return HAL_BUSY;
  }

  return HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
}

CAN_RxHeaderTypeDef rx_header;
uint8_t rx_data[8];

void DJI_Motor_CAN_Rx(CAN_HandleTypeDef *hcan) {

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
  {
    return;
  }

  if ((rx_header.IDE != CAN_ID_STD) ||
      (rx_header.RTR != CAN_RTR_DATA) ||
      (rx_header.DLC != 8U))
  {
    return;
  }

  if (rx_header.StdId >= 0x201 && rx_header.StdId <= 0x204)
  {
    MotorID_e id = (MotorID_e)(rx_header.StdId - 0x201U);

    for (uint8_t i = 0; i < 8U; i++)
    {
      motor_feedback[id].raw_data[i] = rx_data[i];
    }

    motor_feedback[id].angle =
        (uint16_t)(((uint16_t)rx_data[0] << 8) | rx_data[1]);

    motor_feedback[id].speed_rmp =
        (int16_t)(((uint16_t)rx_data[2] << 8) | rx_data[3]);

    motor_feedback[id].feedback_current =
        (int16_t)(((uint16_t)rx_data[4] << 8) | rx_data[5]);

    motor_feedback[id].temperature = rx_data[6];

    motor_feedback[id].last_rx_tick = DWT_Read();
    motor_feedback[id].rx_count++;
  }
  else
  {
    DM_Motor_CAN_Decode(&rx_header, rx_data);
  }
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
      DJI_Motor_CAN_Rx(hcan);
  else if (hcan->Instance == CAN2)
      DM_Motor_CAN_Rx(hcan);
}

uint8_t Motor_Feedback_Valid(MotorID_e id, uint32_t now_us)
{
  const int actual_rpm = motor_feedback[id].speed_rmp;
  const uint32_t feedback_age = now_us - motor_feedback[id].last_rx_tick;

  return (motor_feedback[id].rx_count > 0U) &&
         (feedback_age <= C620_FEEDBACK_TIMEOUT_US) &&
         (abs(actual_rpm) <= C620_MAX_VALID_RPM);
}

