#include "headfile.h"

volatile uint32_t g_can1_filter_ret = HAL_ERROR;
volatile uint32_t g_can1_start_ret = HAL_ERROR;
volatile uint32_t g_can1_notify_ret = HAL_ERROR;
volatile uint32_t g_can2_filter_ret = HAL_ERROR;
volatile uint32_t g_can2_start_ret = HAL_ERROR;
volatile uint32_t g_can2_notify_ret = HAL_ERROR;

void Can_Dispatch_Init(void) {
  CAN_FilterTypeDef can1_filter_st = {0};
  can1_filter_st.FilterActivation = ENABLE;
  can1_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
  can1_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
  can1_filter_st.FilterIdHigh = 0x0000;
  can1_filter_st.FilterIdLow = 0x0000;
  can1_filter_st.FilterMaskIdHigh = 0x0000;
  can1_filter_st.FilterMaskIdLow = 0x0000;
  can1_filter_st.FilterBank = 0;
  can1_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
  can1_filter_st.SlaveStartFilterBank = 14;

  g_can1_filter_ret = HAL_CAN_ConfigFilter(&hcan1, &can1_filter_st);

  /*从 Bank 14 开始划给 CAN2  (can1,2的 Fliter Bank是共享的)*/
  CAN_FilterTypeDef can2_filter_st = {0};
  can2_filter_st.FilterActivation = ENABLE;
  can2_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
  can2_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
  can2_filter_st.FilterIdHigh = 0x0000;
  can2_filter_st.FilterIdLow = 0x0000;
  can2_filter_st.FilterMaskIdHigh = 0x0000;
  can2_filter_st.FilterMaskIdLow = 0x0000;
  can2_filter_st.FilterBank = 14;
  can2_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
  can2_filter_st.SlaveStartFilterBank = 14;

  g_can2_filter_ret = HAL_CAN_ConfigFilter(&hcan2, &can2_filter_st);
  g_can1_start_ret = HAL_CAN_Start(&hcan1);
  g_can2_start_ret = HAL_CAN_Start(&hcan2);

  if (g_can1_start_ret == HAL_OK) {
    g_can1_notify_ret = HAL_CAN_ActivateNotification(
        &hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  }
  if (g_can2_start_ret == HAL_OK) {
    g_can2_notify_ret = HAL_CAN_ActivateNotification(
        &hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  }

}

HAL_StatusTypeDef Can_Send_Std(CAN_HandleTypeDef *hcan,uint16_t std_id,uint8_t *data,uint8_t len) {
  CAN_TxHeaderTypeDef tx_header = {0};
  /*tx_data 是软件在内存（SRAM）里定义的一块数据缓存；Mailbox 是芯片 CAN 外设里的硬件寄存器。数据必须从 SRAM 拷贝到硬件寄存器，CAN 发送引擎才能把它通过 CAN_TX 引脚发送到总线。*/
  uint32_t tx_mailbox;

  if ((hcan == NULL) || (data == NULL) || (std_id > 0x7FFU) || (len > 8U)) {
    return HAL_ERROR;
  }
  if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U) {
    return HAL_BUSY;
  }

  tx_header.StdId = std_id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = len;
  //防止时间戳覆盖电机控制数据
  tx_header.TransmitGlobalTime = DISABLE;//默认在data[6]和[7]填充发送时的时间

  return HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox);
}

// 使能电机
HAL_StatusTypeDef dm4310_enable(CAN_HandleTypeDef *hcan,uint16_t motor_id) {
  uint8_t enable_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  return Can_Send_Std(hcan,motor_id,enable_data,8);
}
// 失能电机
HAL_StatusTypeDef dm4310_disable(CAN_HandleTypeDef *hcan, uint16_t motor_id) {
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
  return Can_Send_Std(hcan, motor_id, data,8);
}

HAL_StatusTypeDef dm4310_clear_error(CAN_HandleTypeDef *hcan, uint16_t motor_id) {
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
  return Can_Send_Std(hcan, motor_id, data,8);
}

// 保存当前位置为零点
HAL_StatusTypeDef dm4310_set_zero(CAN_HandleTypeDef *hcan, uint16_t motor_id) {
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
  return Can_Send_Std(hcan, motor_id, data,8);
}

