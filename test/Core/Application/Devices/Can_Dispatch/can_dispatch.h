
#pragma once
#include "headfile.h"

extern volatile uint32_t g_can1_filter_ret;
extern volatile uint32_t g_can1_start_ret;
extern volatile uint32_t g_can1_notify_ret;
extern volatile uint32_t g_can2_filter_ret;
extern volatile uint32_t g_can2_start_ret;
extern volatile uint32_t g_can2_notify_ret;

void Can_Dispatch_Init(void);
HAL_StatusTypeDef Can_Send_Std(CAN_HandleTypeDef *hcan, uint16_t std_id,
                               uint8_t *data, uint8_t len);
HAL_StatusTypeDef dm4310_enable(CAN_HandleTypeDef *hcan, uint16_t motor_id);
HAL_StatusTypeDef dm4310_disable(CAN_HandleTypeDef *hcan, uint16_t motor_id);
HAL_StatusTypeDef dm4310_clear_error(CAN_HandleTypeDef *hcan, uint16_t motor_id);
HAL_StatusTypeDef dm4310_set_zero(CAN_HandleTypeDef *hcan, uint16_t motor_id);
