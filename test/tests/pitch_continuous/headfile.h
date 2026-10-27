#pragma once
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
typedef int CAN_HandleTypeDef;
typedef int CAN_RxHeaderTypeDef;
typedef int HAL_StatusTypeDef;
typedef int GPIO_PinState;
#define HAL_OK 0
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define KEY_GPIO_Port 0
#define KEY_Pin 0
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
extern CAN_HandleTypeDef hcan2;
extern uint32_t g_can2_filter_ret, g_can2_start_ret, g_can2_notify_ret;
uint32_t HAL_GetTick(void);
GPIO_PinState HAL_GPIO_ReadPin(int port, int pin);
HAL_StatusTypeDef dm4310_disable(CAN_HandleTypeDef *, uint16_t);
HAL_StatusTypeDef dm4310_enable(CAN_HandleTypeDef *, uint16_t);
HAL_StatusTypeDef dm4310_clear_error(CAN_HandleTypeDef *, uint16_t);
#include "dm_motor.h"
#include "gimbal.h"
#include "pitch_calibration.h"
