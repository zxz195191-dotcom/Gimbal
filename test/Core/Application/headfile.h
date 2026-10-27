#pragma once
#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include "stdlib.h"
#include "ctype.h"
#include "stddef.h"
#include "stdbool.h"

#include "pid.h"
#include "motor.h"
#include "app.h"
#include "dm_motor.h"
#include "pid.h"
#include "gimbal.h"
#include "can_dispatch.h"
#include "vision_protocol.h"
#include "shooter.h"
#include "saftery.h"
#include "pitch_calibration.h"


#define P 6.0f
#define I 0.2f
#define D 0.0f
#define MAX_Out 6000.0f
#define MAX_Integral 5000.0f
#define CONTROL_PERIOD_US  5000U   // 5ms控制周期
#define CAN_TX_PERIOD_US   2000U   // 2ms发一次CAN
#define VISION_TX_PERIOD_US 5000U  // 5ms向NUC发送一帧
#define KEY_SCAN_PERIOD_US 10000U  // 10ms扫一次按键
/* Telemetry only; does not change motor control or IMU sample periods. */
#ifndef TELEMETRY_HZ
#define TELEMETRY_HZ 50U
#endif
#if TELEMETRY_HZ != 10U && TELEMETRY_HZ != 20U && TELEMETRY_HZ != 50U
#error "TELEMETRY_HZ must be 10, 20, or 50"
#endif
#define DBG_PRINT_PERIOD_US (1000000U / TELEMETRY_HZ)
#define C620_FEEDBACK_TIMEOUT_US 20000U
#define C620_MAX_VALID_RPM       12000
#define FRICTION_MAX_RPM         9000.0f
