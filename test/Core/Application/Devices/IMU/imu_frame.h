#ifndef IMU_FRAME_H
#define IMU_FRAME_H

#include "imu_mpu6500.h"

/* 15 bytes from INT_STATUS (0x3A): status, accel[6], temp[2], gyro[6]. */
uint8_t IMU_DecodeFrame(const uint8_t frame[15], IMU_Sample_t *sample);
float IMU_CycleDeltaSeconds(uint32_t previous, uint32_t current,
                           uint32_t core_clock_hz);

#endif
