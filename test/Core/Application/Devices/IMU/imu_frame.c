#include "imu_frame.h"
#include <stddef.h>

static int16_t read_be16(const uint8_t *p) {
    const uint32_t bits = ((uint32_t)p[0] << 8) | p[1];
    return (int16_t)((bits >= 32768U) ? (int32_t)bits - 65536 : (int32_t)bits);
}

uint8_t IMU_DecodeFrame(const uint8_t frame[15], IMU_Sample_t *sample) {
    if ((frame == NULL) || (sample == NULL)) return 0U;
    uint8_t all_zero = 1U, all_ff = 1U;
    sample->flags = 0U;
    for (unsigned i = 0; i < 15U; ++i) {
        if (frame[i] != 0U) all_zero = 0U;
        if (frame[i] != 0xFFU) all_ff = 0U;
    }
    if (all_zero || all_ff) sample->flags |= IMU_FRAME_BAD_BUS;
    if ((frame[0] & 1U) == 0U) sample->flags |= IMU_FRAME_NO_DRDY;
    for (unsigned axis = 0; axis < 3U; ++axis) {
        const int16_t accel = read_be16(&frame[1U + axis * 2U]);
        const int16_t gyro = read_be16(&frame[9U + axis * 2U]);
        sample->accel_raw[axis] = accel;
        sample->gyro_raw[axis] = gyro;
        /* ACCEL_CONFIG=0x10: +/-8g; GYRO_CONFIG=0x18: +/-2000 deg/s. */
        sample->accel_mps2[axis] = (float)accel * (9.80665f / 4096.0f);
        sample->gyro_rad_s[axis] = (float)gyro * (0.017453292519943295f / 16.4f);
        if ((accel >= 32760) || (accel <= -32760) ||
            (gyro >= 32760) || (gyro <= -32760)) {
            sample->flags |= IMU_FRAME_SATURATED;
        }
    }
    sample->temperature_raw = read_be16(&frame[7]);
    sample->temperature_c = 21.0f + (float)sample->temperature_raw / 333.87f;
    sample->valid = (sample->flags == 0U);
    return sample->valid;
}

float IMU_CycleDeltaSeconds(uint32_t previous, uint32_t current,
                           uint32_t core_clock_hz) {
    if (core_clock_hz == 0U) return 0.0f;
    /* Subtract before converting units: CYCCNT wraps every ~23.86 s at 180MHz. */
    return (float)(uint32_t)(current - previous) / (float)core_clock_hz;
}
