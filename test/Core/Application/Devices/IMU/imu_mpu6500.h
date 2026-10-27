#ifndef IMU_MPU6500_H
#define IMU_MPU6500_H

#include <stdint.h>

/* Raw acquisition: native SENSOR axes, not gimbal axes. No bias subtraction.
 * Units: angular velocity rad/s, acceleration m/s^2, temperature Celsius.
 * No attitude output is implied by sample.valid. */
typedef struct {
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t temperature_raw;
    float accel_mps2[3];
    float gyro_rad_s[3];
    float temperature_c;
    float dt_s;
    uint32_t timestamp_cycles;
    uint32_t timestamp_ms;
    uint32_t sequence;
    uint8_t flags;
    uint8_t valid;
} IMU_Sample_t;

enum {
    IMU_FRAME_NO_DRDY = 1U,
    IMU_FRAME_BAD_BUS = 2U,
    IMU_FRAME_SATURATED = 4U,
    IMU_FRAME_BAD_DT = 8U
};

typedef enum {
    IMU_OFF = 0, IMU_RESET, IMU_WAIT_RESET, IMU_WAIT_WAKE,
    IMU_CONFIGURE, IMU_WAIT_DATA, IMU_RUNNING, IMU_FAULT
} IMU_State_t;

typedef enum {
    IMU_ERROR_NONE = 0, IMU_ERROR_TIMEBASE, IMU_ERROR_SPI,
    IMU_ERROR_ID, IMU_ERROR_READBACK, IMU_ERROR_NO_DATA,
    IMU_ERROR_DMA_TIMEOUT, IMU_ERROR_BAD_FRAMES
} IMU_Error_t;

typedef struct {
    IMU_State_t state;
    IMU_Error_t error;
    uint8_t who_am_i;
    uint8_t failed_register;
    uint8_t expected_value;
    uint8_t readback_value;
    uint32_t drdy_count;
    uint32_t dma_count;
    uint32_t busy_drop_count;
    uint32_t queue_overrun_count;
    uint32_t stale_drop_count;
    uint32_t invalid_frame_count;
    uint32_t timing_gap_count;
    uint32_t spi_error_count;
    uint32_t hal_spi_error;
    uint32_t valid_count;
    uint32_t sample_age_ms;
    uint32_t dt_min_cycles;
    uint32_t dt_max_cycles;
    float sample_rate_hz;
    uint8_t queue_pending;
    IMU_Sample_t sample;
} IMU_Debug_t;

/* Debugger watch point. Mutated by ISR and main; use IMU_GetSample in MAIN. */
extern volatile IMU_Debug_t g_imu_debug;
/* 0: gimbal; 1: raw; 2: health; 3: attitude; 4: calibration/heat; 5: capture. */
extern volatile uint8_t g_imu_log_mode;

/* Main-loop APIs. Init/restart require DWT and MX_SPI5_Init already complete.
 * Init waits for reset/wakeup through Service, never HAL_Delay. */
void IMU_Init(void);
void IMU_Service(void);
uint8_t IMU_GetSample(IMU_Sample_t *sample);
void IMU_RequestRestart(void);
/* Main-context consumer: receives EVERY decoded frame, including invalid ones.
 * NULL sample signals stream restart. Registration survives IMU_Init. */
typedef void (*IMU_SampleCallback_t)(const IMU_Sample_t *sample);
void IMU_SetSampleCallback(IMU_SampleCallback_t callback);

/* Called from central HAL callbacks, only for SPI5 / MPU6500_INT. */
void IMU_OnDataReady(void);
void IMU_OnTransferComplete(void);
void IMU_OnSpiError(uint32_t error);

#endif
