#ifndef IMU_ATTITUDE_H
#define IMU_ATTITUDE_H
#include "imu_mpu6500.h"

typedef enum {
    IMU_ATT_WAIT = 0, IMU_ATT_CALIBRATING, IMU_ATT_ALIGNING,
    IMU_ATT_READY, IMU_ATT_FAULT
} IMU_AttitudeState_t;
typedef enum {
    IMU_ATT_OK = 0, IMU_ATT_CAL_TIMEOUT, IMU_ATT_STREAM_GAP,
    IMU_ATT_TEMPERATURE_CHANGED, IMU_ATT_NUMERIC
} IMU_AttitudeError_t;
typedef struct {
    IMU_AttitudeState_t state;
    IMU_AttitudeError_t error;
    uint8_t valid, calibrated, accel_rejected, thermal_ready;
    uint32_t sample_count, window_count, motion_resets, gap_count, timestamp_ms;
    float roll_deg, pitch_up_deg, yaw_left_deg;
    float gyro_rad_s[3], bias_rad_s[3], quaternion[4];
    float calibration_temperature_c, temperature_c, progress;
    float accel_norm_mps2;
    double yaw_continuous_deg;
} IMU_AttitudeDebug_t;

/* Public snapshot, MAIN context only (not ISR/RTOS-safe). Angles radians,
 * body angular velocity rad/s, acceleration m/s^2, quaternion order w,x,y,z.
 * Quaternion maps the native right-handed sensor/body frame to NWU world.
 * pitch_up is NEGATED ZYX pitch; quaternion and gyro axes are not negated.
 * Body gyro Z is NOT Euler yaw rate when the board is tilted.
 * All sample fields refer to the LAST FUSION UPDATE, not the latest raw frame. */
typedef struct {
    float quaternion_wxyz[4];
    float gyro_body_rad_s[3];
    float accel_body_mps2[3];
    float roll_rad, pitch_up_rad, yaw_left_rad;
    double yaw_continuous_rad;
    float temperature_c, dt_s;
    uint32_t timestamp_ms, timestamp_cycles, sequence, age_ms;
    /* Changes on init/recalibration; consumers must discard old relative origins.
     * RAM session counter only: does not persist across an MCU power cycle. */
    uint32_t reference_id;
    IMU_AttitudeState_t state;
    IMU_AttitudeError_t error;
    uint8_t valid, heading_continuous_valid, accel_rejected, thermal_ready;
} IMU_AttitudeSample_t;
extern volatile IMU_AttitudeDebug_t g_imu_attitude;
void IMU_AttitudeInit(void);
/* Main-context only. Restart or data loss never silently re-zeros heading. */
void IMU_AttitudeProcess(const IMU_Sample_t *sample);
void IMU_AttitudeService(uint32_t now_ms, uint8_t stream_ok, uint8_t thermal_ready);
/* Queues request from command parser; permits a ROOM-TEMPERATURE calibration.
 * Fix the board motionless; slow constant rotation is not observable as bias. */
void IMU_AttitudeRequestCalibration(void);
/* Returns 1 only for READY data no older than 5 ms. Always fills out when
 * non-NULL, including fault/status/age; stale angles are never valid feedback.
 * now_ms must be the same HAL_GetTick clock used by the acquisition driver. */
uint8_t IMU_AttitudeGet(uint32_t now_ms, IMU_AttitudeSample_t *out);
#endif
