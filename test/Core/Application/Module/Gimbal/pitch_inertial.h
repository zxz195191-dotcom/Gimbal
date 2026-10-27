#pragma once
#include <stdint.h>

/* Encoder-positive rotation about native sensor +Y. This matches the current
 * p_enc / displayed imu_pitch direction; NOT the pitch-up Euler convention.
 * Change only after checking installation and encoder increments on hardware. */
#ifndef PITCH_ENCODER_ROTATION_SIGN
#define PITCH_ENCODER_ROTATION_SIGN 1.0f
#endif
typedef enum { PITCH_MODE_ENCODER=0, PITCH_MODE_INERTIAL=1 } Pitch_Control_Mode_t;
typedef enum {
    PI_OK=0, PI_AXIS, PI_BUSY, PI_IMU, PI_REFERENCE, PI_GEOMETRY, PI_BOUNDS
} Pitch_Inertial_Reason_t;
typedef struct {
    Pitch_Control_Mode_t mode;
    Pitch_Inertial_Reason_t reason;
    float target_elevation_rad, elevation_rad, correction_rad, encoder_target_rad;
    float elevation_per_encoder_rad;
    uint32_t reference_id, age_ms;
    uint8_t valid, limited;
} Pitch_Inertial_Status_t;
extern volatile Pitch_Inertial_Status_t g_pitch_inertial;
void Pitch_Inertial_Init(void);
void Pitch_Inertial_Reset(void);
/* Refresh report-only observation without changing the captured goal or reason. */
void Pitch_Inertial_Observe(uint32_t now_ms);
uint8_t Pitch_Inertial_Capture(uint32_t now_ms, float encoder_rad);
uint8_t Pitch_Inertial_Correct(uint32_t now_ms, float encoder_rad,
                              float min_rad, float max_rad, float *target_rad);
/* Quaternion body->NWU world; forward=body X, joint axis=body Y. Returns the
 * nearest encoder increment solving world elevation, not an Euler difference. */
uint8_t Pitch_Inertial_Geometry(const float q[4], float target_elevation_rad,
                               float *elevation_rad, float *increment_rad,
                               float *elevation_per_encoder_rad);
