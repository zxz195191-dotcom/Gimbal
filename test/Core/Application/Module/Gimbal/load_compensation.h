#pragma once
#include <stdint.h>

/* Logical joint coordinates: rad, rad/s, N.m at the joint output.
 * Pitch's proven B/F table is supplied as base_nm, never re-identified here. */
typedef struct {
    float ki, integral_limit_nm, additional_limit_nm, total_limit_nm;
    float slew_nm_s, hold_speed_rad_s, hold_error_rad, deadband_rad;
    uint32_t dwell_ms;
} LoadCompConfig;
typedef struct {
    uint8_t integral_enabled;
    uint32_t dwell;
    float integral_nm, friction_nm, output_nm;
} LoadCompState;
void LoadComp_Reset(LoadCompState *s);
float LoadComp_Update(LoadCompState *s, const LoadCompConfig *c,
                      float base_nm, float friction_goal_nm, float error_rad,
                      float actual_rate, float desired_rate, float pd_nm,
                      uint32_t dt_ms, uint8_t holding, uint8_t eligible);

