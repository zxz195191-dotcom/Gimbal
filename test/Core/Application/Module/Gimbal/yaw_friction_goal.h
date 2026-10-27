#pragma once
#include <stdint.h>

/* Selects the existing Coulomb feedforward goal; no extra torque controller.
 * Angles rad, joint rate rad/s, F logical joint N.m. */
typedef struct {
    uint8_t correcting, braking, fresh_target;
    int8_t direction;
} YawFrictionGoal;
void YawFrictionGoal_Reset(YawFrictionGoal *state);
float YawFrictionGoal_Update(YawFrictionGoal *state,float final_error_rad,
    float actual_rate_rad_s,float brake_distance_rad,uint8_t arrived,float f_nm);
