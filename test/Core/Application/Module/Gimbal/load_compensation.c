#include "load_compensation.h"
#include <math.h>

static float bound(float x, float lo, float hi) { return fminf(hi, fmaxf(lo, x)); }
static float approach(float x, float y, float step) { return x + bound(y-x, -step, step); }
void LoadComp_Reset(LoadCompState *s) {
    s->dwell = 0;
    s->integral_nm = s->friction_nm = s->output_nm = 0;
}
float LoadComp_Update(LoadCompState *s, const LoadCompConfig *c,
                      float base_nm, float friction_goal_nm, float error_rad,
                      float actual_rate, float desired_rate, float pd_nm,
                      uint32_t dt_ms, uint8_t holding, uint8_t eligible) {
    if (!eligible || !isfinite(base_nm) || !isfinite(friction_goal_nm) ||
        !isfinite(error_rad) || !isfinite(actual_rate) || !isfinite(desired_rate) ||
        !isfinite(pd_nm) || dt_ms == 0 || dt_ms > 20U) {
        LoadComp_Reset(s);
        return isfinite(base_nm) ? base_nm : 0;
    }
    const float dt = dt_ms * .001f, step = c->slew_nm_s * dt;
    s->friction_nm = approach(s->friction_nm,
        bound(friction_goal_nm, -c->additional_limit_nm, c->additional_limit_nm), step);
    const uint8_t quiet = holding && fabsf(desired_rate) < .001f &&
        fabsf(actual_rate) <= c->hold_speed_rad_s &&
        fabsf(error_rad) <= c->hold_error_rad && fabsf(s->friction_nm) <= .01f;
    if (!s->integral_enabled || !quiet) {
        s->dwell = 0;
        s->integral_nm = approach(s->integral_nm, 0, step);
    } else {
        if (s->dwell < c->dwell_ms) s->dwell += dt_ms;
        if (s->dwell >= c->dwell_ms && fabsf(error_rad) > c->deadband_rad) {
            const float candidate = bound(s->integral_nm + c->ki * error_rad * dt,
                                          -c->integral_limit_nm, c->integral_limit_nm);
            const float change = candidate - s->integral_nm;
            const float extra = s->friction_nm + candidate;
            const float total = base_nm + extra;
            // Conditional integration: allow unwinding but not deeper saturation,
            // including estimated driver PD + commanded feedforward saturation.
            if (!((change > 0 && (extra > c->additional_limit_nm ||
                    total > c->total_limit_nm || total + pd_nm > 10.0f)) ||
                  (change < 0 && (extra < -c->additional_limit_nm ||
                    total < -c->total_limit_nm || total + pd_nm < -10.0f))))
                s->integral_nm = candidate;
        }
    }
    const float extra = bound(s->friction_nm + s->integral_nm,
                             -c->additional_limit_nm, c->additional_limit_nm);
    s->output_nm = bound(base_nm + extra, -c->total_limit_nm, c->total_limit_nm);
    return s->output_nm;
}

