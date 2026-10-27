#include "yaw_friction_goal.h"
#include <math.h>
#include <string.h>

#define RAD .017453292519943295f
#define EXIT_RAD (.10f*RAD)      /* Same arrival distance as the existing planner. */
#define REARM_RAD (.15f*RAD)     /* 0.05 deg hysteresis > two MIT position bins. */
#define BRAKE_MARGIN_RAD (.05f*RAD)
#define ZERO_RATE (30.0f/4095.0f+1e-6f)

void YawFrictionGoal_Reset(YawFrictionGoal *s) {
    memset(s,0,sizeof(*s));
    s->fresh_target=1;
}
float YawFrictionGoal_Update(YawFrictionGoal *s,float error,float rate,
                            float brake,uint8_t arrived,float f) {
    if (!isfinite(error) || !isfinite(rate) || !isfinite(brake) ||
        !isfinite(f) || brake<0 || f<0) {
        YawFrictionGoal_Reset(s);
        return 0;
    }
    const float distance=fabsf(error);
    const int8_t direction=error>=0?1:-1;
    if (arrived) {
        s->correcting=s->braking=s->fresh_target=0;
        s->direction=0;
        return 0;
    }
    if (!s->correcting) {
        const float enter=s->fresh_target?EXIT_RAD:REARM_RAD;
        s->fresh_target=0;
        if (distance<=enter) return 0;
        s->correcting=1;
    }
    if (s->direction!=direction) {
        s->direction=direction;
        s->braking=0;
    }
    /* Use exactly the planner's zero-bin treatment, not its attenuated v_des. */
    const float measured=fabsf(rate)<=ZERO_RATE?0:rate;
    const float toward=direction*measured;
    const float remaining=fmaxf(0,distance-EXIT_RAD);
    if (toward<=0) s->braking=0;
    else if (brake>=remaining) s->braking=1;
    else if (s->braking && brake+BRAKE_MARGIN_RAD<remaining) s->braking=0;

    /* Brake/arrival explicitly withdraw assistance. Otherwise incomplete
     * correction may establish F even when drive/v_des are small. The caller
     * still applies the original slew, reversal handling and torque limits. */
    return s->braking?0:direction*f;
}
