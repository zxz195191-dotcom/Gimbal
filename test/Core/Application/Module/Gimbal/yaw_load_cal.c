#include "yaw_load_cal.h"
#include <math.h>
#include <string.h>
#include "feedforward_saved.h"
YawCalStatus g_yaw_cal;
/* Committed model is independent of candidate samples and diagnostics. */
static float model_f;
static uint8_t model_ready;
static uint32_t started, quiet_since, last_rx;
static float previous_angle, previous_torque;
static uint8_t seen, quiet, previous_valid;
float YawCal_NodeAngle(unsigned i) { return g_yaw_cal.center_deg-YAW_CAL_NODE_HALF_DEG+5.0f*i; }
void YawCal_Init(void) {
    memset(&g_yaw_cal,0,sizeof(g_yaw_cal));
    model_f=0; model_ready=0; seen=quiet=previous_valid=0;
#if YAW_SAVED_PRESENT
    if (isfinite(yaw_saved_f_nm) && yaw_saved_f_nm>=0) {
        model_f=yaw_saved_f_nm; model_ready=1;
        g_yaw_cal.ready=1; g_yaw_cal.f_mean=model_f;
    }
#endif
}
uint8_t YawCal_Active(void) { return g_yaw_cal.state>=YCAL_SETTLE_LOW && g_yaw_cal.state<=YCAL_REVERSE; }
void YawCal_Stop(uint8_t reason) {
    if (YawCal_Active()) { g_yaw_cal.state=YCAL_ABORT; g_yaw_cal.error=reason; }
    g_yaw_cal.armed=0; g_yaw_cal.ready=model_ready;
}
static void phase(uint8_t state) {
    g_yaw_cal.state=state;
    g_yaw_cal.target_deg=g_yaw_cal.center_deg+
        ((state==YCAL_FORWARD || state==YCAL_SETTLE_HIGH)?YAW_CAL_TRAVEL_DEG:-YAW_CAL_TRAVEL_DEG);
    quiet=0; /* Keep previous actual feedback across command transitions. */
}
uint8_t YawCal_Start(uint32_t now,float angle) {
    if (!g_yaw_cal.armed || YawCal_Active() || !isfinite(angle)) return 0;
    memset(&g_yaw_cal,0,sizeof(g_yaw_cal));
    g_yaw_cal.ready=model_ready;
    g_yaw_cal.center_deg=angle; g_yaw_cal.speed_deg_s=5;
    for(unsigned i=0;i<YAW_CAL_NODES;++i) {
        g_yaw_cal.b[i]=g_yaw_cal.f[i]=NAN;
        g_yaw_cal.positive[0][i]=g_yaw_cal.negative[0][i]=NAN;
    }
    started=now; seen=quiet=previous_valid=0;
    phase(YCAL_SETTLE_LOW); return 1;
}
static void sample_crossings(const YawCalInput *in) {
    const uint8_t forward=g_yaw_cal.state==YCAL_FORWARD || g_yaw_cal.state==YCAL_SETTLE_HIGH;
    const uint8_t reverse=g_yaw_cal.state==YCAL_REVERSE ||
        (g_yaw_cal.state==YCAL_SETTLE_LOW && g_yaw_cal.lap==1);
    if (previous_valid && ((forward && in->angle_deg>previous_angle) ||
                           (reverse && in->angle_deg<previous_angle))) {
        unsigned d=reverse?1:0;
        for(unsigned i=0;i<YAW_CAL_NODES;++i) {
            if(g_yaw_cal.counts[0][d][i]) continue;
            const float node=YawCal_NodeAngle(i);
            if ((forward && previous_angle<=node && in->angle_deg>=node) ||
                (reverse && previous_angle>=node && in->angle_deg<=node)) {
                const float ratio=(node-previous_angle)/(in->angle_deg-previous_angle);
                const float torque=previous_torque+ratio*(in->torque_nm-previous_torque);
                if (!isfinite(torque)) continue;
                if(d) g_yaw_cal.negative[0][i]=torque;
                else g_yaw_cal.positive[0][i]=torque;
                g_yaw_cal.counts[0][d][i]=1;
            }
        }
    }
    previous_angle=in->angle_deg; previous_torque=in->torque_nm; previous_valid=1;
}
static uint8_t build(void) {
    float b=0,f=0; g_yaw_cal.paired=0;
    for(unsigned i=0;i<YAW_CAL_NODES;++i) {
        if (!g_yaw_cal.counts[0][0][i] || !g_yaw_cal.counts[0][1][i]) continue;
        const float p=g_yaw_cal.positive[0][i], n=g_yaw_cal.negative[0][i];
        g_yaw_cal.b[i]=(p+n)*.5f; g_yaw_cal.f[i]=(p-n)*.5f;
        b+=g_yaw_cal.b[i]; f+=g_yaw_cal.f[i]; ++g_yaw_cal.paired;
    }
    g_yaw_cal.b_mean=g_yaw_cal.paired?b/g_yaw_cal.paired:NAN;
    g_yaw_cal.f_mean=g_yaw_cal.paired?f/g_yaw_cal.paired:NAN;
    if(g_yaw_cal.paired<YAW_CAL_MIN_PAIRS) return YCAL_SAMPLES;
    if(!isfinite(g_yaw_cal.f_mean) || !isfinite(g_yaw_cal.b_mean) || g_yaw_cal.f_mean<0) return YCAL_SIGN;
    model_f=g_yaw_cal.f_mean; model_ready=1; g_yaw_cal.ready=1;
    return YCAL_OK;
}
int YawCal_Update(const YawCalInput *in) {
    if(!YawCal_Active()) return 0;
    if(!in->valid || !isfinite(in->angle_deg) || !isfinite(in->torque_nm) ||
        !isfinite(in->speed_deg_s) || !isfinite(in->command_deg)) { YawCal_Stop(YCAL_FEEDBACK); return -1; }
    if((uint32_t)(in->now_ms-started)>75000U) { YawCal_Stop(YCAL_TIMEOUT); return -1; }
    if(!seen || in->rx_count!=last_rx) { seen=1; last_rx=in->rx_count; sample_crossings(in); }
    if(fabsf(in->command_deg-g_yaw_cal.target_deg)>=.02f) { quiet=0; return 1; }
    if(g_yaw_cal.state==YCAL_FORWARD) { phase(YCAL_SETTLE_HIGH); return 1; }
    if(g_yaw_cal.state==YCAL_REVERSE) { g_yaw_cal.lap=1; phase(YCAL_SETTLE_LOW); return 1; }
    /* Existing finite dwell; actual crossing collection continues throughout it. */
    if(!quiet) { quiet=1; quiet_since=in->now_ms; }
    if((uint32_t)(in->now_ms-quiet_since)<500U) return 1;
    if(g_yaw_cal.state==YCAL_SETTLE_LOW && g_yaw_cal.lap==1) {
        uint8_t error=build();
        if(error) { YawCal_Stop(error); return -1; }
        g_yaw_cal.state=YCAL_DONE; return 0;
    }
    phase(g_yaw_cal.state==YCAL_SETTLE_LOW?YCAL_FORWARD:YCAL_REVERSE); return 1;
}
float YawCal_Friction(float angle) { return model_ready && isfinite(angle)?model_f:0; }
