#include "pitch_inertial.h"
#include "imu_attitude.h"
#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846f
#define MAX_ELEVATION (80.0f*PI/180.0f)
volatile Pitch_Inertial_Status_t g_pitch_inertial;
static float bound(float x,float a,float b) { return fminf(b,fmaxf(a,x)); }
static float wrap(float x) {
    if(x>PI) x-=2*PI;
    if(x< -PI) x+=2*PI;
    return x;
}
uint8_t Pitch_Inertial_Geometry(const float q[4],float goal,float *elevation,
                               float *increment,float *gain) {
    float n=0;
    for(unsigned i=0;i<4;i++) {
        if(!isfinite(q[i])) return 0;
        n+=q[i]*q[i];
    }
    if(!isfinite(n) || n<.5f || n>1.5f || !isfinite(goal) ||
       fabsf(goal)>MAX_ELEVATION ||
       fabsf(PITCH_ENCODER_ROTATION_SIGN)!=1.0f) return 0;
    const float inv=1/sqrtf(n);
    const float w=q[0]*inv,x=q[1]*inv,y=q[2]*inv,z=q[3]*inv;
    /* World vertical of forward after joint rotation d:
     * z(d)=R20*cos(d)-sign*R22*sin(d). This includes current yaw/roll tilt. */
    const float a=bound(2*(x*z-w*y),-1,1);
    const float b=-PITCH_ENCODER_ROTATION_SIGN*(1-2*(x*x+y*y));
    const float horizontal=sqrtf(fmaxf(0,1-a*a));
    *elevation=atan2f(a,horizontal);
    if(fabsf(*elevation)>MAX_ELEVATION || horizontal<.17f) return 0;
    *gain=b/horizontal;
    /* No useful pitch authority with the joint axis close to vertical. */
    if(fabsf(*gain)<.25f) return 0;
    const float amplitude=hypotf(a,b), requested=sinf(goal);
    if(amplitude<.17f || fabsf(requested)>amplitude+1e-6f) return 0;
    const float phase=atan2f(b,a), offset=acosf(bound(requested/amplitude,-1,1));
    const float d1=wrap(phase+offset),d2=wrap(phase-offset);
    *increment=fabsf(d1)<=fabsf(d2)?d1:d2;
    return isfinite(*increment);
}
void Pitch_Inertial_Init(void) {
    memset((void *)&g_pitch_inertial,0,sizeof(g_pitch_inertial));
}
void Pitch_Inertial_Reset(void) {
    g_pitch_inertial.mode=PITCH_MODE_ENCODER;
    g_pitch_inertial.correction_rad=0;
    g_pitch_inertial.limited=0;
}
void Pitch_Inertial_Observe(uint32_t now) {
    IMU_AttitudeSample_t s;
    const uint8_t ok=IMU_AttitudeGet(now,&s);
    g_pitch_inertial.valid=ok && s.age_ms<=5;
    g_pitch_inertial.age_ms=s.age_ms;
    float elevation,increment,gain;
    if(g_pitch_inertial.valid && Pitch_Inertial_Geometry(s.quaternion_wxyz,0,
           &elevation,&increment,&gain)) {
        g_pitch_inertial.elevation_rad=elevation;
        g_pitch_inertial.elevation_per_encoder_rad=gain;
    }
}
static uint8_t sample(uint32_t now,IMU_AttitudeSample_t *s) {
    const uint8_t ok=IMU_AttitudeGet(now,s);
    g_pitch_inertial.age_ms=s->age_ms;
    g_pitch_inertial.valid=ok && s->age_ms<=5;
    if(!g_pitch_inertial.valid) g_pitch_inertial.reason=PI_IMU;
    return g_pitch_inertial.valid;
}
uint8_t Pitch_Inertial_Capture(uint32_t now,float encoder) {
    IMU_AttitudeSample_t s;
    if(!sample(now,&s)) return 0;
    float elevation,increment,gain;
    if(!isfinite(encoder) ||
       !Pitch_Inertial_Geometry(s.quaternion_wxyz,0,&elevation,&increment,&gain)) {
        g_pitch_inertial.reason=PI_GEOMETRY; return 0;
    }
    g_pitch_inertial.reference_id=s.reference_id;
    g_pitch_inertial.target_elevation_rad=g_pitch_inertial.elevation_rad=elevation;
    g_pitch_inertial.elevation_per_encoder_rad=gain;
    g_pitch_inertial.encoder_target_rad=encoder;
    g_pitch_inertial.correction_rad=0;
    g_pitch_inertial.limited=0;
    g_pitch_inertial.reason=PI_OK;
    g_pitch_inertial.mode=PITCH_MODE_INERTIAL;
    return 1;
}
uint8_t Pitch_Inertial_Correct(uint32_t now,float encoder,float lo,float hi,float *target) {
    IMU_AttitudeSample_t s;
    if(!sample(now,&s)) return 0;
    if(s.reference_id!=g_pitch_inertial.reference_id) {
        g_pitch_inertial.reason=PI_REFERENCE; return 0;
    }
    float elevation,increment,gain;
    if(!Pitch_Inertial_Geometry(s.quaternion_wxyz,
        g_pitch_inertial.target_elevation_rad,&elevation,&increment,&gain)) {
        g_pitch_inertial.reason=PI_GEOMETRY; return 0;
    }
    if(!isfinite(encoder) || !isfinite(lo) || !isfinite(hi) || lo>hi) {
        g_pitch_inertial.reason=PI_BOUNDS; return 0;
    }
    const float raw=encoder+increment;
    *target=bound(raw,lo,hi);
    g_pitch_inertial.elevation_rad=elevation;
    g_pitch_inertial.correction_rad=increment;
    g_pitch_inertial.elevation_per_encoder_rad=gain;
    g_pitch_inertial.encoder_target_rad=*target;
    g_pitch_inertial.limited=raw<lo || raw>hi;
    g_pitch_inertial.reason=PI_OK;
    return 1;
}
