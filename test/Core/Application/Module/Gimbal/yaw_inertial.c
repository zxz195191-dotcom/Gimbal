#include "yaw_inertial.h"
#include "imu_attitude.h"
#include <math.h>
#include <string.h>

volatile Yaw_Inertial_Status_t g_yaw_inertial;
static double continuous_rad, origin_rad;
static float last_heading_rad;
static uint32_t last_sequence;
static uint8_t seen;

void Yaw_Inertial_Init(void) {
    memset((void *)&g_yaw_inertial,0,sizeof(g_yaw_inertial));
    continuous_rad=origin_rad=0;
    last_heading_rad=0;
    last_sequence=0;
    seen=0;
    g_yaw_inertial.direction=(int8_t)YAW_IMU_DIRECTION;
}

void Yaw_Inertial_Zero(float encoder_rad) {
    origin_rad=continuous_rad;
    g_yaw_inertial.relative_rad=0;
    g_yaw_inertial.encoder_origin_rad=encoder_rad;
}

void Yaw_Inertial_Update(uint32_t now_ms) {
    IMU_AttitudeSample_t s;
    const uint8_t valid=IMU_AttitudeGet(now_ms,&s);
    g_yaw_inertial.timestamp_ms=s.timestamp_ms;
    g_yaw_inertial.age_ms=s.age_ms;
    g_yaw_inertial.valid=0;
    if (g_yaw_inertial.reference_id!=s.reference_id) {
        g_yaw_inertial.reference_id=s.reference_id;
        seen=0;
    }
    if (!valid) return; /* 沿用IMU接口的新鲜度/故障判定，旧值不冒充实时反馈。 */
    if (seen && last_sequence==s.sequence) {
        g_yaw_inertial.valid=1;
        return;
    }
    const float w=s.quaternion_wxyz[0], x=s.quaternion_wxyz[1];
    const float y=s.quaternion_wxyz[2], z=s.quaternion_wxyz[3];
    const float r[2][3]={
        {1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)},
        {2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)}};
    const float f[3]={YAW_FORWARD_SENSOR_X,YAW_FORWARD_SENSOR_Y,YAW_FORWARD_SENSOR_Z};
    const float *gyro=s.gyro_body_rad_s;
    /* 世界前向u=R*f；其导数du=R*(gyro叉乘f)，包含Pitch转动后的三轴贡献。 */
    const float df[3]={gyro[1]*f[2]-gyro[2]*f[1],
                       gyro[2]*f[0]-gyro[0]*f[2],
                       gyro[0]*f[1]-gyro[1]*f[0]};
    float u[2]={0},du[2]={0};
    for (unsigned i=0;i<2;i++) for (unsigned j=0;j<3;j++) {
        u[i]+=r[i][j]*f[j]; du[i]+=r[i][j]*df[j];
    }
    const float horizontal_squared=u[0]*u[0]+u[1]*u[1];
    if (!isfinite(horizontal_squared) || horizontal_squared<1e-6f) return;
    const float heading=atan2f(u[1],u[0]);
    const float rate=g_yaw_inertial.direction*(u[0]*du[1]-u[1]*du[0])/horizontal_squared;
    if (!isfinite(heading) || !isfinite(rate)) return;
    if (!seen) {
        continuous_rad=g_yaw_inertial.direction*heading;
        origin_rad=continuous_rad;
    } else {
        float delta=heading-last_heading_rad;
        if (delta>3.14159265359f) delta-=6.28318530718f;
        if (delta< -3.14159265359f) delta+=6.28318530718f;
        continuous_rad+=g_yaw_inertial.direction*delta;
    }
    last_heading_rad=heading;
    last_sequence=s.sequence;
    seen=1;
    g_yaw_inertial.relative_rad=(float)(continuous_rad-origin_rad);
    g_yaw_inertial.rate_rad_s=rate;
    g_yaw_inertial.valid=1;
}
