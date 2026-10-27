#pragma once
#include <stdint.h>

/* 炮口前向在传感器坐标中的单位向量。沿用现有安装：X前、Y左、Z上。
 * 改安装时改此向量；不修改 IMU 融合使用的原生右手坐标。 */
#define YAW_FORWARD_SENSOR_X 1.0f
#define YAW_FORWARD_SENSOR_Y 0.0f
#define YAW_FORWARD_SENSOR_Z 0.0f
/* 逻辑Yaw正向相对左转的符号；电机接线方向仍由GIMBAL_YAW_DIRECTION处理。 */
#ifndef YAW_IMU_DIRECTION
#define YAW_IMU_DIRECTION 1.0f
#endif
/* 与dm_motor.c现有MIT协议量程一致，不代表电机持续额定力矩。 */
#define YAW_MIT_TORQUE_MAX_NM 10.0f

typedef enum { YAW_MODE_ENCODER=0, YAW_MODE_INERTIAL=1 } Yaw_Control_Mode_t;
typedef struct {
    Yaw_Control_Mode_t mode;
    float relative_rad, rate_rad_s, encoder_origin_rad;
    uint32_t timestamp_ms, age_ms, reference_id;
    uint8_t valid;
    int8_t direction; /* +1左转为正，-1右转为正 */
    uint8_t switch_result; /* 0成功，1轴未使能，2姿态不可用，3模式参数错误 */
} Yaw_Inertial_Status_t;
extern volatile Yaw_Inertial_Status_t g_yaw_inertial;
void Yaw_Inertial_Init(void);
void Yaw_Inertial_Update(uint32_t now_ms);
void Yaw_Inertial_Zero(float encoder_rad);
