#ifndef IMU_HEATER_H
#define IMU_HEATER_H
#include "imu_mpu6500.h"
typedef enum { IMU_HEAT_OFF = 0, IMU_HEAT_WARMING, IMU_HEAT_STABLE, IMU_HEAT_FAULT } IMU_HeatState_t;
typedef enum { IMU_HEAT_OK = 0, IMU_HEAT_SENSOR, IMU_HEAT_OVER_TEMP,
               IMU_HEAT_NO_RISE, IMU_HEAT_TIMEOUT } IMU_HeatError_t;
typedef struct {
    IMU_HeatState_t state;
    IMU_HeatError_t error;
    uint8_t enabled, stable;
    float temperature_c, target_c, duty;
    uint32_t timestamp_ms;
} IMU_HeaterDebug_t;
extern volatile IMU_HeaterDebug_t g_imu_heater;
void IMU_HeaterInit(void);
/* Main context; starts with zero output, faults latch until explicit enable. */
void IMU_HeaterSetEnabled(uint8_t enabled);
void IMU_HeaterService(uint32_t now_ms, const IMU_Sample_t *sample, uint8_t stream_ok);
/* Board port: bounded single pulse, cannot remain on if main/CPU stops. */
void IMU_HeaterPortInit(void);
void IMU_HeaterPortWrite(float duty);
#endif
