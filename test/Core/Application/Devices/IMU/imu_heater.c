#include "imu_heater.h"
#include <math.h>
#include <string.h>

volatile IMU_HeaterDebug_t g_imu_heater;
static uint32_t last_update, started, stable_start, rise_start;
static float integral, rise_temperature;
static uint8_t has_sample, in_band;

static void heat_fault(IMU_HeatError_t error) {
    IMU_HeaterPortWrite(0.0f);
    g_imu_heater.duty = 0.0f;
    g_imu_heater.stable = 0U;
    g_imu_heater.state = IMU_HEAT_FAULT;
    g_imu_heater.error = error;
}

void IMU_HeaterSetEnabled(uint8_t enabled) {
    IMU_HeaterPortWrite(0.0f);
    g_imu_heater.enabled = enabled != 0U;
    g_imu_heater.stable = 0U;
    g_imu_heater.duty = 0.0f;
    g_imu_heater.error = IMU_HEAT_OK;
    g_imu_heater.state = enabled ? IMU_HEAT_WARMING : IMU_HEAT_OFF;
    has_sample = in_band = 0U;
    integral = 0.0f;
}

void IMU_HeaterInit(void) {
    memset((void *)&g_imu_heater, 0, sizeof(g_imu_heater));
    IMU_HeaterPortInit();
    g_imu_heater.target_c = 45.0f;
    IMU_HeaterSetEnabled(1U);
}

void IMU_HeaterService(uint32_t now, const IMU_Sample_t *s, uint8_t stream_ok) {
    if (!g_imu_heater.enabled || g_imu_heater.state == IMU_HEAT_FAULT) return;
    if (!stream_ok || s == NULL || !s->valid || (uint32_t)(now - s->timestamp_ms) > 5U) {
        /* Wait safely for the very first frame; loss after heating latches. */
        IMU_HeaterPortWrite(0.0f);
        if (has_sample) heat_fault(IMU_HEAT_SENSOR);
        return;
    }
    float t = s->temperature_c;
    g_imu_heater.temperature_c = t;
    g_imu_heater.timestamp_ms = s->timestamp_ms;
    if (!isfinite(t) || t < -20.0f || t > 85.0f) { heat_fault(IMU_HEAT_SENSOR); return; }
    if (t >= 55.0f) { heat_fault(IMU_HEAT_OVER_TEMP); return; }
    if (!has_sample) {
        has_sample = 1U;
        started = last_update = now;
        rise_start = now;
        rise_temperature = t;
        return;
    }
    if ((uint32_t)(now - last_update) < 20U) return;
    const float dt = (float)(now - last_update) * 0.001f;
    last_update = now;
    if (dt > 0.1f) { heat_fault(IMU_HEAT_SENSOR); return; }
    const float error = g_imu_heater.target_c - t;
    if (error > 2.0f) {
        if (t >= rise_temperature + 1.0f) {
            rise_start = now;
            rise_temperature = t;
        } else if ((uint32_t)(now - rise_start) >= 60000U) {
            heat_fault(IMU_HEAT_NO_RISE);
            return;
        }
    } else {
        rise_start = now;
        rise_temperature = t;
    }
    /* Duty units 0..1. Conservative initial PI, hardware acceptance required.
     * Integrate only when not pushing farther into saturation. */
    float p = 0.15f * error;
    float candidate = integral + 0.025f * error * dt;
    if (candidate < 0.0f) candidate = 0.0f;
    if (candidate > 0.8f) candidate = 0.8f;
    if ((p + integral < 0.8f || error < 0.0f) &&
        (p + integral > 0.0f || error > 0.0f)) integral = candidate;
    float duty = p + integral;
    if (duty > 0.8f) duty = 0.8f;
    if (duty < 0.0f || t >= 47.0f) duty = 0.0f;
    g_imu_heater.duty = duty;
    if (fabsf(error) <= 0.5f) {
        if (!in_band) { stable_start = now; in_band = 1U; }
        g_imu_heater.stable = (uint32_t)(now - stable_start) >= 5000U;
    } else {
        in_band = 0U;
        g_imu_heater.stable = 0U;
    }
    g_imu_heater.state = g_imu_heater.stable ? IMU_HEAT_STABLE : IMU_HEAT_WARMING;
    if (g_imu_heater.stable) started = now;
    else if ((uint32_t)(now - started) >= 180000U) {
        heat_fault(IMU_HEAT_TIMEOUT);
        return;
    }
    IMU_HeaterPortWrite(duty);
}
