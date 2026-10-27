#include "imu_attitude.h"
#include "imu_heater.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static IMU_Sample_t sample;
static uint32_t now;
static float heater_output;
void IMU_HeaterPortInit(void) { heater_output = 0.0f; }
void IMU_HeaterPortWrite(float duty) { heater_output = duty; }
static void reset(void) {
    memset(&sample, 0, sizeof(sample));
    sample.valid = 1U;
    sample.dt_s = 0.001f;
    sample.accel_mps2[2] = 9.80665f;
    sample.temperature_c = 33.0f;
    now = 0U;
    IMU_AttitudeInit();
    IMU_HeaterInit();
}
static void tick(uint8_t thermal) {
    sample.timestamp_ms = ++now;
    ++sample.sequence;
    IMU_AttitudeProcess(&sample);
    IMU_AttitudeService(now, sample.valid, thermal);
}
static void run(unsigned n) { while (n--) tick(0U); }
static void cal(void) {
    IMU_AttitudeRequestCalibration();
    IMU_AttitudeService(now, 1U, 0U);
    run(6100);
    assert(g_imu_attitude.state == IMU_ATT_READY);
    assert(g_imu_attitude.valid);
}
static void close_to(float a, float b, float tolerance) { assert(fabsf(a-b) < tolerance); }

static void test_bias_and_rotation(void) {
    reset();
    sample.gyro_rad_s[0] = 0.0011f;
    sample.gyro_rad_s[1] = -0.0234f;
    sample.gyro_rad_s[2] = 0.002f;
    cal();
    close_to(g_imu_attitude.bias_rad_s[1], -0.0234f, 0.00001f);
    close_to(g_imu_attitude.gyro_rad_s[1], 0.0f, 0.00001f);
    sample.gyro_rad_s[2] += 1.5707963268f;
    run(1000);
    close_to(g_imu_attitude.yaw_left_deg, 90.0f, 0.1f);
    float before = g_imu_attitude.yaw_left_deg;
    IMU_AttitudeProcess(&sample); /* duplicate must not integrate */
    close_to(g_imu_attitude.yaw_left_deg, before, 0.0001f);
    sample.gyro_rad_s[2] -= 3.1415926536f;
    run(1000);
    close_to(g_imu_attitude.yaw_left_deg, 0.0f, 0.1f);
    puts("PASS bias, units, yaw direction, duplicate rejection");
}
static void test_pitch_roll(void) {
    reset();
    cal();
    for (unsigned i = 1; i <= 1000; ++i) {
        const float a = 0.5235987756f * (float)i / 1000.0f;
        sample.gyro_rad_s[1] = -0.5235987756f;
        sample.accel_mps2[0] = 9.80665f * sinf(a);
        sample.accel_mps2[2] = 9.80665f * cosf(a);
        tick(0U);
    }
    close_to(g_imu_attitude.pitch_up_deg, 30.0f, 0.1f);
    reset();
    sample.accel_mps2[0] = 4.903325f;
    sample.accel_mps2[2] = 8.492808f;
    cal();
    close_to(g_imu_attitude.pitch_up_deg, 30.0f, 0.1f);
    reset();
    sample.accel_mps2[1] = -4.903325f;
    sample.accel_mps2[2] = 8.492808f;
    cal();
    close_to(g_imu_attitude.roll_deg, -30.0f, 0.1f);
    puts("PASS pitch-up sign, tilted startup, negative roll");
}
static void test_cal_rejection(void) {
    reset();
    IMU_AttitudeRequestCalibration();
    IMU_AttitudeService(now, 1U, 0U);
    sample.gyro_rad_s[0] = 0.1f;
    run(30001);
    assert(g_imu_attitude.state == IMU_ATT_FAULT);
    assert(g_imu_attitude.error == IMU_ATT_CAL_TIMEOUT);
    assert(!g_imu_attitude.calibrated);
    sample.gyro_rad_s[0] = 0.0f;
    cal();
    reset();
    IMU_AttitudeRequestCalibration();
    IMU_AttitudeService(now, 1U, 0U);
    for (unsigned i=0; i<6000; ++i) {
        sample.temperature_c = 33.0f + (float)i * 0.0003f;
        tick(0U);
    }
    assert(!g_imu_attitude.calibrated);
    reset();
    IMU_AttitudeRequestCalibration();
    IMU_AttitudeService(now, 1U, 0U);
    for (unsigned i=0; i<6000; ++i) {
        sample.gyro_rad_s[0] = (i & 1U) ? 0.005f : -0.005f;
        tick(0U);
    }
    assert(!g_imu_attitude.calibrated);
    puts("PASS movement, temperature instability, gyro variance rejection and timeout");
}
static void test_faults(void) {
    reset(); cal();
    sample.sequence += 2U;
    tick(0U);
    assert(!g_imu_attitude.valid && g_imu_attitude.error == IMU_ATT_STREAM_GAP);
    run(100);
    assert(!g_imu_attitude.valid); /* no silent recovery/re-zero */
    cal();
    sample.temperature_c += 2.1f;
    tick(0U);
    assert(g_imu_attitude.error == IMU_ATT_TEMPERATURE_CHANGED);
    reset(); cal();
    sample.accel_mps2[0] = NAN;
    tick(0U);
    assert(g_imu_attitude.error == IMU_ATT_NUMERIC);
    reset(); cal();
    IMU_AttitudeService(now + 6U, 1U, 0U);
    assert(!g_imu_attitude.valid);
    reset(); cal();
    sample.dt_s = 0.002f;
    tick(0U);
    assert(!g_imu_attitude.valid);
    IMU_AttitudeProcess(NULL);
    assert(!g_imu_attitude.calibrated && g_imu_attitude.state == IMU_ATT_WAIT);
    puts("PASS gaps, stale data, bad dt, temperature shift, NaN, restart");
}
static void test_heater(void) {
    reset();
    for (now = 0; now < 1000; ++now) {
        sample.timestamp_ms = now;
        IMU_HeaterService(now, &sample, 1U);
        assert(heater_output >= 0.0f && heater_output <= 0.8f);
    }
    assert(heater_output > 0.0f);
    sample.temperature_c = 55.0f;
    sample.timestamp_ms = now;
    IMU_HeaterService(now, &sample, 1U);
    assert(heater_output == 0.0f && g_imu_heater.error == IMU_HEAT_OVER_TEMP);
    sample.temperature_c = 45.0f;
    IMU_HeaterService(now, &sample, 1U);
    assert(g_imu_heater.state == IMU_HEAT_FAULT);
    IMU_HeaterSetEnabled(1U);
    for (now = 0; now <= 5100; ++now) {
        sample.timestamp_ms = now;
        IMU_HeaterService(now, &sample, 1U);
    }
    assert(g_imu_heater.stable);
    IMU_HeaterService(now, &sample, 0U);
    assert(heater_output == 0.0f && g_imu_heater.error == IMU_HEAT_SENSOR);
    reset();
    for (now = 0; now <= 60100; ++now) {
        sample.timestamp_ms = now;
        IMU_HeaterService(now, &sample, 1U);
    }
    assert(g_imu_heater.error == IMU_HEAT_NO_RISE && heater_output == 0.0f);
    IMU_HeaterSetEnabled(0U);
    assert(g_imu_heater.state == IMU_HEAT_OFF && heater_output == 0.0f);
    puts("PASS heater limits, overtemperature, data loss, no power/no rise, disable");
}
static void test_automatic_and_wrap(void) {
    reset(); run(5000);
    assert(g_imu_attitude.state == IMU_ATT_WAIT);
    for (unsigned i=0;i<6200;++i) tick(1U);
    assert(g_imu_attitude.valid);
    reset();
    now = UINT32_MAX - 1000U;
    cal();
    assert(g_imu_attitude.valid);
    sample.accel_mps2[0] = 10.0f;
    tick(0U);
    assert(g_imu_attitude.accel_rejected && g_imu_attitude.valid);
    puts("PASS thermal auto-cal, time wrap, acceleration magnitude rejection");
}
static void test_public_interface(void) {
    IMU_AttitudeSample_t out, before;
    reset();
    assert(!IMU_AttitudeGet(now, NULL));
    assert(!IMU_AttitudeGet(now, &out) && out.age_ms == UINT32_MAX);
    cal();
    sample.timestamp_cycles = 123456U;
    tick(0U);
    assert(IMU_AttitudeGet(now, &before));
    assert(before.heading_continuous_valid && before.sequence == sample.sequence);
    assert(before.timestamp_ms == now && before.timestamp_cycles == 123456U);
    close_to(before.dt_s, 0.001f, 1e-6f);
    close_to(before.accel_body_mps2[2], 9.80665f, 1e-6f);
    close_to(before.quaternion_wxyz[0], 1.0f, 1e-6f);
    assert(IMU_AttitudeGet(now + 5U, &out));
    assert(!IMU_AttitudeGet(now + 6U, &out) && out.age_ms == 6U);
    /* Fresh RAW frames after a fault cannot retimestamp the frozen attitude. */
    sample.sequence += 2U;
    tick(0U);
    run(100);
    assert(!IMU_AttitudeGet(now, &out));
    assert(out.state == IMU_ATT_FAULT && out.error == IMU_ATT_STREAM_GAP);
    assert(out.timestamp_ms == before.timestamp_ms && out.sequence == before.sequence);
    IMU_AttitudeRequestCalibration();
    IMU_AttitudeService(now, 1U, 0U);
    assert(!IMU_AttitudeGet(now, &out));
    assert(out.reference_id != before.reference_id && out.age_ms == UINT32_MAX);
    run(6100);
    assert(IMU_AttitudeGet(now, &before));
    IMU_AttitudeProcess(NULL);
    assert(!IMU_AttitudeGet(now, &out) && out.reference_id != before.reference_id);
    reset();
    now = UINT32_MAX - 6200U;
    cal();
    /* Query across the HAL tick wrap while sample remains 5 ms old. */
    run(100);
    assert(IMU_AttitudeGet(now + 5U, &out));
    assert(!IMU_AttitudeGet(now + 6U, &out));
    puts("PASS snapshot units, freshness, frozen timestamp, reset reference and rollover");
}
static void test_continuous_yaw(void) {
    IMU_AttitudeSample_t out;
    reset(); cal();
    sample.gyro_rad_s[2] = 1.5707963268f;
    double previous = 0.0;
    for (unsigned i = 0; i < 8000; ++i) {
        tick(0U);
        assert(IMU_AttitudeGet(now, &out) && out.heading_continuous_valid);
        assert(fabs(out.yaw_continuous_rad - previous) < 0.002);
        previous = out.yaw_continuous_rad;
    }
    assert(fabs(out.yaw_continuous_rad - 12.5663706144) < 0.005);
    sample.gyro_rad_s[2] = -1.5707963268f;
    for (unsigned i = 0; i < 12000; ++i) {
        tick(0U);
        assert(IMU_AttitudeGet(now, &out) && out.heading_continuous_valid);
        assert(fabs(out.yaw_continuous_rad - previous) < 0.002);
        previous = out.yaw_continuous_rad;
    }
    assert(fabs(out.yaw_continuous_rad + 6.2831853072) < 0.005);
    float yaw = out.yaw_left_rad;
    IMU_AttitudeProcess(&sample);
    assert(IMU_AttitudeGet(now, &out));
    assert(out.yaw_continuous_rad == previous && out.yaw_left_rad == yaw);
    sample.gyro_rad_s[2] = 0.0f;
    cal();
    assert(IMU_AttitudeGet(now, &out));
    assert(fabs(out.yaw_continuous_rad) < 0.001);
    /* An Euler heading singularity must not silently publish a turn count. */
    reset();
    sample.accel_mps2[0] = 9.80665f;
    sample.accel_mps2[2] = 0.0f;
    cal();
    assert(IMU_AttitudeGet(now, &out));
    assert(!out.heading_continuous_valid);
    puts("PASS continuous +720/-1080 deg, duplicates, recalibration and vertical heading guard");
}
int main(void) {
    test_bias_and_rotation(); test_pitch_roll(); test_cal_rejection();
    test_faults(); test_heater(); test_automatic_and_wrap();
    test_public_interface(); test_continuous_yaw();
    puts("ALL attitude/heater tests passed");
    return 0;
}
