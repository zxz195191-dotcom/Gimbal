#include "imu_attitude.h"
#include "FusionAhrs.h"
#include <math.h>
#include <string.h>

#define CAL_SAMPLES 3000U
#define CAL_TIMEOUT_MS 30000U
#define G 9.80665f
#define RAD_TO_DEG_IMU 57.2957795131f
volatile IMU_AttitudeDebug_t g_imu_attitude;
static FusionAhrs fusion;
static volatile uint8_t cal_requested;
static uint32_t last_sequence, started_ms, count;
static uint8_t seen, manual_cal;
static float mean[6], m2[6], temp_sum, temp_min, temp_max;
static IMU_AttitudeSample_t published;
static uint32_t reference_id;
static uint8_t published_once, yaw_seen, heading_trackable;
static float previous_yaw_deg;
static int32_t yaw_turns;

static void reset_publication(void) {
    memset(&published, 0, sizeof(published));
    published_once = yaw_seen = 0U;
    heading_trackable = 1U;
    previous_yaw_deg = 0.0f;
    yaw_turns = 0;
    ++reference_id;
    g_imu_attitude.yaw_continuous_deg = 0.0;
}

uint8_t IMU_AttitudeGet(uint32_t now_ms, IMU_AttitudeSample_t *out) {
    if (out == NULL) return 0U;
    *out = published;
    out->reference_id = reference_id;
    out->state = g_imu_attitude.state;
    out->error = g_imu_attitude.error;
    out->thermal_ready = g_imu_attitude.thermal_ready;
    out->age_ms = published_once ? now_ms - published.timestamp_ms : UINT32_MAX;
    out->valid = published_once && g_imu_attitude.valid &&
                 out->state == IMU_ATT_READY && out->age_ms <= 5U;
    out->heading_continuous_valid = out->valid && published.heading_continuous_valid;
    return out->valid;
}

static void reset_window(void) {
    count = 0U;
    memset(mean, 0, sizeof(mean));
    memset(m2, 0, sizeof(m2));
    temp_sum = 0.0f;
    temp_min = 1000.0f;
    temp_max = -1000.0f;
    g_imu_attitude.window_count = 0U;
    g_imu_attitude.progress = 0.0f;
}

void IMU_AttitudeInit(void) {
    memset((void *)&g_imu_attitude, 0, sizeof(g_imu_attitude));
    reset_publication();
    last_sequence = started_ms = 0U;
    seen = manual_cal = cal_requested = 0U;
    reset_window();
    FusionAhrsInitialise(&fusion);
    const FusionAhrsSettings settings = {
        .sampleRate = 1000.0f, .convention = FusionConventionNwu,
        .gain = 0.5f, .gyroscopeRange = 2000.0f,
        .accelerationRejection = 10.0f, .magneticRejection = 0.0f,
        .rejectionTimeout = 5.0f
    };
    FusionAhrsSetSettings(&fusion, &settings);
}

static void fault(IMU_AttitudeError_t reason) {
    g_imu_attitude.valid = 0U;
    g_imu_attitude.state = IMU_ATT_FAULT;
    g_imu_attitude.error = reason;
}

void IMU_AttitudeRequestCalibration(void) { cal_requested = 1U; }

static void start_cal(uint32_t now) {
    reset_publication();
    g_imu_attitude.state = IMU_ATT_CALIBRATING;
    g_imu_attitude.error = IMU_ATT_OK;
    g_imu_attitude.valid = g_imu_attitude.calibrated = 0U;
    started_ms = now;
    reset_window();
}

void IMU_AttitudeService(uint32_t now, uint8_t stream_ok, uint8_t thermal_ready) {
    g_imu_attitude.thermal_ready = thermal_ready;
    if (cal_requested) {
        cal_requested = 0U;
        manual_cal = 1U;
        start_cal(now);
    } else if (g_imu_attitude.state == IMU_ATT_WAIT && stream_ok && thermal_ready) {
        manual_cal = 0U;
        start_cal(now);
    }
    if (!stream_ok || (seen && (uint32_t)(now - g_imu_attitude.timestamp_ms) > 5U)) {
        if (g_imu_attitude.calibrated) fault(IMU_ATT_STREAM_GAP);
        else reset_window();
    }
    if (g_imu_attitude.state == IMU_ATT_CALIBRATING &&
        (uint32_t)(now - started_ms) >= CAL_TIMEOUT_MS) fault(IMU_ATT_CAL_TIMEOUT);
}

static void calibrate(const IMU_Sample_t *s) {
    const float norm = g_imu_attitude.accel_norm_mps2;
    uint8_t moving = fabsf(norm - G) > 0.5f;
    if (!manual_cal && !g_imu_attitude.thermal_ready) {
        reset_window();
        return;
    }
    for (unsigned i = 0; i < 3U; ++i) {
        if (fabsf(s->gyro_rad_s[i]) > 0.05236f) moving = 1U; /* 3 deg/s */
        if (count > 20U && (fabsf(s->gyro_rad_s[i] - mean[i]) > 0.01f ||
                           fabsf(s->accel_mps2[i] - mean[i+3]) > 0.25f)) moving = 1U;
    }
    if (moving) {
        ++g_imu_attitude.motion_resets;
        reset_window();
        return;
    }
    ++count;
    for (unsigned i = 0; i < 6U; ++i) {
        const float x = i < 3U ? s->gyro_rad_s[i] : s->accel_mps2[i-3U];
        const float delta = x - mean[i];
        mean[i] += delta / (float)count;
        m2[i] += delta * (x - mean[i]);
    }
    temp_sum += s->temperature_c;
    if (s->temperature_c < temp_min) temp_min = s->temperature_c;
    if (s->temperature_c > temp_max) temp_max = s->temperature_c;
    g_imu_attitude.window_count = count;
    g_imu_attitude.progress = (float)count / CAL_SAMPLES;
    if (count < CAL_SAMPLES) return;
    uint8_t noisy = (temp_max - temp_min) > 0.3f;
    for (unsigned i = 0; i < 6U; ++i) {
        float limit = i < 3U ? 0.003f : 0.08f; /* standard deviation */
        if (m2[i] / (float)(count - 1U) > limit * limit) noisy = 1U;
    }
    if (noisy) {
        ++g_imu_attitude.motion_resets;
        reset_window();
        return;
    }
    for (unsigned i = 0; i < 3U; ++i) g_imu_attitude.bias_rad_s[i] = mean[i];
    g_imu_attitude.calibration_temperature_c = temp_sum / (float)count;
    g_imu_attitude.calibrated = 1U;
    /* Gravity fixes initial roll/pitch, NOT absolute heading. Preserve a proper
     * right-handed frame internally; pitch-up sign changes only at the output. */
    const float roll = atan2f(mean[4], mean[5]);
    const float pitch = atan2f(-mean[3], hypotf(mean[4], mean[5]));
    const float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    const FusionQuaternion q = {.array = {cr*cp, sr*cp, cr*sp, -sr*sp}};
    FusionAhrsRestart(&fusion);
    FusionAhrsSetQuaternion(&fusion, q);
    g_imu_attitude.state = IMU_ATT_ALIGNING;
}

void IMU_AttitudeProcess(const IMU_Sample_t *s) {
    if (s == NULL) { IMU_AttitudeInit(); return; }
    if (seen && s->sequence == last_sequence) return;
    uint8_t gap = seen && (uint32_t)(s->sequence - last_sequence) != 1U;
    last_sequence = s->sequence;
    seen = 1U;
    g_imu_attitude.timestamp_ms = s->timestamp_ms;
    g_imu_attitude.temperature_c = s->temperature_c;
    if (!s->valid || gap || !(s->dt_s >= 0.0005f && s->dt_s <= 0.0015f)) {
        ++g_imu_attitude.gap_count;
        if (g_imu_attitude.calibrated) fault(IMU_ATT_STREAM_GAP);
        else reset_window();
        return;
    }
    for (unsigned i = 0; i < 3U; ++i) {
        if (!isfinite(s->gyro_rad_s[i]) || !isfinite(s->accel_mps2[i])) {
            fault(IMU_ATT_NUMERIC);
            return;
        }
    }
    if (!isfinite(s->temperature_c)) { fault(IMU_ATT_NUMERIC); return; }
    ++g_imu_attitude.sample_count;
    float norm2 = 0.0f;
    for (unsigned i = 0; i < 3U; ++i) norm2 += s->accel_mps2[i] * s->accel_mps2[i];
    g_imu_attitude.accel_norm_mps2 = sqrtf(norm2);
    if (g_imu_attitude.state == IMU_ATT_CALIBRATING) { calibrate(s); return; }
    if (g_imu_attitude.state != IMU_ATT_ALIGNING && g_imu_attitude.state != IMU_ATT_READY) return;
    if (fabsf(s->temperature_c - g_imu_attitude.calibration_temperature_c) > 2.0f) {
        fault(IMU_ATT_TEMPERATURE_CHANGED);
        return;
    }
    FusionVector gyro, accel;
    for (unsigned i = 0; i < 3U; ++i) {
        g_imu_attitude.gyro_rad_s[i] = s->gyro_rad_s[i] - g_imu_attitude.bias_rad_s[i];
        gyro.array[i] = g_imu_attitude.gyro_rad_s[i] * RAD_TO_DEG_IMU;
        accel.array[i] = s->accel_mps2[i] / G;
    }
    /* External magnitude gate supplements Fusion's angular rejection. */
    uint8_t reject = fabsf(g_imu_attitude.accel_norm_mps2 - G) > 1.5f;
    if (reject) accel = FUSION_VECTOR_ZERO;
    FusionAhrsSetSamplePeriod(&fusion, s->dt_s);
    FusionAhrsUpdateNoMagnetometer(&fusion, gyro, accel);
    FusionQuaternion q = FusionAhrsGetQuaternion(&fusion);
    for (unsigned i = 0; i < 4U; ++i) {
        if (!isfinite(q.array[i])) { fault(IMU_ATT_NUMERIC); return; }
        g_imu_attitude.quaternion[i] = q.array[i];
    }
    FusionEuler e = FusionQuaternionToEuler(q);
    g_imu_attitude.roll_deg = e.angle.roll;
    g_imu_attitude.pitch_up_deg = -e.angle.pitch;
    g_imu_attitude.yaw_left_deg = e.angle.yaw;
    g_imu_attitude.accel_rejected = reject || FusionAhrsGetInternalStates(&fusion).accelerometerIgnored;
    FusionAhrsFlags flags = FusionAhrsGetFlags(&fusion);
    g_imu_attitude.valid = !flags.startup && !flags.overrangeRecovery;
    g_imu_attitude.state = g_imu_attitude.valid ? IMU_ATT_READY : IMU_ATT_ALIGNING;
    /* Unwrap Euler heading without modifying Fusion or its origin. Passing a
     * pitch singularity makes turn count ambiguous: latch heading invalid until
     * recalibration, while quaternion/body rates remain usable via valid. */
    if (fabsf(e.angle.pitch) >= 85.0f || flags.overrangeRecovery) heading_trackable = 0U;
    if (heading_trackable) {
        if (yaw_seen) {
            const float delta = e.angle.yaw - previous_yaw_deg;
            if (delta > 180.0f) {
                if (yaw_turns == INT32_MIN) heading_trackable = 0U;
                else --yaw_turns;
            } else if (delta < -180.0f) {
                if (yaw_turns == INT32_MAX) heading_trackable = 0U;
                else ++yaw_turns;
            }
        }
        previous_yaw_deg = e.angle.yaw;
        yaw_seen = 1U;
        g_imu_attitude.yaw_continuous_deg = (double)yaw_turns * 360.0 + e.angle.yaw;
    }
    const float deg_to_rad = 0.017453292519943295f;
    for (unsigned i = 0; i < 4U; ++i) published.quaternion_wxyz[i] = q.array[i];
    for (unsigned i = 0; i < 3U; ++i) {
        published.gyro_body_rad_s[i] = g_imu_attitude.gyro_rad_s[i];
        published.accel_body_mps2[i] = s->accel_mps2[i];
    }
    published.roll_rad = e.angle.roll * deg_to_rad;
    published.pitch_up_rad = -e.angle.pitch * deg_to_rad;
    published.yaw_left_rad = e.angle.yaw * deg_to_rad;
    published.yaw_continuous_rad = g_imu_attitude.yaw_continuous_deg * 0.017453292519943295;
    published.temperature_c = s->temperature_c;
    published.dt_s = s->dt_s;
    published.timestamp_ms = s->timestamp_ms;
    published.timestamp_cycles = s->timestamp_cycles;
    published.sequence = s->sequence;
    published.accel_rejected = g_imu_attitude.accel_rejected;
    published.heading_continuous_valid = heading_trackable;
    published_once = 1U;
}
