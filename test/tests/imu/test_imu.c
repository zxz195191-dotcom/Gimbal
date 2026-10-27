#include "spi.h"
#include "imu_frame.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

SPI_HandleTypeDef hspi5;
FakeDWT fake_dwt;
uint32_t fake_primask;
static uint32_t tick;
static uint8_t regs[128], frame[15], *dma_rx;
static uint8_t fail_spi, corrupt_register;
static GPIO_PinState chip_select;

uint32_t HAL_GetTick(void) { return tick; }
uint32_t HAL_RCC_GetHCLKFreq(void) { return 180000000U; }
void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState value) {
    (void)port; (void)pin; chip_select = value;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi, uint8_t *out,
                                         uint8_t *in, uint16_t length, uint32_t timeout) {
    (void)spi;
    assert(fake_primask == 0U);
    assert(chip_select == GPIO_PIN_RESET && length == 2U && timeout <= 2U);
    if (fail_spi) return HAL_TIMEOUT;
    if (out[0] & 0x80U) {
        in[1] = regs[out[0] & 0x7FU];
        if ((out[0] & 0x7FU) == corrupt_register) in[1] ^= 1U;
    } else {
        regs[out[0]] = out[1];
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *spi, uint8_t *out,
                                             uint8_t *in, uint16_t length) {
    assert(chip_select == GPIO_PIN_RESET && out[0] == 0xBAU && length == 16U);
    if (fail_spi) return HAL_ERROR;
    dma_rx = in;
    spi->State = HAL_SPI_STATE_BUSY_TX_RX;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *spi) {
    assert(fake_primask == 0U);
    spi->State = HAL_SPI_STATE_READY;
    dma_rx = NULL;
    return HAL_OK;
}

static void advance(uint32_t ms) {
    tick += ms;
    fake_dwt.CYCCNT += ms * 180000U;
}
static void setup(void) {
    memset(regs, 0, sizeof(regs));
    memset(frame, 0, sizeof(frame));
    regs[0x75] = 0x70;
    frame[0] = 1U;
    frame[5] = 0x10; /* accel Z = +1g at +/-8g */
    hspi5 = (SPI_HandleTypeDef){SPI5, (void *)1, (void *)2, HAL_SPI_STATE_READY, 0};
    fake_dwt = (FakeDWT){1U, 12345U};
    fake_primask = tick = fail_spi = corrupt_register = 0U;
    dma_rx = NULL;
    IMU_Init();
    assert(fake_dwt.CYCCNT == 12345U); /* no shared clock reset */
}
static void start(void) {
    for (unsigned i = 0; i < 200U && g_imu_debug.state != IMU_WAIT_DATA &&
                         g_imu_debug.state != IMU_FAULT; ++i) {
        IMU_Service();
        advance(1);
    }
}
static void finish_dma(void) {
    assert(dma_rx != NULL);
    memcpy(&dma_rx[1], frame, sizeof(frame));
    hspi5.State = HAL_SPI_STATE_READY;
    IMU_OnTransferComplete();
    dma_rx = NULL;
}
static void sample(void) {
    advance(1);
    IMU_OnDataReady();
    finish_dma();
    IMU_Service();
}

static void test_decode_and_wrap(void) {
    IMU_Sample_t s = {0};
    uint8_t bytes[15] = {1, 0x10, 0, 0xF0, 0, 0, 0, 0, 0,
                        0x06, 0x68, 0xF9, 0x98, 0, 0};
    assert(IMU_DecodeFrame(bytes, &s));
    assert(fabsf(s.accel_mps2[0] - 9.80665f) < 1e-5f);
    assert(fabsf(s.accel_mps2[1] + 9.80665f) < 1e-5f);
    assert(fabsf(s.gyro_rad_s[0] - 1.74532925f) < 1e-5f);
    assert(fabsf(s.gyro_rad_s[1] + 1.74532925f) < 1e-5f);
    assert(s.temperature_c == 21.0f);
    bytes[9] = 0x80; bytes[10] = 0;
    assert(!IMU_DecodeFrame(bytes, &s) && s.gyro_raw[0] == -32768);
    assert(s.flags & IMU_FRAME_SATURATED);
    memset(bytes, 0xFF, sizeof(bytes));
    assert(!IMU_DecodeFrame(bytes, &s) && (s.flags & IMU_FRAME_BAD_BUS));
    memset(bytes, 0, sizeof(bytes));
    assert(!IMU_DecodeFrame(bytes, &s) && (s.flags & IMU_FRAME_NO_DRDY));
    assert(fabsf(IMU_CycleDeltaSeconds(UINT32_MAX - 179999U, 0U, 180000000U) - 0.001f) < 1e-8f);
    assert(IMU_CycleDeltaSeconds(0, 10, 0) == 0.0f);
}

static void test_stream(void) {
    setup(); start();
    assert(g_imu_debug.state == IMU_WAIT_DATA && g_imu_debug.who_am_i == 0x70);
    for (unsigned i = 0; i < 1100U; ++i) sample();
    IMU_Sample_t s;
    assert(IMU_GetSample(&s));
    assert(g_imu_debug.valid_count == 1100U && g_imu_debug.invalid_frame_count == 0U);
    assert(fabsf(g_imu_debug.sample_rate_hz - 1000.0f) < 3.0f);
    assert(fabsf(s.dt_s - 0.001f) < 1e-8f);
    /* DWT rollover inside normal operation. */
    fake_dwt.CYCCNT = UINT32_MAX - 180000U;
    sample(); sample();
    assert(IMU_GetSample(&s) && fabsf(s.dt_s - 0.001f) < 1e-8f);
    advance(6);
    assert(!IMU_GetSample(&s)); /* age check works even without Service */
    advance(5); IMU_Service();
    assert(g_imu_debug.state == IMU_FAULT && g_imu_debug.error == IMU_ERROR_NO_DATA);
    IMU_RequestRestart(); IMU_Service(); start(); sample();
    assert(IMU_GetSample(&s));
}

static void test_faults(void) {
    setup(); fail_spi = 1; IMU_Service();
    assert(g_imu_debug.error == IMU_ERROR_SPI && chip_select == GPIO_PIN_SET);
    setup(); regs[0x75] = 0; start();
    assert(g_imu_debug.error == IMU_ERROR_ID);
    setup(); corrupt_register = 0x1B; start();
    assert(g_imu_debug.error == IMU_ERROR_READBACK && g_imu_debug.failed_register == 0x1B);
    setup(); start(); advance(11); IMU_Service();
    assert(g_imu_debug.error == IMU_ERROR_NO_DATA);
    setup(); start(); IMU_OnDataReady(); IMU_OnDataReady();
    assert(g_imu_debug.busy_drop_count == 1U);
    advance(3); IMU_Service();
    assert(g_imu_debug.error == IMU_ERROR_DMA_TIMEOUT && chip_select == GPIO_PIN_SET);
    setup(); start(); fail_spi = 1; IMU_OnDataReady(); IMU_Service();
    assert(g_imu_debug.error == IMU_ERROR_SPI);
    setup(); start(); frame[0] = 0;
    for (unsigned i = 0; i < 10U; ++i) sample();
    assert(g_imu_debug.error == IMU_ERROR_BAD_FRAMES && !g_imu_debug.sample.valid);
}

static void test_backlog_and_ticks(void) {
    setup(); start();
    for (unsigned i = 0; i < 9U; ++i) {
        advance(1); IMU_OnDataReady(); finish_dma();
    }
    assert(g_imu_debug.queue_overrun_count == 2U);
    IMU_Service();
    assert(g_imu_debug.stale_drop_count == 3U);
    assert(g_imu_debug.sample_age_ms <= 5U);
    setup(); start(); tick = UINT32_MAX - 150U;
    /* Restart at tick rollover to exercise all startup deadline comparisons. */
    IMU_Init(); start();
    for (unsigned i = 0; i < 40U; ++i) sample();
    assert(g_imu_debug.state == IMU_RUNNING && g_imu_debug.sample.valid);
}

static unsigned consumed, restarts;
static void consume(const IMU_Sample_t *s) {
    if (s == NULL) ++restarts;
    else ++consumed;
}
static void test_consumer(void) {
    IMU_SetSampleCallback(consume);
    setup(); start();
    assert(restarts == 1U);
    for (unsigned i=0;i<4;++i) { advance(1); IMU_OnDataReady(); finish_dma(); }
    IMU_Service();
    assert(consumed == 4U); /* every queued frame, not just latest */
    frame[0] = 0U;
    sample();
    assert(consumed == 5U); /* invalid frame reaches consumer for invalidation */
    IMU_RequestRestart(); IMU_Service();
    assert(restarts == 2U);
    IMU_SetSampleCallback(NULL);
}

int main(void) {
    test_decode_and_wrap();
    test_stream();
    test_faults();
    test_backlog_and_ticks();
    test_consumer();
    puts("IMU tests passed: decode, units, rollover, startup, stale data, SPI/DMA faults, restart, backlog.");
    return 0;
}
