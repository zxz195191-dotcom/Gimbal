#include "imu_mpu6500.h"
#include "imu_frame.h"
#include "spi.h"
#include <string.h>

#define IMU_QUEUE_SIZE 8U
#define IMU_MAX_AGE_MS 5U
#define IMU_STREAM_TIMEOUT_MS 10U
#define IMU_SPI_TIMEOUT_MS 2U

typedef struct {
    uint8_t bytes[15];
    uint32_t cycles;
    uint32_t tick;
    uint32_t sequence;
} IMU_Frame_t;

/* Writes use the same <=1 MHz bus as sampling; no runtime clock switching.
 * 90 MHz / 128 = 703.125 kHz. 16 bytes take about 182 us at 1 kHz.
 * MPU6500 register map: gyro and accel DLPF both 92 Hz, ODR 1 kHz. */
static const uint8_t registers[][2] = {
    {0x6C, 0x00}, /* axes enabled */
    {0x6A, 0x10}, /* I2C_IF_DIS; auxiliary I2C/FIFO/DMP disabled */
    {0x19, 0x00}, /* sample rate divider */
    {0x1A, 0x02}, /* gyro DLPF */
    {0x1B, 0x18}, /* gyro +/-2000 deg/s */
    {0x1C, 0x10}, /* accel +/-8g */
    {0x1D, 0x02}, /* accel DLPF */
    {0x37, 0x00}, /* active-high pulse, clear only by INT_STATUS read */
    {0x38, 0x01}  /* RAW_RDY_EN, enabled last */
};

volatile IMU_Debug_t g_imu_debug;
volatile uint8_t g_imu_log_mode;
static IMU_Frame_t queue[IMU_QUEUE_SIZE];
static volatile uint8_t head, tail, sampling, transfer_active;
static volatile uint8_t spi_failed;
static volatile uint8_t restart_requested;
static volatile uint32_t transfer_tick, last_complete_tick;
static uint8_t tx[16] = {0xBA};
static uint8_t rx[16]; /* Normal SRAM .bss; DMA cannot access CCMRAM. */
static uint32_t transfer_cycles, transfer_sequence;
static uint32_t phase_tick, previous_cycles, core_clock;
static uint32_t rate_tick, rate_count;
static uint8_t register_index, previous_valid, bad_consecutive;
static IMU_Sample_t latest;
static IMU_SampleCallback_t sample_callback;

void IMU_SetSampleCallback(IMU_SampleCallback_t callback) {
    sample_callback = callback;
}

static void restore_irq(uint32_t primask) {
    __set_PRIMASK(primask);
}

static HAL_StatusTypeDef register_transfer(uint8_t address, uint8_t value,
                                           uint8_t *readback) {
    uint8_t out[2] = {address, value};
    uint8_t in[2];
    HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(
        &hspi5, out, in, 2U, IMU_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_SET);
    if ((result == HAL_OK) && (readback != NULL)) *readback = in[1];
    return result;
}

static void enter_fault(IMU_Error_t error) {
    sampling = 0U;
    /* Abort only in main; bounded by HAL, never inside the critical section. */
    if (transfer_active || (hspi5.State != HAL_SPI_STATE_READY)) {
        (void)HAL_SPI_Abort(&hspi5);
    }
    HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_SET);
    transfer_active = 0U;
    latest.valid = 0U;
    g_imu_debug.sample.valid = 0U;
    g_imu_debug.error = error;
    g_imu_debug.state = IMU_FAULT;
}

static uint8_t write_register(uint8_t reg, uint8_t value) {
    g_imu_debug.failed_register = reg;
    g_imu_debug.expected_value = value;
    if (register_transfer(reg, value, NULL) != HAL_OK) {
        ++g_imu_debug.spi_error_count;
        g_imu_debug.hal_spi_error = hspi5.ErrorCode;
        enter_fault(IMU_ERROR_SPI);
        return 0U;
    }
    return 1U;
}

static uint8_t read_register(uint8_t reg, uint8_t *value) {
    g_imu_debug.failed_register = reg;
    if (register_transfer(reg | 0x80U, 0U, value) != HAL_OK) {
        ++g_imu_debug.spi_error_count;
        g_imu_debug.hal_spi_error = hspi5.ErrorCode;
        enter_fault(IMU_ERROR_SPI);
        return 0U;
    }
    g_imu_debug.readback_value = *value;
    return 1U;
}

void IMU_Init(void) {
    if (sample_callback != NULL) sample_callback(NULL);
    sampling = 0U;
    if ((hspi5.Instance == SPI5) &&
        (transfer_active || (hspi5.State != HAL_SPI_STATE_READY))) {
        (void)HAL_SPI_Abort(&hspi5);
    }
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memset((void *)&g_imu_debug, 0, sizeof(g_imu_debug));
    memset(&latest, 0, sizeof(latest));
    head = tail = transfer_active = spi_failed = 0U;
    restart_requested = previous_valid = bad_consecutive = register_index = 0U;
    restore_irq(primask);
    HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_SET);
    core_clock = HAL_RCC_GetHCLKFreq();
    g_imu_debug.dt_min_cycles = UINT32_MAX;
    g_imu_debug.sample_age_ms = UINT32_MAX;
    if ((core_clock == 0U) || ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)) {
        g_imu_debug.state = IMU_FAULT;
        g_imu_debug.error = IMU_ERROR_TIMEBASE;
    } else if ((hspi5.Instance != SPI5) || (hspi5.hdmarx == NULL) ||
               (hspi5.hdmatx == NULL)) {
        g_imu_debug.state = IMU_FAULT;
        g_imu_debug.error = IMU_ERROR_SPI;
    } else {
        g_imu_debug.state = IMU_RESET;
    }
}

void IMU_RequestRestart(void) {
    restart_requested = 1U;
}

void IMU_OnDataReady(void) {
    if (!sampling) return;
    ++g_imu_debug.drdy_count;
    if (transfer_active || (hspi5.State != HAL_SPI_STATE_READY)) {
        ++g_imu_debug.busy_drop_count;
        return;
    }
    transfer_active = 1U;
    transfer_cycles = DWT->CYCCNT;
    transfer_tick = HAL_GetTick();
    transfer_sequence = g_imu_debug.drdy_count;
    HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive_DMA(&hspi5, tx, rx, sizeof(tx)) != HAL_OK) {
        IMU_OnSpiError(hspi5.ErrorCode);
    }
}

void IMU_OnTransferComplete(void) {
    HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_SET);
    if (sampling && transfer_active) {
        const uint8_t next = (head + 1U) % IMU_QUEUE_SIZE;
        ++g_imu_debug.dma_count;
        last_complete_tick = HAL_GetTick();
        if (next == tail) {
            ++g_imu_debug.queue_overrun_count;
        } else {
            memcpy(queue[head].bytes, &rx[1], 15U);
            queue[head].cycles = transfer_cycles;
            queue[head].tick = transfer_tick;
            queue[head].sequence = transfer_sequence;
            __DMB();
            head = next;
        }
    }
    transfer_active = 0U;
}

void IMU_OnSpiError(uint32_t error) {
    HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_SET);
    ++g_imu_debug.spi_error_count;
    g_imu_debug.hal_spi_error = error;
    sampling = 0U;
    spi_failed = 1U;
    /* Leave active set so Service performs the abort before restart. */
}

static uint8_t pop_frame(IMU_Frame_t *frame) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (tail == head) {
        restore_irq(primask);
        return 0U;
    }
    __DMB();
    *frame = queue[tail];
    tail = (tail + 1U) % IMU_QUEUE_SIZE;
    restore_irq(primask);
    return 1U;
}

static void process_frames(void) {
    IMU_Frame_t frame;
    /* Bound CPU work per main-loop pass. Never publish delayed samples as fresh. */
    for (unsigned i = 0; i < 4U && pop_frame(&frame); ++i) {
        if ((uint32_t)(HAL_GetTick() - frame.tick) > IMU_MAX_AGE_MS) {
            ++g_imu_debug.stale_drop_count;
            continue;
        }
        IMU_Sample_t sample = {0};
        (void)IMU_DecodeFrame(frame.bytes, &sample);
        sample.timestamp_cycles = frame.cycles;
        sample.timestamp_ms = frame.tick;
        sample.sequence = frame.sequence;
        if (previous_valid) {
            uint32_t delta = frame.cycles - previous_cycles;
            sample.dt_s = IMU_CycleDeltaSeconds(previous_cycles, frame.cycles, core_clock);
            if (delta < g_imu_debug.dt_min_cycles) g_imu_debug.dt_min_cycles = delta;
            if (delta > g_imu_debug.dt_max_cycles) g_imu_debug.dt_max_cycles = delta;
            if ((sample.dt_s < 0.0005f) || (sample.dt_s > 0.0015f)) {
                ++g_imu_debug.timing_gap_count;
                sample.flags |= IMU_FRAME_BAD_DT;
            }
        }
        previous_cycles = frame.cycles;
        previous_valid = 1U;
        sample.valid = (sample.flags == 0U);
        latest = sample;
        if (sample_callback != NULL) sample_callback(&sample);
        if (sample.valid) {
            ++g_imu_debug.valid_count;
            bad_consecutive = 0U;
            g_imu_debug.state = IMU_RUNNING;
        } else {
            ++g_imu_debug.invalid_frame_count;
            if (++bad_consecutive >= 10U) {
                enter_fault(IMU_ERROR_BAD_FRAMES);
                break;
            }
        }
    }
}

void IMU_Service(void) {
    if (restart_requested) IMU_Init();
    const uint32_t now = HAL_GetTick();
    uint8_t value;
    if (spi_failed && (g_imu_debug.state != IMU_FAULT)) enter_fault(IMU_ERROR_SPI);
    switch (g_imu_debug.state) {
    case IMU_RESET:
        if (write_register(0x6B, 0x80)) {
            phase_tick = now;
            g_imu_debug.state = IMU_WAIT_RESET;
        }
        break;
    case IMU_WAIT_RESET:
        if ((uint32_t)(now - phase_tick) < 100U) break;
        if (!read_register(0x75, &value)) break;
        g_imu_debug.who_am_i = value;
        if (value != 0x70U) { enter_fault(IMU_ERROR_ID); break; }
        if (write_register(0x6B, 0x01)) {
            phase_tick = now;
            g_imu_debug.state = IMU_WAIT_WAKE;
        }
        break;
    case IMU_WAIT_WAKE:
        if ((uint32_t)(now - phase_tick) < 30U) break;
        if (!read_register(0x6B, &value)) break;
        if (value != 0x01U) { enter_fault(IMU_ERROR_READBACK); break; }
        g_imu_debug.state = IMU_CONFIGURE;
        break;
    case IMU_CONFIGURE: {
        uint8_t reg = registers[register_index][0];
        uint8_t expected = registers[register_index][1];
        if (!write_register(reg, expected) || !read_register(reg, &value)) break;
        if (value != expected) { enter_fault(IMU_ERROR_READBACK); break; }
        if (++register_index == sizeof(registers) / sizeof(registers[0])) {
            /* Discard any startup DRDY; the next physical edge starts the stream. */
            if (!read_register(0x3A, &value)) break;
            __HAL_GPIO_EXTI_CLEAR_IT(MPU6500_INT_Pin);
            last_complete_tick = rate_tick = now;
            rate_count = 0U;
            g_imu_debug.failed_register = 0U;
            g_imu_debug.state = IMU_WAIT_DATA;
            sampling = 1U;
        }
        break;
    }
    case IMU_WAIT_DATA:
    case IMU_RUNNING: {
        /* Capture ISR-owned timestamps and the clock in a coherent snapshot. */
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        uint32_t check_now = HAL_GetTick();
        uint32_t transfer_age = check_now - transfer_tick;
        uint32_t stream_age = check_now - last_complete_tick;
        uint8_t active = transfer_active;
        restore_irq(primask);
        if (active && transfer_age > 2U) {
            enter_fault(IMU_ERROR_DMA_TIMEOUT);
            break;
        }
        if (stream_age > IMU_STREAM_TIMEOUT_MS) {
            enter_fault(IMU_ERROR_NO_DATA);
            break;
        }
        process_frames();
        break;
    }
    default:
        break;
    }
    if (latest.sequence != 0U) {
        g_imu_debug.sample_age_ms = HAL_GetTick() - latest.timestamp_ms;
        if ((g_imu_debug.state != IMU_RUNNING) ||
            (g_imu_debug.sample_age_ms > IMU_MAX_AGE_MS)) latest.valid = 0U;
    }
    g_imu_debug.sample = latest;
    g_imu_debug.queue_pending = (head + IMU_QUEUE_SIZE - tail) % IMU_QUEUE_SIZE;
    if ((uint32_t)(now - rate_tick) >= 1000U) {
        uint32_t count = g_imu_debug.dma_count;
        g_imu_debug.sample_rate_hz = (float)(count - rate_count) * 1000.0f / (float)(now - rate_tick);
        rate_count = count;
        rate_tick = now;
    }
}

uint8_t IMU_GetSample(IMU_Sample_t *sample) {
    if (sample == NULL) return 0U;
    *sample = latest;
    if ((g_imu_debug.state != IMU_RUNNING) || spi_failed ||
        ((uint32_t)(HAL_GetTick() - sample->timestamp_ms) > IMU_MAX_AGE_MS)) {
        sample->valid = 0U;
    }
    return sample->valid;
}
