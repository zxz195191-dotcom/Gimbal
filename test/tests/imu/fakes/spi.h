#ifndef TEST_SPI_H
#define TEST_SPI_H
#include <stdint.h>
#include <stddef.h>

typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
enum { HAL_SPI_STATE_READY, HAL_SPI_STATE_BUSY_TX_RX };
typedef struct { volatile uint32_t CTRL, CYCCNT; } FakeDWT;
typedef struct {
    void *Instance;
    void *hdmarx;
    void *hdmatx;
    uint32_t State;
    uint32_t ErrorCode;
} SPI_HandleTypeDef;
extern SPI_HandleTypeDef hspi5;
extern FakeDWT fake_dwt;
extern uint32_t fake_primask;
#define SPI5 ((void *)5)
#define DWT (&fake_dwt)
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define MPU6500_CS_GPIO_Port ((void *)6)
#define MPU6500_CS_Pin 64U
#define MPU6500_INT_Pin 256U
#define __get_PRIMASK() (fake_primask)
#define __disable_irq() (fake_primask = 1U)
#define __set_PRIMASK(value) (fake_primask = (value))
#define __DMB() ((void)0)
#define __HAL_GPIO_EXTI_CLEAR_IT(pin) ((void)(pin))
uint32_t HAL_GetTick(void);
uint32_t HAL_RCC_GetHCLKFreq(void);
void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState value);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi, uint8_t *out,
                                         uint8_t *in, uint16_t length, uint32_t timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *spi, uint8_t *out,
                                             uint8_t *in, uint16_t length);
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *spi);
#endif
