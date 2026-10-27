#include "imu_heater.h"
#include "main.h"

/* A-board PB5 = TIM3_CH2, active high. Exclusive ownership of TIM3 here.
 * 1 MHz counter, 20 ms SINGLE pulse, PWM2: idle low at CNT=0, high only in
 * the tail of the period. OPM stops automatically at overflow/CNT=0, LOW.
 * Unlike continuous PWM, a CPU fault, breakpoint or blocked main cannot
 * leave the heater continuously energised. Max high time is 16 ms (80%).
 * Delayed service produces less average heat, never an extended pulse.
 * Do NOT enable DBGMCU TIM3 freeze: the pulse must end during debug halt. */
void IMU_HeaterPortInit(void) {
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    TIM3->CR1 = 0U;
    TIM3->CCER = 0U;
    TIM3->DIER = 0U;
    TIM3->SMCR = 0U;
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U) timer_clock *= 2U;
    TIM3->PSC = timer_clock / 1000000U - 1U;
    TIM3->ARR = 19999U;
    TIM3->CCR2 = 20000U;
    TIM3->CCMR1 = TIM_CCMR1_OC2M; /* OC2M=111: PWM2, CC2S=00 */
    TIM3->CR1 = TIM_CR1_OPM;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CNT = 0U;
    TIM3->SR = 0U;
    TIM3->CCER = TIM_CCER_CC2E;
    DBGMCU->APB1FZ &= ~DBGMCU_APB1_FZ_DBG_TIM3_STOP;
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = IMU_HEATER_Pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(IMU_HEATER_GPIO_Port, &gpio);
}

void IMU_HeaterPortWrite(float duty) {
    if (!(duty > 0.0f)) {
        TIM3->CCR2 = 20000U; /* force PWM2 inactive immediately */
        TIM3->CR1 &= ~TIM_CR1_CEN;
        TIM3->CNT = 0U;
        return;
    }
    if (TIM3->CR1 & TIM_CR1_CEN) return; /* do not extend an in-flight pulse */
    if (duty > 0.8f) duty = 0.8f;
    TIM3->CCR2 = 20000U - (uint32_t)(20000.0f * duty);
    TIM3->CNT = 0U;
    TIM3->SR = 0U;
    TIM3->CR1 = TIM_CR1_OPM | TIM_CR1_CEN;
}
