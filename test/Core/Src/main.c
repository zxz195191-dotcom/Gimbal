/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "headfile.h"
#include "pitch_continuous_cal.h"
#include "spi.h"
#include "imu_mpu6500.h"
#include "imu_attitude.h"
#include "imu_heater.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define TIME_TRIGGER(now,last,period) \
  for(uint32_t __t = (now) ; ((__t) - (last)) >= (period) ; (last) = __t)

volatile uint32_t g_tx_ret = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t Key_State(void) {
  // 由 TIME_TRIGGER 每 10ms 采样一次，采样周期本身已消抖，直接读引脚（非阻塞）
  return HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
}

void Set_Led_State(uint8_t state) {
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, state);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, state);
  // LED3 留给 CAN 错误诊断用，不再随按键变化
}

#if DM_ID_CONFIG_MODE
static void DM_Id_Config_Led_Update(uint32_t now_ms) {
  GPIO_PinState led1 = GPIO_PIN_RESET;
  GPIO_PinState led2 = GPIO_PIN_RESET;
  GPIO_PinState led3 = GPIO_PIN_RESET;

  switch (g_dm_id_config.state) {
    case DM_ID_CONFIG_WAIT_KEY:
      led1 = ((now_ms / 500U) & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
      break;
    case DM_ID_CONFIG_RUNNING:
      led2 = GPIO_PIN_SET;
      break;
    case DM_ID_CONFIG_SUCCESS:
      led1 = GPIO_PIN_SET;
      led2 = GPIO_PIN_SET;
      break;
    case DM_ID_CONFIG_SAVED_REBOOT_REQUIRED:
      led1 = GPIO_PIN_SET;
      led2 = ((now_ms / 250U) & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
      break;
    case DM_ID_CONFIG_FAILED:
    default:
      led3 = GPIO_PIN_SET;
      break;
  }

  /* Board LEDs are active-low; ledN variables above mean logical "on". */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,
                    (led1 == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,
                    (led2 == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin,
                    (led3 == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
#endif


// Debugger-only diagnostics; the CSV contains six angles plus successful-send interval.
volatile uint32_t g_uart_tx_started, g_uart_tx_busy_skips;
volatile uint32_t g_uart_tx_errors, g_uart_tx_recoveries;
volatile uint32_t g_uart_tx_last_start_ms;

// USART2 115200, DMA. Never queue old telemetry behind a busy transfer.
static uint8_t UART_Debug_Send(const char *msg, uint8_t telemetry) {
  static uint8_t tx_buffer[320];
  static uint32_t busy_since_ms;
  static uint8_t busy_seen, resync;
  static uint8_t have_previous_tx;
  static uint32_t previous_tx_cycles;
  if (msg == NULL) return 0;
  const uint32_t now = HAL_GetTick();
  if (huart2.gState != HAL_UART_STATE_READY) {
    ++g_uart_tx_busy_skips;
    if (!busy_seen) { busy_seen = 1U; busy_since_ms = now; }
    if ((uint32_t)(now - busy_since_ms) < 100U) return 0;
    // Even a full 320-byte buffer takes less than 28 ms at 115200/8N1.
    // Abort TX only; command reception must remain armed.
    busy_since_ms = now;
    if (HAL_UART_AbortTransmit(&huart2) != HAL_OK) {
      ++g_uart_tx_errors;
      return 0;
    }
    ++g_uart_tx_recoveries;
    resync = 1U;
  }
  busy_seen = 0U;
  const size_t prefix = resync ? 2U : 0U;
  size_t length = strlen(msg);
  while (length > 0U && (msg[length - 1U] == '\r' || msg[length - 1U] == '\n')) --length;
  // Reserve interval, two feedback torques, CRLF and terminating NUL.
  if (length == 0U || length > sizeof(tx_buffer) - prefix - (telemetry ? 48U : 3U)) {
    ++g_uart_tx_errors;
    return 0; // Never transmit a truncated CSV row.
  }
  if (resync) { tx_buffer[0] = '\r'; tx_buffer[1] = '\n'; }
  memcpy(tx_buffer + prefix, msg, length);
  // Read once after copying the six angles and checking DMA readiness.
  // Only interval formatting remains before DMA starts; this is a software
  // launch timestamp, not the exact first-bit time or the IMU sample time.
  if (!telemetry) {
    tx_buffer[prefix+length]='\r'; tx_buffer[prefix+length+1]='\n';
    if (HAL_UART_Transmit_DMA(&huart2,tx_buffer,(uint16_t)(prefix+length+2))!=HAL_OK) return 0;
    resync=0; return 1;
  }
  const float p_t_int = dm_motor[Pitch].t_int;
  const float y_t_int = dm_motor[Yaw].t_int;
  const uint32_t dwt_cycles = DWT->CYCCNT;
  // Unsigned subtraction handles CYCCNT wrap when successful sends are less
  // than one wrap period apart (~23.861 s at 180 MHz). First frame has no interval.
  const double tx_dt_ms = have_previous_tx ?
      (double)(uint32_t)(dwt_cycles - previous_tx_cycles) * 1000.0 / (double)SystemCoreClock : 0.0;
  const int tail = snprintf((char *)tx_buffer + prefix + length,
                            sizeof(tx_buffer) - prefix - length,
                            ",%.3f,%.3f,%.3f\r\n", tx_dt_ms, (double)p_t_int, (double)y_t_int);
  if (tail <= 0 || (size_t)tail >= sizeof(tx_buffer) - prefix - length) {
    ++g_uart_tx_errors;
    return 0;
  }
  if (HAL_UART_Transmit_DMA(&huart2, tx_buffer, (uint16_t)(length + prefix + (size_t)tail)) == HAL_OK) {
    previous_tx_cycles = dwt_cycles;
    have_previous_tx = 1U;
    ++g_uart_tx_started;
    g_uart_tx_last_start_ms = now;
    resync = 0U;
    return 1;
  } else {
    ++g_uart_tx_errors;
  }
  return 0;
}
void UART_Debug(const char *msg) { (void)UART_Debug_Send(msg,1U); }

#if !DM_ID_CONFIG_MODE
static void Gimbal_Status_Led_Update(void) {
  const GPIO_PinState yaw_level =
      (g_yaw_test.state == YAW_TEST_ACTIVE) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  const GPIO_PinState pitch_level =
      (g_pitch_test.state == YAW_TEST_ACTIVE) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  const GPIO_PinState fault_level =
      ((g_yaw_test.state == YAW_TEST_FAULT) ||
       (g_pitch_test.state == YAW_TEST_FAULT))
          ? GPIO_PIN_RESET : GPIO_PIN_SET;

  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, yaw_level);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, pitch_level);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, fault_level);
}
#endif

static uint32_t cycles_per_us = 0;

void DWT_Init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  cycles_per_us = SystemCoreClock / 1000000;
}

uint32_t DWT_Read(void) {
  return DWT->CYCCNT / cycles_per_us;
}

#define RAD_TO_DEG 57.295779513f




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  if (HAL_RCC_DeInit() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
  MX_CAN2_Init();
  MX_SPI5_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init();
  IMU_HeaterInit();
  IMU_SetSampleCallback(IMU_AttitudeProcess);
  IMU_Init();
  Can_Dispatch_Init();

#if DM_ID_CONFIG_MODE
  DM_Motor_Id_Config_Init();
#else
  int16_t target_command_rpm[Motor_Count] = {0};

  Shooter_Init(target_command_rpm);
  Gimbal_Yaw_Test_Init();
#if VISION_PROTOCOL_ENABLED
  const uint8_t vision_ready = Vision_Protocol_Init() ? 1U : 0U;
#endif

  uint32_t last_can_tx = 0;
#if VISION_PROTOCOL_ENABLED
  uint32_t last_vision_tx = 0;
  VisionMode_e vision_mode = VISION_MODE_AIM;
#else
  uint32_t last_dbg_ms = 0;
#endif
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    IMU_Service();
    IMU_Sample_t imu_latest;
    const uint8_t imu_stream_ok = IMU_GetSample(&imu_latest);
    IMU_HeaterService(HAL_GetTick(), &imu_latest, imu_stream_ok);
    IMU_AttitudeService(HAL_GetTick(), imu_stream_ok, g_imu_heater.stable);


#if DM_ID_CONFIG_MODE
    const uint32_t now_ms = HAL_GetTick();
    DM_Motor_Id_Config_Update(now_ms, HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin));
    DM_Id_Config_Led_Update(now_ms);
#else
    uint32_t now_us = DWT_Read();
    uint32_t now_ms = HAL_GetTick();

    Gimbal_Yaw_Test_Update(now_ms, HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin));

#if VISION_PROTOCOL_ENABLED
    Vision_Protocol_Service();

    VisionToGimbalCommand_t vision_command;
    if (vision_ready && Vision_Get_Latest_Command(
                            &vision_command, now_ms)) {
      if (vision_command.mode == VISION_MODE_DISABLED) {
        Gimbal_Stop_All();
      } else {
        Gimbal_Enable_All();
        (void)Gimbal_Yaw_Set_Max_Speed_Deg(
            fabsf(vision_command.target_yaw_vel_rad_s) * RAD_TO_DEG);
        (void)Gimbal_Pitch_Set_Max_Speed_Deg(
            fabsf(vision_command.target_pitch_vel_rad_s) * RAD_TO_DEG);
        (void)Gimbal_Yaw_Set_Target_Rad(vision_command.target_yaw_rad);
        (void)Gimbal_Pitch_Set_Target_Rad(vision_command.target_pitch_rad);
      }
    }
#else
    UART_Poll_Command(target_command_rpm);
    const uint8_t command_ack_visible = UART_Command_Ack_Update(now_ms);
    if (!command_ack_visible) {
      Gimbal_Status_Led_Update();
    }
#endif

#if VISION_PROTOCOL_ENABLED
    TIME_TRIGGER(now_us, last_vision_tx, VISION_TX_PERIOD_US) {
      if (vision_ready) {
        GimbalToVisionData_t vision_state = {
            .mode = vision_mode,
            .q = {1.0f, 0.0f, 0.0f, 0.0f},
            .yaw_rad = g_yaw_test.measured_relative_rad,
            .yaw_vel_rad_s = g_yaw_test.measured_velocity_rad_s,
            .pitch_rad = g_pitch_test.measured_relative_rad,
            .pitch_vel_rad_s = g_pitch_test.measured_velocity_rad_s,
            .bullet_speed_m_s = 23.0f,
            .bullet_count = 0U,
        };
        (void)Vision_Send_Gimbal_State(&vision_state);
      }
    }
#endif

    TIME_TRIGGER(now_us,last_can_tx,CAN_TX_PERIOD_US) {
      for (uint8_t i = 0; i < Motor_Count; i++)
      {
        MotorID_e id = (MotorID_e)i;
        target_speed_rpm[id] = (float)target_command_rpm[id];

        if (Motor_Feedback_Valid(id, now_us))
        {
          current_cmd[id] = (int16_t)PID_Calculate(
              &pid_spd[id], motor_feedback[id].speed_rmp,
              target_speed_rpm[id]);
        }
        else
        {
          current_cmd[id] = 0;
          PID_Reset(&pid_spd[id]);
        }
      }

      // The four 0x200 data slots map to feedback IDs 0x201 through 0x204.
      g_tx_ret = CAN_Send_Motor_Current(
          current_cmd[Friction_Left], current_cmd[Friction_Right],
          current_cmd[Feeder], current_cmd[DJI_Motor_Reserved]);
  }

#if !VISION_PROTOCOL_ENABLED
  TIME_TRIGGER(HAL_GetTick(), last_dbg_ms, DBG_PRINT_PERIOD_US / 1000U)
  {
    char report[320];
    if (Gimbal_Load_Report_Line(report,sizeof(report))) {
      if (UART_Debug_Send(report,0U)) Gimbal_Load_Report_Advance();
    } else if (!Gimbal_Load_Telemetry_Paused()) {
    /* Fixed FireWater CSV: six angles in degrees plus successful-send interval in ms:
     * p_cmd,p_enc,y_cmd,y_enc,imu_pitch,imu_yaw,tx_dt_ms,p_t_int,y_t_int.
     * Commands are planned control setpoints, not final user targets.
     * Encoders retain the first-enable zero; do not subtract the IMU origin.
     * Display pitch sign matches the measured installation; fusion is unchanged.
     * Yaw is wrapped ZYX heading (+left in the native sensor frame).
     * Before publication IMU values are zero; on a fault they hold the last
     * published sample. Validity/fault details remain available in the debugger.
     * In inertial Yaw mode y_cmd uses the inertial origin, while y_enc remains
     * encoder-relative. Their difference is only meaningful in encoder mode.
     */
    IMU_AttitudeSample_t attitude;
    (void)IMU_AttitudeGet(HAL_GetTick(), &attitude);
    char dbg_buf[320];
    snprintf(dbg_buf, sizeof(dbg_buf),
             "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
             g_pitch_test.command_relative_rad * RAD_TO_DEG,
             g_pitch_test.measured_relative_rad * RAD_TO_DEG,
             g_yaw_test.command_relative_rad * RAD_TO_DEG,
             g_yaw_test.measured_relative_rad * RAD_TO_DEG,
             -attitude.pitch_up_rad * RAD_TO_DEG,
             attitude.yaw_left_rad * RAD_TO_DEG);
    UART_Debug(dbg_buf);
    }
  }
#endif
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
  if (pin == MPU6500_INT_Pin) IMU_OnDataReady();
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *spi) {
  if (spi == &hspi5) IMU_OnTransferComplete();
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *spi) {
  if (spi == &hspi5) IMU_OnSpiError(spi->ErrorCode);
}

/**
  * @brief  CAN FIFO0 接收中断回调
  * @note   必须在此取出消息并释放 FIFO0，否则 C620 持续回发反馈帧时，
  *         RX_FIFO0_MSG_PENDING 会反复触发最高优先级中断，卡死主循环。
  */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
