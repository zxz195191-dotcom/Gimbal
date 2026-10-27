#include "yaw_diagnostics.h"
#include "headfile.h"
#include "pitch_continuous_cal.h"
#include "imu_mpu6500.h"
#include "imu_attitude.h"
#include "imu_heater.h"

#define COMMAND_ACK_HALF_PERIOD_MS 100U
#define COMMAND_ACK_HALF_CYCLES    6U

static uint8_t command_ack_active;
static uint32_t command_ack_start_ms;
static uint8_t command_parse_ok;

static void UART_Command_Ack_Trigger(void) {
  command_ack_start_ms = HAL_GetTick();
  command_ack_active = 1U;
}




uint8_t UART_Command_Ack_Update(uint32_t now_ms) {
  if (!command_ack_active) return 0U;

  const uint32_t phase =
      (now_ms - command_ack_start_ms) / COMMAND_ACK_HALF_PERIOD_MS;
  if (phase >= COMMAND_ACK_HALF_CYCLES) {
    command_ack_active = 0U;
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    return 0U;
  }

  /* Board LEDs are active-low. Even phases are the visible flashes. */
  const GPIO_PinState level =
      ((phase & 1U) == 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, level);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, level);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, level);
  return 1U;
}






static void UART_Command_Result(const char *name, uint8_t ok) {
  /* Keep the UART stream numeric-only for VOFA+. LEDs acknowledge reception. */
  (void)name;
  command_parse_ok = ok;
}

static uint8_t Parse_Axis_Target(const char *cmd, uint8_t is_yaw) {
  const char *arg = cmd + 1;
  float target_deg;
  float speed_deg_s;
  char extra;

  while ((*arg == '=') || (*arg == ':') ||
         (*arg == ' ') || (*arg == '\t')) {
    ++arg;
  }

  if (sscanf(arg, "%f %f %c", &target_deg, &speed_deg_s, &extra) != 2) {
    return 0U;
  }
  if (is_yaw) {
    return Gimbal_Yaw_Set_Target_Speed_Deg(target_deg, speed_deg_s);
  }
  return Gimbal_Pitch_Set_Target_Speed_Deg(target_deg, speed_deg_s);
}

static uint8_t Parse_Axis_Relative_Target(const char *cmd,
                                          uint8_t is_yaw) {
  float delta_deg;
  float speed_deg_s;
  char extra;

  if (sscanf(cmd, "%*s %f %f %c",
             &delta_deg, &speed_deg_s, &extra) != 2) {
    return 0U;
  }
  if (is_yaw) {
    return Gimbal_Yaw_Move_Relative_Speed_Deg(delta_deg, speed_deg_s);
  }
  return Gimbal_Pitch_Move_Relative_Speed_Deg(delta_deg, speed_deg_s);
}

static void Parse_Command(const char *cmd, int16_t target[Motor_Count]) {
  char name[12] = {0};
  float value = 0.0f;

  while ((*cmd == ' ') || (*cmd == '\t')) ++cmd;
  command_parse_ok = 0U;

  /* Absolute target in the boot-zero frame plus maximum approach speed. */
  if (((cmd[0] == 'y') || (cmd[0] == 'Y')) &&
      !isalpha((unsigned char)cmd[1])) {
    UART_Command_Result("y", Parse_Axis_Target(cmd, 1U));
    return;
  }
  if (((cmd[0] == 'p') || (cmd[0] == 'P')) &&
      !isalpha((unsigned char)cmd[1])) {
    UART_Command_Result("p", Parse_Axis_Target(cmd, 0U));
    return;
  }

  if (sscanf(cmd, "%11s", name) != 1) return;
  for (uint8_t i = 0U; name[i] != '\0'; ++i) {
    name[i] = (char)tolower((unsigned char)name[i]);
  }

  if (strcmp(name,"yprobe")==0) {
    unsigned request_id; char extra;
    const uint8_t ok=sscanf(cmd,"%*s %u %c",&request_id,&extra)==1;
    if (ok) Gimbal_Yaw_Probe_Request((uint32_t)request_id);
    UART_Command_Result("yprobe",ok); return;
  }
  if (strcmp(name,"ydiag")==0) {
    char action[8], extra;
    uint8_t ok=sscanf(cmd,"%*s %7s %c",action,&extra)==1;
    unsigned operation=3;
    if (ok) {
      if (strcmp(action,"arm")==0) operation=0;
      else if (strcmp(action,"stop")==0) operation=1;
      else if (strcmp(action,"dump")==0) operation=2;
      ok=Gimbal_Yaw_Diag_Command(operation);
    }
    UART_Command_Result("ydiag",ok); return;
  }
  if (strcmp(name,"ycalarm")==0 || strcmp(name,"ycal")==0 ||
      strcmp(name,"ycalstop")==0 || strcmp(name,"ydump")==0 || strcmp(name,"ystat")==0) {
    char extra; uint8_t ok=sscanf(cmd,"%*s %c",&extra)!=1;
    if (ok) {
      if (strcmp(name,"ycalarm")==0) ok=Gimbal_Yaw_Cal_Arm();
      else if (strcmp(name,"ycal")==0) ok=Gimbal_Yaw_Cal_Start();
      else if (strcmp(name,"ycalstop")==0) Gimbal_Yaw_Cal_Stop();
    }
    Gimbal_Load_Command_Result(name,ok);
    UART_Command_Result(name,ok); return;
  }
  if (strcmp(name,"yboost")==0) {
    unsigned enable; char extra;
    const uint8_t ok=sscanf(cmd,"%*s %u %c",&enable,&extra)==1 && enable<=1 &&
        Gimbal_Yaw_Position_Boost_Enable((uint8_t)enable);
    UART_Command_Result(name,ok); return;
  }
  if (strcmp(name,"yff")==0 || strcmp(name,"yi")==0 || strcmp(name,"pi")==0) {
    unsigned enable; char extra;
    uint8_t ok=sscanf(cmd,"%*s %u %c",&enable,&extra)==1 && enable<=1;
    if (ok) ok=strcmp(name,"yff")==0 ? Gimbal_Yaw_Friction_Enable((uint8_t)enable) :
      Gimbal_Load_Integral_Enable(name[0]=='y'?Yaw:Pitch,(uint8_t)enable);
    Gimbal_Load_Command_Result(name,ok);
    UART_Command_Result(name,ok); return;
  }
  if (strcmp(name, "imu") == 0) {
    unsigned mode;
    char extra;
    uint8_t ok = (sscanf(cmd, "%*s %u %c", &mode, &extra) == 1) && (mode <= 5U);
    /* Legacy imu 0..5 commands remain accepted; telemetry is always six columns. */
    if (ok) g_imu_log_mode = 0U;
    UART_Command_Result("imu", ok);
    return;
  }
  if (strcmp(name, "imureset") == 0) {
    char extra;
    const uint8_t ok = (sscanf(cmd, "%*s %c", &extra) != 1);
    if (ok) IMU_RequestRestart();
    UART_Command_Result("imureset", ok);
    return;
  }
  if (strcmp(name, "imucal") == 0) {
    char extra;
    uint8_t ok = sscanf(cmd, "%*s %c", &extra) != 1;
    if (ok) IMU_AttitudeRequestCalibration();
    UART_Command_Result("imucal", ok);
    return;
  }
  if (strcmp(name, "imuheat") == 0) {
    unsigned enabled;
    char extra;
    uint8_t ok = sscanf(cmd, "%*s %u %c", &enabled, &extra) == 1 && enabled <= 1U;
    if (ok) IMU_HeaterSetEnabled((uint8_t)enabled);
    UART_Command_Result("imuheat", ok);
    return;
  }
  if (strcmp(name, "ge") == 0) {
    Gimbal_Enable_All();
    UART_Command_Result("ge", 1U);
    return;
  }
  if (strcmp(name, "gd") == 0) {
    Gimbal_Stop_All();
    UART_Command_Result("gd", 1U);
    return;
  }
  if (strcmp(name, "yr") == 0) {
    UART_Command_Result("yr", Parse_Axis_Relative_Target(cmd, 1U));
    return;
  }
  if (strcmp(name, "pr") == 0) {
    UART_Command_Result("pr", Parse_Axis_Relative_Target(cmd, 0U));
    return;
  }
  if (strcmp(name,"pmode")==0) {
    unsigned mode; char extra;
    const uint8_t ok=sscanf(cmd,"%*s %u %c",&mode,&extra)==1 && mode<=1U &&
        Gimbal_Pitch_Set_Mode((Pitch_Control_Mode_t)mode);
    Gimbal_Pitch_Probe();
    UART_Command_Result("pmode",ok); return;
  }
  if (strcmp(name,"pprobe")==0) {
    char extra;
    const uint8_t ok=sscanf(cmd,"%*s %c",&extra)!=1;
    if (ok) Gimbal_Pitch_Probe();
    UART_Command_Result("pprobe",ok); return;
  }
  if (strcmp(name, "ymode") == 0) {
    unsigned mode;
    char extra;
    const uint8_t ok = sscanf(cmd, "%*s %u %c", &mode, &extra) == 1 &&
        mode <= 1U && Gimbal_Yaw_Set_Mode((Yaw_Control_Mode_t)mode);
    if (ok) g_imu_log_mode = 0U;
    UART_Command_Result("ymode", ok);
    return;
  }
  if (strcmp(name, "ye") == 0) {
    Gimbal_Yaw_Enable();
    UART_Command_Result("ye", g_yaw_test.state != YAW_TEST_FAULT);
    return;
  }
  if (strcmp(name, "yd") == 0) {
    Gimbal_Yaw_Test_Stop();
    UART_Command_Result("yd", 1U);
    return;
  }
  if (strcmp(name, "pe") == 0) {
    Gimbal_Pitch_Enable();
    UART_Command_Result("pe", g_pitch_test.state != YAW_TEST_FAULT);
    return;
  }
  if (strcmp(name, "pcal") == 0) {
    char extra;
    const uint8_t ok = (sscanf(cmd, "%*s %c", &extra) != 1) && Pitch_Cont_Start();
    if (ok) g_imu_log_mode = 0U;
    UART_Command_Result("pcal", ok);
    return;
  }
  if (strcmp(name, "ptest") == 0 || strcmp(name, "ytest") == 0) {
    const uint8_t is_yaw = name[0] == 'y';
    const float center_deg = Gimbal_Yaw_Control_Angle_Rad() * 57.2957795f;
    float min_deg = is_yaw ? center_deg - 10.0f : 25.0f;
    float max_deg = is_yaw ? center_deg + 10.0f : PITCH_SCAN_MAX_ACTUAL_DEG;
    float speed_deg_s = 5.0f;
    char extra;
    const uint8_t no_arg = sscanf(cmd, "%*s %c", &extra) != 1;
    uint8_t ok = no_arg || sscanf(cmd, "%*s %f %f %f %c",
        &min_deg, &max_deg, &speed_deg_s, &extra) == 3;
    if (ok) ok = is_yaw ? Gimbal_Yaw_Repeat_Start(min_deg, max_deg, speed_deg_s) :
                          Gimbal_Pitch_Repeat_Start(min_deg, max_deg, speed_deg_s);
    if (ok) g_imu_log_mode = 0U;
    UART_Command_Result(name, ok);
    return;
  }
  if (strcmp(name, "ycancel") == 0) {
    char extra;
    const uint8_t ok = sscanf(cmd, "%*s %c", &extra) != 1;
    if (ok) Gimbal_Yaw_Repeat_Cancel();
    UART_Command_Result(name, ok);
    return;
  }
  if (strcmp(name, "pscan") == 0) {
    char extra;
    const uint8_t no_arg = (sscanf(cmd, "%*s %c", &extra) != 1);
    const uint8_t ok = no_arg && Gimbal_Pitch_Scan_Start();
    if (ok) g_imu_log_mode = 0U;
    UART_Command_Result("pscan", ok);
    return;
  }
  if (strcmp(name, "pcancel") == 0) {
    char extra;
    const uint8_t ok = (sscanf(cmd, "%*s %c", &extra) != 1);
    if (ok) Gimbal_Pitch_Scan_Cancel();
    UART_Command_Result("pcancel", ok);
    return;
  }
  if (strcmp(name, "pd") == 0) {
    Gimbal_Pitch_Stop();
    UART_Command_Result("pd", 1U);
    return;
  }
  if (strcmp(name, "ykp") == 0) {
    const uint8_t ok = (sscanf(cmd, "%*s %f", &value) == 1) &&
                       Gimbal_Yaw_Set_Kp(value);
    UART_Command_Result("ykp", ok);
    return;
  }
  if (strcmp(name, "ykd") == 0) {
    const uint8_t ok = (sscanf(cmd, "%*s %f", &value) == 1) &&
                       Gimbal_Yaw_Set_Kd(value);
    UART_Command_Result("ykd", ok);
    return;
  }
  if (strcmp(name, "pkp") == 0) {
    const uint8_t ok = (sscanf(cmd, "%*s %f", &value) == 1) &&
                       Gimbal_Pitch_Set_Kp(value);
    UART_Command_Result("pkp", ok);
    return;
  }
  if (strcmp(name, "pkd") == 0) {
    const uint8_t ok = (sscanf(cmd, "%*s %f", &value) == 1) &&
                       Gimbal_Pitch_Set_Kd(value);
    UART_Command_Result("pkd", ok);
    return;
  }
  if (strcmp(name, "pff") == 0) {
    char extra = '\0';
    const uint8_t ok = (sscanf(cmd, "%*s %f %c", &value, &extra) == 1) &&
                       Gimbal_Pitch_Set_Gravity_FF_Scale(value);
    UART_Command_Result("pff", ok);
    return;
  }
  if (strcmp(name, "stop") == 0) {
    Gimbal_Stop_All();
    Shooter_Stop(target);
    UART_Command_Result("stop", 1U);
    return;
  }
  if (strcmp(name, "move") == 0) {
    char extra = '\0';
    const uint8_t ok = (sscanf(cmd, "%*s %f %c", &value, &extra) == 1) &&
                       Shooter_Set_Friction_Percent(target, value);
    UART_Command_Result("move", ok);
    return;
  }
  if ((strcmp(name, "friction") == 0) ||
      (strcmp(name, "fr") == 0)) {
    char extra = '\0';
    const uint8_t ok = (sscanf(cmd, "%*s %f %c", &value, &extra) == 1) &&
                       Shooter_Set_Friction_Rpm(target, value);
    UART_Command_Result("friction", ok);
    return;
  }
  if (strcmp(name, "feed") == 0) {
    char extra = '\0';
    const uint8_t ok = (sscanf(cmd, "%*s %f %c", &value, &extra) == 1) &&
                       Shooter_Set_Feeder_Percent(target, value);
    UART_Command_Result("feed", ok);
    return;
  }
  if (strcmp(name, "feedrpm") == 0) {
    char extra = '\0';
    const uint8_t ok = (sscanf(cmd, "%*s %f %c", &value, &extra) == 1) &&
                       Shooter_Set_Feeder_Rpm(target, value);
    UART_Command_Result("feedrpm", ok);
    return;
  }
  if (strcmp(name, "shoot") == 0) {
    float friction_percent;
    float feeder_percent;
    char extra = '\0';
    const uint8_t ok =
        (sscanf(cmd, "%*s %f %f %c",
                &friction_percent, &feeder_percent, &extra) == 2) &&
        Shooter_Set_All(target, friction_percent, feeder_percent);
    UART_Command_Result("shoot", ok);
    return;
  }
  if ((strcmp(name, "ss") == 0) ||
      (strcmp(name, "shootstop") == 0)) {
    Shooter_Stop(target);
    UART_Command_Result("shootstop", 1U);
    return;
  }

  UART_Command_Result("cmd", 0U);
}


#define UART_RX_RING_SIZE 128U
static volatile uint8_t uart_rx_ring[UART_RX_RING_SIZE];
static volatile uint16_t uart_rx_head, uart_rx_tail;
static volatile uint8_t uart_rx_restart_pending;
uint8_t uart2_rx_byte;
volatile bool ring_overflow = false, ore = false;
// Inspect in debugger without changing the wire protocol.
volatile uint32_t g_uart_rx_errors, g_uart_rx_last_error;
volatile uint32_t g_uart_rx_recoveries, g_uart_rx_bytes;
volatile uint32_t g_uart_command_count, g_uart_command_rejected;
volatile uint32_t g_uart_poll_ms;

void UART_Command_RxError(uint32_t error) {
  ++g_uart_rx_errors;
  g_uart_rx_last_error = error;
  if (error & HAL_UART_ERROR_ORE) ore = true;
  uart_rx_restart_pending = 1U;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart != &huart2) return;
  ++g_uart_rx_bytes;
  const uint16_t next = (uint16_t)((uart_rx_head + 1U) % UART_RX_RING_SIZE);
  if (next != uart_rx_tail) {
    uart_rx_ring[uart_rx_head] = uart2_rx_byte;
    uart_rx_head = next;
  } else {
    ring_overflow = true;
    uart_rx_restart_pending = 1U;
  }
  if (HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1) != HAL_OK)
    uart_rx_restart_pending = 1U;
}

void UART_Poll_Command(int16_t target[Motor_Count]) {
  static char rx_buf[32];
  static uint8_t rx_len, discard_line;
  g_uart_poll_ms = HAL_GetTick();
  // Bound work per loop so continuous host traffic cannot starve control/IMU.
  for (unsigned consumed = 0U; consumed < UART_RX_RING_SIZE; ++consumed) {
    if (uart_rx_restart_pending) {
      // Debug mode uses byte interrupts, not RX DMA. Keep this critical section
      // short; TX is independent and must not be aborted here.
      const uint32_t primask = __get_PRIMASK();
      __disable_irq();
      (void)HAL_UART_AbortReceive(&huart2);
      __HAL_UART_CLEAR_OREFLAG(&huart2);
      uart_rx_tail = uart_rx_head;
      rx_len = 0U;
      discard_line = 1U; // Never execute the tail of a damaged command.
      uart_rx_restart_pending = 0U;
      if (HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1) != HAL_OK)
        uart_rx_restart_pending = 1U;
      else
        ++g_uart_rx_recoveries;
      if (!primask) __enable_irq();
      return;
    }
    if (uart_rx_tail == uart_rx_head) return;
    const uint8_t c = uart_rx_ring[uart_rx_tail];
    uart_rx_tail = (uint16_t)((uart_rx_tail + 1U) % UART_RX_RING_SIZE);
    if (c == '\r' || c == '\n') {
      if (!discard_line && rx_len > 0U) {
        rx_buf[rx_len] = '\0';
        // An ISR may have flagged a lost byte since this iteration began.
        if (uart_rx_restart_pending) continue;
        Parse_Command(rx_buf, target);
        ++g_uart_command_count;
        if (!command_parse_ok) ++g_uart_command_rejected;
      }
      rx_len = 0U;
      discard_line = 0U;
    } else if (!discard_line) {
      if (rx_len < sizeof(rx_buf) - 1U) rx_buf[rx_len++] = (char)c;
      else { rx_len = 0U; discard_line = 1U; ++g_uart_command_rejected; }
    }
  }
}
