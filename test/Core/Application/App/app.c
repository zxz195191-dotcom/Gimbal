#include "headfile.h"

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


#define UART_RX_RING_SIZE 128

static volatile uint8_t uart_rx_ring[UART_RX_RING_SIZE];
static volatile uint16_t uart_rx_head = 0;
static volatile uint16_t uart_rx_tail = 0;
uint8_t uart2_rx_byte;
volatile bool ring_overflow  = false, ore = false;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    uint16_t next =
        (uint16_t)((uart_rx_head + 1U) % UART_RX_RING_SIZE);

    if (next != uart_rx_tail)
    {
      uart_rx_ring[uart_rx_head] = uart2_rx_byte;
      uart_rx_head = next;
    }else {
      ring_overflow  = true;
    }

    HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
  }
}



void UART_Poll_Command(int16_t target[Motor_Count]) {
  static char rx_buf[32];
  static uint8_t rx_len;

  while (uart_rx_tail != uart_rx_head) {

    if (uart_rx_tail == uart_rx_head) return;

    const uint8_t c = uart_rx_ring[uart_rx_tail];

    uart_rx_tail = ((uart_rx_tail + 1U) % UART_RX_RING_SIZE);

      if ((c == '\r') || (c == '\n')) {
            if (rx_len > 0U) {

              rx_buf[rx_len] = '\0';

              Parse_Command(rx_buf, target);

              rx_len = 0U;
            }
        }
        else if (rx_len < (sizeof(rx_buf) - 1U))
        {
          rx_buf[rx_len++] = (char)c;
        }
        else
        {
          rx_len = 0U;
        }
    }
}
