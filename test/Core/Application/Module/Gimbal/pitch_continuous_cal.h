#pragma once
#include <stdint.h>
/* 装车后的实际测试范围；扫描、校准、往返共用此处上端。 */
#define PITCH_CONT_MIN_DEG 0
#define PITCH_CONT_MAX_DEG 41
#define PITCH_CONT_STEP_DEG 1
#define PITCH_CONT_NODE_COUNT ((PITCH_CONT_MAX_DEG-PITCH_CONT_MIN_DEG)/PITCH_CONT_STEP_DEG+1U)
/* 保留轮次编号；校准结束后由pscan比较，不自动复扫。 */
typedef enum { PITCH_CONT_IDLE, PITCH_CONT_COLLECT, PITCH_CONT_COMPARE,
               PITCH_CONT_DONE, PITCH_CONT_STOPPED } Pitch_Cont_Pass_e;
typedef struct {
  float angle_deg, up_nm, down_nm, gravity_nm, friction_nm;
  uint8_t up_valid, down_valid, valid;
} Pitch_Cont_Node_t;
typedef struct {
  Pitch_Cont_Pass_e pass;
  uint8_t selected, table_ready, last_sample_reason;
  uint32_t samples_seen, feedback_skipped, last_rx_count, last_sample_ms;
  uint16_t up_nodes, down_nodes, valid_nodes;
  float valid_min_deg, valid_max_deg;
  char summary[180];
} Pitch_Cont_Status_t;
extern volatile Pitch_Cont_Status_t g_pitch_cont;
extern Pitch_Cont_Node_t pitch_cont_table[PITCH_CONT_NODE_COUNT];
uint8_t Pitch_Cont_Start(void);
uint8_t Pitch_Cont_Active(void);
void Pitch_Cont_Sample(uint32_t rx_count, uint32_t sample_ms,
                       float actual_deg, float speed_deg_s, float torque_nm);
int8_t Pitch_Cont_Finish_Lap(void);
void Pitch_Cont_Stop(uint8_t complete);
uint8_t Pitch_Cont_Lookup(float actual_deg, int8_t direction, float *torque_nm);

void Pitch_Cont_Init(void);
uint8_t Pitch_Cont_Export_Line(char *buf, unsigned capacity);
void Pitch_Cont_Export_Advance(void);
