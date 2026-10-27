#include "headfile.h"
#include "pitch_continuous_cal.h"
#include <math.h>
#include "feedforward_saved.h"

#define RAD_TO_DEG 57.29577951308232f
volatile Pitch_Cont_Status_t g_pitch_cont;
Pitch_Cont_Node_t pitch_cont_table[PITCH_CONT_NODE_COUNT];
static Pitch_Cont_Node_t saved_table[PITCH_CONT_NODE_COUNT];
static Pitch_Cont_Status_t saved_status;
static float saved_scale;
static unsigned export_line;
static uint8_t export_pending, export_failed;

void Pitch_Cont_Init(void) {
  memset((void *)&g_pitch_cont,0,sizeof(g_pitch_cont));
  memset(pitch_cont_table,0,sizeof(pitch_cont_table));
  export_pending=export_failed=0;
  (void)Pitch_FF_Set_Scale(0);
#if PITCH_SAVED_PRESENT
  _Static_assert(sizeof(pitch_saved_up_nm)/sizeof(float)==PITCH_CONT_NODE_COUNT,"Pitch up count");
  _Static_assert(sizeof(pitch_saved_down_nm)/sizeof(float)==PITCH_CONT_NODE_COUNT,"Pitch down count");
  for (unsigned i=0;i<PITCH_CONT_NODE_COUNT;i++)
    if (!isfinite(pitch_saved_up_nm[i]) || !isfinite(pitch_saved_down_nm[i])) return;
  for (unsigned i=0;i<PITCH_CONT_NODE_COUNT;i++) {
    const float u=pitch_saved_up_nm[i], d=pitch_saved_down_nm[i];
    pitch_cont_table[i]=(Pitch_Cont_Node_t){PITCH_CONT_MIN_DEG+i*PITCH_CONT_STEP_DEG,
      u,d,(u+d)*.5f,(u-d)*.5f,1,1,1};
  }
  g_pitch_cont.selected=g_pitch_cont.table_ready=1;
  g_pitch_cont.up_nodes=g_pitch_cont.down_nodes=g_pitch_cont.valid_nodes=PITCH_CONT_NODE_COUNT;
  g_pitch_cont.valid_min_deg=PITCH_CONT_MIN_DEG;
  g_pitch_cont.valid_max_deg=PITCH_CONT_MAX_DEG;
  g_pitch_cont.pass=PITCH_CONT_DONE;
  (void)Pitch_FF_Set_Scale(1);
#endif
}
static void restore_table(void) {
  memcpy(pitch_cont_table,saved_table,sizeof(saved_table));
  g_pitch_cont=saved_status;
  (void)Pitch_FF_Set_Scale(saved_scale);
}
/* Existing UART report queue: advance only after DMA accepts each line.
 * Nine significant digits round-trip binary32 without changing table values. */
uint8_t Pitch_Cont_Export_Line(char *buf, unsigned capacity) {
  if (!export_pending) return 0;
  if (export_failed) {
    snprintf(buf,capacity,"/* PCAL incomplete: previous table/scale restored; no constants exported */\r\n");
    return 1;
  }
  const unsigned span=PITCH_CONT_NODE_COUNT+2U;
  const unsigned row=export_line%span;
  const uint8_t down=export_line>=span;
  if (!row) snprintf(buf,capacity,"static const float pitch_saved_%s_nm[] = {\r\n",down?"down":"up");
  else if (row==span-1) snprintf(buf,capacity,"};\r\n");
  else snprintf(buf,capacity,"  %.9ef,%s\r\n",
      down?pitch_cont_table[row-1].down_nm:pitch_cont_table[row-1].up_nm,
      row==PITCH_CONT_NODE_COUNT?" /* 41 deg */":"");
  return 1;
}
void Pitch_Cont_Export_Advance(void) {
  if (export_failed || ++export_line>=2U*(PITCH_CONT_NODE_COUNT+2U)) export_pending=0;
}
static struct {
  float actual_deg, torque_nm;
  uint8_t valid, prepare_up;
} previous;

uint8_t Pitch_Cont_Active(void) { return g_pitch_cont.pass==PITCH_CONT_COLLECT; }
uint8_t Pitch_Cont_Start(void) {
  if (export_pending || Pitch_Cont_Active() || !Gimbal_Pitch_Scan_Start()) return 0;
  memcpy(saved_table,pitch_cont_table,sizeof(saved_table));
  saved_status=g_pitch_cont;
  saved_scale=Pitch_FF_Get_Scale();
  /* 只有pcal为重新采集关闭补偿；pscan从不修改补偿开关。 */
  (void)Pitch_FF_Set_Scale(0.0f);
  memset((void *)&g_pitch_cont,0,sizeof(g_pitch_cont));
  memset(&previous,0,sizeof(previous));
  previous.prepare_up=g_pitch_test.measured_relative_rad*RAD_TO_DEG<=PITCH_CONT_MIN_DEG;
  for (unsigned i=0;i<PITCH_CONT_NODE_COUNT;i++) {
    pitch_cont_table[i]=(Pitch_Cont_Node_t){
      .angle_deg=PITCH_CONT_MIN_DEG+i*PITCH_CONT_STEP_DEG,
      .up_nm=NAN,.down_nm=NAN,.gravity_nm=NAN,.friction_nm=NAN};
  }
  g_pitch_cont.valid_min_deg=g_pitch_cont.valid_max_deg=NAN;
  g_pitch_cont.selected=1;
  g_pitch_cont.pass=PITCH_CONT_COLLECT;
  g_pitch_cont.last_rx_count=dm_motor[Pitch].rx_count;
  /* pcal先启动共用轨迹再关前馈，修正启动摘要，避免显示上轮开关。 */
  snprintf((char *)g_pitch_scan.start_summary,sizeof(g_pitch_scan.start_summary),
           "pcal kp=%.2f kd=%.2f zero_raw=%.2fdeg scan=%d..%d FF=0",
           g_pitch_test.kp,g_pitch_test.kd,g_pitch_test.origin_rad*RAD_TO_DEG,
           PITCH_CONT_MIN_DEG,PITCH_CONT_MAX_DEG);
  snprintf((char *)g_pitch_cont.summary,sizeof(g_pitch_cont.summary),
           "采集中：经过%d～%d度各记一次up/down，无速度或力矩筛选",
           PITCH_CONT_MIN_DEG,PITCH_CONT_MAX_DEG);
  return 1;
}

void Pitch_Cont_Sample(uint32_t rx_count,uint32_t sample_ms,
                       float actual_deg,float speed_deg_s,float torque_nm) {
  (void)speed_deg_s; /* 速度仍在波形中，不能用它否决采样。 */
  if (!Pitch_Cont_Active() || rx_count==g_pitch_cont.last_rx_count) return;
  g_pitch_cont.feedback_skipped+=rx_count-g_pitch_cont.last_rx_count-1U;
  g_pitch_cont.last_rx_count=rx_count;
  g_pitch_cont.last_sample_ms=sample_ms;
  g_pitch_cont.samples_seen++;
  g_pitch_cont.last_sample_reason=0;
  // 非数字无法插值；不按速度、力矩波动、时间或数量筛选。
  if (!isfinite(actual_deg) || !isfinite(torque_nm)) {
    previous.valid=0;
    g_pitch_cont.last_sample_reason=1;
    return;
  }
  const uint8_t up=g_pitch_scan.phase==PITCH_SCAN_UP_5 ||
      (g_pitch_scan.phase==PITCH_SCAN_POSITION_START && previous.prepare_up);
  const uint8_t down=g_pitch_scan.phase==PITCH_SCAN_DOWN_5;
  if (up || down) {
    for (unsigned i=0;i<PITCH_CONT_NODE_COUNT;i++) {
      Pitch_Cont_Node_t *node=&pitch_cont_table[i];
      uint8_t *valid=up?&node->up_valid:&node->down_valid;
      if (*valid) continue; /* 各方向每个节点记录第一次经过 */
      float torque_at_node;
      if (actual_deg==node->angle_deg) {
        torque_at_node=torque_nm;
      } else if (previous.valid &&
                 ((up && previous.actual_deg<=node->angle_deg && actual_deg>=node->angle_deg) ||
                  (down && previous.actual_deg>=node->angle_deg && actual_deg<=node->angle_deg)) &&
                 actual_deg!=previous.actual_deg) {
        // 两个真实反馈夹住节点：如24.98到25.03，插值得到25度力矩。
        const float ratio=(node->angle_deg-previous.actual_deg)/(actual_deg-previous.actual_deg);
        torque_at_node=previous.torque_nm+ratio*(torque_nm-previous.torque_nm);
      } else continue;
      if (up) { node->up_nm=torque_at_node; g_pitch_cont.up_nodes++; }
      else { node->down_nm=torque_at_node; g_pitch_cont.down_nodes++; }
      *valid=1;
    }
  }
  previous.actual_deg=actual_deg;
  previous.torque_nm=torque_nm;
  previous.valid=1;
}

int8_t Pitch_Cont_Finish_Lap(void) {
  for (unsigned i=0;i<PITCH_CONT_NODE_COUNT;i++) {
    Pitch_Cont_Node_t *p=&pitch_cont_table[i];
    p->valid=p->up_valid && p->down_valid;
    if (!p->valid) continue;
    // up/down已属于同一个实际角度，直接求G和F。
    p->gravity_nm=(p->up_nm+p->down_nm)*0.5f;
    p->friction_nm=(p->up_nm-p->down_nm)*0.5f;
    if (!g_pitch_cont.valid_nodes) g_pitch_cont.valid_min_deg=p->angle_deg;
    g_pitch_cont.valid_max_deg=p->angle_deg;
    g_pitch_cont.valid_nodes++;
  }
  if (g_pitch_cont.valid_nodes!=PITCH_CONT_NODE_COUNT) {
    restore_table();
    export_failed=export_pending=1;
    snprintf((char *)g_pitch_cont.summary,sizeof(g_pitch_cont.summary),
             "Incomplete calibration; previous table and scale restored");
    return -1;
  }
  g_pitch_cont.table_ready=1;
  export_line=0; export_pending=1; export_failed=0;
  (void)Pitch_FF_Set_Scale(1.0f);
  /* 切入时把原P项支撑转给前馈，避免原支撑与新前馈叠加。
   * 实测可能略过下端，保持使用采到的下端down；不伪造缺测值。 */
  float hold_ff_nm=0;
  if (Pitch_Cont_Lookup(PITCH_CONT_MIN_DEG,-1,&hold_ff_nm)) {
    const float old_ff_nm=GIMBAL_PITCH_DIRECTION*g_pitch_test.applied_t_ff_nm;
    if (g_pitch_test.kp>0)
      g_pitch_test.command_relative_rad-=(hold_ff_nm-old_ff_nm)/g_pitch_test.kp;
    g_pitch_test.target_relative_rad=g_pitch_test.command_relative_rad;
    g_pitch_test.applied_t_ff_nm=GIMBAL_PITCH_DIRECTION*hold_ff_nm;
  }
  g_pitch_cont.pass=PITCH_CONT_DONE;
  snprintf((char *)g_pitch_cont.summary,sizeof(g_pitch_cont.summary),
           "采集完成：up %u/%u，down %u/%u，G/F %u/%u；补偿系数%.1f，pscan保持开关",
           g_pitch_cont.up_nodes,PITCH_CONT_NODE_COUNT,g_pitch_cont.down_nodes,PITCH_CONT_NODE_COUNT,
           g_pitch_cont.valid_nodes,PITCH_CONT_NODE_COUNT,Pitch_FF_Get_Scale());
  return 0;
}
void Pitch_Cont_Stop(uint8_t complete) {
  (void)complete;
  if (!Pitch_Cont_Active()) return;
  restore_table();
  snprintf((char *)g_pitch_cont.summary,sizeof(g_pitch_cont.summary),
           "采集已取消；恢复原补偿表和比例");
}
uint8_t Pitch_Cont_Lookup(float actual_deg,int8_t direction,float *torque_nm) {
  if (!g_pitch_cont.table_ready || !isfinite(actual_deg) || !torque_nm ||
      actual_deg<PITCH_CONT_MIN_DEG || actual_deg>PITCH_CONT_MAX_DEG) return 0;
  const float pos=(actual_deg-PITCH_CONT_MIN_DEG)/PITCH_CONT_STEP_DEG;
  const unsigned lower=(unsigned)floorf(pos);
  const Pitch_Cont_Node_t *a=&pitch_cont_table[lower];
  if (!(direction>=0?a->up_valid:a->down_valid)) return 0;
  const float t0=direction>=0?a->up_nm:a->down_nm;
  if (pos==(float)lower) { *torque_nm=t0; return 1; }
  if (lower+1U>=PITCH_CONT_NODE_COUNT) return 0;
  const Pitch_Cont_Node_t *b=&pitch_cont_table[lower+1U];
  if (!(direction>=0?b->up_valid:b->down_valid)) return 0;
  *torque_nm=t0+((direction>=0?b->up_nm:b->down_nm)-t0)*(pos-lower);
  return 1;
}
