#include "yaw_diagnostics.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#if defined(STM32F427xx)
#include "stm32f4xx.h"
#endif

volatile YawDiagStatus g_yaw_diag;
volatile YawDiagFrame g_yaw_diag_latest;
static YawDiagFrame samples[YAW_DIAG_CAPACITY], pending;
static uint32_t start_ms, pending_flags, prepare_cycles;
static unsigned export_phase, export_index;
static uint8_t prepared, packed;
static uint32_t cycles(void) {
#if defined(STM32F427xx)
  return DWT->CYCCNT;
#else
  return 0;
#endif
}
void YawDiag_Init(void) {
  memset((void *)&g_yaw_diag,0,sizeof(g_yaw_diag));
  memset((void *)&g_yaw_diag_latest,0,sizeof(g_yaw_diag_latest));
  g_yaw_diag.capacity=YAW_DIAG_CAPACITY;
  prepared=packed=0; pending_flags=prepare_cycles=0;
  export_phase=export_index=0;
}
uint8_t YawDiag_Arm(void) {
  if (g_yaw_diag.state==YD_RECORDING || g_yaw_diag.state==YD_DUMPING) return 0;
  g_yaw_diag.count=g_yaw_diag.stop_reason=g_yaw_diag.tx_failures=0;
  g_yaw_diag.max_record_cycles=0;
  prepared=packed=0; pending_flags=0;
  g_yaw_diag.state=YD_ARMED;
  return 1;
}
void YawDiag_TargetAccepted(uint32_t now_ms) {
  if (g_yaw_diag.state==YD_ARMED) {
    start_ms=now_ms;
    g_yaw_diag.state=YD_RECORDING;
  }
}
void YawDiag_MarkReset(void) {
  if (g_yaw_diag.state==YD_ARMED || g_yaw_diag.state==YD_RECORDING)
    pending_flags|=YD_LOAD_RESET;
}
uint8_t YawDiag_Recording(void) { return g_yaw_diag.state==YD_RECORDING; }
void YawDiag_Prepare(const YawDiagFrame *frame) {
  if (!YawDiag_Recording() || frame==NULL) return;
  const uint32_t begin=cycles();
  pending=*frame;
  pending.time_ms-=start_ms;
  pending.flags|=pending_flags; pending_flags=0;
  pending.tx_result=255;
  prepared=1; packed=0;
  prepare_cycles=cycles()-begin;
}
static float unpack(unsigned value,float lo,float span,unsigned max) {
  return lo+(float)value*span/(float)max;
}
void YawDiag_MitPacked(uint16_t motor_id,const uint8_t b[8]) {
  if (motor_id!=2U || !prepared || !YawDiag_Recording()) return;
  const uint32_t begin=cycles();
  pending.wire_p=unpack((b[0]<<8)|b[1],-12.5f,25,65535);
  pending.wire_v=unpack((b[2]<<4)|(b[3]>>4),-30,60,4095);
  pending.wire_kp=unpack(((b[3]&15)<<8)|b[4],0,500,4095);
  pending.wire_kd=unpack((b[5]<<4)|(b[6]>>4),0,5,4095);
  pending.wire_ff=unpack(((b[6]&15)<<8)|b[7],-10,20,4095);
  if (pending.p_des < -12.5f || pending.p_des > 12.5f ||
      pending.v_des < -30 || pending.v_des > 30 ||
      pending.kp < 0 || pending.kp > 500 || pending.kd < 0 || pending.kd > 5 ||
      pending.t_ff < -10 || pending.t_ff > 10) pending.flags|=YD_PACKET_CLIP;
  pending.p_est=pending.wire_kp*(pending.wire_p-pending.raw_position_rad);
  pending.d_est=pending.wire_kd*(pending.wire_v-pending.motor_rate_rad_s);
  pending.total_est=pending.p_est+pending.d_est+pending.wire_ff;
  packed=1;
  prepare_cycles+=cycles()-begin;
}
void YawDiag_MitResult(uint16_t motor_id,uint8_t result) {
  if (motor_id!=2U || !prepared || !packed || !YawDiag_Recording()) return;
  const uint32_t begin=cycles();
  pending.tx_result=result;
  if (result!=0) ++g_yaw_diag.tx_failures;
  samples[g_yaw_diag.count]=pending;
  g_yaw_diag_latest=pending;
  ++g_yaw_diag.count; prepared=packed=0;
  const uint32_t elapsed=prepare_cycles+cycles()-begin;
  if (elapsed>g_yaw_diag.max_record_cycles) g_yaw_diag.max_record_cycles=elapsed;
  if (g_yaw_diag.count==YAW_DIAG_CAPACITY) YawDiag_Stop(YD_STOP_FULL);
}
void YawDiag_Stop(uint32_t reason) {
  if (g_yaw_diag.state!=YD_ARMED && g_yaw_diag.state!=YD_RECORDING) return;
  prepared=packed=0;
  g_yaw_diag.stop_reason=reason;
  g_yaw_diag.state=YD_FROZEN;
}
uint8_t YawDiag_Dump(void) {
  if (g_yaw_diag.state!=YD_FROZEN || !g_yaw_diag.count) return 0;
  export_phase=export_index=0; g_yaw_diag.state=YD_DUMPING; return 1;
}
uint8_t YawDiag_Exporting(void) { return g_yaw_diag.state==YD_DUMPING; }
uint8_t YawDiag_ExportLine(char *buf,unsigned capacity) {
  if (!YawDiag_Exporting() || !buf || capacity<320) return 0;
  int n;
  const double deg=57.29577951308232;
  if (export_phase==0) n=snprintf(buf,capacity,
    "#YD_BEGIN,v=1,count=%lu,stop=%lu,tx_fail=%lu,max_record_cycles=%lu\r\n",
    (unsigned long)g_yaw_diag.count,(unsigned long)g_yaw_diag.stop_reason,
    (unsigned long)g_yaw_diag.tx_failures,(unsigned long)g_yaw_diag.max_record_cycles);
  else if (export_phase==1) n=snprintf(buf,capacity,
    "#YD_A_FIELDS,index,time_ms,dt_ms,flags,tx_result,target_deg,encoder_deg,rate_deg_s,raw_position_rad,command_pre_deg,command_deg,p_des_rad,v_des_rad_s,kp,kd,t_ff_nm\r\n");
  else if (export_phase==2) n=snprintf(buf,capacity,
    "#YD_B_FIELDS,index,drive,arrived,brake_deg,friction_goal_nm,friction_before_nm,friction_actual_nm,feedback_torque_nm,feedback_age_ms,rx_count,mode,axis_state,ff_on,integral_on,boost_on\r\n");
  else if (export_phase==3) n=snprintf(buf,capacity,
    "#YD_C_FIELDS,index,wire_p_rad,wire_v_rad_s,wire_kp,wire_kd,wire_ff_nm,p_est_nm,d_est_nm,total_est_nm\r\n");
  else if (export_phase==7) n=snprintf(buf,capacity,"#YD_END\r\n");
  else {
    const YawDiagFrame *s=&samples[export_index];
    if (export_phase==4) n=snprintf(buf,capacity,
      "#YD_A,%u,%lu,%lu,%lu,%u,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\r\n",
      export_index,(unsigned long)s->time_ms,(unsigned long)s->dt_ms,
      (unsigned long)s->flags,s->tx_result,s->target_rad*deg,s->encoder_rad*deg,
      s->rate_rad_s*deg,(double)s->raw_position_rad,s->command_pre_rad*deg,
      s->command_rad*deg,(double)s->p_des,(double)s->v_des,(double)s->kp,
      (double)s->kd,(double)s->t_ff);
    else if (export_phase==5) n=snprintf(buf,capacity,
      "#YD_B,%u,%.9g,%u,%.9g,%.9g,%.9g,%.9g,%.9g,%lu,%lu,%u,%u,%u,%u,%u\r\n",
      export_index,(double)s->drive,s->arrived,s->brake_rad*deg,
      (double)s->friction_goal,(double)s->friction_before,(double)s->friction_actual,
      (double)s->feedback_torque,(unsigned long)s->feedback_age_ms,
      (unsigned long)s->rx_count,s->mode,s->axis_state,s->ff_on,s->integral_on,s->boost_on);
    else n=snprintf(buf,capacity,
      "#YD_C,%u,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\r\n",
      export_index,(double)s->wire_p,(double)s->wire_v,(double)s->wire_kp,
      (double)s->wire_kd,(double)s->wire_ff,(double)s->p_est,(double)s->d_est,
      (double)s->total_est);
  }
  return n>0 && (unsigned)n<capacity;
}
void YawDiag_ExportAdvance(void) {
  if (!YawDiag_Exporting()) return;
  if (export_phase==7) { g_yaw_diag.state=YD_FROZEN; return; }
  if (export_phase==6) {
    ++export_index;
    export_phase=export_index==g_yaw_diag.count?7:4;
  } else ++export_phase;
}
