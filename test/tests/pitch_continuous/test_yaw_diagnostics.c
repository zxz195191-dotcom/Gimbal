#define dm4310_send_mit unused_mock_mit
#include "scan_check.c"
#undef dm4310_send_mit
#include "yaw_diagnostics.h"
#include "load_compensation.h"
extern LoadCompState g_load_comp[DM_NUM];
static uint32_t wire_hash;
static uint8_t tx_failed;
static float decoded_p,decoded_v,decoded_kp,decoded_kd,decoded_ff;

HAL_StatusTypeDef Can_Send_Std(CAN_HandleTypeDef *h,uint16_t id,uint8_t *d,uint8_t len) {
  (void)h; assert(len==8);
  float p=uint_to_float((d[0]<<8)|d[1],-12.5f,12.5f,16);
  float v=uint_to_float((d[2]<<4)|(d[3]>>4),-30,30,12);
  float k=uint_to_float(((d[3]&15)<<8)|d[4],0,500,12);
  float damping=uint_to_float((d[5]<<4)|(d[6]>>4),0,5,12);
  float ff=uint_to_float(((d[6]&15)<<8)|d[7],-10,10,12);
  if(id==DM_YAW_CAN_ID) {
    for(unsigned j=0;j<8;j++) wire_hash=(wire_hash^d[j])*16777619U;
    yaw_p_des=decoded_p=p; yaw_v_des=decoded_v=v;
    yaw_kp=decoded_kp=k; yaw_kd=decoded_kd=damping; yaw_ff=decoded_ff=ff;
    if (tx_failed) return 1;
  } else { p_des=p;v_des=v;kp=k;kd=damping;applied_ff=ff; }
  return HAL_OK;
}
static void plant(void) {
  ++tick;
  float t=yaw_enabled?yaw_kp*(yaw_p_des-yaw_position)+yaw_kd*(yaw_v_des-yaw_velocity)+yaw_ff:0;
  t=fmaxf(-10,fminf(10,t));
  const float fc=.4f;
  float resistance=fabsf(yaw_velocity)<.0001f?fmaxf(-fc,fminf(fc,t)):(yaw_velocity>0?fc:-fc);
  float v=yaw_velocity+(t-resistance-.1f*yaw_velocity)/.02f*.001f;
  if(v*yaw_velocity<0 && fabsf(t)<fc) v=0;
  if(!yaw_enabled) v=0;
  yaw_velocity=v; yaw_position+=v*.001f;
  dm_motor[Pitch].last_rx_ms=tick; ++dm_motor[Pitch].rx_count;
#ifndef YAW_TEST_FRAME_PERIOD
#define YAW_TEST_FRAME_PERIOD 2U
#endif
  if(tick%YAW_TEST_FRAME_PERIOD==0) {
#if defined(YAW_TEST_QUANTIZED_FEEDBACK)
    dm_motor[Yaw].p_int=uint_to_float(float_to_uint(yaw_position,-12.5f,12.5f,16),-12.5f,12.5f,16);
    dm_motor[Yaw].v_int=uint_to_float(float_to_uint(yaw_velocity,-30,30,12),-30,30,12);
    dm_motor[Yaw].t_int=uint_to_float(float_to_uint(t,-10,10,12),-10,10,12);
#else
    dm_motor[Yaw].p_int=yaw_position; dm_motor[Yaw].v_int=yaw_velocity;
    dm_motor[Yaw].t_int=t;
#endif
    dm_motor[Yaw].last_rx_ms=tick; ++dm_motor[Yaw].rx_count;
  }
  Gimbal_Yaw_Test_Update(tick,GPIO_PIN_RESET);
}
static void run(unsigned ms) { while(ms--) plant(); }
static void init(void) {
  tx_failed=0; ready(); Gimbal_Yaw_Enable(); run(100);
  assert(Gimbal_Yaw_Set_Mode(YAW_MODE_ENCODER));
  wire_hash=2166136261U;
}
static void export_capture(const char *directory,unsigned speed) {
  assert(!Gimbal_Yaw_Diag_Command(2)); /* Formatting is excluded from active Yaw. */
  Gimbal_Yaw_Test_Stop();
  assert(Gimbal_Yaw_Diag_Command(2));
  char path[1024],line[320],again[320];
  snprintf(path,sizeof(path),"%s/yaw_diag_%udps.log",directory,speed);
  FILE *out=fopen(path,"w"); assert(out);
  unsigned rows=0,groups[3]={0};
  while(YawDiag_Exporting()) {
    assert(Gimbal_Load_Report_Line(line,sizeof(line)));
    assert(Gimbal_Load_Report_Line(again,sizeof(again)) && strcmp(line,again)==0);
    assert(strlen(line)<sizeof(line)-1);
    for(unsigned j=0;j<3;j++) if(line[0]=='#' && line[4]==(char)('A'+j) && line[5]==',') ++groups[j];
    fputs(line,out); ++rows;
    Gimbal_Load_Report_Advance();
  }
  fclose(out);
  assert(rows==5+3*YAW_DIAG_CAPACITY);
  for(unsigned j=0;j<3;j++) assert(groups[j]==YAW_DIAG_CAPACITY);
}
int main(int argc,char **argv) {
  assert(argc==2);
  float speeds[]={1,5,20};
  for(unsigned i=0;i<3;i++) {
    init(); assert(Gimbal_Yaw_Move_Relative_Speed_Deg(.5f,speeds[i])); run(5200);
    const uint32_t baseline=wire_hash;
    const float baseline_position=yaw_position;
    init(); assert(Gimbal_Yaw_Diag_Command(0));
    assert(g_yaw_diag.state==YD_ARMED && !g_yaw_diag.count);
    assert(Gimbal_Yaw_Move_Relative_Speed_Deg(.5f,speeds[i])); run(5200);
    assert(wire_hash==baseline && yaw_position==baseline_position);
    assert(g_yaw_diag.state==YD_FROZEN && g_yaw_diag.count==YAW_DIAG_CAPACITY);
    assert(g_yaw_diag.stop_reason==YD_STOP_FULL && !g_yaw_diag.tx_failures);
    assert(g_yaw_diag_latest.flags&YD_DYNAMIC);
    assert(g_yaw_diag_latest.wire_kp==decoded_kp && g_yaw_diag_latest.wire_kd==decoded_kd);
    assert(g_yaw_diag_latest.wire_p==decoded_p && g_yaw_diag_latest.wire_v==decoded_v);
    assert(g_yaw_diag_latest.wire_ff==decoded_ff);
    assert(g_yaw_diag_latest.ff_on && !g_yaw_diag_latest.integral_on && !g_yaw_diag_latest.boost_on);
    export_capture(argv[1],(unsigned)speeds[i]);
    printf("PASS %.0f dps: identical CAN bytes/position, 1024 frames, packed parameters and retry-safe export\n",(double)speeds[i]);
  }
  init(); assert(Gimbal_Yaw_Diag_Command(0));
  assert(Gimbal_Yaw_Move_Relative_Speed_Deg(.5f,20)); tx_failed=1; run(6);
  assert(g_yaw_diag.state==YD_FROZEN && g_yaw_diag.stop_reason==YD_STOP_FAULT);
  assert(g_yaw_diag.count==1 && g_yaw_diag.tx_failures==1 && g_yaw_diag_latest.tx_result==1);
  assert(!yaw_enabled && g_yaw_test.state==YAW_TEST_FAULT);
  init(); assert(Gimbal_Yaw_Diag_Command(0));
  assert(Gimbal_Yaw_Move_Relative_Speed_Deg(.5f,5)); run(50);
  assert(Gimbal_Yaw_Diag_Command(1)); const unsigned count=g_yaw_diag.count; run(50);
  assert(g_yaw_diag.count==count && yaw_enabled && g_yaw_test.state==YAW_TEST_ACTIVE);
  init(); assert(Gimbal_Yaw_Position_Boost_Enable(1)); assert(!Gimbal_Yaw_Diag_Command(0));
  assert(Gimbal_Yaw_Position_Boost_Enable(0));
  assert(Gimbal_Load_Integral_Enable(Yaw,1)); assert(!Gimbal_Yaw_Diag_Command(0));
  puts("PASS fault freeze, manual freeze without motion changes, and invalid configuration rejection");
  printf("Frame bytes=%u; buffer bytes=%u\n",(unsigned)sizeof(YawDiagFrame),(unsigned)(sizeof(YawDiagFrame)*YAW_DIAG_CAPACITY));
  return 0;
}
