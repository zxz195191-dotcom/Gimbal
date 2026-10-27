/* Existing direct-C HAL mock; exercise the production controller and sampler. */
#include "scan_check.c"

static void run_lap(float ff_scale) {
  int last=PITCH_SCAN_POSITION_START, transitions=0;
  const int phases[]={PITCH_SCAN_UP_5,PITCH_SCAN_DOWN_5};
  for(int i=0;i<35000 && g_pitch_scan.active;i++) {
    step(1,1);
    if(g_pitch_scan.active && (int)g_pitch_scan.phase!=last) {
      assert(transitions<2 && (int)g_pitch_scan.phase==phases[transitions++]);
      last=g_pitch_scan.phase;
    }
    if(g_pitch_scan.active) assert(Pitch_FF_Get_Scale()==ff_scale);
  }
  assert(transitions==2 && g_pitch_scan.result==PITCH_SCAN_RESULT_COMPLETE);
  assert(!g_pitch_scan.active && enabled && g_pitch_scan.holding);
}
static void synthetic_lap(void) {
  uint32_t sequence=g_pitch_cont.last_rx_count;
  for(int direction=0;direction<2;direction++) {
    g_pitch_scan.phase=direction?PITCH_SCAN_DOWN_5:PITCH_SCAN_UP_5;
    for(int i=0;i<=(PITCH_CONT_MAX_DEG-PITCH_CONT_MIN_DEG)*20+4;i++) {
      float actual=direction?(PITCH_CONT_MAX_DEG+0.1f)-i*0.05f:(PITCH_CONT_MIN_DEG-0.1f)+i*0.05f;
      // Speeds and torques that the previous implementation rejected.
      float speed=direction?-7.98f:20;
      float measured_torque=4.0f+0.02f*actual+(direction?-0.15f:0.15f);
      g_pitch_test.target_relative_rad=g_pitch_test.command_relative_rad=45*0.0174532925f;
      tick+=10;
      Pitch_Cont_Sample(++sequence,tick,actual,speed,measured_torque);
      uint32_t seen=g_pitch_cont.samples_seen;
      Pitch_Cont_Sample(sequence,tick,actual,speed,999);
      assert(g_pitch_cont.samples_seen==seen);
    }
  }
}
int main(void) {
  setbuf(stdout,NULL);
  reset();
  for(int i=0;i<100;i++) step(1,0);
  assert(!enabled && !g_pitch_scan.active);
  ready(); Gimbal_Enable_All(); assert(!g_pitch_scan.active);
  assert(Gimbal_Pitch_Scan_Start()); run_lap(0);
  float held=g_pitch_test.command_relative_rad;
  for(int i=0;i<2000;i++) step(1,1);
  assert(g_pitch_test.command_relative_rad==held && fabsf(velocity)<0.002f);
  assert(Gimbal_Pitch_Scan_Start()); Gimbal_Pitch_Scan_Cancel();
  Gimbal_Pitch_Stop(); Gimbal_Pitch_Enable();
  for(int i=0;i<100;i++) step(1,0);
  assert(!g_pitch_scan.active);

  ready(); assert(Gimbal_Pitch_Scan_Start());
  for(int i=0;i<2500;i++) step(1,0);
  assert(g_pitch_scan.active && g_pitch_scan.event_count==0);
  assert(g_pitch_test.command_relative_rad*57.29578f>12);
  assert(torque>3); // No lead, stall or scan torque abort.
  Gimbal_Pitch_Stop(); assert(!g_pitch_scan.active && !enabled);
  ready(); assert(Gimbal_Pitch_Scan_Start());
  for(int i=0;i<60;i++) step(0,0);
  assert(!g_pitch_scan.active && !enabled && !g_pitch_scan.feedback_valid);
  ready(); assert(Gimbal_Pitch_Scan_Start());
  assert(Gimbal_Pitch_Set_Target_Speed_Deg(12,5));
  assert(!g_pitch_scan.active && fabsf(g_pitch_test.target_relative_rad-12*0.0174532925f)<1e-6f);

  ready(); assert(Pitch_Cont_Start()); synthetic_lap();
  assert(strstr((const char *)g_pitch_scan.start_summary,"pcal "));
  assert(strstr((const char *)g_pitch_scan.start_summary,"FF=0"));
  assert(Pitch_Cont_Finish_Lap()==0);
  assert(g_pitch_cont.pass==PITCH_CONT_DONE && Pitch_FF_Get_Scale()==1);
  assert(g_pitch_cont.up_nodes==PITCH_CONT_NODE_COUNT && g_pitch_cont.down_nodes==PITCH_CONT_NODE_COUNT && g_pitch_cont.valid_nodes==PITCH_CONT_NODE_COUNT);
  for(unsigned i=0;i<PITCH_CONT_NODE_COUNT;i++) {
    const Pitch_Cont_Node_t *n=&pitch_cont_table[i];
    assert(n->valid && n->up_valid && n->down_valid);
    assert(fabsf(n->up_nm-(4+0.02f*n->angle_deg+0.15f))<0.001f);
    assert(fabsf(n->down_nm-(4+0.02f*n->angle_deg-0.15f))<0.001f);
    assert(fabsf(n->gravity_nm-(4+0.02f*n->angle_deg))<0.001f);
    assert(fabsf(n->friction_nm-0.15f)<0.001f);
  }
  float value=99;
  for(int deg=0;deg<=5;deg++) {
    assert(pitch_cont_table[deg].up_valid && pitch_cont_table[deg].down_valid);
    assert(Pitch_Cont_Lookup((float)deg,1,&value));
    assert(fabsf(value-(4+0.02f*deg+0.15f))<0.001f);
  }
  assert(Pitch_Cont_Lookup(0.5f,-1,&value) && fabsf(value-3.86f)<0.001f);
  assert(!Pitch_Cont_Lookup(-0.01f,1,&value));
  assert(Pitch_Cont_Lookup(30.4f,1,&value) && fabsf(value-4.758f)<0.001f);
  assert(fabsf(Pitch_FF_Update(30.4f,1,5)-4.758f)<0.001f); // No added FF cap/slew.
  Gimbal_Pitch_Scan_Cancel(); assert(Pitch_FF_Get_Scale()==1);
  dm_motor[Pitch].last_rx_ms=tick;
  assert(Gimbal_Pitch_Scan_Start()); assert(Pitch_FF_Get_Scale()==1);
  Gimbal_Pitch_Scan_Cancel(); assert(Pitch_FF_Get_Scale()==1);

  ready(); assert(Pitch_Cont_Start());
  uint32_t seq=g_pitch_cont.last_rx_count;
  g_pitch_scan.phase=PITCH_SCAN_UP_5;
  Pitch_Cont_Sample(++seq,++tick,24.9f,0,2);
  seq+=3;
  Pitch_Cont_Sample(seq,++tick,26.1f,99,3.2f);
  assert(g_pitch_cont.feedback_skipped==2 && g_pitch_cont.up_nodes==2);
  assert(fabsf(pitch_cont_table[25-PITCH_CONT_MIN_DEG].up_nm-2.1f)<0.0001f);
  assert(isnan(pitch_cont_table[24-PITCH_CONT_MIN_DEG].up_nm) && !pitch_cont_table[24-PITCH_CONT_MIN_DEG].up_valid);
  Pitch_Cont_Finish_Lap();
  assert(g_pitch_cont.valid_nodes==0 && isnan(pitch_cont_table[25-PITCH_CONT_MIN_DEG].gravity_nm));
  assert(Pitch_Cont_Lookup(25.5f,1,&value));
  assert(!Pitch_Cont_Lookup(25.5f,-1,&value));
  assert(!Pitch_Cont_Lookup(24.5f,1,&value));
  assert(!Pitch_Cont_Lookup(4,1,&value));

  ready(); assert(Pitch_Cont_Start()); Gimbal_Pitch_Stop();
  assert(!Pitch_Cont_Active() && !g_pitch_scan.active && !enabled);
  Gimbal_Pitch_Enable(); for(int i=0;i<100;i++) step(1,0);
  assert(!Pitch_Cont_Active() && !g_pitch_scan.active);
  ready(); assert(Pitch_Cont_Start());
  for(int i=0;i<60;i++) step(0,0);
  assert(!Pitch_Cont_Active() && !g_pitch_scan.active);

  ready(); gravity_slope=1.4f; assert(Pitch_Cont_Start());
  run_lap(0);
  printf("model pcal: up=%u down=%u paired=%u scale=%.1f\n",
         g_pitch_cont.up_nodes,g_pitch_cont.down_nodes,g_pitch_cont.valid_nodes,Pitch_FF_Get_Scale());
  assert(g_pitch_cont.pass==PITCH_CONT_DONE && g_pitch_cont.valid_nodes==PITCH_CONT_NODE_COUNT);
  assert(Pitch_FF_Get_Scale()==1);
  held=g_pitch_test.command_relative_rad;
  for(int i=0;i<2000;i++) step(1,1);
  assert(g_pitch_test.command_relative_rad==held && enabled && fabsf(velocity)<0.002f);
  for(int repeat=0;repeat<2;repeat++) {
    assert(Gimbal_Pitch_Scan_Start()); run_lap(1);
    assert(Pitch_FF_Get_Scale()==1);
    for(int i=0;i<1000;i++) step(1,1);
  }
  assert(Gimbal_Pitch_Set_Gravity_FF_Scale(0));
  assert(Gimbal_Pitch_Scan_Start()); run_lap(0);
  assert(PITCH_CONT_NODE_COUNT==42 && pitch_cont_table[41].angle_deg==41);
  assert(!Pitch_Cont_Lookup(41.01f,1,&value));
  assert(!Gimbal_Pitch_Repeat_Start(25,45,5));
  assert(Gimbal_Pitch_Set_Target_Deg(80));
  assert(fabsf(g_pitch_test.target_relative_rad-41*0.0174532925f)<1e-6f);
  assert(Gimbal_Pitch_Move_Relative_Deg(100));
  assert(fabsf(g_pitch_test.target_relative_rad-41*0.0174532925f)<1e-6f);
  // Same full table, unchanged gains: repeat several actual-angle reversals.
  assert(Pitch_FF_Set_Scale(1));
  Pitch_Cont_Node_t table_before[PITCH_CONT_NODE_COUNT];
  memcpy(table_before,pitch_cont_table,sizeof(table_before));
  float saved_kp=g_pitch_test.kp, saved_kd=g_pitch_test.kd;
  assert(Gimbal_Pitch_Repeat_Start(25,41,5));
  int reversals=0, direction=g_gimbal_repeat[Pitch].direction;
  for(int i=0;i<35000;i++) {
    float previous=g_pitch_test.command_relative_rad;
    step(1,1);
    assert(g_gimbal_repeat[Pitch].active && !g_pitch_scan.active);
    assert(Pitch_FF_Get_Scale()==1 && g_pitch_test.kp==saved_kp && g_pitch_test.kd==saved_kd);
    assert(fabsf(g_pitch_test.command_relative_rad-previous)<0.00044f);
    if(g_gimbal_repeat[Pitch].direction!=direction) {
      direction=g_gimbal_repeat[Pitch].direction;
      assert(direction<0 ? position*57.29578f>=41 : position*57.29578f<=25);
      reversals++;
    }
  }
  assert(reversals>=5 && memcmp(table_before,pitch_cont_table,sizeof(table_before))==0);
  Gimbal_Pitch_Scan_Cancel(); held=g_pitch_test.command_relative_rad;
  for(int i=0;i<1000;i++) step(1,1);
  assert(!g_gimbal_repeat[Pitch].active && g_pitch_test.command_relative_rad==held && enabled);
  assert(Pitch_FF_Get_Scale()==1);
  assert(Gimbal_Pitch_Repeat_Start(39.5f,40.5f,2.5f));
  assert(Gimbal_Pitch_Set_Target_Speed_Deg(30,5));
  assert(!g_gimbal_repeat[Pitch].active);
  assert(Gimbal_Pitch_Repeat_Start(25,41,5));
  assert(Gimbal_Pitch_Scan_Start() && !g_gimbal_repeat[Pitch].active);
  Gimbal_Pitch_Scan_Cancel();
  assert(Gimbal_Pitch_Repeat_Start(25,41,5));
  assert(Pitch_Cont_Start() && !g_gimbal_repeat[Pitch].active);
  Gimbal_Pitch_Scan_Cancel();
  assert(Gimbal_Pitch_Repeat_Start(25,41,5));
  Gimbal_Pitch_Stop(); Gimbal_Pitch_Enable();
  for(int i=0;i<100;i++) step(1,0);
  assert(!g_gimbal_repeat[Pitch].active);
  assert(Gimbal_Pitch_Repeat_Start(25,41,5));
  for(int i=0;i<60;i++) step(0,0);
  assert(!g_gimbal_repeat[Pitch].active && !enabled);

  ready(); assert(!Gimbal_Yaw_Repeat_Start(-10,10,5));
  Gimbal_Yaw_Enable(); for(int i=0;i<100;i++) step(1,0);
  assert(g_yaw_test.state==YAW_TEST_ACTIVE && !g_gimbal_repeat[Yaw].active);
  assert(!Gimbal_Yaw_Repeat_Start(10,-10,5));
  assert(!Gimbal_Yaw_Repeat_Start(-10,10,0));
  assert(!Gimbal_Yaw_Repeat_Start(NAN,10,5));
  assert(Gimbal_Yaw_Repeat_Start(-10,10,5) && g_gimbal_log_axis==Yaw);
  reversals=0; direction=g_gimbal_repeat[Yaw].direction;
  for(int i=0;i<25000;i++) {
    float previous=g_yaw_test.command_relative_rad;
    step(1,1);
    assert(g_gimbal_repeat[Yaw].active);
    assert(!g_gimbal_repeat[Pitch].active);
    assert(fabsf(g_yaw_test.command_relative_rad-previous)<0.00044f);
    if(g_gimbal_repeat[Yaw].direction!=direction) {
      direction=g_gimbal_repeat[Yaw].direction;
      assert(direction>0 ? yaw_position*57.29578f<=-10 : yaw_position*57.29578f>=10);
      reversals++;
    }
  }
  assert(reversals>=5);
  Gimbal_Yaw_Repeat_Cancel(); held=g_yaw_test.command_relative_rad;
  for(int i=0;i<1000;i++) step(1,1);
  assert(!g_gimbal_repeat[Yaw].active && g_yaw_test.command_relative_rad==held && yaw_enabled);
  assert(Gimbal_Yaw_Repeat_Start(-2.5f,2.5f,3));
  assert(Gimbal_Yaw_Move_Relative_Deg(2)); assert(!g_gimbal_repeat[Yaw].active);
  assert(Gimbal_Yaw_Repeat_Start(-10,10,5));
  Gimbal_Yaw_Test_Stop(); Gimbal_Yaw_Enable();
  for(int i=0;i<100;i++) step(1,0);
  assert(!g_gimbal_repeat[Yaw].active);
  assert(Gimbal_Yaw_Repeat_Start(-10,10,5)); yaw_feedback=0;
  for(int i=0;i<60;i++) step(1,0);
  assert(!g_gimbal_repeat[Yaw].active && !yaw_enabled);
  yaw_feedback=1; Gimbal_Yaw_Enable(); for(int i=0;i<100;i++) step(1,0);
  assert(Gimbal_Yaw_Repeat_Start(-10,10,5));
  assert(Gimbal_Pitch_Repeat_Start(25,35,5)); Gimbal_Stop_All();
  assert(!g_gimbal_repeat[Yaw].active && !g_gimbal_repeat[Pitch].active);
  assert(!yaw_enabled && !enabled);
  puts("PASS: pitch/yaw repeated measured endpoints, continuous command, unchanged table/gains/FF, cancel hold, manual handoff, scan/cal handoff, disabled/fault no restart");
  puts("PASS: single roundtrip, no implicit start, scan leaves FF unchanged, actual-angle nodes, dedup, no quality filters, missing values, full FF, cancellation, existing feedback fault, hold");
  return 0;
}
