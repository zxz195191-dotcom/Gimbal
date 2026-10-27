#define main diagnostics_regression_main
#include "test_yaw_diagnostics.c"
#undef main
#include "yaw_friction_goal.h"

#define RAD .017453292519943295f
static void selector_tests(void) {
  YawFrictionGoal s;
  YawFrictionGoal_Reset(&s);
  assert(YawFrictionGoal_Update(&s,.12f*RAD,0,0,0,.456f)==.456f);
  assert(YawFrictionGoal_Update(&s,.5f*RAD,20*RAD,.45f*RAD,0,.456f)==0 && s.braking);
  assert(YawFrictionGoal_Update(&s,.5f*RAD,20*RAD,.38f*RAD,0,.456f)==0);
  assert(YawFrictionGoal_Update(&s,.5f*RAD,10*RAD,.30f*RAD,0,.456f)==.456f && !s.braking);
  assert(YawFrictionGoal_Update(&s,.08f*RAD,0,0,1,.456f)==0 && !s.correcting);
  for(unsigned n=0;n<20;n++) {
    assert(YawFrictionGoal_Update(&s,.11f*RAD,0,0,0,.456f)==0);
    assert(YawFrictionGoal_Update(&s,.14f*RAD,0,0,0,.456f)==0);
  }
  assert(YawFrictionGoal_Update(&s,.16f*RAD,0,0,0,.456f)==.456f);
  assert(YawFrictionGoal_Update(&s,-.5f*RAD,5*RAD,0,0,.456f)==-.456f);
  assert(YawFrictionGoal_Update(&s,NAN,0,0,0,.456f)==0 && !s.correcting);
  puts("PASS selector: stationary residual, brake release hysteresis, genuine arrival, position-bin hysteresis, reversal, invalid inputs");
}
static void repeat(float span,float speed) {
  init();
  for(unsigned lap=0;lap<3;lap++) for(unsigned side=0;side<2;side++) {
    const float target=side?0:span;
    assert(Gimbal_Yaw_Set_Target_Speed_Deg(target,speed));
    float lo=1e9f,hi=-1e9f,overshoot=0;
    for(unsigned ms=0;ms<3000;ms++) {
      plant();
      const float actual=g_yaw_test.measured_relative_rad/RAD;
      if (ms>=2500) { lo=fminf(lo,actual); hi=fmaxf(hi,actual); }
      overshoot=fmaxf(overshoot,side?target-actual:actual-target);
      assert(g_yaw_test.state==YAW_TEST_ACTIVE);
      assert(fabsf(g_load_comp[Yaw].output_nm)<=1.00001f);
    }
    const float error=(g_yaw_test.target_relative_rad-g_yaw_test.measured_relative_rad)/RAD;
    printf("span=%.1f speed=%.0f lap=%u target=%.1f error=%.5f stable_p2p=%.5f overshoot=%.5f\n",
      (double)span,(double)speed,lap+1,(double)target,(double)error,(double)(hi-lo),(double)overshoot);
    assert(fabsf(error)<=.5f && hi-lo<=.1f && overshoot<=(span<=.5f?.25f:.5f));
    assert(!g_load_comp[Yaw].integral_enabled);
  }
}
int main(void) {
  selector_tests();
  repeat(.5f,5); repeat(.5f,20); repeat(10,20);
  init(); assert(Gimbal_Yaw_Set_Target_Speed_Deg(.5f,20)); run(80);
  assert(g_load_comp[Yaw].friction_nm>0);
  Gimbal_Yaw_Test_Stop(); assert(!yaw_enabled && g_load_comp[Yaw].friction_nm==0);
  init(); assert(Gimbal_Yaw_Set_Target_Speed_Deg(.5f,20)); tx_failed=1;run(6);
  assert(g_yaw_test.state==YAW_TEST_FAULT && !yaw_enabled && g_load_comp[Yaw].friction_nm==0);
  init(); tick+=DM_FEEDBACK_TIMEOUT_MS+1; Gimbal_Yaw_Test_Update(tick,GPIO_PIN_RESET);
  assert(g_yaw_test.state==YAW_TEST_FAULT && !yaw_enabled && g_load_comp[Yaw].friction_nm==0);
  init(); Gimbal_Yaw_Probe_Request(12345);
  char line[320],retry[320];
  assert(Gimbal_Load_Report_Line(line,sizeof(line)) && strstr(line,"#YP,id=12345,"));
  assert(strstr(line,"state=4,") && strstr(line,"i=0,boost=0,"));
  assert(Gimbal_Load_Report_Line(retry,sizeof(retry)) && !strcmp(line,retry));
  assert(strlen(line)<sizeof(line)-1); Gimbal_Load_Report_Advance();
  puts("PASS preserved stop/feedback/TX faults and bounded, retry-safe read-only probe");
  return 0;
}
