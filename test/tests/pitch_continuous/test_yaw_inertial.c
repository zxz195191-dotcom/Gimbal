#include "scan_check.c"
#define RAD 0.017453292519943295f
static void yaw_ready(void) {
  ready();
  Gimbal_Yaw_Enable();
  imu_valid=imu_auto=1;
  for (int i=0;i<100;i++) step(1,0);
  assert(g_yaw_test.state==YAW_TEST_ACTIVE && g_yaw_inertial.valid);
}
int main(void) {
  reset();
  imu_valid=1;
  if (YAW_IMU_DIRECTION<0) {
    mock_pose(0,41*RAD,1,0,0,0.2f); Yaw_Inertial_Update(tick);
    Yaw_Inertial_Zero(0);
    mock_pose(0,41*RAD,1.02f,0,0,0.2f); Yaw_Inertial_Update(tick);
    assert(fabsf(g_yaw_inertial.relative_rad+0.02f)<1e-6f);
    assert(fabsf(g_yaw_inertial.rate_rad_s+0.2f)<1e-6f);
    puts("PASS: compile-time reverse sign flips heading and transformed rate together");
    return 0;
  }
  for (int pitch=0;pitch<=41;pitch++) {
    tick++;
    mock_pose(30*RAD,pitch*RAD,70*RAD,2*RAD,5*RAD,10*RAD);
    Yaw_Inertial_Update(tick);
    assert(g_yaw_inertial.valid);
    assert(fabsf(g_yaw_inertial.rate_rad_s-10*RAD)<1e-6f);
    mock_pose(-25*RAD,pitch*RAD,70*RAD,3*RAD,8*RAD,0);
    Yaw_Inertial_Update(tick);
    assert(fabsf(g_yaw_inertial.rate_rad_s)<1e-6f);
  }
  mock_pose(0,41*RAD,179*RAD,0,0,10*RAD); Yaw_Inertial_Update(tick);
  Yaw_Inertial_Zero(0);
  mock_pose(0,41*RAD,-179*RAD,0,0,10*RAD); Yaw_Inertial_Update(tick);
  assert(fabsf(g_yaw_inertial.relative_rad-2*RAD)<1e-5f);
  Yaw_Inertial_Update(tick+6);
  assert(!g_yaw_inertial.valid);
  assert(!Gimbal_Yaw_Set_Mode(YAW_MODE_INERTIAL));

  yaw_ready();
  imu_valid=0;
  assert(!Gimbal_Yaw_Set_Mode(YAW_MODE_INERTIAL));
  assert(g_yaw_inertial.switch_result==2 && g_yaw_inertial.mode==YAW_MODE_ENCODER);
  imu_valid=1;
  // 非零机械位置和航向切入，不能追旧零点。
  yaw_position=30*RAD; base_heading=70*RAD; mock_pitch_up=41*RAD;
  for(int i=0;i<10;i++) step(1,0);
  assert(Gimbal_Yaw_Repeat_Start(25,35,5));
  assert(Gimbal_Yaw_Set_Mode(YAW_MODE_INERTIAL));
  assert(!g_gimbal_repeat[Yaw].active);
  assert(g_yaw_test.target_relative_rad==0 && g_yaw_test.command_relative_rad==0);
  const float encoder_zero=g_yaw_test.origin_rad;
  const Yaw_Test_t pitch_before=g_pitch_test;
  const float pitch_scale=Pitch_FF_Get_Scale();
  for(int i=0;i<10;i++) step(1,0);
  assert(yaw_kp==0 && yaw_kd==0 && fabsf(yaw_ff)<1e-5f);
  assert(g_yaw_test.origin_rad==encoder_zero);
  assert(g_pitch_test.kp==pitch_before.kp && g_pitch_test.kd==pitch_before.kd);
  assert(g_pitch_test.command_relative_rad==pitch_before.command_relative_rad);
  assert(Pitch_FF_Get_Scale()==pitch_scale);
  assert(Gimbal_Yaw_Move_Relative_Speed_Deg(5,5));
  for(int i=0;i<3000;i++) step(1,1);
  assert(fabsf(g_yaw_inertial.relative_rad-5*RAD)<0.005f);
  assert(fabsf(yaw_position-35*RAD)<0.005f);
  // 底座转动时编码器应反向补偿，惯性目标保持。
  base_rate=10*RAD;
  for(int i=0;i<2000;i++) step(1,1);
  base_rate=0;
  for(int i=0;i<1000;i++) step(1,1);
  assert(fabsf(g_yaw_inertial.relative_rad-5*RAD)<0.005f);
  assert(fabsf(yaw_position-15*RAD)<0.008f);
  assert(Gimbal_Yaw_Repeat_Start(-2,2,5));
  int direction=g_gimbal_repeat[Yaw].direction,reversals=0;
  for(int i=0;i<7000;i++) {
    step(1,1);
    if(direction!=g_gimbal_repeat[Yaw].direction) {
      direction=g_gimbal_repeat[Yaw].direction; reversals++;
    }
  }
  assert(reversals>=4 && yaw_kp==0 && yaw_kd==0);
  Gimbal_Yaw_Repeat_Cancel();
  assert(!g_gimbal_repeat[Yaw].active);
  assert(Gimbal_Yaw_Set_Mode(YAW_MODE_ENCODER));
  assert(g_yaw_test.command_relative_rad==g_yaw_test.measured_relative_rad);
  for(int i=0;i<10;i++) step(1,0);
  assert(yaw_kp==g_yaw_test.kp && yaw_kd==g_yaw_test.kd && yaw_ff==0);
  assert(Gimbal_Yaw_Set_Mode(YAW_MODE_INERTIAL));
  // IMU重标定改变参考系时退出，不悄悄重置目标后续跑。
  imu_sample.reference_id++;
  step(1,0);
  assert(g_yaw_test.state==YAW_TEST_FAULT && g_yaw_test.fault==YAW_FAULT_IMU && !yaw_enabled);
  Gimbal_Yaw_Enable(); for(int i=0;i<100;i++) step(1,0);
  assert(g_yaw_inertial.mode==YAW_MODE_ENCODER && !g_gimbal_repeat[Yaw].active);
  imu_auto=1;
  step(1,0);
  assert(Gimbal_Yaw_Set_Mode(YAW_MODE_INERTIAL));
  imu_valid=0;
  step(1,0);
  assert(g_yaw_test.state==YAW_TEST_FAULT && !yaw_enabled);
  Gimbal_Yaw_Test_Stop(); Gimbal_Yaw_Enable();
  for(int i=0;i<100;i++) step(1,0);
  assert(g_yaw_inertial.mode==YAW_MODE_ENCODER && yaw_enabled);
  puts("PASS: tilted gyro projection, pitch/roll rejection, heading wrap, current-pose switching, external torque PD, base rejection, inertial repeat, reverse sign, IMU loss/reference change, encoder fallback after re-enable; Pitch gains/FF untouched");
  return 0;
}
