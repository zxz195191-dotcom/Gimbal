#include "headfile.h"
#include "pitch_continuous_cal.h"
#include "imu_attitude.h"
#include <assert.h>
#undef assert
#define assert(check) do { if (!(check)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #check); exit(1); } } while (0)

CAN_HandleTypeDef hcan2;
uint32_t g_can2_filter_ret, g_can2_start_ret, g_can2_notify_ret;
volatile DM_Motor_Data_t dm_motor[DM_NUM];
static uint32_t tick;
static float position, velocity, torque, p_des, v_des, kp, kd;
static float gravity_slope;
static float applied_ff;
static int enabled;
static float yaw_position, yaw_velocity, yaw_p_des, yaw_v_des, yaw_kp, yaw_kd;
static int yaw_enabled, yaw_feedback=1;
static float yaw_ff, base_heading, base_rate, mock_pitch_up;
static int imu_valid, imu_auto;
static IMU_AttitudeSample_t imu_sample;
uint8_t IMU_AttitudeGet(uint32_t now, IMU_AttitudeSample_t *out) {
  *out=imu_sample;
  out->age_ms=now-imu_sample.timestamp_ms;
  return imu_valid && out->age_ms<=5;
}
/* 独立用ZYX欧拉运动学生成传感器值，测试生产代码的四元数投影。 */
static void mock_pose(float roll,float pitch_up,float yaw,float rd,float pd,float yd) {
  float theta=-pitch_up, td=-pd;
  float cr=cosf(roll/2),sr=sinf(roll/2),cp=cosf(theta/2),sp=sinf(theta/2);
  float cy=cosf(yaw/2),sy=sinf(yaw/2);
  imu_sample.quaternion_wxyz[0]=cr*cp*cy+sr*sp*sy;
  imu_sample.quaternion_wxyz[1]=sr*cp*cy-cr*sp*sy;
  imu_sample.quaternion_wxyz[2]=cr*sp*cy+sr*cp*sy;
  imu_sample.quaternion_wxyz[3]=cr*cp*sy-sr*sp*cy;
  imu_sample.gyro_body_rad_s[0]=rd-yd*sinf(theta);
  imu_sample.gyro_body_rad_s[1]=td*cosf(roll)+yd*sinf(roll)*cosf(theta);
  imu_sample.gyro_body_rad_s[2]=-td*sinf(roll)+yd*cosf(roll)*cosf(theta);
  imu_sample.timestamp_ms=tick;
  ++imu_sample.sequence;
}
uint32_t HAL_GetTick(void) { return tick; }
GPIO_PinState HAL_GPIO_ReadPin(int port, int pin) { (void)port; (void)pin; return GPIO_PIN_RESET; }
HAL_StatusTypeDef dm4310_disable(CAN_HandleTypeDef *h, uint16_t id) { (void)h; if(id==1) enabled=0; if(id==2) yaw_enabled=0; return HAL_OK; }
HAL_StatusTypeDef dm4310_enable(CAN_HandleTypeDef *h, uint16_t id) { (void)h; if(id==1) enabled=1; if(id==2) yaw_enabled=1; return HAL_OK; }
HAL_StatusTypeDef dm4310_clear_error(CAN_HandleTypeDef *h, uint16_t id) { (void)h; (void)id; return HAL_OK; }
HAL_StatusTypeDef dm4310_send_mit(CAN_HandleTypeDef *h, uint16_t id, float p, float v, float k, float d, float ff) {
  (void)h;
  if(id==1) { p_des=p; v_des=v; kp=k; kd=d; applied_ff=ff; if(!g_pitch_cont.selected) assert(ff==0); }
  if(id==2) { yaw_p_des=p; yaw_v_des=v; yaw_kp=k; yaw_kd=d; yaw_ff=ff; }
  return HAL_OK;
}
uint8_t DM_Motor_Feedback_Valid(DM_Motor_ID_e id, uint32_t now) {
  return dm_motor[id].rx_count && now-dm_motor[id].last_rx_ms<=DM_FEEDBACK_TIMEOUT_MS;
}
static void step(int feedback, int move) {
  ++tick;
  torque=enabled ? kp*(p_des-position)+kd*(v_des-velocity)+applied_ff : 0;
  if(move) { velocity+=(torque-1.2f-gravity_slope*fmaxf(position,0)-0.1f*velocity)/0.06f*0.001f; position+=velocity*0.001f; }
  else velocity=0;
  if(feedback) {
    dm_motor[Pitch].p_int=position;
    dm_motor[Pitch].v_int=velocity;
    dm_motor[Pitch].t_int=torque;
    dm_motor[Pitch].last_rx_ms=tick;
    ++dm_motor[Pitch].rx_count;
  }
  float yaw_torque=yaw_enabled ? yaw_kp*(yaw_p_des-yaw_position)+yaw_kd*(yaw_v_des-yaw_velocity)+yaw_ff : 0;
  if(move) { yaw_velocity+=(yaw_torque-0.1f*yaw_velocity)/0.02f*0.001f; yaw_position+=yaw_velocity*0.001f; }
  else yaw_velocity=0;
  if(yaw_feedback) {
    dm_motor[Yaw].p_int=yaw_position;
    dm_motor[Yaw].v_int=yaw_velocity;
    dm_motor[Yaw].t_int=yaw_torque;
    dm_motor[Yaw].last_rx_ms=tick;
    ++dm_motor[Yaw].rx_count;
  }
  if (imu_auto && imu_valid) {
    base_heading+=base_rate*0.001f;
    mock_pose(0,mock_pitch_up,yaw_position+base_heading,0,0,yaw_velocity+base_rate);
  }
  Gimbal_Yaw_Test_Update(tick, GPIO_PIN_RESET);
}
static void reset(void) {
  tick=0; position=velocity=torque=p_des=v_des=kp=kd=0; enabled=0;
  applied_ff=gravity_slope=0;
  yaw_position=yaw_velocity=yaw_p_des=yaw_v_des=yaw_kp=yaw_kd=0;
  yaw_enabled=0; yaw_feedback=1;
  yaw_ff=base_heading=base_rate=mock_pitch_up=0;
  imu_valid=imu_auto=0;
  memset(&imu_sample,0,sizeof(imu_sample));
  imu_sample.reference_id=1;
  memset((void *)&g_pitch_cont,0,sizeof(g_pitch_cont));
  (void)Pitch_FF_Set_Scale(0);
  memset((void *)dm_motor,0,sizeof(dm_motor));
  Pitch_Cal_Cancel(); Gimbal_Yaw_Test_Init();
}
static void ready(void) {
  reset(); Gimbal_Pitch_Enable();
  for(int i=0;i<100;i++) step(1,0);
  assert(g_pitch_test.state==YAW_TEST_ACTIVE);
  assert(!g_pitch_scan.active);
}
