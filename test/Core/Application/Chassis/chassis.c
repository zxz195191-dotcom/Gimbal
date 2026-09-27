#include "headfile.h"

void chassis_Init(void) {
  for (uint8_t i = 0; i < Motor_Count; i++) {
    PID_Init(&pid_spd[i],P,I,D,MAX_Out,MAX_Integral);
    target_speed_rpm[i] = 0.0f;
    current_cmd[i] = 0;
  }
}

int16_t Clamp_Target_Rpm(int value)
{
  if (value > 32767) value = 32767;
  if (value < -32768) value = -32768;
  return (int16_t)value;
}


// 底盘运动学解算函数
void Chassis_Kinematics_Resolve(float vx, float vy, float vw, int16_t target[Motor_Count]) {
  // 根据之前测试得出的 X 型麦轮正负号规律进行分配
  target[Right_front] = (int16_t)( vx - vy + vw);
  target[Left_front]  = (int16_t)(-vx - vy + vw);
  target[Left_Rear]   = (int16_t)(-vx + vy + vw);
  target[Right_Rear]  = (int16_t)( vx + vy + vw);

  // 限制最大速度，防止计算后溢出（使用你原有的限制函数）
  target[Right_front] = Clamp_Target_Rpm(target[Right_front]);
  target[Left_front]  = Clamp_Target_Rpm(target[Left_front]);
  target[Left_Rear]   = Clamp_Target_Rpm(target[Left_Rear]);
  target[Right_Rear]  = Clamp_Target_Rpm(target[Right_Rear]);
}
