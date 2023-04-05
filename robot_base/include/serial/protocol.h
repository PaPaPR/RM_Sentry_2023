#pragma once

struct CMDRobotBuff {
  uint8_t start = (unsigned)'S';
  float chassis_lx = 0.f;     // 底盘 x 轴速度
  float chassis_ly = 0.f;     // 底盘 y 轴速度
  float chassis_az = 0.f;     // 底盘自旋（顺时针为负）
  float gimbal_yaw = 0.f;     // 云台 yaw 运动角度
  float gimbal_pitch = 0.f;   // 云台 pitch 运动角度
  uint8_t auto_fire = false;  // 自动开火
  uint8_t end = (unsigned)'E';
} __attribute__((packed));

#define INF_CHASSIS_GIMBAL (unsigned)'A'
struct INFChassisGimbalBuf {
  float gimbal_yaw = 0.f;
  float gimbal_yaw_speed = 0.f;
  float gimbal_pitch = 0.f;            // 云台 pitch 角度
  float gimbal_chassis_yaw_dif = 0.f;  // 云台与底盘差值
  uint8_t end;
} __attribute__((packed));