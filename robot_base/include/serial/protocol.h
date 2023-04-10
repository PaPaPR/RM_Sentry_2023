#pragma once

struct CMDRobotBuff {
  uint8_t start = (unsigned)'S';
  float chassis_lx = 0.f;       // 底盘 x 轴速度
  float chassis_ly = 0.f;       // 底盘 y 轴速度
  float chassis_az = 0.f;       // 底盘自旋（顺时针为负）
  float chassis_forward = 0.f;  // 底盘前进方向（角度）
  float gimbal_yaw = 0.f;       // 云台 yaw 运动角度
  float gimbal_pitch = 0.f;     // 云台 pitch 运动角度
  uint8_t auto_fire = false;    // 自动开火
  // 底盘运动模式：0 底盘云台分开控制, 1 低速小陀螺, 2 高速小陀螺
  uint8_t chassis_mode = 0;
  uint8_t end = (unsigned)'E';
} __attribute__((packed));

#define INF_CHASSIS_GIMBAL (unsigned)'A'
struct INFChassisGimbalBuf {
  float gimbal_yaw = 0.f;         // 云台 yaw 角度
  float gimbal_yaw_speed = 0.f;   // 云台 yaw 速度
  float gimbal_pitch = 0.f;       // 云台 pitch 角度
  float gimbal_to_forward = 0.f;  // 云台相对于前进方向的角度
  float forward_to_start = 0.f;   // 前进方向相对于启动朝向的角度
  uint8_t end;
} __attribute__((packed));

// RMUL 协议
#define REFEREE_RMUL (unsigned)'B'
struct RefereeRMULBuf {
  uint8_t game_progress{0};         // 比赛阶段
  uint8_t game_progress_remain{0};  // 该比赛阶段剩余时间
  uint8_t robot_id{7};
  uint16_t sentry_hp{600};
  uint8_t end;
} __attribute__((packed));

// RMUC 协议
#define REFEREE_RMUC (unsigned)'C'
struct INFRMUCBuf {
  uint8_t end;
} __attribute__((packed));