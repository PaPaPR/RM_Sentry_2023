#include "serial/robot_serial.hpp"

RobotSerial::RobotSerial(std::string port, unsigned long baud) {
  auto timeout = serial::Timeout::simpleTimeout(serial::Timeout::max());
  this->setPort(port);
  this->setBaudrate(baud);
  this->setTimeout(timeout);
}

bool RobotSerial::Init() {
  if (!this->isOpen()) {
    try {
      this->open();
      return true;
    } catch (const std::exception &e) {
      return false;
    }
  }
  return false;
}

bool RobotSerial::SendCMD() {
  std::lock_guard<std::mutex> lck(cmd_mtx_);

  try {
    this->write((uint8_t *)&robot_cmd_buff_, sizeof(robot_cmd_buff_));
  } catch (const serial::PortNotOpenedException &e) {
    try {
      this->open();
    } catch (const std::exception &e) {
      return false;
    }
  } catch (const std::exception &e) {
    this->close();
    return false;
  }
  return true;
}

int RobotSerial::RecvCMD() {
  uint8_t temp;
  this->read(&temp, 1);
  while (temp != 'S') this->read(&temp, 1);
  this->read(&temp, 1);
  switch (temp) {
    case INF_CHASSIS_GIMBAL: {
      this->read((uint8_t *)&robot_inf_buf_, sizeof(robot_inf_buf_));
      if (robot_inf_buf_.end == (unsigned)'E') {
        std::lock_guard<std::mutex> lck(inf_mtx_);
        return INF_CHASSIS_GIMBAL;
      }
    }
    default:
      break;
  }
  return false;
}

void RobotSerial::ChassisCMD(const float _lx, const float _ly,
                             const float _az) {
  std::lock_guard<std::mutex> lck(cmd_mtx_);
  robot_cmd_buff_.chassis_lx = _lx;
  robot_cmd_buff_.chassis_ly = -_ly;
  robot_cmd_buff_.chassis_az = -_az;
}

void RobotSerial::GimbalCMD(const float _yaw, const float _pitch,
                            const bool _auto_fire) {
  std::lock_guard<std::mutex> lck(cmd_mtx_);
  robot_cmd_buff_.gimbal_yaw = _yaw;
  robot_cmd_buff_.gimbal_pitch = _pitch;
  robot_cmd_buff_.auto_fire = _auto_fire;
}

void RobotSerial::ReadINF(INFChassisGimbalBuf &_chassis_inf) {
  std::lock_guard<std::mutex> lck(inf_mtx_);
  _chassis_inf = robot_inf_buf_;
}
