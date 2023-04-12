#include "robot_base_node.h"

robot_base_node::robot_base_node()
    : timeout_dur_(ros::Duration(0.5)), tf_listener_(tf_buffer_) {
  ros::NodeHandle nhp("~");
  nhp.param<double>("map_init_x", map_init_x_, 0.);
  nhp.param<double>("map_init_y", map_init_y_, 0.);
  nhp.param<double>("map_init_yaw", map_init_yaw_, 0.0);

  update_map2odom_srv_ = nh_.advertiseService(
      "update_map2odom_service", &robot_base_node::UpdateMap2OdomTFSRV, this);
  InitTFS();
  UpdateMap2OdomTF();

  std::string serial_port;
  std::string serial_baud;
  nhp.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
  nhp.param<std::string>("serial_baud", serial_baud, "500000");
  robot_serial_ =
      std::make_shared<RobotSerial>(serial_port, std::stoul(serial_baud));
  vel_acc_sub_ = nh_.subscribe<roborts_msgs::TwistAccel>(
      "/cmd_vel_acc", 1, &robot_base_node::VelAccCB, this);
  ros_sub_chassis_cmd_ =
      nh_.subscribe("/chassis/cmd", 1, &robot_base_node::ChassisCmdCB, this);
  if (robot_serial_->Init()) {
    ROS_INFO("serial initial success.");
  } else {
    ROS_WARN("serial initial failed.");
  }

  referee_rmul_pub_ = nh_.advertise<robot_base::RefereeRMUL>("/referee", 1);

  serial_send_thread_ = std::thread([this] {
    while (ros::ok()) {
      if (!robot_serial_->SendCMD()) ROS_WARN("serial send failed.");
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });

  serial_recv_thread_ = std::thread([this] {
    while (ros::ok()) {
      switch (robot_serial_->RecvCMD()) {
        case INF_CHASSIS_GIMBAL: {
          INFChassisGimbalBuf robot_inf;
          robot_serial_->ReadINF(robot_inf);
          this->ChassisGimbalCB(robot_inf);
          break;
        }
        case REFEREE_RMUL: {
          RefereeRMULBuf referee_rmul_buf_;
          robot_serial_->ReadCompetition(referee_rmul_buf_);
          RefereeRMULCB(referee_rmul_buf_);
        }
        default:
          break;
      }
    }
  });

  tf_thread_ = std::thread([this] {
    ros::Rate rate(150.0);
    while (ros::ok()) {
      SendTF();
      ListenTF();
      rate.sleep();
    }
  });

  vel_update_thread_ = std::thread([this] {
    ros::Rate rate(100.0);
    while (ros::ok()) {
      UpdateVelLoop();
      rate.sleep();
    }
  });
}

robot_base_node::~robot_base_node() { robot_serial_->close(); }

void robot_base_node::InitTFS() {
  odom_tf_.header.frame_id = "odom";
  odom_tf_.child_frame_id = "gimbal_link";
  odom_tf_.transform.translation.x = 0.;
  odom_tf_.transform.translation.y = 0.;
  odom_tf_.transform.translation.z = 0.;
  odom_tf_.transform.rotation.x = 0.;
  odom_tf_.transform.rotation.y = 0.;
  odom_tf_.transform.rotation.z = 0.;
  odom_tf_.transform.rotation.w = 0.;

  base_link_tf_.header.frame_id = "gimbal_link";
  base_link_tf_.child_frame_id = "base_link";
  base_link_tf_.transform.translation.x = 0.;
  base_link_tf_.transform.translation.y = 0.;
  base_link_tf_.transform.translation.z = 0.;
  base_link_tf_.transform.rotation.x = 0.;
  base_link_tf_.transform.rotation.y = 0.;
  base_link_tf_.transform.rotation.z = 0.;
  base_link_tf_.transform.rotation.w = 1.;

  map_tf_.header.frame_id = "map";
  map_tf_.child_frame_id = "odom";
  map_tf_.transform.translation.x = map_init_x_;
  map_tf_.transform.translation.y = map_init_y_;
  map_tf_.transform.translation.z = 0.;
  map_tf_.transform.rotation.x = 0.;
  map_tf_.transform.rotation.y = 0.;
  map_tf_.transform.rotation.z = 0.;
  map_tf_.transform.rotation.w = 1.;

  laser_lio_tfl_.transform.translation.x = 0.;
  laser_lio_tfl_.transform.translation.y = 0.;
  laser_lio_tfl_.transform.translation.z = 0.;
  laser_lio_tfl_.transform.rotation.x = 0.;
  laser_lio_tfl_.transform.rotation.y = 0.;
  laser_lio_tfl_.transform.rotation.z = 0.;
  laser_lio_tfl_.transform.rotation.w = 1.;
  
  odom_tfl_.transform.translation.x = 0.;
  odom_tfl_.transform.translation.y = 0.;
  odom_tfl_.transform.translation.z = 0.;
  odom_tfl_.transform.rotation.x = 0.;
  odom_tfl_.transform.rotation.y = 0.;
  odom_tfl_.transform.rotation.z = 0.;
  odom_tfl_.transform.rotation.w = 1.;
}

void robot_base_node::ChassisCmdCB(
    const robot_base::ChassisCmd::ConstPtr &_cmd) {
  robot_serial_->ChassisModeSet(_cmd->mode);
}

void robot_base_node::ChassisGimbalCB(const INFChassisGimbalBuf &robot_inf) {}

void robot_base_node::VelAccCB(const roborts_msgs::TwistAccel::ConstPtr &msg) {
  if (vel_acc_first_cb_) vel_acc_first_cb_ = false;
  std::lock_guard<std::mutex> vel_acc_guard(vel_acc_mutex);
  new_acc_cmd_ = true;
  vel_acc_ = *msg;
  time_begin_acc_ = std::chrono::high_resolution_clock::now();
  time_begin_ = std::chrono::high_resolution_clock::now();
}

void robot_base_node::UpdateVelLoop() {
  if (!vel_acc_first_cb_) {
    if (new_acc_cmd_) {
      std::lock_guard<std::mutex> vel_acc_guard(vel_acc_mutex);
      cmd_vel_ = vel_acc_.twist;
      new_acc_cmd_ = false;
      auto actual_time =
          std::chrono::duration<double, std::ratio<1, 1>>(
              std::chrono::high_resolution_clock::now() - time_begin_acc_)
              .count();
      base_link_yaw_rad_ +=
          (cmd_vel_.angular.z * actual_time +
           0.5 * vel_acc_.accel.angular.z * actual_time * actual_time);
      while (base_link_yaw_rad_ > 360.) base_link_yaw_rad_ -= 360.;
      while (base_link_yaw_rad_ < -360.) base_link_yaw_rad_ += 360.;
      double yaw = base_link_yaw_rad_ * 180. / 3.14159265358979323846;

      CHASSISCMD cmd;
      cmd.lx = cmd_vel_.linear.x;
      cmd.ly = cmd_vel_.linear.y;
      cmd.az = cmd_vel_.angular.z;
      cmd.forward = yaw;
      robot_serial_->ChassisCMD(cmd);
    } else {
      auto actual_time =
          std::chrono::duration<double, std::ratio<1, 1>>(
              std::chrono::high_resolution_clock::now() - time_begin_)
              .count();
      time_begin_ = std::chrono::high_resolution_clock::now();

      cmd_vel_.linear.x =
          cmd_vel_.linear.x + actual_time * vel_acc_.accel.linear.x;
      cmd_vel_.linear.y =
          cmd_vel_.linear.y + actual_time * vel_acc_.accel.linear.y;
      cmd_vel_.angular.z =
          cmd_vel_.angular.z + actual_time * vel_acc_.accel.angular.z;
      base_link_yaw_rad_ += (cmd_vel_.angular.z * actual_time);
      while (base_link_yaw_rad_ > 360.) base_link_yaw_rad_ -= 360.;
      while (base_link_yaw_rad_ < -360.) base_link_yaw_rad_ += 360.;
      double yaw = base_link_yaw_rad_ * 180. / 3.14159265358979323846;

      CHASSISCMD cmd;
      cmd.lx = cmd_vel_.linear.x;
      cmd.ly = cmd_vel_.linear.y;
      cmd.az = cmd_vel_.angular.z;
      cmd.forward = yaw;
      robot_serial_->ChassisCMD(cmd);
    }
  }
}

// To-do: 初始化后位置大概正确但有偏转
void robot_base_node::UpdateMap2OdomTF() {
  map_tf_.transform.translation.x =
      odom_tfl_.transform.translation.x + map_init_x_;
  map_tf_.transform.translation.y =
      odom_tfl_.transform.translation.y + map_init_y_;

  tf2::Quaternion q_orig(
      odom_tfl_.transform.rotation.x, odom_tfl_.transform.rotation.y,
      odom_tfl_.transform.rotation.z, odom_tfl_.transform.rotation.w);
  tf2::Matrix3x3 m_new(q_orig);
  double roll, pitch, yaw;
  m_new.getRPY(roll, pitch, yaw);

  tf2::Quaternion q_new;
  q_new.setRPY(0., 0., map_init_yaw_ + yaw);
  q_new.normalize();

  map_tf_.transform.rotation.x = q_new.x();
  map_tf_.transform.rotation.y = q_new.y();
  map_tf_.transform.rotation.z = q_new.z();
  map_tf_.transform.rotation.w = q_new.w();
}

bool robot_base_node::UpdateMap2OdomTFSRV(
    robot_base::UpdateMap2Odom::Request &req,
    robot_base::UpdateMap2Odom::Response &res) {
  UpdateMap2OdomTF();
  return true;
}

void robot_base_node::ListenTF() {
  try {
    laser_lio_tfl_ = tf_buffer_.lookupTransform(
        "laser_lio_odom", "gimbal_odom_center", ros::Time(0));

    odom_tfl_ =
        tf_buffer_.lookupTransform("gimbal_link", "odom", ros::Time(0));

  } catch (tf2::TransformException &ex) {
    // ROS_WARN("%s", ex.what());
  }
}

void robot_base_node::SendTF() {
  odom_tf_.header.stamp = ros::Time::now();
  odom_tf_.transform.translation.x = laser_lio_tfl_.transform.translation.x;
  odom_tf_.transform.translation.y = laser_lio_tfl_.transform.translation.y;
  odom_tf_.transform.translation.z = 0.;

  tf2::Quaternion q_orig(
      laser_lio_tfl_.transform.rotation.x, laser_lio_tfl_.transform.rotation.y,
      laser_lio_tfl_.transform.rotation.z, laser_lio_tfl_.transform.rotation.w);
  tf2::Matrix3x3 m_new(q_orig);
  double roll, pitch, yaw;
  m_new.getRPY(roll, pitch, yaw);

  tf2::Quaternion q_new;
  q_new.setRPY(0., 0., yaw);
  q_new.normalize();

  // odom_tf_.transform.rotation.x = laser_lio_tfl_.transform.rotation.x;
  // odom_tf_.transform.rotation.y = laser_lio_tfl_.transform.rotation.y;
  // odom_tf_.transform.rotation.z = laser_lio_tfl_.transform.rotation.z;
  // odom_tf_.transform.rotation.w = laser_lio_tfl_.transform.rotation.w;
  odom_tf_.transform.rotation.x = q_new.getX();
  odom_tf_.transform.rotation.y = q_new.getY();
  odom_tf_.transform.rotation.z = q_new.getZ();
  odom_tf_.transform.rotation.w = q_new.getW();
  odom_br_.sendTransform(odom_tf_);

  map_tf_.header.stamp = ros::Time::now();
  map_br_.sendTransform(map_tf_);

  tf2::Quaternion q_gb;
  q_gb.setRPY(0, 0, base_link_yaw_rad_);

  base_link_tf_.transform.rotation.x = q_gb.x();
  base_link_tf_.transform.rotation.y = q_gb.y();
  base_link_tf_.transform.rotation.z = q_gb.z();
  base_link_tf_.transform.rotation.w = q_gb.w();

  base_link_tf_.header.stamp = ros::Time::now();
  base_link_br_.sendTransform(base_link_tf_);
}

void robot_base_node::RefereeRMULCB(const RefereeRMULBuf &_referee) {
  robot_base::RefereeRMUL referee;
  referee.game_progress = _referee.game_progress;
  referee.game_progress_remain = _referee.game_progress_remain;
  referee.robot_id = _referee.robot_id;
  referee.sentry_hp = _referee.sentry_hp;

  referee_rmul_pub_.publish(referee);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_base_node");
  robot_base_node node;

  ros::spin();
}