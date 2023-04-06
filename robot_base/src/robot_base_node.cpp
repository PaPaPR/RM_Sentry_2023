#include "robot_base_node.h"

robot_base_node::robot_base_node()
    : timeout_dur_(ros::Duration(1.0)), tf_listener_(tf_buffer_) {
  ros::NodeHandle nhp("~");
  nhp.param<double>("map_init_x", map2odom_tfS_.transform.translation.x, 0.);
  nhp.param<double>("map_init_y", map2odom_tfS_.transform.translation.y, 0.);
  nhp.param<double>("map_init_yaw", map_init_yaw_, 0.0);
  nhp.param<double>("gimbal2chassis_x", gimbal2chassis_x_, 0.0);
  nhp.param<double>("gimbal2chassis_y", gimbal2chassis_y_, 0.0);

  update_map2odom_srv_ = nh_.advertiseService(
      "update_map2odom_service", &robot_base_node::UpdateMap2OdomTFSRV, this);
  InitMap2OdomTF();

  std::string serial_port;
  std::string serial_baud;
  nhp.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
  nhp.param<std::string>("serial_baud", serial_baud, "500000");
  robot_serial_ =
      std::make_shared<RobotSerial>(serial_port, std::stoul(serial_baud));
  ros_sub_cmd_vel_ =
      nh_.subscribe("/cmd_vel", 1, &robot_base_node::ChassisCallBack, this);
  if (robot_serial_->Init()) {
    ROS_INFO("serial initial success.");
  } else {
    ROS_WARN("serial initial failed.");
  }

  serial_send_thread_ = std::thread([this] {
    while (ros::ok()) {
      auto now = ros::Time::now();
      // 底盘超时
      // if (now - ros_sub_chassis_lasttime_ > timeout_dur_)
      //   robot_serial_->ChassisCMD(0.f, 0.f, 0.f);
      robot_serial_->SendCMD();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });

  serial_recv_thread_ = std::thread([this] {
    while (ros::ok()) {
      auto now = ros::Time::now();
      // if (now - ros_pub_inf_lasttime_ > timeout_dur_) {
      //   INFChassisGimbalBuf robot_inf;
      //   this->INFCallBack(robot_inf);
      // }
      switch (robot_serial_->RecvCMD()) {
        ros_pub_inf_lasttime_ = now;
        case INF_CHASSIS_GIMBAL: {
          INFChassisGimbalBuf robot_inf;
          robot_serial_->ReadINF(robot_inf);
          this->INFCallBack(robot_inf);
          break;
        }
        default:
          break;
      }
    }
  });

  static_tf_thread_ = std::thread([this] {
    ros::Rate rate(10.0);
    while (ros::ok()) {
      SendMap2OdomTF();
      rate.sleep();
    }
  });
}

robot_base_node::~robot_base_node() { robot_serial_->close(); }

void robot_base_node::ChassisCallBack(
    const geometry_msgs::Twist::ConstPtr &vel) {
  robot_serial_->ChassisCMD(vel->linear.x, vel->linear.y, vel->angular.z);
  ros_sub_chassis_lasttime_ = ros::Time::now();
}

void robot_base_node::INFCallBack(const INFChassisGimbalBuf &robot_inf) {
  gimbal2baselink_tfs_.header.stamp = ros::Time::now();
  gimbal2baselink_tfs_.header.frame_id = "laser_link";
  gimbal2baselink_tfs_.child_frame_id = "base_link";
  gimbal2baselink_tfs_.transform.translation.x = gimbal2chassis_x_;
  gimbal2baselink_tfs_.transform.translation.y = gimbal2chassis_y_;
  tf2::Quaternion q;
  q.setRPY(0, 0,
           robot_inf.gimbal_chassis_yaw_dif / 180.0 * 3.14159265358979323846);
  q.normalize();
  gimbal2baselink_tfs_.transform.rotation.x = q.x();
  gimbal2baselink_tfs_.transform.rotation.y = q.y();
  gimbal2baselink_tfs_.transform.rotation.z = q.z();
  gimbal2baselink_tfs_.transform.rotation.w = q.w();
  gimbal2baselink_br_.sendTransform(gimbal2baselink_tfs_);

  chassis2baselink_tfs_.header.stamp = ros::Time::now();
  chassis2baselink_tfs_.header.frame_id = "base_link";
  chassis2baselink_tfs_.child_frame_id = "chassis_link";
  chassis2baselink_tfs_.transform.translation.x = 0.;
  chassis2baselink_tfs_.transform.translation.y = 0.;
  tf2::Quaternion chassis_tfs_q;
  // chassis_tfs_q.setRPY(0, 0, robot_inf.);
}

void robot_base_node::InitMap2OdomTF() {
  map2odom_tfS_.header.frame_id = "map";
  map2odom_tfS_.child_frame_id = "odom";
  map2odom_tfS_.transform.translation.x = 0.;
  map2odom_tfS_.transform.translation.y = 0.;
  map2odom_tfS_.transform.translation.z = 0.;
  map2odom_tfS_.transform.rotation.x = 0.;
  map2odom_tfS_.transform.rotation.y = 0.;
  map2odom_tfS_.transform.rotation.z = 0.;
  map2odom_tfS_.transform.rotation.w = 0.;
}

void robot_base_node::UpdateMap2OdomTF() {
  map2odom_tfS_.transform.translation.x =
      baselink2odom_tfS_.transform.translation.x;
  map2odom_tfS_.transform.translation.y =
      baselink2odom_tfS_.transform.translation.y;

  tf2::Quaternion q_orig(baselink2odom_tfS_.transform.rotation.x,
                         baselink2odom_tfS_.transform.rotation.y,
                         baselink2odom_tfS_.transform.rotation.z,
                         baselink2odom_tfS_.transform.rotation.w);
  tf2::Matrix3x3 m_new(q_orig);
  double roll, pitch, yaw;
  m_new.getRPY(roll, pitch, yaw);

  tf2::Quaternion q_new;
  q_new.setRPY(0, 0, -map_init_yaw_ + yaw);
  q_new.normalize();

  map2odom_tfS_.transform.rotation.x = q_new.x();
  map2odom_tfS_.transform.rotation.y = q_new.y();
  map2odom_tfS_.transform.rotation.z = q_new.z();
  map2odom_tfS_.transform.rotation.w = q_new.w();
}
bool robot_base_node::UpdateMap2OdomTFSRV(
    robot_base::UpdateMap2Odom::Request &req,
    robot_base::UpdateMap2Odom::Response &res) {
  UpdateMap2OdomTF();
  return true;
}

void robot_base_node::SendMap2OdomTF() {
  try {
    baselink2odom_tfS_ =
        tf_buffer_.lookupTransform("base_link", "odom", ros::Time(0));
    UpdateMap2OdomTF();
  } catch (tf2::TransformException &ex) {
    // ROS_WARN("%s", ex.what());
  }

  map2odom_tfS_.header.stamp = ros::Time::now();
  map2odom_br_.sendTransform(map2odom_tfS_);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_base_node");
  robot_base_node node;

  ros::spin();
}