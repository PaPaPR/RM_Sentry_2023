#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "robot_base/ChassisCmd.h"
#include "robot_base/UpdateMap2Odom.h"
#include "robot_base/RefereeRMUL.h"
#include "roborts_msgs/TwistAccel.h"
#include "geometry_msgs/Twist.h"
#include "robot_serial.hpp"
#include "serial/protocol.h"

class robot_base_node {
 private:
  ros::NodeHandle nh_;
  ros::Subscriber ros_sub_cmd_vel_;
  ros::Subscriber ros_sub_chassis_cmd_;
  ros::Subscriber vel_acc_sub_;

  ros::Duration timeout_dur_;           // 数据超时时间

  std::shared_ptr<RobotSerial> robot_serial_;

  std::thread serial_send_thread_;
  std::thread serial_recv_thread_;
  std::thread tf_thread_;
  std::thread vel_update_thread_;

  ros::ServiceServer update_map2odom_srv_;

  double map_init_x_{0.f};   // 地图初始化 x
  double map_init_y_{0.f};   // 地图初始化 y
  double map_init_yaw_{0.};  // 地图初始化 yaw

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  geometry_msgs::TransformStamped laser_lio_tfl_;
  geometry_msgs::TransformStamped odom_tfl_;

  tf2_ros::TransformBroadcaster odom_br_;
  geometry_msgs::TransformStamped odom_tf_;
  tf2_ros::TransformBroadcaster base_link_br_;
  geometry_msgs::TransformStamped base_link_tf_;

  tf2_ros::StaticTransformBroadcaster map2odom_br_;
  geometry_msgs::TransformStamped map_tf_;

  // 底盘运动速度计算以及前进方向确定
  roborts_msgs::TwistAccel vel_acc_;
  geometry_msgs::Twist cmd_vel_;
  std::mutex vel_acc_mutex;
  bool new_acc_cmd_ {false};
  bool vel_acc_first_cb_ {true};
  std::chrono::high_resolution_clock::time_point time_begin_;
  std::chrono::high_resolution_clock::time_point time_begin_acc_;
  double base_link_yaw_rad_ {0.}; // 底盘运动方向相对于云台的角度

  // 比赛信息
  ros::Publisher referee_rmul_pub_;

 public:
  robot_base_node();
  ~robot_base_node();
  void InitTFS();
  void ChassisCmdCB(const robot_base::ChassisCmd::ConstPtr &_cmd);
  void ChassisGimbalCB(const INFChassisGimbalBuf &robot_inf);
  void VelAccCB(const roborts_msgs::TwistAccel::ConstPtr& msg);
  void UpdateVelLoop();
  void UpdateMap2OdomTF();
  bool UpdateMap2OdomTFSRV(robot_base::UpdateMap2Odom::Request &req,
                           robot_base::UpdateMap2Odom::Response &res);
  void ListenTF();
  void SendTF();
  void RefereeRMULCB(const RefereeRMULBuf &_inf);
};
