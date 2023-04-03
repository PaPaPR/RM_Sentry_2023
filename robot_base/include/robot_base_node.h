#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "robot_serial.hpp"
#include "serial/protocol.h"

class robot_base_node {
 private:
  ros::NodeHandle nh_;
  ros::Subscriber ros_sub_cmd_vel_;

  ros::Time ros_sub_chassis_lasttime_;  // 底盘话题最后一次接收时间
  ros::Time ros_pub_inf_lasttime_;      // 串口信息最后一次接收时间
  ros::Duration timeout_dur_;           // 数据超时时间

  std::shared_ptr<RobotSerial> robot_serial_;

  std::thread serial_send_thread_;
  std::thread serial_recv_thread_;
  std::thread static_tf_thread_;

  ros::ServiceServer update_map2odom_service_;

  tf2_ros::StaticTransformBroadcaster map2odom_broadcaster_;
  geometry_msgs::TransformStamped map2odom_transformStamped_;
  geometry_msgs::TransformStamped transformStamped_;
  double map_init_yaw_{0.};  // 逆时针为正
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double gimbal2chassis_x_ = 0.f;
  double gimbal2chassis_y_ = 0.f;
  tf2_ros::TransformBroadcaster gimbal_baselink_br_;
  geometry_msgs::TransformStamped gimbal_baselink_tfs_;

 public:
  robot_base_node();
  ~robot_base_node();
  void ChassisCallBack(const geometry_msgs::Twist::ConstPtr &vel);
  void INFCallBack(const INFChassisGimbalBuf &robot_inf);
  void InitMap2OdomTF();
  void UpdateMap2OdomTF();
  void SendMap2OdomTF();
};
