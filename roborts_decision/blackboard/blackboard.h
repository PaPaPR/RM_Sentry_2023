/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#ifndef ROBORTS_DECISION_BLACKBOARD_H
#define ROBORTS_DECISION_BLACKBOARD_H

#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "roborts_msgs/ArmorDetectionAction.h"

#include "io/io.h"
#include "../proto/decision.pb.h"
#include "costmap/costmap_interface.h"

#include "robot_base/RefereeRMUL.h"
#include "robot_base/AutoAimCmd.h"

namespace roborts_decision{

enum GameProgress {
  WAIT,  // 开始前
  INPROGRESS, // 进行中
  END,  // 结束
};

class Blackboard {
 public:
  typedef std::shared_ptr<Blackboard> Ptr;
  typedef roborts_costmap::CostmapInterface CostMap;
  typedef roborts_costmap::Costmap2D CostMap2D;
  explicit Blackboard(const std::string &proto_file_path):
      enemy_detected_(false),
      armor_detection_actionlib_client_("armor_detection_node_action", true){

    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

    std::string map_path = ros::package::getPath("roborts_costmap") + \
      "/config/costmap_parameter_config_for_decision.prototxt";
    costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,
                                             map_path);
    charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();

    costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();

    // Enemy fake pose
    ros::NodeHandle rviz_nh("/move_base_simple");
    enemy_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &Blackboard::GoalCallback, this);

    ros::NodeHandle nh;

    referee_rmul_sub_ = nh.subscribe("referee", 1, &Blackboard::RefereeCB, this);
    autoaim_enemy_detect_sub_ = nh.subscribe("cmd_autoaim", 1, &Blackboard::EnemyDetectCB, this);
    autoaim_enemy_sub_ = nh.subscribe("enemy_autoaim", 1, &Blackboard::EnemyCB, this);

    roborts_decision::DecisionConfig decision_config;
    roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config);

    // if (!decision_config.simulate()){

    //   armor_detection_actionlib_client_.waitForServer();

    //   ROS_INFO("Armor detection module has been connected!");

    //   armor_detection_goal_.command = 1;
    //   armor_detection_actionlib_client_.sendGoal(armor_detection_goal_,
    //                                              actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleDoneCallback(),
    //                                              actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleActiveCallback(),
    //                                              boost::bind(&Blackboard::ArmorDetectionFeedbackCallback, this, _1));
    // }


  }

  ~Blackboard() = default;


  // Enemy
  void ArmorDetectionFeedbackCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback){
    if (feedback->detected){
      enemy_detected_ = true;
      ROS_INFO("Find Enemy!");

      tf::Stamped<tf::Pose> tf_pose, global_tf_pose;
      geometry_msgs::PoseStamped camera_pose_msg, global_pose_msg;
      camera_pose_msg = feedback->enemy_pos;

      double distance = std::sqrt(camera_pose_msg.pose.position.x * camera_pose_msg.pose.position.x +
          camera_pose_msg.pose.position.y * camera_pose_msg.pose.position.y);
      double yaw = atan(camera_pose_msg.pose.position.y / camera_pose_msg.pose.position.x);

      //camera_pose_msg.pose.position.z=camera_pose_msg.pose.position.z;
      tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,
                                                              0,
                                                              yaw);
      camera_pose_msg.pose.orientation.w = quaternion.w();
      camera_pose_msg.pose.orientation.x = quaternion.x();
      camera_pose_msg.pose.orientation.y = quaternion.y();
      camera_pose_msg.pose.orientation.z = quaternion.z();
      poseStampedMsgToTF(camera_pose_msg, tf_pose);

      tf_pose.stamp_ = ros::Time(0);
      try
      {
        tf_ptr_->transformPose("map", tf_pose, global_tf_pose);
        tf::poseStampedTFToMsg(global_tf_pose, global_pose_msg);

        if(GetDistance(global_pose_msg, enemy_pose_)>0.2 || GetAngle(global_pose_msg, enemy_pose_) > 0.2){
          enemy_pose_ = global_pose_msg;

        }
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("tf error when transform enemy pose from camera to map");
      }
    } else{
      enemy_detected_ = false;
    }

  }

  void RefereeCB(const robot_base::RefereeRMUL::ConstPtr& _msg) {
    game_progress_ = _msg->game_progress;

    if(time_cycle_ && _msg->game_progress == 4) {
      game_progress_remain_ = _msg->game_progress_remain + 255;
    } else {
      game_progress_remain_ = _msg->game_progress_remain;
    }

    if(_msg->game_progress_remain == 0 && _msg->game_progress == 4) {
      time_cycle_ = false;
    }

    robot_id_ = _msg->robot_id;
    bullet_remain_ = _msg->bullet_remain;
    sentry_hp_ = _msg->sentry_hp;
  }

  void EnemyCB(const geometry_msgs::PoseStamped::ConstPtr& _msg) {
    enemy_pose_ = *_msg;
  }

  void EnemyDetectCB(const robot_base::AutoAimCmd::ConstPtr& _msg) {
    enemy_detected_ = _msg->armor_detected;
  }

  geometry_msgs::PoseStamped GetEnemy() const {
    return enemy_pose_;
  }

  bool IsEnemyDetected() const{
    // ROS_INFO("%s: %d", __FUNCTION__, (int)enemy_detected_);
    if((ros::Time::now() - enemy_pose_.header.stamp) < ros::Duration(1)) {
      return true;
    } else {
      return false;
    }
  }

  // Goal
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal){
    new_goal_ = true;
    goal_ = *goal;
  }

  geometry_msgs::PoseStamped GetGoal() const {
    return goal_;
  }

  bool IsNewGoal(){
    if(new_goal_){
      new_goal_ =  false;
      return true;
    } else{
      return false;
    }
  }

  // 比赛开始信号
  bool IsGameFirstStart() {
    static int last_game_progress {0};
    if (game_progress_ == 4 && last_game_progress < 4){
      last_game_progress = game_progress_;
      time_cycle_ = true;
      std::cout << "time_cycle_" << time_cycle_ << std::endl;
      std::cout << "game_progress_" << game_progress_ << std::endl;
      game_start_time_ = ros::Time::now();
      return true;
    }
    last_game_progress = game_progress_;
    return false;
  }

  int GetHP() {
    return sentry_hp_;
  }

  GameProgress GetGameProgress() {
    if(game_progress_ < 4) {
      return WAIT;
    } else if(game_progress_ == 4){
      return INPROGRESS;
    } else if(game_progress_ == 5){
      return END;
    }
  }

  // game_progress_remain_ 为裁判系统数据，五分钟比赛，计时方式：30s-0s + 250s-0s
  int GetGameRemain() {
    // auto remain_dur = ros::Time::now() - game_start_time_;
    // auto game_start_time_sec= game_start_time_.toSec();
    // auto nowsec= ros::Time::now().toSec(); // 此时间为正计时
    // auto remain_time = static_cast<int>(remain_dur.toSec());
    // return remain_time;
    return game_progress_remain_;
  }

  int GetBulletRemain() {
    return bullet_remain_;
  }

  // 判断是否受到攻击
  // previous 为一秒前数据，last 为两秒前数据。
  bool IsUnderHitting() {
    static auto last_time = ros::Time::now();
    static int last_hp = sentry_hp_;
    static auto previous_time = ros::Time::now();
    static int previous_hp = sentry_hp_;
    if(last_hp - sentry_hp_ > 20) {
      return true;
    } else {
      return false;
    }
    if(ros::Time::now() - last_time > ros::Duration(2)) {
      last_time = previous_time;
      last_hp = previous_hp;
    }
    if(ros::Time::now() - previous_time > ros::Duration(1)) {
      previous_time = ros::Time::now();
      previous_hp = sentry_hp_;
    }
  }

  /*---------------------------------- Tools ------------------------------------------*/

  double GetDistance(const geometry_msgs::PoseStamped &pose1,
                     const geometry_msgs::PoseStamped &pose2) {
    const geometry_msgs::Point point1 = pose1.pose.position;
    const geometry_msgs::Point point2 = pose2.pose.position;
    const double dx = point1.x - point2.x;
    const double dy = point1.y - point2.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double GetAngle(const geometry_msgs::PoseStamped &pose1,
                  const geometry_msgs::PoseStamped &pose2) {
    const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
    const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
    tf::Quaternion rot1, rot2;
    tf::quaternionMsgToTF(quaternion1, rot1);
    tf::quaternionMsgToTF(quaternion2, rot2);
    return rot1.angleShortestPath(rot2);
  }

  const geometry_msgs::PoseStamped GetRobotMapPose() {
    UpdateRobotPose();
    return robot_map_pose_;
  }

  const std::shared_ptr<CostMap> GetCostMap(){
    return costmap_ptr_;
  }

  const CostMap2D* GetCostMap2D() {
    return costmap_2d_;
  }

  const unsigned char* GetCharMap() {
    return charmap_;
  }

 private:
  void UpdateRobotPose() {
    tf::Stamped<tf::Pose> robot_tf_pose;
    robot_tf_pose.setIdentity();

    robot_tf_pose.frame_id_ = "base_link";
    robot_tf_pose.stamp_ = ros::Time();
    try {
      geometry_msgs::PoseStamped robot_pose;
      tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
      tf_ptr_->transformPose("map", robot_pose, robot_map_pose_);
    }
    catch (tf::LookupException &ex) {
      ROS_ERROR("Transform Error looking up robot pose: %s", ex.what());
    }
  }
  //! tf
  std::shared_ptr<tf::TransformListener> tf_ptr_;

  //! Enenmy detection
  ros::Subscriber enemy_sub_;

  //! Goal info
  geometry_msgs::PoseStamped goal_;
  bool new_goal_;

  //! Enemy info
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> armor_detection_actionlib_client_;
  ros::Subscriber autoaim_cmd_sub_;
  float gimbal_cmd_yaw;
  float gimbal_cmd_pitch;
  roborts_msgs::ArmorDetectionGoal armor_detection_goal_;
  geometry_msgs::PoseStamped enemy_pose_;
  bool enemy_detected_;  // 自瞄原始数据，波动较大

  //! cost map
  std::shared_ptr<CostMap> costmap_ptr_;
  CostMap2D* costmap_2d_;
  unsigned char* charmap_;

  //! robot map pose
  geometry_msgs::PoseStamped robot_map_pose_;

  // Game info
  ros::Subscriber referee_rmul_sub_;
  ros::Subscriber autoaim_enemy_sub_;
  ros::Subscriber autoaim_enemy_detect_sub_;
  ros::Time game_start_time_;
  // 裁判系统数据
  uint8_t game_progress_; // 比赛阶段
  uint16_t game_progress_remain_; 
  uint8_t robot_id_;
  uint16_t sentry_hp_;
  uint16_t bullet_remain_;

  bool time_cycle_{true}; // 裁判系统时间循环标志位，true 则为比赛开始前30秒阶段
  
};
} //namespace roborts_decision
#endif //ROBORTS_DECISION_BLACKBOARD_H
