#include <ros/ros.h>

#include "executor/chassis_executor.h"
// #include "executor/gimbal_executor.h"

#include "behavior/chase_behavior.h"
#include "behavior/search_behavior.h"

#include "robot_base/UpdateMapTF.h"

enum RobotBehavior {
  IDLE,    // 无状态
  OFFEND,  // 堵敌人补给点门口（较近）
  OFFEND_FAR,  // 堵敌人补给点门口（较远）
  SUPPLY,  // 占领增益点
  GUARD,   // 看家
  RECOVER, // 最后一分钟补血
  RUN,     // 最后一分钟满血之后跑全图
  ATTACK,  // 遇敌钟摆
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "behavior_test_node");
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto blackboard = new roborts_decision::Blackboard(full_path);

  ros::NodeHandle nh;
  auto client = nh.serviceClient<robot_base::UpdateMapTF>("update_map");

  RobotBehavior last_state{IDLE};
  RobotBehavior cur_state {IDLE};

  geometry_msgs::PoseStamped recover_pose;
  recover_pose.pose.position.x = 7.8;
  recover_pose.pose.position.y = 11.8;
  recover_pose.pose.position.z = 0.;
  recover_pose.pose.orientation.x = 0.;
  recover_pose.pose.orientation.y = 0.;
  recover_pose.pose.orientation.z = 0.;
  recover_pose.pose.orientation.w = 1.;

  roborts_decision::ChaseBehavior        chase_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::SearchBehavior       search_behavior(chassis_executor, blackboard, full_path);
  // 击打模式可以打断任何模式

  // ros::Rate rate(10);
  ros::Rate rate(2); // 调试用
  while(ros::ok()) {
    ros::spinOnce();
    // 比赛开始，重置定位
    if(blackboard->IsGameFirstStart()) {
      robot_base::UpdateMapTF srv;
      client.call(srv);
      ROS_INFO("GameStart");
    }
    ROS_INFO("game progress: %d, remain hp: %d, remain bullet: %d", blackboard->GetGameProgress(),
      blackboard->GetHP(), blackboard->GetBulletRemain());

    if(blackboard->GetGameProgress() == roborts_decision::INPROGRESS) {
      // 巡逻对方补给区(近）：血量高于450，且比赛过去不到一分钟。
      if(blackboard->GetHP() > 450 &&
        blackboard->GetGameRemain() > 240) {
        cur_state = OFFEND;
        ROS_INFO("OFFEND");
      }
      // 巡逻对方补给区(远）：血量低于450，且比赛过去不到一分钟。
      if(blackboard->GetHP() < 450 &&
        blackboard->GetGameRemain() > 240) {
        cur_state = OFFEND_FAR;
        ROS_INFO("OFFEND_FAR");
      }
      // 巡逻增益：比赛过去一分钟，距离结束超过一分钟。
      if(blackboard->GetGameRemain() < 240 && blackboard->GetGameRemain() > 60) {
        cur_state = SUPPLY;
        ROS_INFO("SUPPLY");
      }
      // 巡逻家门口：子弹少于150或者血量少于150，距离比赛结束大于一分钟。
      if((blackboard->GetHP() < 150 ||
        blackboard->GetBulletRemain() < 150) &&
        blackboard->GetGameRemain() > 60) {
        cur_state = GUARD;
        ROS_INFO("GUARD");
      }
      // 加血：比赛剩余一分钟，如果血量太低，则加血，剩余不足 45 秒则退出。
      if(blackboard->GetHP() < 300 &&
        blackboard->GetGameRemain() < 60 &&
        blackboard->GetGameRemain() > 45) {
        cur_state = RECOVER;
        ROS_INFO("RECOVER");
      }
      // 在 60-45 秒内加满血或者血没有加满，剩下 45 秒则直接全图跑
      if(blackboard->GetGameRemain() < 45 ||
         (blackboard->GetHP() > 500 &&
         blackboard->GetGameRemain() < 60)) {
        cur_state = RUN;
        ROS_INFO("RUN");
      }
      // 侦测到敌人且钟摆规划成功，则钟摆
      // 侦测到敌人但无法钟摆（被逼墙角），则继续巡逻
      if(blackboard->IsEnemyDetected()) {
        cur_state = ATTACK;
        ROS_INFO("ATTACK");
      }
    }

    switch (cur_state)
    {
    case OFFEND: {
      if(last_state != cur_state) {
        chassis_executor->Cancel();
        search_behavior.SwitchSeachRegion(1);
        last_state = OFFEND;
      }
      search_behavior.Run();
      break;
    }
    case OFFEND_FAR: {
      if(last_state != cur_state) {
        chassis_executor->Cancel();
        search_behavior.SwitchSeachRegion(2);
        last_state = OFFEND_FAR;
      }
      search_behavior.Run();
      break;
    }
    case SUPPLY: {
      if(last_state != cur_state) {
        chassis_executor->Cancel();
        search_behavior.SwitchSeachRegion(3);
        last_state = SUPPLY;
      }
      search_behavior.Run();
      break;
    }
    case GUARD: {
      if(last_state != cur_state) {
        chassis_executor->Cancel();
        search_behavior.SwitchSeachRegion(4);
        last_state = GUARD;
      }
      search_behavior.Run();
      break;
    }
    case RECOVER: {
      if(last_state != cur_state) {
        chassis_executor->Cancel();
        geometry_msgs::PoseStamped recover_pose;
        recover_pose.header.stamp = ros::Time::now();
        chassis_executor->Execute(recover_pose);
        last_state = RECOVER;
      }
      break;
    }
    case RUN: {
      // to-do:前往补给区
      if(last_state != cur_state) {
        chassis_executor->Cancel();
        search_behavior.SwitchSeachRegion(5);
        last_state = RUN;
      }
      search_behavior.Run();
      break;
    }
    case ATTACK: {
      // to-do:前往补给区
      if(last_state != cur_state) {
        chassis_executor->Cancel();
        last_state = ATTACK;
      }
      if(!chase_behavior.Run()){
        search_behavior.Run();
      }
      break;
    }
    default:
      break;
    }

    rate.sleep();
  }


  return 0;
}