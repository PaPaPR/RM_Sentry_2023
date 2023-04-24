#include <ros/ros.h>

#include "executor/chassis_executor.h"
// #include "executor/gimbal_executor.h"

#include "behavior/chase_behavior.h"
#include "behavior/search_behavior.h"
#include "behavior/recover_behavior.h"

#include "robot_base/UpdateMapTF.h"
#include "robot_base/AutoAimWorking.h"

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
  auto autoaim_working_client = nh.serviceClient<robot_base::AutoAimWorking>("autoaim_working");

  bool debug {false};
  ros::NodeHandle nhp("~");
  nhp.param<bool>("debug", debug, false);

  RobotBehavior last_state{IDLE};
  RobotBehavior cur_state {IDLE};

  geometry_msgs::PoseStamped recover_pose;
  recover_pose.pose.position.x = 8.0;
  recover_pose.pose.position.y = 12.0;
  recover_pose.pose.position.z = 0.;
  recover_pose.pose.orientation.x = 0.;
  recover_pose.pose.orientation.y = 0.;
  recover_pose.pose.orientation.z = 0.;
  recover_pose.pose.orientation.w = 1.;

  roborts_decision::ChaseBehavior        chase_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::SearchBehavior       search_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::RecoverBehavior      recover_behavior(chassis_executor, blackboard, full_path);
  // 击打模式可以打断任何模式

  ros::Rate rate(10);
  if(debug) {
    rate = ros::Rate(2); // 调试用
  }

  while(ros::ok()) {
    ros::spinOnce();
    // 比赛开始，重置定位
    if(blackboard->IsGameFirstStart()) {
      robot_base::UpdateMapTF srv;
      client.call(srv);
      robot_base::AutoAimWorking autoaim_req;
      autoaim_req.request.working = true;
      autoaim_working_client.call(autoaim_req);
      ROS_INFO("GameStart");
      chassis_executor->Execute(roborts_decision::SPINFAST); // 默认开启小陀螺
    }
    ROS_INFO("game progress: %d, game remian time: %d, remain hp: %d, remain bullet: %d",
      blackboard->GetGameProgress(), blackboard->GetGameRemain(),
      blackboard->GetHP(), blackboard->GetBulletRemain());
    if(blackboard->GetGameProgress() == roborts_decision::INPROGRESS) {
      // 比赛开始前一分钟：堵对方补给点门口，根据血量选择距离
      // 比赛一分钟至四分钟：占领增益区
      // 最后一分钟血量或者弹量低于150：守家
      // 追击：敌人家门口不允许

      if(blackboard->GetGameRemain() > 240) {
        // 巡逻对方补给区(远）：血量大于300，且比赛过去不到一分钟。
        if(blackboard->GetHP() > 450) {
          cur_state = OFFEND_FAR;
          ROS_INFO("OFFEND_FAR");
        }
        if(blackboard->GetHP() < 450 && blackboard->GetHP() > 300) {
          cur_state = OFFEND_FAR;
          ROS_INFO("OFFEND_FAR");
        }
        // 血量小于300，补血
        if(blackboard->GetHP() < 300) {
          cur_state = RECOVER;
          ROS_INFO("RECOVER");
        }
      }
      // 巡逻增益：比赛过去一分钟，距离结束超过一分钟
      if(blackboard->GetGameRemain() < 240 && blackboard->GetGameRemain() > 60) {
        cur_state = SUPPLY;
        ROS_INFO("SUPPLY");
        if(blackboard->GetHP() < 300) {
          cur_state = RECOVER;
          ROS_INFO("RECOVER");
        }
      }
      if(blackboard->GetGameRemain() < 60) {
        cur_state = GUARD;
        ROS_INFO("GUARD");
      }
      // 弹量低于150与血量低于200（补不到血）：守家
      if(blackboard->GetBulletRemain() < 150 || blackboard->GetHP() < 200) {
        cur_state = GUARD;
        ROS_INFO("GUARD");
      }
      // 侦测到敌人且钟摆规划成功，则钟摆
      // 侦测到敌人但无法钟摆（被逼墙角），则继续巡逻
      if(blackboard->IsEnemyDetected()) {
        // 1.8, 4.7 为敌人基地附近
        cur_state = ATTACK;
        ROS_INFO("ATTACK");
        // 如果在敌人家附近则不进行
        auto pose = blackboard->GetRobotMapPose();
        if (pose.pose.position.x > 1.8 && pose.pose.position.y < 4.7) {
          cur_state = last_state;
        }
        // 补血模式不钟摆
        if (last_state == RECOVER) {
          cur_state = last_state;
        }
      }
      // 补血模式补到 500 才离开
      // 可能卡住，建议加上行为执行时间，超时切换。
      // if (last_state == RECOVER && blackboard->GetHP() < 500) {
      //   cur_state = RECOVER;
      // }
    }
    ROS_INFO("status now: %d, status last: %d", cur_state, last_state);

    if(debug) {
      chassis_executor->Cancel(); // 调试用
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
        last_state = RECOVER;
      }
      recover_behavior.Run();
      break;
    }
    case RUN: {
      if(last_state != cur_state) {
        chassis_executor->Cancel();
        search_behavior.SwitchSeachRegion(5);
        last_state = RUN;
      }
      search_behavior.Run();
      break;
    }
    case ATTACK: {
      // 如果击打无法规划（被逼墙角，则继续巡逻）
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