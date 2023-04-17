#include <ros/ros.h>

#include "executor/chassis_executor.h"
#include "executor/gimbal_executor.h"

#include "example_behavior/goal_behavior.h"

#include "behavior/chase_behavior.h"
#include "behavior/search_behavior.h"

void Command();
char command = '0';

int main(int argc, char **argv) {
  ros::init(argc, argv, "behavior_test_node");
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto gimbal_executor = new roborts_decision::GimbalExecutor; // for testing
  auto blackboard = new roborts_decision::Blackboard(full_path);

  roborts_decision::ChaseBehavior        chase_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::SearchBehavior       search_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::GoalBehavior       goal_behavior(chassis_executor, blackboard);

  auto command_thread= std::thread(Command);
  ros::Rate rate(3);
  while(ros::ok()){
    ros::spinOnce();
    switch (command) {
      case '1':{
        if(blackboard->IsEnemyDetected()) {
          if(chase_behavior.Run()) {
            ROS_INFO("attack plan succeed.");
          } else {
            ROS_INFO("attack plan failed.");
          }
        }
        break;
      }
      case '2':{
        search_behavior.Run();
        break;
      }
      case '3':{
        goal_behavior.Run();
        break;
      }
      case '4':{
        // goal_behavior.Run();
        break;
      }
      case '5':{
        chassis_executor->Execute(roborts_decision::NORMAL);
        break;
      }
      case '6':{
        chassis_executor->Execute(roborts_decision::SPINFAST);
        break;
      }
      case '7':{
        search_behavior.SwitchSeachRegion(1);
        break;
      }
      case '8':{
        chase_behavior.Cancel();
        break;
      }
      case '9':{
        break;
      }
      case 27:
        if (command_thread.joinable()){
          command_thread.join();
        }
        return 0;
      default:
        break;
    }
    rate.sleep();
  }


  return 0;
}

void Command() {

  while (command != 27) {
    std::cout << "decision ok!" << std::endl;
    std::cin >> command;
  }
}

