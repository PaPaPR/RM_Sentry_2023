#ifndef ROBORTS_DECISION_CHASE_BEHAVIOR_H
#define ROBORTS_DECISION_CHASE_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

#include "nodelet/nodelet.h"

namespace roborts_decision {
class ChaseBehavior {
 public:
  ChaseBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {


    chase_goal_.header.frame_id = "map";
    chase_goal_.pose.orientation.x = 0;
    chase_goal_.pose.orientation.y = 0;
    chase_goal_.pose.orientation.z = 0;
    chase_goal_.pose.orientation.w = 1;

    chase_goal_.pose.position.x = 0;
    chase_goal_.pose.position.y = 0;
    chase_goal_.pose.position.z = 0;

    // chase_buffer_.resize(2);
    // chase_count_ = 0;

    if(debug) {
    }
  }

  // 如果规划失败，且底盘无动作，则返回 false，其他情况为 true
  // 对战保持距离：1.5m，钟摆范围：，角度：30
  // 计算自己与敌人所在直线上，从敌人出发，以对战保持距离的线段的末尾坐标 end_pose。
  // 使用圆的参数方程，敌人作为圆心，线段末尾坐标作为 xy，多阶段计算出角度
  // 与上次摆动方向相反，加上钟摆角度，计算坐标。
  // 检查坐标是否在地图内。
  // 检查敌人到坐标所在线段上每一点的代价，代价过大即终止，如果最终点距离小于对战距离一半，舍弃。
  // 如果钟摆角度过小，则开启高速陀螺。
  bool Run() {
    if(!blackboard_->IsEnemyDetected()) {
      return false;
    }
    double attack_distance_max {2.3};
    double attack_distance_min {1.5};
    int count_slice {3}; // 角度切分判断次数

    auto enemy_pose = blackboard_->GetEnemy();
    auto robot_pose = blackboard_->GetRobotMapPose();
    auto dx = enemy_pose.pose.position.x - robot_pose.pose.position.x;
    auto dy = enemy_pose.pose.position.y - robot_pose.pose.position.y;
    double distance = sqrt(dx * dx + dy * dy);
    double cos_end = -dx / distance;
    double sin_end = -dy / distance;
    auto goal_start_pose = robot_pose;
    auto goal_end_pose = robot_pose;
    auto goal_pose = robot_pose;
    goal_end_pose.pose.position.x = attack_distance_max * cos_end + enemy_pose.pose.position.x;
    goal_end_pose.pose.position.y = attack_distance_max * sin_end + enemy_pose.pose.position.y;
    // 圆的参数方程
    double circle_rcos = goal_end_pose.pose.position.x - enemy_pose.pose.position.x;
    double circle_rsin = goal_end_pose.pose.position.y - enemy_pose.pose.position.y;
    if(circle_rcos != 0. && circle_rsin != 0.) {
      if(circle_rcos > 0. && circle_rsin > 0.) circle_angle = atan(circle_rsin / circle_rcos) * 180. / 3.14159265;
      if(circle_rcos < 0. && circle_rsin > 0.) circle_angle = atan(circle_rsin / circle_rcos) * 180. / 3.14159265 + 180.;
      if(circle_rcos < 0. && circle_rsin < 0.) circle_angle = atan(circle_rsin / circle_rcos) * 180. / 3.14159265 + 180.;
      if(circle_rcos > 0. && circle_rsin < 0.) circle_angle = atan(circle_rsin / circle_rcos) * 180. / 3.14159265 + 360.;
    }
    bool interrupt {false};
    goal_angle = circle_angle;
    double goal_x = goal_pose.pose.position.x;
    double goal_y = goal_pose.pose.position.y;
    double goal_start_x = goal_pose.pose.position.x;
    double goal_start_y = goal_pose.pose.position.y;
    double goal_end_x = goal_pose.pose.position.x;
    double goal_end_y = goal_pose.pose.position.y;
    for (size_t i = 0; i < count_slice && !interrupt; i++) {
      goal_angle += pendulum_angle / count_slice;
      goal_start_x = enemy_pose.pose.position.x + attack_distance_min * cos(goal_angle * 3.14159265 / 180.);
      goal_start_y = enemy_pose.pose.position.y + attack_distance_min * sin(goal_angle * 3.14159265 / 180.);
      goal_end_x = enemy_pose.pose.position.x + attack_distance_max * cos(goal_angle * 3.14159265 / 180.);
      goal_end_y = enemy_pose.pose.position.y + attack_distance_max * sin(goal_angle * 3.14159265 / 180.);
      unsigned int goal_start_cell_x, goal_start_cell_y; // 目标最近点在地图上的栅格坐标
      unsigned int goal_end_cell_x, goal_end_cell_y; // 目标最远点在地图上的栅格坐标

      // 将临时目标的世界坐标转换为地图栅格坐标，并判断是否成功
      auto get_goal_start_cell = blackboard_->GetCostMap2D()->World2Map(goal_start_x, goal_start_y,
                                                                  goal_start_cell_x, goal_start_cell_y);
      auto get_goal_end_cell = blackboard_->GetCostMap2D()->World2Map(goal_end_x, goal_end_y,
                                                                  goal_end_cell_x, goal_end_cell_y);
      if((!get_goal_start_cell) && (!get_goal_end_cell)) {
        interrupt = true;
      }
      // 快速直线迭代器，从敌人所在的栅格开始，沿着目标点和敌人连线的反方向遍历每个栅格
      for (FastLineIterator line(goal_end_cell_x, goal_end_cell_y, goal_start_cell_x, goal_start_cell_y); line.IsValid() && !interrupt; line.Advance()) {
        auto point_cost = blackboard_->GetCostMap2D()->GetCost((unsigned int) (line.GetX()), (unsigned int) (line.GetY()));
        if(point_cost < 100) {
          blackboard_->GetCostMap2D()->Map2World((unsigned int) (line.GetX()),
                                                 (unsigned int) (line.GetY()),
                                                 goal_x,
                                                 goal_y);
        }
      }
      // std::cout << "----------" << std::endl;
      // std::cout << "circle_angle:" << circle_angle << std::endl;
      // std::cout << "count_slice:" << count_slice << std::endl;
      // std::cout << "goal_angle:" << goal_angle << std::endl;
      // std::cout << "goal_x:" << goal_x << std::endl;
      // std::cout << "goal_y:" << goal_y << std::endl;
      // std::cout << "enemy_x:" << enemy_pose.pose.position.x << std::endl;
      // std::cout << "enemy_y:" << enemy_pose.pose.position.y << std::endl;
      // std::cout << "cos_goal_angle:" << cos(goal_angle * 3.14159265 / 180.) << std::endl;
      // std::cout << "sin_goal_angle:" << sin(goal_angle * 3.14159265 / 180.) << std::endl;
    }
    pendulum_angle = -pendulum_angle;
    // 如果目标在敌方基地附近则返回失败
    if (goal_x > 1.8 && goal_y < 4.7) {
      return false;
    }
    // 规划失败，如果底盘正在运行则返回成功，否则返回失败，使用其他行为。
    if(interrupt || goal_x == robot_pose.pose.position.x) {
      auto executor_state = Update();
      if (executor_state != BehaviorState::RUNNING) {
        return false;
      }
    } else {
      // 规划成功
      auto executor_state = Update();
      if (executor_state != BehaviorState::RUNNING) {
        // 平移角度过小，开启快速自旋。
        // if(-pendulum_angle * 0.5 < (circle_angle - goal_angle) && (circle_angle - goal_angle) < pendulum_angle * 0.5) {
        //   chassis_executor_->Execute(roborts_decision::SPINFAST);
        // } else {
        //   // 低速自旋
        //   chassis_executor_->Execute(roborts_decision::SPINLOW);
        // }
        chassis_executor_->Execute(roborts_decision::SPINFAST);
        goal_pose.header.stamp = ros::Time::now();
        goal_pose.pose.position.x = goal_x;
        goal_pose.pose.position.y = goal_y;
        chassis_executor_->Execute(goal_pose);
      }
    }
    return true;
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  void SetGoal(geometry_msgs::PoseStamped chase_goal) {
    chase_goal_ = chase_goal;
  }

  ~ChaseBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! chase goal
  geometry_msgs::PoseStamped chase_goal_;

  //! chase buffer
  // std::vector<geometry_msgs::PoseStamped> chase_buffer_;
  // unsigned int chase_count_;

  //! cancel flag
  bool cancel_goal_;

  double pendulum_angle {40.};
  double circle_angle {0.}; // 机器人目前所在圆的角度
  double goal_angle {0.};   // 目的地所在圆的角度

  bool debug {true};
};
}

#endif //ROBORTS_DECISION_CHASE_BEHAVIOR_H
