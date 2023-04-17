#ifndef ROBORTS_DECISION_CHASE_BEHAVIOR_H
#define ROBORTS_DECISION_CHASE_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

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

    chase_buffer_.resize(2);
    chase_count_ = 0;

    cancel_goal_ = true;
  }

  void Run() {

    auto executor_state = Update();

    auto robot_map_pose = blackboard_->GetRobotMapPose();
    if (executor_state != BehaviorState::RUNNING) {

      // 用一个数组和一个计数器来记录最近两次敌人的位置信息
      chase_buffer_[chase_count_++ % 2] = blackboard_->GetEnemy();

      // 计数器取模，保证在0和1之间循环
      chase_count_ = chase_count_ % 2;

      // 计算敌人和机器人之间的距离和角度
      auto dx = chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.x - robot_map_pose.pose.position.x;
      auto dy = chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.y - robot_map_pose.pose.position.y;
      auto yaw = std::atan2(dy, dx);

      // 如果距离在1到2米之间，说明机器人已经靠近敌人，不需要再追击
      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) >= 1.0 && std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 2.0) {
        if (cancel_goal_) {
          chassis_executor_->Cancel();
          cancel_goal_ = false;
        }
        return;

      } else {

        auto orientation = tf::createQuaternionMsgFromYaw(yaw);
        // 定义一个目标位置变量，用来存储机器人需要移动到的位置
        geometry_msgs::PoseStamped reduce_goal;
        reduce_goal.pose.orientation = robot_map_pose.pose.orientation;

        reduce_goal.header.frame_id = "map";
        reduce_goal.header.stamp = ros::Time::now();
        // 目标位置的坐标是敌人位置减去一个固定距离（1.2米），这样可以保证机器人不会撞到敌人
        reduce_goal.pose.position.x = chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.x - 1.2 * cos(yaw);
        reduce_goal.pose.position.y = chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.y - 1.2 * sin(yaw);
        // 获取敌人在地图上的坐标
        auto enemy_x = reduce_goal.pose.position.x;
        auto enemy_y = reduce_goal.pose.position.y;
        // 设置目标位置的高度为1米（这个可能是为了避免地形影响）
        reduce_goal.pose.position.z = 1;
        // 定义两个变量，用来存储敌人在地图上的栅格坐标
        unsigned int goal_cell_x, goal_cell_y;

        // 如果需要的话，加上互斥锁，防止对地图数据的并发访问
        // if necessary add mutex lock
        //blackboard_->GetCostMap2D()->GetMutex()->lock();
        auto get_enemy_cell = blackboard_->GetCostMap2D()->World2Map(enemy_x,
                                                                   enemy_y,
                                                                   goal_cell_x,
                                                                   goal_cell_y);
        //blackboard_->GetCostMap2D()->GetMutex()->unlock();

        // 如果转换失败，说明敌人不在地图范围内，返回，不执行后面的代码
        if (!get_enemy_cell) {
          return;
        }

        // 获取机器人在地图上的位置和栅格坐标
        auto robot_x = robot_map_pose.pose.position.x;
        auto robot_y = robot_map_pose.pose.position.y;
        unsigned int robot_cell_x, robot_cell_y;
        double goal_x, goal_y;
        blackboard_->GetCostMap2D()->World2Map(robot_x,
                                              robot_y,
                                              robot_cell_x,
                                              robot_cell_y);

        // 如果敌人所在的栅格的代价值大于等于253，说明敌人处于障碍物或者未知区域，机器人不能直接移动过去
        if (blackboard_->GetCostMap2D()->GetCost(goal_cell_x, goal_cell_y) >= 253) {

          // 定义一个布尔变量，用来标记是否找到了合适的目标位置
          bool find_goal = false;
          // 用一个快速直线迭代器，从敌人所在的栅格开始，沿着机器人和敌人连线的反方向遍历每个栅格
          for(FastLineIterator line( goal_cell_x, goal_cell_y, robot_cell_x, robot_cell_x); line.IsValid(); line.Advance()) {

            // 获取当前栅格的代价值
            auto point_cost = blackboard_->GetCostMap2D()->GetCost((unsigned int) (line.GetX()), (unsigned int) (line.GetY())); //current point's cost

            // 如果代价值大于等于253，说明当前栅格也是障碍物或者未知区域，跳过，继续遍历下一个栅格
            if(point_cost >= 253){
              continue;

            } else {
              // 否则，说明当前栅格是可以到达的，将布尔变量设为真，将当前栅格的坐标转换为世界坐标，并赋值给目标位置变量
              find_goal = true;
              blackboard_->GetCostMap2D()->Map2World((unsigned int) (line.GetX()),
                                                     (unsigned int) (line.GetY()),
                                                     goal_x,
                                                     goal_y);

              reduce_goal.pose.position.x = goal_x;
              reduce_goal.pose.position.y = goal_y;
              // 跳出循环，不再遍历其他栅格
              break;
            }

          }
          // 如果找到了合适的目标位置，就将目标位置发送给底盘执行器，并将取消目标的标志设为真
          if (find_goal) {
            cancel_goal_ = true;
            chassis_executor_->Execute(reduce_goal);
          } else {
            // 如果没有找到合适的目标位置，就取消之前发送给底盘执行器的目标（如果有的话），并将取消目标的标志设为假
            if (cancel_goal_) {
              chassis_executor_->Cancel();
              cancel_goal_ = false;
            }
            return;
          }

        } else {
          // 如果敌人所在的栅格的代价值小于253，说明敌人处于可以到达的区域，就直接将目标位置发送给底盘执行器，并将取消目标的标志设为真
          cancel_goal_ = true;
          chassis_executor_->Execute(reduce_goal);
        }
      }
    }
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
  std::vector<geometry_msgs::PoseStamped> chase_buffer_;
  unsigned int chase_count_;

  //! cancel flag
  bool cancel_goal_;
};
}

#endif //ROBORTS_DECISION_CHASE_BEHAVIOR_H
