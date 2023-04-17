#ifndef ROBORTS_DECISION_SEARCH_BEHAVIOR_H
#define ROBORTS_DECISION_SEARCH_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class SearchBehavior {
 public:
  SearchBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }
    SwitchSeachRegion(1);

  }

  void Run() {

    auto executor_state = Update();

    if (executor_state != BehaviorState::RUNNING) {
      if(region_iter_ == curent_region_->end()) {
        std::reverse(curent_region_->begin(), curent_region_->end());
        region_iter_ = curent_region_->begin();
      }
      auto search_goal = *region_iter_;
      region_iter_++;
      search_goal.header.frame_id = "map";
      search_goal.header.stamp = ros::Time::now();
      chassis_executor_->Execute(search_goal);
    }
  }

  // 切换搜索区域
  void SwitchSeachRegion(unsigned int _seach_index) {
    switch (_seach_index)
    {
    case 1:{
      if(curent_region_ != &search_region_1_) {
        region_iter_ = search_region_1_.begin();
        curent_region_ = &search_region_1_;
      }
      break;
    }
    case 2:{
      if(curent_region_ != &search_region_2_) {
        region_iter_ = search_region_2_.begin();
        curent_region_ = &search_region_2_;
      }
      break;
    }
    case 3:{
      if(curent_region_ != &search_region_3_) {
        region_iter_ = search_region_3_.begin();
        curent_region_ = &search_region_3_;
      }
      break;
    }
    case 4:{
      if(curent_region_ != &search_region_4_) {
        region_iter_ = search_region_4_.begin();
        curent_region_ = &search_region_4_;
      }
      break;
    }
    default:
      break;
    }
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }

    // may have more efficient way to search a region(enemy where disappear)
    search_region_.resize((unsigned int)(decision_config.search_region_1().size()));
    for (int i = 0; i != decision_config.search_region_1().size(); i++) {
      geometry_msgs::PoseStamped search_point;
      search_point.header.frame_id = "map";
      search_point.pose.position.x = decision_config.search_region_1(i).x();
      search_point.pose.position.y = decision_config.search_region_1(i).y();
      search_point.pose.position.z = decision_config.search_region_1(i).z();

      auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.search_region_1(i).roll(),
                                                                decision_config.search_region_1(i).pitch(),
                                                                decision_config.search_region_1(i).yaw());
      search_point.pose.orientation = quaternion;
      search_region_1_.push_back(search_point);
    }

    for (int i = 0; i != decision_config.search_region_2().size(); i++) {
      geometry_msgs::PoseStamped search_point;
      search_point.header.frame_id = "map";
      search_point.pose.position.x = decision_config.search_region_2(i).x();
      search_point.pose.position.y = decision_config.search_region_2(i).y();
      search_point.pose.position.z = decision_config.search_region_2(i).z();

      auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.search_region_2(i).roll(),
                                                                decision_config.search_region_2(i).pitch(),
                                                                decision_config.search_region_2(i).yaw());
      search_point.pose.orientation = quaternion;
      search_region_2_.push_back(search_point);
    }

    for (int i = 0; i != decision_config.search_region_3().size(); i++) {
      geometry_msgs::PoseStamped search_point;
      search_point.header.frame_id = "map";
      search_point.pose.position.x = decision_config.search_region_3(i).x();
      search_point.pose.position.y = decision_config.search_region_3(i).y();
      search_point.pose.position.z = decision_config.search_region_3(i).z();

      auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.search_region_3(i).roll(),
                                                                decision_config.search_region_3(i).pitch(),
                                                                decision_config.search_region_3(i).yaw());
      search_point.pose.orientation = quaternion;
      search_region_3_.push_back(search_point);
    }

    for (int i = 0; i != decision_config.search_region_4().size(); i++) {
      geometry_msgs::PoseStamped search_point;
      search_point.header.frame_id = "map";
      search_point.pose.position.x = decision_config.search_region_4(i).x();
      search_point.pose.position.y = decision_config.search_region_4(i).y();
      search_point.pose.position.z = decision_config.search_region_4(i).z();

      auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.search_region_4(i).roll(),
                                                                decision_config.search_region_4(i).pitch(),
                                                                decision_config.search_region_4(i).yaw());
      search_point.pose.orientation = quaternion;
      search_region_4_.push_back(search_point);
    }

    for (int i = 0; i != decision_config.search_region_5().size(); i++) {
      geometry_msgs::PoseStamped search_point;
      search_point.header.frame_id = "map";
      search_point.pose.position.x = decision_config.search_region_5(i).x();
      search_point.pose.position.y = decision_config.search_region_5(i).y();
      search_point.pose.position.z = decision_config.search_region_5(i).z();

      auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.search_region_5(i).roll(),
                                                                decision_config.search_region_5(i).pitch(),
                                                                decision_config.search_region_5(i).yaw());
      search_point.pose.orientation = quaternion;
      search_region_5_.push_back(search_point);
    }
    return true;
  }

  // void SetLastPosition(geometry_msgs::PoseStamped last_position) {
  //   last_position_ = last_position;
  //   search_count_ = 5;
  // }

  ~SearchBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! chase goal
  // geometry_msgs::PoseStamped last_position_;

  //! search buffer
  std::vector<geometry_msgs::PoseStamped> search_region_1_;
  std::vector<geometry_msgs::PoseStamped> search_region_2_;
  std::vector<geometry_msgs::PoseStamped> search_region_3_;
  std::vector<geometry_msgs::PoseStamped> search_region_4_;
  std::vector<geometry_msgs::PoseStamped> search_region_5_;
  std::vector<geometry_msgs::PoseStamped> search_region_;
  std::vector<geometry_msgs::PoseStamped>::iterator region_iter_;
  std::vector<geometry_msgs::PoseStamped>* curent_region_;
  // unsigned int search_index_;

};
}

#endif //ROBORTS_DECISION_SEARCH_BEHAVIOR_H
