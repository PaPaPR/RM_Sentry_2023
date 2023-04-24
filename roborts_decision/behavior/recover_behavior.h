#ifndef ROBORTS_DECISION_BACK_BOOT_AREA_BEHAVIOR_H
#define ROBORTS_DECISION_BACK_BOOT_AREA_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class RecoverBehavior {
 public:
  RecoverBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {


    recover_position_.header.frame_id = "map";
    recover_position_.pose.orientation.x = 0;
    recover_position_.pose.orientation.y = 0;
    recover_position_.pose.orientation.z = 0;
    recover_position_.pose.orientation.w = 1;

    recover_position_.pose.position.x = 8.0;
    recover_position_.pose.position.y = 12.0;
    recover_position_.pose.position.z = 0;

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

  }

  void Run() {

    auto executor_state = Update();

    if (executor_state != BehaviorState::RUNNING) {
      chassis_executor_->Execute(roborts_decision::SPINLOW);
      chassis_executor_->Execute(recover_position_);
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

    recover_position_.header.frame_id = "map";

    // recover_position_.pose.position.x = decision_config.master_bot().start_position().x();
    // recover_position_.pose.position.z = decision_config.master_bot().start_position().z();
    // recover_position_.pose.position.y = decision_config.master_bot().start_position().y();

    // auto master_quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.master_bot().start_position().roll(),
    //                                                                  decision_config.master_bot().start_position().pitch(),
    //                                                                  decision_config.master_bot().start_position().yaw());
    // recover_position_.pose.orientation = master_quaternion;

    return true;
  }

  ~RecoverBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! boot position
  geometry_msgs::PoseStamped recover_position_;

  //! chase buffer
  std::vector<geometry_msgs::PoseStamped> chase_buffer_;
  unsigned int chase_count_;

};
}


#endif //ROBORTS_DECISION_BACK_BOOT_AREA_BEHAVIOR_H
