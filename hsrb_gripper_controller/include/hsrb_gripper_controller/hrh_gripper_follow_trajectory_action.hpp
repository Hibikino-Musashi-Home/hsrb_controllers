/*
Copyright (c) 2016 TOYOTA MOTOR CORPORATION
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#ifndef HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_FOLLOW_TRAJECTORY_ACTION_HPP_
#define HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_FOLLOW_TRAJECTORY_ACTION_HPP_
#include <limits>
#include <memory>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <joint_trajectory_controller/trajectory.hpp>

#include "hsrb_gripper_controller/hrh_gripper_action.hpp"

namespace hsrb_gripper_controller {

/// @class HrhGripperFollowTrajectoryAction
/// @brief HRH orbital tracking action class
class HrhGripperFollowTrajectoryAction : public HrhGripperAction<control_msgs::action::FollowJointTrajectory> {
 public:
  /// constructor
  /// @param [IN] Controller parent controller
  explicit HrhGripperFollowTrajectoryAction(HrhGripperController* controller);
  virtual ~HrhGripperFollowTrajectoryAction() = default;

  bool Activate() override;

  void Update(const rclcpp::Time& time) override;

  void PreemptActiveGoal() override;

 protected:
  /// Implementation of initialization of action
  bool InitImpl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) override;
  /// Check if the goal is acceptable
  bool ValidateGoal(const control_msgs::action::FollowJointTrajectory::Goal& goal) override;
  /// Update action goals
  void UpdateActionImpl(const control_msgs::action::FollowJointTrajectory::Goal& goal) override;

  /// Tolerance error in the default goal position [RAD]
  double default_goal_tolerance_;
  /// Tolerance error of default goal arrival time [S]
  double default_goal_time_tolerance_;

  // Whether to connect from the existing desired when a new orbit comes
  // Variables and behaviors are tailored to JointTrajectoryController
  bool open_loop_control_;
  // Finally sampled state
  trajectory_msgs::msg::JointTrajectoryPoint last_command_state_;

  /// Callback when the orbit command arrives at the topic
  /// @param [in] MSG track
  void TrajectoryCommandCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  /// Track command reception
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_command_sub_;

  /// Hold the orbit
  std::shared_ptr<joint_trajectory_controller::Trajectory>* trajectory_active_ptr_;
  std::shared_ptr<joint_trajectory_controller::Trajectory> trajectory_ptr_;
  realtime_tools::RealtimeBuffer<trajectory_msgs::msg::JointTrajectory::SharedPtr> trajectory_msg_buffer_;

  /// @struct GoalCondition
  /// @brief Goal conditions
  struct GoalCondition {
    /// Instruction location [RAD]
    double position;
    /// Objective time
    rclcpp::Time expected_arrival_time;
    /// Time to stop tracking orbit
    rclcpp::Time abort_time;
    /// Tolerance error in the goal position
    double goal_tolerance;
  };
  /// Goal condition buffer
  realtime_tools::RealtimeBuffer<GoalCondition> goal_condition_buffer_;
};

}  // namespace hsrb_gripper_controller

#endif  // HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_FOLLOW_TRAJECTORY_ACTION_HPP_
