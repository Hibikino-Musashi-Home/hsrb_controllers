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
#include <optional>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <joint_trajectory_controller/trajectory.hpp>

#include "hsrb_gripper_controller/hrh_gripper_action.hpp"

namespace hsrb_gripper_controller {

/// @class HrhGripperFollowTrajectoryAction
/// @brief Hrh Orbit Following Action Class
class HrhGripperFollowTrajectoryAction : public HrhGripperAction<control_msgs::action::FollowJointTrajectory> {
 public:
  /// Constructor
  /// @param [in] controller Parent Controller
  explicit HrhGripperFollowTrajectoryAction(HrhGripperController* controller);
  virtual ~HrhGripperFollowTrajectoryAction() = default;

  void Update(const rclcpp::Time& time) override;

  void PreemptActiveGoal() override;

  trajectory_msgs::msg::JointTrajectoryPoint GetReferenceState() override;
  trajectory_msgs::msg::JointTrajectoryPoint GetFeedbackState() override;

 protected:
  /// Implementation of Action Initialization
  bool InitImpl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) override;
  /// Check if the goal is acceptable
  bool ValidateGoal(const control_msgs::action::FollowJointTrajectory::Goal& goal) override;
  /// Update the action's target
  void UpdateActionImpl(const control_msgs::action::FollowJointTrajectory::Goal& goal) override;
  /// Get the current control target joint position
  /// @return Joint Position
  double GetPosition() const;

  /// Default Allowable Error for Goal Position [rad]
  double default_goal_tolerance_;
  /// Default Allowable Error for Goal Arrival Time [s]
  double default_goal_time_tolerance_;

  // Whether to connect from the existing desired when a new orbit arrives
  // Variable names and behavior are aligned with JointTrajectoryController
  bool open_loop_control_;
  /// Flag for whether to perform control with an output axis corrected for spring amount
  bool do_output_position_control_;
  /// Lower limit of current when closing [A], if 0.0 or higher, no correction of command value due to overcurrent
  double current_min_;
  /// Step width of correction value to increase (open) command value in case of overcurrent when closing
  double position_correction_incresing_step_;
  /// Step width to return (decrease) correction value when not in overcurrent, more stable if smaller than increasing
  double position_correction_decresing_step_;
  /// Correction value to the command value
  double position_correction_value_;

  // Last sampled state
  rclcpp::Time last_sampled_time_;
  std::optional<trajectory_msgs::msg::JointTrajectoryPoint> last_command_state_;

  /// Callback when trajectory command arrives via topic
  /// @param [in] msg Trajectory
  void TrajectoryCommandCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  /// Trajectory Command Reception
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_command_sub_;

  /// Hold the trajectory
  std::shared_ptr<joint_trajectory_controller::Trajectory>* trajectory_active_ptr_;
  std::shared_ptr<joint_trajectory_controller::Trajectory> trajectory_ptr_;
  realtime_tools::RealtimeBuffer<trajectory_msgs::msg::JointTrajectory::SharedPtr> trajectory_msg_buffer_;

  /// @struct GoalCondition
  /// @brief Goal Conditions
  struct GoalCondition {
    /// Command Position [rad]
    double position;
    /// Target Arrival Time
    rclcpp::Time expected_arrival_time;
    /// Time to stop orbit following
    rclcpp::Time abort_time;
    /// Allowable Error for Goal Arrival Time
    rclcpp::Time goal_time_tolerance;
    /// Allowable Error for Goal Position
    double goal_tolerance;
  };
  /// Buffer for Goal Conditions
  realtime_tools::RealtimeBuffer<GoalCondition> goal_condition_buffer_;
};

}  // namespace hsrb_gripper_controller

#endif  // HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_FOLLOW_TRAJECTORY_ACTION_HPP_
