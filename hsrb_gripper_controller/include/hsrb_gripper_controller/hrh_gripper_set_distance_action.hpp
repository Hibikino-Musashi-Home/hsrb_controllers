/*
Copyright (c) 2025 TOYOTA MOTOR CORPORATION
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
#ifndef HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_SET_DISTANCE_ACTION_HPP_
#define HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_SET_DISTANCE_ACTION_HPP_
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <std_msgs/msg/float32.hpp>
#include <tmc_control_msgs/action/gripper_set_distance.hpp>

#include "hsrb_gripper_controller/hrh_gripper_action.hpp"

namespace hsrb_gripper_controller {

/// @class HrhGripperSetDistanceCalculator
/// @brief Hrh Gripper Fingertip Distance Calculation Class
class HrhGripperSetDistanceCalculator {
 public:
  using Ptr = std::shared_ptr<HrhGripperSetDistanceCalculator>;
  /// Constructor
  HrhGripperSetDistanceCalculator();

  /// Destructor
  virtual ~HrhGripperSetDistanceCalculator() = default;

  /// Initialize the physical parameters of the hand
  bool InitializeHandSizeData(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);

  /// Calculation of fingertip distance
  /// @return Fingertip distance [m]
  double GetDistanceFromPosition(double hand_motor_pos) const;

  /// Calculation of fingertip distance
  /// @return Fingertip distance [m]
  double GetDistanceFromPosition(
      double hand_motor_pos, double left_spring_proximal_joint_pos,
      double right_spring_proximal_joint_pos) const;

  /// Calculate joint angle from fingertip distance
  /// @return Joint angle [rad]
  double GetPositionFromDistance(double distance) const;

 private:
  /// URDF retrieval
  std::string GetRobotDescription(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);

  /// Finger length [m]
  double proximal_to_distal_z_;
  double distance_palm_to_tip_;
};

/// @class HrhGripperSetDistanceAction
/// @brief Hrh Fingertip Distance Setting Action Class
class HrhGripperSetDistanceAction : public HrhGripperAction<tmc_control_msgs::action::GripperSetDistance> {
 public:
  /// Constructor
  /// @param [in] controller Parent controller
  explicit HrhGripperSetDistanceAction(HrhGripperController* controller);
  virtual ~HrhGripperSetDistanceAction() = default;

  void Update(const rclcpp::Time& time) override;

  void PreemptActiveGoal() override;

 protected:
  /// Implementation of action initialization
  bool InitImpl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) override;
  /// Update action target
  void UpdateActionImpl(const tmc_control_msgs::action::GripperSetDistance::Goal& goal) override;

  /// Calculate target position from error between command value and current opening width
  /// @param [in] current_distance Current opening width
  /// @return Target position
  double GetCommandPos(const double current_distance);

  /// Success determination
  /// @param [in] time             Current time
  /// @param [in] current_distance Current opening width
  void CheckForSuccess(const rclcpp::Time& time, const double current_distance);

  /// Callback when opening width command is received on topic
  /// @param [in] msg Opening width
  void DistanceCommandCallback(const std_msgs::msg::Float32::SharedPtr msg);

  /// Command set
  /// @param [in] distance  Opening width
  /// @param [in] stop_flag Control stop flag
  void SetCommandValue(const double distance);

  /// Goal position permissible error [m]
  double goal_tolerance_;
  /// Velocity threshold for stall determination [rad/s]
  double stall_velocity_threshold_;
  /// Time for stall determination [s]
  double distance_control_stall_timeout_;
  /// Opening width control P gain
  double distance_control_pgain_;
  /// Opening width control I gain
  double distance_control_igain_;
  /// Opening width control D gain
  double distance_control_dgain_;
  /// Maximum angle [rad]
  double hand_motor_joint_max_;
  /// Minimum angle [rad]
  double hand_motor_joint_min_;

  // Maximum opening width [m]
  double distance_max_;
  // Minimum opening width [m]
  double distance_min_;
  // Integrated error value
  double integrated_distance_error_;
  // Previous error value
  double last_error_;

  // Last operation time
  rclcpp::Time last_movement_time_;

  /// Opening width command reception
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_command_sub_;

  /// Fingertip distance calculator
  HrhGripperSetDistanceCalculator::Ptr distance_calculator_;

  /// Goal buffer
  realtime_tools::RealtimeBuffer<double> goal_buffer_;

  /// Motion stop flag buffer
  realtime_tools::RealtimeBuffer<bool> stop_flag_buffer_;
};

}  // namespace hsrb_gripper_controller

#endif  // HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_SET_DISTANCE_ACTION_HPP_
