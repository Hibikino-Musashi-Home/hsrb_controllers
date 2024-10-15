/*
Copyright (c) 2019 TOYOTA MOTOR CORPORATION
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
/// @file omni_base_control_method.cpp
/// @brief Omnidistant bogie control mode class

#include <hsrb_base_controllers/omni_base_control_method.hpp>

#include "utils.hpp"

namespace {
// Default value of the bogie speed specified default time [S]
constexpr double kDefaultCommandTimeout = 0.5;
// The threshold of the speed determined to be stopped
constexpr double kStopVelocityThreshold = 0.001;
// Time margin to stop tracking route [S]
constexpr double kStopTimeMergin = 0.2;
// Control for the gap in track follow -up P gain
constexpr double kDefaultPGain = 1.0;

// Create an array for sorting two named sequences with different described order
std::vector<uint32_t> MakePermutationVector(
    const std::vector<std::string>& names1,
    const std::vector<std::string>& names2) {
  // If the size of the input array does not match, it will end
  if (names1.size() != names2.size()) {
    return std::vector<uint32_t>();
  }

  // Find a matching name and create an array for sorting
  std::vector<uint32_t> permutation_vector(names1.size());
  for (std::vector<std::string>::const_iterator it1 = names1.begin(); it1 != names1.end(); ++it1) {
    std::vector<std::string>::const_iterator it2 = std::find(names2.begin(), names2.end(), *it1);
    if (names2.end() == it2) {
      return std::vector<uint32_t>();
    } else {
      const uint32_t t1 = std::distance(names1.begin(), it1);
      const uint32_t t2 = std::distance(names2.begin(), it2);
      permutation_vector[t1] = t2;
    }
  }
  return permutation_vector;
}
}  // namespace

namespace hsrb_base_controllers {

OmniBaseVelocityControl::OmniBaseVelocityControl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
    : node_(node) {
  // Acquire the speed command value cut judgment time
  command_timeout_ = GetParameter(node, "command_timeout", kDefaultCommandTimeout);
  if (command_timeout_ <= 0.0) {
    RCLCPP_INFO(
      node->get_logger(),
      "command_timeout must be positive. Use default value [%lf]", kDefaultCommandTimeout);
    command_timeout_ = kDefaultCommandTimeout;
  }
  // Call it by constructor and initialize it just in case
  Activate();
}

// Initialize
void OmniBaseVelocityControl::Activate() {
  std::lock_guard<std::mutex> lock(command_mutex_);
  command_velocity_ = Eigen::Vector3d::Zero();
  last_velocity_subscribed_time_ = node_->get_clock()->now();
}

// Get command speed
Eigen::Vector3d OmniBaseVelocityControl::GetOutputVelocity() {
  std::lock_guard<std::mutex> lock(command_mutex_);

  Eigen::Vector3d output_velocity = command_velocity_;
  if (node_->get_clock()->now() - last_velocity_subscribed_time_ > rclcpp::Duration::from_seconds(command_timeout_)) {
    output_velocity = Eigen::Vector3d::Zero();
  }
  return output_velocity;
}

// Update the command speed
void OmniBaseVelocityControl::UpdateCommandVelocity(const geometry_msgs::msg::Twist::SharedPtr& msg) {
  std::lock_guard<std::mutex> lock(command_mutex_);

  command_velocity_ << msg->linear.x, msg->linear.y, msg->angular.z;
  last_velocity_subscribed_time_ = node_->get_clock()->now();
}


// Initialize constructors and parameters
OmniBaseTrajectoryControl::OmniBaseTrajectoryControl(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::vector<std::string>& cordinates) : node_(node), coordinate_names_(cordinates) {
  stop_velocity_threshold_ = GetPositiveParameter(node, "stop_velocity_threshold", kStopVelocityThreshold);
  feedback_gain_(kIndexBaseX) = GetPositiveParameter(node, "odom_x.p_gain", kDefaultPGain);
  feedback_gain_(kIndexBaseY) = GetPositiveParameter(node, "odom_y.p_gain", kDefaultPGain);
  feedback_gain_(kIndexBaseTheta) = GetPositiveParameter(node, "odom_t.p_gain", kDefaultPGain);
  open_loop_control_ = GetParameter(node, "open_loop_control", false);
}

// Initialization
void OmniBaseTrajectoryControl::Activate() {
  trajectory_ptr_ = std::make_shared<joint_trajectory_controller::Trajectory>();
  trajectory_active_ptr_ = &trajectory_ptr_;
  trajectory_msg_buffer_.writeFromNonRT(std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());
  has_last_command_state_ = false;
}

// Get command speed
Eigen::Vector3d OmniBaseTrajectoryControl::GetOutputVelocity(
    const ControllerState& base_state) {
  // Add a term that is proportional to the position difference in the command speed and make it a target speed.
  const Eigen::Vector3d desired(base_state.desired.velocities.data());
  const Eigen::Vector3d error(base_state.error.positions.data());
  Eigen::Vector3d output_velocity = desired + feedback_gain_.cwiseProduct(error);

  // Convert the speed of the standard coordinate system to the upper body coordinate system
  const double current_yaw = base_state.actual.positions.at(kIndexBaseTheta);
  Eigen::Matrix3d robot_to_floor;
  robot_to_floor <<
      std::cos(current_yaw), std::sin(current_yaw), 0.0,
     -std::sin(current_yaw), std::cos(current_yaw), 0.0,
      0.0, 0.0, 1.0;
  output_velocity = robot_to_floor * output_velocity;

  return output_velocity;
}

  // Update the track during follow -up, return True if there is a trajectory
bool OmniBaseTrajectoryControl::UpdateActiveTrajectory() {
  const auto current_msg = trajectory_ptr_->get_trajectory_msg();
  const auto new_msg = trajectory_msg_buffer_.readFromRT();
  if (current_msg != *new_msg) {
    trajectory_ptr_->update(*new_msg);
  }
  if (trajectory_active_ptr_ && (*trajectory_active_ptr_)->has_trajectory_msg()) {
    if ((*trajectory_active_ptr_)->get_trajectory_msg()->points.empty()) {
      return false;
    } else {
      return true;
    }
  } else {
    return false;
  }
}

// Get the target status of track tracking
bool OmniBaseTrajectoryControl::SampleDesiredState(
    const rclcpp::Time& time,
    const std::vector<double>& current_positions,
    const std::vector<double>& current_velocities,
    trajectory_msgs::msg::JointTrajectoryPoint& desired_state,
    bool& before_last_point,
    double& time_from_point) {
  if (!(*trajectory_active_ptr_)->is_sampled_already()) {
    if (open_loop_control_ && has_last_command_state_) {
      (*trajectory_active_ptr_)->set_point_before_trajectory_msg(time, last_command_state_);
    } else {
      trajectory_msgs::msg::JointTrajectoryPoint current_state;
      current_state.positions = current_positions;
      current_state.velocities = current_velocities;
      (*trajectory_active_ptr_)->set_point_before_trajectory_msg(time, current_state);
    }
  }
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator start_segment_it;
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator end_segment_it;
  const bool is_ok = (*trajectory_active_ptr_)->sample(time,
      joint_trajectory_controller::interpolation_methods::DEFAULT_INTERPOLATION,
      desired_state, start_segment_it, end_segment_it);
  if (is_ok) {
    before_last_point = end_segment_it != (*trajectory_active_ptr_)->end();
    const rclcpp::Time start_stamp = (*trajectory_active_ptr_)->time_from_start();
    const rclcpp::Time end_stamp = start_stamp + start_segment_it->time_from_start;
    time_from_point = time.seconds() - end_stamp.seconds();

    last_command_state_ = desired_state;
    has_last_command_state_ = true;
  }
  return is_ok;
}

// Verify the input orbit command
bool OmniBaseTrajectoryControl::ValidateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) const {
  // Disable if the joint name is not right
  if (trajectory.joint_names.size() != coordinate_names_.size()) {
    RCLCPP_ERROR(node_->get_logger(), "Trajectory's joint_size mismatch.");
    return false;
  }
  for (const auto& joint_name : trajectory.joint_names) {
    if (std::find(coordinate_names_.begin(), coordinate_names_.end(), joint_name) == coordinate_names_.end()) {
      RCLCPP_ERROR(node_->get_logger(), "Trajectory's joint_name mismatch.");
      return false;
    }
  }

  // Check if the contents are valid for each JointTrajectoryPoint
  double last_time = -std::numeric_limits<double>::max();
  for (const auto& point : trajectory.points) {
    // Disable if the number of elements of Position does not match the joint number
    if (point.positions.size() != coordinate_names_.size()) {
      RCLCPP_ERROR(node_->get_logger(), "Trajectory's position size is wrong.");
      return false;
    }
    // Disable if the number of Velocity does not match the joint number
    // If Velocity is empty, accept
    if (!point.velocities.empty() && point.velocities.size() != coordinate_names_.size()) {
      RCLCPP_ERROR(node_->get_logger(), "Trajectory's velocity size is wrong.");
      return false;
    }
    // Disabled if the number of Acceleration elements does not match the joint number
    // If Acceleration is empty, accept
    if (!point.accelerations.empty() && point.accelerations.size() != coordinate_names_.size()) {
      RCLCPP_ERROR(node_->get_logger(), "Trajectory's acceleration size is wrong.");
      return false;
    }
    // Disable if Time_from_start is going backwards
    double time_from_start = static_cast<rclcpp::Duration>(point.time_from_start).seconds();
    if (time_from_start - last_time <= 0.0) {
      RCLCPP_ERROR(node_->get_logger(), "Trajectory's time_from_start is going reverse.");
      return false;
    }
    last_time = time_from_start;
  }
  return true;
}

// Update the track
void OmniBaseTrajectoryControl::AcceptTrajectory(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr& trajectory,
    const Eigen::Vector3d& base_positions) {
  // If there is no point in the input orbit, stop
  if (trajectory->points.empty()) {
    ResetCurrentTrajectory();
    return;
  }

  // Create replacement arrays of input MSG
  const std::vector<std::string> trajectory_joint_names = trajectory->joint_names;
  const std::vector<uint32_t> permutation_vector = MakePermutationVector(coordinate_names_, trajectory_joint_names);

  // Create tracks that take into account the order of the name of the axis
  trajectory_msgs::msg::JointTrajectory permutated_trajectory;
  permutated_trajectory.header = trajectory->header;
  permutated_trajectory.joint_names = trajectory->joint_names;
  for (const auto& input_point : trajectory->points) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (const auto index : permutation_vector) {
      if (input_point.positions.size() == permutation_vector.size()) {
        point.positions.push_back(input_point.positions.at(index));
      }
      if (input_point.velocities.size() == permutation_vector.size()) {
        point.velocities.push_back(input_point.velocities.at(index));
      }
      if (input_point.accelerations.size() == permutation_vector.size()) {
        point.accelerations.push_back(input_point.accelerations.at(index));
      }
    }
    point.time_from_start = input_point.time_from_start;
    permutated_trajectory.points.push_back(point);
  }
  // Correction of turning axis
  double prev_position;
  if (open_loop_control_ && has_last_command_state_) {
    prev_position = last_command_state_.positions[kIndexBaseTheta];
  } else {
    prev_position = base_positions[kIndexBaseTheta];
  }
  for (auto& point : permutated_trajectory.points) {
    double diff = angles::shortest_angular_distance(prev_position, point.positions[kIndexBaseTheta]);
    point.positions[kIndexBaseTheta] = prev_position + diff;
    prev_position = point.positions[kIndexBaseTheta];
  }

  trajectory_msg_buffer_.writeFromNonRT(std::make_shared<trajectory_msgs::msg::JointTrajectory>(permutated_trajectory));
}

// If you meet the conditions, end the track follow -up
void OmniBaseTrajectoryControl::TerminateControl(const rclcpp::Time& time, const ControllerState& base_state) {
  if (!trajectory_active_ptr_ || !(*trajectory_active_ptr_)->has_trajectory_msg()) {
    return;
  }
  if ((*trajectory_active_ptr_)->get_trajectory_msg()->points.empty()) {
    return;
  }

  const double time_from_start = (time - (*trajectory_active_ptr_)->time_from_start()).seconds();
  const Eigen::Vector3d current_velocity(base_state.actual.velocities.data());
  // Finish the trajectory following the scheduled tracking time of the orbital follow
  const rclcpp::Duration command_trajectory_period = (--((*trajectory_active_ptr_)->end()))->time_from_start;
  if ((time_from_start > command_trajectory_period.seconds() + kStopTimeMergin) &&
      (current_velocity.norm() < stop_velocity_threshold_)) {
    ResetCurrentTrajectory();
  }
}

// Reset the trajectory currently following
void OmniBaseTrajectoryControl::ResetCurrentTrajectory() {
  trajectory_msgs::msg::JointTrajectory empty_msg;
  empty_msg.header.stamp = rclcpp::Time(0);
  trajectory_msg_buffer_.writeFromNonRT(std::make_shared<trajectory_msgs::msg::JointTrajectory>(empty_msg));
  has_last_command_state_ = false;
}

}  // namespace hsrb_base_controllers
