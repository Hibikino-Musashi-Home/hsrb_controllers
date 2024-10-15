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
/// @file omni_base_control_method.hpp
/// @brief Omnidistant bogie control mode class
#ifndef HSRB_BASE_CONTROLLERS_OMNI_BASE_CONTROL_METHOD_HPP_
#define HSRB_BASE_CONTROLLERS_OMNI_BASE_CONTROL_METHOD_HPP_

#include <memory>
#include <string>
#include <vector>

#include <boost/noncopyable.hpp>

#include <Eigen/Core>

#include <geometry_msgs/msg/twist.hpp>
#include <joint_trajectory_controller/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.h>

#include <hsrb_base_controllers/omni_base_state.hpp>


namespace hsrb_base_controllers {

/// Bogie control means interface class
class IBaseControlMethod : private boost::noncopyable {
 public:
  using Ptr = std::shared_ptr<IBaseControlMethod>;

  virtual ~IBaseControlMethod() {}
  // Initialize
  virtual void Activate() = 0;
};


/// Bogie speed tracking
class OmniBaseVelocityControl : public IBaseControlMethod {
 public:
  using Ptr = std::shared_ptr<OmniBaseVelocityControl>;

  explicit OmniBaseVelocityControl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
  virtual ~OmniBaseVelocityControl() = default;
  // Initialize
  void Activate() override;

  // Get command speed
  Eigen::Vector3d GetOutputVelocity();
  // Update the command speed
  void UpdateCommandVelocity(const geometry_msgs::msg::Twist::SharedPtr& msg);

 private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  // Exclusive control
  std::mutex command_mutex_;
  // Instruction speed
  Eigen::Vector3d command_velocity_;
  // The last time I received the speed command value
  rclcpp::Time last_velocity_subscribed_time_;
  // Speed ​​command value cut judgment time
  double command_timeout_;
};


/// Trolley track follow -up
class OmniBaseTrajectoryControl : public IBaseControlMethod {
 public:
  using Ptr = std::shared_ptr<OmniBaseTrajectoryControl>;

  explicit OmniBaseTrajectoryControl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                     const std::vector<std::string>& cordinates);
  virtual ~OmniBaseTrajectoryControl() = default;
  // Initialize
  void Activate() override;

  // Get command speed
  Eigen::Vector3d GetOutputVelocity(const ControllerState& base_state);
  // Update the track during follow -up, return True if there is a trajectory
  bool UpdateActiveTrajectory();
  // Get the target status of track tracking
  bool SampleDesiredState(const rclcpp::Time& time,
                          const std::vector<double>& current_positions,
                          const std::vector<double>& current_velocities,
                          trajectory_msgs::msg::JointTrajectoryPoint& desired_state,
                          bool& before_last_point,
                          double& time_from_point);
  // Verify the input orbit command
  bool ValidateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) const;
  // Update the track
  void AcceptTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr& trajectory,
                        const Eigen::Vector3d& base_positions);
  // If you meet the conditions, end the track follow -up
  void TerminateControl(const rclcpp::Time& time, const ControllerState& base_state);
  // Reset the trajectory currently following
  void ResetCurrentTrajectory();

 private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  // Control feedback gain
  Eigen::Vector3d feedback_gain_;
  // Bogie coordinate axis name
  std::vector<std::string> coordinate_names_;
  // Speed ​​threshold for judging the completion of track follow -up
  double stop_velocity_threshold_;
  // Whether to connect from the existing desired when a new orbit comes
  // Variables and behaviors are tailored to JointTrajectoryController
  bool open_loop_control_;
  // Finally sampled state
  trajectory_msgs::msg::JointTrajectoryPoint last_command_state_;
  // Last_command_state_ is a value
  // It is correct to put the current price at the time of Activate, like JointtrajectoryController
  // Since the change part is wider, it is implemented by flag management.
  bool has_last_command_state_;

  std::shared_ptr<joint_trajectory_controller::Trajectory>* trajectory_active_ptr_ = nullptr;
  std::shared_ptr<joint_trajectory_controller::Trajectory> trajectory_ptr_ = nullptr;
  realtime_tools::RealtimeBuffer<trajectory_msgs::msg::JointTrajectory::SharedPtr>  trajectory_msg_buffer_;
};

}  // namespace hsrb_base_controllers

#endif /*HSRB_BASE_CONTROLLERS_OMNI_BASE_CONTROL_METHOD_HPP_*/
