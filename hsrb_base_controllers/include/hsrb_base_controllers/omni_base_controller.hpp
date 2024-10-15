/*
Copyright (c) 2015 TOYOTA MOTOR CORPORATION
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
/// @file omni_base_controller.hpp
/// @brief Omnidistant bogie controller class
#ifndef HSRB_BASE_CONTROLLERS_OMNI_BASE_CONTROLLER_HPP_
#define HSRB_BASE_CONTROLLERS_OMNI_BASE_CONTROLLER_HPP_

#include <string>

#include <controller_interface/controller_interface.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <hsrb_base_controllers/command_subscriber.hpp>
#include <hsrb_base_controllers/controller_command_interface.hpp>
#include <hsrb_base_controllers/omni_base_control_method.hpp>
#include <hsrb_base_controllers/omni_base_joint_controller.hpp>
#include <hsrb_base_controllers/omni_base_odometry.hpp>

#include "tolerances.hpp"

namespace hsrb_base_controllers {

/// Omnidistant bogie speed controller class
class OmniBaseController
    : public controller_interface::ControllerInterface,
      public IControllerCommandInterface {
 public:
  OmniBaseController() = default;
  ~OmniBaseController() = default;

  // Initialization of controller
  controller_interface::CallbackReturn on_init() override;

  // Ros2_control interface settings
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // Calculate and update the bogie joint speed
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  // Functions called at configure
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  // Functions called during Activate
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  // Functions called during DEACTIVATE
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  // Return if you can accept the command
  bool IsAcceptable() override;

  // Set the input speed command
  void UpdateVelocity(const geometry_msgs::msg::Twist::SharedPtr& msg) override;

  // Verify the input orbit command
  bool ValidateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) override;
  // Set the input orbit command
  void UpdateTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr& trajectory) override;
  // Reset the input orbit
  void ResetTrajectory() override;

 protected:
  // ControllerInterface :: Initialization and testing for parts other than INIT
  bool InitImpl();

  // Available in track tracking
  SegmentTolerances default_tolerances_;
  SegmentTolerances active_tolerances_;
  // Check for Tolerances while tracking
  // If you continue to follow, return the positive number. If you stop following, return the constrol_msgs/action/FollowJointtrajectory error code (0 ~ -5).
  int32_t CheckTorelances(const ControllerBaseState& state, bool before_last_point, double time_from_trajectory_end);

  // Input Speed ​​Directors Subscler
  CommandVelocitySubscriber::Ptr velocity_subscriber_;
  // Input orbital command subcliver
  CommandTrajectorySubscriber::Ptr trajectory_subscriber_;
  // Input orbit Action command server
  TrajectoryActionServer::Ptr trajectory_action_;

  // Joint controller
  OmniBaseJointController::Ptr joint_controller_;

  // Bogie wheel and dometry calculation class
  BaseOdometry::Ptr base_odometry_;
  WheelOdometry::Ptr wheel_odometry_;

  // Class to calculate the command speed of the bogie
  OmniBaseVelocityControl::Ptr velocity_control_;
  OmniBaseTrajectoryControl::Ptr trajectory_control_;

  // Issuance of a bogie condition
  StatePublisher::Ptr joint_state_publisher_;
  StatePublisher::Ptr base_state_publisher_;
};

}  // namespace hsrb_base_controllers

#endif/*HSRB_BASE_CONTROLLERS_OMNI_BASE_CONTROLLER_HPP_*/
