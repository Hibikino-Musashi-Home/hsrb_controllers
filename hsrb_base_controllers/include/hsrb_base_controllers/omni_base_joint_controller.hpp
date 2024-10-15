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
/// @file omni_base_joint_controller.hpp
/// @brief All -sided bogie joint controller class
#ifndef HSRB_BASE_CONTROLLERS_OMNI_BASE_JOINT_CONTROLLER_HPP_
#define HSRB_BASE_CONTROLLERS_OMNI_BASE_JOINT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <hsrb_base_controllers/filter.hpp>
#include <hsrb_base_controllers/twin_caster_drive.hpp>

namespace hsrb_base_controllers {

/// Limit of turning axle speed and wheel speed
struct VelocityLimit {
  // Turning axis speed limit [RAD/S]
  double yaw_limit;
  // Wheel speed limit [RAD/S]
  double wheel_limit;
};

/// Joint controller class
class OmniBaseJointController {
 public:
  using Ptr = std::shared_ptr<OmniBaseJointController>;

  explicit OmniBaseJointController(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
  ~OmniBaseJointController() = default;

  // Parameter initialization
  bool Init();

  // Interface settings
  std::vector<std::string> command_interface_names() const;
  std::vector<std::string> state_interface_names() const;
  bool Activate(std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
                std::vector<hardware_interface::LoanedStateInterface>& state_interfaces);
  // Calculate the command value
  void SetJointCommand(double period, const Eigen::Vector3d output_velocity);
  // Get the axis position
  bool GetJointPositions(Eigen::Vector3d& positions_out) const;
  // Get the axial speed
  bool GetJointVelocities(Eigen::Vector3d& velocities_out) const;
  // Reset the target position of the turning axis
  void ResetDesiredSteerPosition() {
    desired_steer_pos_ = current_position_interfaces_[kJointIDSteer].get().get_value();
  }

  // Accessor
  Eigen::Vector3d joint_command() const { return joint_command_; }
  OmniBaseSize omnibase_size() const { return omnibase_size_; }
  std::string l_wheel_joint_name() const { return joint_names_[kJointIDLeftWheel]; }
  std::string r_wheel_joint_name() const { return joint_names_[kJointIDRightWheel]; }
  std::string steer_joint_name() const { return joint_names_[kJointIDSteer]; }
  std::vector<std::string> joint_names() const { return joint_names_; }
  double desired_steer_pos() const { return desired_steer_pos_; }

 private:
  // Controller node handle
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  // Joint command value
  Eigen::Vector3d joint_command_;
  // Name of each axis
  std::vector<std::string> joint_names_;

  // Handler of each axis of the bogie
  template <typename T>
  using InterfaceReferences = std::vector<std::reference_wrapper<T>>;
  InterfaceReferences<hardware_interface::LoanedCommandInterface> command_interfaces_;
  InterfaceReferences<hardware_interface::LoanedStateInterface> current_position_interfaces_;
  InterfaceReferences<hardware_interface::LoanedStateInterface> current_velocity_interfaces_;

  // Dimensions of bogie
  OmniBaseSize omnibase_size_;
  // Target position of the turning axis
  double desired_steer_pos_;
  // Bogie athletic model
  TwinCasterDrive::Ptr twin_drive_;
  // Command speed limit
  VelocityLimit velocity_limit_;
  // Encoder value speed threshold
  VelocityLimit actual_velocity_threshold_;
  // Speed ​​filter
  std::vector<Filter<> > velocity_filters_;
};

}  // namespace hsrb_base_controllers

#endif /*HSRB_BASE_CONTROLLERS_OMNI_BASE_JOINT_CONTROLLER_HPP_*/
