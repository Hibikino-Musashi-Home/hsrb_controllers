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
/// @brief Omnidirectional Cart Joint Controller Class
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
#include <hsrb_base_controllers/omni_base_state.hpp>
#include <hsrb_base_controllers/twin_caster_drive.hpp>

namespace hsrb_base_controllers {

/// Joint Controller Class
class OmniBaseJointControllerBase {
 public:
  using Ptr = std::shared_ptr<OmniBaseJointControllerBase>;

  explicit OmniBaseJointControllerBase(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
  ~OmniBaseJointControllerBase() = default;

  // Initialize parameters
  bool Init();

  // Set interface
  virtual std::vector<std::string> command_interface_names() const = 0;
  std::vector<std::string> state_interface_names() const;
  bool Activate(std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
                std::vector<hardware_interface::LoanedStateInterface>& state_interfaces);
  // Calculate command value
  void SetJointCommand(const double period, const Eigen::Vector3d output_velocity);
  // Set command value
  void SetJointCommand(const double period, const State& desired_state);
  // Get axis position
  bool GetJointPositions(Eigen::Vector3d& positions_out) const;
  // Get axis speed
  bool GetJointVelocities(Eigen::Vector3d& velocities_out) const;
  // Reset target position of the turning axis
  virtual void ResetDesiredSteerPosition() = 0;

  // Accessor
  Eigen::Vector3d joint_command_position() const { return joint_command_position_; }
  Eigen::Vector3d joint_desired_velocity() const { return joint_desired_velocity_; }
  Eigen::Vector3d joint_output_velocity() const { return joint_output_velocity_; }
  Eigen::Vector3d base_output_velocity() const { return base_output_velocity_; }
  OmniBaseSize omnibase_size() const { return omnibase_size_; }
  std::string l_wheel_joint_name() const { return joint_names_[kJointIDLeftWheel]; }
  std::string r_wheel_joint_name() const { return joint_names_[kJointIDRightWheel]; }
  std::string steer_joint_name() const { return joint_names_[kJointIDSteer]; }
  std::vector<std::string> joint_names() const { return joint_names_; }

 protected:
  virtual void SetCommandToCommandInterface(double period) = 0;

  // Joint command value
  Eigen::Vector3d joint_command_position_;
  // Joint command value
  Eigen::Vector3d joint_desired_velocity_;
  // Joint command value (after filtering)
  Eigen::Vector3d joint_output_velocity_;
  // Speed in cart direction
  Eigen::Vector3d base_output_velocity_;

  // Handler for each axis of the cart
  template <typename T>
  using InterfaceReferences = std::vector<std::reference_wrapper<T>>;
  InterfaceReferences<hardware_interface::LoanedCommandInterface> command_interfaces_;
  InterfaceReferences<hardware_interface::LoanedStateInterface> current_position_interfaces_;
  InterfaceReferences<hardware_interface::LoanedStateInterface> current_velocity_interfaces_;

  // Joint name of CommandInterface
  std::vector<std::string> command_joint_names_;
  std::vector<std::string> interface_types_;

 private:
  // Determine joint speed that satisfies acceleration limit
  std::optional<Eigen::Vector3d> ApplyAccelerationLimits(
      const Eigen::Vector3d& joint_velocity_current,
      const Eigen::Vector3d& joint_velocity_prev,
      const double period);

  // Node handle of the controller
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  // Name of each axis
  std::vector<std::string> joint_names_;

  // Dimension information of the cart
  OmniBaseSize omnibase_size_;
  // Kinematic model of the cart
  TwinCasterDrive::Ptr twin_drive_;
  // Command speed limit
  Eigen::Vector3d velocity_limit_;
  // Acceleration limit
  Eigen::Vector3d acceleration_limit_;
  // Encoder value speed threshold
  Eigen::Vector3d actual_velocity_threshold_;
  // Speed filter
  std::vector<Filter<> > velocity_filters_;
};

// Turning axis is a controller that performs position commands
class OmniBaseJointControllerBaseRollPosition : public OmniBaseJointControllerBase {
 public:
  explicit OmniBaseJointControllerBaseRollPosition(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
  ~OmniBaseJointControllerBaseRollPosition() = default;

  std::vector<std::string> command_interface_names() const override;

  // Reset target position of the turning axis
  void ResetDesiredSteerPosition() override {
    joint_command_position_(kJointIDSteer) = current_position_interfaces_[kJointIDSteer].get().get_value();
  }

 protected:
  void SetCommandToCommandInterface(double period) override;
};

// Turning axis is a controller that performs speed commands
class OmniBaseJointControllerBaseRollVelocity : public OmniBaseJointControllerBase {
 public:
  explicit OmniBaseJointControllerBaseRollVelocity(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
      : OmniBaseJointControllerBase(node) {}
  ~OmniBaseJointControllerBaseRollVelocity() = default;

  std::vector<std::string> command_interface_names() const override;

  // Reset target position of the turning axis
  void ResetDesiredSteerPosition() override {}

 protected:
  void SetCommandToCommandInterface(double period) override;
};

}  // namespace hsrb_base_controllers

#endif /*HSRB_BASE_CONTROLLERS_OMNI_BASE_JOINT_CONTROLLER_HPP_*/
