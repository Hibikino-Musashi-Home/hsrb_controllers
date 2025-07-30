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
/// @file omni_base_joint_controller-test.cpp
/// @brief Test of the omni-directional cart joint controller class

#include <fstream>
#include <string>
#include <vector>

#include <boost/thread.hpp>
#include <gtest/gtest.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <hsrb_base_controllers/omni_base_joint_controller.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include "hardware_stub.hpp"
#include "utils.hpp"

namespace {
constexpr double kEpsilon = 1.0e-5;

void spin_some(const rclcpp::Node::SharedPtr& node) {
  while (node) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

}  // namespace

namespace hsrb_base_controllers {

template<typename _JointControllerType, typename _CommandHandleType>
struct TypePair {
  using JointControllerType = _JointControllerType;
  using CommandHandleType = _CommandHandleType;
};

template<typename TypePair>
class OmniBaseJointControllerBaseTest : public ::testing::Test {
 public:
  void SetUp() override;

 protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Node::SharedPtr urdf_node_;
  OmniBaseJointControllerBase::Ptr controller_;
  HardwareStub<typename TypePair::CommandHandleType> hardware_;

  void TestCommandBaseVelocity(const Eigen::Vector3d& base_velocity,
                               const Eigen::Vector3d& expected_joint_velocity);
  void TestCommandBaseVelocity(const Eigen::Vector3d& base_velocity,
                               const Eigen::Vector3d& expected_joint_velocity,
                               const Eigen::Vector3d& expected_interface_command);
};

template<typename TypePair>
void OmniBaseJointControllerBaseTest<TypePair>::SetUp() {
  node_ = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  node_->configure();
  controller_ = std::make_shared<typename TypePair::JointControllerType>(node_);

  auto parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor();
  parameter_descriptor.dynamic_typing = true;
  node_->declare_parameter("joints.steer", "base_roll_joint", parameter_descriptor);
  node_->declare_parameter("joints.r_wheel", "base_r_drive_wheel_joint", parameter_descriptor);
  node_->declare_parameter("joints.l_wheel", "base_l_drive_wheel_joint", parameter_descriptor);

  node_->declare_parameter("robot_description", GetRobotDescription(), parameter_descriptor);
  node_->declare_parameter("parameter_connection_timeout", 0);

  node_->activate();

  urdf_node_ = rclcpp::Node::make_shared("urdf_node");
  urdf_node_->declare_parameter("robot_description", GetRobotDescription());
}

template<typename TypePair>
void OmniBaseJointControllerBaseTest<TypePair>::TestCommandBaseVelocity(
    const Eigen::Vector3d& base_velocity,
    const Eigen::Vector3d& expected_joint_velocity) {
  EXPECT_TRUE(controller_->Init());
  EXPECT_TRUE(controller_->Activate(hardware_.command_interfaces, hardware_.state_interfaces));

  controller_->SetJointCommand(0.1, base_velocity);

  auto joint_command = controller_->joint_command();
  EXPECT_NEAR(expected_joint_velocity(kJointIDRightWheel), joint_command(kJointIDRightWheel), kEpsilon);
  EXPECT_NEAR(expected_joint_velocity(kJointIDLeftWheel), joint_command(kJointIDLeftWheel), kEpsilon);
  EXPECT_NEAR(expected_joint_velocity(kJointIDSteer), joint_command(kJointIDSteer), kEpsilon);
}

template<typename TypePair>
void OmniBaseJointControllerBaseTest<TypePair>::TestCommandBaseVelocity(
    const Eigen::Vector3d& base_velocity,
    const Eigen::Vector3d& expected_joint_velocity,
    const Eigen::Vector3d& expected_interface_command) {
  TestCommandBaseVelocity(base_velocity, expected_joint_velocity);

  EXPECT_NEAR(expected_interface_command(kJointIDRightWheel), hardware_.r_wheel_handle->command(), kEpsilon);
  EXPECT_NEAR(expected_interface_command(kJointIDLeftWheel), hardware_.l_wheel_handle->command(), kEpsilon);
  EXPECT_NEAR(expected_interface_command(kJointIDSteer), hardware_.steer_handle->command(), kEpsilon);
}


typedef ::testing::Types<
    TypePair<OmniBaseJointControllerBaseRollPosition, CommandPositionHandle>,
    TypePair<OmniBaseJointControllerBaseRollVelocity, CommandVelocityHandle>
> TestTypes;

TYPED_TEST_SUITE(OmniBaseJointControllerBaseTest, TestTypes);

/// Failure in initialization when the steering axis name is not in the list of prepared interfaces
TYPED_TEST(OmniBaseJointControllerBaseTest, BadSteerJointName) {
  this->node_->set_parameter({rclcpp::Parameter("joints.steer", "bad_joint_name")});
  EXPECT_FALSE(this->controller_->Init());
}

/// Failure in initialization when the left wheel axis name is different from the list of prepared interfaces
TYPED_TEST(OmniBaseJointControllerBaseTest, BadLeftWheelJointName) {
  this->node_->set_parameter({rclcpp::Parameter("joints.l_wheel", "bad_joint_name")});
  EXPECT_FALSE(this->controller_->Init());
}

/// Failure in initialization when the right wheel axis name is different from the list of prepared interfaces
TYPED_TEST(OmniBaseJointControllerBaseTest, BadRightWheelJointName) {
  this->node_->set_parameter({rclcpp::Parameter("joints.r_wheel", "bad_joint_name")});
  EXPECT_FALSE(this->controller_->Init());
}

/// Failure in initialization when no steering axis name is specified
TYPED_TEST(OmniBaseJointControllerBaseTest, NoSteerJointName) {
  this->node_->undeclare_parameter("joints.steer");
  EXPECT_FALSE(this->controller_->Init());
}

/// Failure in initialization when no left wheel axis name is specified
TYPED_TEST(OmniBaseJointControllerBaseTest, NoLeftWheelJointName) {
  this->node_->undeclare_parameter("joints.l_wheel");
  EXPECT_FALSE(this->controller_->Init());
}

/// Failure in initialization when no right wheel axis name is specified
TYPED_TEST(OmniBaseJointControllerBaseTest, NoRightWheelJointName) {
  this->node_->undeclare_parameter("joints.r_wheel");
  EXPECT_FALSE(this->controller_->Init());
}

/// Failure in initialization due to an invalid robot_description
TYPED_TEST(OmniBaseJointControllerBaseTest, InvalidRobotDescription) {
  this->node_->set_parameter({rclcpp::Parameter("robot_description", "invalid")});
  EXPECT_FALSE(this->controller_->Init());
}

/// Retrieve and initialize robot model from a different node
TYPED_TEST(OmniBaseJointControllerBaseTest, RobotDescriptionFromAnotherNode) {
  this->node_->undeclare_parameter("robot_description");
  this->node_->declare_parameter("model_node_name", "urdf_node");

  auto spin_thread = std::thread(std::bind(spin_some, std::ref(this->urdf_node_)));
  EXPECT_TRUE(this->controller_->Init());
  this->urdf_node_.reset();
  spin_thread.join();
}

/// No robot model exists in the specified node
TYPED_TEST(OmniBaseJointControllerBaseTest, NoRobotDescriptionOnAnotherNode) {
  this->node_->undeclare_parameter("robot_description");
  this->node_->declare_parameter("model_node_name", "no_description_node");
  auto node = rclcpp::Node::make_shared("no_description_node");

  auto spin_thread = std::thread(std::bind(spin_some, std::ref(node)));
  EXPECT_FALSE(this->controller_->Init());
  node.reset();
  spin_thread.join();
}

/// Attempt to reference a different node, but a nonexistent node is specified
TYPED_TEST(OmniBaseJointControllerBaseTest, NoRobotDescriptionNode) {
  this->node_->undeclare_parameter("robot_description");
  EXPECT_FALSE(this->controller_->Init());
}

/// Ability to obtain cart size
TYPED_TEST(OmniBaseJointControllerBaseTest, GetOmniBaseSize) {
  EXPECT_TRUE(this->controller_->Init());

  auto omnibase_size = this->controller_->omnibase_size();
  EXPECT_EQ(0.266, omnibase_size.tread);
  EXPECT_EQ(0.11, omnibase_size.caster_offset);
  EXPECT_EQ(0.04, omnibase_size.wheel_radius);
}

/// Ability to obtain l_wheel_joint_name
TYPED_TEST(OmniBaseJointControllerBaseTest, GetLeftWhellJointName) {
  EXPECT_TRUE(this->controller_->Init());
  EXPECT_EQ(this->controller_->l_wheel_joint_name(), "base_l_drive_wheel_joint");
}

/// Ability to obtain r_wheel_joint_name
TYPED_TEST(OmniBaseJointControllerBaseTest, GetRightWhellJointName) {
  EXPECT_TRUE(this->controller_->Init());
  EXPECT_EQ(this->controller_->r_wheel_joint_name(), "base_r_drive_wheel_joint");
}

/// Ability to obtain steer_joint_name
TYPED_TEST(OmniBaseJointControllerBaseTest, GetSteerJointName) {
  EXPECT_TRUE(this->controller_->Init());
  EXPECT_EQ(this->controller_->steer_joint_name(), "base_roll_joint");
}

/// Ability to obtain input interface names for joint states
TYPED_TEST(OmniBaseJointControllerBaseTest, GetStateInterfaceNames) {
  EXPECT_TRUE(this->controller_->Init());

  auto names = this->controller_->state_interface_names();
  EXPECT_EQ(names.size(), 6);
  EXPECT_NE(std::find(names.begin(), names.end(), "base_roll_joint/position"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "base_roll_joint/velocity"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "base_l_drive_wheel_joint/position"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "base_l_drive_wheel_joint/velocity"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "base_r_drive_wheel_joint/position"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "base_r_drive_wheel_joint/velocity"), names.end());
}

/// Failure to activate due to missing command_interface
TYPED_TEST(OmniBaseJointControllerBaseTest, CommandInterfaceShortage) {
  EXPECT_TRUE(this->controller_->Init());

  this->hardware_.command_interfaces.clear();
  EXPECT_FALSE(this->controller_->Activate(this->hardware_.command_interfaces, this->hardware_.state_interfaces));
}

/// Failure to activate due to missing state_interface
TYPED_TEST(OmniBaseJointControllerBaseTest, StateInterfaceShortage) {
  EXPECT_TRUE(this->controller_->Init());

  this->hardware_.state_interfaces.clear();
  EXPECT_FALSE(this->controller_->Activate(this->hardware_.command_interfaces, this->hardware_.state_interfaces));
}

/// Ability to activate repeatedly
TYPED_TEST(OmniBaseJointControllerBaseTest, ReActivate) {
  EXPECT_TRUE(this->controller_->Init());
  EXPECT_TRUE(this->controller_->Activate(this->hardware_.command_interfaces, this->hardware_.state_interfaces));
  EXPECT_TRUE(this->controller_->Activate(this->hardware_.command_interfaces, this->hardware_.state_interfaces));
}

/// Steering axis speed limit applies when setting command values
TYPED_TEST(OmniBaseJointControllerBaseTest, SetJointCommandWithYawLimit) {
  this->TestCommandBaseVelocity(Eigen::Vector3d(0.02, 0.03, 10.0), Eigen::Vector3d(0.260327, -0.0752804, -1.8));
}

/// Wheel speed limit applies when setting command values
TYPED_TEST(OmniBaseJointControllerBaseTest, SetJointCommandWidhWheelLimit) {
  this->TestCommandBaseVelocity(Eigen::Vector3d(1.0, 0.03, 0.01), Eigen::Vector3d(8.5, 7.90495, 0.0862005));
}

/// Steering axis speed limit parameter is applied
TYPED_TEST(OmniBaseJointControllerBaseTest, SetYawVelocityLimit) {
  this->node_->declare_parameter("yaw_velocity_limit", 0.18);
  this->TestCommandBaseVelocity(Eigen::Vector3d(0.02, 0.03, 10.0), Eigen::Vector3d(0.0260327, -0.00752804, -0.18));

  // Default value is used for invalid parameters
  this->node_->set_parameter({rclcpp::Parameter("yaw_velocity_limit", -1.0)});
  this->TestCommandBaseVelocity(Eigen::Vector3d(0.02, 0.03, 10.0), Eigen::Vector3d(0.260327, -0.0752804, -1.8));
}

/// Wheel axis speed limit parameter is applied
TYPED_TEST(OmniBaseJointControllerBaseTest, SetWheelVelocityLimit) {
  this->node_->declare_parameter("wheel_velocity_limit", 0.85);
  this->TestCommandBaseVelocity(Eigen::Vector3d(1.0, 0.03, 0.01), Eigen::Vector3d(0.85, 0.790495, 0.00862005));

  // Default value is used for invalid parameters
  this->node_->set_parameter({rclcpp::Parameter("wheel_velocity_limit", -1.0)});
  this->TestCommandBaseVelocity(Eigen::Vector3d(1.0, 0.03, 0.01), Eigen::Vector3d(8.5, 7.90495, 0.0862005));
}

/// Ability to obtain axis position
TYPED_TEST(OmniBaseJointControllerBaseTest, GetJointPositions) {
  EXPECT_TRUE(this->controller_->Init());
  EXPECT_TRUE(this->controller_->Activate(this->hardware_.command_interfaces, this->hardware_.state_interfaces));

  this->hardware_.r_wheel_handle->set_current_pos(0.1);
  this->hardware_.l_wheel_handle->set_current_pos(0.2);
  this->hardware_.steer_handle->set_current_pos(0.3);

  Eigen::Vector3d joint_positions;
  EXPECT_TRUE(this->controller_->GetJointPositions(joint_positions));
  EXPECT_EQ(joint_positions(kJointIDRightWheel), 0.1);
  EXPECT_EQ(joint_positions(kJointIDLeftWheel), 0.2);
  EXPECT_EQ(joint_positions(kJointIDSteer), 0.3);
}

/// Ability to obtain axis speed
TYPED_TEST(OmniBaseJointControllerBaseTest, GetJointVelocities) {
  EXPECT_TRUE(this->controller_->Init());
  EXPECT_TRUE(this->controller_->Activate(this->hardware_.command_interfaces, this->hardware_.state_interfaces));

  this->hardware_.r_wheel_handle->set_current_vel(0.1);
  this->hardware_.l_wheel_handle->set_current_vel(0.2);
  this->hardware_.steer_handle->set_current_vel(0.3);

  Eigen::Vector3d joint_velocities;
  EXPECT_TRUE(this->controller_->GetJointVelocities(joint_velocities));
  EXPECT_EQ(joint_velocities(kJointIDRightWheel), 0.1);
  EXPECT_EQ(joint_velocities(kJointIDLeftWheel), 0.2);
  EXPECT_EQ(joint_velocities(kJointIDSteer), 0.3);
}

/// Excessively high wheel axis speed results in error
TYPED_TEST(OmniBaseJointControllerBaseTest, TooBigWheelVelocities) {
  EXPECT_TRUE(this->controller_->Init());
  EXPECT_TRUE(this->controller_->Activate(this->hardware_.command_interfaces, this->hardware_.state_interfaces));

  Eigen::Vector3d joint_velocities;
  this->hardware_.r_wheel_handle->set_current_vel(1000.0 + kEpsilon);
  EXPECT_FALSE(this->controller_->GetJointVelocities(joint_velocities));

  this->hardware_.r_wheel_handle->set_current_vel(1000.0 - kEpsilon);
  EXPECT_TRUE(this->controller_->GetJointVelocities(joint_velocities));

  this->node_->set_parameter({rclcpp::Parameter("wheel_actual_velocity_threshold", 1.0)});
  EXPECT_TRUE(this->controller_->Init());

  this->hardware_.r_wheel_handle->set_current_vel(1.0 + kEpsilon);
  EXPECT_FALSE(this->controller_->GetJointVelocities(joint_velocities));

  this->hardware_.r_wheel_handle->set_current_vel(1.0 - kEpsilon);
  EXPECT_TRUE(this->controller_->GetJointVelocities(joint_velocities));

  this->node_->set_parameter({rclcpp::Parameter("wheel_actual_velocity_threshold", -1.0)});
  EXPECT_TRUE(this->controller_->Init());

  this->hardware_.r_wheel_handle->set_current_vel(1000.0 + kEpsilon);
  EXPECT_FALSE(this->controller_->GetJointVelocities(joint_velocities));

  this->hardware_.r_wheel_handle->set_current_vel(1000.0 - kEpsilon);
  EXPECT_TRUE(this->controller_->GetJointVelocities(joint_velocities));
}

/// Excessively high steering axis speed results in error
TYPED_TEST(OmniBaseJointControllerBaseTest, TooBigSteerVelocities) {
  EXPECT_TRUE(this->controller_->Init());
  EXPECT_TRUE(this->controller_->Activate(this->hardware_.command_interfaces, this->hardware_.state_interfaces));

  Eigen::Vector3d joint_velocities;
  this->hardware_.steer_handle->set_current_vel(1000.0 + kEpsilon);
  EXPECT_FALSE(this->controller_->GetJointVelocities(joint_velocities));

  this->hardware_.steer_handle->set_current_vel(1000.0 - kEpsilon);
  EXPECT_TRUE(this->controller_->GetJointVelocities(joint_velocities));

  this->node_->set_parameter({rclcpp::Parameter("yaw_actual_velocity_threshold", 1.0)});
  EXPECT_TRUE(this->controller_->Init());

  this->hardware_.steer_handle->set_current_vel(1.0 + kEpsilon);
  EXPECT_FALSE(this->controller_->GetJointVelocities(joint_velocities));

  this->hardware_.steer_handle->set_current_vel(1.0 - kEpsilon);
  EXPECT_TRUE(this->controller_->GetJointVelocities(joint_velocities));

  this->node_->set_parameter({rclcpp::Parameter("yaw_actual_velocity_threshold", -1.0)});
  EXPECT_TRUE(this->controller_->Init());

  this->hardware_.steer_handle->set_current_vel(1000.0 + kEpsilon);
  EXPECT_FALSE(this->controller_->GetJointVelocities(joint_velocities));

  this->hardware_.steer_handle->set_current_vel(1000.0 - kEpsilon);
  EXPECT_TRUE(this->controller_->GetJointVelocities(joint_velocities));
}


class OmniBaseJointControllerBaseRollPositionTest
    : public OmniBaseJointControllerBaseTest<TypePair<OmniBaseJointControllerBaseRollPosition,
                                                      CommandPositionHandle>> {
};

/// Ability to obtain interface names for sending commands to joints
TEST_F(OmniBaseJointControllerBaseRollPositionTest, GetCommandInterfaceNames) {
  EXPECT_TRUE(controller_->Init());

  auto names = controller_->command_interface_names();
  EXPECT_EQ(names.size(), 3);
  EXPECT_NE(std::find(names.begin(), names.end(), "base_roll_joint/position"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "base_l_drive_wheel_joint/velocity"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "base_r_drive_wheel_joint/velocity"), names.end());
}

/// Ability to obtain command values
TEST_F(OmniBaseJointControllerBaseRollPositionTest, GetJointCommand) {
  TestCommandBaseVelocity(Eigen::Vector3d(0.02, 0.03, 0.01),
                          Eigen::Vector3d(1.40682, -0.406818, 0.262727),
                          Eigen::Vector3d(1.40682, -0.406818, 0.0262727));
}

/// Ability to reset steering axis target position
TEST_F(OmniBaseJointControllerBaseRollPositionTest, ResetDesiredSteerPosition) {
  hardware_.steer_handle->set_current_pos(1.0);
  EXPECT_TRUE(controller_->Init());
  EXPECT_TRUE(controller_->Activate(hardware_.command_interfaces, hardware_.state_interfaces));
  EXPECT_DOUBLE_EQ(controller_->desired_steer_pos(), 1.0);

  hardware_.steer_handle->set_current_pos(2.0);
  controller_->ResetDesiredSteerPosition();
  EXPECT_DOUBLE_EQ(controller_->desired_steer_pos(), 2.0);
}


class OmniBaseJointControllerBaseRollVelocityTest
    : public OmniBaseJointControllerBaseTest<TypePair<OmniBaseJointControllerBaseRollVelocity,
                                                      CommandVelocityHandle>> {
};

/// Ability to obtain interface names for sending commands to joints
TEST_F(OmniBaseJointControllerBaseRollVelocityTest, GetCommandInterfaceNames) {
  EXPECT_TRUE(controller_->Init());

  auto names = controller_->command_interface_names();
  EXPECT_EQ(names.size(), 3);
  EXPECT_NE(std::find(names.begin(), names.end(), "base_roll_joint/velocity"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "base_l_drive_wheel_joint/velocity"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "base_r_drive_wheel_joint/velocity"), names.end());
}

/// Ability to obtain command values
TEST_F(OmniBaseJointControllerBaseRollVelocityTest, GetJointCommand) {
  TestCommandBaseVelocity(Eigen::Vector3d(0.02, 0.03, 0.01),
                          Eigen::Vector3d(1.40682, -0.406818, 0.262727),
                          Eigen::Vector3d(1.40682, -0.406818, 0.262727));
}

/// Steering axis target position is always zero
TEST_F(OmniBaseJointControllerBaseRollVelocityTest, ResetDesiredSteerPosition) {
  hardware_.steer_handle->set_current_pos(1.0);
  EXPECT_TRUE(controller_->Init());
  EXPECT_TRUE(controller_->Activate(hardware_.command_interfaces, hardware_.state_interfaces));
  EXPECT_DOUBLE_EQ(controller_->desired_steer_pos(), 0.0);

  hardware_.steer_handle->set_current_pos(2.0);
  controller_->ResetDesiredSteerPosition();
  EXPECT_DOUBLE_EQ(controller_->desired_steer_pos(), 0.0);
}

}  // namespace hsrb_base_controllers

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
