/*
Copyright (c) 2022 TOYOTA MOTOR CORPORATION
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
/// @brief Utility functions and classes for testing

#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hsrb_gripper_controller/hrh_gripper_action.hpp>
#include <hsrb_gripper_controller/hrh_gripper_controller.hpp>

#include "hardware_stub.hpp"

namespace hsrb_gripper_controller {

const char* const kControllerNodeName = "controller_manager";
const char* const kClientNodeName = "test_node";

const char* const kHandJointName = "hand_motor_joint";

constexpr double kEpsilon = 1e-6;


template <typename Action, typename NodeType>
bool WaitForStatus(const std::shared_ptr<HrhGripperController>& controller,
                   const typename std::vector<NodeType>& node_ptrs,
                   typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr goal_handle, int8_t expected) {
  const auto end_time = std::chrono::system_clock::now() + std::chrono::duration<double>(1.0);
  while (goal_handle->get_status() != expected) {
    for (auto node_ptr : node_ptrs) {
      rclcpp::spin_some(node_ptr);
    }
    controller->update(controller->get_node()->now(), rclcpp::Duration::from_seconds(0.1));
    if (std::chrono::system_clock::now() > end_time) {
      std::cout << "The last status is " << static_cast<int32_t>(goal_handle->get_status()) << std::endl;
      return false;
    }
  }
  return true;
}

trajectory_msgs::msg::JointTrajectory MakeTrajectory(double command_position = 1.0) {
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = { command_position };
  point.time_from_start = rclcpp::Duration(0, 500000000);

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = { kHandJointName };
  trajectory.points = { point };
  return trajectory;
}

trajectory_msgs::msg::JointTrajectory MakeDistanceTrajectory(double command_distance = 0.1) {
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = { command_distance };
  point.time_from_start = rclcpp::Duration(0, 500000000);

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = { kHandJointName };
  trajectory.points = { point };
  return trajectory;
}

std::string ReadRobotDescriptionFromFile() {
  std::fstream xml_file("robot.xml", std::fstream::in);
  std::string robot_description;
  while (xml_file.good()) {
    std::string line;
    std::getline(xml_file, line);
    robot_description += (line + "\n");
  }
  xml_file.close();
  return robot_description;
}


class TestableHrhGripperController : public HrhGripperController {
 public:
  using Ptr = std::shared_ptr<TestableHrhGripperController>;

  TestableHrhGripperController() : TestableHrhGripperController(rclcpp::NodeOptions()) {}
  explicit TestableHrhGripperController(rclcpp::NodeOptions node_options);
  ~TestableHrhGripperController() = default;

  controller_interface::return_type init(const std::string& controller_name, const std::string& namespace_ = "",
                                         const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions()) override;

  void SkipConfigure();
};

TestableHrhGripperController::TestableHrhGripperController(rclcpp::NodeOptions node_options) {
  node_options.append_parameter_override<std::vector<std::string> >("joints", { kHandJointName });
  node_options.append_parameter_override<std::string>("left_spring_joint", "hand_l_spring_proximal_joint");
  node_options.append_parameter_override<std::string>("right_spring_joint", "hand_r_spring_proximal_joint");
  node_options.append_parameter_override<bool>("do_output_position_control", true);
  node_options.append_parameter_override<double>("position_control_current_min", -1.0);
  EXPECT_EQ(ControllerInterface::init(kControllerNodeName, "", node_options),
            controller_interface::return_type::OK);
}

controller_interface::return_type TestableHrhGripperController::init(const std::string& controller_name,
                                                                     const std::string& namespace_,
                                                                     const rclcpp::NodeOptions& node_options) {
  if (InitImpl()) {
    return controller_interface::return_type::OK;
  } else {
    return controller_interface::return_type::ERROR;
  }
}

void TestableHrhGripperController::SkipConfigure() {
}

template <typename RosActionType>
class GripperActionTestBase : public ::testing::Test {
 public:
  explicit GripperActionTestBase(const std::string& action_name) : action_name_(action_name) {}
  virtual ~GripperActionTestBase() = default;

  void SetUp() override;

 protected:
  void StartupController();
  void StartupController(rclcpp::NodeOptions node_options);

  TestableHrhGripperController::Ptr controller_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  HardwareStub::Ptr hardware_;

  using ActionType = RosActionType;
  std::string action_name_;
  typename rclcpp_action::Client<RosActionType>::SharedPtr action_client_;
};

template <typename RosActionType>
void GripperActionTestBase<RosActionType>::SetUp() {
  hardware_ = std::make_shared<HardwareStub>(kHandJointName);
}

template <typename RosActionType>
void GripperActionTestBase<RosActionType>::StartupController() {
  StartupController(rclcpp::NodeOptions());
}

template <typename RosActionType>
void GripperActionTestBase<RosActionType>::StartupController(rclcpp::NodeOptions node_options) {
  node_options.append_parameter_override<std::string>("robot_description", ReadRobotDescriptionFromFile());
  controller_ = std::make_shared<TestableHrhGripperController>(node_options);
  node_ = controller_->get_node();

  ASSERT_EQ(controller_->init(kControllerNodeName), controller_interface::return_type::OK);

  controller_->assign_interfaces(std::move(hardware_->command_interfaces), std::move(hardware_->state_interfaces));
  ASSERT_EQ(controller_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(controller_->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  action_client_ =
      rclcpp_action::create_client<RosActionType>(node_, std::string(kControllerNodeName) + "/" + action_name_);
  ASSERT_TRUE(action_client_->wait_for_action_server());
}

}  // namespace hsrb_gripper_controller
