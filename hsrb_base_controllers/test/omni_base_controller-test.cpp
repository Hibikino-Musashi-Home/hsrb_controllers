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
/// @file omni_base_controller-test.cpp
/// @brief Test of all -sided bogie speed controller

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <hsrb_base_controllers/omni_base_controller.hpp>

#include "hardware_stub.hpp"
#include "utils.hpp"

namespace {

const char* const kControllerNodeName = "controller_manager";
const char* const kClientNodeName = "test_node";
constexpr double kEpsilon = 0.02;
constexpr double kUpdateFrequency = 100.0;
constexpr double kPositionErrorThreshold = 0.05;
constexpr double kVelocityErrorThreshold = 0.1;
constexpr double kWheelVelocityLimitThreshold = 8.5;
constexpr double kYawVelocityLimitThreshold = 1.8;

// Creating a test input orbit
trajectory_msgs::msg::JointTrajectory GetTestTrajectory() {
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"odom_x", "odom_y", "odom_t"};

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {0.1, 0.1, 0.3};
  point.velocities = {0.0, 0.0, 0.0};
  point.time_from_start = rclcpp::Duration(1, 0);
  trajectory.points.push_back(point);

  point.positions = {0.0, 0.0, 0.0};
  point.time_from_start = rclcpp::Duration(2, 0);
  trajectory.points.push_back(point);

  return trajectory;
}

}  // namespace

namespace hsrb_base_controllers {

class OmniBaseControllerTest : public ::testing::Test {
 public:
  void SetupController();

 protected:
  std::shared_ptr<OmniBaseController> controller_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr controller_node_;
  rclcpp::Node::SharedPtr client_node_;
  HardwareStub::Ptr hardware_;
  TopicRelay<nav_msgs::msg::Odometry>::Ptr odom_relay_;
  SubscriptionCounter<control_msgs::msg::JointTrajectoryControllerState>::Ptr state_counter_;
  rclcpp::Time last_update_time_;

  void SpinOnce(rclcpp::WallRate& rate, bool do_update = true);

  template <typename Action>
  bool WaitForStatus(typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr goal_handle,
                     int8_t expected, bool do_update = true);

  template <typename Handle>
  void WaitForReady(typename std::shared_future<Handle>& future, rclcpp::WallRate& rate);
};

void OmniBaseControllerTest::SetupController() {
  controller_ = std::make_shared<OmniBaseController>();

  rclcpp::NodeOptions options = rclcpp::NodeOptions()
      .allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
  options.append_parameter_override<std::vector<std::string>>("base_coordinates", {"odom_x", "odom_y", "odom_t"});
  options.append_parameter_override<std::string>("robot_description", GetRobotDescription());
  options.append_parameter_override<float>("odometry_publish_rate", 200.0);
  options.append_parameter_override<std::string>("joints.steer", "base_roll_joint");
  options.append_parameter_override<std::string>("joints.r_wheel", "base_r_drive_wheel_joint");
  options.append_parameter_override<std::string>("joints.l_wheel", "base_l_drive_wheel_joint");
  options.append_parameter_override<float>("odom_x.p_gain", 0.01);
  options.append_parameter_override<float>("odom_y.p_gain", 0.01);
  options.append_parameter_override<float>("odom_t.p_gain", 0.01);
  options.append_parameter_override<float>("constraints.odom_x.trajectory", 0.5);
  options.append_parameter_override<float>("constraints.goal_time", 0.5);

  EXPECT_EQ(controller_->init(kControllerNodeName, "", options), controller_interface::return_type::OK);
  EXPECT_EQ(controller_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  hardware_ = std::make_shared<HardwareStub>(kUpdateFrequency);
  controller_->assign_interfaces(std::move(hardware_->command_interfaces), std::move(hardware_->state_interfaces));
  EXPECT_EQ(controller_->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  client_node_ = rclcpp::Node::make_shared(kClientNodeName);
  odom_relay_ = std::make_shared<TopicRelay<nav_msgs::msg::Odometry>>(
      client_node_, std::string(kControllerNodeName) + "/wheel_odom", "odom");
  state_counter_ = std::make_shared<SubscriptionCounter<control_msgs::msg::JointTrajectoryControllerState>>(
      client_node_, std::string(kControllerNodeName) + "/state");

  controller_node_ = controller_->get_node();
  last_update_time_ = controller_node_->get_clock()->now();
}

void OmniBaseControllerTest::SpinOnce(rclcpp::WallRate& rate, bool do_update) {
  rate.sleep();
  rclcpp::spin_some(client_node_);
  rclcpp::spin_some(controller_node_->get_node_base_interface());
  const auto current_time = controller_node_->get_clock()->now();
  EXPECT_EQ(controller_->update(current_time, current_time - last_update_time_),
      controller_interface::return_type::OK);
  rclcpp::spin_some(client_node_);
  rclcpp::spin_some(controller_node_->get_node_base_interface());
  if (do_update) hardware_->Update();
  last_update_time_ = current_time;
}

template <typename Action>
bool OmniBaseControllerTest::WaitForStatus(typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr goal_handle,
                                           int8_t expected, bool do_update) {
  const auto end_time = std::chrono::system_clock::now() + std::chrono::duration<double>(5.0);
  rclcpp::WallRate loop_rate(kUpdateFrequency);
  while (goal_handle->get_status() != expected) {
    SpinOnce(loop_rate, do_update);
    if (std::chrono::system_clock::now() > end_time) {
      std::cout << "The last status is " << static_cast<int32_t>(goal_handle->get_status()) << std::endl;
      return false;
    }
  }
  return true;
}

template <typename Handle>
void OmniBaseControllerTest::WaitForReady(typename std::shared_future<Handle>& future, rclcpp::WallRate& rate) {
  const auto end_time = std::chrono::system_clock::now() + std::chrono::duration<double>(5.0);
  while (rclcpp::ok()) {
    if (std::chrono::system_clock::now() > end_time) {
      FAIL();
    }
    const auto result = future.wait_for(std::chrono::milliseconds(1));
    if (result == std::future_status::ready) {
      return;
    } else {
      SpinOnce(rate);
    }
  }
}

/// Give X and Y a equivalent speed command and move the bogie
TEST_F(OmniBaseControllerTest, CommandVelocity) {
  SetupController();
  auto publisher = client_node_->create_publisher<geometry_msgs::msg::Twist>(
      std::string(kControllerNodeName) + "/cmd_vel", rclcpp::SystemDefaultsQoS());
  auto wheel_odom_counter = std::make_shared<SubscriptionCounter<nav_msgs::msg::Odometry>>(
      client_node_, std::string(kControllerNodeName) + "/wheel_odom");

  geometry_msgs::msg::Twist command_velocity;
  command_velocity.linear.x = -0.1;
  command_velocity.linear.y = -0.05;

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  for (int i = 0; i < 199; ++i) {
    publisher->publish(command_velocity);
    SpinOnce(loop_rate);
  }

  // The default is 50Hz
  EXPECT_EQ(state_counter_->count(), 99);
  auto base_state = state_counter_->last_msg();

  ASSERT_EQ(base_state.joint_names.size(), 3);
  EXPECT_EQ(base_state.joint_names[0], "odom_x");
  EXPECT_EQ(base_state.joint_names[1], "odom_y");
  EXPECT_EQ(base_state.joint_names[2], "odom_t");

  ASSERT_EQ(base_state.actual.positions.size(), 3);
  EXPECT_NEAR(base_state.actual.positions[0], -0.2, kEpsilon);
  EXPECT_NEAR(base_state.actual.positions[1], -0.1, kEpsilon);
  EXPECT_NEAR(base_state.actual.positions[2], 0.0, kEpsilon);

  ASSERT_EQ(base_state.actual.velocities.size(), 3);
  EXPECT_NEAR(base_state.actual.velocities[0], -0.1, kEpsilon);
  EXPECT_NEAR(base_state.actual.velocities[1], -0.05, kEpsilon);
  EXPECT_NEAR(base_state.actual.velocities[2], 0.0, kEpsilon);

  auto wheel_odom = wheel_odom_counter->last_msg();
  EXPECT_EQ(wheel_odom.header.frame_id, "odom");
  EXPECT_EQ(wheel_odom.child_frame_id, "base_footprint_wheel");
  EXPECT_NEAR(wheel_odom.pose.pose.position.x, -0.2, kEpsilon);
  EXPECT_NEAR(wheel_odom.pose.pose.position.y, -0.1, kEpsilon);
  EXPECT_NEAR(wheel_odom.pose.pose.orientation.z, 0.0, kEpsilon);
  EXPECT_NEAR(wheel_odom.pose.pose.orientation.w, 1.0, kEpsilon);
  EXPECT_NEAR(wheel_odom.twist.twist.linear.x, -0.1, kEpsilon);
  EXPECT_NEAR(wheel_odom.twist.twist.linear.y, -0.05, kEpsilon);
  EXPECT_NEAR(wheel_odom.twist.twist.angular.z, 0.0, kEpsilon);
}

/// Whether you can correctly follow the test track entered in the topic
TEST_F(OmniBaseControllerTest, SendTrajectoryTopic) {
  SetupController();
  auto publisher = client_node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      std::string(kControllerNodeName) + "/joint_trajectory", rclcpp::SystemDefaultsQoS());
  publisher->publish(GetTestTrajectory());

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  for (int i = 0; i < 100; ++i) {
    SpinOnce(loop_rate);
  }
  auto base_state = state_counter_->last_msg();
  ASSERT_EQ(base_state.actual.positions.size(), 3);
  EXPECT_NEAR(base_state.actual.positions[0], 0.1, kEpsilon);
  EXPECT_NEAR(base_state.actual.positions[1], 0.1, kEpsilon);
  EXPECT_NEAR(base_state.actual.positions[2], 0.3, kEpsilon);

  for (int i = 0; i < 100; ++i) {
    SpinOnce(loop_rate);

    // It existed in the original test, so check it out.
    auto state = state_counter_->last_msg();
    for (int i = 0; i < 3; ++i) {
      ASSERT_LT(fabs(state.error.positions[i]),  kPositionErrorThreshold);
      ASSERT_LT(fabs(state.error.velocities[i]), kVelocityErrorThreshold);
    }
  }
  base_state = state_counter_->last_msg();
  ASSERT_EQ(base_state.actual.positions.size(), 3);
  EXPECT_NEAR(base_state.actual.positions[0], 0.0, kEpsilon);
  EXPECT_NEAR(base_state.actual.positions[1], 0.0, kEpsilon);
  EXPECT_NEAR(base_state.actual.positions[2], 0.0, kEpsilon);

  ASSERT_EQ(base_state.actual.velocities.size(), 3);
  EXPECT_NEAR(base_state.actual.velocities[0], 0.0, kVelocityErrorThreshold);
  EXPECT_NEAR(base_state.actual.velocities[1], 0.0, kVelocityErrorThreshold);
  EXPECT_NEAR(base_state.actual.velocities[2], 0.0, kVelocityErrorThreshold);
}

/// Whether you can correctly follow the test track entered via the action
TEST_F(OmniBaseControllerTest, SendTrajectoryAction) {
  SetupController();
  using ActionType = control_msgs::action::FollowJointTrajectory;
  auto action_client = rclcpp_action::create_client<ActionType>(
      client_node_, std::string(kControllerNodeName) + "/follow_joint_trajectory");
  EXPECT_TRUE(action_client->wait_for_action_server());

  ActionType::Goal goal;
  goal.trajectory = GetTestTrajectory();

  ActionType::Result result;
  auto result_callback = [&result](const rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult& _result) {
    result = *_result.result;
  };
  ActionType::Feedback feedback;
  auto feedback_callback = [&feedback](rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr,
                                       const std::shared_ptr<const ActionType::Feedback> _feedback) {
    feedback = *_feedback;
  };

  auto send_goal_options = rclcpp_action::Client<ActionType>::SendGoalOptions();
  send_goal_options.result_callback = result_callback;
  send_goal_options.feedback_callback = feedback_callback;

  auto future_goal_handle = action_client->async_send_goal(goal, send_goal_options);

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  for (int i = 0; i < 100; ++i) {
    SpinOnce(loop_rate);
  }
  auto base_state = state_counter_->last_msg();
  ASSERT_EQ(base_state.actual.positions.size(), 3);
  EXPECT_NEAR(base_state.actual.positions[0], 0.1, kEpsilon);
  EXPECT_NEAR(base_state.actual.positions[1], 0.1, kEpsilon);
  EXPECT_NEAR(base_state.actual.positions[2], 0.3, kEpsilon);

  ASSERT_EQ(feedback.actual.positions.size(), 3);
  EXPECT_NEAR(feedback.actual.positions[0], 0.1, kEpsilon);
  EXPECT_NEAR(feedback.actual.positions[1], 0.1, kEpsilon);
  EXPECT_NEAR(feedback.actual.positions[2], 0.3, kEpsilon);

  EXPECT_EQ(feedback.actual.velocities.size(), 3);
  EXPECT_TRUE(feedback.actual.accelerations.empty());

  ASSERT_EQ(feedback.desired.positions.size(), 3);
  EXPECT_NEAR(feedback.desired.positions[0], 0.1, kEpsilon);
  EXPECT_NEAR(feedback.desired.positions[1], 0.1, kEpsilon);
  EXPECT_NEAR(feedback.desired.positions[2], 0.3, kEpsilon);

  EXPECT_EQ(feedback.desired.velocities.size(), 3);
  EXPECT_EQ(feedback.desired.accelerations.size(), 3);

  ASSERT_EQ(feedback.error.positions.size(), 3);
  EXPECT_NEAR(feedback.error.positions[0], 0.0, kEpsilon);
  EXPECT_NEAR(feedback.error.positions[1], 0.0, kEpsilon);
  EXPECT_NEAR(feedback.error.positions[2], 0.0, kEpsilon);

  ASSERT_EQ(feedback.error.velocities.size(), 3);
  EXPECT_NEAR(feedback.error.velocities[0], 0.0, kEpsilon);
  EXPECT_NEAR(feedback.error.velocities[1], 0.0, kEpsilon);
  EXPECT_NEAR(feedback.error.velocities[2], 0.0, kEpsilon);

  EXPECT_TRUE(feedback.error.accelerations.empty());

  for (int i = 0; i < 100; ++i) {
    SpinOnce(loop_rate);
  }

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(WaitForStatus<ActionType>(goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));

  SpinOnce(loop_rate);
  EXPECT_EQ(result.error_code, control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL);

  base_state = state_counter_->last_msg();
  ASSERT_EQ(base_state.actual.positions.size(), 3);
  EXPECT_NEAR(base_state.actual.positions[0], 0.0, kEpsilon);
  EXPECT_NEAR(base_state.actual.positions[1], 0.0, kEpsilon);
  EXPECT_NEAR(base_state.actual.positions[2], 0.0, kEpsilon);
}

/// Is it possible to properly generate or follow the order even if the command value exceeds the PI or higher in the rotation direction
TEST_F(OmniBaseControllerTest, OverPISteerTrajectory) {
  SetupController();
  using ActionType = control_msgs::action::FollowJointTrajectory;
  auto action_client = rclcpp_action::create_client<ActionType>(
      client_node_, std::string(kControllerNodeName) + "/follow_joint_trajectory");
  EXPECT_TRUE(action_client->wait_for_action_server());

  ActionType::Goal goal;
  goal.trajectory.joint_names = {"odom_x", "odom_y", "odom_t"};

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {0.0, 0.0, 3.0};
  point.time_from_start = rclcpp::Duration(3, 0);
  goal.trajectory.points.push_back(point);

  point.positions = {0.0, 0.0, -3.0};
  point.time_from_start = rclcpp::Duration(4, 0);
  goal.trajectory.points.push_back(point);

  point.positions = {0.0, 0.0, -2.5};
  point.time_from_start = rclcpp::Duration(5, 0);
  goal.trajectory.points.push_back(point);

  auto future_goal_handle = action_client->async_send_goal(goal);

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  for (int i = 0; i < 300; ++i) {
    SpinOnce(loop_rate);
  }
  auto base_state = state_counter_->last_msg();
  ASSERT_EQ(base_state.actual.positions.size(), 3);
  EXPECT_GT(base_state.actual.positions[2], 2.9);

  for (int i = 0; i < 100; ++i) {
    SpinOnce(loop_rate);
  }
  base_state = state_counter_->last_msg();
  ASSERT_EQ(base_state.actual.positions.size(), 3);
  EXPECT_LT(base_state.actual.positions[2], -2.9);

  for (int i = 0; i < 120; ++i) {
    SpinOnce(loop_rate);
  }

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(WaitForStatus<ActionType>(goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));

  base_state = state_counter_->last_msg();
  ASSERT_EQ(base_state.actual.positions.size(), 3);
  EXPECT_NEAR(base_state.actual.positions[2], -2.5, kEpsilon);
}

/// Can you stop properly when you can't follow the test track entered via a topic?
TEST_F(OmniBaseControllerTest, StopFollowingInTopic) {
  SetupController();
  auto publisher = client_node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      std::string(kControllerNodeName) + "/joint_trajectory", rclcpp::SystemDefaultsQoS());

  auto trajectory = GetTestTrajectory();
  trajectory.points[0].positions[0] = 5.0;
  publisher->publish(trajectory);

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  double max_error = 0.0;
  for (int i = 0; i < 100; ++i) {
    SpinOnce(loop_rate);

    auto base_state = state_counter_->last_msg();
    if (base_state.error.positions.size() == 3) {
      max_error = std::max(max_error, std::abs(base_state.error.positions[0]));
    }
  }
  EXPECT_GT(max_error, 0.5);

  // Appropriate threshold to check that it was stopped without much progress
  auto base_state = state_counter_->last_msg();
  EXPECT_LT(base_state.actual.positions[0], 0.2);

  ASSERT_EQ(base_state.actual.velocities.size(), 3);
  EXPECT_NEAR(base_state.actual.velocities[0], 0.0, kEpsilon);
  EXPECT_NEAR(base_state.actual.velocities[1], 0.0, kEpsilon);
  EXPECT_NEAR(base_state.actual.velocities[2], 0.0, kEpsilon);
}

/// Can you stop properly when you can't follow the test trajectory entered via an action
TEST_F(OmniBaseControllerTest, StopFollowingInAction) {
  SetupController();
  using ActionType = control_msgs::action::FollowJointTrajectory;
  auto action_client = rclcpp_action::create_client<ActionType>(
      client_node_, std::string(kControllerNodeName) + "/follow_joint_trajectory");
  EXPECT_TRUE(action_client->wait_for_action_server());

  ActionType::Goal goal;
  goal.trajectory = GetTestTrajectory();
  goal.trajectory.points[0].positions[0] = 5.0;

  ActionType::Result result;
  auto result_callback = [&result](const rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult& _result) {
    result = *_result.result;
  };
  auto send_goal_options = rclcpp_action::Client<ActionType>::SendGoalOptions();
  send_goal_options.result_callback = result_callback;

  auto future_goal_handle = action_client->async_send_goal(goal, send_goal_options);

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  for (int i = 0; i < 100; ++i) {
    SpinOnce(loop_rate);
  }

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(WaitForStatus<ActionType>(goal_handle, action_msgs::msg::GoalStatus::STATUS_ABORTED));

  SpinOnce(loop_rate);
  EXPECT_EQ(result.error_code, control_msgs::action::FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED);
}

/// Can I correctly return the failure when the accuracy is not enough for the goal of the test orbit entered via the action
TEST_F(OmniBaseControllerTest, OverGoalTolerance) {
  SetupController();
  using ActionType = control_msgs::action::FollowJointTrajectory;
  auto action_client = rclcpp_action::create_client<ActionType>(
      client_node_, std::string(kControllerNodeName) + "/follow_joint_trajectory");
  EXPECT_TRUE(action_client->wait_for_action_server());

  ActionType::Goal goal;
  goal.trajectory = GetTestTrajectory();

  ActionType::Result result;
  auto result_callback = [&result](const rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult& _result) {
  result = *_result.result;
  };
  auto send_goal_options = rclcpp_action::Client<ActionType>::SendGoalOptions();
  send_goal_options.result_callback = result_callback;

  auto future_goal_handle = action_client->async_send_goal(goal, send_goal_options);

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  for (int i = 0; i < 200; ++i) {
  SpinOnce(loop_rate);
  }

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(WaitForStatus<ActionType>(goal_handle, action_msgs::msg::GoalStatus::STATUS_ABORTED, false));

  SpinOnce(loop_rate);
  EXPECT_EQ(result.error_code, control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED);
}

/// Action cancellation test
TEST_F(OmniBaseControllerTest, ActionCancel) {
  SetupController();
  using ActionType = control_msgs::action::FollowJointTrajectory;
  auto action_client = rclcpp_action::create_client<ActionType>(
      client_node_, std::string(kControllerNodeName) + "/follow_joint_trajectory");
  EXPECT_TRUE(action_client->wait_for_action_server());

  ActionType::Goal goal;
  goal.trajectory = GetTestTrajectory();

  auto future_goal_handle = action_client->async_send_goal(goal);

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  WaitForReady(future_goal_handle, loop_rate);

  auto goal_handle = future_goal_handle.get();
  auto future_cancel = action_client->async_cancel_goal(goal_handle);
  WaitForReady(future_cancel, loop_rate);

  auto cancel_response = future_cancel.get();
  EXPECT_EQ(cancel_response->return_code, action_msgs::srv::CancelGoal::Response::ERROR_NONE);

  EXPECT_TRUE(WaitForStatus<ActionType>(goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));
}

/// Continue Goal and send it for 2 times (the first is ClearactiveGoal)
TEST_F(OmniBaseControllerTest, SendGoalTwice) {
  SetupController();
  using ActionType = control_msgs::action::FollowJointTrajectory;
  auto action_client = rclcpp_action::create_client<ActionType>(
      client_node_, std::string(kControllerNodeName) + "/follow_joint_trajectory");
  EXPECT_TRUE(action_client->wait_for_action_server());

  ActionType::Goal goal;
  goal.trajectory = GetTestTrajectory();

  auto future_goal_handle_first = action_client->async_send_goal(goal);

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  WaitForReady(future_goal_handle_first, loop_rate);

  auto future_goal_handle_second = action_client->async_send_goal(goal);
  WaitForReady(future_goal_handle_second, loop_rate);

  auto goal_handle_first = future_goal_handle_first.get();
  EXPECT_TRUE(WaitForStatus<ActionType>(goal_handle_first, action_msgs::msg::GoalStatus::STATUS_CANCELED));

  auto goal_handle_second = future_goal_handle_second.get();
  EXPECT_TRUE(WaitForStatus<ActionType>(goal_handle_second, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));
}

/// Will Configure fail when there is no bogie coordinate axis parameter?
TEST_F(OmniBaseControllerTest, NoOdomCoordParameter) {
  controller_ = std::make_shared<OmniBaseController>();

  rclcpp::NodeOptions options = rclcpp::NodeOptions()
      .allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
  options.append_parameter_override<std::string>("robot_description", GetRobotDescription());
  options.append_parameter_override<float>("odometry_publish_rate", 200.0);
  options.append_parameter_override<std::string>("joints.steer", "base_roll_joint");
  options.append_parameter_override<std::string>("joints.r_wheel", "base_r_drive_wheel_joint");
  options.append_parameter_override<std::string>("joints.l_wheel", "base_l_drive_wheel_joint");
  options.append_parameter_override<float>("odom_x.p_gain", 0.01);
  options.append_parameter_override<float>("odom_y.p_gain", 0.01);
  options.append_parameter_override<float>("odom_t.p_gain", 0.01);
  options.append_parameter_override<float>("constraints.odom_x.trajectory", 0.5);
  options.append_parameter_override<float>("constraints.goal_time", 0.5);

  EXPECT_EQ(controller_->init(kControllerNodeName, "", options), controller_interface::return_type::OK);
  EXPECT_EQ(controller_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

/// Send a GOAL that does not set orbit
TEST_F(OmniBaseControllerTest, EmptyTrajectoryGoal) {
  SetupController();
  using ActionType = control_msgs::action::FollowJointTrajectory;
  auto action_client = rclcpp_action::create_client<ActionType>(
      client_node_, std::string(kControllerNodeName) + "/follow_joint_trajectory");
  EXPECT_TRUE(action_client->wait_for_action_server());

  ActionType::Goal goal;

  auto future_goal_handle = action_client->async_send_goal(goal);
  rclcpp::WallRate loop_rate(kUpdateFrequency);
  for (int i = 0; i < 10; ++i) {
    SpinOnce(loop_rate);
  }
  auto goal_handle = future_goal_handle.get();
  EXPECT_EQ(goal_handle, nullptr);
}

/// Whether the speed limit will be applied when the speed command exceeds the threshold
TEST_F(OmniBaseControllerTest, VelocityLimit) {
  SetupController();
  auto publisher = client_node_->create_publisher<geometry_msgs::msg::Twist>(
      std::string(kControllerNodeName) + "/cmd_vel", rclcpp::SystemDefaultsQoS());
  auto internal_state_counter =
      std::make_shared<SubscriptionCounter<control_msgs::msg::JointTrajectoryControllerState>>(
          client_node_, std::string(kControllerNodeName) + "/internal_state");

  for (int32_t i = 0; i < 4; i++) {
    geometry_msgs::msg::Twist command_velocity;
    switch (i) {
      case 0:
        // Positive speed beyond the limit in the x direction
        command_velocity.linear.x = 10.0;
        command_velocity.linear.y = 0.0;
        command_velocity.angular.z = 0.0;
        break;
      case 1:
        // Negative speed exceeding the limit in the Y direction
        command_velocity.linear.x = 0.0;
        command_velocity.linear.y = -10.0;
        command_velocity.angular.z = 0.0;
        break;
      case 2:
        // Speed ​​exceeding the limit in the YAW direction
        command_velocity.linear.x = 0.0;
        command_velocity.linear.y = 0.0;
        command_velocity.angular.z = 40.0;
        break;
      case 3:
        // Speed ​​exceeding the limit in the x, yaw direction (whether the limit is applied twice)
        command_velocity.linear.x = 10.0;
        command_velocity.linear.y = 0.0;
        command_velocity.angular.z = 40.0;
        break;
      default:
        break;
    }

    rclcpp::WallRate loop_rate(kUpdateFrequency);
    for (int i = 0; i < 100; ++i) {
      publisher->publish(command_velocity);
      SpinOnce(loop_rate);

      auto state = internal_state_counter->last_msg();
      if (state.desired.velocities.size() == 3) {
        // Compare with a buffer of 1 / 10,000 in consideration of the decimal error.
        ASSERT_LE(std::abs(state.desired.velocities[0]), kWheelVelocityLimitThreshold * 1.0001);
        ASSERT_LE(std::abs(state.desired.velocities[1]), kWheelVelocityLimitThreshold * 1.0001);
        ASSERT_LE(std::abs(state.desired.velocities[2]), kYawVelocityLimitThreshold * 1.0001);
      }
    }
  }
}

/// Testing of bogie control means
TEST_F(OmniBaseControllerTest, ChangeControlMethod) {
  SetupController();
  // First, speed
  auto vel_publisher = client_node_->create_publisher<geometry_msgs::msg::Twist>(
      std::string(kControllerNodeName) + "/cmd_vel", rclcpp::SystemDefaultsQoS());

  geometry_msgs::msg::Twist command_velocity;
  command_velocity.linear.x = 0.1;
  command_velocity.linear.y = 0.1;

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  for (int i = 0; i < 200; ++i) {
    vel_publisher->publish(command_velocity);
    SpinOnce(loop_rate);
  }

  auto base_state = state_counter_->last_msg();
  ASSERT_EQ(base_state.actual.positions.size(), 3);
  EXPECT_NEAR(base_state.actual.positions[0], 0.2, kEpsilon);
  EXPECT_NEAR(base_state.actual.positions[1], 0.2, kEpsilon);
  EXPECT_NEAR(base_state.actual.positions[2], 0.0, kEpsilon);

  // Track from here
  auto trj_publisher = client_node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      std::string(kControllerNodeName) + "/joint_trajectory", rclcpp::SystemDefaultsQoS());
  trj_publisher->publish(GetTestTrajectory());

  for (int i = 0; i < 100; ++i) {
    SpinOnce(loop_rate);
  }

  base_state = state_counter_->last_msg();
  ASSERT_EQ(base_state.actual.positions.size(), 3);
  EXPECT_NEAR(base_state.actual.positions[0], 0.1, kEpsilon);
  EXPECT_NEAR(base_state.actual.positions[1], 0.1, kEpsilon);
  EXPECT_NEAR(base_state.actual.positions[2], 0.3, kEpsilon);

  for (int i = 0; i < 100; ++i) {
    SpinOnce(loop_rate);
  }

  base_state = state_counter_->last_msg();
  ASSERT_EQ(base_state.actual.positions.size(), 3);
  EXPECT_NEAR(base_state.actual.positions[0], 0.0, kEpsilon);
  EXPECT_NEAR(base_state.actual.positions[1], 0.0, kEpsilon);
  EXPECT_NEAR(base_state.actual.positions[2], 0.0, kEpsilon);

  // Also speed
  for (int i = 0; i < 200; ++i) {
    vel_publisher->publish(command_velocity);
    SpinOnce(loop_rate);
  }

  base_state = state_counter_->last_msg();
  ASSERT_EQ(base_state.actual.positions.size(), 3);
  EXPECT_NEAR(base_state.actual.positions[0], 0.2, kEpsilon);
  EXPECT_NEAR(base_state.actual.positions[1], 0.2, kEpsilon);
  EXPECT_NEAR(base_state.actual.positions[2], 0.0, kEpsilon);
}

}  // namespace hsrb_base_controllers

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
