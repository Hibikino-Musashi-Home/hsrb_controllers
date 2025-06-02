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
#include "hsrb_gripper_controller/hrh_gripper_set_distance_action.hpp"

#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>

#include <tmc_exxx_servo_motor_protocol/exxx_common.hpp>

#include "hsrb_gripper_controller/hrh_gripper_controller.hpp"


namespace {

// Default goal tolerance [m]
const double kDefaultDistanceGoalTolerance = 0.003;
// Default stall determination speed threshold [rad/s]
const double kDefaultStallVelocityThreshold = 0.05;
// Default arrival determination time [s]
const double kDefaultDistanceControlStallTimeout = 1.0;
// Default opening width control P gain
const double kDefaultDistanceControlPgain = 2.0;
// Default opening width control I gain
const double kDefaultDistanceControlIgain = 0.5;
// Default opening width control D gain
const double kDefaultDistanceControlDgain = 2.5;
// Default hand upper limit angle [rad]
const double kDefaultHandMotorJointMax = 1.2;
// Default hand lower limit angle [rad]
const double kDefaultHandMotorJointMin = -0.5;
// Default physical parameters of the hand
const double kDefaultProximalToDistalZ = 0.07;
const double kDefaultDistancePalmToTip = 0.002194;
// Default name of the URDF robot model to load
const char* kDefaultRobotModelName = "robot_description";
// Default name of the node that loads the URDF robot model
const char* kDefaultRobotModelNode = "robot_state_publisher";
// Default name of the axis to read from the URDF robot model
const char* kDefaultProximalJointName = "hand_l_proximal_joint";
const char* kDefaultDistalJointName = "hand_l_distal_joint";
const char* kDefaultMimicDistalJointName = "hand_l_mimic_distal_joint";
const char* kDefaultFingerTipFrameJointName = "hand_l_finger_tip_frame_joint";

}  // unnamed namespace

namespace hsrb_gripper_controller {

HrhGripperSetDistanceCalculator::HrhGripperSetDistanceCalculator()
    : proximal_to_distal_z_(kDefaultProximalToDistalZ),
      distance_palm_to_tip_(kDefaultDistancePalmToTip) {}

bool HrhGripperSetDistanceCalculator::InitializeHandSizeData(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
  // URDF loading
  auto urdf = std::make_shared<urdf::Model>();
  if (!urdf->initString(GetRobotDescription(node))) {
    RCLCPP_ERROR(node->get_logger(), "Failed to parse URDF");
    return false;
  }

  // Axis name to retrieve
  auto proximal_joint_name = GetParameter(node, "proximal_joint", kDefaultProximalJointName);
  auto distal_joint_name = GetParameter(node, "distal_joint", kDefaultDistalJointName);
  auto mimic_distal_joint_name = GetParameter(node, "mimic_distal_joint", kDefaultMimicDistalJointName);
  auto finger_tip_frame_joint_name =
      GetParameter(node, "finger_tip_frame_joint", kDefaultFingerTipFrameJointName);

  // Retrieve each axis from the URDF
  auto proximal_joint = urdf->getJoint(proximal_joint_name);
  auto distal_joint = urdf->getJoint(distal_joint_name);
  auto mimic_distal_joint = urdf->getJoint(mimic_distal_joint_name);
  auto finger_tip_frame_joint = urdf->getJoint(finger_tip_frame_joint_name);
  if (!proximal_joint || !distal_joint || !mimic_distal_joint || !finger_tip_frame_joint) {
    RCLCPP_ERROR(node->get_logger(), "Could not get joint param from urdf");
    return false;
  }

  // Retrieve necessary parameters
  double distal_to_tip_y = fabs(finger_tip_frame_joint->parent_to_joint_origin_transform.position.y);
  double distal_to_tip_z = fabs(finger_tip_frame_joint->parent_to_joint_origin_transform.position.z);
  double distal_joint_angle_offset = fabs(distal_joint->mimic->offset);
  double palm_to_proximal_y = fabs(proximal_joint->parent_to_joint_origin_transform.position.y);

  // Store values used for calculating opening width
  proximal_to_distal_z_ = fabs(mimic_distal_joint->parent_to_joint_origin_transform.position.z);
  distance_palm_to_tip_ = palm_to_proximal_y
                          - (distal_to_tip_y * cos(distal_joint_angle_offset)
                             + distal_to_tip_z * sin(distal_joint_angle_offset));

  return true;
}

// Load URDF
std::string HrhGripperSetDistanceCalculator::GetRobotDescription(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
  // First try to load from one's own node, if not possible, try loading from model_node_name
  // In the case of Gazebo, since robot_description cannot be placed in controller_manager, it must be loaded from a separate node
  const std::string model_name = GetParameter(node, "model_name", kDefaultRobotModelName);
  const std::string robot_description_out = GetParameter(node, model_name, "");
  if (!robot_description_out.empty()) {
    return robot_description_out;
  }

  const std::string model_node_name = GetParameter(node, "model_node_name", kDefaultRobotModelNode);
  const int32_t timeout = GetParameter(node, "parameter_connection_timeout", 60);
  // Temporarily generate a node object.
  std::string node_name = std::string(node->get_name());
  auto gripper_controller_node = rclcpp_lifecycle::LifecycleNode::make_shared(node_name);
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
      gripper_controller_node, model_node_name);
  int32_t wait_for_service_count = 0;
  while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
    ++wait_for_service_count;
    if (!rclcpp::ok()) {
      return "";
    } else if (wait_for_service_count >= timeout) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Could not connect parameter server of " << model_node_name);
      return "";
    }
  }
  return parameters_client->get_parameter(model_name, std::string());
}

/// Calculate opening width from hand angle
double HrhGripperSetDistanceCalculator::GetDistanceFromPosition(double hand_motor_pos) const {
  return GetDistanceFromPosition(hand_motor_pos, 0.0, 0.0);
}

/// Calculate opening width from hand angle and each finger angle
double HrhGripperSetDistanceCalculator::GetDistanceFromPosition(
    double hand_motor_pos, double left_spring_proximal_joint_pos,
    double right_spring_proximal_joint_pos) const {
  double hand_left_position = left_spring_proximal_joint_pos + hand_motor_pos;
  double hand_right_position = right_spring_proximal_joint_pos + hand_motor_pos;
  double ploximal_to_distal = proximal_to_distal_z_ * (sin(hand_left_position) + sin(hand_right_position));
  return ploximal_to_distal + 2.0 * distance_palm_to_tip_;
}

/// Calculate hand angle from opening width
double HrhGripperSetDistanceCalculator::GetPositionFromDistance(double distance) const {
  return asin((distance / 2.0 - distance_palm_to_tip_) / proximal_to_distal_z_);
}

/// Constructor
HrhGripperSetDistanceAction::HrhGripperSetDistanceAction(HrhGripperController* controller)
    : HrhGripperAction(controller, "~/set_distance", tmc_exxx_servo_motor_protocol::kDriveModeHandPosition),
      goal_tolerance_(kDefaultDistanceGoalTolerance),
      stall_velocity_threshold_(kDefaultStallVelocityThreshold),
      distance_control_stall_timeout_(kDefaultDistanceControlStallTimeout),
      distance_control_pgain_(kDefaultDistanceControlPgain),
      distance_control_igain_(kDefaultDistanceControlIgain),
      distance_control_dgain_(kDefaultDistanceControlDgain),
      integrated_distance_error_(0.0),
      last_error_(0.0) {}

/// Periodic update process
void HrhGripperSetDistanceAction::Update(const rclcpp::Time& time) {
  if (!IsActive() && *(stop_flag_buffer_.readFromRT())) {
    return;
  }

  // Calculate current fingertip distance
  double current_distance =
      distance_calculator_->GetDistanceFromPosition(controller_->GetCurrentPosition(),
                                                    controller_->GetLeftSpringPosition(),
                                                    controller_->GetRightSpringPosition());

  // Send feedback
  const auto active_goal_handle = *goal_handle_buffer_.readFromNonRT();
  if (active_goal_handle) {
    const auto feedback = std::make_shared<tmc_control_msgs::action::GripperSetDistance::Feedback>();
    feedback->distance = current_distance;
    active_goal_handle->setFeedback(feedback);
  }

  // Calculate and set command value
  controller_->SetComandPosition(GetCommandPos(current_distance));

  // Success or failure determination
  CheckForSuccess(time, current_distance);
}

/// Cancel ongoing action
void HrhGripperSetDistanceAction::PreemptActiveGoal() {
  HrhGripperAction::PreemptActiveGoal();
  stop_flag_buffer_.writeFromNonRT(true);
}

/// Implement action initialization
bool HrhGripperSetDistanceAction::InitImpl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
  // Set parameters
  goal_tolerance_ =
      GetPositiveParameter(node, "distance_goal_tolerance", kDefaultDistanceGoalTolerance);
  stall_velocity_threshold_ =
      GetPositiveParameter(node, "stall_velocity_threshold", kDefaultStallVelocityThreshold);
  distance_control_stall_timeout_ =
      GetPositiveParameter(node, "distance_control_stall_timeout", kDefaultDistanceControlStallTimeout);
  distance_control_pgain_ =
      GetNonNegativeParameter(node, "distance_control_pgain", kDefaultDistanceControlPgain);
  distance_control_igain_ =
      GetNonNegativeParameter(node, "distance_control_igain", kDefaultDistanceControlIgain);
  distance_control_dgain_ =
      GetNonNegativeParameter(node, "distance_control_dgain", kDefaultDistanceControlDgain);
  hand_motor_joint_max_ = GetParameter(node, "hand_motor_joint_max", kDefaultHandMotorJointMax);
  hand_motor_joint_min_ = GetParameter(node, "hand_motor_joint_min", kDefaultHandMotorJointMin);

  // Initialize member variables
  goal_buffer_.initRT(0.0);
  stop_flag_buffer_.initRT(true);

  // Initialize class for calculating opening width
  distance_calculator_ = std::make_shared<HrhGripperSetDistanceCalculator>();
  if (!distance_calculator_->InitializeHandSizeData(node)) {
    return false;
  }

  // Calculate upper and lower limits of opening width
  distance_max_ = distance_calculator_->GetDistanceFromPosition(hand_motor_joint_max_);
  distance_min_ = distance_calculator_->GetDistanceFromPosition(hand_motor_joint_min_);

  // Receive opening width
  distance_command_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "~/command_distance",
      1,
      std::bind(&HrhGripperSetDistanceAction::DistanceCommandCallback, this, std::placeholders::_1));

  return true;
}

/// Update action target
void HrhGripperSetDistanceAction::UpdateActionImpl(
    const tmc_control_msgs::action::GripperSetDistance::Goal& goal) {
  SetCommandValue(goal.distance);
}

/// Calculate target position from the error between opening width command value and current value
double HrhGripperSetDistanceAction::GetCommandPos(const double current_distance) {
  // Calculate error
  const double ref_distance = *(goal_buffer_.readFromRT());
  const double error = ref_distance - current_distance;

  // Operation command
  integrated_distance_error_ += error;
  double command_position = controller_->GetCurrentPosition()
                            + distance_control_pgain_ * error
                            + distance_control_igain_ * integrated_distance_error_
                            + distance_control_dgain_ * (error - last_error_);

  last_error_ = error;

  return std::max(std::min(command_position, hand_motor_joint_max_), hand_motor_joint_min_);
}

/// Success determination
void HrhGripperSetDistanceAction::CheckForSuccess(const rclcpp::Time& time, const double current_distance) {
  double current_velocity = controller_->GetCurrentVelocity();
  if (fabs(current_velocity) > stall_velocity_threshold_) {
    // Determine that it is moving and update the last movement time
    last_movement_time_ = time;
  } else if ((time - last_movement_time_).seconds() > distance_control_stall_timeout_) {
    // Determine as stall state
    auto result = std::make_shared<tmc_control_msgs::action::GripperSetDistance::Result>();
    result->distance = current_distance;
    result->stalled = true;

    // Stop operation
    stop_flag_buffer_.writeFromNonRT(true);

    const auto active_goal = *goal_handle_buffer_.readFromNonRT();
    if (!active_goal) {
      return;
    }

    double command = *(goal_buffer_.readFromRT());
    if (fabs(command - current_distance) < goal_tolerance_) {
      active_goal->gh_->succeed(result);
    } else {
      active_goal->gh_->abort(result);
    }

    goal_handle_buffer_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
}

/// Callback when opening width command arrives via topic
void HrhGripperSetDistanceAction::DistanceCommandCallback(const std_msgs::msg::Float32::SharedPtr msg) {
  controller_->PreemptActiveGoal();
  controller_->ChangeControlMode(shared_from_this());
  SetCommandValue(msg->data);
}

/// Set command
void HrhGripperSetDistanceAction::SetCommandValue(const double distance) {
  goal_buffer_.writeFromNonRT(std::max(std::min(distance, distance_max_), distance_min_));
  stop_flag_buffer_.writeFromNonRT(false);
  last_movement_time_ = node_->now();
  integrated_distance_error_ = 0.0;
  last_error_ = 0.0;
}

}  // namespace hsrb_gripper_controller
