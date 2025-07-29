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
#include "hsrb_gripper_controller/hrh_gripper_distance.hpp"

#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>

#include "hsrb_gripper_controller/hrh_gripper_action.hpp"

namespace {

// Fingertip distance publish frequency [Hz]
const double kDefaultDistancePublishRate = 50.0;

// Default hand physical parameters
const double kDefaultProximalToDistalZ = 0.07;
const double kDefaultDistancePalmToTip = 0.002194;
// Default name of the URDF robot model to load
const char* kDefaultRobotModelName = "robot_description";
// Default name of the node to load the URDF robot model
const char* kDefaultRobotModelNode = "robot_state_publisher";
// Default name of the axis to load from the URDF robot model
const char* kDefaultProximalJointName = "hand_l_proximal_joint";
const char* kDefaultDistalJointName = "hand_l_distal_joint";
const char* kDefaultMimicDistalJointName = "hand_l_mimic_distal_joint";
const char* kDefaultFingerTipFrameJointName = "hand_l_finger_tip_frame_joint";

}  // namespace

namespace hsrb_gripper_controller {
HrhGripperDistanceCalculator::HrhGripperDistanceCalculator()
    : proximal_to_distal_z_(kDefaultProximalToDistalZ),
      distance_palm_to_tip_(kDefaultDistancePalmToTip) {}

bool HrhGripperDistanceCalculator::InitializeHandSizeData(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
  // Load URDF
  auto urdf = std::make_shared<urdf::Model>();
  if (!urdf->initString(GetRobotDescription(node))) {
    RCLCPP_ERROR(node->get_logger(), "Failed to parse URDF");
    return false;
  }

  // Axis name to be retrieved
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

  // Obtain necessary parameters
  double distal_to_tip_y = fabs(finger_tip_frame_joint->parent_to_joint_origin_transform.position.y);
  double distal_to_tip_z = fabs(finger_tip_frame_joint->parent_to_joint_origin_transform.position.z);
  double distal_joint_angle_offset = fabs(distal_joint->mimic->offset);
  double palm_to_proximal_y = fabs(proximal_joint->parent_to_joint_origin_transform.position.y);

  // Retain values used for width calculation
  proximal_to_distal_z_ = fabs(mimic_distal_joint->parent_to_joint_origin_transform.position.z);
  distance_palm_to_tip_ = palm_to_proximal_y
                          - (distal_to_tip_y * cos(distal_joint_angle_offset)
                             + distal_to_tip_z * sin(distal_joint_angle_offset));

  return true;
}

// Load URDF
std::string HrhGripperDistanceCalculator::GetRobotDescription(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
  // First, try to load from one's own node. If unsuccessful, try to load from model_node_name
  // In the case of gazebo, it's necessary to load from another node as robot_description cannot be placed in the controller_manager
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

/// Calculate width from hand angle
double HrhGripperDistanceCalculator::GetDistanceFromPosition(double hand_motor_pos) const {
  return GetDistanceFromPosition(hand_motor_pos, 0.0, 0.0);
}

/// Calculate width from hand angle and angle of each finger
double HrhGripperDistanceCalculator::GetDistanceFromPosition(
    double hand_motor_pos, double left_spring_proximal_joint_pos,
    double right_spring_proximal_joint_pos) const {
  double hand_left_position = left_spring_proximal_joint_pos + hand_motor_pos;
  double hand_right_position = right_spring_proximal_joint_pos + hand_motor_pos;
  double ploximal_to_distal = proximal_to_distal_z_ * (sin(hand_left_position) + sin(hand_right_position));
  return ploximal_to_distal + 2.0 * distance_palm_to_tip_;
}

/// Calculate hand angle from width
double HrhGripperDistanceCalculator::GetPositionFromDistance(double distance) const {
  return asin((distance / 2.0 - distance_palm_to_tip_) / proximal_to_distal_z_);
}

DistancePublisher::DistancePublisher(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                     const std::string& topic_name)
    : node_(node),
      distance_publish_period_(0, 0),
      last_distance_published_time_(node->now()) {
  const double distance_publish_rate = GetPositiveParameter(
      node, "distance_publish_rate", kDefaultDistancePublishRate);
  distance_publish_period_ = rclcpp::Duration::from_seconds(1.0 / distance_publish_rate);

  publisher_impl_ = node->create_publisher<std_msgs::msg::Float32>(topic_name, rclcpp::SystemDefaultsQoS());
  publisher_ = std::make_unique<RealtimePublisher>(publisher_impl_);

  // Initialize class for width calculation
  distance_calculator_ = std::make_shared<HrhGripperDistanceCalculator>();
  distance_calculator_->InitializeHandSizeData(node);
}

void DistancePublisher::Publish(const double current_position, const rclcpp::Time& stamp) {
  if ((stamp - last_distance_published_time_) >= distance_publish_period_) {
    if (publisher_ && publisher_->trylock()) {
      publisher_->msg_.data = distance_calculator_->GetDistanceFromPosition(current_position);
      publisher_->unlockAndPublish();

      last_distance_published_time_ += distance_publish_period_;
    }
  }
}

}  // namespace hsrb_gripper_controller
