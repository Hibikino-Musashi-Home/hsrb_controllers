/// @file omni_base_state.cpp
/// @brief 全方位台車の台車状態クラス
/// @Copyright (C) 2019 Toyota Motor Corporation
#include <hsrc_ex_base_controllers/omni_base_state.hpp>

#include <string>
#include <vector>

#include <angles/angles.h>

#include "utils.hpp"

namespace {
// 台車状態パブリッシュ周波数[Hz]
const double kDefaultStatePublishRate = 50.0;

void ConvertVector(const Eigen::VectorXd& input_vector,
                   std::vector<double>& dst_vector) {
  dst_vector.resize(input_vector.size());
  Eigen::Map<Eigen::VectorXd> map(&dst_vector[0], dst_vector.size());
  map = input_vector;
}
}  // namespace

namespace hsrc_ex_base_controllers {

void ControllerState::UpdateError() {
  if (actual.positions.size() == desired.positions.size()) {
    error.positions.resize(actual.positions.size());
    for (uint32_t i = 0; i < actual.positions.size(); ++i) {
      error.positions[i] = desired.positions[i] - actual.positions[i];
    }
  } else {
    error.positions.clear();
  }

  if (actual.velocities.size() == desired.velocities.size()) {
    error.velocities.resize(actual.velocities.size());
    for (uint32_t i = 0; i < actual.velocities.size(); ++i) {
      error.velocities[i] = desired.velocities[i] - actual.velocities[i];
    }
  } else {
    error.velocities.clear();
  }

  if (actual.accelerations.size() == desired.accelerations.size()) {
    error.accelerations.resize(actual.accelerations.size());
    for (uint32_t i = 0; i < actual.accelerations.size(); ++i) {
      error.accelerations[i] = desired.accelerations[i] - actual.accelerations[i];
    }
  } else {
    error.accelerations.clear();
  }
}


ControllerBaseState::ControllerBaseState(const Eigen::Vector3d& actual_positions,
                                         const Eigen::Vector3d& actual_velocities,
                                         const std::vector<double>& desired_positions,
                                         const std::vector<double>& desired_velocities,
                                         const std::vector<double>& desired_accelerations) {
  ConvertVector(actual_positions, actual.positions);
  // TODO(Takeshita) ここで変換しているのが微妙だなぁ
  // base_velocity_はbase_footprint基準なので,odom基準に変換
  Eigen::Matrix3d rot_mat;
  rot_mat << cos(actual_positions[kIndexBaseTheta]), -sin(actual_positions[kIndexBaseTheta]), 0.0,
             sin(actual_positions[kIndexBaseTheta]), cos(actual_positions[kIndexBaseTheta]), 0.0,
             0.0, 0.0, 1.0;
  Eigen::Vector3d transformed_velocity(rot_mat * actual_velocities);
  ConvertVector(transformed_velocity, actual.velocities);
  desired.positions = desired_positions;
  desired.velocities = desired_velocities;
  desired.accelerations = desired_accelerations;

  UpdateError();
}

ControllerBaseState::ControllerBaseState(const Eigen::Vector3d& actual_positions,
                                         const Eigen::Vector3d& actual_velocities)
    : ControllerBaseState(actual_positions, actual_velocities, {}, {}, {}) {
  desired = actual;
  UpdateError();
}

void ControllerBaseState::UpdateError() {
  ControllerState::UpdateError();

  if (error.positions.size() == kNumBaseCoordinateIDs) {
    error.positions[kIndexBaseTheta] = angles::shortest_angular_distance(
        actual.positions[kIndexBaseTheta], desired.positions[kIndexBaseTheta]);
  }
}

ControllerJointState::ControllerJointState(const Eigen::Vector3d& actual_positions,
                                           const Eigen::Vector3d& actual_velocities,
                                           double desired_yaw_position,
                                           const Eigen::Vector3d& desired_velocities) {
  ConvertVector(actual_positions, actual.positions);
  ConvertVector(actual_velocities, actual.velocities);
  ConvertVector(desired_velocities, desired.velocities);
  desired.positions.resize(3, 0.0);
  desired.positions[kJointIDSteer] = desired_yaw_position;

  UpdateError();
}

void ControllerJointState::UpdateError() {
  ControllerState::UpdateError();

  error.positions[kJointIDRightWheel] = 0.0;
  error.positions[kJointIDLeftWheel] = 0.0;
}

void Convert(const ControllerState& in, const rclcpp::Time& stamp, const std::vector<std::string>& joint_names,
             control_msgs::msg::JointTrajectoryControllerState& out) {
  out.header.stamp = stamp;
  out.joint_names = joint_names;
  Convert(in.actual, out.actual);
  Convert(in.desired, out.desired);
  Convert(in.error, out.error);
}

void Convert(const State& in, trajectory_msgs::msg::JointTrajectoryPoint& out) {
  out.positions = in.positions;
  out.velocities = in.velocities;
  out.accelerations = in.accelerations;
}


StatePublisher::StatePublisher(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                               const std::string& topic_name,
                               const std::vector<std::string>& joint_names)
    : node_(node), joint_names_(joint_names), last_state_published_time_(node->now()), state_publish_period_(0, 0) {
  const double state_publish_rate = GetPositiveParameter(node, "state_publish_rate", kDefaultStatePublishRate);
  state_publish_period_ = rclcpp::Duration::from_seconds(1.0 / state_publish_rate);

  publisher_impl_ = node->create_publisher<control_msgs::msg::JointTrajectoryControllerState>(
      topic_name, rclcpp::SystemDefaultsQoS());
  publisher_ = std::make_unique<RealtimePublisher>(publisher_impl_);
}

void StatePublisher::Publish(const ControllerState& state, const rclcpp::Time& stamp) {
  if (stamp - last_state_published_time_ >= state_publish_period_) {
    last_state_published_time_ += state_publish_period_;
    if (publisher_ && publisher_->trylock()) {
      Convert(state, stamp, joint_names_, publisher_->msg_);
      publisher_->unlockAndPublish();
    }
  }
}

}  // namespace hsrc_ex_base_controllers
