/// @file omni_base_state.hpp
/// @brief 全方位台車の台車状態クラス
/// @Copyright (C) 2019 Toyota Motor Corporation
#ifndef HSRC_EX_BASE_CONTROLLERS_OMNI_BASE_STATE_HPP_
#define HSRC_EX_BASE_CONTROLLERS_OMNI_BASE_STATE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_publisher.h>

#include <hsrc_ex_base_controllers/twin_caster_drive.hpp>

namespace hsrc_ex_base_controllers {

struct State {
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;
};

struct ControllerState {
  State actual;
  State desired;
  State error;

  virtual ~ControllerState() = default;
  virtual void UpdateError();
};

struct ControllerBaseState : public ControllerState {
  ControllerBaseState(const Eigen::Vector3d& actual_positions,
                      const Eigen::Vector3d& actual_velocities,
                      const std::vector<double>& desired_positions,
                      const std::vector<double>& desired_velocities,
                      const std::vector<double>& desired_accelerations);

  ControllerBaseState(const Eigen::Vector3d& actual_positions,
                      const Eigen::Vector3d& actual_velocities);

  void UpdateError() override;
};

struct ControllerJointState : public ControllerState {
  ControllerJointState(const Eigen::Vector3d& actual_positions,
                       const Eigen::Vector3d& actual_velocities,
                       double desired_yaw_position,
                       const Eigen::Vector3d& desired_velocities);

  void UpdateError() override;
};

void Convert(const ControllerState& in, const rclcpp::Time& stamp, const std::vector<std::string>& joint_names,
             control_msgs::msg::JointTrajectoryControllerState& out);

void Convert(const State& in, trajectory_msgs::msg::JointTrajectoryPoint& out);


class StatePublisher {
 public:
  using Ptr = std::shared_ptr<StatePublisher>;

  StatePublisher(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                 const std::string& topic_name,
                 const std::vector<std::string>& joint_names);

  void Publish(const ControllerState& state, const rclcpp::Time& stamp);

  void set_last_state_published_time(const rclcpp::Time& time) {
    last_state_published_time_ = time;
  }

 private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::vector<std::string> joint_names_;
  rclcpp::Duration state_publish_period_;

  using RealtimePublisher = realtime_tools::RealtimePublisher<control_msgs::msg::JointTrajectoryControllerState>;
  std::unique_ptr<RealtimePublisher> publisher_;
  rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr publisher_impl_;

  // 最後に状態を発行した時間
  rclcpp::Time last_state_published_time_;
};

}  // namespace hsrc_ex_base_controllers

#endif /*HSRC_EX_BASE_CONTROLLERS_OMNI_BASE_STATE_HPP_*/
