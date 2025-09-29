/// @file omni_base_input_odometry.hpp
/// @brief 全方位台車のオドメトリクラス
/// @Copyright (C) 2019 Toyota Motor Corporation
#ifndef HSRC_EX_BASE_CONTROLLERS_OMNI_BASE_INPUT_ODOMETRY_HPP_
#define HSRC_EX_BASE_CONTROLLERS_OMNI_BASE_INPUT_ODOMETRY_HPP_

#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_buffer.h>

namespace hsrc_ex_base_controllers {

typedef realtime_tools::RealtimeBuffer<nav_msgs::msg::Odometry> RealtimeOdometryBuffer;

class InputOdometry {
 public:
  using Ptr = std::shared_ptr<InputOdometry>;

  explicit InputOdometry(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);

  // オドメトリを初期化する
  void InitOdometry();
  // 現在のオドメトリを取得する
  nav_msgs::msg::Odometry& GetOdometry() {
    return *odometry_buffer_.readFromRT();
  }

 private:
  // オドメトリコールバック
  void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // オドメトリサブスクライバ
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  // リアルタイムで使用するオドメトリデータ
  RealtimeOdometryBuffer odometry_buffer_;
};

}  // namespace hsrc_ex_base_controllers

#endif /*HSRC_EX_BASE_CONTROLLERS_OMNI_BASE_INPUT_ODOMETRY_HPP_*/
