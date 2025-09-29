/// @file omni_base_input_odometry.cpp
/// @brief 全方位台車のオドメトリクラス
/// @Copyright (C) 2019 Toyota Motor Corporation

#include <hsrc_ex_base_controllers/omni_base_input_odometry.hpp>

namespace hsrc_ex_base_controllers {

/// 外部から入力されるオドメトリ
InputOdometry::InputOdometry(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
  odometry_subscriber_ = node->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 1, std::bind(&InputOdometry::OdometryCallback, this, std::placeholders::_1));
  InitOdometry();
}

void InputOdometry::InitOdometry() {
  nav_msgs::msg::Odometry initial_odom;
  initial_odom.header.stamp = rclcpp::Time(0);
  initial_odom.pose.pose.orientation.w = 1.0;
  odometry_buffer_.initRT(initial_odom);
}

/// オドメトリコールバック
void InputOdometry::OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odometry_buffer_.writeFromNonRT(*msg);
}

}  // namespace hsrc_ex_base_controllers
