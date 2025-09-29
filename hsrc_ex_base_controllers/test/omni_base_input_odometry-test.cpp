/// @file omni_base_input_odometry-test.cpp
/// @brief 外部入力オドメトリクラスのテスト
/// @copyright Copyright (C) 2019 Toyota Motor Corporation

#include <gtest/gtest.h>

#include <hsrc_ex_base_controllers/omni_base_input_odometry.hpp>

#include "utils.hpp"

namespace hsrc_ex_base_controllers {

/// オドメトリを初期化すること
TEST(OmniBaseInputOdometryTest, InitOdometry) {
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  node->configure();
  auto odom = InputOdometry(node);
  node->activate();

  odom.InitOdometry();

  auto output = odom.GetOdometry();
  EXPECT_EQ(output.header.stamp, rclcpp::Time(0));
  EXPECT_EQ(output.pose.pose.position.x, 0.0);
  EXPECT_EQ(output.pose.pose.position.y, 0.0);
  EXPECT_EQ(output.pose.pose.position.z, 0.0);
  EXPECT_EQ(output.pose.pose.orientation.x, 0.0);
  EXPECT_EQ(output.pose.pose.orientation.y, 0.0);
  EXPECT_EQ(output.pose.pose.orientation.z, 0.0);
  EXPECT_EQ(output.pose.pose.orientation.w, 1.0);
}

/// 現在のオドメトリを取得すること
TEST(OmniBaseInputOdometryTest, GetOdometry) {
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  node->configure();
  auto odom = InputOdometry(node);
  auto publisher = node->create_publisher<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SystemDefaultsQoS());
  node->activate();

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = node->now();
  msg.pose.pose.position.x = 1.0;
  msg.pose.pose.position.y = 2.0;
  msg.pose.pose.position.z = 3.0;
  msg.pose.pose.orientation.x = 4.0;
  msg.pose.pose.orientation.y = 5.0;
  msg.pose.pose.orientation.z = 6.0;
  msg.pose.pose.orientation.w = 7.0;

  publisher->publish(msg);
  auto timeout = TimeoutDetection(node->get_clock());
  while (odom.GetOdometry().header.stamp == rclcpp::Time(0)) {
    timeout.Run();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  auto output = odom.GetOdometry();
  EXPECT_EQ(output.pose.pose.position.x, 1.0);
  EXPECT_EQ(output.pose.pose.position.y, 2.0);
  EXPECT_EQ(output.pose.pose.position.z, 3.0);
  EXPECT_EQ(output.pose.pose.orientation.x, 4.0);
  EXPECT_EQ(output.pose.pose.orientation.y, 5.0);
  EXPECT_EQ(output.pose.pose.orientation.z, 6.0);
  EXPECT_EQ(output.pose.pose.orientation.w, 7.0);
}

}  // namespace hsrc_ex_base_controllers

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
