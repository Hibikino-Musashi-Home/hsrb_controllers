/// @file omni_base_odometry.hpp
/// @brief 全方位台車オドメトリクラス
/// @Copyright (C) 2015 Toyota Motor Corporation
#ifndef HSRC_EX_BASE_CONTROLLERS_OMNI_BASE_ODOMETRY_HPP_
#define HSRC_EX_BASE_CONTROLLERS_OMNI_BASE_ODOMETRY_HPP_

#include <memory>
#include <string>

#include <Eigen/Core>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <tf2_msgs/msg/tf_message.hpp>

#include <hsrc_ex_base_controllers/omni_base_input_odometry.hpp>
#include <hsrc_ex_base_controllers/twin_caster_drive.hpp>

namespace hsrc_ex_base_controllers {

/// オドメトリ計算クラス
class Odometry {
 public:
  using Ptr = std::shared_ptr<Odometry>;

  explicit Odometry(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
  virtual ~Odometry() {}

  // オドメトリを更新する
  virtual void UpdateOdometry(double period,
                              const Eigen::Vector3d& positions,
                              const Eigen::Vector3d& velocities) = 0;

  // アクセサ
  Eigen::Vector3d odometry() const { return odometry_; }
  Eigen::Vector3d velocity() const { return velocity_; }

 protected:
  // 台車オドメトリ
  Eigen::Vector3d odometry_;
  // 台車速度
  Eigen::Vector3d velocity_;
};

/// 全方位台車オドメトリ計算クラス
class BaseOdometry : public Odometry {
 public:
  using Ptr = std::shared_ptr<BaseOdometry>;

  explicit BaseOdometry(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
  virtual ~BaseOdometry() {}

  // オドメトリを更新する
  virtual void UpdateOdometry(double period,
                              const Eigen::Vector3d& positions,
                              const Eigen::Vector3d& velocities);
  // オドメトリデータを初期化する
  void InitOdometry();

 private:
  // オドメトリ
  InputOdometry::Ptr input_odom_;
};

/// 全方位台車ホイールオドメトリ計算クラス
class WheelOdometry : public Odometry {
 public:
  using Ptr = std::shared_ptr<WheelOdometry>;

  WheelOdometry(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const OmniBaseSize& omnibase_size);
  virtual ~WheelOdometry() {}
  // オドメトリを更新する
  virtual void UpdateOdometry(double period, const Eigen::Vector3d& positions, const Eigen::Vector3d& velocities);
  // オドメトリを発行する
  virtual void PublishOdometry(const rclcpp::Time& time);

  void set_last_odometry_published_time(const rclcpp::Time& time) {
    last_odometry_published_time_ = time;
  }
  void set_last_transform_published_time(const rclcpp::Time& time) {
    last_transform_published_time_ = time;
  }

 private:
  // オドメトリに関するフレーム名
  std::string tf_prefix_;
  std::string wheel_base_frame_;
  std::string wheel_odom_frame_;
  // 全方位台車モデル
  TwinCasterDrive::Ptr twin_drive_;

  // 最後にオドメトリを発行した時間
  rclcpp::Time last_odometry_published_time_;
  // 最後にオドメトリtfを発行した時間
  rclcpp::Time last_transform_published_time_;

  // オドメトリパブリッシャ
  using OdometryPublisher = realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>;
  using OdometryPublisherPtr = std::unique_ptr<OdometryPublisher>;
  OdometryPublisherPtr odometry_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_impl_;

  // オドメトリ発行の周期
  rclcpp::Duration odometry_publish_period_;

  // オドメトリtfパブリッシャ
  using TFPublisher = realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>;
  using TFPublisherPtr = std::unique_ptr<TFPublisher>;
  TFPublisherPtr transform_publisher_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr transform_publisher_impl_;

  // オドメトリtf発行の周期
  rclcpp::Duration transform_publish_period_;
};

}  // namespace hsrc_ex_base_controllers

#endif /*HSRC_EX_BASE_CONTROLLERS_OMNI_BASE_ODOMETRY_HPP_*/
