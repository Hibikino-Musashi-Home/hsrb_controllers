/// @file omni_base_controller.hpp
/// @brief 全方位台車コントローラクラス
/// @Copyright (C) 2015 Toyota Motor Corporation
#ifndef HSRC_EX_BASE_CONTROLLERS_OMNI_BASE_CONTROLLER_HPP_
#define HSRC_EX_BASE_CONTROLLERS_OMNI_BASE_CONTROLLER_HPP_

#include <string>

#include <controller_interface/controller_interface.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <hsrc_ex_base_controllers/command_subscriber.hpp>
#include <hsrc_ex_base_controllers/controller_command_interface.hpp>
#include <hsrc_ex_base_controllers/omni_base_control_method.hpp>
#include <hsrc_ex_base_controllers/omni_base_joint_controller.hpp>
#include <hsrc_ex_base_controllers/omni_base_odometry.hpp>

#include "tolerances.hpp"

namespace hsrc_ex_base_controllers {

/// 全方位台車速度コントローラクラス
class OmniBaseController
    : public controller_interface::ControllerInterface,
      public IControllerCommandInterface {
 public:
  OmniBaseController() = default;
  ~OmniBaseController() = default;

  // コントローラ初期化
  controller_interface::CallbackReturn on_init() override;

  // ros2_controlのインターフェース設定
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // 台車ジョイント角速度を計算し更新
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  // configure時に呼ばれる関数
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  // activate時に呼ばれる関数
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  // deactivate時に呼ばれる関数
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  // 指令を受付可能か返す
  bool IsAcceptable() override;

  // 入力速度指令をセットする
  void UpdateVelocity(const geometry_msgs::msg::Twist::SharedPtr& msg) override;

  // 入力軌道指令を検証する
  bool ValidateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) override;
  // 入力軌道指令をセットする
  void UpdateTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr& trajectory) override;
  // 入力軌道をリセットする
  void ResetTrajectory() override;

 protected:
  // ControllerInterface::init以外の部分の初期化，テストのための分割
  bool InitImpl();

  // 軌道追従の許容幅
  SegmentTolerances default_tolerances_;
  SegmentTolerances active_tolerances_;
  // 軌道追従中のtolerancesのチェック
  // 追従を続ける場合は正の数を，追従を止める場合はcontrol_msgs/action/FollowJointTrajectoryのエラーコード(0 ~ -5)を返す
  int32_t CheckTorelances(const ControllerBaseState& state, bool before_last_point, double time_from_trajectory_end);

  // 入力速度指令サブスクライバ
  CommandVelocitySubscriber::Ptr velocity_subscriber_;
  // 入力軌道指令サブスクライバ
  CommandTrajectorySubscriber::Ptr trajectory_subscriber_;
  // 入力軌道Action指令サーバ
  TrajectoryActionServer::Ptr trajectory_action_;

  // Jointコントローラ
  OmniBaseJointControllerBase::Ptr joint_controller_;

  // 台車のホイールオドメトリ計算クラス
  BaseOdometry::Ptr base_odometry_;
  WheelOdometry::Ptr wheel_odometry_;

  // 台車の指令速度を計算するクラス
  OmniBaseVelocityControl::Ptr velocity_control_;
  OmniBaseTrajectoryControl::Ptr trajectory_control_;

  // 台車の状態の発行
  StatePublisher::Ptr joint_state_publisher_;
  StatePublisher::Ptr base_state_publisher_;
};

}  // namespace hsrc_ex_base_controllers

#endif/*HSRC_EX_BASE_CONTROLLERS_OMNI_BASE_CONTROLLER_HPP_*/
