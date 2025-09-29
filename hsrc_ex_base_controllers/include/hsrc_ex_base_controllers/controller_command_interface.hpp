/// @file controller_command_interface.hpp
/// @brief コントローラとROSからの指令値を繋ぐインターフェースクラス
/// @Copyright (C) 2022 Toyota Motor Corporation
#ifndef HSRC_EX_BASE_CONTROLLERS_CONTROLLER_COMMAND_INTERFACE_HPP_
#define HSRC_EX_BASE_CONTROLLERS_CONTROLLER_COMMAND_INTERFACE_HPP_

#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <joint_trajectory_controller/tolerances.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace hsrc_ex_base_controllers {

class IControllerCommandInterface {
 public:
  using Ptr = std::shared_ptr<IControllerCommandInterface>;

  virtual ~IControllerCommandInterface() = default;

  // 指令を受付可能か返す
  virtual bool IsAcceptable() = 0;

  // 入力速度指令をセットする
  virtual void UpdateVelocity(const geometry_msgs::msg::Twist::SharedPtr& msg) = 0;

  // 入力軌道指令を検証する
  virtual bool ValidateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) = 0;
  // 入力軌道指令をセットする
  virtual void UpdateTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr& trajectory) = 0;
  // // TODO(Takeshita) アクションのgoalに含まれるtolerancesを扱うための関数を追加する
  // // 入力軌道指令とtolerancesをセットする
  // virtual void UpdateTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr& trajectory,
  //                               const joint_trajectory_controller::SegmentTolerances& tolerances) = 0;
  // 入力軌道をリセットする
  virtual void ResetTrajectory() = 0;
};

}  // namespace hsrc_ex_base_controllers

#endif /*HSRC_EX_BASE_CONTROLLERS_CONTROLLER_COMMAND_INTERFACE_HPP_*/
