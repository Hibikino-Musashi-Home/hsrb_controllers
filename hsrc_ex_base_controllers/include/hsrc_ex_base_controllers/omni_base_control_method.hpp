/// @file omni_base_control_method.hpp
/// @brief 全方位台車制御モードクラス
/// @Copyright (C) 2019 Toyota Motor Corporation
#ifndef HSRC_EX_BASE_CONTROLLERS_OMNI_BASE_CONTROL_METHOD_HPP_
#define HSRC_EX_BASE_CONTROLLERS_OMNI_BASE_CONTROL_METHOD_HPP_

#include <memory>
#include <string>
#include <vector>

#include <boost/noncopyable.hpp>

#include <Eigen/Core>

#include <geometry_msgs/msg/twist.hpp>
#include <joint_trajectory_controller/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.h>

#include <hsrc_ex_base_controllers/omni_base_state.hpp>


namespace hsrc_ex_base_controllers {

/// 台車の制御手段インターフェースクラス
class IBaseControlMethod : private boost::noncopyable {
 public:
  using Ptr = std::shared_ptr<IBaseControlMethod>;

  virtual ~IBaseControlMethod() {}
  // 初期化する
  virtual void Activate() = 0;
};


/// 台車速度追従
class OmniBaseVelocityControl : public IBaseControlMethod {
 public:
  using Ptr = std::shared_ptr<OmniBaseVelocityControl>;

  explicit OmniBaseVelocityControl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
  virtual ~OmniBaseVelocityControl() = default;
  // 初期化する
  void Activate() override;

  // 指令速度を取得する
  Eigen::Vector3d GetOutputVelocity();
  // 指令速度を更新する
  void UpdateCommandVelocity(const geometry_msgs::msg::Twist::SharedPtr& msg);

 private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  // 排他制御
  std::mutex command_mutex_;
  // 指令速度
  Eigen::Vector3d command_velocity_;
  // 最後に速度指令値を受け取った時間
  rclcpp::Time last_velocity_subscribed_time_;
  // 速度指令値途絶判定時間
  double command_timeout_;
};


/// 台車軌道追従
class OmniBaseTrajectoryControl : public IBaseControlMethod {
 public:
  using Ptr = std::shared_ptr<OmniBaseTrajectoryControl>;

  explicit OmniBaseTrajectoryControl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                     const std::vector<std::string>& cordinates);
  virtual ~OmniBaseTrajectoryControl() = default;
  // 初期化する
  void Activate() override;

  // 指令速度を取得する
  Eigen::Vector3d GetOutputVelocity(const ControllerState& base_state);
  // 追従中の軌道を更新する，軌道が存在するならtrueを返す
  bool UpdateActiveTrajectory();
  // 軌道追従の目標状態を取得
  bool SampleDesiredState(const rclcpp::Time& time,
                          const std::vector<double>& current_positions,
                          const std::vector<double>& current_velocities,
                          trajectory_msgs::msg::JointTrajectoryPoint& desired_state,
                          bool& before_last_point,
                          double& time_from_point);
  // 入力軌道指令を検証する
  bool ValidateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) const;
  // 追従軌道を更新する
  void AcceptTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr& trajectory,
                        const Eigen::Vector3d& base_positions);
  // 条件を満たしていたら軌道追従を終了させる
  void TerminateControl(const rclcpp::Time& time, const ControllerState& base_state);
  // 現在追従中の軌道をリセットする
  void ResetCurrentTrajectory();

 private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  // 制御のフィードバックゲイン
  Eigen::Vector3d feedback_gain_;
  // 台車の座標軸名
  std::vector<std::string> coordinate_names_;
  // 軌道追従完了を判断する速度の閾値
  double stop_velocity_threshold_;
  // 新しい軌道が来た際に既存のdesiredからつなげるか否か
  // 変数名や振る舞いはJointTrajectoryControllerに合わせてある
  bool open_loop_control_;
  // 最後にサンプリングした状態
  rclcpp::Time last_sampled_time_;
  trajectory_msgs::msg::JointTrajectoryPoint last_command_state_;
  // last_command_state_に値が入っているか
  // JointTrajectoryControllerのようにActivate時に現在値を入れるのが正しいが
  // 変更箇所が広くなるので，フラグ管理で実装する
  bool has_last_command_state_;

  std::shared_ptr<joint_trajectory_controller::Trajectory>* trajectory_active_ptr_ = nullptr;
  std::shared_ptr<joint_trajectory_controller::Trajectory> trajectory_ptr_ = nullptr;
  realtime_tools::RealtimeBuffer<trajectory_msgs::msg::JointTrajectory::SharedPtr>  trajectory_msg_buffer_;
};

}  // namespace hsrc_ex_base_controllers

#endif /*HSRC_EX_BASE_CONTROLLERS_OMNI_BASE_CONTROL_METHOD_HPP_*/
