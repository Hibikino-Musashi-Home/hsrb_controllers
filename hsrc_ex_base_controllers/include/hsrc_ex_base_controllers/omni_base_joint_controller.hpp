/// @file omni_base_joint_controller.hpp
/// @brief 全方位台車ジョイントコントローラクラス
/// @Copyright (C) 2019 Toyota Motor Corporation
#ifndef HSRC_EX_BASE_CONTROLLERS_OMNI_BASE_JOINT_CONTROLLER_HPP_
#define HSRC_EX_BASE_CONTROLLERS_OMNI_BASE_JOINT_CONTROLLER_HPP_

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <hsrc_ex_base_controllers/filter.hpp>
#include <hsrc_ex_base_controllers/twin_caster_drive.hpp>

namespace hsrc_ex_base_controllers {

/// 旋回軸速度と車輪速度のリミット
struct ControlLimit {
  // 旋回軸速度リミット[rad/s]
  double yaw_limit;
  // 車輪速度リミット[rad/s]
  double wheel_limit;
};

/// Jointコントローラクラス
class OmniBaseJointControllerBase {
 public:
  using Ptr = std::shared_ptr<OmniBaseJointControllerBase>;

  explicit OmniBaseJointControllerBase(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
  ~OmniBaseJointControllerBase() = default;

  // パラメータ初期化
  bool Init();

  // インターフェース設定
  virtual std::vector<std::string> command_interface_names() const = 0;
  std::vector<std::string> state_interface_names() const;
  bool Activate(std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
                std::vector<hardware_interface::LoanedStateInterface>& state_interfaces);
  // 指令値を計算する
  void SetJointCommand(double period, const Eigen::Vector3d output_velocity);
  // 軸位置を取得する
  bool GetJointPositions(Eigen::Vector3d& positions_out) const;
  // 軸速度を取得する
  bool GetJointVelocities(Eigen::Vector3d& velocities_out) const;
  // 旋回軸の目標位置をリセットする
  virtual void ResetDesiredSteerPosition() = 0;

  // アクセサ
  Eigen::Vector3d joint_command() const { return joint_command_; }
  OmniBaseSize omnibase_size() const { return omnibase_size_; }
  std::string l_wheel_joint_name() const { return joint_names_[kJointIDLeftWheel]; }
  std::string r_wheel_joint_name() const { return joint_names_[kJointIDRightWheel]; }
  std::string steer_joint_name() const { return joint_names_[kJointIDSteer]; }
  std::vector<std::string> joint_names() const { return joint_names_; }
  double desired_steer_pos() const { return desired_steer_pos_; }

  Eigen::Vector3d desired_joint_command() const { return desired_command_; }
  Eigen::Vector3d base_command() const { return base_command_; }

 protected:
  std::ofstream log_file_;

  virtual void SetCommandToCommandInterface(double period) = 0;

  // Joint指令値
  Eigen::Vector3d joint_command_;

  // 台車の各軸のハンドラ
  template <typename T>
  using InterfaceReferences = std::vector<std::reference_wrapper<T>>;
  InterfaceReferences<hardware_interface::LoanedCommandInterface> command_interfaces_;
  InterfaceReferences<hardware_interface::LoanedStateInterface> current_position_interfaces_;
  InterfaceReferences<hardware_interface::LoanedStateInterface> current_velocity_interfaces_;

  // 旋回軸の目標位置
  double desired_steer_pos_;

 private:
  Eigen::Vector3d desired_command_;
  Eigen::Vector3d base_command_;

  // コントローラのノードハンドル
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  // 各軸の名前
  std::vector<std::string> joint_names_;

  // 台車の寸法情報
  OmniBaseSize omnibase_size_;
  // 台車の運動学モデル
  TwinCasterDrive::Ptr twin_drive_;
  // 指令速度リミット
  ControlLimit velocity_limit_;
  // エンコーダ値速度閾値
  ControlLimit actual_velocity_threshold_;
  // 加速度リミット
  ControlLimit acceleration_acc_limit_;
  ControlLimit acceleration_dec_limit_;
  // 速度フィルタ
  std::vector<Filter<> > velocity_filters_;
};

// 旋回軸は位置指令を行うコントローラ
class OmniBaseJointControllerBaseRollPosition : public OmniBaseJointControllerBase {
 public:
  explicit OmniBaseJointControllerBaseRollPosition(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
      : OmniBaseJointControllerBase(node) {}
  ~OmniBaseJointControllerBaseRollPosition() = default;

  std::vector<std::string> command_interface_names() const override;

  // 旋回軸の目標位置をリセットする
  void ResetDesiredSteerPosition() override {
    desired_steer_pos_ = current_position_interfaces_[kJointIDSteer].get().get_value();
  }

 protected:
  void SetCommandToCommandInterface(double period) override;
};

// 旋回軸は速度指令を行うコントローラ
class OmniBaseJointControllerBaseRollVelocity : public OmniBaseJointControllerBase {
 public:
  explicit OmniBaseJointControllerBaseRollVelocity(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
      : OmniBaseJointControllerBase(node) {}
  ~OmniBaseJointControllerBaseRollVelocity() = default;

  std::vector<std::string> command_interface_names() const override;

  // 旋回軸の目標位置をリセットする
  void ResetDesiredSteerPosition() override {}

 protected:
  void SetCommandToCommandInterface(double period) override;
};

}  // namespace hsrc_ex_base_controllers

#endif /*HSRC_EX_BASE_CONTROLLERS_OMNI_BASE_JOINT_CONTROLLER_HPP_*/
