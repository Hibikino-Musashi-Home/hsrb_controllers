/// @file omni_base_joint_controller.cpp
/// @brief 全方位台車ジョイントコントローラクラス
/// @Copyright (C) 2019 Toyota Motor Corporation
#include <hsrc_ex_base_controllers/omni_base_joint_controller.hpp>

#include <string>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <urdf/model.h>

#include "utils.hpp"

namespace {

// 旋回軸指令速度リミット [rad/s]
constexpr double kYawVelocityLimit = 1.8;
// 車輪指令速度リミット [rad/s]
constexpr double kWheelVelocityLimit = 8.5;
/// エンコーダ速度閾値
/// ごくまれに(数時間に一回とか)、エンコーダの値が飛ぶ(4000rad/sec程度となる)ことが発見されたため、それを弾く為の閾値
/// 異常値は4000程度なので、default値を1000とすることで異常値を弾く
/// TODO(kazuhito_tanaka): 本質対策がされたら本設定を削除
// 旋回軸エンコーダ速度閾値 [rad/s]
constexpr double kYawActualVelocityThreshold = 1000.0;
// 車輪エンコーダ速度閾値 [rad/s]
constexpr double kWheelActualVelocityThreshold = 1000.0;
// 読み込むurdfロボットモデルのデフォルト名
const char* const kDefaultRobotModelName = "robot_description";
// urdfロボットモデルを読み込むノードのデフォルト名
const char* const kDefaultRobotModelNode = "robot_state_publisher";

// 関節名の取得，取得できない場合はエラー
bool GetJointName(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::string& parameter_name, std::string& joint_name_out) {
  joint_name_out = hsrc_ex_base_controllers::GetParameter(node, parameter_name, "");
  if (joint_name_out.empty()) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Could not find " << parameter_name);
    return false;
  } else {
    return true;
  }
}

// URDFを読み込む
std::string GetRobotDescription(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
  // まずは自身のノードから読み込めるかを試す，ダメならmodel_node_nameからの読み込みを試す
  // gazeboの場合，controller_managerにrobot_descriptionをおけないので別ノードから読み込むしかない
  const std::string model_name = hsrc_ex_base_controllers::GetParameter(node, "model_name", kDefaultRobotModelName);
  const std::string robot_description_out = hsrc_ex_base_controllers::GetParameter(node, model_name, "");
  if (!robot_description_out.empty()) {
    return robot_description_out;
  }

  const std::string model_node_name = hsrc_ex_base_controllers::GetParameter(
      node, "model_node_name", kDefaultRobotModelNode);
  const int32_t timeout = hsrc_ex_base_controllers::GetParameter(node, "parameter_connection_timeout", 60);
  // 引数nodeを使用すると、get_parameterでパラメータを取得する際にエラーとなるため、
  // 一時的に、ノードオブジェクトを生成する。
  std::string node_name = std::string(node->get_name());
  auto omni_base_controller_node = rclcpp_lifecycle::LifecycleNode::make_shared(node_name);
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(omni_base_controller_node, model_node_name);
  int32_t wait_for_service_count = 0;
  while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
    ++wait_for_service_count;
    if (!rclcpp::ok()) {
      return "";
    } else if (wait_for_service_count >= timeout) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Could not connect parameter server of " << model_node_name);
      return "";
    }
  }
  return parameters_client->get_parameter(model_name, std::string());
}

// OmniBaseSizeを初期化する
bool InitializeOmniBaseSize(const rclcpp::Logger& logger,
                            const std::string& robot_description,
                            const std::vector<std::string>& joint_names,
                            hsrc_ex_base_controllers::OmniBaseSize & base_size_out) {
  auto urdf = std::make_shared<urdf::Model>();
  if (!urdf->initString(robot_description)) {
    RCLCPP_ERROR_STREAM(logger, "Failed to parse URDF");
    return false;
  }

  const auto steer_joint = urdf->getJoint(joint_names[hsrc_ex_base_controllers::kJointIDSteer]);
  const auto l_wheel_joint = urdf->getJoint(joint_names[hsrc_ex_base_controllers::kJointIDLeftWheel]);
  const auto r_wheel_joint = urdf->getJoint(joint_names[hsrc_ex_base_controllers::kJointIDRightWheel]);
  if (!steer_joint || !l_wheel_joint || !r_wheel_joint) {
    RCLCPP_ERROR(logger, "Could not get joint param from urdf");
    return false;
  }

  base_size_out.tread = fabs(l_wheel_joint->parent_to_joint_origin_transform.position.y -
                              r_wheel_joint->parent_to_joint_origin_transform.position.y);
  base_size_out.caster_offset = fabs(l_wheel_joint->parent_to_joint_origin_transform.position.x);
  base_size_out.wheel_radius = fabs(l_wheel_joint->parent_to_joint_origin_transform.position.z);
  return true;
}

}  // namespace

namespace hsrc_ex_base_controllers {

// コンストラクタ，パラメータの初期化を行う
OmniBaseJointControllerBase::OmniBaseJointControllerBase(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
    : node_(node),
      log_file_("/tmp/omni_base_joint_controller.csv"),
      joint_command_(Eigen::Vector3d::Zero()),
      base_command_(Eigen::Vector3d::Zero()),
      desired_steer_pos_(0.0) {
}

// パラメータ初期化
bool OmniBaseJointControllerBase::Init() {
  joint_names_.resize(kNumOmniBaseJointIDs);
  if (!GetJointName(node_, "joints.steer", joint_names_[kJointIDSteer]) ||
      !GetJointName(node_, "joints.l_wheel", joint_names_[kJointIDLeftWheel]) ||
      !GetJointName(node_, "joints.r_wheel", joint_names_[kJointIDRightWheel])) {
    return false;
  }
  if (!InitializeOmniBaseSize(node_->get_logger(), GetRobotDescription(node_), joint_names_, omnibase_size_)) {
    return false;
  }
  twin_drive_ = std::make_shared<TwinCasterDrive>(omnibase_size_);

  velocity_limit_.yaw_limit = GetPositiveParameter(node_, "yaw_velocity_limit", kYawVelocityLimit);
  velocity_limit_.wheel_limit = GetPositiveParameter(node_, "wheel_velocity_limit", kWheelVelocityLimit);

  acceleration_acc_limit_.yaw_limit = GetPositiveParameter(node_, "yaw_acceleration_limit", 1.8);
  acceleration_acc_limit_.wheel_limit = GetPositiveParameter(node_, "wheel_acceleration_limit", 5.0);
  acceleration_dec_limit_.yaw_limit = GetPositiveParameter(node_, "yaw_deceleration_limit",
                                                           5.0 * acceleration_acc_limit_.yaw_limit);
  acceleration_dec_limit_.wheel_limit = GetPositiveParameter(node_, "wheel_deceleration_limit",
                                                             5.0 * acceleration_acc_limit_.wheel_limit);

  /// ごくまれに(数時間に一回とか)、エンコーダの値が飛ぶ(4000rad/sec程度となる)ことが発見されたため、それを弾く為の閾値
  /// TODO(kazuhito_tanaka): 本質対策がされたら本設定を削除
  actual_velocity_threshold_.yaw_limit = GetPositiveParameter(
      node_, "yaw_actual_velocity_threshold", kYawActualVelocityThreshold);
  actual_velocity_threshold_.wheel_limit = GetPositiveParameter(
      node_, "wheel_actual_velocity_threshold", kWheelActualVelocityThreshold);

  velocity_filters_.resize(kNumOmniBaseJointIDs);
  std::vector<double> coeff_a = GetParameter(node_, "wheel_command_velocity_filter.a", std::vector<double>());
  std::vector<double> coeff_b = GetParameter(node_, "wheel_command_velocity_filter.b", std::vector<double>());
  if (coeff_a.size() > 0 && coeff_b.size() > 0) {
    Filter<> filter(coeff_a, coeff_b);
    velocity_filters_[kJointIDRightWheel] = filter;
    velocity_filters_[kJointIDLeftWheel] = filter;
  }
  coeff_a = GetParameter(node_, "steer_command_velocity_filter.a", std::vector<double>());
  coeff_b = GetParameter(node_, "steer_command_velocity_filter.b", std::vector<double>());
  if (coeff_a.size() > 0 && coeff_b.size() > 0) {
    velocity_filters_[kJointIDSteer] = Filter<>(coeff_a, coeff_b);
  }

  return true;
}

std::vector<std::string> OmniBaseJointControllerBase::state_interface_names() const {
  std::vector<std::string> names;
  for (const auto& name : joint_names_) {
    names.push_back(name + '/' + hardware_interface::HW_IF_POSITION);
    names.push_back(name + '/' + hardware_interface::HW_IF_VELOCITY);
  }
  return names;
}

/// インターフェース設定
bool OmniBaseJointControllerBase::Activate(std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
                                           std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
  command_interfaces_.clear();
  current_position_interfaces_.clear();
  current_velocity_interfaces_.clear();

  for (const auto& name : joint_names_) {
    for (auto& interface : command_interfaces) {
      if (interface.get_prefix_name() == name) {
        command_interfaces_.emplace_back(std::ref(interface));
      }
    }
    for (auto& interface : state_interfaces) {
      if (interface.get_prefix_name() == name) {
        if (interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
          current_position_interfaces_.emplace_back(std::ref(interface));
        } else if (interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY) {
          current_velocity_interfaces_.emplace_back(std::ref(interface));
        }
      }
    }
  }
  if (command_interfaces_.size() != kNumOmniBaseJointIDs ||
      current_position_interfaces_.size() != kNumOmniBaseJointIDs ||
      current_velocity_interfaces_.size() != kNumOmniBaseJointIDs) {
    return false;
  }

  ResetDesiredSteerPosition();
  return true;
}

/// 軸位置を取得する
bool OmniBaseJointControllerBase::GetJointPositions(Eigen::Vector3d& positions_out) const {
  positions_out(kJointIDRightWheel) = current_position_interfaces_[kJointIDRightWheel].get().get_value();
  positions_out(kJointIDLeftWheel) = current_position_interfaces_[kJointIDLeftWheel].get().get_value();
  positions_out(kJointIDSteer) = current_position_interfaces_[kJointIDSteer].get().get_value();
  return true;
}

/// 軸速度を取得する
bool OmniBaseJointControllerBase::GetJointVelocities(Eigen::Vector3d& velocities_out) const {
  velocities_out(kJointIDRightWheel) = current_velocity_interfaces_[kJointIDRightWheel].get().get_value();
  velocities_out(kJointIDLeftWheel) = current_velocity_interfaces_[kJointIDLeftWheel].get().get_value();
  velocities_out(kJointIDSteer) = current_velocity_interfaces_[kJointIDSteer].get().get_value();
  /// ごくまれに(数時間に一回とか)、エンコーダの値が飛ぶ(4000rad/secくらいになる)ことが発見されたため、それを弾く
  /// joint速度が閾値以上なら、returnする
  /// TODO(kazuhito_tanaka): 本質対策がされたら本設定を削除
  if ((fabs(velocities_out(kJointIDRightWheel)) > actual_velocity_threshold_.wheel_limit) ||
      (fabs(velocities_out(kJointIDLeftWheel)) > actual_velocity_threshold_.wheel_limit) ||
      (fabs(velocities_out(kJointIDSteer)) > actual_velocity_threshold_.yaw_limit)) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "Too big joint velocity! [right, left, steer]=[%lf, %lf, %lf]",
        velocities_out(kJointIDRightWheel), velocities_out(kJointIDLeftWheel), velocities_out(kJointIDSteer));
    return false;
  } else {
    return true;
  }
}

void CalcNextMinMax(double prev_vel,
                    double vel_limit,
                    double acc_acc_limit,
                    double acc_dec_limit,
                    double period,
                    double& next_min,
                    double& next_max) {
  const double acc_range = acc_acc_limit * period;
  const double dec_range = acc_dec_limit * period;
  if (prev_vel > dec_range) {
    next_min = prev_vel - dec_range;
    next_max = prev_vel + acc_range;
  } else if (prev_vel < -dec_range) {
    next_min = prev_vel - acc_range;
    next_max = prev_vel + dec_range;
  } else if (prev_vel > 0.0) {
    next_min = acc_range / dec_range * prev_vel - acc_range;
    next_max = prev_vel + acc_range;
  } else if (prev_vel <= 0.0) {
    next_min = prev_vel - acc_range;
    next_max = acc_range / dec_range * prev_vel + acc_range;
  }
  next_min = std::clamp(next_min, -vel_limit, vel_limit);
  next_max = std::clamp(next_max, -vel_limit, vel_limit);
}

bool IsCommandInRange(const Eigen::Vector3d& command,
                      const Eigen::Vector3d& next_min,
                      const Eigen::Vector3d& next_max) {
  constexpr double kEpsilon = 1.0e-5;
  return (command(kJointIDSteer) >= next_min(kJointIDSteer) - kEpsilon &&
          command(kJointIDSteer) <= next_max(kJointIDSteer) + kEpsilon &&
          command(kJointIDRightWheel) >= next_min(kJointIDRightWheel) - kEpsilon &&
          command(kJointIDRightWheel) <= next_max(kJointIDRightWheel) + kEpsilon&&
          command(kJointIDLeftWheel) >= next_min(kJointIDLeftWheel) - kEpsilon&&
          command(kJointIDLeftWheel) <= next_max(kJointIDLeftWheel) + kEpsilon);
}

double CalcRatio(double current_vel, double next_min, double next_max, double acc_limit, double period) {
  if (std::abs(current_vel) < acc_limit * period) {
    return 0.0;
  } else if (std::abs(next_min) < std::abs(next_max)) {
    return std::abs(next_min) / std::abs(current_vel);
  } else {
    return std::abs(next_max) / std::abs(current_vel);
  }
}

/// 指令値を計算する
void OmniBaseJointControllerBase::SetJointCommand(double period, const Eigen::Vector3d output_velocity) {
  // const double vel_angle_on_base = std::atan2(output_velocity(1), output_velocity(0));
  // const double roll_angle_on_base = current_position_interfaces_[kJointIDSteer].get().get_value();
  // double diff_angle = angles::shortest_angular_distance(roll_angle_on_base, vel_angle_on_base);

  const auto current_vel = joint_command_;
  // GetJointVelocities(current_vel);

  // TODO(Takeshita) actualを取ってきたほうがいいかも
  const auto current_base_vel = base_command_;

  Eigen::Vector3d next_command_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d next_command_max = Eigen::Vector3d::Zero();
  CalcNextMinMax(current_vel[kJointIDSteer], velocity_limit_.yaw_limit,
                 acceleration_acc_limit_.yaw_limit, acceleration_dec_limit_.yaw_limit, period,
                 next_command_min(kJointIDSteer), next_command_max(kJointIDSteer));
  CalcNextMinMax(current_vel[kJointIDRightWheel], velocity_limit_.wheel_limit,
                 acceleration_acc_limit_.wheel_limit, acceleration_dec_limit_.wheel_limit, period,
                 next_command_min(kJointIDRightWheel), next_command_max(kJointIDRightWheel));
  CalcNextMinMax(current_vel[kJointIDLeftWheel], velocity_limit_.wheel_limit,
                 acceleration_acc_limit_.wheel_limit, acceleration_dec_limit_.wheel_limit, period,
                 next_command_min(kJointIDLeftWheel), next_command_max(kJointIDLeftWheel));
  // std::cerr << next_command_min.transpose() << "   " << next_command_max.transpose() << std::endl;

  // ロボット上体座標系の指令速度をジョイント指令速度に変換
  twin_drive_->Update(current_position_interfaces_[kJointIDSteer].get().get_value());
  // joint_command_ = twin_drive_->ConvertInverse(output_velocity);
  desired_command_ = twin_drive_->ConvertInverse(output_velocity);

  // 目標速度を各軸速度リミットを考慮し値に変換して指令値として使う
  auto joint_command_raw = twin_drive_->ConvertInverse(output_velocity);

  if (fabs(joint_command_raw(kJointIDSteer)) > velocity_limit_.yaw_limit) {
    double ratio = fabs(joint_command_raw(kJointIDSteer)) / velocity_limit_.yaw_limit;
    joint_command_raw(kJointIDSteer) /= ratio;
    joint_command_raw(kJointIDRightWheel) /= ratio;
    joint_command_raw(kJointIDLeftWheel) /= ratio;
  }

  // 車輪速度に対してリミットを掛ける
  if (fabs(joint_command_raw(kJointIDRightWheel)) > velocity_limit_.wheel_limit ||
      fabs(joint_command_raw(kJointIDLeftWheel)) > velocity_limit_.wheel_limit) {
    const double ratio = std::max(fabs(joint_command_raw(kJointIDRightWheel)),
                                  fabs(joint_command_raw(kJointIDLeftWheel))) / velocity_limit_.wheel_limit;
    joint_command_raw(kJointIDSteer) /= ratio;
    joint_command_raw(kJointIDRightWheel) /= ratio;
    joint_command_raw(kJointIDLeftWheel) /= ratio;
  }
  const auto filtered_output_velocity = twin_drive_->ConvertForward(joint_command_raw);

  // std::cerr << "filtered_output_velocity: " << filtered_output_velocity.transpose() << std::endl;
  // std::cerr << "current_base_vel: " << current_base_vel.transpose() << std::endl;

  // const auto start = std::chrono::system_clock::now();

  // // 試作2
  // if (current_base_vel.norm() < 1.0e-3 || output_velocity.norm() < 1.0e-3) {
  //   diff_angle = 0.0;
  // } else {
  //   diff_angle = std::acos(output_velocity.dot(current_base_vel) / (output_velocity.norm() * current_base_vel.norm()));
  // }

  // if (std::abs(diff_angle) > 1.5 && current_base_vel.norm() > 0.1) {
  //   // 方向を大きく変更する場合，全力で停止する
  //   const double steer_ratio = CalcRatio(
  //       current_vel(kJointIDSteer), next_command_min(kJointIDSteer), next_command_max(kJointIDSteer),
  //       acceleration_dec_limit_.yaw_limit, period);
  //   const double right_ratio = CalcRatio(
  //       current_vel(kJointIDRightWheel), next_command_min(kJointIDRightWheel), next_command_max(kJointIDRightWheel),
  //       acceleration_dec_limit_.wheel_limit, period);
  //   const double left_ratio = CalcRatio(
  //       current_vel(kJointIDLeftWheel), next_command_min(kJointIDLeftWheel), next_command_max(kJointIDLeftWheel),
  //       acceleration_dec_limit_.wheel_limit, period);

  //   const double ratio = std::max(steer_ratio, std::max(right_ratio, left_ratio));
  //   desired_steer_pos_ = 3.0 + ratio;
  //   joint_command_ = current_vel * ratio;
  //   base_command_ = twin_drive_->ConvertForward(joint_command_);
  //   SetCommandToCommandInterface(period);
  //   return;
  // }

  // 試作1

  // 目標速度が出せるならそれでOK
  bool is_found = false;
  {
    const auto joint_command_candidate = twin_drive_->ConvertInverse(filtered_output_velocity);
    if (IsCommandInRange(joint_command_candidate, next_command_min, next_command_max)) {
      joint_command_ = joint_command_candidate;
      base_command_ = filtered_output_velocity;
      desired_steer_pos_ = 1.0;
      is_found = true;
    }
  }
  // 目標速度と現在速度の線形補間から実現可能な台車速度を探す
  if (!is_found) {
    double left = 0.0;
    double right = 1.0;
    while (right - left > 1.0e-5) {
      const double ratio = (left + right) / 2.0;
      const auto base_command_candidate = filtered_output_velocity * ratio + current_base_vel * (1.0 - ratio);
      const auto joint_command_candidate = twin_drive_->ConvertInverse(base_command_candidate);
      if (IsCommandInRange(joint_command_candidate, next_command_min, next_command_max)) {
        joint_command_ = joint_command_candidate;
        base_command_ = base_command_candidate;
        desired_steer_pos_ = ratio;
        is_found = true;
        left = ratio;
      } else {
        right = ratio;
      }
    }
  }

  // 線形補間で不可能なら，現在速度を比率を保ったまま下げれる速度を探す
  // もしかしたら，ここの処理は不要かも
  if (!is_found) {
    const auto joint_command_candidate = Eigen::Vector3d::Zero();
    if (IsCommandInRange(joint_command_candidate, next_command_min, next_command_max)) {
      joint_command_ = joint_command_candidate;
      base_command_ = Eigen::Vector3d::Zero();
      desired_steer_pos_ = 2.0;
      is_found = true;
    }
  }

  if (!is_found) {
    double left = 0.0;
    double right = 1.0;
    while (right - left > 1.0e-5) {
      const double ratio = (left + right) / 2.0;
      const auto base_command_candidate = current_base_vel * (1.0 - ratio);
      const auto joint_command_candidate = twin_drive_->ConvertInverse(base_command_candidate);
      if (IsCommandInRange(joint_command_candidate, next_command_min, next_command_max)) {
        joint_command_ = joint_command_candidate;
        base_command_ = base_command_candidate;
        desired_steer_pos_ = 1.0 + ratio;
        is_found = true;
        left = ratio;
      } else {
        right = ratio;
      }
    }
  }

  // 見つからなかった場合は止める，つまりjoint_command_.norm()が大きい時と同じ処理
  if (!is_found) {
    const double steer_ratio = CalcRatio(
        current_vel(kJointIDSteer), next_command_min(kJointIDSteer), next_command_max(kJointIDSteer),
        acceleration_dec_limit_.yaw_limit, period);
    const double right_ratio = CalcRatio(
        current_vel(kJointIDRightWheel), next_command_min(kJointIDRightWheel), next_command_max(kJointIDRightWheel),
        acceleration_dec_limit_.wheel_limit, period);
    const double left_ratio = CalcRatio(
        current_vel(kJointIDLeftWheel), next_command_min(kJointIDLeftWheel), next_command_max(kJointIDLeftWheel),
        acceleration_dec_limit_.wheel_limit, period);

    const double ratio = std::max(steer_ratio, std::max(right_ratio, left_ratio));
    desired_steer_pos_ = 2.0 + ratio;
    joint_command_ = current_vel * ratio;
    base_command_ = twin_drive_->ConvertForward(joint_command_);
  }
  // log_file_ << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - start).count() << ","
  //           << desired_steer_pos_ << std::endl;

  // 旋回軸速度に対してリミットを掛ける
  if (fabs(joint_command_(kJointIDSteer)) > velocity_limit_.yaw_limit) {
    double ratio = fabs(joint_command_(kJointIDSteer)) / velocity_limit_.yaw_limit;
    joint_command_(kJointIDSteer) /= ratio;
    joint_command_(kJointIDRightWheel) /= ratio;
    joint_command_(kJointIDLeftWheel) /= ratio;
  }

  // 車輪速度に対してリミットを掛ける
  if (fabs(joint_command_(kJointIDRightWheel)) > velocity_limit_.wheel_limit ||
      fabs(joint_command_(kJointIDLeftWheel)) > velocity_limit_.wheel_limit) {
    const double ratio = std::max(fabs(joint_command_(kJointIDRightWheel)),
                                  fabs(joint_command_(kJointIDLeftWheel))) / velocity_limit_.wheel_limit;
    joint_command_(kJointIDSteer) /= ratio;
    joint_command_(kJointIDRightWheel) /= ratio;
    joint_command_(kJointIDLeftWheel) /= ratio;
  }

  // 速度指令値にフィルタをかける
  joint_command_(kJointIDRightWheel) =
      velocity_filters_[kJointIDRightWheel].update(joint_command_(kJointIDRightWheel));
  joint_command_(kJointIDLeftWheel) =
      velocity_filters_[kJointIDLeftWheel].update(joint_command_(kJointIDLeftWheel));
  joint_command_(kJointIDSteer) =
      velocity_filters_[kJointIDSteer].update(joint_command_(kJointIDSteer));

  // 指令値をコマンドインターフェースにセット
  SetCommandToCommandInterface(period);
}


std::vector<std::string> OmniBaseJointControllerBaseRollPosition::command_interface_names() const {
  std::vector<std::string> names;
  names.push_back(steer_joint_name() + '/' + hardware_interface::HW_IF_POSITION);
  names.push_back(l_wheel_joint_name() + '/' + hardware_interface::HW_IF_VELOCITY);
  names.push_back(r_wheel_joint_name() + '/' + hardware_interface::HW_IF_VELOCITY);
  return names;
}

void OmniBaseJointControllerBaseRollPosition::SetCommandToCommandInterface(double period) {
  // 旋回軸の指令位置を更新してセット
  desired_steer_pos_ += joint_command_(kJointIDSteer) * period;
  command_interfaces_[kJointIDSteer].get().set_value(desired_steer_pos_);

  // 車輪の指令速度をセット
  command_interfaces_[kJointIDRightWheel].get().set_value(joint_command_(kJointIDRightWheel));
  command_interfaces_[kJointIDLeftWheel].get().set_value(joint_command_(kJointIDLeftWheel));
}


std::vector<std::string> OmniBaseJointControllerBaseRollVelocity::command_interface_names() const {
  std::vector<std::string> names;
  names.push_back(steer_joint_name() + '/' + hardware_interface::HW_IF_VELOCITY);
  names.push_back(l_wheel_joint_name() + '/' + hardware_interface::HW_IF_VELOCITY);
  names.push_back(r_wheel_joint_name() + '/' + hardware_interface::HW_IF_VELOCITY);
  return names;
}

void OmniBaseJointControllerBaseRollVelocity::SetCommandToCommandInterface(double period) {
  command_interfaces_[kJointIDSteer].get().set_value(joint_command_(kJointIDSteer));
  command_interfaces_[kJointIDRightWheel].get().set_value(joint_command_(kJointIDRightWheel));
  command_interfaces_[kJointIDLeftWheel].get().set_value(joint_command_(kJointIDLeftWheel));
}

}  // namespace hsrc_ex_base_controllers
