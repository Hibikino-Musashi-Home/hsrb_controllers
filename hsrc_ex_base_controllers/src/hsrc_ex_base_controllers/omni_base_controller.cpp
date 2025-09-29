/// @file omni_base_controller.cpp
/// @brief 全方位台車コントローラクラス
/// @Copyright (C) 2015 Toyota Motor Corporation

#include <hsrc_ex_base_controllers/omni_base_controller.hpp>

#include <pluginlib/class_list_macros.hpp>

#include "utils.hpp"


namespace hsrc_ex_base_controllers {

controller_interface::CallbackReturn OmniBaseController::on_init() {
  // 特になにもしない，on_configureとon_activateに全ての処理がある
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration OmniBaseController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names = joint_controller_->command_interface_names();
  return conf;
}

controller_interface::InterfaceConfiguration OmniBaseController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names = joint_controller_->state_interface_names();
  return conf;
}

controller_interface::return_type OmniBaseController::update(
    const rclcpp::Time& current_time, const rclcpp::Duration& period) {
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    return controller_interface::return_type::OK;
  }

  // 台車の各軸の位置・速度取得
  Eigen::Vector3d joint_positions;
  Eigen::Vector3d joint_velocities;
  if (!joint_controller_->GetJointPositions(joint_positions) ||
      !joint_controller_->GetJointVelocities(joint_velocities)) {
    return controller_interface::return_type::ERROR;
  }

  const double period_sec = period.seconds();
  // オドメトリの更新
  wheel_odometry_->UpdateOdometry(period_sec, joint_positions, joint_velocities);
  base_odometry_->UpdateOdometry(period_sec, wheel_odometry_->odometry(), wheel_odometry_->velocity());

  // 台車の追従状態をアップデート
  ControllerBaseState base_state(base_odometry_->odometry(), base_odometry_->velocity());

  Eigen::Vector3d output_velocity = Eigen::Vector3d::Zero();
  if (trajectory_control_->UpdateActiveTrajectory()) {
    // 軌道追従
    trajectory_msgs::msg::JointTrajectoryPoint desired_state;
    bool before_last_point;
    double time_from_point;
    const auto is_valid = trajectory_control_->SampleDesiredState(
        current_time, base_state.actual.positions, base_state.actual.velocities,
        desired_state, before_last_point, time_from_point);
    if (is_valid) {
      base_state = ControllerBaseState(
          base_odometry_->odometry(), base_odometry_->velocity(),
          desired_state.positions, desired_state.velocities, desired_state.accelerations);
      output_velocity = trajectory_control_->GetOutputVelocity(base_state);

      auto status = CheckTorelances(base_state, before_last_point, time_from_point);
      if (status <= 0) {
        trajectory_action_->UpdateActionResult(status);
        trajectory_control_->ResetCurrentTrajectory();
      } else {
        trajectory_action_->SetFeedback(base_state, current_time);
      }
    }
  } else {
    // 速度追従
    output_velocity = velocity_control_->GetOutputVelocity();
  }
  joint_controller_->SetJointCommand(period_sec, output_velocity);

  // 台車の現状態(指令値、現在地、差分)をパブリッシュ
  // const ControllerJointState joint_state(joint_positions,
  const ControllerJointState joint_state(joint_controller_->desired_joint_command(),
                                         joint_velocities,
                                         joint_controller_->desired_steer_pos(),
                                         joint_controller_->joint_command());
  // base_state = ControllerBaseState(
  //     base_odometry_->odometry(), base_odometry_->velocity(),
  //     joint_controller_->base_command(), output_velocity, Eigen::Vector3d::Zero());

  joint_state_publisher_->Publish(joint_state, current_time);
  base_state_publisher_->Publish(base_state, current_time);

  // ホイールオドメトリ、TFをパブリッシュ
  wheel_odometry_->PublishOdometry(current_time);

  trajectory_control_->TerminateControl(current_time, base_state);

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn OmniBaseController::on_configure(const rclcpp_lifecycle::State& previous_state) {
  // 制御する台車の座標軸名を取得
  const auto base_coordinate_names = GetParameter<std::vector<std::string>>(get_node(), "base_coordinates", {});
  if (base_coordinate_names.size() != kNumBaseCoordinateIDs) {
    RCLCPP_ERROR(get_node()->get_logger(), "The size of joints must be three.");
    return controller_interface::CallbackReturn::ERROR;
  }
  default_tolerances_ = get_segment_tolerances(get_node(), base_coordinate_names);

  // 速度のサブスクライバをセット
  velocity_subscriber_ = std::make_shared<CommandVelocitySubscriber>(get_node(), this);
  // 軌道のサブスクライバをセット
  trajectory_subscriber_ = std::make_shared<CommandTrajectorySubscriber>(get_node(), this);
  // アクションサーバ立ちあげ
  trajectory_action_ = std::make_shared<TrajectoryActionServer>(get_node(), base_coordinate_names, this);

  // Jointコントローラクラス
  const auto command_base_roll_velocity = GetParameter<bool>(get_node(), "use_base_roll_velocity", false);
  if (command_base_roll_velocity) {
    joint_controller_ = std::make_shared<OmniBaseJointControllerBaseRollVelocity>(get_node());
  } else {
    joint_controller_ = std::make_shared<OmniBaseJointControllerBaseRollPosition>(get_node());
  }
  if (!joint_controller_->Init()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Initializing OmniBaseJointController is failed.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // ホイールオドメトリのパブリッシャをセット
  wheel_odometry_ = std::make_shared<WheelOdometry>(get_node(), joint_controller_->omnibase_size());
  // 台車オドメトリのパブリッシャをセット
  base_odometry_ = std::make_shared<BaseOdometry>(get_node());

  // コントローラを生成
  velocity_control_ = std::make_shared<OmniBaseVelocityControl>(get_node());
  trajectory_control_ = std::make_shared<OmniBaseTrajectoryControl>(get_node(), base_coordinate_names);

  // 内部関節状態のパブリッシャをセット
  base_state_publisher_ = std::make_shared<StatePublisher>(
      get_node(), "~/state", base_coordinate_names);
  joint_state_publisher_ = std::make_shared<StatePublisher>(
      get_node(), "~/internal_state", joint_controller_->joint_names());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmniBaseController::on_activate(const rclcpp_lifecycle::State& previous_state) {
  if (!joint_controller_->Activate(command_interfaces_, state_interfaces_)) {
    return controller_interface::CallbackReturn::ERROR;
  }
  // JointTrajectoryControllerに合わせて，ここでActivate
  velocity_control_->Activate();
  trajectory_control_->Activate();

  const auto current_time = get_node()->get_clock()->now();
  wheel_odometry_->set_last_odometry_published_time(current_time);
  wheel_odometry_->set_last_transform_published_time(current_time);
  joint_state_publisher_->set_last_state_published_time(current_time);
  base_state_publisher_->set_last_state_published_time(current_time);

  // 立ち上げ時にオドメトリを初期化
  base_odometry_->InitOdometry();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmniBaseController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
  // コントローラ立ち下げ時は現在のアクションゴールをリセット
  trajectory_action_->PreemptActiveGoal();
  // 指令速度をクリア
  const auto zero_velocity = std::make_shared<geometry_msgs::msg::Twist>();
  velocity_control_->UpdateCommandVelocity(zero_velocity);
  return controller_interface::CallbackReturn::SUCCESS;
}

bool OmniBaseController::IsAcceptable() {
  return get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
}

void OmniBaseController::UpdateVelocity(const geometry_msgs::msg::Twist::SharedPtr& msg) {
  velocity_control_->UpdateCommandVelocity(msg);
}

bool OmniBaseController::ValidateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) {
  return trajectory_control_->ValidateTrajectory(trajectory);
}

void OmniBaseController::UpdateTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr& trajectory) {
  active_tolerances_ = default_tolerances_;
  trajectory_control_->AcceptTrajectory(trajectory, base_odometry_->odometry());
}

void OmniBaseController::ResetTrajectory() {
  trajectory_control_->ResetCurrentTrajectory();
}

int32_t OmniBaseController::CheckTorelances(const ControllerBaseState& state,
                                            bool before_last_point,
                                            double time_from_trajectory_end) {
  trajectory_msgs::msg::JointTrajectoryPoint error;
  Convert(state.error, error);

  if (before_last_point) {
    // 軌道追従中なので，経路がズレていないかチェックするだけ
    for (uint32_t i = 0; i < active_tolerances_.state_tolerance.size(); ++i) {
      if (!check_state_tolerance_per_joint(error, i, active_tolerances_.state_tolerance[i])) {
        RCLCPP_ERROR(get_node()->get_logger(), "Path tolerance violated.");
        return control_msgs::action::FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
      }
    }
  } else {
    // ゴールしたかのチェック，時間内なら何もせずに待つ
    bool abort = false;
    for (uint32_t i = 0; i < active_tolerances_.goal_state_tolerance.size(); ++i) {
      if (!check_state_tolerance_per_joint(error, i, active_tolerances_.goal_state_tolerance[i])) {
        abort = true;
        break;
      }
    }
    if (!abort) {
      return control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
    } else if (active_tolerances_.goal_time_tolerance != 0.0) {
      // 0.0との!=は危険だが，デフォルト値が0.0なので，これでいく
      if (time_from_trajectory_end > active_tolerances_.goal_time_tolerance) {
        RCLCPP_ERROR(get_node()->get_logger(), "Goal tolerance violated.");
        return control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
      }
    }
  }
  // 定義されているエラーコードは0以下なので，どれでもないを示すために正の数を返す
  return 1;
}

}  // namespace hsrc_ex_base_controllers

PLUGINLIB_EXPORT_CLASS(hsrc_ex_base_controllers::OmniBaseController,
                       controller_interface::ControllerInterface);
