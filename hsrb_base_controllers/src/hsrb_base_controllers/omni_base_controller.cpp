/*
Copyright (c) 2015 TOYOTA MOTOR CORPORATION
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
/// @file omni_base_controller.cpp
/// @brief Omnidistant bogie controller class

#include <hsrb_base_controllers/omni_base_controller.hpp>

#include <pluginlib/class_list_macros.hpp>

#include "utils.hpp"


namespace hsrb_base_controllers {

controller_interface::CallbackReturn OmniBaseController::on_init() {
  // There is all processing in ON_CONFIGURE and ON_ACTIVATE that do nothing
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

  // Acquire the position / speed of each axis of the bogie
  Eigen::Vector3d joint_positions;
  Eigen::Vector3d joint_velocities;
  if (!joint_controller_->GetJointPositions(joint_positions) ||
      !joint_controller_->GetJointVelocities(joint_velocities)) {
    return controller_interface::return_type::ERROR;
  }

  const double period_sec = period.seconds();
  // Update of odmetry
  wheel_odometry_->UpdateOdometry(period_sec, joint_positions, joint_velocities);
  base_odometry_->UpdateOdometry(period_sec, wheel_odometry_->odometry(), wheel_odometry_->velocity());

  // Update the truck following
  ControllerBaseState base_state(base_odometry_->odometry(), base_odometry_->velocity());

  Eigen::Vector3d output_velocity = Eigen::Vector3d::Zero();
  if (trajectory_control_->UpdateActiveTrajectory()) {
    // Tracking
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
    // Speed ​​tracking
    output_velocity = velocity_control_->GetOutputVelocity();
  }
  joint_controller_->SetJointCommand(period_sec, output_velocity);

  // Publish the current state of the bogie (command value, current location, difference)
  const ControllerJointState joint_state(joint_positions,
                                         joint_velocities,
                                         joint_controller_->desired_steer_pos(),
                                         joint_controller_->joint_command());
  joint_state_publisher_->Publish(joint_state, current_time);
  base_state_publisher_->Publish(base_state, current_time);

  // Wheel and dometry, publish TF
  wheel_odometry_->PublishOdometry(current_time);

  trajectory_control_->TerminateControl(current_time, base_state);

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn OmniBaseController::on_configure(const rclcpp_lifecycle::State& previous_state) {
  // Obtain the coordinates axis of the controlled bogie
  const auto base_coordinate_names = GetParameter<std::vector<std::string>>(get_node(), "base_coordinates", {});
  if (base_coordinate_names.size() != kNumBaseCoordinateIDs) {
    RCLCPP_ERROR(get_node()->get_logger(), "The size of joints must be three.");
    return controller_interface::CallbackReturn::ERROR;
  }
  default_tolerances_ = get_segment_tolerances(get_node(), base_coordinate_names);

  // Set a speed subclan
  velocity_subscriber_ = std::make_shared<CommandVelocitySubscriber>(get_node(), this);
  // Set track subclan
  trajectory_subscriber_ = std::make_shared<CommandTrajectorySubscriber>(get_node(), this);
  // Action server standing up
  trajectory_action_ = std::make_shared<TrajectoryActionServer>(get_node(), base_coordinate_names, this);

  // Joint controller class
  joint_controller_ = std::make_shared<OmniBaseJointController>(get_node());
  if (!joint_controller_->Init()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Initializing OmniBaseJointController is failed.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Set a wheeldeodometry publisher
  wheel_odometry_ = std::make_shared<WheelOdometry>(get_node(), joint_controller_->omnibase_size());
  // Set a bogie Odometri's publisher
  base_odometry_ = std::make_shared<BaseOdometry>(get_node());

  // Generate a controller
  velocity_control_ = std::make_shared<OmniBaseVelocityControl>(get_node());
  trajectory_control_ = std::make_shared<OmniBaseTrajectoryControl>(get_node(), base_coordinate_names);

  // Set a publisher in the internal joint
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
  // ACTIVATE here according to JointTrajectoryController
  velocity_control_->Activate();
  trajectory_control_->Activate();

  const auto current_time = get_node()->get_clock()->now();
  wheel_odometry_->set_last_odometry_published_time(current_time);
  wheel_odometry_->set_last_transform_published_time(current_time);
  joint_state_publisher_->set_last_state_published_time(current_time);
  base_state_publisher_->set_last_state_published_time(current_time);

  // Initialize the odmetry at the time of launch
  base_odometry_->InitOdometry();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmniBaseController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
  // Reset the current action goal when the controller is down
  trajectory_action_->PreemptActiveGoal();
  // Clear command speed
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
    // Since the trajectory is being followed, just check if the route is out of order.
    for (uint32_t i = 0; i < active_tolerances_.state_tolerance.size(); ++i) {
      if (!check_state_tolerance_per_joint(error, i, active_tolerances_.state_tolerance[i])) {
        RCLCPP_ERROR(get_node()->get_logger(), "Path tolerance violated.");
        return control_msgs::action::FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
      }
    }
  } else {
    // Check if you finished the goal, wait without doing anything within time
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
      // It is dangerous to say 0.0, but the default value is 0.0, so this is
      if (time_from_trajectory_end > active_tolerances_.goal_time_tolerance) {
        RCLCPP_ERROR(get_node()->get_logger(), "Goal tolerance violated.");
        return control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
      }
    }
  }
  // Since the defined error code is 0 or less, return the positive number to indicate nothing.
  return 1;
}

}  // namespace hsrb_base_controllers

PLUGINLIB_EXPORT_CLASS(hsrb_base_controllers::OmniBaseController,
                       controller_interface::ControllerInterface);
