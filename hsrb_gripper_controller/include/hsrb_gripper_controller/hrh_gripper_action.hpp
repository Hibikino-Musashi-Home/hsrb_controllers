/*
Copyright (c) 2016 TOYOTA MOTOR CORPORATION
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
#ifndef HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_ACTION_HPP_
#define HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_ACTION_HPP_

#include <limits>
#include <memory>
#include <string>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_server_goal_handle.h>

#include <hsrb_gripper_controller/hrh_gripper_controller.hpp>

namespace hsrb_gripper_controller {

// Default action monitor cycle [Hz]
constexpr double kDefaultActionMonitorRate = 20.0;

// Acquire parameters with default values
template <typename ParameterType>
auto GetParameter(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const std::string& name,
                  const ParameterType& default_value) {
  if (!node->has_parameter(name)) {
    return node->declare_parameter<ParameterType>(name, default_value);
  } else {
    return node->get_parameter(name).get_value<ParameterType>();
  }
}

// Acquire parameters with the default value, use the default value even if the parameters are non -correct
template <typename ParameterType>
auto GetPositiveParameter(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const std::string& name,
                          const ParameterType& default_value) {
  auto value = GetParameter(node, name, default_value);
  if (value < std::numeric_limits<double>::min()) {
    RCLCPP_WARN_STREAM(node->get_logger(), name << " must be positive. Use default value " << default_value);
    value = default_value;
  }
  return value;
}

// Use the default value even if the parameters with the default value are negative.
template <typename ParameterType>
auto GetNonNegativeParameter(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const std::string& name,
                             const ParameterType& default_value) {
  auto value = GetParameter(node, name, default_value);
  if (value < 0.0) {
    RCLCPP_WARN_STREAM(node->get_logger(), name << " must not be negative. Use default value " << default_value);
    value = default_value;
  }
  return value;
}

/// @class IHrhGripperAction
/// @brief Gripper action interface class
class IHrhGripperAction : public std::enable_shared_from_this<IHrhGripperAction> {
 public:
  using Ptr = std::shared_ptr<IHrhGripperAction>;

  virtual ~IHrhGripperAction() = default;

  /// Returns control mode used by action
  virtual int32_t target_mode() const = 0;

  /// Initialization
  /// @param [IN] Sharedptr of node node
  /// @return true: Successful False: Failure
  virtual bool Init(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) = 0;

  /// Activation
  /// @param [IN] Sharedptr of node node
  /// @return true: Successful False: Failure
  virtual bool Activate() = 0;

  /// Circular renewal processing
  /// @param [in] time now
  virtual void Update(const rclcpp::Time& time) = 0;

  /// Insert an active goal
  virtual void PreemptActiveGoal() = 0;
};


/// @class HrhGripperAction
/// @brief Gripper action class
template <class ActionType>
class HrhGripperAction : public IHrhGripperAction {
 public:
  /// constructor
  /// @param [IN] Controller controller
  /// @param [IN] Action_name action name provided
  /// @param [IN] Control mode used by Target_mode action
  HrhGripperAction(HrhGripperController* controller, const std::string& action_name, int32_t target_mode)
      : controller_(controller), action_name_(action_name), target_mode_(target_mode) {}
  virtual ~HrhGripperAction() = default;

  /// Returns control mode used by action
  int32_t target_mode() const override { return target_mode_; }

  /// Initialization
  /// @param [IN] Sharedptr of node node
  /// @return true: Successful False: Failure
  bool Init(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) override {
    node_ = node;

    double action_monitor_rate = GetPositiveParameter(node, "action_monitor_rate", kDefaultActionMonitorRate);
    action_monitor_period_ = 1.0 / action_monitor_rate;

    if (!InitImpl(node)) {
      return false;
    }
    action_server_ = rclcpp_action::create_server<ActionType>(
        node, action_name_,
        std::bind(&HrhGripperAction<ActionType>::GoalCallback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&HrhGripperAction<ActionType>::CancelCallback, this, std::placeholders::_1),
        std::bind(&HrhGripperAction<ActionType>::FeedbackSetupCallback, this, std::placeholders::_1));
    return true;
  }

  /// Activation
  /// @param [IN] Sharedptr of node node
  /// @return true: Successful False: Failure
  bool Activate() override { return true; }

  /// Insert an active goal
  void PreemptActiveGoal() override {
    auto active_goal = *goal_handle_buffer_.readFromNonRT();
    active_goal.reset();
    goal_handle_timer_.reset();
    goal_handle_buffer_.writeFromNonRT(RealtimeGoalHandlePtr());
  }

  /// Check if the action is active
  /// @return True: Active False: Inactive
  bool IsActive() const {
    const auto active_goal = *goal_handle_buffer_.readFromNonRT();
    if (!active_goal || !active_goal->valid()) {
      return false;
    }
    return true;
  }

 protected:
  // Goalhandle for action
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionType>;
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<ActionType>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;

  realtime_tools::RealtimeBuffer<RealtimeGoalHandlePtr> goal_handle_buffer_;

  /// Action server
  typename rclcpp_action::Server<ActionType>::SharedPtr action_server_;

  /// Processing at the time of action reception
  rclcpp_action::GoalResponse GoalCallback(const rclcpp_action::GoalUUID& uuid,
                                           std::shared_ptr<const typename ActionType::Goal> goal) {
    if (!controller_->IsAcceptable()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (ValidateGoal(*goal)) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  /// Processing at the start of action execution
  void FeedbackSetupCallback(std::shared_ptr<GoalHandle> goal_handle) {
    controller_->PreemptActiveGoal();
    controller_->ChangeControlMode(shared_from_this());
    UpdateActionImpl(*goal_handle->get_goal());

    auto realtime_goal_handle = std::make_shared<RealtimeGoalHandle>(goal_handle);
    realtime_goal_handle->execute();
    goal_handle_buffer_.writeFromNonRT(realtime_goal_handle);

    goal_handle_timer_ = node_->create_wall_timer(std::chrono::duration<double>(action_monitor_period_),
                                                  std::bind(&RealtimeGoalHandle::runNonRealtime, realtime_goal_handle));
  }

  /// Processing at the time of action cancellation
  /// @param [IN] Goal_handle goal handle
  rclcpp_action::CancelResponse CancelCallback(const std::shared_ptr<GoalHandle> goal_handle) {
    const auto active_goal = *goal_handle_buffer_.readFromNonRT();
    if (active_goal && active_goal->gh_ == goal_handle) {
      PreemptActiveGoal();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /// Implementation of initialization of action
  virtual bool InitImpl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) { return true; }
  /// Check if the goal is acceptable
  virtual bool ValidateGoal(const typename ActionType::Goal& goal) { return true; }
  /// Update action goals
  virtual void UpdateActionImpl(const typename ActionType::Goal& goal) = 0;

  /// controller
  // std::shared_ptr<HrhGripperController> controller_;
  HrhGripperController* controller_;
  /// Action name
  std::string action_name_;
  /// Control mode for this action
  int32_t target_mode_;
  /// Action status update cycle
  double action_monitor_period_;

  /// Node Sharedptr
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  /// Timer for action execution
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;
};

}  // namespace hsrb_gripper_controller

#endif  // HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_ACTION_HPP_
