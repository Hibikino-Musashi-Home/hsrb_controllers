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
#ifndef HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_APPLY_FORCE_ACTION_HPP_
#define HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_APPLY_FORCE_ACTION_HPP_

#include <memory>
#include <string>
#include <vector>
#include <tmc_control_msgs/action/gripper_apply_effort.hpp>

#include "hsrb_gripper_controller/hrh_gripper_action.hpp"

namespace hsrb_gripper_controller {

/// @class HrhGripperApplyForceCalculator
/// @brief HRH Grippers Finger Tip Power Calculation Class
class HrhGripperApplyForceCalculator {
 public:
  using Ptr = std::shared_ptr<HrhGripperApplyForceCalculator>;
  /// constructor
  HrhGripperApplyForceCalculator();
  /// constructor
  /// @param [IN] Path calibration file path
  HrhGripperApplyForceCalculator(const std::string& calibration_file_path, const rclcpp::Logger& logger);

  virtual ~HrhGripperApplyForceCalculator() = default;

  /// The left and right fingers are integrated and calculated
  // @return Currently finger tip [n]
  double GetCurrentForce(double hand_motor_pos, double left_spring_proximal_joint_pos,
                         double right_spring_proximal_joint_pos) const;

 private:
  /// Read of calibration data for force control
  void LoadForceCalibrationData(const std::string& path, const rclcpp::Logger& logger);

  /// Calculate the current value of finger strength in light of internal force
  /// @return Currently finger tip [n]
  double CalculateForce(double hand_motor_pos, double spring_proximal_joint_pos,
                        const std::vector<std::vector<double> >& calib_data) const;

  /// Calculate the internal force of the grippers from the calibi data
  /// @return Internal force [n]
  double CalculateInternalForce(double hand_motor_pos, const std::vector<double>& calib_p0,
                                const std::vector<double>& calib_p1) const;

  /// Fingering calibration data [N]
  std::vector<std::vector<double>> hand_left_force_calib_data_;
  std::vector<std::vector<double>> hand_right_force_calib_data_;

  /// Spring constant of finger root joint [NM/RAD]
  double hand_spring_coeff_;

  /// Finger length [M]
  double arm_length_;
};


/// @class HrhGripperApplyForceAction
/// @brief HRH Grippers Control Action Class
class HrhGripperApplyForceAction : public HrhGripperAction<tmc_control_msgs::action::GripperApplyEffort> {
 public:
  /// constructor
  /// @param [IN] Controller parent controller
  explicit HrhGripperApplyForceAction(HrhGripperController* controller);
  virtual ~HrhGripperApplyForceAction() = default;

  void Update(const rclcpp::Time& time) override;

  void PreemptActiveGoal() override;

 private:
  /// Implementation of initialization of action
  bool InitImpl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) override;
  /// Update action goals
  void UpdateActionImpl(const tmc_control_msgs::action::GripperApplyEffort::Goal& goal) override;

  /// Tolerance error of goal power [N]
  double goal_tolerance_;
  /// Speed ​​threshold to be determined [RAD/S]
  double stall_velocity_threshold_;
  /// Time to determine Stall [S]
  double stall_timeout_;

  /// PID gain for power control
  double force_control_pgain_;
  double force_control_igain_;
  double force_control_dgain_;

  /// I controlled incorrect integration restriction value
  double force_ierr_max_;
  /// I controlled incorrect integration value buffer
  double force_ierr_buff_;

  /// Low -pass filter coefficient of finger tips
  double force_lpf_coeff_;
  /// Low -pass filter buffer with fingertips [n]
  double force_lpf_buff_;

  /// Refers
  HrhGripperApplyForceCalculator::Ptr force_calculator_;

  /// Directive value buffer
  realtime_tools::RealtimeBuffer<double> command_buffer_;
  /// Action continuation flag buffer
  realtime_tools::RealtimeBuffer<bool> stop_flag_buffer_;

  /// Calculate the target position from the error between the finger tissue and the present value
  /// @return Target location
  double GetCommandPos();
  /// Current finger tip force through the low -pass filter [N]
  double current_force_lpf_;

  /// Action success or failure judgment
  /// @param [in] time now
  void CheckForSuccess(const rclcpp::Time& time);
  /// The last time
  rclcpp::Time last_movement_time_;
};

}  // namespace hsrb_gripper_controller

#endif  // HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_APPLY_FORCE_ACTION_HPP_
