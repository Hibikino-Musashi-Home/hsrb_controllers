/// @file omni_base_control_method-test.cpp
/// @brief 全方位台車制御モードクラスのテスト
/// @copyright Copyright (C) 2019 Toyota Motor Corporation

#include <gtest/gtest.h>

#include <hsrc_ex_base_controllers/omni_base_control_method.hpp>


namespace {
constexpr double kEpsilon = 1.0e-5;
}  // namespace

namespace hsrc_ex_base_controllers {

class OmniBaseVelocityControlTest : public ::testing::Test {
 protected:
  void SetUp() override;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  OmniBaseVelocityControl::Ptr control_;
  geometry_msgs::msg::Twist::SharedPtr input_;
};

void OmniBaseVelocityControlTest::SetUp() {
  node_ = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  node_->configure();
  node_->declare_parameter("command_timeout", 0.1);
  control_ = std::make_shared<OmniBaseVelocityControl>(node_);
  node_->activate();

  input_ = std::make_shared<geometry_msgs::msg::Twist>();
  input_->linear.x = 0.1;
  input_->linear.y = 0.2;
  input_->angular.z = 0.3;
}

// 速度指令を更新して取得できること
TEST_F(OmniBaseVelocityControlTest, UpdateCommandVelocity) {
  control_->UpdateCommandVelocity(input_);
  auto output = control_->GetOutputVelocity();

  EXPECT_DOUBLE_EQ(output[0], 0.1);
  EXPECT_DOUBLE_EQ(output[1], 0.2);
  EXPECT_DOUBLE_EQ(output[2], 0.3);
}

// 指令速度が一定区間ないと速度が0になること
TEST_F(OmniBaseVelocityControlTest, OutputVelocityIsZero) {
  control_->UpdateCommandVelocity(input_);
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.2)));
  auto output = control_->GetOutputVelocity();

  EXPECT_DOUBLE_EQ(output[0], 0.0);
  EXPECT_DOUBLE_EQ(output[1], 0.0);
  EXPECT_DOUBLE_EQ(output[2], 0.0);
}

// 指令速度をクリアできること
TEST_F(OmniBaseVelocityControlTest, Activate) {
  control_->UpdateCommandVelocity(input_);
  control_->Activate();
  auto output = control_->GetOutputVelocity();

  EXPECT_DOUBLE_EQ(output[0], 0.0);
  EXPECT_DOUBLE_EQ(output[1], 0.0);
  EXPECT_DOUBLE_EQ(output[2], 0.0);
}


class OmniBaseTrajectoryControlTest : public ::testing::Test {
 protected:
  void SetUp() override;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  OmniBaseTrajectoryControl::Ptr control_;
  trajectory_msgs::msg::JointTrajectory::SharedPtr input_trajectory_;
};

void OmniBaseTrajectoryControlTest::SetUp() {
  node_ = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  node_->configure();
  std::vector<std::string> base_coordinates = {"odom_x", "odom_y", "odom_t"};
  control_ = std::make_shared<OmniBaseTrajectoryControl>(node_, base_coordinates);

  node_->activate();
  control_->Activate();

  input_trajectory_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  input_trajectory_->header.stamp = node_->now();
  input_trajectory_->joint_names = base_coordinates;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.resize(base_coordinates.size(), 0.0);
  point.velocities.resize(base_coordinates.size(), 0.0);
  point.accelerations.resize(base_coordinates.size(), 0.0);
  point.time_from_start = rclcpp::Duration(10, 0);
  input_trajectory_->points.push_back(point);
}

// 軌道追従モードで指令速度を取得できること
TEST_F(OmniBaseTrajectoryControlTest, GetTrajOutputVelocity) {
  ControllerState state;
  state.actual.positions = {0.0, 0.0, 0.0};
  state.error.positions = {1.0, 2.0, 3.0};
  state.desired.velocities = {1.1, 1.2, 1.3};

  // Pゲインのデフォルト値は1.0
  auto output_vel = control_->GetOutputVelocity(state);
  EXPECT_NEAR(output_vel[0], 2.1, kEpsilon);
  EXPECT_NEAR(output_vel[1], 3.2, kEpsilon);
  EXPECT_NEAR(output_vel[2], 4.3, kEpsilon);

  state.actual.positions = {0.0, 0.0, M_PI};

  output_vel = control_->GetOutputVelocity(state);
  EXPECT_NEAR(output_vel[0], -2.1, kEpsilon);
  EXPECT_NEAR(output_vel[1], -3.2, kEpsilon);
  EXPECT_NEAR(output_vel[2], 4.3, kEpsilon);
}

// 軌道追従モードでゲインを反映した指令速度を取得できること
TEST_F(OmniBaseTrajectoryControlTest, GetTrajOutputVelocityWithGain) {
  node_->set_parameter({rclcpp::Parameter("odom_x.p_gain", 2.0)});
  node_->set_parameter({rclcpp::Parameter("odom_y.p_gain", 3.0)});
  node_->set_parameter({rclcpp::Parameter("odom_t.p_gain", 4.0)});

  std::vector<std::string> base_coordinates = {"odom_x", "odom_y", "odom_t"};
  control_ = std::make_shared<OmniBaseTrajectoryControl>(node_, base_coordinates);
  control_->Activate();

  ControllerState state;
  state.actual.positions = {0.0, 0.0, 0.0};
  state.error.positions = {1.0, 2.0, 3.0};
  state.desired.velocities = {1.1, 1.2, 1.3};

  auto output_vel = control_->GetOutputVelocity(state);
  EXPECT_NEAR(output_vel[0], 3.1, kEpsilon);
  EXPECT_NEAR(output_vel[1], 7.2, kEpsilon);
  EXPECT_NEAR(output_vel[2], 13.3, kEpsilon);
}

// 不正なゲインを指定した際は，デフォルト値が使われること
TEST_F(OmniBaseTrajectoryControlTest, NotPositivePGain) {
  node_->set_parameter({rclcpp::Parameter("odom_x.p_gain", 0.0)});
  node_->set_parameter({rclcpp::Parameter("odom_y.p_gain", 0.0)});
  node_->set_parameter({rclcpp::Parameter("odom_t.p_gain", 0.0)});

  std::vector<std::string> base_coordinates = {"odom_x", "odom_y", "odom_t"};
  control_ = std::make_shared<OmniBaseTrajectoryControl>(node_, base_coordinates);
  control_->Activate();

  ControllerState state;
  state.actual.positions = {0.0, 0.0, 0.0};
  state.error.positions = {1.0, 2.0, 3.0};
  state.desired.velocities = {1.1, 1.2, 1.3};

  auto output_vel = control_->GetOutputVelocity(state);
  EXPECT_NEAR(output_vel[0], 2.1, kEpsilon);
  EXPECT_NEAR(output_vel[1], 3.2, kEpsilon);
  EXPECT_NEAR(output_vel[2], 4.3, kEpsilon);
}

// 軌道の有無のチェック
TEST_F(OmniBaseTrajectoryControlTest, UpdateActiveTrajectory) {
  EXPECT_FALSE(control_->UpdateActiveTrajectory());

  // 有効な軌道がacceptされたら，軌道があるうちは，Updateは何回でも成功する
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  control_->ResetCurrentTrajectory();
  EXPECT_FALSE(control_->UpdateActiveTrajectory());

  input_trajectory_->points.clear();
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_FALSE(control_->UpdateActiveTrajectory());
}

// あるべき状態の取得
TEST_F(OmniBaseTrajectoryControlTest, SampleDesiredState) {
  input_trajectory_->points.back().positions = {1.0, 2.0, 3.0};
  input_trajectory_->points.back().velocities.clear();
  input_trajectory_->points.back().accelerations.clear();
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  std::vector<double> current_positions = {0.0, 0.0, 0.0};
  std::vector<double> current_velocities = {0.0, 0.0, 0.0};

  trajectory_msgs::msg::JointTrajectoryPoint desired_state;
  bool before_last_point;
  double time_from_point;

  rclcpp::Time stamp = input_trajectory_->header.stamp;
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.2);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.3);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -10.0);

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(5, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.5);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 1.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 1.5);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.2);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.3);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -5.0);

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(10, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 1.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 2.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 3.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.0);
  EXPECT_FALSE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, 0.0);

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(11, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 1.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 2.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 3.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.0);
  EXPECT_FALSE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, 1.0);
}

// あるべき状態の取得
TEST_F(OmniBaseTrajectoryControlTest, SampleDesiredStateWithOpenLoop) {
  node_->set_parameter({rclcpp::Parameter("open_loop_control", true)});

  std::vector<std::string> base_coordinates = {"odom_x", "odom_y", "odom_t"};
  control_ = std::make_shared<OmniBaseTrajectoryControl>(node_, base_coordinates);
  control_->Activate();

  input_trajectory_->points.back().positions = {1.0, 2.0, 3.0};
  input_trajectory_->points.back().velocities.clear();
  input_trajectory_->points.back().accelerations.clear();
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  std::vector<double> current_positions = {0.0, 0.0, 0.0};
  std::vector<double> current_velocities = {0.0, 0.0, 0.0};

  trajectory_msgs::msg::JointTrajectoryPoint desired_state;
  bool before_last_point;
  double time_from_point;

  rclcpp::Time stamp = input_trajectory_->header.stamp;
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.2);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.3);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -10.0);

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(5, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.5);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 1.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 1.5);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.2);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.3);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -5.0);

  input_trajectory_->header.stamp.sec += 5;
  // πを超えるために旋回軸の入力を調整
  input_trajectory_->points.back().positions = {1.5, 2.0, 3.5 - 2 * M_PI};
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  // open_loop_controlがtrueなので，さっきサンプリングした時刻の状態基準となる
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.5);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 1.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 1.5);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.2);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -10.0);

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(5, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 1.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 1.5);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 2.5);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.2);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -5.0);

  // 追従を一旦完了させる
  ControllerState zero_velocity_state;
  zero_velocity_state.actual.velocities = {0.0, 0.0, 0.0};
  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(15, 0);
  control_->TerminateControl(stamp, zero_velocity_state);
  EXPECT_FALSE(control_->UpdateActiveTrajectory());

  // 軌道追従完了後は，現在の状態からサンプリング，速度制御等で現在位置が変わっている可能性があるため
  input_trajectory_->header.stamp = stamp;
  input_trajectory_->points.back().positions = {1.0, 2.0, 3.0};
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.2);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.3);

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(5, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.5);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 1.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 1.5);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.2);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.3);
}

// あるべき状態の取得
TEST_F(OmniBaseTrajectoryControlTest, SampleDesiredStateWithoutOpenLoop) {
  node_->set_parameter({rclcpp::Parameter("open_loop_control", false)});

  std::vector<std::string> base_coordinates = {"odom_x", "odom_y", "odom_t"};
  control_ = std::make_shared<OmniBaseTrajectoryControl>(node_, base_coordinates);
  control_->Activate();

  input_trajectory_->points.back().positions = {1.0, 2.0, 3.0};
  input_trajectory_->points.back().velocities.clear();
  input_trajectory_->points.back().accelerations.clear();
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  std::vector<double> current_positions = {0.0, 0.0, 0.0};
  std::vector<double> current_velocities = {0.0, 0.0, 0.0};

  trajectory_msgs::msg::JointTrajectoryPoint desired_state;
  bool before_last_point;
  double time_from_point;

  rclcpp::Time stamp = input_trajectory_->header.stamp;
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.2);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.3);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -10.0);

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(5, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.5);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 1.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 1.5);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.2);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.3);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -5.0);

  input_trajectory_->header.stamp.sec += 5;
  input_trajectory_->points.back().positions = {1.5, 2.0, -3.0};
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  // open_loop_controlがfalseなので，current_positions/velocitiesを基準にサンプリング
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.15);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.2);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], -0.3);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -10.0);

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(5, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.75);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 1.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], -1.5);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.15);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.2);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], -0.3);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -5.0);
}

// 関節名の並べ替えが行われる
TEST_F(OmniBaseTrajectoryControlTest, PermutatedTrajectory) {
  input_trajectory_->joint_names = {input_trajectory_->joint_names[0],
                                    input_trajectory_->joint_names[2],
                                    input_trajectory_->joint_names[1]};
  input_trajectory_->points.back().positions = {1.0, 2.0, 3.0};
  input_trajectory_->points.back().velocities.clear();
  input_trajectory_->points.back().accelerations.clear();
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  std::vector<double> current_positions = {0.0, 0.0, 0.0};
  std::vector<double> current_velocities = {0.0, 0.0, 0.0};

  trajectory_msgs::msg::JointTrajectoryPoint desired_state;
  bool before_last_point;
  double time_from_point;

  rclcpp::Time stamp = input_trajectory_->header.stamp;
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(5, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.5);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 1.5);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 1.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.3);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.2);
}

// 旋回軸の補正が行われている
TEST_F(OmniBaseTrajectoryControlTest, OverPISteerTrajectory) {
  input_trajectory_->points.back().positions = {1.0, 2.0, M_PI - 1.0};
  input_trajectory_->points.back().velocities.clear();
  input_trajectory_->points.back().accelerations.clear();
  input_trajectory_->points.push_back(input_trajectory_->points.back());
  input_trajectory_->points.back().positions = {1.0, 2.0, -M_PI + 1.0};
  input_trajectory_->points.back().time_from_start = rclcpp::Duration(20, 0);
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  std::vector<double> current_positions = {0.0, 0.0, 0.0};
  std::vector<double> current_velocities = {0.0, 0.0, 0.0};

  trajectory_msgs::msg::JointTrajectoryPoint desired_state;
  bool before_last_point;
  double time_from_point;

  rclcpp::Time stamp = input_trajectory_->header.stamp;
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(15, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[2], M_PI);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.2);
}

// ValidateTrajectoryの正常系
TEST_F(OmniBaseTrajectoryControlTest, ValidateTrajectory) {
  EXPECT_TRUE(control_->ValidateTrajectory(*input_trajectory_));
}

// JointTrajectoryの有効性チェックNG：Joint数不一致
TEST_F(OmniBaseTrajectoryControlTest, DoNotMatchJointsSize) {
  std::vector<std::string> base_coordinates = {"odom_x", "odom_y", "odom_t", "test_joint"};
  control_ = std::make_shared<OmniBaseTrajectoryControl>(node_, base_coordinates);
  control_->Activate();
  EXPECT_FALSE(control_->ValidateTrajectory(*input_trajectory_));
}

// JointTrajectoryの有効性チェックNG：positionの要素数がJoint数と不一致
TEST_F(OmniBaseTrajectoryControlTest, DoNotMatchPositionSize) {
  input_trajectory_->points.back().positions.push_back(0.0);
  EXPECT_FALSE(control_->ValidateTrajectory(*input_trajectory_));
}

// JointTrajectoryの有効性チェックNG：velocityの要素数がJoint数と不一致
TEST_F(OmniBaseTrajectoryControlTest, DoNotMatchVelocitySize) {
  input_trajectory_->points.back().velocities.push_back(0.0);
  EXPECT_FALSE(control_->ValidateTrajectory(*input_trajectory_));
}

// JointTrajectoryの有効性チェックOK：velocityの要素数が空
TEST_F(OmniBaseTrajectoryControlTest, VelocityIsEmpty) {
  input_trajectory_->points.back().velocities.clear();
  EXPECT_TRUE(control_->ValidateTrajectory(*input_trajectory_));
}

// JointTrajectoryの有効性チェックNG：accelerationの要素数がJoint数と不一致
TEST_F(OmniBaseTrajectoryControlTest, DoNotMatchAccelerationSize) {
  input_trajectory_->points.back().accelerations.push_back(0.0);
  EXPECT_FALSE(control_->ValidateTrajectory(*input_trajectory_));
}

// JointTrajectoryの有効性チェックOK：accelerationの要素数が空
TEST_F(OmniBaseTrajectoryControlTest, AccelerationIsEmpty) {
  input_trajectory_->points.back().accelerations.clear();
  EXPECT_TRUE(control_->ValidateTrajectory(*input_trajectory_));
}

// JointTrajectoryの有効性チェックNG：time_from_startが逆行していれば無効
TEST_F(OmniBaseTrajectoryControlTest, TimeFromStartIdGoingReverse) {
  input_trajectory_->points.push_back(input_trajectory_->points.back());
  input_trajectory_->points.back().time_from_start = rclcpp::Duration(1, 0);
  EXPECT_FALSE(control_->ValidateTrajectory(*input_trajectory_));
}

// 不正な関節名を含んでいる
TEST_F(OmniBaseTrajectoryControlTest, IncludeInvalidJointName) {
  input_trajectory_->joint_names[0] = "unknown";
  EXPECT_FALSE(control_->ValidateTrajectory(*input_trajectory_));
}

// 停止状態＋時間経過で軌道追従を完了させる
TEST_F(OmniBaseTrajectoryControlTest, TerminateControl) {
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  ControllerState state;
  state.actual.velocities = {0.1, 0.0, 0.0};

  auto stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(10, 0);
  control_->TerminateControl(stamp, state);
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  state.actual.velocities = {0.0, 0.0, 0.0};
  control_->TerminateControl(stamp, state);
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(11, 0);
  state.actual.velocities = {0.1, 0.0, 0.0};
  control_->TerminateControl(stamp, state);
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  state.actual.velocities = {0.0, 0.0, 0.0};
  control_->TerminateControl(stamp, state);
  EXPECT_FALSE(control_->UpdateActiveTrajectory());
}

// 停止状態の閾値を変化させる
TEST_F(OmniBaseTrajectoryControlTest, ChangeStopVelocityThreshold) {
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  ControllerState state;
  state.actual.velocities = {0.001 + 1e-6, 0.0, 0.0};

  auto stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(11, 0);
  control_->TerminateControl(stamp, state);
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  state.actual.velocities = {0.001 - 1e-6, 0.0, 0.0};
  control_->TerminateControl(stamp, state);
  EXPECT_FALSE(control_->UpdateActiveTrajectory());

  node_->set_parameter({rclcpp::Parameter("stop_velocity_threshold", 0.0)});

  std::vector<std::string> base_coordinates = {"odom_x", "odom_y", "odom_t"};
  control_ = std::make_shared<OmniBaseTrajectoryControl>(node_, base_coordinates);
  control_->Activate();
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  state.actual.velocities = {0.001 + 1e-6, 0.0, 0.0};
  control_->TerminateControl(stamp, state);
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  state.actual.velocities = {0.001 - 1e-6, 0.0, 0.0};
  control_->TerminateControl(stamp, state);
  EXPECT_FALSE(control_->UpdateActiveTrajectory());

  node_->set_parameter({rclcpp::Parameter("stop_velocity_threshold", 0.1)});

  control_ = std::make_shared<OmniBaseTrajectoryControl>(node_, base_coordinates);
  control_->Activate();
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  state.actual.velocities = {0.1 + 1e-6, 0.0, 0.0};
  control_->TerminateControl(stamp, state);
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  state.actual.velocities = {0.1 - 1e-6, 0.0, 0.0};
  control_->TerminateControl(stamp, state);
  EXPECT_FALSE(control_->UpdateActiveTrajectory());
}

}  // namespace hsrc_ex_base_controllers

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
