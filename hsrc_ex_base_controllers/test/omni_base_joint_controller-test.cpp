/// @file omni_base_joint_controller-test.cpp
/// @brief 全方位台車ジョイントコントローラクラスのテスト
/// @copyright Copyright (C) 2019 Toyota Motor Corporation

#include <fstream>
#include <string>
#include <vector>

#include <boost/thread.hpp>
#include <gtest/gtest.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <hsrc_ex_base_controllers/omni_base_joint_controller.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include "hardware_stub.hpp"
#include "utils.hpp"

namespace {
constexpr double kEpsilon = 1.0e-5;

void spin_some(const rclcpp::Node::SharedPtr& node) {
  while (node) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

}  // namespace

namespace hsrc_ex_base_controllers {

template<typename _JointControllerType, typename _CommandHandleType>
struct TypePair {
  using JointControllerType = _JointControllerType;
  using CommandHandleType = _CommandHandleType;
};

template<typename TypePair>
class OmniBaseJointControllerBaseTest : public ::testing::Test {
 public:
  void SetUp() override;

 protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Node::SharedPtr urdf_node_;
  OmniBaseJointControllerBase::Ptr controller_;
  HardwareStub<typename TypePair::CommandHandleType> hardware_;

  void TestCommandBaseVelocity(const Eigen::Vector3d& base_velocity,
                               const Eigen::Vector3d& expected_joint_velocity);
  void TestCommandBaseVelocity(const Eigen::Vector3d& base_velocity,
                               const Eigen::Vector3d& expected_joint_velocity,
                               const Eigen::Vector3d& expected_interface_command);
};

template<typename TypePair>
void OmniBaseJointControllerBaseTest<TypePair>::SetUp() {
  node_ = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  node_->configure();
  controller_ = std::make_shared<typename TypePair::JointControllerType>(node_);

  auto parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor();
  parameter_descriptor.dynamic_typing = true;
  node_->declare_parameter("joints.steer", "base_roll_joint", parameter_descriptor);
  node_->declare_parameter("joints.r_wheel", "base_r_drive_wheel_joint", parameter_descriptor);
  node_->declare_parameter("joints.l_wheel", "base_l_drive_wheel_joint", parameter_descriptor);

  node_->declare_parameter("robot_description", GetRobotDescription(), parameter_descriptor);
  node_->declare_parameter("parameter_connection_timeout", 0);

  node_->activate();

  urdf_node_ = rclcpp::Node::make_shared("urdf_node");
  urdf_node_->declare_parameter("robot_description", GetRobotDescription());
}

template<typename TypePair>
void OmniBaseJointControllerBaseTest<TypePair>::TestCommandBaseVelocity(
    const Eigen::Vector3d& base_velocity,
    const Eigen::Vector3d& expected_joint_velocity) {
  EXPECT_TRUE(controller_->Init());
  EXPECT_TRUE(controller_->Activate(hardware_.command_interfaces, hardware_.state_interfaces));

  controller_->SetJointCommand(0.1, base_velocity);

  auto joint_command = controller_->joint_command();
  EXPECT_NEAR(expected_joint_velocity(kJointIDRightWheel), joint_command(kJointIDRightWheel), kEpsilon);
  EXPECT_NEAR(expected_joint_velocity(kJointIDLeftWheel), joint_command(kJointIDLeftWheel), kEpsilon);
  EXPECT_NEAR(expected_joint_velocity(kJointIDSteer), joint_command(kJointIDSteer), kEpsilon);
}

template<typename TypePair>
void OmniBaseJointControllerBaseTest<TypePair>::TestCommandBaseVelocity(
    const Eigen::Vector3d& base_velocity,
    const Eigen::Vector3d& expected_joint_velocity,
    const Eigen::Vector3d& expected_interface_command) {
  TestCommandBaseVelocity(base_velocity, expected_joint_velocity);

  EXPECT_NEAR(expected_interface_command(kJointIDRightWheel), hardware_.r_wheel_handle->command(), kEpsilon);
  EXPECT_NEAR(expected_interface_command(kJointIDLeftWheel), hardware_.l_wheel_handle->command(), kEpsilon);
  EXPECT_NEAR(expected_interface_command(kJointIDSteer), hardware_.steer_handle->command(), kEpsilon);
}


typedef ::testing::Types<
    TypePair<OmniBaseJointControllerBaseRollPosition, CommandPositionHandle>,
    TypePair<OmniBaseJointControllerBaseRollVelocity, CommandVelocityHandle>
> TestTypes;

TYPED_TEST_SUITE(OmniBaseJointControllerBaseTest, TestTypes);

/// ステア軸名が用意されたインターフェースのリスト内に無いときに初期化が失敗するか
TYPED_TEST(OmniBaseJointControllerBaseTest, BadSteerJointName) {
  this->node_->set_parameter({rclcpp::Parameter("joints.steer", "bad_joint_name")});
  EXPECT_FALSE(this->controller_->Init());
}

/// 左車輪軸名が用意されたインターフェースのリストと違うときに初期化が失敗するか
TYPED_TEST(OmniBaseJointControllerBaseTest, BadLeftWheelJointName) {
  this->node_->set_parameter({rclcpp::Parameter("joints.l_wheel", "bad_joint_name")});
  EXPECT_FALSE(this->controller_->Init());
}

/// 右車輪軸名が用意されたインターフェースのリストと違うときに初期化が失敗するか
TYPED_TEST(OmniBaseJointControllerBaseTest, BadRightWheelJointName) {
  this->node_->set_parameter({rclcpp::Parameter("joints.r_wheel", "bad_joint_name")});
  EXPECT_FALSE(this->controller_->Init());
}

/// ステア軸名の指定が無いときに初期化が失敗するか
TYPED_TEST(OmniBaseJointControllerBaseTest, NoSteerJointName) {
  this->node_->undeclare_parameter("joints.steer");
  EXPECT_FALSE(this->controller_->Init());
}

/// 左車輪軸名の指定が無いときに初期化が失敗するか
TYPED_TEST(OmniBaseJointControllerBaseTest, NoLeftWheelJointName) {
  this->node_->undeclare_parameter("joints.l_wheel");
  EXPECT_FALSE(this->controller_->Init());
}

/// 右車輪軸名の指定が無いときに初期化が失敗するか
TYPED_TEST(OmniBaseJointControllerBaseTest, NoRightWheelJointName) {
  this->node_->undeclare_parameter("joints.r_wheel");
  EXPECT_FALSE(this->controller_->Init());
}

/// 不正なrobot_descriptionで初期化が失敗するか
TYPED_TEST(OmniBaseJointControllerBaseTest, InvalidRobotDescription) {
  this->node_->set_parameter({rclcpp::Parameter("robot_description", "invalid")});
  EXPECT_FALSE(this->controller_->Init());
}

/// 別のノードのロボットモデルを取得して初期化
TYPED_TEST(OmniBaseJointControllerBaseTest, RobotDescriptionFromAnotherNode) {
  this->node_->undeclare_parameter("robot_description");
  this->node_->declare_parameter("model_node_name", "urdf_node");

  auto spin_thread = std::thread(std::bind(spin_some, std::ref(this->urdf_node_)));
  EXPECT_TRUE(this->controller_->Init());
  this->urdf_node_.reset();
  spin_thread.join();
}

/// 指定されたノードにロボットモデルが存在しない
TYPED_TEST(OmniBaseJointControllerBaseTest, NoRobotDescriptionOnAnotherNode) {
  this->node_->undeclare_parameter("robot_description");
  this->node_->declare_parameter("model_node_name", "no_description_node");
  auto node = rclcpp::Node::make_shared("no_description_node");

  auto spin_thread = std::thread(std::bind(spin_some, std::ref(node)));
  EXPECT_FALSE(this->controller_->Init());
  node.reset();
  spin_thread.join();
}

/// 別のノードを参照しようとするが，存在しないノードが指定されている
TYPED_TEST(OmniBaseJointControllerBaseTest, NoRobotDescriptionNode) {
  this->node_->undeclare_parameter("robot_description");
  EXPECT_FALSE(this->controller_->Init());
}

/// 台車サイズが取得できること
TYPED_TEST(OmniBaseJointControllerBaseTest, GetOmniBaseSize) {
  EXPECT_TRUE(this->controller_->Init());

  auto omnibase_size = this->controller_->omnibase_size();
  EXPECT_EQ(0.266, omnibase_size.tread);
  EXPECT_EQ(0.11, omnibase_size.caster_offset);
  EXPECT_EQ(0.04, omnibase_size.wheel_radius);
}

/// l_wheel_joint_nameが取得できること
TYPED_TEST(OmniBaseJointControllerBaseTest, GetLeftWhellJointName) {
  EXPECT_TRUE(this->controller_->Init());
  EXPECT_EQ(this->controller_->l_wheel_joint_name(), "base_l_drive_wheel_joint");
}

/// r_wheel_joint_nameが取得できること
TYPED_TEST(OmniBaseJointControllerBaseTest, GetRightWhellJointName) {
  EXPECT_TRUE(this->controller_->Init());
  EXPECT_EQ(this->controller_->r_wheel_joint_name(), "base_r_drive_wheel_joint");
}

/// steer_joint_nameが取得できること
TYPED_TEST(OmniBaseJointControllerBaseTest, GetSteerJointName) {
  EXPECT_TRUE(this->controller_->Init());
  EXPECT_EQ(this->controller_->steer_joint_name(), "base_roll_joint");
}

/// 関節の状態を入力するインターフェースの名前が取得できる
TYPED_TEST(OmniBaseJointControllerBaseTest, GetStateInterfaceNames) {
  EXPECT_TRUE(this->controller_->Init());

  auto names = this->controller_->state_interface_names();
  EXPECT_EQ(names.size(), 6);
  EXPECT_NE(std::find(names.begin(), names.end(), "base_roll_joint/position"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "base_roll_joint/velocity"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "base_l_drive_wheel_joint/position"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "base_l_drive_wheel_joint/velocity"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "base_r_drive_wheel_joint/position"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "base_r_drive_wheel_joint/velocity"), names.end());
}

/// command_interfaceが不足してActivateに失敗する
TYPED_TEST(OmniBaseJointControllerBaseTest, CommandInterfaceShortage) {
  EXPECT_TRUE(this->controller_->Init());

  this->hardware_.command_interfaces.clear();
  EXPECT_FALSE(this->controller_->Activate(this->hardware_.command_interfaces, this->hardware_.state_interfaces));
}

/// state_interfaceが不足してActivateに失敗する
TYPED_TEST(OmniBaseJointControllerBaseTest, StateInterfaceShortage) {
  EXPECT_TRUE(this->controller_->Init());

  this->hardware_.state_interfaces.clear();
  EXPECT_FALSE(this->controller_->Activate(this->hardware_.command_interfaces, this->hardware_.state_interfaces));
}

/// 何回でもActivate出来ること
TYPED_TEST(OmniBaseJointControllerBaseTest, ReActivate) {
  EXPECT_TRUE(this->controller_->Init());
  EXPECT_TRUE(this->controller_->Activate(this->hardware_.command_interfaces, this->hardware_.state_interfaces));
  EXPECT_TRUE(this->controller_->Activate(this->hardware_.command_interfaces, this->hardware_.state_interfaces));
}

/// 指令値設定で旋回軸速度リミットがかかること
TYPED_TEST(OmniBaseJointControllerBaseTest, SetJointCommandWithYawLimit) {
  this->TestCommandBaseVelocity(Eigen::Vector3d(0.02, 0.03, 10.0), Eigen::Vector3d(0.260327, -0.0752804, -1.8));
}

/// 指令値設定で車輪速度リミットがかかること
TYPED_TEST(OmniBaseJointControllerBaseTest, SetJointCommandWidhWheelLimit) {
  this->TestCommandBaseVelocity(Eigen::Vector3d(1.0, 0.03, 0.01), Eigen::Vector3d(8.5, 7.90495, 0.0862005));
}

/// 旋回軸速度リミットのパラメータが反映されること
TYPED_TEST(OmniBaseJointControllerBaseTest, SetYawVelocityLimit) {
  this->node_->declare_parameter("yaw_velocity_limit", 0.18);
  this->TestCommandBaseVelocity(Eigen::Vector3d(0.02, 0.03, 10.0), Eigen::Vector3d(0.0260327, -0.00752804, -0.18));

  // 不正なパラメータではデフォルト値が使われる
  this->node_->set_parameter({rclcpp::Parameter("yaw_velocity_limit", -1.0)});
  this->TestCommandBaseVelocity(Eigen::Vector3d(0.02, 0.03, 10.0), Eigen::Vector3d(0.260327, -0.0752804, -1.8));
}

/// 車輪軸速度リミットのパラメータが反映されること
TYPED_TEST(OmniBaseJointControllerBaseTest, SetWheelVelocityLimit) {
  this->node_->declare_parameter("wheel_velocity_limit", 0.85);
  this->TestCommandBaseVelocity(Eigen::Vector3d(1.0, 0.03, 0.01), Eigen::Vector3d(0.85, 0.790495, 0.00862005));

  // 不正なパラメータではデフォルト値が使われる
  this->node_->set_parameter({rclcpp::Parameter("wheel_velocity_limit", -1.0)});
  this->TestCommandBaseVelocity(Eigen::Vector3d(1.0, 0.03, 0.01), Eigen::Vector3d(8.5, 7.90495, 0.0862005));
}

/// 軸位置を取得できること
TYPED_TEST(OmniBaseJointControllerBaseTest, GetJointPositions) {
  EXPECT_TRUE(this->controller_->Init());
  EXPECT_TRUE(this->controller_->Activate(this->hardware_.command_interfaces, this->hardware_.state_interfaces));

  this->hardware_.r_wheel_handle->set_current_pos(0.1);
  this->hardware_.l_wheel_handle->set_current_pos(0.2);
  this->hardware_.steer_handle->set_current_pos(0.3);

  Eigen::Vector3d joint_positions;
  EXPECT_TRUE(this->controller_->GetJointPositions(joint_positions));
  EXPECT_EQ(joint_positions(kJointIDRightWheel), 0.1);
  EXPECT_EQ(joint_positions(kJointIDLeftWheel), 0.2);
  EXPECT_EQ(joint_positions(kJointIDSteer), 0.3);
}

/// 軸速度を取得できること
TYPED_TEST(OmniBaseJointControllerBaseTest, GetJointVelocities) {
  EXPECT_TRUE(this->controller_->Init());
  EXPECT_TRUE(this->controller_->Activate(this->hardware_.command_interfaces, this->hardware_.state_interfaces));

  this->hardware_.r_wheel_handle->set_current_vel(0.1);
  this->hardware_.l_wheel_handle->set_current_vel(0.2);
  this->hardware_.steer_handle->set_current_vel(0.3);

  Eigen::Vector3d joint_velocities;
  EXPECT_TRUE(this->controller_->GetJointVelocities(joint_velocities));
  EXPECT_EQ(joint_velocities(kJointIDRightWheel), 0.1);
  EXPECT_EQ(joint_velocities(kJointIDLeftWheel), 0.2);
  EXPECT_EQ(joint_velocities(kJointIDSteer), 0.3);
}

/// 大きすぎる車輪軸速度はエラーになる
TYPED_TEST(OmniBaseJointControllerBaseTest, TooBigWheelVelocities) {
  EXPECT_TRUE(this->controller_->Init());
  EXPECT_TRUE(this->controller_->Activate(this->hardware_.command_interfaces, this->hardware_.state_interfaces));

  Eigen::Vector3d joint_velocities;
  this->hardware_.r_wheel_handle->set_current_vel(1000.0 + kEpsilon);
  EXPECT_FALSE(this->controller_->GetJointVelocities(joint_velocities));

  this->hardware_.r_wheel_handle->set_current_vel(1000.0 - kEpsilon);
  EXPECT_TRUE(this->controller_->GetJointVelocities(joint_velocities));

  this->node_->set_parameter({rclcpp::Parameter("wheel_actual_velocity_threshold", 1.0)});
  EXPECT_TRUE(this->controller_->Init());

  this->hardware_.r_wheel_handle->set_current_vel(1.0 + kEpsilon);
  EXPECT_FALSE(this->controller_->GetJointVelocities(joint_velocities));

  this->hardware_.r_wheel_handle->set_current_vel(1.0 - kEpsilon);
  EXPECT_TRUE(this->controller_->GetJointVelocities(joint_velocities));

  this->node_->set_parameter({rclcpp::Parameter("wheel_actual_velocity_threshold", -1.0)});
  EXPECT_TRUE(this->controller_->Init());

  this->hardware_.r_wheel_handle->set_current_vel(1000.0 + kEpsilon);
  EXPECT_FALSE(this->controller_->GetJointVelocities(joint_velocities));

  this->hardware_.r_wheel_handle->set_current_vel(1000.0 - kEpsilon);
  EXPECT_TRUE(this->controller_->GetJointVelocities(joint_velocities));
}

/// 大きすぎる旋回軸速度はエラーになる
TYPED_TEST(OmniBaseJointControllerBaseTest, TooBigSteerVelocities) {
  EXPECT_TRUE(this->controller_->Init());
  EXPECT_TRUE(this->controller_->Activate(this->hardware_.command_interfaces, this->hardware_.state_interfaces));

  Eigen::Vector3d joint_velocities;
  this->hardware_.steer_handle->set_current_vel(1000.0 + kEpsilon);
  EXPECT_FALSE(this->controller_->GetJointVelocities(joint_velocities));

  this->hardware_.steer_handle->set_current_vel(1000.0 - kEpsilon);
  EXPECT_TRUE(this->controller_->GetJointVelocities(joint_velocities));

  this->node_->set_parameter({rclcpp::Parameter("yaw_actual_velocity_threshold", 1.0)});
  EXPECT_TRUE(this->controller_->Init());

  this->hardware_.steer_handle->set_current_vel(1.0 + kEpsilon);
  EXPECT_FALSE(this->controller_->GetJointVelocities(joint_velocities));

  this->hardware_.steer_handle->set_current_vel(1.0 - kEpsilon);
  EXPECT_TRUE(this->controller_->GetJointVelocities(joint_velocities));

  this->node_->set_parameter({rclcpp::Parameter("yaw_actual_velocity_threshold", -1.0)});
  EXPECT_TRUE(this->controller_->Init());

  this->hardware_.steer_handle->set_current_vel(1000.0 + kEpsilon);
  EXPECT_FALSE(this->controller_->GetJointVelocities(joint_velocities));

  this->hardware_.steer_handle->set_current_vel(1000.0 - kEpsilon);
  EXPECT_TRUE(this->controller_->GetJointVelocities(joint_velocities));
}


class OmniBaseJointControllerBaseRollPositionTest
    : public OmniBaseJointControllerBaseTest<TypePair<OmniBaseJointControllerBaseRollPosition,
                                                      CommandPositionHandle>> {
};

/// 関節へ指令を投げるインターフェースの名前が取得できる
TEST_F(OmniBaseJointControllerBaseRollPositionTest, GetCommandInterfaceNames) {
  EXPECT_TRUE(controller_->Init());

  auto names = controller_->command_interface_names();
  EXPECT_EQ(names.size(), 3);
  EXPECT_NE(std::find(names.begin(), names.end(), "base_roll_joint/position"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "base_l_drive_wheel_joint/velocity"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "base_r_drive_wheel_joint/velocity"), names.end());
}

/// 指令値が取得できること
TEST_F(OmniBaseJointControllerBaseRollPositionTest, GetJointCommand) {
  TestCommandBaseVelocity(Eigen::Vector3d(0.02, 0.03, 0.01),
                          Eigen::Vector3d(1.40682, -0.406818, 0.262727),
                          Eigen::Vector3d(1.40682, -0.406818, 0.0262727));
}

/// 旋回軸の目標位置をリセットできること
TEST_F(OmniBaseJointControllerBaseRollPositionTest, ResetDesiredSteerPosition) {
  hardware_.steer_handle->set_current_pos(1.0);
  EXPECT_TRUE(controller_->Init());
  EXPECT_TRUE(controller_->Activate(hardware_.command_interfaces, hardware_.state_interfaces));
  EXPECT_DOUBLE_EQ(controller_->desired_steer_pos(), 1.0);

  hardware_.steer_handle->set_current_pos(2.0);
  controller_->ResetDesiredSteerPosition();
  EXPECT_DOUBLE_EQ(controller_->desired_steer_pos(), 2.0);
}


class OmniBaseJointControllerBaseRollVelocityTest
    : public OmniBaseJointControllerBaseTest<TypePair<OmniBaseJointControllerBaseRollVelocity,
                                                      CommandVelocityHandle>> {
};

/// 関節へ指令を投げるインターフェースの名前が取得できる
TEST_F(OmniBaseJointControllerBaseRollVelocityTest, GetCommandInterfaceNames) {
  EXPECT_TRUE(controller_->Init());

  auto names = controller_->command_interface_names();
  EXPECT_EQ(names.size(), 3);
  EXPECT_NE(std::find(names.begin(), names.end(), "base_roll_joint/velocity"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "base_l_drive_wheel_joint/velocity"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "base_r_drive_wheel_joint/velocity"), names.end());
}

/// 指令値が取得できること
TEST_F(OmniBaseJointControllerBaseRollVelocityTest, GetJointCommand) {
  TestCommandBaseVelocity(Eigen::Vector3d(0.02, 0.03, 0.01),
                          Eigen::Vector3d(1.40682, -0.406818, 0.262727),
                          Eigen::Vector3d(1.40682, -0.406818, 0.262727));
}

/// 旋回軸の目標位置は常にゼロであること
TEST_F(OmniBaseJointControllerBaseRollVelocityTest, ResetDesiredSteerPosition) {
  hardware_.steer_handle->set_current_pos(1.0);
  EXPECT_TRUE(controller_->Init());
  EXPECT_TRUE(controller_->Activate(hardware_.command_interfaces, hardware_.state_interfaces));
  EXPECT_DOUBLE_EQ(controller_->desired_steer_pos(), 0.0);

  hardware_.steer_handle->set_current_pos(2.0);
  controller_->ResetDesiredSteerPosition();
  EXPECT_DOUBLE_EQ(controller_->desired_steer_pos(), 0.0);
}

}  // namespace hsrc_ex_base_controllers

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
