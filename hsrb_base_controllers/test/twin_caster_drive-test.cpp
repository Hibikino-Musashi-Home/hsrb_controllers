/*
Copyright (c) 2014 TOYOTA MOTOR CORPORATION
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
/// @brief Exercise model class test of all -sided bogie
#include <cmath>
#include <iostream>
#include <limits>

#include <gtest/gtest.h>

#include <boost/random.hpp>
#include <boost/scoped_ptr.hpp>
#include <Eigen/Core>

#include <hsrb_base_controllers/twin_caster_drive.hpp>


const double kLinearErrorLimit = 0.0001;
const double kAngularErrorLimit = 0.0001;
const double kJointErrorLimit = 0.0001;
const double kOdomLinearErrorLimit = 0.01;
const double kOdomAngularErrorLimit = 0.01;

namespace hsrb_base_controllers {
const OmniBaseSize kOmniBaseCorrectSize = { 1.0, 1.0, 1.0 };

// Initial setting parameter test
TEST(TwinCasterDriveTest, InvalidParameter) {
  // Initialize with invalid parameters
  // Can I make an exception correctly?
  OmniBaseSize minus_tread_size = { -1.0, 1.0, 1.0 };
  EXPECT_ANY_THROW(TwinCasterDrive minus_tread(minus_tread_size));
  OmniBaseSize zero_tread_size = { 0.0, 1.0, 1.0 };
  EXPECT_ANY_THROW(TwinCasterDrive zero_tread(zero_tread_size));
  OmniBaseSize minus_offset_size = { 1.0, -1.0, 1.0 };
  EXPECT_ANY_THROW(TwinCasterDrive minus_offset(minus_offset_size));
  OmniBaseSize minus_radius_size = { 1.0, 1.0, -1.0 };
  EXPECT_ANY_THROW(TwinCasterDrive minus_radius(minus_radius_size));
  OmniBaseSize zero_radius_size = { 1.0, 1.0, 0.0 };
  EXPECT_ANY_THROW(TwinCasterDrive zero_radius(zero_radius_size));
  EXPECT_NO_THROW(TwinCasterDrive correct(kOmniBaseCorrectSize));
}


// Testing whether athletic and reverse athleticiology correspond correctly
TEST(TwinCasterDriveTest, CircularConversion) {
  TwinCasterDrive drive(kOmniBaseCorrectSize);

  // Do you want to return to the original value by performing ordering and reverse athletic conversion at an arbitrary steer axis angle.
  for (int i = -360; i < 360; ++i) {
    const double angle = M_PI * static_cast<double>(i) / 180.0;
    drive.Update(angle);
    Eigen::Vector3d joint_velocities(1.0, 0.0, 0.0);
    Eigen::Vector3d base_velocity = drive.ConvertForward(joint_velocities);
    Eigen::Vector3d joint_velocities2 = drive.ConvertInverse(base_velocity);

    EXPECT_NEAR(joint_velocities2(kJointIDRightWheel),
                joint_velocities(kJointIDRightWheel),
                kJointErrorLimit);
    EXPECT_NEAR(joint_velocities2(kJointIDLeftWheel),
                joint_velocities(kJointIDLeftWheel),
                kJointErrorLimit);
    EXPECT_NEAR(joint_velocities2(kJointIDSteer),
                joint_velocities(kJointIDSteer),
                kJointErrorLimit);
  }
}


// Testing whether the conversion of speed 0 is performed correctly
TEST(TwinCasterDriveTest, ZeroVelocityConversion) {
  TwinCasterDrive drive(kOmniBaseCorrectSize);

  // Half-angle speed-> loading speed
  Eigen::Vector3d joint_velocities(0.0, 0.0, 0.0);
  Eigen::Vector3d base_velocity =
      drive.ConvertForward(joint_velocities);
  EXPECT_NEAR(base_velocity(kIndexBaseX), 0.0, kLinearErrorLimit);
  EXPECT_NEAR(base_velocity(kIndexBaseY), 0.0, kLinearErrorLimit);
  EXPECT_NEAR(base_velocity(kIndexBaseTheta), 0.0, kLinearErrorLimit);

  // Backdle speed-> joint angle speed
  base_velocity << 0.0, 0.0, 0.0;
  joint_velocities = drive.ConvertInverse(base_velocity);
  EXPECT_NEAR(joint_velocities(kJointIDRightWheel), 0.0, kLinearErrorLimit);
  EXPECT_NEAR(joint_velocities(kJointIDLeftWheel), 0.0, kLinearErrorLimit);
  EXPECT_NEAR(joint_velocities(kJointIDSteer), 0.0, kLinearErrorLimit);
}


// Half-angle speed-> Testing whether the loading speed is converted correctly
TEST(TwinCasterDriveTest, ForwardConversion) {
  TwinCasterDrive drive(kOmniBaseCorrectSize);

  // Move the two wheel shaft axis to the same speed and go straight
  // Is it possible to calculate the bogie speed correctly at the arbitrary speed, arbitrary angle of the steer axis
  boost::mt19937 rng(static_cast<uint64_t>(time(0)));
  boost::uniform_real<> linear_dist(-100.0, 100.0);
  boost::variate_generator<
      boost::mt19937, boost::uniform_real<> > linear_rand(rng, linear_dist);
  for (int i = -360; i < 360; ++i) {
    const double angle = M_PI * static_cast<double>(i) / 180.0;
    const double velocity = linear_rand();
    drive.Update(angle);
    Eigen::Vector3d joint_velocities(velocity, velocity, 0.0);
    Eigen::Vector3d base_velocity = drive.ConvertForward(joint_velocities);

    EXPECT_NEAR(
        base_velocity(kIndexBaseX), velocity * cos(angle), kLinearErrorLimit);
    EXPECT_NEAR(
        base_velocity(kIndexBaseY), velocity * sin(angle), kLinearErrorLimit);
    EXPECT_NEAR(base_velocity(kIndexBaseTheta), 0.0, kAngularErrorLimit);
  }

  // Express the speed to move only the steer axis
  // Whether the bogie does not move in parallel direction and only has the speed in the direction of turning
  boost::uniform_real<> angular_dist(-10.0, 10.0);
  boost::variate_generator<
      boost::mt19937, boost::uniform_real<> > angular_rand(rng, angular_dist);
  for (int i = -360; i < 360; ++i) {
    const double angle = M_PI * static_cast<double>(i) / 180.0;
    const double velocity = angular_rand();
    drive.Update(angle);
    Eigen::Vector3d joint_velocities(0.0, 0.0, velocity);
    Eigen::Vector3d base_velocity = drive.ConvertForward(joint_velocities);

    EXPECT_NEAR(base_velocity(kIndexBaseX), 0.0, kLinearErrorLimit);
    EXPECT_NEAR(base_velocity(kIndexBaseY), 0.0, kLinearErrorLimit);
    EXPECT_NEAR(base_velocity(kIndexBaseTheta), -velocity, kAngularErrorLimit);
  }
}


// Captable speed-> Testing whether the joint angle speed is converted correctly
TEST(TwinCasterDriveTest, InverseConversion) {
  TwinCasterDrive drive(kOmniBaseCorrectSize);

  // The steer axis goes straight through the bogie at an arbitrary angle
  // Whether the wheel shaft is moving at the same speed at the correct speed
  boost::mt19937 rng(static_cast<uint64_t>(time(0)));
  boost::uniform_real<> linear_dist(-100.0, 100.0);
  boost::variate_generator<
      boost::mt19937, boost::uniform_real<> > linear_rand(rng, linear_dist);
  for (int i = -360; i < 360; ++i) {
    const double angle = M_PI * static_cast<double>(i) / 180.0;
    const double velocity = linear_rand();
    drive.Update(angle);
    Eigen::Vector3d base_velocity(
        velocity * cos(angle), velocity * sin(angle), 0.0);
    Eigen::Vector3d joint_velocities = drive.ConvertInverse(base_velocity);

    EXPECT_NEAR(
        joint_velocities(kJointIDRightWheel), velocity, kJointErrorLimit);
    EXPECT_NEAR(
        joint_velocities(kJointIDLeftWheel), velocity, kJointErrorLimit);
    EXPECT_NEAR(joint_velocities(kJointIDSteer), 0.0, kJointErrorLimit);
  }

  // The steer axis rotates the bogie at an arbitrary angle
  // Is the steer axis moving at the correct speed?
  boost::uniform_real<> angular_dist(-10.0, 10.0);
  boost::variate_generator<
      boost::mt19937, boost::uniform_real<> > angular_rand(rng, angular_dist);
  for (int i = -360; i < 360; ++i) {
    const double angle = M_PI * static_cast<double>(i) / 180.0;
    const double velocity = angular_rand();
    drive.Update(angle);
    Eigen::Vector3d base_velocity(0.0, 0.0, velocity);
    Eigen::Vector3d joint_velocities = drive.ConvertInverse(base_velocity);

    EXPECT_NEAR(joint_velocities(kJointIDRightWheel), 0.0, kJointErrorLimit);
    EXPECT_NEAR(joint_velocities(kJointIDLeftWheel), 0.0, kJointErrorLimit);
    EXPECT_NEAR(joint_velocities(kJointIDSteer), -velocity, kJointErrorLimit);
  }
}


// Testing whether you can calculate the bogie odometry correctly from the joint angle speed
TEST(TwinCasterDriveTest, BaseOdometryUpdate) {
  boost::scoped_ptr<class TwinCasterDrive> drive(
      new TwinCasterDrive(kOmniBaseCorrectSize));

  boost::mt19937 rng(static_cast<uint64_t>(time(0)));
  boost::uniform_real<> joint_dist(-100.0, 100.0);
  boost::variate_generator<
      boost::mt19937, boost::uniform_real<> > rand_r(rng, joint_dist);

  // Can I calculate the odmetry correctly even if I start from any joint angle?
  for (int i = 0; i < 10; ++i) {
    drive.reset(new TwinCasterDrive(kOmniBaseCorrectSize));

    double period = 0.001;
    Eigen::Vector3d joint_positions(rand_r(), rand_r(), rand_r());
    Eigen::Vector3d joint_velocities = Eigen::Vector3d::Zero();
    Eigen::Vector3d base_odometry =
        drive->UpdateOdometry(period, joint_positions, joint_velocities);

    // Is the odometori zero in the first time?
    EXPECT_NEAR(base_odometry(kIndexBaseX), 0.0, kLinearErrorLimit);
    EXPECT_NEAR(base_odometry(kIndexBaseY), 0.0, kLinearErrorLimit);
    EXPECT_NEAR(base_odometry(kIndexBaseTheta), 0.0, kAngularErrorLimit);

    double velocity = 1.0;
    int32_t update_num = 1000;
    double expected_distance =
        velocity * period * static_cast<double>(update_num);

    // Concert movement-> Specify the joint speed speed that turns on the spot
    // Is the odometori calculated in the correct position?
    Eigen::Vector3d base_velocity(velocity, 0.0 , 0.0);
    for (int j = 0; j < update_num; ++j) {
      drive->Update(joint_positions(kJointIDSteer));
      joint_velocities = drive->ConvertInverse(base_velocity);
      base_odometry =
          drive->UpdateOdometry(period, joint_positions, joint_velocities);
      joint_positions += joint_velocities * period;
    }

    EXPECT_NEAR(
        base_odometry(kIndexBaseX), expected_distance, kOdomLinearErrorLimit);
    EXPECT_NEAR(base_odometry(kIndexBaseY), 0.0, kOdomLinearErrorLimit);
    EXPECT_NEAR(base_odometry(kIndexBaseTheta), 0.0, kOdomAngularErrorLimit);


    base_velocity << 0.0, velocity, 0.0;
    for (int j = 0; j < update_num; ++j) {
      drive->Update(joint_positions(kJointIDSteer));
      joint_velocities = drive->ConvertInverse(base_velocity);
      base_odometry =
          drive->UpdateOdometry(period, joint_positions, joint_velocities);
      joint_positions += joint_velocities * period;
    }

    EXPECT_NEAR(
        base_odometry(kIndexBaseX), expected_distance, kOdomLinearErrorLimit);
    EXPECT_NEAR(
        base_odometry(kIndexBaseY), expected_distance, kOdomLinearErrorLimit);
    EXPECT_NEAR(base_odometry(kIndexBaseTheta), 0.0, kOdomAngularErrorLimit);

    base_velocity << 0.0, 0.0, velocity;
    for (int j = 0; j < update_num; ++j) {
      drive->Update(joint_positions(kJointIDSteer));
      joint_velocities = drive->ConvertInverse(base_velocity);
      base_odometry =
          drive->UpdateOdometry(period, joint_positions, joint_velocities);
      joint_positions += joint_velocities * period;
    }

    EXPECT_NEAR(
        base_odometry(kIndexBaseX), expected_distance, kOdomLinearErrorLimit);
    EXPECT_NEAR(
        base_odometry(kIndexBaseY), expected_distance, kOdomLinearErrorLimit);
    EXPECT_NEAR(
        base_odometry(kIndexBaseTheta), expected_distance, kOdomAngularErrorLimit);
  }
}

}  // namespace hsrb_base_controllers

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
