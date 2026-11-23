#include <gtest/gtest.h>

#include <cmath>

#include "dynamixel_ros_control/servo_law.hpp"

using namespace dynamixel_ros_control;

namespace {

constexpr ServoModel MODEL{1.5, 0.4, 30.0};

ServoCommand powered()
{
  ServoCommand command;
  command.torque_on = true;
  command.position = 1.0;
  command.velocity = 0.2;
  command.current_limit = 2.0;
  return command;
}

}  // namespace

TEST(ServoLaw, PoweredServoPullsTheJointOntoTheReferenceUpToItsCurrentLimit)
{
  const Transmission transmission{2.0, 0.5};
  // Actuator position (0.95 - 0.5) * 2 = 0.9, so 0.1 behind the reference
  const JointDrive drive = servoDrive(powered(), MODEL, transmission, 0.95);
  EXPECT_NEAR(drive.velocity, (0.2 + 30.0 * 0.1) / 2.0, 1e-12);
  EXPECT_NEAR(drive.effort_limit, 1.5 * 2.0 * 2.0, 1e-12);
}

TEST(ServoLaw, NegativeReductionTurnsTheJointTheOtherWayUnderThePositiveLimit)
{
  const Transmission transmission{-2.0, 0.0};
  // Actuator position -0.45 * -2 = 0.9
  const JointDrive drive = servoDrive(powered(), MODEL, transmission, -0.45);
  EXPECT_NEAR(drive.velocity, (0.2 + 30.0 * 0.1) / -2.0, 1e-12);
  EXPECT_NEAR(drive.effort_limit, 1.5 * 2.0 * 2.0, 1e-12);
}

TEST(ServoLaw, LimpServoHoldsOnlyByFriction)
{
  ServoCommand command = powered();
  command.torque_on = false;
  const JointDrive drive = servoDrive(command, MODEL, Transmission{-2.0, 0.3}, 0.7);
  EXPECT_EQ(drive.velocity, 0.0);
  EXPECT_NEAR(drive.effort_limit, 0.4 * 2.0, 1e-12);
}

TEST(ServoLaw, CurrentComesFromTheJointTorqueAndIsZeroWhileLimp)
{
  const Transmission transmission{-2.0, 0.0};
  EXPECT_NEAR(servoCurrent(powered(), MODEL, transmission, 6.0), 6.0 / (-2.0 * 1.5), 1e-12);
  ServoCommand limp = powered();
  limp.torque_on = false;
  EXPECT_EQ(servoCurrent(limp, MODEL, transmission, 6.0), 0.0);
}

TEST(ServoLaw, CurrentNeverExceedsTheCurrentLimit)
{
  // A joint stop or the engine's velocity cap carries 30 Nm, 10 A at the actuator, beyond the 2 A limit
  const Transmission transmission{-2.0, 0.0};
  EXPECT_EQ(servoCurrent(powered(), MODEL, transmission, 30.0), -2.0);
  EXPECT_EQ(servoCurrent(powered(), MODEL, transmission, -30.0), 2.0);
}

TEST(ServoLaw, FollowersShareTheServoEffortAndPullOntoTheirMimic)
{
  const std::vector<MimicFollower> followers{{1.0, 0.0}, {-0.5, 0.1}};
  const double servo_position = followers[0].servoPosition(0.2);
  EXPECT_NEAR(servo_position, 0.2, 1e-12);
  // The second follower's mimic is -0.5 * 0.2 + 0.1 = 0, it lags at -0.02
  const auto drives = followerDrives(JointDrive{0.1, 4.0}, 30.0, servo_position, followers, {0.2, -0.02});
  ASSERT_EQ(drives.size(), 2u);
  EXPECT_NEAR(drives[0].velocity, 0.1, 1e-12);
  EXPECT_NEAR(drives[0].effort_limit, 4.0 / 2.0, 1e-12);
  EXPECT_NEAR(drives[1].velocity, -0.5 * 0.1 + 30.0 * 0.02, 1e-12);
  EXPECT_NEAR(drives[1].effort_limit, 4.0 / (2.0 * 0.5), 1e-12);
}

TEST(ServoLaw, FollowerTorquesAddUpToTheServoTorque)
{
  // Each follower at its share of a 4 Nm servo limit, as when both fingers press on an object
  EXPECT_NEAR(servoTorque({{1.0, 0.0}, {-0.5, 0.1}}, {2.0, -4.0}), 4.0, 1e-12);
}

// arm_joint_1 on athena: homing pi in the motor, offset -pi/4 in the transmission.
// Torque on where the joint stands must leave it there.
TEST(ServoLaw, ArmJoint1HoldsWhereItStandsAtTorqueOn)
{
  auto& manager = MockDynamixelManager::instance();
  manager.reset();
  manager.addMotor(11, 2020);
  const auto motor = manager.getMotor(11);
  motor->setHomingOffset(static_cast<int32_t>(std::lround(M_PI / 0.00000625911)));
  motor->write1Byte(motor->getAddress("operating_mode"), static_cast<uint8_t>(OperatingMode::EXTENDED_POSITION_CONTROL));
  const Transmission transmission{1.0, -M_PI / 4};
  const double joint_position = 0.3;
  motor->followPlant(0.02, transmission.actuatorPosition(joint_position), 0.0, 0.0);

  // What the plugin does at activation
  motor->write4Byte(motor->getAddress("goal_position"), motor->read4Byte(motor->getAddress("present_position")));
  motor->write2Byte(motor->getAddress("goal_current"), 2000);
  motor->write1Byte(motor->getAddress("torque_enable"), 1);
  const ServoCommand command = motor->followPlant(0.02, transmission.actuatorPosition(joint_position), 0.0, 0.0);
  EXPECT_NEAR(servoDrive(command, MODEL, transmission, joint_position).velocity, 0.0, 1e-3);
  manager.reset();
}
