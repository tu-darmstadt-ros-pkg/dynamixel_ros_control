#ifndef DYNAMIXEL_ROS_CONTROL_SERVO_LAW_HPP
#define DYNAMIXEL_ROS_CONTROL_SERVO_LAW_HPP

#include <vector>

#include "dynamixel_ros_control/mock_dynamixel.hpp"

namespace dynamixel_ros_control {

/// @brief Constants of one servo model at its output shaft.
struct ServoModel
{
  double torque_constant = 0.0;  ///< Nm per A
  double friction_torque = 0.0;  ///< Nm, what holds a limp servo against its load
  double bandwidth = 30.0;       ///< 1/s, how hard the drive pulls the plant onto the reference
};

/// @brief The transmission of the ros2_control xacro, joint = actuator / reduction + offset. Joint
/// quantities are rad or m, the latter for a prismatic joint such as a gripper finger.
struct Transmission
{
  double reduction = 1.0;  ///< actuator rad per joint rad or m
  double offset = 0.0;     ///< rad or m

  [[nodiscard]] double actuatorPosition(double joint_position) const
  {
    return (joint_position - offset) * reduction;
  }
  [[nodiscard]] double actuatorVelocity(double joint_velocity) const
  {
    return joint_velocity * reduction;
  }
};

/// @brief What a physics engine applies to one joint, a velocity servo under an effort limit.
struct JointDrive
{
  double velocity = 0.0;      ///< rad/s or m/s
  double effort_limit = 0.0;  ///< Nm or N
};

/// @brief The drive that makes a joint follow the servo's reference. A limp servo holds only by
/// friction, a powered one gives way at its current limit as a stalled Dynamixel does.
JointDrive servoDrive(const ServoCommand& command, const ServoModel& model, const Transmission& transmission,
                      double joint_position);

/// @brief The current the servo reports for the torque its joint transmits about the axis.
double servoCurrent(const ServoCommand& command, const ServoModel& model, const Transmission& transmission,
                    double joint_torque);

/// @brief A joint that moves with the servo joint, follower = multiplier * servo + offset, as a URDF
/// mimic says. It stands in for the mimic constraint an engine without one cannot build.
struct MimicFollower
{
  double multiplier = 1.0;
  double offset = 0.0;

  [[nodiscard]] double servoPosition(double follower_position) const
  {
    return (follower_position - offset) / multiplier;
  }
};

/// @brief Drives for the followers that carry a servo joint. Each pulls onto its mimic of the servo
/// joint and gets an equal share of the servo's effort.
std::vector<JointDrive> followerDrives(const JointDrive& servo, double bandwidth, double servo_position,
                                       const std::vector<MimicFollower>& followers,
                                       const std::vector<double>& follower_positions);

/// @brief The servo joint torque the followers' torques add up to, by virtual work.
double servoTorque(const std::vector<MimicFollower>& followers, const std::vector<double>& follower_torques);

}  // namespace dynamixel_ros_control

#endif  // DYNAMIXEL_ROS_CONTROL_SERVO_LAW_HPP
