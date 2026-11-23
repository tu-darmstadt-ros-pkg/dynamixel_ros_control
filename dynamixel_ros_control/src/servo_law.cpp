#include "dynamixel_ros_control/servo_law.hpp"

#include <algorithm>
#include <cmath>

namespace dynamixel_ros_control {

JointDrive servoDrive(const ServoCommand& command, const ServoModel& model, const Transmission& transmission,
                      const double joint_position)
{
  const double reduction = transmission.reduction;
  if (!command.torque_on) {
    return {0.0, model.friction_torque * std::abs(reduction)};
  }
  const double position_error = command.position - transmission.actuatorPosition(joint_position);
  return {(command.velocity + model.bandwidth * position_error) / reduction,
          model.torque_constant * command.current_limit * std::abs(reduction)};
}

double servoCurrent(const ServoCommand& command, const ServoModel& model, const Transmission& transmission,
                    const double joint_torque)
{
  // A limp servo draws nothing, its friction is the gearbox's
  if (!command.torque_on) {
    return 0.0;
  }
  // Joint limits and the engine's velocity cap carry load beyond the servo's, which it never reports
  const double current = joint_torque / (transmission.reduction * model.torque_constant);
  return std::clamp(current, -command.current_limit, command.current_limit);
}

std::vector<JointDrive> followerDrives(const JointDrive& servo, const double bandwidth, const double servo_position,
                                       const std::vector<MimicFollower>& followers,
                                       const std::vector<double>& follower_positions)
{
  std::vector<JointDrive> drives;
  drives.reserve(followers.size());
  for (size_t i = 0; i < followers.size(); ++i) {
    const MimicFollower& follower = followers[i];
    const double mimic = follower.multiplier * servo_position + follower.offset;
    drives.push_back({follower.multiplier * servo.velocity + bandwidth * (mimic - follower_positions[i]),
                      servo.effort_limit / (static_cast<double>(followers.size()) * std::abs(follower.multiplier))});
  }
  return drives;
}

double servoTorque(const std::vector<MimicFollower>& followers, const std::vector<double>& follower_torques)
{
  double torque = 0.0;
  for (size_t i = 0; i < followers.size(); ++i) {
    torque += followers[i].multiplier * follower_torques[i];
  }
  return torque;
}

}  // namespace dynamixel_ros_control
