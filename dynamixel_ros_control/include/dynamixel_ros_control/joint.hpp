#ifndef DYNAMIXEL_ROS_CONTROL_JOINT_H
#define DYNAMIXEL_ROS_CONTROL_JOINT_H

#include <dynamixel_ros_control/dynamixel.hpp>
#include <dynamixel_ros_control/dynamixel_driver.hpp>
#include <hardware_interface/hardware_info.hpp>

namespace dynamixel_ros_control {

struct State
{
  double position{0};
  double velocity{0};
  double effort{0};
  bool torque{false};
};

class Joint
{
public:
  Joint() = default;

  bool loadConfiguration(DynamixelDriver& driver, const hardware_interface::ComponentInfo& info);

  std::string name;
  std::shared_ptr<Dynamixel> dynamixel;

  State current_state;
  State goal_state;

  double mounting_offset{0.0};
  double offset{0.0};

  double estop_position{0.0};

  ControlMode getControlMode() const;
  bool setControlMode(const ControlMode& value);

  bool isPositionControlled() const;
  bool isVelocityControlled() const;
  bool isEffortControlled() const;
  const std::vector<std::string>& getStateInterfaces() const;

private:
  ControlMode control_mode;
  std::vector<std::string> state_interfaces;
};

}  // namespace dynamixel_ros_control

#endif
