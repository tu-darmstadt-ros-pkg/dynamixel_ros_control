#ifndef DYNAMIXEL_ROS_CONTROL_JOINT_H
#define DYNAMIXEL_ROS_CONTROL_JOINT_H

#include <dynamixel_ros_control/dynamixel.hpp>
#include <dynamixel_ros_control/dynamixel_driver.hpp>
#include <hardware_interface/hardware_info.hpp>

namespace dynamixel_ros_control {

struct State
{
  State()
      : position(0), velocity(0), effort(0), torque(false)
  {}
  double position;
  double velocity;
  double effort;
  bool torque;
};

class Joint
{
public:
  Joint() = default;

  bool init(DynamixelDriver& driver, const hardware_interface::ComponentInfo& info);

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

private:
  ControlMode control_mode;
};

}  // namespace dynamixel_ros_control

#endif
