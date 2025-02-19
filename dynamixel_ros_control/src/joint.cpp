#include "dynamixel_ros_control/common.hpp"

#include <dynamixel_ros_control/joint.hpp>

namespace dynamixel_ros_control {

bool Joint::init(DynamixelDriver& driver, const hardware_interface::ComponentInfo& info)
{
  name = info.name;
  getParameter(info.parameters, "mounting_offset", mounting_offset, 0.0);
  getParameter(info.parameters, "offset", offset, 0.0);
  uint8_t id;
  if (!getParameter(info.parameters, "id", id)) {
    return false;
  }
  dynamixel = std::make_shared<Dynamixel>(id, driver);

  return true;
}

ControlMode Joint::getControlMode() const
{
  return control_mode;
}

bool Joint::setControlMode(const ControlMode& value)
{
  control_mode = value;
  return true;
}

bool Joint::isPositionControlled() const
{
  return getControlMode() == POSITION || getControlMode() == EXTENDED_POSITION ||
         getControlMode() == CURRENT_BASED_POSITION;
}

bool Joint::isVelocityControlled() const
{
  return getControlMode() == VELOCITY;
}

bool Joint::isEffortControlled() const
{
  return getControlMode() == CURRENT;
}

}  // namespace dynamixel_ros_control
