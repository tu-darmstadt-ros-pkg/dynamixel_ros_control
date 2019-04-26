#include <dynamixel_ros_control/joint.h>

namespace dynamixel_ros_control {

Joint::Joint(std::string _name, uint8_t id, uint16_t model_number, DynamixelDriver& driver)
  : name(_name),
    dynamixel(id, model_number, driver),
    mounting_offset(0.0),
    offset(0.0),
    control_mode(POSITION)
{}

ControlMode Joint::getControlMode() const
{
  return control_mode;
}

bool Joint::setControlMode(const ControlMode& value)
{
  control_mode = value;
}

bool Joint::isPositionControlled()
{
  return getControlMode() == POSITION || getControlMode() == EXTENDED_POSITION || getControlMode() == CURRENT_BASED_POSITION;
}

bool Joint::isVelocityControlled()
{
  return getControlMode() == VELOCITY;
}

bool Joint::isEffortControlled()
{
  return getControlMode() == CURRENT;
}

}
