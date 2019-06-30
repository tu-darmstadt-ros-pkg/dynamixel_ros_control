#include <dynamixel_ros_control/joint.h>

namespace dynamixel_ros_control {

Joint::Joint(dynamixel_ros_control::DynamixelDriver& driver)
  : dynamixel(driver)
{}

Joint::Joint(std::string _name, uint8_t id, DynamixelDriver& driver)
  : name(_name),
    dynamixel(id, driver),
    mounting_offset(0.0),
    offset(0.0),
    control_mode(POSITION)
{}

bool Joint::initFromNh(const ros::NodeHandle& nh)
{
  nh_ = nh;
  nh_.param("mounting_offset", mounting_offset, 0.0);
  nh_.param("offset", offset, 0.0);
}

bool Joint::initDxl()
{
  dynamixel.initFromNh(nh_);
}

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
