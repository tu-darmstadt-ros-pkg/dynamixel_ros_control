#include <dynamixel_ros_control/joint.h>

#include <utility>

namespace dynamixel_ros_control {

Joint::Joint(std::string joint_name, dynamixel_ros_control::DynamixelDriver& driver)
  : name(std::move(joint_name)), dynamixel(driver)
{}

Joint::Joint(std::string _name, uint8_t id, DynamixelDriver& driver)
  : name(std::move(_name)),
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

  // Local control mode can override default control mode
  std::string control_mode_str;
  if (nh.getParam("control_mode", control_mode_str)) {
    try {
      setControlMode(stringToControlMode(control_mode_str));
    } catch(const std::invalid_argument& e) {
      ROS_ERROR_STREAM(e.what() << std::endl << "Using default control mode.");
    }
  }
  return true;
}

bool Joint::initDxl()
{
  return dynamixel.initFromNh(nh_);
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
  return getControlMode() == POSITION || getControlMode() == EXTENDED_POSITION || getControlMode() == CURRENT_BASED_POSITION;
}

bool Joint::isVelocityControlled() const
{
  return getControlMode() == VELOCITY;
}

bool Joint::isEffortControlled() const
{
  return getControlMode() == CURRENT;
}

}
