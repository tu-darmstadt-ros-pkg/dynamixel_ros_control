#include <dynamixel_ros_control/dynamixel_hardware_interface.h>

namespace dynamixel_ros_control {

DynamixelHardwareInterface::DynamixelHardwareInterface(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
{

}

DynamixelHardwareInterface::~DynamixelHardwareInterface()
{
  if (torque_off_on_shutdown_)
  {
    setTorque(false);
  }
}

bool DynamixelHardwareInterface::init()
{
  // Load Parameters
  nh_.param<bool>("debug", debug_, false);
  if (debug_) {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
      ros::console::notifyLoggerLevelsChanged();
    }
  }
  nh_.param("torque_on_startup", torque_on_startup_, false);
  nh_.param("torque_off_on_shutdown", torque_off_on_shutdown_, false);

  ros::NodeHandle dxl_nh(pnh_, "dynamixels");
  if (!driver_.loadDynamixels(dxl_nh, joints_)) {
    return false;
  }

  // Register interfaces
  for (Joint& joint: joints_)
  {
    hardware_interface::JointStateHandle state_handle(joint.name, &joint.current_state.position, &joint.current_state.velocity,
                                                      &joint.current_state.effort);
    jnt_state_interface_.registerHandle(state_handle);

    if (joint.control_mode == POSITION) {
      hardware_interface::JointHandle pos_handle(state_handle, &joint.goal_state.position);
      jnt_pos_interface_.registerHandle(pos_handle);
    } else if (joint.control_mode == VELOCITY) {
      hardware_interface::JointHandle vel_handle(state_handle, &joint.goal_state.velocity);
      jnt_vel_interface_.registerHandle(vel_handle);
    } else if (joint.control_mode == CURRENT) {
      hardware_interface::JointHandle eff_handle(state_handle, &joint.goal_state.effort);
      jnt_eff_interface_.registerHandle(eff_handle);
    }
  }
  registerInterface(&jnt_state_interface_);
  if (!jnt_pos_interface_.getNames().empty())
  {
    registerInterface(&jnt_pos_interface_);
  }
  if (!jnt_vel_interface_.getNames().empty())
  {
    registerInterface(&jnt_vel_interface_);
  }
  if (!jnt_eff_interface_.getNames().empty())
  {
    registerInterface(&jnt_eff_interface_);
  }

  if (torque_on_startup_) {
    setTorque(true);
  }

  // Initialize subscribers and publishers

  return true;
}

void DynamixelHardwareInterface::read()
{

}

void DynamixelHardwareInterface::write()
{

}


void DynamixelHardwareInterface::setTorque(bool enabled)
{

}

void DynamixelHardwareInterface::setTorque(std_msgs::BoolConstPtr enabled)
{

}

}
