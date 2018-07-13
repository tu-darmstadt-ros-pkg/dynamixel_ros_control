#include <dynamixel_ros_control/dynamixel_hardware_interface.h>

namespace dynamixel_ros_control {

DynamixelHardwareInterface::DynamixelHardwareInterface(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
{

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
  for (unsigned int i = 0; i < joint_names_.size(); i++)
  {
    hardware_interface::JointStateHandle state_handle(joint_names_[i], &current_position_[i], &current_velocity_[i], &current_effort_[i]);
    jnt_state_interface_.registerHandle(state_handle);

    hardware_interface::JointHandle pos_handle(state_handle, &goal_position_[i]);
    jnt_pos_interface_.registerHandle(pos_handle);

    hardware_interface::JointHandle vel_handle(state_handle, &goal_velocity_[i]);
    jnt_vel_interface_.registerHandle(vel_handle);

    hardware_interface::JointHandle eff_handle(state_handle, &goal_effort_[i]);
    jnt_eff_interface_.registerHandle(eff_handle);

  }
  registerInterface(&jnt_state_interface_);
  if (control_mode_ == PositionControl)
  {
    registerInterface(&jnt_pos_interface_);
  } else if (control_mode_ == VelocityControl)
  {
    registerInterface(&jnt_vel_interface_);
  } else if (control_mode_ == EffortControl)
  {
    registerInterface(&jnt_eff_interface_);
  }

  if (nh.param("auto_torque", false)) {
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
