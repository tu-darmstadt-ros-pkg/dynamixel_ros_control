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

  // Register sync reads/writes
  for (Joint& joint: joints_) {
    // Register writes
    control_write_manager_.addRegister(joint.dynamixel, "torque_enable", joint.goal_state.torque); // TODO bool to uint32_t?
    if (joint.control_mode == POSITION) {
      control_write_manager_.addRegister(joint.dynamixel, "goal_position", joint.goal_state.position); // TODO read register name from config
    } else if (joint.control_mode == VELOCITY) {
      control_write_manager_.addRegister(joint.dynamixel, "goal_velocity", joint.goal_state.velocity);
    } else if (joint.control_mode == CURRENT) {
      control_write_manager_.addRegister(joint.dynamixel, "goal_current", joint.goal_state.effort); // TODO effort = current?
    }

    // Register reads
    if (joint.read_position) {
      read_manager_.addRegister(joint.dynamixel, "present_position", joint.current_state.position);
    }
    if (joint.read_velocity) {
      read_manager_.addRegister(joint.dynamixel, "present_velocity", joint.current_state.velocity);
    }
    if (joint.read_effort) {
      read_manager_.addRegister(joint.dynamixel, "present_current", joint.current_state.effort);
    }
  }

  if (torque_on_startup_) {
    setTorque(true);
  }

  // Initialize subscribers and publishers

  return true;
}

void DynamixelHardwareInterface::read()
{
  if (!read_manager_.read()) {
    ROS_ERROR_STREAM("Sync read failed!");
  }
}

void DynamixelHardwareInterface::write()
{
  if (!control_write_manager_.write()) {
    ROS_ERROR_STREAM("Sync write failed!");
  }
}


void DynamixelHardwareInterface::setTorque(bool enabled)
{
  for (Joint& joint: joints_) {
    joint.goal_state.torque = enabled;
  }
  if (!torque_write_manager_.write()) {
    ROS_ERROR_STREAM("Setting torque failed!");
  }
}

void DynamixelHardwareInterface::setTorque(std_msgs::BoolConstPtr enabled)
{
  setTorque(enabled->data);
}

}
