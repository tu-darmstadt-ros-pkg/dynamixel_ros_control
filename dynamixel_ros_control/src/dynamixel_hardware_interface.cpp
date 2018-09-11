#include <dynamixel_ros_control/dynamixel_hardware_interface.h>

#include <dynamixel_ros_control/common.h>

namespace dynamixel_ros_control {

DynamixelHardwareInterface::DynamixelHardwareInterface(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), first_cycle_(true), initialized_(false)
{

}

DynamixelHardwareInterface::~DynamixelHardwareInterface()
{
  if (torque_off_on_shutdown_)
  {
    std::cout << "Disabling torque on shutdown!" << std::endl;
    if (initialized_) setTorque(false);
  }
}

bool DynamixelHardwareInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
  first_cycle_ = true;
  // Load Parameters
  pnh_.param<bool>("debug", debug_, false);
  if (debug_) {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
      ros::console::notifyLoggerLevelsChanged();
    }
  }
  pnh_.param("torque_on_startup", torque_on_startup_, false);
  pnh_.param("torque_off_on_shutdown", torque_off_on_shutdown_, false);

  // Load dynamixels
  ros::NodeHandle dxl_nh(pnh_, "dynamixels");
  if (!driver_.init(dxl_nh) || !loadDynamixels(dxl_nh)) {
    return false;
  }

  // Register sync reads/writes
  pnh_.param("dynamixels/read_values/read_position", read_position_, true);
  pnh_.param("dynamixels/read_values/read_velocity", read_velocity_, false);
  pnh_.param("dynamixels/read_values/read_effort", read_effort_, false);

  DxlValueMappingList<double> position_mapping;
  DxlValueMappingList<double> velocity_mapping;
  DxlValueMappingList<double> effort_mapping;
  for (Joint& joint: joints_) {
    // Register writes
    torque_write_manager_.addRegister(joint.dynamixel, "torque_enable", joint.goal_state.torque);
    if (joint.getControlMode() == POSITION || joint.getControlMode() == EXTENDED_POSITION || joint.getControlMode() == CURRENT_BASED_POSITION) {
      control_write_manager_.addRegister(joint.dynamixel, "goal_position", joint.goal_state.position); // TODO read register name from config
    } else if (joint.getControlMode() == VELOCITY) {
      control_write_manager_.addRegister(joint.dynamixel, "goal_velocity", joint.goal_state.velocity);
    } else if (joint.getControlMode() == CURRENT) {
      control_write_manager_.addRegister(joint.dynamixel, "goal_torque", joint.goal_state.effort);
    }

    // Register reads
    read_manager_.addDynamixel(&joint.dynamixel);
    if (read_position_) {
      position_mapping.push_back(std::make_pair<Dynamixel*, double*>(&joint.dynamixel, &joint.current_state.position));
//      read_manager_.addRegister(joint.dynamixel, "present_position", joint.current_state.position);
    }
    if (read_velocity_) {
      velocity_mapping.push_back(std::make_pair<Dynamixel*, double*>(&joint.dynamixel, &joint.current_state.velocity));
//      read_manager_.addRegister(joint.dynamixel, "present_velocity", joint.current_state.velocity);
    }
    if (read_effort_) {
      effort_mapping.push_back(std::make_pair<Dynamixel*, double*>(&joint.dynamixel, &joint.current_state.effort));
//      read_manager_.addRegister(joint.dynamixel, "present_current", joint.current_state.effort);
    }
  }

  if (read_position_) {
    read_manager_.addRegister("present_position", position_mapping);
  }
  if (read_velocity_) {
    read_manager_.addRegister("present_velocity", velocity_mapping);
  }
  if (read_effort_) {
    read_manager_.addRegister("present_current", effort_mapping);
  }

  // Initialize sync reads/writes
  if (!torque_write_manager_.init(driver_)) {
    return false;
  }
  if (!control_write_manager_.init(driver_)) {
    return false;
  }
  if (!read_manager_.init(driver_)) {
    return false;
  }

  initialized_ = true;

  setTorque(false);

  // Write control mode
  bool write_control_mode;
  pnh_.param("write_control_mode", write_control_mode, true);
  if (write_control_mode) {
    writeControlMode();
  }

  // Write initial values
  writeInitialValues(dxl_nh);

  // Register interfaces
  for (Joint& joint: joints_)
  {
    hardware_interface::JointStateHandle state_handle(joint.name, &joint.current_state.position, &joint.current_state.velocity,
                                                      &joint.current_state.effort);
    jnt_state_interface_.registerHandle(state_handle);

    if (joint.getControlMode() == POSITION || joint.getControlMode() == EXTENDED_POSITION || joint.getControlMode() == CURRENT_BASED_POSITION) {
      hardware_interface::JointHandle pos_handle(state_handle, &joint.goal_state.position);
      jnt_pos_interface_.registerHandle(pos_handle);
    } else if (joint.getControlMode() == VELOCITY) {
      hardware_interface::JointHandle vel_handle(state_handle, &joint.goal_state.velocity);
      jnt_vel_interface_.registerHandle(vel_handle);
    } else if (joint.getControlMode() == CURRENT) {
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
    ROS_INFO_STREAM("Enabling torque on startup");
    setTorque(true);
  }

  return true;
}

void DynamixelHardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
  read_manager_.read();

  for (const Joint& j: joints_) {
    double goal_torque;
    j.dynamixel.readRegister("goal_torque", goal_torque);
    ROS_DEBUG_STREAM("[GOAL TORQUE] " << j.name << ": " << goal_torque);
  }
  if (first_cycle_) {
    first_cycle_ = false;
    for (Joint& joint: joints_) {
      joint.goal_state = joint.current_state;
    }
  }
}

void DynamixelHardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
  control_write_manager_.write();
}

bool DynamixelHardwareInterface::loadDynamixels(const ros::NodeHandle& nh)
{
  // Load default control mode for all joints
  std::string default_control_mode_str;
  nh.param<std::string>("control_mode", default_control_mode_str, "position_control");
  ControlMode default_control_mode = POSITION;
  try {
    default_control_mode = stringToControlMode(default_control_mode_str);
  } catch (const std::invalid_argument& e) {
    ROS_ERROR_STREAM(e.what() << std::endl << "Defaulting to position control.");
  }

  // Get dxl info
  XmlRpc::XmlRpcValue dxls;
  nh.getParam("device_info", dxls);
  ROS_ASSERT(dxls.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = dxls.begin(); it != dxls.end(); ++it)
  {
    std::string joint_name = static_cast<std::string>(it->first);
    ros::NodeHandle dxl_nh(nh, "device_info/" + joint_name);

    int id_int;
    if (!loadRequiredParameter(dxl_nh, "id", id_int)) {
      return false;
    }
    uint8_t id;
    if (id_int >= 0 && id_int < 256) {
      id = static_cast<uint8_t>(id_int);
    } else {
      ROS_ERROR_STREAM("ID " << id_int << " is not in the valid range [0;255]");
      continue;
    }

    int model_number_config;
    bool model_number_set = dxl_nh.param("model_number", model_number_config, -1);

    // Ping dynamixel to retrieve model number
    uint16_t model_number_ping;
    if (!driver_.ping(id, model_number_ping)) {
      ROS_ERROR_STREAM("Failed to ping motor '" << joint_name << "' with id " << id_int);
      return false;
    } else {
      // Ping successful, add to list
      if (model_number_set && model_number_config != model_number_ping) {
        ROS_WARN_STREAM("Model number in config [" << model_number_config
                        << "] does not match servo model number [" << model_number_ping << "] for joint '"
                        << joint_name << "', ID: " << id_int);
      }

      Joint joint(joint_name, id, model_number_ping, driver_);
      dxl_nh.param("mounting_offset", joint.mounting_offset, 0.0); // TODO unused
      dxl_nh.param("offset", joint.offset, 0.0);
      if (!joint.dynamixel.loadControlTable()) {
        return false;
      }

      // Local control mode can override default control mode
      std::string control_mode_str;
      if (dxl_nh.getParam("control_mode", control_mode_str)) {
        try {
          joint.setControlMode(stringToControlMode(control_mode_str));
        } catch(const std::invalid_argument& e) {
          ROS_ERROR_STREAM(e.what() << std::endl << "Using default control mode.");
          joint.setControlMode(default_control_mode);
        }
      } else {
        joint.setControlMode(default_control_mode);
      }

      std::stringstream ss;
      ss << "Loaded dynamixel:" << std::endl;
      ss << "-- name: " << joint.name << std::endl;
      ss << "-- id: " << static_cast<int>(joint.dynamixel.getId()) << std::endl;
      ss << "-- model number: " << joint.dynamixel.getModelNumber() << std::endl;
      ROS_DEBUG_STREAM(ss.str());

      joints_.push_back(std::move(joint));
    }
  }
  return true;
}

void DynamixelHardwareInterface::writeInitialValues(const ros::NodeHandle& nh)
{
  XmlRpc::XmlRpcValue joints;
  if (!nh.getParam("write_registers", joints)) {
    return;
  }
  ROS_INFO_STREAM("Writing initial values:");
  ROS_ASSERT(joints.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = joints.begin(); it != joints.end(); ++it)
  {
    std::string joint_name = static_cast<std::string>(it->first);
    Joint* joint = getJointByName(joint_name);
    if (!joint) {
      ROS_ERROR_STREAM("Unknown joint '" << joint_name << "'.");
      continue;
    }
    ROS_INFO_STREAM(joint_name << ":");

    XmlRpc::XmlRpcValue registers;
    nh.getParam("write_registers/" + joint_name, registers);
    ROS_ASSERT(registers.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for(XmlRpc::XmlRpcValue::ValueStruct::iterator it = registers.begin(); it != registers.end(); ++it)
    {
      std::string register_name = static_cast<std::string>(it->first);
      if (it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        double value = static_cast<double>(it->second);
        joint->dynamixel.writeRegister(register_name, value);
        ROS_INFO_STREAM("--- " << register_name << ": " << value);
      } else if (it->second.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
        bool value = static_cast<bool>(it->second);
        joint->dynamixel.writeRegister(register_name, value);
        ROS_INFO_STREAM("--- " << register_name << ": " << value);
      }
    }
  }
}

void DynamixelHardwareInterface::writeControlMode()
{
  for (const Joint& joint: joints_) {
    if (!joint.dynamixel.writeControlMode(joint.getControlMode())) {
      ROS_ERROR_STREAM("Failed to set control mode for joint '" << joint.name << "'.");
    }
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

Joint* DynamixelHardwareInterface::getJointByName(std::string name)
{
  for (Joint& joint: joints_) {
    if (joint.name == name) {
      return &joint;
    }
  }
  return nullptr;
}

}
