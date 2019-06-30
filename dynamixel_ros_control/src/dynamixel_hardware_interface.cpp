#include <dynamixel_ros_control/dynamixel_hardware_interface.h>

#include <dynamixel_ros_control/common.h>

namespace dynamixel_ros_control {

DynamixelHardwareInterface::DynamixelHardwareInterface()
  : connected_(false), first_cycle_(true), estop_(false), reset_required_(false), time_sync_joint_idx_(0)
{}

DynamixelHardwareInterface::~DynamixelHardwareInterface()
{
  if (torque_off_on_shutdown_)
  {
    std::cout << "Disabling torque on shutdown!" << std::endl;
    if (connected_) setTorque(false);
  }
}

bool DynamixelHardwareInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
  nh_ = root_nh;
  pnh_ = robot_hw_nh;
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
  pnh_.param("reset_controllers_after_estop", reset_controllers_after_estop_, true);

  // Load dynamixels
  ros::NodeHandle dxl_nh(pnh_, "dynamixels");
  if (!driver_.init(dxl_nh) || !loadDynamixels(dxl_nh)) {
    return false;
  }

  // Register sync reads/writes
  pnh_.param("dynamixels/read_values/read_position", read_position_, true);
  pnh_.param("dynamixels/read_values/read_velocity", read_velocity_, false);
  pnh_.param("dynamixels/read_values/read_effort", read_effort_, false);

  // Register interfaces
  for (Joint& joint: joints_) {
    hardware_interface::JointStateHandle state_handle(joint.name, &joint.current_state.position, &joint.current_state.velocity,
                                                      &joint.current_state.effort);
    jnt_state_interface_.registerHandle(state_handle);

    if (joint.isPositionControlled()) {
      hardware_interface::JointHandle pos_handle(state_handle, &joint.goal_state.position);
      jnt_pos_interface_.registerHandle(pos_handle);
    } else if (joint.isVelocityControlled()) {
      hardware_interface::JointHandle vel_handle(state_handle, &joint.goal_state.velocity);
      jnt_vel_interface_.registerHandle(vel_handle);
    } else if (joint.isEffortControlled()) {
      hardware_interface::JointHandle eff_handle(state_handle, &joint.goal_state.effort);
      jnt_eff_interface_.registerHandle(eff_handle);
    }
  }
  registerInterface(&jnt_state_interface_);
  if (!jnt_pos_interface_.getNames().empty()) {
    registerInterface(&jnt_pos_interface_);
  }
  if (!jnt_vel_interface_.getNames().empty()) {
    registerInterface(&jnt_vel_interface_);
  }
  if (!jnt_eff_interface_.getNames().empty()) {
    registerInterface(&jnt_eff_interface_);
  }

  // Initialize subscribers
  estop_sub_ = pnh_.subscribe("estop", 100, &DynamixelHardwareInterface::estopCb, this);
  set_torque_sub_ = pnh_.subscribe("set_torque", 100, &DynamixelHardwareInterface::setTorque, this);
  reboot_server_ = pnh_.advertiseService("reboot_if_error_state", &DynamixelHardwareInterface::rebootCb, this);

  // Try to connect
  connect();

  return true;
}

bool DynamixelHardwareInterface::connect()
{
  last_connect_try_ = ros::Time::now();
  // Connect port
  if (!driver_.connect()) {
    return false;
  }

  // Ping dynamixels and load control table
  for (Joint& j: joints_) {
    if (!j.dynamixel.loadControlTable()) {
      return false;
    }
  }

  // Time sync
  setUpTimeSync();

  // Disable torque for indirect address remapping
  for (const Joint& joint: joints_) {
    if (!joint.dynamixel.writeRegister("torque_enable", false)) {
      return false;
    }
  }

  // Initialize sync reads/writes
  DxlValueMappingList position_mapping;
  std::vector<double> position_offsets;
  DxlValueMappingList velocity_mapping;
  DxlValueMappingList effort_mapping;
  DxlValueMappingList clock_mapping;
  DxlValueMappingList status_mapping;

  control_write_manager_ = SyncWriteManager();
  torque_write_manager_ = SyncWriteManager();
  read_manager_ = SyncReadManager();
  status_read_manager_ = SyncReadManager();

  for (Joint& joint: joints_) {
    double position_offset = joint.offset + joint.mounting_offset;
    // Register writes
    torque_write_manager_.addRegister(joint.dynamixel, "torque_enable", joint.goal_state.torque);
    if (joint.isPositionControlled()) {
      control_write_manager_.addRegister(joint.dynamixel, "goal_position", joint.goal_state.position, position_offset); // TODO read register name from config
    } else if (joint.isVelocityControlled()) {
      control_write_manager_.addRegister(joint.dynamixel, "goal_velocity", joint.goal_state.velocity);
    } else if (joint.isEffortControlled()) {
      control_write_manager_.addRegister(joint.dynamixel, "goal_torque", joint.goal_state.effort);
    }

    // Register reads
    read_manager_.addDynamixel(&joint.dynamixel);
    if (time_sync_available_) {
      clock_mapping.push_back(std::make_pair<Dynamixel*, DxlValue>(&joint.dynamixel, DxlValue(&joint.dynamixel.realtime_tick_ms_)));
    }
    if (read_position_) {
      position_mapping.push_back(std::make_pair<Dynamixel*, DxlValue>(&joint.dynamixel, DxlValue(&joint.current_state.position)));
      position_offsets.push_back(-position_offset); // Subtract offset on read
//      read_manager_.addRegister(joint.dynamixel, "present_position", joint.current_state.position);
    }
    if (read_velocity_) {
      velocity_mapping.push_back(std::make_pair<Dynamixel*, DxlValue>(&joint.dynamixel, DxlValue(&joint.current_state.velocity)));
//      read_manager_.addRegister(joint.dynamixel, "present_velocity", joint.current_state.velocity);
    }
    if (read_effort_) {
      effort_mapping.push_back(std::make_pair<Dynamixel*, DxlValue>(&joint.dynamixel, DxlValue(&joint.current_state.effort)));
//      read_manager_.addRegister(joint.dynamixel, "present_current", joint.current_state.effort);
    }

    status_read_manager_.addDynamixel(&joint.dynamixel);
    status_mapping.push_back(std::make_pair<Dynamixel*, DxlValue>(&joint.dynamixel, DxlValue(&joint.dynamixel.hardware_error_status)));
  }

  if (time_sync_available_) {
    read_manager_.addRegister("realtime_tick", clock_mapping);
  }
  status_read_manager_.addRegister("hardware_error_status", status_mapping);

  if (read_position_) {
    read_manager_.addRegister("present_position", position_mapping, position_offsets);
  }
  if (read_velocity_) {
    read_manager_.addRegister("present_velocity", velocity_mapping);
  }
  if (read_effort_) {
    read_manager_.addRegister("present_current", effort_mapping);
  }

  if (!torque_write_manager_.init(driver_)) {
    return false;
  }
  if (!control_write_manager_.init(driver_)) {
    return false;
  }
  if (!read_manager_.init(driver_)) {
    return false;
  }
  if (!status_read_manager_.init(driver_)) {
    return false;
  }

  // Write control mode
  bool write_control_mode;
  pnh_.param("write_control_mode", write_control_mode, true);
  if (write_control_mode) {
    writeControlMode();
  }

  // Write initial values
  ros::NodeHandle dxl_nh(pnh_, "dynamixels");
  writeInitialValues(dxl_nh);

  if (torque_on_startup_) {
    ROS_INFO_STREAM("Enabling torque on startup");
    setTorque(true);
  }

  reset_required_ = true;

  connected_ = true;
  first_cycle_ = true;
  return true;
}

void DynamixelHardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
  last_read_time_ = time;
  if (!connected_ && last_connect_try_ + ros::Duration(1) < ros::Time::now()) {
    if (!connect()) {
      ROS_WARN_STREAM("Failed to connect. Retrying in 1s..");
    }
  }
  if (connected_) {
    if (read_manager_.isOk()) {
      read_manager_.read();
      if (time_sync_available_) {
        joints_[time_sync_joint_idx_].dynamixel.translateTime(time);
      }
      if (first_cycle_) {
        first_cycle_ = false;
        for (Joint& joint: joints_) {
          joint.goal_state.position = joint.current_state.position;
        }
      }
    } else {
      ROS_ERROR_STREAM("Read manager lost connection");
      connected_ = false;
    }

  }
}

void DynamixelHardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
  if (connected_) {
    if (control_write_manager_.isOk()) {
      if (estop_) {
        for (Joint& joint: joints_) {
          joint.goal_state.position = joint.estop_position;
          joint.goal_state.velocity = 0;
          joint.goal_state.effort = 0;
        }
      }
      control_write_manager_.write();
    } else {
      ROS_ERROR_STREAM("Write manager lost connection");
      connected_ = false;
    }
  }
}

ros::Time DynamixelHardwareInterface::getLastReadTime() const
{
  if (time_sync_available_) {
    return joints_[time_sync_joint_idx_].dynamixel.getStamp();
  } else {
    return last_read_time_;
  }
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

    Joint joint(joint_name, driver_);
    joint.initFromNh(dxl_nh);

    // Local control mode can override default control mode
    std::string control_mode_str;
    if (nh_.getParam("control_mode", control_mode_str)) {
      try {
        joint.setControlMode(stringToControlMode(control_mode_str));
      } catch(const std::invalid_argument& e) {
        ROS_ERROR_STREAM(e.what() << std::endl << "Using default control mode.");
        joint.setControlMode(default_control_mode);
      }
    } else {
      joint.setControlMode(default_control_mode);
    }

    joint.initDxl();

    std::stringstream ss;
    ss << "Loaded dynamixel:" << std::endl;
    ss << "-- name: " << joint.name << std::endl;
    ss << "-- id: " << static_cast<int>(joint.dynamixel.getId()) << std::endl;
//    ss << "-- model number: " << joint.dynamixel.getModelNumber() << std::endl;
    ss << "-- mounting_offset: " << joint.mounting_offset << std::endl;
    ss << "-- offset: " << joint.offset << std::endl;
    ROS_DEBUG_STREAM(ss.str());

    joints_.push_back(std::move(joint));
  }
  return true;
}

void DynamixelHardwareInterface::setUpTimeSync()
{
  time_sync_available_ = true;
  for (const Joint& j: joints_) {
    if (!j.dynamixel.registerAvailable("realtime_tick")) {
      time_sync_available_ = false;
      return;
    }
  }
  if (time_sync_available_) {
    std::string time_sync_joint_name;
    if (pnh_.getParam("time_sync_joint", time_sync_joint_name)) {
      bool found = false;
      for (unsigned int i = 0; i < joints_.size(); i++) {
        if (joints_[i].name == time_sync_joint_name) {
          time_sync_joint_idx_ = i;
          found = true;
          break;
        }
      }
      // If no matching joint is found, the default of '0' is used.
      if (!found) {
        ROS_ERROR_STREAM("No joint with the name '" << time_sync_joint_name << "' was found for time synchronization.");
        time_sync_joint_idx_ = 0;
      }
    } else {
      time_sync_joint_idx_ = 0;
    }
    joints_[time_sync_joint_idx_].dynamixel.addTimeTranslator(ros::NodeHandle(pnh_, "time_translator"));
  }
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

void DynamixelHardwareInterface::setTorque(const std_msgs::BoolConstPtr& enabled)
{
  setTorque(enabled->data);
}

bool DynamixelHardwareInterface::rebootCb(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response)
{
  // Read status
  status_read_manager_.read();
  // Check for abnormalities
  for (Joint& j: joints_) {
    if (j.dynamixel.hardware_error_status != HardwareErrorStatus::OK) {
      ROS_WARN_STREAM("Joint " << j.name << " has shutdown status: " << j.dynamixel.getHardwareErrorStatusString());
      j.dynamixel.reboot();
    }
  }
  // Wait for motors to come back
  ros::Duration(0.5).sleep();
  // write torque
  if (!torque_write_manager_.write()) {
    return false;
  }
  // reset controllers
  reset_required_ = true;
  return true;
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

void DynamixelHardwareInterface::estopCb(const std_msgs::BoolConstPtr& bool_ptr)
{
  if (estop_ != bool_ptr->data) {
    estop_ = bool_ptr->data;
    ROS_WARN_STREAM("E-Stop has been " << (estop_ ? "activated" : "deactivated"));

    // Reset controllers if e-stop has been turned off
    if (!estop_ && reset_controllers_after_estop_) {
      reset_required_ = true;
    }

    // Save current position
    if (estop_) {
      for (Joint& joint: joints_) {
        joint.estop_position = joint.current_state.position;
      }
    }
  }
}

bool DynamixelHardwareInterface::resetRequired() const
{
  return reset_required_;
}

void DynamixelHardwareInterface::clearResetRequired()
{
  reset_required_ = false;
}

}
