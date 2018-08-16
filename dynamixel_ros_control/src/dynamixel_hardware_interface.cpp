#include <dynamixel_ros_control/dynamixel_hardware_interface.h>

#include <dynamixel_ros_control/common.h>

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

bool DynamixelHardwareInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
  // Load Parameters
  pnh_.param<bool>("debug", debug_, false);
  if (debug_) {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
      ros::console::notifyLoggerLevelsChanged();
    }
  }
  pnh_.param("torque_on_startup", torque_on_startup_, false);
  pnh_.param("torque_off_on_shutdown", torque_off_on_shutdown_, false);

  ros::NodeHandle dxl_nh(pnh_, "dynamixels");
  if (!driver_.init(dxl_nh) || !loadDynamixels(dxl_nh)) {
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
  DxlValueMappingList<double> position_mapping;
  DxlValueMappingList<double> velocity_mapping;
  DxlValueMappingList<double> effort_mapping;
  for (Joint& joint: joints_) {
    // Register writes
    control_write_manager_.addRegister(joint.dynamixel, "torque_enable", joint.goal_state.torque);
    if (joint.control_mode == POSITION) {
      control_write_manager_.addRegister(joint.dynamixel, "goal_position", joint.goal_state.position); // TODO read register name from config
    } else if (joint.control_mode == VELOCITY) {
      control_write_manager_.addRegister(joint.dynamixel, "goal_velocity", joint.goal_state.velocity);
    } else if (joint.control_mode == CURRENT) {
      control_write_manager_.addRegister(joint.dynamixel, "goal_current", joint.goal_state.effort); // TODO effort = current?
    }

    // Register reads
    if (joint.read_position) {
      position_mapping.push_back(std::make_pair<Dynamixel*, double*>(&joint.dynamixel, &joint.current_state.position));
//      read_manager_.addRegister(joint.dynamixel, "present_position", joint.current_state.position);
    }
    if (joint.read_velocity) {
      velocity_mapping.push_back(std::make_pair<Dynamixel*, double*>(&joint.dynamixel, &joint.current_state.velocity));
//      read_manager_.addRegister(joint.dynamixel, "present_velocity", joint.current_state.velocity);
    }
    if (joint.read_effort) {
      effort_mapping.push_back(std::make_pair<Dynamixel*, double*>(&joint.dynamixel, &joint.current_state.effort));
//      read_manager_.addRegister(joint.dynamixel, "present_current", joint.current_state.effort);
    }
  }

  read_manager_.addRegister("present_position", position_mapping);
  read_manager_.addRegister("present_velocity", velocity_mapping);
  read_manager_.addRegister("present_current", effort_mapping);

  // Initialize sync reads/writes
  control_write_manager_.init(driver_);
  read_manager_.init(driver_);

  if (torque_on_startup_) {
    setTorque(true);
  }

  // Initialize subscribers and publishers

  return true;
}

void DynamixelHardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
  if (!read_manager_.read()) {
    ROS_ERROR_STREAM("Sync read failed!");
  }
}

void DynamixelHardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
  if (!control_write_manager_.write()) {
    ROS_ERROR_STREAM("Sync write failed!");
  }
}

bool DynamixelHardwareInterface::loadDynamixels(const ros::NodeHandle& nh)
{
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
    if (id_int < 256) {
      id = static_cast<uint8_t>(id_int);
    } else {
      ROS_ERROR_STREAM("ID " << id_int << " exceeds 256.");
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
      dxl_nh.param("mounting_offset", joint.mounting_offset, 0.0);
      dxl_nh.param("offset", joint.offset, 0.0);
      if (!joint.dynamixel.loadControlTable(nh)) {
        return false;
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
