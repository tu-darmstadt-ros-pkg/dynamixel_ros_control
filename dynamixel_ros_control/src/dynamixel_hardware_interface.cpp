#include "dynamixel_ros_control/common.hpp"
#include "dynamixel_ros_control/log.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <dynamixel_ros_control/dynamixel_hardware_interface.hpp>

namespace {

std::unordered_map<std::string, std::string> loadInterfaceRegisterTranslationMap(const YAML::Node& node)
{
  if (!node.IsSequence()) {
    DXL_LOG_ERROR("Conversion list is not a sequence.");
    return {};
  }
  std::unordered_map<std::string, std::string> conversion_map;
  for (const auto& entry : node) {
    if (!entry.IsMap()) {
      DXL_LOG_ERROR("Conversion entry is not a map.");
      return {};
    }
    const std::string& interface_name = entry["interface_name"].as<std::string>();
    const std::string& register_name = entry["register_name"].as<std::string>();
    conversion_map.emplace(interface_name, register_name);
  }
  return conversion_map;
}

bool loadInterfaceRegisterNameTranslation(std::unordered_map<std::string, std::string>& state_interface_to_register,
                                          std::unordered_map<std::string, std::string>& command_interface_to_register)
{
  std::string package_path_ = ament_index_cpp::get_package_share_directory("dynamixel_ros_control");
  const std::string path = package_path_ + "/devices/interface_to_register_names.yaml";
  YAML::Node config;
  try {
    config = YAML::LoadFile(path);
  }
  catch (YAML::BadFile&) {
    DXL_LOG_ERROR("Failed to read interface to register name translation at '" << path << "'. Does the file exist?");
    return false;
  }
  if (!config.IsMap()) {
    DXL_LOG_ERROR("interface_to_register_names.yaml is not a map (wrong format).");
    return false;
  }

  state_interface_to_register = loadInterfaceRegisterTranslationMap(config["state_interfaces"]);
  command_interface_to_register = loadInterfaceRegisterTranslationMap(config["command_interfaces"]);
  return true;
}

}  // namespace

namespace dynamixel_ros_control {

hardware_interface::CallbackReturn
DynamixelHardwareInterface::on_init(const hardware_interface::HardwareInfo& hardware_info)
{
  // Load hardware configuration
  const auto result = SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS) {
    return result;
  }

  // Load parameters
  std::string port_name;
  if (!getParameter<std::string>(info_.hardware_parameters, "port_name", port_name)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  int baud_rate;
  if (!getParameter<int>(info_.hardware_parameters, "baud_rate", baud_rate)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  getParameter(info_.hardware_parameters, "debug", debug_, false);
  if (debug_) {
    rclcpp::get_logger(DXL_LOGGER_NAME).set_level(rclcpp::Logger::Level::Debug);
  }
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_init");
  getParameter(info_.hardware_parameters, "torque_on_startup", torque_on_startup_, false);
  getParameter(info_.hardware_parameters, "torque_off_on_shutdown", torque_off_on_shutdown_, false);
  getParameter(info_.hardware_parameters, "reboot_on_hardware_error", reboot_on_hardware_error_, false);

  // Initialize driver
  if (!driver_.init(port_name, baud_rate)) {
    DXL_LOG_ERROR("Failed to initialize driver");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Interface to register translation
  std::unordered_map<std::string, std::string> state_interface_to_register;
  std::unordered_map<std::string, std::string> command_interface_to_register;
  if (!loadInterfaceRegisterNameTranslation(state_interface_to_register, command_interface_to_register)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Load joints
  joints_.reserve(info_.joints.size());
  for (const auto& joint_info : info_.joints) {
    Joint joint;
    if (!joint.loadConfiguration(driver_, joint_info, state_interface_to_register, command_interface_to_register)) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    std::stringstream ss;
    ss << "Loaded dynamixel:" << std::endl;
    ss << "-- name: " << joint.name << std::endl;
    ss << "-- id: " << static_cast<int>(joint.dynamixel->getId()) << std::endl;
    ss << "-- command interfaces: " << iterableToString(joint.getAvailableCommandInterfaces()) << std::endl;
    ss << "-- state interfaces: " << iterableToString(joint.getAvailableStateInterfaces()) << std::endl;
    ss << "-- mounting_offset: " << joint.mounting_offset << std::endl;
    ss << "-- offset: " << joint.offset << std::endl;
    ss << "-- initial values: " << mapToString(joint.dynamixel->getInitialRegisterValues()) << std::endl;
    DXL_LOG_DEBUG(ss.str());
    joints_.emplace(joint.name, std::move(joint));
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DynamixelHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_configure from " << previous_state.label());
  first_read_successful_ = false;
  if (!driver_.connect()) {
    return hardware_interface::CallbackReturn::FAILURE;
  }

  for (auto& [name, joint] : joints_) {
    if (!joint.connect()) {
      return hardware_interface::CallbackReturn::FAILURE;
    }
    joint.reset();
  }

  // const bool torque = !joints_.empty() && joints_.begin()->second.torque;
  // if (torque) {
  //   setTorque(false, true);
  // }

  // Set up sync read / write managers
  if (!setUpStatusReadManager() || !setUpStateReadManager() || !setUpTorqueWriteManager() || !setUpControlWriteManager()) {
    return hardware_interface::CallbackReturn::FAILURE;
  }

  // if (torque) {
  //   setTorque(true);
  // }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_cleanup from " << previous_state.label());
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_activate from " << previous_state.label());
  if (torque_on_startup_) {
    if (!setTorque(true)) {
      return CallbackReturn::ERROR;
    }
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DynamixelHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_deactivate from " << previous_state.label());
  if (torque_off_on_shutdown_) {
    if (!setTorque(false)) {
      return CallbackReturn::ERROR;
    }
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr> DynamixelHardwareInterface::on_export_state_interfaces()
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_export_state_interfaces");
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;

  // Read all requested fields from all motors
  std::set<std::string> configured_state_interface_names;
  for (const auto& [name, joint] : joints_) {
    configured_state_interface_names.insert(joint.getAvailableStateInterfaces().begin(),
                                            joint.getAvailableStateInterfaces().end());
  }
  // Create the state interfaces
  for (auto& [name, joint] : joints_) {
    joint.current_state.reserve(configured_state_interface_names.size());
    for (const auto& interface_name : configured_state_interface_names) {
      joint.current_state[interface_name] = 0.0;
      const auto state_interface = std::make_shared<hardware_interface::StateInterface>(
          joint.name, interface_name, &joint.current_state[interface_name]);
      state_interfaces.emplace_back(state_interface);
    }
  }
  DXL_LOG_DEBUG("State interfaces: " << iterableToString(configured_state_interface_names));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr> DynamixelHardwareInterface::on_export_command_interfaces()
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_export_command_interfaces");
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;

  // Create the state interfaces
  for (auto& [name, joint] : joints_) {
    joint.goal_state.reserve(joint.getAvailableCommandInterfaces().size());
    for (const auto& interface_name : joint.getAvailableCommandInterfaces()) {
      joint.goal_state[interface_name] = 0.0;
      const auto command_interface = std::make_shared<hardware_interface::CommandInterface>(
          joint.name, interface_name, &joint.goal_state[interface_name]);
      command_interfaces.emplace_back(command_interface);
    }
  }
  return command_interfaces;
}

hardware_interface::return_type
DynamixelHardwareInterface::perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                        const std::vector<std::string>& stop_interfaces)
{
  // Set up write manager
  DXL_LOG_DEBUG("DynamixelHardwareInterface::perform_command_mode_switch");
  DXL_LOG_DEBUG("start_interfaces: " << iterableToString(start_interfaces));
  DXL_LOG_DEBUG("stop_interfaces: " << iterableToString(stop_interfaces));

  if (!first_read_successful_) {
    DXL_LOG_ERROR("No successful read() before a controller is loaded.");
    return hardware_interface::return_type::ERROR;
  }

  // Start interfaces
  if (!processCommandInterfaceUpdates(start_interfaces, false)) {
    return hardware_interface::return_type::ERROR;
  }
  if (!processCommandInterfaceUpdates(stop_interfaces, true)) {
    return hardware_interface::return_type::ERROR;
  }

  // write control mode
  for (auto& [name, joint] : joints_) {
    if (!joint.updateControlMode()) {
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn DynamixelHardwareInterface::on_error(const rclcpp_lifecycle::State& previous_state)
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_error from " << previous_state.label());
  if (isHardwareOk()) {
    return CallbackReturn::SUCCESS;
  }
  // Hardware reports error
  if (!reboot_on_hardware_error_) {
    return CallbackReturn::FAILURE;
  }

  get_clock()->sleep_for(rclcpp::Duration(3.0, 0));
  if (!reboot()) {
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DynamixelHardwareInterface::read(const rclcpp::Time& time,
                                                                 const rclcpp::Duration& /*period*/)
{
  // Check for hardware errors
  status_read_manager_.read();
  if (!isHardwareOk()) {
    return hardware_interface::return_type::ERROR;
  }

  read_manager_.read();
  if (!read_manager_.isOk()) {
    DXL_LOG_ERROR("Read manager lost connection");
    return hardware_interface::return_type::ERROR;
  }
  if (!first_read_successful_) {
    first_read_successful_ = true;
    for (auto& [name, joint]: joints_) {
      joint.resetGoalState();
    }
  }

  last_successful_read_time_ = time;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                                  const rclcpp::Duration& /*period*/)
{
  if (!first_read_successful_) {
    // DXL_LOG_ERROR("Write called without successful read. This should not happen.");
    return hardware_interface::return_type::OK;
  }

  control_write_manager_.write();

  if (!control_write_manager_.isOk()) {
    DXL_LOG_ERROR("Write manager lost connection");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

bool DynamixelHardwareInterface::processCommandInterfaceUpdates(const std::vector<std::string>& interface_updates,
                                                                bool stopping)
{
  for (const auto& full_interface_name : interface_updates) {
    std::string joint_name;
    std::string interface_name;
    if (!splitFullInterfaceName(full_interface_name, joint_name, interface_name)) {
      DXL_LOG_ERROR("Invalid interface name: " << full_interface_name);
      return false;
    }

    try {
      Joint& joint = joints_.at(joint_name);
      if (!stopping) {
        if (!joint.addActiveCommandInterface(interface_name)) {
          return false;
        }
      } else {
        if (!joint.removeActiveCommandInterface(interface_name)) {
          return false;
        }
      }
    }
    catch (std::out_of_range&) {
      DXL_LOG_ERROR("Unknown joint name: " << joint_name);
      return false;
    }
  }
  return true;
}

bool DynamixelHardwareInterface::setUpStateReadManager()
{
  read_manager_ = SyncReadManager();
  std::unordered_map<std::string, DxlValueMappingList> register_dynamixel_mappings;
  for (auto& [name, joint] : joints_) {
    read_manager_.addDynamixel(joint.dynamixel.get());
    for (auto& [interface_name, interface_value] : joint.current_state) {
      std::string register_name = joint.stateInterfaceToRegisterName(interface_name);
      register_dynamixel_mappings[register_name].push_back(
          std::make_pair<Dynamixel*, DxlValue>(joint.dynamixel.get(), DxlValue(&interface_value)));
    }
  }

  for (const auto& [register_name, dynamixel_mapping] : register_dynamixel_mappings) {
    read_manager_.addRegister(register_name, dynamixel_mapping);
  }

  return read_manager_.init(driver_);
}

bool DynamixelHardwareInterface::setUpStatusReadManager()
{
  status_read_manager_ = SyncReadManager();
  DxlValueMappingList status_mapping;
  for (auto& [name, joint] : joints_) {
    status_read_manager_.addDynamixel(joint.dynamixel.get());
    status_mapping.push_back(
        std::make_pair<Dynamixel*, DxlValue>(joint.dynamixel.get(), DxlValue(&joint.dynamixel->hardware_error_status)));
  }

  status_read_manager_.addRegister(DXL_REGISTER_HARDWARE_ERROR, status_mapping);
  return status_read_manager_.init(driver_);
}
bool DynamixelHardwareInterface::setUpTorqueWriteManager()
{
  torque_write_manager_ = SyncWriteManager();
  for (auto& [name, joint] : joints_) {
    torque_write_manager_.addRegister(*joint.dynamixel, DXL_REGISTER_CMD_TORQUE, joint.torque);
  }

  return torque_write_manager_.init(driver_);
}

bool DynamixelHardwareInterface::setUpControlWriteManager()
{
  control_write_manager_ = SyncWriteManager();
  for (auto& [name, joint] : joints_) {
    // DXL_LOG_DEBUG("Active command interfaces for joint '"
    //               << joint.name << "': " << iterableToString(joint.getActiveCommandInterfaces()));
    if (joint.getAvailableCommandInterfaces().empty()) {
      // Nothing to register
      continue;
    }
    for (const auto& interface_name : joint.getAvailableCommandInterfaces()) {
      const std::string register_name = joint.commandInterfaceToRegisterName(interface_name);
      control_write_manager_.addRegister(*joint.dynamixel, register_name, joint.goal_state.at(interface_name));
    }
  }

  return control_write_manager_.init(driver_);
}

bool DynamixelHardwareInterface::isHardwareOk() const
{
  bool ok = true;
  for (auto& [name, joint] : joints_) {
    if (joint.dynamixel->hardware_error_status != OK) {
      DXL_LOG_ERROR("Joint '" << name
                              << "' reports hardware error: " << joint.dynamixel->getHardwareErrorStatusString());
      ok = false;
    }
  }
  return ok;
}

bool DynamixelHardwareInterface::reboot() const
{
  for (auto& [name, joint] : joints_) {
    if (joint.dynamixel->hardware_error_status != OK && !joint.dynamixel->reboot()) {
      return false;
    }
  }
  return true;
}

bool DynamixelHardwareInterface::setTorque(const bool enabled, const bool direct_write)
{
  DXL_LOG_INFO((enabled ? "Enabling" : "Disabling") << " motor torque.");
  for (auto& [name, joint] : joints_) {
    joint.torque = enabled;
    if (direct_write && !joint.dynamixel->writeRegister(DXL_REGISTER_CMD_TORQUE, joint.torque)) {
      return false;
    }
  }

  if (!direct_write && !torque_write_manager_.write()) {
    DXL_LOG_ERROR("Setting torque failed!");
    return false;
  }

  return true;
}

}  // namespace dynamixel_ros_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_ros_control::DynamixelHardwareInterface, hardware_interface::SystemInterface)
