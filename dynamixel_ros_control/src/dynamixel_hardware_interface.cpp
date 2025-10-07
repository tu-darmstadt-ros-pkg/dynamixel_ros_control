#include "dynamixel_ros_control/common.hpp"
#include "dynamixel_ros_control/log.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <dynamixel_ros_control/dynamixel_hardware_interface.hpp>

#include <transmission_interface/simple_transmission_loader.hpp>
#include <transmission_interface/transmission.hpp>
#include <transmission_interface/transmission_interface_exception.hpp>
#include <hector_transmission_interface/adjustable_offset_transmission_loader.hpp>

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
                                          std::unordered_map<std::string, std::string>& command_interface_to_register,
                                          std::unordered_map<std::string, std::string>& interface_to_register_limits)
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
  interface_to_register_limits = loadInterfaceRegisterTranslationMap(config["limits"]);
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
  std::unordered_map<std::string, std::string> interface_to_register_limits;  // e.g. velocity and current limits
  if (!loadInterfaceRegisterNameTranslation(state_interface_to_register, command_interface_to_register,
                                            interface_to_register_limits)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Load joints
  joints_.reserve(info_.joints.size());
  for (const auto& joint_info : info_.joints) {
    Joint joint;
    if (!joint.loadConfiguration(driver_, joint_info, state_interface_to_register, command_interface_to_register,
                                 interface_to_register_limits)) {
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

  // create and spinn a ros2 node in a separate thread
  // (making sure it gets a separate name but the same namespace as the controller manager)
  auto tmp_node = rclcpp::Node::make_shared("dynamixel_ros_control_node");
  auto ns = std::string(tmp_node->get_namespace());
  node_ = std::make_shared<rclcpp::Node>(hardware_info.name, ns, rclcpp::NodeOptions().use_global_arguments(false));
  exe_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  exe_->add_node(node_);
  exe_thread_ = std::thread([this] { exe_->spin(); });

  // create a service to set torque
  set_torque_service_ = node_->create_service<std_srvs::srv::SetBool>(
      "~/set_torque", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                             const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        DXL_LOG_INFO("Request to set torque to " << (request->data ? "ON" : "OFF") << " received.");
        response->success = setTorque(request->data);
        response->message = response->success ? "Torque set successfully" : "Failed to set torque";
      });

  adjust_offset_service_ = node_->create_service<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets>(
      "~/adjust_transmission_offsets", std::bind(&DynamixelHardwareInterface::adjustTransmissionOffsetsCallback, this,
                                                 std::placeholders::_1, std::placeholders::_2));
  // setup controller orchestrator
  controller_orchestrator_ = std::make_shared<controller_orchestrator::ControllerOrchestrator>(node_);

  // set up e-stop subscription
  soft_e_stop_subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
      "~/soft_e_stop", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) { setEStop(msg->data); });
  // Transmissions
  if (!loadTransmissionConfiguration()) {
    return hardware_interface::CallbackReturn::ERROR;
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

  bool connection_successful = true;
  for (auto& [name, joint] : joints_) {
    if (!joint.connect()) {
      connection_successful = false;
    }
    joint.reset();
  }
  if (!connection_successful)
    return hardware_interface::CallbackReturn::FAILURE;

  // const bool torque = !joints_.empty() && joints_.begin()->second.torque;
  // if (torque) {
  //   setTorque(false, true);
  // }

  // Set up sync read / write managers
  if (!setUpStatusReadManager() || !setUpStateReadManager() || !setUpTorqueWriteManager() ||
      !setUpControlWriteManager() || !setUpCmdReadManager()) {
    return hardware_interface::CallbackReturn::FAILURE;
  }

  // if (torque) {
  //   setTorque(true);
  // }
  updateColorLED(hardware_interface::lifecycle_state_names::INACTIVE);

  return CallbackReturn::SUCCESS;
}

DynamixelHardwareInterface::~DynamixelHardwareInterface()
{
  try {
    if (exe_) {
      exe_->cancel();
      if (node_) {
        try {
          exe_->remove_node(node_);
        }
        catch (...) {
        }
      }
    }
    if (exe_thread_.joinable()) {
      exe_thread_.join();
    }
  }
  catch (...) {
  }
  exe_.reset();
  node_.reset();
}

hardware_interface::CallbackReturn DynamixelHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_cleanup from " << previous_state.label());
  if (exe_) {
    exe_->cancel();
  }
  if (exe_thread_.joinable()) {
    exe_thread_.join();
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_activate from " << previous_state.label());
  if (!setTorque(torque_on_startup_, true)) {
    DXL_LOG_ERROR("Failed to set torque on activation to " << (torque_on_startup_ ? "ON" : "OFF"));
    return hardware_interface::CallbackReturn::ERROR;
  }
  is_torqued_ = torque_on_startup_;  // TODO: check if successful
  if (!resetGoalStateAndVerify()) {
    return CallbackReturn::ERROR;
  }
  updateColorLED(hardware_interface::lifecycle_state_names::ACTIVE);
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DynamixelHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_deactivate from " << previous_state.label());
  if (!setTorque(!torque_off_on_shutdown_, true)) {
    DXL_LOG_ERROR("Failed to set torque on deactivation to " << (torque_off_on_shutdown_ ? "ON" : "OFF"));
    return CallbackReturn::ERROR;
  }
  updateColorLED(hardware_interface::lifecycle_state_names::INACTIVE);
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr> DynamixelHardwareInterface::on_export_state_interfaces()
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_export_state_interfaces");
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;

  // Read all configured state fields from all motors
  std::set<std::string> configured_state_interface_names;
  for (const auto& [name, joint] : joints_) {
    configured_state_interface_names.insert(joint.getAvailableStateInterfaces().begin(),
                                            joint.getAvailableStateInterfaces().end());
  }

  // Create the state interfaces
  for (auto& [name, joint] : joints_) {
    joint.joint_state.current.reserve(configured_state_interface_names.size());
    for (const auto& interface_name : configured_state_interface_names) {
      joint.joint_state.current[interface_name] = 0.0;
      const auto state_interface = std::make_shared<hardware_interface::StateInterface>(
          joint.name, interface_name, &joint.joint_state.current[interface_name]);
      state_interfaces.emplace_back(state_interface);
    }
  }
  DXL_LOG_DEBUG("State interfaces: " << iterableToString(configured_state_interface_names));

  // Create the transmission interface
  for (const auto& transmission_info : info_.transmissions) {
    std::vector<transmission_interface::JointHandle> joint_handles;
    std::vector<transmission_interface::ActuatorHandle> actuator_handles;
    const std::string& joint_name = transmission_info.joints.front().name;
    const std::string& actuator_name = transmission_info.actuators.front().name;
    Joint& joint = joints_.at(joint_name);  // This should exist
    for (const auto& interface_name : configured_state_interface_names) {
      transmission_interface::JointHandle joint_handle(transmission_info.joints.front().name, interface_name,
                                                       &joint.joint_state.current[interface_name]);
      joint_handles.push_back(joint_handle);

      double& actuator_state = joint.actuator_state.current[interface_name];
      actuator_state = 0.0;
      transmission_interface::ActuatorHandle actuator_handle(actuator_name, interface_name, &actuator_state);
      actuator_handles.push_back(actuator_handle);
    }
    joint.state_transmission->configure(joint_handles, actuator_handles);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr> DynamixelHardwareInterface::on_export_command_interfaces()
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_export_command_interfaces");
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;

  // Create the state interfaces
  for (auto& [name, joint] : joints_) {
    joint.joint_state.goal.reserve(joint.getAvailableCommandInterfaces().size());
    for (const auto& interface_name : joint.getAvailableCommandInterfaces()) {
      joint.joint_state.goal[interface_name] = 0.0;
      const auto command_interface = std::make_shared<hardware_interface::CommandInterface>(
          joint.name, interface_name, &joint.joint_state.goal[interface_name]);
      command_interfaces.emplace_back(command_interface);
    }
  }

  // Set up transmission interface
  for (const auto& transmission_info : info_.transmissions) {
    std::vector<transmission_interface::JointHandle> joint_handles;
    std::vector<transmission_interface::ActuatorHandle> actuator_handles;
    const std::string& joint_name = transmission_info.joints.front().name;
    const std::string& actuator_name = transmission_info.actuators.front().name;
    Joint& joint = joints_.at(joint_name);  // This should exist
    for (const auto& interface_name : joint.getAvailableCommandInterfaces()) {
      transmission_interface::JointHandle joint_handle(joint_name, interface_name,
                                                       &joint.joint_state.goal[interface_name]);
      joint_handles.push_back(joint_handle);

      double& actuator_state = joint.actuator_state.goal[interface_name];
      actuator_state = 0.0;
      transmission_interface::ActuatorHandle actuator_handle(actuator_name, interface_name, &actuator_state);
      actuator_handles.push_back(actuator_handle);
    }
    joint.command_transmission->configure(joint_handles, actuator_handles);
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

  // Start & stop interfaces
  if (!processCommandInterfaceUpdates(start_interfaces, false)) {
    return hardware_interface::return_type::ERROR;
  }
  if (!processCommandInterfaceUpdates(stop_interfaces, true)) {
    return hardware_interface::return_type::ERROR;
  }

  // Reset all goal states and verify that the cmds were written correctly
  if (!resetGoalStateAndVerify()) {
    return hardware_interface::return_type::ERROR;
  }

  // Write control mode
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
  std::unique_lock<std::mutex> lock(dynamixel_comm_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    // Another operation is holding dynamixel_comm_mutex_; skipping read
    return hardware_interface::return_type::OK;
  }
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

  for (auto& [name, joint] : joints_) {
    if (joint.state_transmission) {
      joint.state_transmission->actuator_to_joint();
    }

    // reset after first read (e.g. goal position = current position)
    if (!first_read_successful_) {
      joint.resetGoalState();
    }
  }

  first_read_successful_ = true;
  last_successful_read_time_ = time;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                                  const rclcpp::Duration& /*period*/)
{
  std::unique_lock<std::mutex> lock(dynamixel_comm_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    // Another operation is holding dynamixel_comm_mutex_; skipping write
    return hardware_interface::return_type::OK;
  }
  // Wait for a successful read after changing the control mode
  for (auto& [name, joint] : joints_) {
    if (joint.command_transmission) {
      joint.command_transmission->joint_to_actuator();
    }
  }

  if (!first_read_successful_) {
    // DXL_LOG_ERROR("Write called without successful read. This should not happen.");
    return hardware_interface::return_type::OK;
  }

  // do not write controller commands if e-stop is active
  if (e_stopp_active_)
    return hardware_interface::return_type::OK;

  control_write_manager_.write();

  if (!control_write_manager_.isOk()) {
    DXL_LOG_ERROR("Write manager lost connection");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

bool DynamixelHardwareInterface::loadTransmissionConfiguration()
{
  auto simple_transmission_loader = transmission_interface::SimpleTransmissionLoader();
  auto adjustable_offset_transmission_loader = hector_transmission_interface::AdjustableOffsetTransmissionLoader();
  for (const auto& transmission_info : info_.transmissions) {
    // only simple transmissions are supported right now
    if (transmission_info.type != "transmission_interface/SimpleTransmission" &&
        transmission_info.type != "hector_transmission_interface/AdjustableOffsetTransmission") {
      RCLCPP_FATAL(get_logger(), "Transmission '%s' of type '%s' not supported.", transmission_info.name.c_str(),
                   transmission_info.type.c_str());
      return false;
    }
    // Only exactly one joint and actuator per transmission
    if (transmission_info.joints.size() != 1 || transmission_info.actuators.size() != 1) {
      RCLCPP_FATAL(get_logger(), "Only transmissions with exactly one joint and actuator are supported");
      return false;
    }

    // Load transmission from info
    std::shared_ptr<transmission_interface::Transmission> state_transmission;
    std::shared_ptr<transmission_interface::Transmission> command_transmission;
    try {
      if (transmission_info.type == "transmission_interface/SimpleTransmission") {
        state_transmission = simple_transmission_loader.load(transmission_info);
        command_transmission = simple_transmission_loader.load(transmission_info);
      } else if (transmission_info.type == "hector_transmission_interface/AdjustableOffsetTransmission") {
        state_transmission = adjustable_offset_transmission_loader.load(transmission_info);
        command_transmission = adjustable_offset_transmission_loader.load(transmission_info);
      }
    }
    catch (const transmission_interface::TransmissionInterfaceException& exc) {
      RCLCPP_FATAL(get_logger(), "Error while loading %s: %s", transmission_info.name.c_str(), exc.what());
      return false;
    }

    // Assign to joint
    try {
      Joint& joint = joints_.at(transmission_info.joints.front().name);
      joint.state_transmission = state_transmission;
      joint.command_transmission = command_transmission;
    }
    catch (std::out_of_range&) {
      DXL_LOG_ERROR("Unknown joint '" << transmission_info.joints.front().name << "' in transmission interface");
      return false;
    }
  }
  return true;
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
    auto& [current_state, goal_state] = joint.getActuatorState();
    for (auto& [interface_name, interface_value] : current_state) {
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
bool DynamixelHardwareInterface::setUpCmdReadManager()
{
  cmd_read_manager_ = SyncReadManager();
  std::unordered_map<std::string, DxlValueMappingList> register_dynamixel_mappings;
  for (auto& [name, joint] : joints_) {
    cmd_read_manager_.addDynamixel(joint.dynamixel.get());
    for (auto& cmd_interface : joint.getAvailableCommandInterfaces()) {
      DXL_LOG_WARN("SetupCmdReadManager: Registering command interface '" << cmd_interface << "' for joint '"
                                                                          << joint.name << "'");
      joint.read_goal_values_[cmd_interface] = std::numeric_limits<double>::quiet_NaN();  // Initialize read goal values
      std::string register_name = joint.commandInterfaceToRegisterName(cmd_interface);
      register_dynamixel_mappings[register_name].push_back(std::make_pair<Dynamixel*, DxlValue>(
          joint.dynamixel.get(), DxlValue(&joint.read_goal_values_.at(cmd_interface))));
    }
  }

  for (const auto& [register_name, dynamixel_mapping] : register_dynamixel_mappings) {
    cmd_read_manager_.addRegister(register_name, dynamixel_mapping);
  }

  return cmd_read_manager_.init(driver_);
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
    auto& [current_state, goal_state] = joint.getActuatorState();
    for (const auto& interface_name : joint.getAvailableCommandInterfaces()) {
      const std::string register_name = joint.commandInterfaceToRegisterName(interface_name);
      control_write_manager_.addRegister(*joint.dynamixel, register_name, goal_state.at(interface_name));
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

bool DynamixelHardwareInterface::setTorque(const bool do_enable, bool skip_controller_unloading, int retries,
                                           const bool direct_write)
{
  std::lock_guard<std::mutex> lock(dynamixel_comm_mutex_);

  // Check if already in desired state
  bool all_torqued = true;
  bool all_torqued_off = true;
  for (const auto& [name, joint] : joints_) {
    bool joint_torqued;
    if (!joint.dynamixel->readRegister(DXL_REGISTER_CMD_TORQUE, joint_torqued)) {
      return false;
    }
    all_torqued &= joint_torqued;
    all_torqued_off &= !joint_torqued;
  }
  // skip if all joints are already in the desired state
  if (all_torqued != all_torqued_off && all_torqued == do_enable) {
    DXL_LOG_INFO("All joints already have torque " << (do_enable ? "ENABLED" : "DISABLED") << ". Skipping ...");
    return true;
  }

  if (do_enable) {
    // reset goal state before enabling torque && verify that goal positions are set correctly
    if (!resetGoalStateAndVerify())
      return false;
  }

  // unload all controllers of the hardware interface
  if (!skip_controller_unloading && !unloadControllers()) {
    DXL_LOG_ERROR("Failed to deactivate controllers before changing torque. Still adapting torque...");
  }
  DXL_LOG_INFO((do_enable ? "Enabling" : "Disabling") << " motor torque.");

  bool success = false;
  retries = std::max(retries, 1);  // ensure at least one attempt
  int counter = 0;
  while (counter < retries && !success) {
    success = true;
    for (auto& [name, joint] : joints_) {
      joint.torque = do_enable;  // set torque of each joint -> also relevant for indirect write
      if (direct_write && !joint.dynamixel->writeRegister(DXL_REGISTER_CMD_TORQUE, joint.torque)) {
        success = false;
        break;  // Break if direct write fails
      }
    }

    if (!direct_write && !torque_write_manager_.write()) {
      success = false;
    }

    counter++;
    if (!success) {
      DXL_LOG_WARN("Failed to set torque for all joints. Retrying... (" << counter << " of " << retries << ")");
    }
  }
  if (success) {
    is_torqued_ = do_enable;
    updateColorLED();
    return true;
  }
  return false;
}

bool DynamixelHardwareInterface::resetGoalStateAndVerify()
{
  // Read current values (positions, velocities, etc.) before enabling torque
  if (!read_manager_.read() || !read_manager_.isOk() || !isHardwareOk()) {
    DXL_LOG_ERROR("Failed to read current positions before enabling torque. Cannot enable torque.");
    return false;
  }

  // reset goal state -> goal position = current position, goal velocity = 0, etc.
  for (auto& [name, joint] : joints_) {
    joint.resetGoalState();
  }

  // Write goal positions (will only write for the values belonging to the active command interfaces!)
  if (!control_write_manager_.write() || !control_write_manager_.isOk() || !isHardwareOk()) {
    DXL_LOG_ERROR("Failed to write goal positions before enabling torque. Cannot enable torque.");
    return false;
  }

  // Re-read goal values for verification
  if (!cmd_read_manager_.read() || !cmd_read_manager_.isOk()) {
    DXL_LOG_ERROR("Failed to re-read goal positions before enabling torque. Cannot verify goal positions.");
    return false;
  }

  // Verify goal command values match the read values (for active command interfaces)
  for (auto& [name, joint] : joints_) {
    for (const auto& interface_name : joint.getAvailableCommandInterfaces()) {
      if (joint.read_goal_values_.count(interface_name) == 0) {
        DXL_LOG_ERROR("Cannot verify cmd values from motor " << name << "!");
        return false;
      }
      const auto& interface_value = joint.read_goal_values_.at(interface_name);
      if (std::abs(interface_value - joint.getActuatorState().goal[interface_name]) > 1e-2) {
        DXL_LOG_ERROR("Joint '" << name << "' goal " << interface_name
                                << " does not match read goal position before enabling torque. "
                                << "(Current: " << joint.getActuatorState().goal[interface_name]
                                << ", Read Goal Position: " << interface_value << ")");
        return false;
      }
    }
  }
  return true;
}

bool DynamixelHardwareInterface::unloadControllers() const
{
  auto ctrls = controller_orchestrator_->getActiveControllerOfHardwareInterface(get_name());
  if (!controller_orchestrator_->deactivateControllers(ctrls)) {
    DXL_LOG_ERROR("Failed to deactivate controllers.");
    return false;
  }
  return true;
}

void DynamixelHardwareInterface::setColorLED(const int& red, const int& green, const int& blue)
{
  for (auto& [name, joint] : joints_) {
    if (!joint.dynamixel->writeRegister(DXL_REGISTER_LED_RED, red) ||
        !joint.dynamixel->writeRegister(DXL_REGISTER_LED_GREEN, green) ||
        !joint.dynamixel->writeRegister(DXL_REGISTER_LED_BLUE, blue)) {
      DXL_LOG_ERROR("Failed to set color LED for joint '" << name << "'");
    }
  }
}

void DynamixelHardwareInterface::setColorLED(const std::string& color)
{
  DXL_LOG_INFO("Setting color LED '" << color << "'");
  if (color == COLOR_RED) {
    setColorLED(COLOR_RED_VALUES[0], COLOR_RED_VALUES[1], COLOR_RED_VALUES[2]);
  } else if (color == COLOR_GREEN) {
    setColorLED(COLOR_GREEN_VALUES[0], COLOR_GREEN_VALUES[1], COLOR_GREEN_VALUES[2]);
  } else if (color == COLOR_BLUE) {
    setColorLED(COLOR_BLUE_VALUES[0], COLOR_BLUE_VALUES[1], COLOR_BLUE_VALUES[2]);
  } else if (color == COLOR_ORANGE) {
    setColorLED(COLOR_ORANGE_VALUES[0], COLOR_ORANGE_VALUES[1], COLOR_ORANGE_VALUES[2]);
  } else {
    DXL_LOG_ERROR("Unknown color: " << color);
  }
}

void DynamixelHardwareInterface::updateColorLED(std::string new_state)
{
  if (new_state.empty())
    new_state = lifecycle_state_.label();
  if (new_state == hardware_interface::lifecycle_state_names::UNCONFIGURED ||
      new_state == hardware_interface::lifecycle_state_names::INACTIVE) {
    setColorLED(COLOR_RED);
  } else {
    // hardware interface is active
    if (!is_torqued_) {
      setColorLED(COLOR_GREEN);
    } else if (e_stopp_active_) {
      setColorLED(COLOR_ORANGE);
    } else {
      setColorLED(COLOR_BLUE);
    }
  }
}

void DynamixelHardwareInterface::adjustTransmissionOffsetsCallback(
    const std::shared_ptr<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets::Request> request,
    const std::shared_ptr<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets::Response> response)
{
  DXL_LOG_INFO("Request to adjust transmission offsets received.");
  response->success = true;

  if (!unloadControllers()) {
    DXL_LOG_INFO("Failed to unload controllers. Cannot adjust offsets.");
    response->success = false;
    response->message = "Failed to deactivate controllers. Cannot adjust offsets.";
  }

  for (size_t i = 0; i < request->external_joint_measurements.name.size(); ++i) {
    const auto& joint_name = request->external_joint_measurements.name[i];
    const auto& external_joint_position = request->external_joint_measurements.position[i];
    const auto& internal_joint_position = joints_[joint_name].joint_state.current["position"];
    double corrected_offset = std::numeric_limits<double>::quiet_NaN();
    std::string transmission_type;
    for (const auto& info : info_.transmissions) {
      if (info.joints.front().name == joint_name) {
        transmission_type = info.type;
        break;
      }
    }
    if (transmission_type == "hector_transmission_interface/AdjustableOffsetTransmission") {
      auto adjustable_state = std::dynamic_pointer_cast<hector_transmission_interface::AdjustableOffsetTransmission>(
          joints_[joint_name].state_transmission);
      auto adjustable_command = std::dynamic_pointer_cast<hector_transmission_interface::AdjustableOffsetTransmission>(
          joints_[joint_name].command_transmission);

      if (!adjustable_state || !adjustable_command) {
        DXL_LOG_ERROR("Failed to cast transmission for joint '" << joint_name << "'.");
        response->success = false;
        response->message = "Transmission cast failed for joint: " + joint_name;
        return;
      }
      double current_offset = adjustable_state->get_joint_offset();
      corrected_offset = external_joint_position - internal_joint_position + current_offset;

      adjustable_state->adjustTransmissionOffset(corrected_offset);
      adjustable_command->adjustTransmissionOffset(corrected_offset);
      DXL_LOG_INFO("Adjusted offset for joint '" << joint_name << "' to " << corrected_offset);
    } else {
      DXL_LOG_ERROR("Transmission type '" << transmission_type << "' is not supported for offset adjustment.");
      response->success = false;
      response->message = "Unsupported transmission type: " + transmission_type;
      return;
    }

    response->adjusted_offsets.push_back(corrected_offset);
  }

  response->success = true;
  response->message = "Offsets adjusted successfully";
}

bool DynamixelHardwareInterface::setEStop(bool do_enable)
{
  if (do_enable != e_stopp_active_) {
    if (do_enable) {
      if (!is_torqued_) {
        DXL_LOG_WARN("Torqued not set, cannot activate e-stop");
        return false;
      }
      // Activating e-stop
      DXL_LOG_WARN("E-STOP ACTIVATED via topic");
      if (!unloadControllers()) {
        DXL_LOG_ERROR("Failed to unload controllers. Cannot activate e-stop.");
        return false;
      }
      activateEStop();
    } else {
      DXL_LOG_WARN("E-STOP INACTIVATED via topic");
      e_stopp_active_ = false;
      std::lock_guard<std::mutex> lock(dynamixel_comm_mutex_);
      updateColorLED();
    }
  }
  return true;
}

bool DynamixelHardwareInterface::activateEStop()
{
  std::lock_guard<std::mutex> lock(dynamixel_comm_mutex_);
  // switch to position mode if not already in it
  for (auto& [name, joint] : joints_) {
    const auto available_interfaces = joint.getAvailableCommandInterfaces();
    if (std::find(available_interfaces.begin(), available_interfaces.end(), hardware_interface::HW_IF_POSITION) ==
        available_interfaces.end()) {
      DXL_LOG_ERROR("Joint '" << name << "' does not support position control. Cannot activate e-stop.");
      return false;
    }
    if (!joint.isPositionControlled()) {
      const auto active_interfaces = joint.getActiveCommandInterfaces();
      for (const auto& active_interface : active_interfaces) {
        joint.removeActiveCommandInterface(active_interface);
      }
      joint.addActiveCommandInterface(hardware_interface::HW_IF_POSITION);
      if (!joint.updateControlMode())
        return false;
    }
  }
  // resetGoalStates
  if (!resetGoalStateAndVerify()) {
    DXL_LOG_WARN("Failed to reset goal state while attempting to activate the software e-stop.");
  }

  // clear active command interfaces [no controller is active - no command interfaces should be active]
  for (auto& [name, joint] : joints_) {
    const auto active_interfaces = joint.getActiveCommandInterfaces();
    for (const auto& active_interface : active_interfaces) {
      joint.removeActiveCommandInterface(active_interface);
    }
  }
  // Note: the motor is still in position control mode with the last goal position set

  e_stopp_active_ = true;
  updateColorLED();
  return true;
}

}  // namespace dynamixel_ros_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_ros_control::DynamixelHardwareInterface, hardware_interface::SystemInterface)
