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
DynamixelHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& param)
{
  // Load hardware configuration
  const auto result = SystemInterface::on_init(param);
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

  bool use_dummy = false;
  getParameter(info_.hardware_parameters, "use_dummy", use_dummy, false);

  // Initialize driver
  if (!driver_.init(port_name, baud_rate, use_dummy)) {
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
  std::vector<std::string> mimic_joint_names;
  mimic_joint_names.reserve(info_.mimic_joints.size());
  for (const auto& mimic_joint : info_.mimic_joints) {
    mimic_joint_names.emplace_back(info_.joints[mimic_joint.joint_index].name);
  }
  for (const auto& joint_info : info_.joints) {
    // skip if it is a mimic joint -> either mimic attribute is set or it is listed in the mimic joints
    if (joint_info.is_mimic == hardware_interface::MimicAttribute::TRUE ||
        std::find(mimic_joint_names.begin(), mimic_joint_names.end(), joint_info.name) != mimic_joint_names.end())
      continue;
    if (use_dummy) {
      // Register mock motor before joint configuration (needs ID and model number)
      int id_val;
      if (getParameter(joint_info.parameters, "id", id_val)) {
        int model_number = DEFAULT_MOCK_MODEL_NUMBER;
        getParameter(joint_info.parameters, "model_number", model_number, static_cast<int>(DEFAULT_MOCK_MODEL_NUMBER));
        driver_.addDummyMotor(static_cast<uint8_t>(id_val), static_cast<uint16_t>(model_number));
      }
    }

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
    joint_names_.emplace_back(joint_info.name);
  }

  // mimic joint setup
  for (const auto& mimic_joint : info_.mimic_joints) {
    const auto& name = info_.joints[mimic_joint.joint_index].name;
    const auto& mimicked_name = info_.joints[mimic_joint.mimicked_joint_index].name;
    if (joints_.count(mimicked_name) > 0) {
      joints_[mimicked_name].setupMimicJoint(name, mimic_joint.offset, mimic_joint.multiplier);
    }
  }

  // create and spinn a ros2 node in a separate thread
  // (making sure it gets a separate name but the same namespace as the controller manager)
  auto tmp_node = rclcpp::Node::make_shared("dynamixel_ros_control_node");
  auto ns = std::string(tmp_node->get_namespace());
  node_ =
      std::make_shared<rclcpp::Node>(param.hardware_info.name, ns, rclcpp::NodeOptions().use_global_arguments(false));
  exe_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  exe_->add_node(node_);
  exe_thread_ = std::thread([this] { exe_->spin(); });

  // create a service to set torque
  set_torque_service_ = node_->create_service<std_srvs::srv::SetBool>(
      "~/set_torque", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                             const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        if (lifecycle_state_.label() != hardware_interface::lifecycle_state_names::ACTIVE) {
          response->success = false;
          response->message = "Hardware Interface must be in 'active' state to set torque";
          return;
        }
        DXL_LOG_INFO("Request to set torque to " << (request->data ? "ON" : "OFF") << " received.");
        response->success = setTorque(request->data);
        response->message = response->success ? "Torque set successfully" : "Failed to set torque";
      });

  // reboot service - allows to reboot actuators if they are in an error state
  // After reboot, the desired torque state is restored and LEDs are updated automatically
  reboot_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/reboot", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void) request;
        RCLCPP_INFO(node_->get_logger(), "Reboot request received.");
        response->success = reboot();
        response->message = response->success ? "Rebooted successfully" : "Failed to reboot";
      });

  // Setup Adjustable Transmission Offset Manager
  auto pre_callback = [this]() { return deactivateControllers(); };
  auto post_callback = [this]() {
    first_read_successful_ = false;  // force read after offset adjustment
    return true;
  };
  offset_manager_ = std::make_shared<hector_transmission_interface::AdjustableOffsetManager>(
      node_, std::ref(dynamixel_comm_mutex_), std::make_optional(pre_callback), std::make_optional(post_callback));

  // setup controller orchestrator
  controller_orchestrator_ = std::make_shared<controller_orchestrator::ControllerOrchestrator>(node_);

  // set up e-stop subscription
  std::string topic = ns != "/" ? ns + "/soft_e_stop" : "/soft_e_stop";
  soft_e_stop_subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
      topic, rclcpp::SystemDefaultsQoS(), [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (lifecycle_state_.label() != hardware_interface::lifecycle_state_names::ACTIVE) {
          DXL_LOG_WARN("E-Stop message received but hardware interface is not in 'active' state.");
          return;
        }
        setEStop(msg->data);
      });
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
  // reset internal variables
  first_read_successful_ = false;
  mode_switch_failed_ = false;
  e_stop_active_ = false;

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

  // Set up sync read / write managers
  if (!setUpStateAndStatusReadManager() || !setUpTorqueWriteManager() || !setUpControlWriteManager() ||
      !setUpCmdReadManager() || !setUpLEDWriteManager()) {
    return hardware_interface::CallbackReturn::FAILURE;
  }

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
  // make sure position control mode is active (safer than leaving it in whatever mode it was before)
  // imagine, torque on startup but actuator from last shutdown in current mode & no controller running
  for (auto& [name, joint] : joints_) {
    if (!joint.readControlMode() || !joint.updateControlMode()) {
      return CallbackReturn::ERROR;
    }
  }
  is_torqued_ = torque_on_startup_;
  if (!resetGoalStateAndVerify(joint_names_)) {
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

    // mimic joints
    for (auto& [mimic_joint_name, mimic_state] : joint.mimic_joints_states_) {
      mimic_state.current.reserve(configured_state_interface_names.size());
      for (const auto& interface_name : configured_state_interface_names) {
        mimic_state.current[interface_name] = 0.0;
        const auto state_interface = std::make_shared<hardware_interface::StateInterface>(
            mimic_joint_name, interface_name, &mimic_state.current[interface_name]);
        state_interfaces.emplace_back(state_interface);
      }
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

  // register offset manager state interfaces
  for (const auto& [name, joint] : joints_) {
    // try to register only adjustable offset transmissions
    offset_manager_->add_joint_state_interface(name, joint.state_transmission, [&joint]() {
      return joint.joint_state.current.at(hardware_interface::HW_IF_POSITION);
    });
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
  // register offset manager command interfaces
  for (const auto& [name, joint] : joints_) {
    // try to register only adjustable offset transmissions
    offset_manager_->add_joint_command_interface(name, joint.command_transmission);
  }
  return command_interfaces;
}

hardware_interface::return_type
DynamixelHardwareInterface::perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                        const std::vector<std::string>& stop_interfaces)
{
  std::lock_guard<std::mutex> lock(dynamixel_comm_mutex_);
  // make sure no commands are written while switching the control mode and in case of errors
  // makes sure not to write commands while control modes out of sync
  mode_switch_failed_ = true;
  // Set up write manager
  DXL_LOG_DEBUG("DynamixelHardwareInterface::perform_command_mode_switch");
  DXL_LOG_DEBUG("start_interfaces: " << iterableToString(start_interfaces));
  DXL_LOG_DEBUG("stop_interfaces: " << iterableToString(stop_interfaces));

  // Ensure we have current state before switching modes
  if (!first_read_successful_) {
    DXL_LOG_INFO("Performing initial read before controller activation.");
    if (!read_manager_.read() || !read_manager_.isOk()) {
      DXL_LOG_ERROR("Failed to read current state before controller activation.");
      return hardware_interface::return_type::ERROR;
    }
    for (auto& [name, joint] : joints_) {
      if (joint.state_transmission) {
        joint.state_transmission->actuator_to_joint();
      }
      joint.resetGoalState();
    }
    first_read_successful_ = true;
  }

  // Start & stop interfaces
  if (!processCommandInterfaceUpdates(stop_interfaces, true)) {
    return hardware_interface::return_type::ERROR;
  }
  if (!processCommandInterfaceUpdates(start_interfaces, false)) {
    return hardware_interface::return_type::ERROR;
  }

  // TODO: refactor - extract all joints that need to be reset
  std::vector<std::string> joints_to_reset;
  for (const auto& full_interface_name : start_interfaces) {
    std::string joint_name;
    std::string interface_name;
    if (!splitFullInterfaceName(full_interface_name, joint_name, interface_name)) {
      return hardware_interface::return_type::ERROR;
    }
    if (std::find(joints_to_reset.begin(), joints_to_reset.end(), joint_name) == joints_to_reset.end())
      joints_to_reset.emplace_back(joint_name);
  }
  for (const auto& full_interface_name : stop_interfaces) {
    std::string joint_name;
    std::string interface_name;
    if (!splitFullInterfaceName(full_interface_name, joint_name, interface_name)) {
      return hardware_interface::return_type::ERROR;
    }
    if (std::find(joints_to_reset.begin(), joints_to_reset.end(), joint_name) == joints_to_reset.end())
      joints_to_reset.emplace_back(joint_name);
  }

  // Reset all goal states and verify that the cmds were written correctly
  if (!resetGoalStateAndVerify(joints_to_reset)) {
    DXL_LOG_ERROR("Failed to reset goal states during command mode switch.");
    return hardware_interface::return_type::ERROR;
  }

  // Write control mode
  for (auto& [name, joint] : joints_) {
    if (!joint.updateControlMode()) {
      DXL_LOG_ERROR("Failed to update control mode for joint '" << joint.name << "' during command mode switch.");
      return hardware_interface::return_type::ERROR;
    }
  }

  first_read_successful_ = false;  // TODO: perform 2nd reset here instead of in read()

  mode_switch_failed_ = false;
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn DynamixelHardwareInterface::on_error(const rclcpp_lifecycle::State& previous_state)
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_error from " << previous_state.label());

  if (isHardwareOk()) {
    return CallbackReturn::SUCCESS;
  }

  // Hardware error detected - set red LEDs for affected motors and activate e-stop
  auto joints_with_error = getJointsWithHardwareError();
  DXL_LOG_ERROR("Hardware error detected in joints: " << iterableToString(joints_with_error));

  {
    std::lock_guard<std::mutex> lock(dynamixel_comm_mutex_);
    // Set red LED for motors with hardware errors
    updateErrorLEDs();
    if (!led_write_manager_.write()) {
      DXL_LOG_WARN("Failed to write error LED colors");
    }
  }

  // Activate e-stop to prevent other actuators from moving
  // This is safer than returning FAILURE which requires controller_manager restart
  if (!activateEStop()) {
    DXL_LOG_ERROR("Failed to activate e-stop during hardware error handling");
  }

  // Note: Automatic reboot is not performed here to avoid blocking the controller manager.
  // Use the reboot service to manually recover from hardware errors.

  // Return SUCCESS to keep the hardware interface running
  // The e-stop ensures safety while allowing recovery attempts via the reboot service
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

  if (!read_manager_.read()) {
    // Single read failure - log but don't return error yet
    DXL_LOG_WARN("Read failed, consecutive errors: " << read_manager_.getErrorCount());
  }

  // Only return error after exceeding the consecutive error threshold
  if (!read_manager_.isOk()) {
    DXL_LOG_ERROR("Read manager lost connection after " << read_manager_.getErrorCount() << " consecutive errors");
    return hardware_interface::return_type::ERROR;
  }

  if (!isHardwareOk()) {
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
    joint.updateMimicJointStates();
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

  // if mode switch failed, bring robot to halt and return error
  if (mode_switch_failed_) {
    // while e-stop not active, try to activate it
    if (!e_stop_active_ && !activateEStop())
      return hardware_interface::return_type::OK;  // sending ok, allows to retry in next write cycle while ignoring cmds
    // forces controller unloading if e-stop is active (-> motors cannot move anymore)
    DXL_LOG_ERROR("In error state, not writing commands.");
    return hardware_interface::return_type::ERROR;
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
  if (e_stop_active_)
    return hardware_interface::return_type::OK;

  if (!control_write_manager_.write() || !control_write_manager_.isOk()) {
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

bool DynamixelHardwareInterface::setUpStateAndStatusReadManager()
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
    register_dynamixel_mappings[DXL_REGISTER_HARDWARE_ERROR].push_back(
        std::make_pair<Dynamixel*, DxlValue>(joint.dynamixel.get(), DxlValue(&joint.dynamixel->hardware_error_status)));
  }

  for (const auto& [register_name, dynamixel_mapping] : register_dynamixel_mappings) {
    if (!read_manager_.addRegister(register_name, dynamixel_mapping)) {
      DXL_LOG_ERROR("Failed to add register '" << register_name << "' to state/status read manager");
      return false;
    }
  }

  return read_manager_.init(driver_);
}

bool DynamixelHardwareInterface::setUpCmdReadManager()
{
  cmd_read_manager_ = SyncReadManager();
  std::unordered_map<std::string, DxlValueMappingList> register_dynamixel_mappings;
  for (auto& [name, joint] : joints_) {
    cmd_read_manager_.addDynamixel(joint.dynamixel.get());
    for (auto& cmd_interface : joint.getAvailableCommandInterfaces()) {
      DXL_LOG_DEBUG("SetupCmdReadManager: Registering command interface '" << cmd_interface << "' for joint '"
                                                                           << joint.name << "'");
      joint.read_goal_values_[cmd_interface] = std::numeric_limits<double>::quiet_NaN();  // Initialize read goal values
      std::string register_name = joint.commandInterfaceToRegisterName(cmd_interface);
      register_dynamixel_mappings[register_name].push_back(std::make_pair<Dynamixel*, DxlValue>(
          joint.dynamixel.get(), DxlValue(&joint.read_goal_values_.at(cmd_interface))));
    }
  }

  for (const auto& [register_name, dynamixel_mapping] : register_dynamixel_mappings) {
    if (!cmd_read_manager_.addRegister(register_name, dynamixel_mapping)) {
      DXL_LOG_ERROR("Failed to add register '" << register_name << "' to command read manager");
      return false;
    }
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

bool DynamixelHardwareInterface::setUpLEDWriteManager()
{
  led_write_manager_ = SyncWriteManager();
  for (auto& [name, joint] : joints_) {
    led_write_manager_.addRegister(*joint.dynamixel, DXL_REGISTER_LED_RED, joint.led_state.red);
    led_write_manager_.addRegister(*joint.dynamixel, DXL_REGISTER_LED_GREEN, joint.led_state.green);
    led_write_manager_.addRegister(*joint.dynamixel, DXL_REGISTER_LED_BLUE, joint.led_state.blue);
  }

  return led_write_manager_.init(driver_);
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

std::vector<std::string> DynamixelHardwareInterface::getJointsWithHardwareError() const
{
  std::vector<std::string> joints_with_error;
  for (const auto& [name, joint] : joints_) {
    if (joint.dynamixel->hardware_error_status != OK) {
      joints_with_error.push_back(name);
    }
  }
  return joints_with_error;
}

bool DynamixelHardwareInterface::reboot()
{
  {
    // lock communication mutex (avoid simultaneous access with read / write)
    std::lock_guard<std::mutex> lock(dynamixel_comm_mutex_);
    for (auto& [name, joint] : joints_) {
      if (joint.dynamixel->hardware_error_status != OK && !joint.dynamixel->reboot()) {
        DXL_LOG_ERROR("Dynamixel '" << name << "' reboot failed.");
        return false;
      }
    }
  }

  // Wait for motors to come back online after reboot
  // Dynamixel motors need time to restart after a reboot command
  get_clock()->sleep_for(rclcpp::Duration(0, REBOOT_WAIT_NS));

  // Refresh hardware status by performing a read
  {
    std::lock_guard<std::mutex> lock(dynamixel_comm_mutex_);
    if (!read_manager_.read() || !read_manager_.isOk()) {
      DXL_LOG_WARN("Failed to read hardware status after reboot, retrying...");
      get_clock()->sleep_for(rclcpp::Duration(0, REBOOT_WAIT_NS));
      if (!read_manager_.read() || !read_manager_.isOk()) {
        DXL_LOG_ERROR("Failed to read hardware status after reboot.");
        return false;
      }
    }
  }

  // Verify hardware is OK after reboot
  if (!isHardwareOk()) {
    DXL_LOG_ERROR("Hardware still reports errors after reboot.");
    return false;
  }

  // Release e-stop since hardware error has been resolved
  if (e_stop_active_) {
    DXL_LOG_INFO("Releasing e-stop after successful reboot.");
    if (!setEStop(false)) {
      DXL_LOG_WARN("Failed to release e-stop after reboot.");
    }
  }

  // Restore desired torque state after reboot (motors default to torque off after reboot)
  DXL_LOG_INFO("Restoring torque state to " << (desired_torque_state_ ? "ON" : "OFF") << " after reboot.");
  if (!setTorque(desired_torque_state_, true)) {
    DXL_LOG_ERROR("Failed to restore torque state after reboot.");
    return false;
  }

  // Ensure LEDs reflect the current state
  updateColorLED();

  return true;
}

bool DynamixelHardwareInterface::setTorque(const bool do_enable, bool skip_controller_unloading, int retries,
                                           const bool direct_write)
{
  // Track the user's desired torque state (for restoration after reboot)
  desired_torque_state_ = do_enable;

  // check if torque change is necessary
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
  }
  // unload all controllers of the hardware interface (if hw is not in unconfigured state)
  RCLCPP_INFO(get_logger(), "Controller unloading before changing torque is %s",
              skip_controller_unloading ? "skipped" : "not skipped");
  if ((!skip_controller_unloading ||
       lifecycle_state_.label() == hardware_interface::lifecycle_state_names::UNCONFIGURED) &&
      !deactivateControllers()) {
    DXL_LOG_ERROR("Failed to deactivate controllers before changing torque. Still adapting torque...");
  }
  DXL_LOG_INFO((do_enable ? "Enabling" : "Disabling") << " motor torque.");

  {
    std::lock_guard<std::mutex> lock(dynamixel_comm_mutex_);
    if (do_enable) {
      // reset goal state before enabling torque && verify that goal positions are set correctly
      if (!resetGoalStateAndVerify(joint_names_))
        return false;
    }
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
  }
  return false;
}

bool DynamixelHardwareInterface::resetGoalStateAndVerify(const std::vector<std::string>& joints)
{
  // Read current values (positions, velocities, etc.) before enabling torque
  if (!read_manager_.read() || !read_manager_.isOk() || !isHardwareOk()) {
    DXL_LOG_ERROR("[resetGoalStateAndVerify] Failed to read current values from actuators.");
    return false;
  }

  for (auto& [name, joint] : joints_) {
    if (joint.state_transmission) {
      joint.state_transmission->actuator_to_joint();
    }
  }

  // reset goal state -> goal position = current position, goal velocity = 0, etc.
  for (auto& name : joints) {
    joints_[name].resetGoalState();
  }

  // Write goal values
  if (!control_write_manager_.write() || !control_write_manager_.isOk() || !isHardwareOk()) {
    DXL_LOG_ERROR("[resetGoalStateAndVerify] Failed to write reset goal values.");
    return false;
  }

  // Re-read goal values for verification
  if (!cmd_read_manager_.read() || !cmd_read_manager_.isOk()) {
    DXL_LOG_ERROR("[resetGoalStateAndVerify] Failed to re-read goal.");
    return false;
  }

  // Verify goal command values match the read values (for active command interfaces)
  for (auto& name : joints) {
    auto& joint = joints_[name];
    for (const auto& interface_name : joints_[name].getAvailableCommandInterfaces()) {
      if (joint.read_goal_values_.count(interface_name) == 0) {
        DXL_LOG_ERROR("[resetGoalStateAndVerify]  Cannot verify cmd values from motor " << name << "!");
        return false;
      }
      const auto& interface_value = joint.read_goal_values_.at(interface_name);
      if (std::abs(interface_value - joint.getActuatorState().goal[interface_name]) > 1e-2) {
        DXL_LOG_ERROR("[resetGoalStateAndVerify] Joint '"
                      << name << "' goal of interface " << interface_name << " does not match read goal value. "
                      << "(Target Goal Value: " << joint.getActuatorState().goal[interface_name]
                      << ", Read Goal Value: " << interface_value);
        return false;
      }
    }
  }
  return true;
}

bool DynamixelHardwareInterface::deactivateControllers() const
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
    joint.led_state.red = red;
    joint.led_state.green = green;
    joint.led_state.blue = blue;
  }
  // Note: Does NOT write to hardware - caller should handle write after all LED changes
}

void DynamixelHardwareInterface::setColorLED(const std::string& color)
{
  DXL_LOG_INFO("Setting color LED '" << color << "'");
  if (color == COLOR_PINK) {
    setColorLED(COLOR_PINK_VALUES[0], COLOR_PINK_VALUES[1], COLOR_PINK_VALUES[2]);
  } else if (color == COLOR_GREEN) {
    setColorLED(COLOR_GREEN_VALUES[0], COLOR_GREEN_VALUES[1], COLOR_GREEN_VALUES[2]);
  } else if (color == COLOR_BLUE) {
    setColorLED(COLOR_BLUE_VALUES[0], COLOR_BLUE_VALUES[1], COLOR_BLUE_VALUES[2]);
  } else if (color == COLOR_ORANGE) {
    setColorLED(COLOR_ORANGE_VALUES[0], COLOR_ORANGE_VALUES[1], COLOR_ORANGE_VALUES[2]);
  } else if (color == COLOR_RED) {
    setColorLED(COLOR_RED_VALUES[0], COLOR_RED_VALUES[1], COLOR_RED_VALUES[2]);
  } else {
    DXL_LOG_ERROR("Unknown color: " << color);
  }
}

void DynamixelHardwareInterface::setJointLED(const std::string& joint_name, const std::string& color)
{
  auto it = joints_.find(joint_name);
  if (it == joints_.end()) {
    DXL_LOG_ERROR("Joint '" << joint_name << "' not found when setting LED");
    return;
  }

  int r = 0, g = 0, b = 0;
  if (color == COLOR_PINK) {
    r = COLOR_PINK_VALUES[0];
    g = COLOR_PINK_VALUES[1];
    b = COLOR_PINK_VALUES[2];
  } else if (color == COLOR_GREEN) {
    r = COLOR_GREEN_VALUES[0];
    g = COLOR_GREEN_VALUES[1];
    b = COLOR_GREEN_VALUES[2];
  } else if (color == COLOR_BLUE) {
    r = COLOR_BLUE_VALUES[0];
    g = COLOR_BLUE_VALUES[1];
    b = COLOR_BLUE_VALUES[2];
  } else if (color == COLOR_ORANGE) {
    r = COLOR_ORANGE_VALUES[0];
    g = COLOR_ORANGE_VALUES[1];
    b = COLOR_ORANGE_VALUES[2];
  } else if (color == COLOR_RED) {
    r = COLOR_RED_VALUES[0];
    g = COLOR_RED_VALUES[1];
    b = COLOR_RED_VALUES[2];
  } else {
    DXL_LOG_ERROR("Unknown color: " << color);
    return;
  }

  it->second.led_state.red = r;
  it->second.led_state.green = g;
  it->second.led_state.blue = b;
}

void DynamixelHardwareInterface::updateErrorLEDs()
{
  // Set red LED for joints with hardware errors
  for (const auto& joint_name : getJointsWithHardwareError()) {
    setJointLED(joint_name, COLOR_RED);
  }
  // Note: Does NOT call led_write_manager_.write() - caller must do this
  // or call this before updateColorLED() which will write all LEDs
}

void DynamixelHardwareInterface::updateColorLED(std::string new_state)
{
  if (new_state.empty())
    new_state = lifecycle_state_.label();
  if (new_state == hardware_interface::lifecycle_state_names::UNCONFIGURED ||
      new_state == hardware_interface::lifecycle_state_names::INACTIVE) {
    setColorLED(COLOR_PINK);  // Pink = inactive/unconfigured
  } else {
    // hardware interface is active
    if (!is_torqued_) {
      setColorLED(COLOR_GREEN);  // Green = torque off (safe to touch)
    } else if (e_stop_active_) {
      setColorLED(COLOR_ORANGE);  // Orange = E-Stop active
    } else {
      setColorLED(COLOR_BLUE);  // Blue = active and torque on
    }
  }
  // Override with red for joints that have hardware errors
  updateErrorLEDs();
  // Write all LED changes to hardware
  if (!led_write_manager_.write()) {
    DXL_LOG_ERROR("Failed to write LED colors.");
  }
}

bool DynamixelHardwareInterface::setEStop(bool do_enable)
{
  if (do_enable != e_stop_active_) {
    if (do_enable) {
      if (!is_torqued_) {
        DXL_LOG_WARN("Torqued not set, cannot activate e-stop");
        return false;
      }
      // Activating e-stop
      DXL_LOG_WARN("E-STOP ACTIVATED via topic");
      // unload controllers (not possible if hardware interface is not configured)
      if (lifecycle_state_.label() != hardware_interface::lifecycle_state_names::UNCONFIGURED) {
        if (!deactivateControllers()) {
          DXL_LOG_WARN("Failed to deactivate controllers during e-stop activation. Proceeding with e-stop anyway.");
        }
      }
      std::lock_guard<std::mutex> lock(dynamixel_comm_mutex_);
      activateEStop();
    } else {
      // Before deactivating e-stop, check if controllers are still active.
      // If controller deactivation failed during e-stop activation, controllers may still be active.
      // Attempt to deactivate them now; if that fails, e-stop must remain active.
      if (lifecycle_state_.label() != hardware_interface::lifecycle_state_names::UNCONFIGURED) {
        auto active_controllers = controller_orchestrator_->getActiveControllerOfHardwareInterface(get_name());
        if (!active_controllers.empty()) {
          DXL_LOG_WARN("Controllers still active during e-stop deactivation: "
                       << iterableToString(active_controllers) << ". Attempting to deactivate them now.");
          if (!deactivateControllers()) {
            DXL_LOG_ERROR("Failed to deactivate controllers. E-stop will remain active for safety.");
            return false;
          }
        }
      }
      DXL_LOG_WARN("E-STOP INACTIVATED via topic");
      e_stop_active_ = false;
      std::lock_guard<std::mutex> lock(dynamixel_comm_mutex_);
      updateColorLED();
    }
  }
  return true;
}

bool DynamixelHardwareInterface::activateEStop()
{
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
        (void) joint.removeActiveCommandInterface(active_interface);  // Intentionally ignore, clearing all
      }
      if (!joint.addActiveCommandInterface(hardware_interface::HW_IF_POSITION)) {
        DXL_LOG_WARN("Failed to add position interface during e-stop activation");
      }
      if (!joint.updateControlMode())
        return false;
    }
  }
  // resetGoalStates
  if (!resetGoalStateAndVerify(joint_names_)) {
    DXL_LOG_WARN("Failed to reset goal state while attempting to activate the software e-stop.");
  }

  // clear active command interfaces [no controller is active - no command interfaces should be active]
  for (auto& [name, joint] : joints_) {
    const auto active_interfaces = joint.getActiveCommandInterfaces();
    for (const auto& active_interface : active_interfaces) {
      (void) joint.removeActiveCommandInterface(active_interface);  // Intentionally ignore, clearing all
    }
  }
  // Note: the motor is still in position control mode with the last goal position set

  e_stop_active_ = true;
  updateColorLED();
  return true;
}

}  // namespace dynamixel_ros_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_ros_control::DynamixelHardwareInterface, hardware_interface::SystemInterface)
