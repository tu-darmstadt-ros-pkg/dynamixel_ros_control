#include "dynamixel_ros_control/common.hpp"
#include "dynamixel_ros_control/log.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <tuple>

#include <ament_index_cpp/get_package_share_path.hpp>
#include <dynamixel_ros_control/dynamixel_hardware_interface.hpp>
#include <limits>
#include <rcl/error_handling.h>
#include <rcl/remap.h>
#include <rclcpp/contexts/default_context.hpp>

#include <transmission_interface/simple_transmission_loader.hpp>
#include <transmission_interface/transmission.hpp>
#include <transmission_interface/transmission_interface_exception.hpp>
#include <hector_transmission_interface/adjustable_offset_transmission_loader.hpp>

namespace dynamixel_ros_control {

namespace {

// How often read() refreshes the diagnostics snapshot. Above the 1 Hz publish timer, so a sample is
// usually newer than the publish that carries it.
constexpr double HEALTH_SNAPSHOT_HZ = 2.0;

// A dropped status packet is ordinary on a shared bus, and every other transaction in this
// interface retries one. Failing activation instead costs a full configure-and-activate round.
constexpr int CONTROL_MODE_READ_ATTEMPTS = 3;

/// @brief Bind one transmission to a joint's values for the given interfaces.
///
/// The state and the command side differ only in which State member and which interface list they
/// use. The wiring itself (build a handle pair per interface, zero the actuator side, configure)
/// lives here so it is not written out twice.
template <typename Interfaces>
void configureTransmission(const hardware_interface::TransmissionInfo& transmission_info,
                           const Interfaces& interface_names, std::unordered_map<std::string, double>& joint_values,
                           std::unordered_map<std::string, double>& actuator_values,
                           transmission_interface::Transmission& transmission,
                           dynamixel_ros_control::PassThrough& pass_through)
{
  const std::string& joint_name = transmission_info.joints.front().name;
  const std::string& actuator_name = transmission_info.actuators.front().name;
  const std::vector<std::string> supported = transmission.get_supported_joint_interfaces();
  std::vector<transmission_interface::JointHandle> joint_handles;
  std::vector<transmission_interface::ActuatorHandle> actuator_handles;
  pass_through.clear();
  for (const auto& interface_name : interface_names) {
    double& joint_value = joint_values[interface_name];
    double& actuator_value = actuator_values[interface_name];
    actuator_value = 0.0;
    if (std::find(supported.begin(), supported.end(), interface_name) == supported.end()) {
      pass_through.emplace_back(&joint_value, &actuator_value);
      continue;
    }
    joint_handles.emplace_back(joint_name, interface_name, &joint_value);
    actuator_handles.emplace_back(actuator_name, interface_name, &actuator_value);
  }
  transmission.configure(joint_handles, actuator_handles);
}

struct InterfaceRef
{
  std::string joint;
  std::string interface;
};

/// Only for names switchRefusal() accepted, which all split into a known joint
std::vector<InterfaceRef> parseInterfaces(const std::vector<std::string>& full_names)
{
  std::vector<InterfaceRef> refs(full_names.size());
  for (size_t i = 0; i < full_names.size(); ++i) {
    splitFullInterfaceName(full_names[i], refs[i].joint, refs[i].interface);
  }
  return refs;
}

/// The joints of the @p refs that @p take accepts, each once, in order
template <typename Predicate>
std::vector<std::string> jointsOf(const std::vector<InterfaceRef>& refs, const Predicate& take)
{
  std::vector<std::string> joints;
  for (const auto& ref : refs) {
    if (take(ref) && std::find(joints.begin(), joints.end(), ref.joint) == joints.end()) {
      joints.push_back(ref.joint);
    }
  }
  return joints;
}

}  // namespace

hardware_interface::CallbackReturn
DynamixelHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& param)
{
  const auto result = SystemInterface::on_init(param);
  if (result != CallbackReturn::SUCCESS) {
    return result;
  }
  logger_ = rclcpp::get_logger(DXL_LOGGER_NAME).get_child(info_.name);

  if (!loadHardwareParameters() || !loadJoints()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  bus_watchdog_.init(&joints_, &groups_.watchdog_write, logger_);
  if (!setUpRosInterfaces(param.hardware_info.name) || !loadTransmissionConfiguration()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool DynamixelHardwareInterface::loadHardwareParameters()
{
  // A misspelt parameter would otherwise be ignored and its default used without a word.
  const auto unknown =
      unknownParameters(info_.hardware_parameters,
                        {"allow_velocity_without_watchdog", "baud_rate", "bus_watchdog_cycles", "debug", "port_name",
                         "publish_goal_joint_states", "publish_read_joint_states", "publish_write_joint_states",
                         "torque_off_on_shutdown", "torque_on_startup", "use_dummy"});
  if (!unknown.empty()) {
    DXL_LOG_ERROR("Unknown hardware parameters: " << iterableToString(unknown) << ".");
    return false;
  }
  std::string port_name;
  int baud_rate;
  // Named individually: an on_init refusal leaves the controller manager with no services and
  // waiting for a new description, so these log lines are all the operator gets.
  if (!getParameter<std::string>(info_.hardware_parameters, "port_name", port_name)) {
    DXL_LOG_ERROR("Hardware parameter 'port_name' is missing or unusable.");
    return false;
  }
  if (!getParameter<int>(info_.hardware_parameters, "baud_rate", baud_rate)) {
    DXL_LOG_ERROR("Hardware parameter 'baud_rate' is missing or unusable.");
    return false;
  }

  bool debug = false;
  bool use_dummy = false;
  const ParameterMap& parameters = info_.hardware_parameters;
  if (!getOptionalParameter(parameters, "debug", debug, false) ||
      !getOptionalParameter(parameters, "torque_on_startup", torque_on_startup_, false) ||
      !getOptionalParameter(parameters, "torque_off_on_shutdown", torque_off_on_shutdown_, false) ||
      !getOptionalParameter(parameters, "allow_velocity_without_watchdog", allow_velocity_without_watchdog_, false) ||
      !getOptionalParameter(parameters, "use_dummy", use_dummy, false)) {
    return false;
  }
  if (debug) {
    logger_.set_level(rclcpp::Logger::Level::Debug);
  }

  if (!driver_.init(port_name, baud_rate, use_dummy, logger_)) {
    DXL_LOG_ERROR("Failed to initialize driver");
    return false;
  }
  return true;
}

bool DynamixelHardwareInterface::loadJoints()
{
  std::unordered_map<std::string, std::string> state_interface_to_register;
  std::unordered_map<std::string, std::string> command_interface_to_register;
  std::unordered_map<std::string, std::string> interface_to_register_limits;
  const std::string translation_path = ament_index_cpp::get_package_share_path("dynamixel_ros_control").string() +
                                       "/devices/interface_to_register_names.yaml";
  if (!loadInterfaceRegisterNameTranslation(translation_path, state_interface_to_register,
                                            command_interface_to_register, interface_to_register_limits)) {
    return false;
  }

  std::vector<std::string> mimic_joint_names;
  mimic_joint_names.reserve(info_.mimic_joints.size());
  for (const auto& mimic_joint : info_.mimic_joints) {
    mimic_joint_names.emplace_back(info_.joints[mimic_joint.joint_index].name);
  }

  joints_.reserve(info_.joints.size());
  std::unordered_map<int, std::string> joint_of_id;
  for (const auto& joint_info : info_.joints) {
    // A mimic joint has no motor of its own. It is driven from the joint it mimics.
    if (joint_info.is_mimic == hardware_interface::MimicAttribute::TRUE ||
        std::find(mimic_joint_names.begin(), mimic_joint_names.end(), joint_info.name) != mimic_joint_names.end()) {
      continue;
    }
    const auto unknown =
        unknownParameters(joint_info.parameters,
                          {"do_not_reset_on_ctrl_change", "id", "model_number", "position_control_mode"}, "registers.");
    if (!unknown.empty()) {
      DXL_LOG_ERROR("Joint '" << joint_info.name << "' has unknown parameters: " << iterableToString(unknown) << ".");
      return false;
    }
    int id_val;
    const bool has_id = getParameter(joint_info.parameters, "id", id_val);
    // Two joints on one id are two names for one motor: each writes its own initial register values
    // and its own goals to it. The sync read group refuses the repeated id, but only once both
    // configurations have reached the EEPROM, so it is refused here, ahead of any bus I/O.
    // A missing or unusable id is loadConfiguration()'s to report.
    if (has_id) {
      const auto [known, inserted] = joint_of_id.emplace(id_val, joint_info.name);
      if (!inserted) {
        DXL_LOG_ERROR("Joints '" << known->second << "' and '" << joint_info.name
                                 << "' are both configured with motor id " << id_val
                                 << ". One motor cannot be two joints: each would write its own "
                                    "configuration and its own goals to it.");
        return false;
      }
    }
    Joint joint;
    if (!joint.loadConfiguration(driver_, joint_info, state_interface_to_register, command_interface_to_register,
                                 interface_to_register_limits)) {
      return false;
    }
    DXL_LOG_DEBUG("Loaded dynamixel '"
                  << joint.name << "' (id " << joint.dynamixel->getIdInt() << "): command interfaces "
                  << iterableToString(joint.getAvailableCommandInterfaces()) << ", state interfaces "
                  << iterableToString(joint.getAvailableStateInterfaces()));
    joint_names_.emplace_back(joint.name);
    joints_.emplace(joint.name, std::move(joint));
  }

  for (const auto& mimic_joint : info_.mimic_joints) {
    const auto& name = info_.joints[mimic_joint.joint_index].name;
    const auto& mimicked_name = info_.joints[mimic_joint.mimicked_joint_index].name;
    if (joints_.count(mimicked_name) > 0) {
      joints_[mimicked_name].setupMimicJoint(name, mimic_joint.offset, mimic_joint.multiplier);
    }
  }

  // Every goal reset holds the present position, so it has to be read. State interfaces are read
  // as the union over all joints, so one joint declaring position is enough for all of them.
  if (!joints_.empty() && configuredStateInterfaceUnion().count(hardware_interface::HW_IF_POSITION) == 0) {
    DXL_LOG_ERROR("No joint declares a position state interface. A goal reset holds the present position, "
                  "and without it every hold goal would be 0, which torque-on then drives to.");
    return false;
  }
  return true;
}

bool DynamixelHardwareInterface::setUpRosInterfaces(const std::string& hardware_name)
{
  // Own node on our own thread. The namespace comes from the process's global remap arguments
  // directly: a throwaway node just to read it would cost a second DDS participant.
  const Stopwatch node_setup_timer;
  std::string ns = "/";
  {
    const auto rcl_context = rclcpp::contexts::get_global_default_context()->get_rcl_context();
    char* remapped_ns = nullptr;
    const rcl_ret_t ret =
        rcl_remap_node_namespace(nullptr, &rcl_context->global_arguments, (hardware_name + "_node").c_str(),
                                 rcl_get_default_allocator(), &remapped_ns);
    if (ret == RCL_RET_OK && remapped_ns != nullptr) {
      ns = remapped_ns;
      rcl_get_default_allocator().deallocate(remapped_ns, rcl_get_default_allocator().state);
    } else if (ret != RCL_RET_OK) {
      DXL_LOG_WARN("Failed to resolve the node namespace from remap arguments (rcl error " << ret << "). Using '/'.");
      rcl_reset_error();
    }
  }
  node_ =
      std::make_shared<rclcpp::Node>(hardware_name + "_node", ns, rclcpp::NodeOptions().use_global_arguments(false));
  exe_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  exe_->add_node(node_);
  DXL_LOG_INFO("[timing] on_init: ROS node setup took " << static_cast<int>(node_setup_timer.elapsedMs()) << " ms");

  reboot_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/reboot", [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                         const std_srvs::srv::Trigger::Response::SharedPtr response) {
        // A reboot cuts torque and wipes RAM, so never under a running read and write. After a fault
        // the component is unconfigured, and this is the recovery path. Refused before the bus lock,
        // so a refusal never costs the control loop a cycle.
        response->success = false;
        response->message = "Reboot refused: set the component unconfigured first.";
        if (lifecycle_phase_ != LifecyclePhase::Unconfigured) {
          DXL_LOG_ERROR(response->message);
          return;
        }
        DXL_LOG_INFO("Reboot request received.");
        {
          const std::lock_guard<BusMutex> lock(dynamixel_comm_mutex_);
          // Again under the lock, which on_configure holds until the component is configured
          if (lifecycle_phase_ == LifecyclePhase::Unconfigured) {
            response->success = rebootLocked();
            response->message = response->success ? "Rebooted successfully" : "Failed to reboot";
          }
        }
        if (!response->success) {
          DXL_LOG_ERROR(response->message);
        }
      });

  offset_manager_ = std::make_shared<hector_transmission_interface::AdjustableOffsetManager>(node_);
  // read() and write() map with the offsets, so an offset changes only while the bus is held
  offset_manager_->set_run_in_control_loop([this](const std::function<void()>& adjustment) {
    const std::lock_guard<BusMutex> lock(dynamixel_comm_mutex_);
    adjustment();
    return true;
  });
  // A controller would see its joint jump by the offset change
  offset_manager_->set_may_adjust([this](const std::string& joint) {
    return commandedJoints({joint}).empty() ? std::string() : std::string("a controller commands it");
  });
  led_manager_.init(&joints_, &groups_.led_write, logger_);
  if (!joint_state_publishers_.init(node_, info_.hardware_parameters, joints_, joint_names_)) {
    return false;
  }
  diagnostics_ = std::make_unique<DynamixelDiagnostics>(node_, get_name(), joints_, joint_names_, groups_.state_read,
                                                        groups_.control_write);

  // spin() again after a cancel() that came before it started, until the destructor's stop is seen
  exe_thread_ = std::thread([this] {
    while (!executor_stop_requested_ && rclcpp::ok()) {
      exe_->spin();
    }
    executor_stopped_ = true;
  });
  return true;
}

hardware_interface::CallbackReturn
DynamixelHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_configure from " << previous_state.label());
  // reset internal variables
  first_read_successful_ = false;
  mode_switch_failed_ = false;

  // The whole configure sequence is bus I/O. Hold the comm mutex so services on the executor
  // thread (reboot, offset adjust) cannot interleave packets.
  std::lock_guard<BusMutex> configure_lock(dynamixel_comm_mutex_);

  // A securing pass of an earlier on_error must not shape the next one
  for (auto& [name, joint] : joints_) {
    joint.securing.reset();
  }

  const Stopwatch configure_timer;
  const CommStats stats_at_start = driver_.getCommStats();

  Stopwatch phase_timer;
  if (!driver_.connect()) {
    return hardware_interface::CallbackReturn::FAILURE;
  }
  DXL_LOG_INFO("[timing] Port connect took " << static_cast<int>(phase_timer.elapsedMs()) << " ms");
  checkUsbLatencyTimer();

  phase_timer.reset();
  bool connection_successful = true;
  for (auto& [name, joint] : joints_) {
    const Stopwatch joint_timer;
    const CommStats joint_stats_before = driver_.getCommStats();
    if (!joint.connect()) {
      connection_successful = false;
    }
    joint.reset();
    DXL_LOG_INFO("[timing] Joint '" << name << "' connect took " << static_cast<int>(joint_timer.elapsedMs()) << " ms ("
                                    << driver_.getCommStats().delta(joint_stats_before).toString() << ")");
  }
  if (!connection_successful) {
    DXL_LOG_ERROR("[timing] on_configure FAILED in joint connect phase after "
                  << static_cast<int>(configure_timer.elapsedMs()) << " ms ("
                  << driver_.getCommStats().delta(stats_at_start).toString() << ")");
    return hardware_interface::CallbackReturn::FAILURE;
  }
  DXL_LOG_INFO("[timing] All joint connects took " << static_cast<int>(phase_timer.elapsedMs()) << " ms");

  // Window in control cycles, from "bus_watchdog_cycles". 0 disables the watchdog. An invalid
  // value fails configuration rather than falling back to a window nobody chose.
  phase_timer.reset();
  double bus_watchdog_cycles = 0.0;
  if (!getOptionalParameter(info_.hardware_parameters, "bus_watchdog_cycles", bus_watchdog_cycles,
                            DEFAULT_BUS_WATCHDOG_CYCLES)) {
    DXL_LOG_ERROR("bus_watchdog_cycles is set but is not a finite number.");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  if (!bus_watchdog_.configure(bus_watchdog_cycles, info_.rw_rate)) {
    return hardware_interface::CallbackReturn::FAILURE;
  }
  if (const auto unprotected = bus_watchdog_.unprotectedJoints();
      allow_velocity_without_watchdog_ && !unprotected.empty()) {
    DXL_LOG_WARN("allow_velocity_without_watchdog is set, so " << iterableToString(unprotected)
                                                               << " may run in velocity mode without a bus watchdog. "
                                                               << "They keep moving if the bus goes silent.");
  }

  // The diagnostics timer publishes once a second, so sampling faster only spends control cycles on
  // a value nobody reads.
  health_snapshot_divisor_ =
      info_.rw_rate > 0 ? std::max(1, static_cast<int>(std::lround(info_.rw_rate / HEALTH_SNAPSHOT_HZ))) : 1;
  health_snapshot_counter_ = 0;

  DXL_LOG_INFO("[timing] Bus watchdog setup took " << static_cast<int>(phase_timer.elapsedMs()) << " ms");

  // Set up sync read / write managers. Stats delta: ~0 writes = warm start (indirect address
  // maps already match), many writes = cold start after motor power cycle or changed config.
  phase_timer.reset();
  const CommStats stats_before_managers = driver_.getCommStats();
  if (!groups_.configure(joints_, driver_, configuredStateInterfaceUnion(), bus_watchdog_.capableMotors())) {
    DXL_LOG_ERROR("[timing] on_configure FAILED in sync manager setup after "
                  << static_cast<int>(configure_timer.elapsedMs()) << " ms ("
                  << driver_.getCommStats().delta(stats_at_start).toString() << ")");
    return hardware_interface::CallbackReturn::FAILURE;
  }
  DXL_LOG_INFO("[timing] Sync manager setup (indirect addresses) took "
               << static_cast<int>(phase_timer.elapsedMs()) << " ms ("
               << driver_.getCommStats().delta(stats_before_managers).toString() << ")");

  updateLeds(false);

  // Manifest reflects live EEPROM state — publish after motors are connected and the control
  // tables are loaded.
  phase_timer.reset();
  diagnostics_->publishManifest(node_->now());
  DXL_LOG_INFO("[timing] Manifest publish took " << static_cast<int>(phase_timer.elapsedMs()) << " ms");

  DXL_LOG_INFO("[timing] on_configure took " << static_cast<int>(configure_timer.elapsedMs()) << " ms total ("
                                             << driver_.getCommStats().delta(stats_at_start).toString() << ")");
  lifecycle_phase_ = LifecyclePhase::Inactive;
  return CallbackReturn::SUCCESS;
}

DynamixelHardwareInterface::~DynamixelHardwareInterface()
{
  // Joined before the executor lets go of the node: once cancelled, a node removal rebuilds the
  // wait set on this thread while the spin thread may still evaluate it
  if (exe_thread_.joinable()) {
    executor_stop_requested_ = true;
    // A cancel() that lands before spin() starts is lost, so it is repeated until the thread is out
    while (!executor_stopped_) {
      exe_->cancel();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    exe_thread_.join();
  }
  exe_.reset();
  node_.reset();
}

hardware_interface::CallbackReturn DynamixelHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_cleanup from " << previous_state.label());
  lifecycle_phase_ = LifecyclePhase::Unconfigured;
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_activate from " << previous_state.label());
  const Stopwatch activate_timer;
  // One lock for the whole sequence: goals verified and torque on must not be split
  const std::lock_guard<BusMutex> lock(dynamixel_comm_mutex_);
  const CommStats stats_at_start = driver_.getCommStats();
  Stopwatch phase_timer;

  // Nothing feeds the watchdog during activation, so it stays cleared until the last step.
  if (!bus_watchdog_.clear()) {
    DXL_LOG_WARN("Failed to clear the bus watchdog before activation; a latch may reject goal writes.");
  }

  // Torque comes on only after the mode is read, corrected and a verified safe goal written. An
  // unread mode resets goals to the velocity limit.
  for (auto& [name, joint] : joints_) {
    // Joints needing a switch stay de-energised until their goals are verified further down
    bool mode_read = false;
    for (int attempt = 1; attempt <= CONTROL_MODE_READ_ATTEMPTS && !mode_read; ++attempt) {
      mode_read = joint.readControlMode();
      if (!mode_read && attempt < CONTROL_MODE_READ_ATTEMPTS) {
        DXL_LOG_WARN("Could not read the control mode of joint '" << name << "' (attempt " << attempt << "/"
                                                                  << CONTROL_MODE_READ_ATTEMPTS << ").");
      }
    }
    if (!mode_read) {
      DXL_LOG_ERROR("Could not read the control mode of joint '" << name << "' in " << CONTROL_MODE_READ_ATTEMPTS
                                                                 << " attempts. Aborting activation.");
      return CallbackReturn::ERROR;
    }
    if (!joint.updateControlMode(true)) {
      return CallbackReturn::ERROR;
    }
  }
  DXL_LOG_DEBUG("[timing] on_activate: control mode check took " << static_cast<int>(phase_timer.elapsedMs()) << " ms");

  phase_timer.reset();
  if (!resetGoalStateAndVerify(joint_names_, max_reset_and_verify_retries_)) {
    return CallbackReturn::ERROR;
  }
  DXL_LOG_DEBUG("[timing] on_activate: resetGoalStateAndVerify took " << static_cast<int>(phase_timer.elapsedMs())
                                                                      << " ms");

  phase_timer.reset();
  if (!setTorqueForAll(torque_on_startup_).fullSuccess()) {
    DXL_LOG_ERROR("Failed to set torque on activation to " << (torque_on_startup_ ? "ON" : "OFF"));
    return CallbackReturn::ERROR;
  }
  DXL_LOG_DEBUG("[timing] on_activate: setTorque took " << static_cast<int>(phase_timer.elapsedMs()) << " ms");
  // Armed last: from here on the periodic read/write loop feeds it.
  if (!armWatchdogVerified()) {
    DXL_LOG_ERROR("Failed to arm bus watchdog. Aborting activation.");
    // Energised moments ago, the motors would be left live with nothing to feed the watchdog
    if (torque_on_startup_ && !setTorqueForAll(false).fullSuccess()) {
      DXL_LOG_ERROR("Could not de-energise every motor while aborting activation. Some may still be live.");
    }
    return CallbackReturn::ERROR;
  }
  updateLeds(true);
  diagnostics_->startHealthTimer();
  DXL_LOG_INFO("[timing] on_activate took " << static_cast<int>(activate_timer.elapsedMs()) << " ms total ("
                                            << driver_.getCommStats().delta(stats_at_start).toString() << ")");
  lifecycle_phase_ = LifecyclePhase::Active;
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DynamixelHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_deactivate from " << previous_state.label());
  // Pessimistic from the first instant: executor-thread code must not treat a deactivating
  // interface as active (e.g. re-arm a watchdog nothing will feed).
  lifecycle_phase_ = LifecyclePhase::Inactive;
  diagnostics_->stopHealthTimer();
  const std::lock_guard<BusMutex> lock(dynamixel_comm_mutex_);
  // Torque first, then the watchdog: a de-energised joint cannot run away. A failure is reported
  // at the end rather than returned. The block below stops a joint the torque-off could not reach.
  bool torque_off_ok = true;
  if (torque_off_on_shutdown_ && !setTorqueForAll(false).fullSuccess()) {
    DXL_LOG_ERROR("Failed to set torque OFF on deactivation");
    torque_off_ok = false;
  }

  if (stopFreeRunningJoints(joint_names_)) {
    if (!bus_watchdog_.clear()) {
      DXL_LOG_WARN("Failed to clear the bus watchdog on deactivation.");
    }
  } else if (!bus_watchdog_.arm()) {
    DXL_LOG_ERROR("Could not stop every free-running joint, and could not arm the bus watchdog to stop "
                  "them either. They may keep running at their last commanded speed.");
  } else {
    DXL_LOG_ERROR("Could not transmit the stop for every free-running joint; leaving the bus watchdog "
                  "armed, since it is the only thing left that will halt them.");
  }
  // Kept, the claims would bring back their mode at the next activation with no controller to command it
  for (auto& [name, joint] : joints_) {
    joint.clearActiveCommandInterfaces();
  }
  updateLeds(false);
  return torque_off_ok ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr> DynamixelHardwareInterface::on_export_state_interfaces()
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_export_state_interfaces");
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;

  const std::set<std::string> configured_state_interface_names = configuredStateInterfaceUnion();

  // Create the state interfaces
  for (auto& [name, joint] : joints_) {
    joint.joint_state.current.reserve(configured_state_interface_names.size());
    for (const auto& interface_name : configured_state_interface_names) {
      // NaN until the first read, like the goal values below. Zero is a position, and a service
      // that calibrates against the last reading cannot tell it from one the bus actually gave.
      joint.joint_state.current[interface_name] = std::numeric_limits<double>::quiet_NaN();
      const auto state_interface = std::make_shared<hardware_interface::StateInterface>(joint.name, interface_name);
      joint.state_handles[interface_name] = state_interface;
      state_interfaces.emplace_back(state_interface);
    }

    // mimic joints
    for (auto& [mimic_joint_name, mimic_state] : joint.mimic_joint_states) {
      mimic_state.current.reserve(configured_state_interface_names.size());
      for (const auto& interface_name : configured_state_interface_names) {
        mimic_state.current[interface_name] = std::numeric_limits<double>::quiet_NaN();
        const auto state_interface =
            std::make_shared<hardware_interface::StateInterface>(mimic_joint_name, interface_name);
        mimic_state.state_handles[interface_name] = state_interface;
        state_interfaces.emplace_back(state_interface);
      }
    }
  }

  DXL_LOG_DEBUG("State interfaces: " << iterableToString(configured_state_interface_names));

  // Under the comm mutex: a respawn re-runs this while the offset manager's service handlers may
  // be mapping through the very transmissions configure() rebuilds.
  {
    std::lock_guard<BusMutex> lock(dynamixel_comm_mutex_);
    for (const auto& transmission_info : info_.transmissions) {
      Joint& joint = joints_.at(transmission_info.joints.front().name);  // This should exist
      configureTransmission(transmission_info, configured_state_interface_names, joint.joint_state.current,
                            joint.actuator_state.current, *joint.state_transmission, joint.state_pass_through);
    }
  }

  // register offset manager state interfaces
  for (const auto& [name, joint] : joints_) {
    // try to register only adjustable offset transmissions
    offset_manager_->add_joint_state_interface(
        name, joint.state_transmission,
        [&joint]() { return joint.joint_state.current.at(hardware_interface::HW_IF_POSITION); },
        joint.command_transmission);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr> DynamixelHardwareInterface::on_export_command_interfaces()
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_export_command_interfaces");
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;

  for (auto& [name, joint] : joints_) {
    joint.joint_state.goal.reserve(joint.getAvailableCommandInterfaces().size());
    for (const auto& interface_name : joint.getAvailableCommandInterfaces()) {
      joint.joint_state.goal[interface_name] = 0.0;
      // The handle stays NaN until a controller or a mode switch writes it, so a controller that
      // starts from the last command cannot mistake it for a goal of zero
      const auto command_interface = std::make_shared<hardware_interface::CommandInterface>(joint.name, interface_name);
      joint.command_handles[interface_name] = command_interface;
      command_interfaces.emplace_back(command_interface);
    }
    // Claimed, never read: the claim is the command
    if (joint.exportsTorqueOff()) {
      command_interfaces.emplace_back(
          std::make_shared<hardware_interface::CommandInterface>(joint.name, TORQUE_OFF_INTERFACE));
    }
  }

  // Set up transmission interface, under the comm mutex for the same reason as the state side.
  {
    std::lock_guard<BusMutex> lock(dynamixel_comm_mutex_);
    for (const auto& transmission_info : info_.transmissions) {
      Joint& joint = joints_.at(transmission_info.joints.front().name);  // This should exist
      configureTransmission(transmission_info, joint.getAvailableCommandInterfaces(), joint.joint_state.goal,
                            joint.actuator_state.goal, *joint.command_transmission, joint.command_pass_through);
    }
  }
  return command_interfaces;
}

hardware_interface::return_type
DynamixelHardwareInterface::perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                        const std::vector<std::string>& stop_interfaces)
{
  const Stopwatch mode_switch_timer;
  std::lock_guard<BusMutex> lock(dynamixel_comm_mutex_);

  // Checked again here, because a controller manager may skip prepare. Before anything changes.
  if (const std::string refusal = switchRefusal(start_interfaces, stop_interfaces); !refusal.empty()) {
    DXL_LOG_ERROR("Refusing the command mode switch: " << refusal);
    return hardware_interface::return_type::ERROR;
  }

  // write() sends nothing until the modes and goals agree again
  mode_switch_failed_ = true;
  DXL_LOG_DEBUG("Mode switch: start " << iterableToString(start_interfaces) << ", stop "
                                      << iterableToString(stop_interfaces));

  const std::vector<InterfaceRef> start = parseInterfaces(start_interfaces);
  const std::vector<InterfaceRef> stop = parseInterfaces(stop_interfaces);
  // torque_off is a mode of its own, so it is split from the motion interfaces
  const auto is_motion = [](const InterfaceRef& ref) { return ref.interface != TORQUE_OFF_INTERFACE; };
  const auto is_torque_off = [&](const InterfaceRef& ref) { return !is_motion(ref); };

  // Every joint the switch touches gets its goal reset, except those that keep it. A kept goal
  // still goes into a started interface, so a controller manager that leaves the last command in a
  // released interface cannot hand it to the next controller.
  const auto keeps_goal = [this](const InterfaceRef& ref) {
    return joints_.at(ref.joint).keepsGoalOnControllerChange();
  };
  std::vector<InterfaceRef> touched = start;
  touched.insert(touched.end(), stop.begin(), stop.end());
  const std::vector<std::string> joints_to_reset =
      jointsOf(touched, [&](const InterfaceRef& ref) { return is_motion(ref) && !keeps_goal(ref); });

  const std::vector<std::string> torque_off_start = jointsOf(start, is_torque_off);
  const std::set<std::string> limp(torque_off_start.begin(), torque_off_start.end());

  // The reset below zeroes these anyway. Sent while the watchdog is still armed, so a bus lost
  // after the clear cannot leave them running. A joint going limp gets no goal before its torque off.
  std::vector<std::string> early_stop;
  for (const auto& name : joints_to_reset) {
    if (limp.count(name) == 0) {
      early_stop.push_back(name);
    }
  }
  if (!stopFreeRunningJoints(early_stop)) {
    DXL_LOG_WARN("Could not stop the free-running joints before a control mode switch.");
  }

  // Ahead of the watchdog clear, so the still armed watchdog stops a joint going limp if the bus is lost
  // before it is limp. One packet, since the other motors hear nothing until the clear.
  std::set<Dynamixel*> limp_motors;
  for (const auto& name : torque_off_start) {
    Joint& joint = joints_.at(name);
    joint.torque = false;
    limp_motors.insert(joint.dynamixel.get());
  }
  if (!groups_.torque_write.writeSelected(limp_motors)) {
    DXL_LOG_WARN("Could not transmit the torque off for " << iterableToString(torque_off_start)
                                                          << " before a control mode switch.");
  }

  // No packet reaches the motors while this holds the mutex through its reset+verify rounds, which
  // is long enough to trip an armed watchdog. Cleared here and re-armed on the way out.
  if (!bus_watchdog_.clear()) {
    DXL_LOG_WARN("Failed to clear the bus watchdog before a control mode switch.");
  }
  // write() feeds nothing after a failed switch, so a joint left running is stopped by the watchdog.
  // It only arms, every goal and mode stays as the failed step left it.
  const auto fail = [this] {
    if (lifecycle_phase_ == LifecyclePhase::Active && !bus_watchdog_.arm()) {
      DXL_LOG_ERROR("Could not re-arm the bus watchdog after a failed control mode switch.");
    }
    return hardware_interface::return_type::ERROR;
  };

  // Off first: the operating mode register only accepts writes without torque, so a mode change
  // of a joint that goes limp costs no extra torque drop.
  if (!setTorqueForJoints(false, limp).fullSuccess()) {
    DXL_LOG_ERROR("Could not switch torque off for " << iterableToString(torque_off_start) << ".");
    return fail();
  }

  // Ensure we have current state before switching modes
  if (!first_read_successful_) {
    DXL_LOG_INFO("Performing initial read before controller activation.");
    if (!groups_.state_read.read() || !groups_.state_read.isOk()) {
      DXL_LOG_ERROR("Failed to read current state before controller activation.");
      return fail();
    }
    for (auto& [name, joint] : joints_) {
      if (joint.state_transmission) {
        joint.stateToJoint();
      }
      joint.resetGoalState();
    }
    first_read_successful_ = true;
  }

  for (const auto& name : torque_off_start) {
    joints_.at(name).setTorqueOffClaimed(true);
  }
  // A stopped torque_off leaves the joint limp. Only a starting motion command energises a limp
  // joint, so a hold that only stops interfaces never brings torque back.
  for (const auto& name : jointsOf(stop, is_torque_off)) {
    joints_.at(name).setTorqueOffClaimed(false);
  }
  const std::vector<std::string> energise =
      jointsOf(start, [&](const InterfaceRef& ref) { return is_motion(ref) && !joints_.at(ref.joint).torque_desired; });

  for (const auto& ref : stop) {
    if (is_motion(ref)) {
      joints_.at(ref.joint).removeActiveCommandInterface(ref.interface);
    }
  }
  for (const auto& ref : start) {
    if (is_motion(ref) && !joints_.at(ref.joint).addActiveCommandInterface(ref.interface)) {
      return fail();
    }
  }

  const std::vector<std::string> kept_goal_joints =
      jointsOf(start, [&](const InterfaceRef& ref) { return is_motion(ref) && keeps_goal(ref); });

  // Reset all goal states and verify that the cmds were written correctly
  if (!resetGoalStateAndVerify(joints_to_reset, max_reset_and_verify_retries_)) {
    DXL_LOG_ERROR("Failed to reset goal states during command mode switch.");
    return fail();
  }

  // Write control mode. Every joint is considered, not just the switched ones: a controllerless
  // joint parked in a non-position mode is pulled to its preferred position mode here too.
  std::vector<std::string> joints_with_mode_change;
  for (auto& [name, joint] : joints_) {
    const bool mode_changes = joint.getRequiredControlModeChange().has_value();
    if (!joint.updateControlMode(false)) {
      DXL_LOG_ERROR("Failed to update control mode for joint '" << joint.name << "' during command mode switch.");
      return fail();
    }
    if (mode_changes) {
      joints_with_mode_change.push_back(name);
    }
  }

  // A mode change makes the firmware re-anchor its goals, so they are verified again before the cyclic write
  if (!joints_with_mode_change.empty() &&
      !resetGoalStateAndVerify(joints_with_mode_change, max_reset_and_verify_retries_)) {
    DXL_LOG_ERROR("Failed to re-verify goal states after changing control modes.");
    return fail();
  }
  // On last, against a mode that is set. The torque-on holds each joint where a hand left it.
  if (!setTorqueForJoints(true, {energise.begin(), energise.end()}).fullSuccess()) {
    DXL_LOG_ERROR("Could not switch torque on for " << iterableToString(energise) << ".");
    return fail();
  }
  if (!seedCommandHandles(joints_to_reset) || !seedCommandHandles(joints_with_mode_change) ||
      !seedCommandHandles(kept_goal_joints) || !seedCommandHandles(energise)) {
    return fail();
  }

  // The one arm of the switch, since every caller of setTorqueForJoints() but this one runs while
  // nothing feeds the watchdog. mode_switch_failed_ stays set on failure, so on_error secures the
  // joints. Proven only for energised motors, so a limp joint whose watchdog cannot be read fails nothing.
  std::set<Dynamixel*> energised;
  for (const auto& name : jointsDesiringTorque(true)) {
    energised.insert(joints_.at(name).dynamixel.get());
  }
  if (lifecycle_phase_ == LifecyclePhase::Active && !armWatchdogVerified(&energised)) {
    DXL_LOG_ERROR("Failed to re-arm the bus watchdog after a control mode switch.");
    // Joints just energised would run without comm loss protection, so they go back to limp
    (void) setTorqueForJoints(false, {energise.begin(), energise.end()});
    return hardware_interface::return_type::ERROR;
  }
  mode_switch_failed_ = false;
  DXL_LOG_INFO("[timing] perform_command_mode_switch took " << static_cast<int>(mode_switch_timer.elapsedMs()) << " ms");
  return hardware_interface::return_type::OK;
}

void DynamixelHardwareInterface::checkUsbLatencyTimer()
{
  const std::optional<int> latency_ms = driver_.usbLatencyTimerMs(sysfs_root_);
  if (!latency_ms) {
    DXL_LOG_DEBUG("Port '" << driver_.portName() << "' has no USB latency timer to check.");
  } else if (*latency_ms > USB_LATENCY_TIMER_TARGET_MS) {
    DXL_LOG_WARN("The USB latency timer of port '"
                 << driver_.portName() << "' is " << *latency_ms << " ms, which delays every status packet. Set it to "
                 << USB_LATENCY_TIMER_TARGET_MS << " ms with a udev rule, ATTR{latency_timer}=\""
                 << USB_LATENCY_TIMER_TARGET_MS << "\".");
  }
  diagnostics_->setUsbLatencyTimer(latency_ms);
}

bool DynamixelHardwareInterface::stopFreeRunningJoints(const std::vector<std::string>& joint_names)
{
  // A bare write, not resetGoalStateAndVerify(): that path is gated on the whole bus being
  // healthy, and a hardware error is an ordinary reason to get here.
  std::set<Dynamixel*> free_running;
  for (const auto& name : joint_names) {
    Joint& joint = joints_.at(name);
    // Velocity and current are the free-running modes: the goal is a rate or a force the motor
    // keeps applying, so a torqued joint left in one runs on once nothing commands it. A
    // position-like goal is itself a standstill. The securing allowlist zeroes exactly these two.
    const ControlMode mode = joint.getControlMode();
    if ((mode == VELOCITY || mode == CURRENT) && torqueMayBeLive(joint.torque_observed)) {
      joint.resetGoalState();  // zero velocity, or zero current
      free_running.insert(joint.dynamixel.get());
    }
  }
  if (free_running.empty()) {
    return true;
  }
  DXL_LOG_INFO("Stopping " << free_running.size() << " torqued free-running joint(s).");
  // Judged on the transmit, not a read-back: a lost confirmation would cost the clear for a stop
  // that did land, and a watchdog left armed with nothing feeding it latches.
  return groups_.control_write.writeSelected(free_running);
}

hardware_interface::return_type
DynamixelHardwareInterface::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                        const std::vector<std::string>& stop_interfaces)
{
  if (const std::string refusal = switchRefusal(start_interfaces, stop_interfaces); !refusal.empty()) {
    DXL_LOG_ERROR("Refusing the command mode switch: " << refusal);
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

std::string DynamixelHardwareInterface::switchRefusal(const std::vector<std::string>& start_interfaces,
                                                      const std::vector<std::string>& stop_interfaces) const
{
  // What each touched joint claims once the switch is done
  struct Claims
  {
    std::set<std::string> motion;
    bool torque_off{false};
  };
  std::map<std::string, Claims> after;
  const auto claims_of = [this, &after](const std::string& joint_name) -> Claims& {
    const auto [it, inserted] = after.try_emplace(joint_name);
    if (inserted) {
      const Joint& joint = joints_.at(joint_name);
      it->second.motion.insert(joint.getActiveCommandInterfaces().begin(), joint.getActiveCommandInterfaces().end());
      it->second.torque_off = joint.torqueOffClaimed();
    }
    return it->second;
  };
  const auto apply = [this, &claims_of](const std::vector<std::string>& interfaces, const bool start) -> std::string {
    for (const auto& full_interface_name : interfaces) {
      std::string joint_name;
      std::string interface_name;
      if (!splitFullInterfaceName(full_interface_name, joint_name, interface_name) || joints_.count(joint_name) == 0) {
        return "unknown interface '" + full_interface_name + "'";
      }
      const Joint& joint = joints_.at(joint_name);
      Claims& claims = claims_of(joint_name);
      if (interface_name == TORQUE_OFF_INTERFACE) {
        if (!joint.exportsTorqueOff()) {
          return "joint '" + joint_name + "' exports no torque_off interface";
        }
        claims.torque_off = start;
        continue;
      }
      const auto& available = joint.getAvailableCommandInterfaces();
      if (std::find(available.begin(), available.end(), interface_name) == available.end()) {
        return "unknown interface '" + full_interface_name + "'";
      }
      if (start) {
        claims.motion.insert(interface_name);
      } else {
        claims.motion.erase(interface_name);
      }
    }
    return "";
  };
  if (std::string error = apply(stop_interfaces, false); !error.empty()) {
    return error;
  }
  if (std::string error = apply(start_interfaces, true); !error.empty()) {
    return error;
  }
  for (const auto& [joint_name, claims] : after) {
    if (claims.torque_off && !claims.motion.empty()) {
      return "joint '" + joint_name + "' cannot be torque_off and commanded at once";
    }
    const Joint& joint = joints_.at(joint_name);
    const bool positions = claims.motion.count(hardware_interface::HW_IF_POSITION) != 0;
    const bool velocities = claims.motion.count(hardware_interface::HW_IF_VELOCITY) != 0;
    const bool currents = claims.motion.count(hardware_interface::HW_IF_CURRENT) != 0 ||
                          claims.motion.count(hardware_interface::HW_IF_EFFORT) != 0;
    const bool max_current = claims.motion.count(MAX_CURRENT_INTERFACE) != 0;
    const bool max_velocity = claims.motion.count(MAX_VELOCITY_INTERFACE) != 0;
    if (positions + velocities + currents > 1) {
      return "joint '" + joint_name + "' would be commanded through more than one of position, velocity and " +
             "current. Limit a position or velocity command with max_current or max_velocity instead.";
    }
    if (max_velocity && !positions) {
      return "joint '" + joint_name + "' would have max_velocity without position, the only command it limits";
    }
    if (max_current && !positions && !velocities) {
      return "joint '" + joint_name + "' would have max_current without position or velocity, the commands it limits";
    }
    if (max_current) {
      const ControlMode mode = positions ? joint.getPreferredPositionControlMode() : VELOCITY;
      if (!joint.dynamixel->limitsCurrentIn(mode)) {
        return "joint '" + joint_name + "' would have max_current in " + controlModeToString(mode) +
               " mode, in which its model does not limit the current by Goal Current";
      }
    }
    if (currents) {
      return "joint '" + joint_name + "' would run in current mode, which is not supported until a joint " +
             "without a current command can be held.";
    }
    if (velocities && !allow_velocity_without_watchdog_ && joint.bus_watchdog_ticks <= 0) {
      return "joint '" + joint_name + "' would run in velocity mode without a bus watchdog, so it keeps moving " +
             "if the bus goes silent. Set allow_velocity_without_watchdog to permit it.";
    }
  }
  return "";
}

hardware_interface::CallbackReturn DynamixelHardwareInterface::on_error(const rclcpp_lifecycle::State& previous_state)
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_error from " << previous_state.label());
  // The phase stays ACTIVE for this callback and is lowered on the way out by a scope guard, so no
  // early return can skip it: the securing and LED work below is suppressed once Unconfigured.
  struct PhaseLowering
  {
    std::atomic<LifecyclePhase>& phase;
    ~PhaseLowering()
    {
      phase = LifecyclePhase::Unconfigured;
    }
  } lower_phase_on_exit{lifecycle_phase_};

  // Nothing refreshes the snapshot once Unconfigured, and the timer would keep republishing the
  // last healthy sample. Stop it first, then publish the fault, so it is the last word on the topic.
  {
    std::lock_guard<BusMutex> diagnostics_lock(dynamixel_comm_mutex_);
    diagnostics_->snapshotHealth(get_clock()->now(), allJointsDesireTorque(), mode_switch_failed_,
                                 last_successful_read_time_, longest_bus_silence_ms_, overlap_skipped_cycles_);
  }
  diagnostics_->stopHealthTimer();
  diagnostics_->publishHealth();

  // One lock scope for the diagnosis AND the response: the counters it judges are written by bus
  // I/O on other threads, which the securing pass's own packets must not interleave with either.
  std::lock_guard<BusMutex> lock(dynamixel_comm_mutex_);

  // Whatever brought us here, nothing writes this component's goals any more, so a velocity or
  // current joint would run on. Returning anything but SUCCESS would finalize the component, which
  // set_hardware_component_state cannot bring back, so a failed stop is only logged.
  if (!stopFreeRunningJoints(joint_names_)) {
    DXL_LOG_ERROR("Could not transmit the stop for every free-running joint while handling the error.");
  }

  const bool communication_lost = !groups_.state_read.isOk() || !groups_.control_write.isOk();

  // A failed mode switch can leave joints in a mode their goals were not written for
  if (mode_switch_failed_ && !communication_lost && isHardwareOk()) {
    const SecuringReport report = secureHardware();
    if (!report.fullySecured()) {
      DXL_LOG_ERROR("A control mode switch failed and not every joint could be secured: "
                    << report.securedCount() << " of " << joints_.size() << " verified secured.");
    }
    updateLeds(false);
    return CallbackReturn::SUCCESS;
  }

  if (!communication_lost && isHardwareOk()) {
    updateLeds(false);
    return CallbackReturn::SUCCESS;
  }

  if (communication_lost) {
    // Runs once: a successful on_error transitions the component to UNCONFIGURED. Differs from the
    // hardware-error branch below only in what it logs.
    const SecuringReport report = secureHardware();
    if (report.securedCount() == 0 && report.of(SecuringOutcome::TorqueOffSentUnverified).empty() &&
        report.of(SecuringOutcome::TorqueOffRejected).empty()) {
      DXL_LOG_ERROR("Bus communication lost (" << groups_.state_read.getErrorCount() << " consecutive read, "
                                               << groups_.control_write.getErrorCount()
                                               << " consecutive write errors). No motor is reachable. Check power "
                                                  "and wiring.");
    } else {
      DXL_LOG_ERROR("Communication lost. Securing: "
                    << report.securedCount() << " joint(s) verified secured, "
                    << report.of(SecuringOutcome::TorqueOffRejected).size() << " still energised, "
                    << report.of(SecuringOutcome::TorqueOffSentUnverified).size() << " sent an unverified torque-off, "
                    << report.of(SecuringOutcome::TransmitFailed).size() << " unreachable.");
    }
    return CallbackReturn::SUCCESS;
  }

  // Hardware error, with communication still fine
  DXL_LOG_ERROR("Hardware error on " << describeHardwareErrors() << ".");
  if (!secureHardware().fullySecured()) {
    DXL_LOG_ERROR("Failed to fully secure the hardware during hardware error handling");
  }
  // The component ends unconfigured, with the faulted motors red
  updateLeds(false);

  // SUCCESS leaves the component unconfigured. Recovery is the reboot service, which clears the
  // motor's error, followed by set_hardware_component_state to active. No reboot here: it takes
  // seconds, and a motor error deserves an operator.
  return CallbackReturn::SUCCESS;
}

void DynamixelHardwareInterface::noteOverlapSkip(const char* call)
{
  const uint64_t skipped = ++overlap_skipped_cycles_;
  if (throttleDue(last_overlap_log_, std::chrono::seconds(1))) {
    DXL_LOG_WARN(call << " skipped: a mode switch, a service or a lifecycle transition holds the bus (" << skipped
                      << " so far). The controller manager runs them together, see the README.");
  }
}

hardware_interface::return_type DynamixelHardwareInterface::read(const rclcpp::Time& time,
                                                                 const rclcpp::Duration& /*period*/)
{
  const std::unique_lock<BusMutex> bus(dynamixel_comm_mutex_, std::try_to_lock);
  if (!bus.owns_lock()) {
    noteOverlapSkip("read()");
    return hardware_interface::return_type::OK;
  }

  const bool read_ok_this_cycle = groups_.state_read.read();
  if (!read_ok_this_cycle && throttleDue(last_read_failure_log_, std::chrono::seconds(1))) {
    DXL_LOG_WARN("Sync read failed (" << groups_.state_read.lastError() << "), " << groups_.state_read.getErrorCount()
                                      << " consecutive errors.");
  }

  if (read_ok_this_cycle) {
    last_successful_read_time_ = time;
    // torque_enable rides in the state group, so every cycle re-proves who is energised. Only a
    // successful read may claim that: stale bytes would call a live motor de-energised.
    for (auto& [name, joint] : joints_) {
      joint.commitTorqueReadBack();
    }
  }

  // Bus silence is what trips a watchdog, so it is measured where it happens and reported early.
  const double silence_ms = driver_.takeLongestSilenceMs();
  longest_bus_silence_ms_ = std::max(longest_bus_silence_ms_, silence_ms);
  if (bus_watchdog_.silenceThreatensWindow(silence_ms) && throttleDue(last_bus_silence_log_, std::chrono::seconds(5))) {
    DXL_LOG_WARN("The bus was silent for " << silence_ms << " ms, against a watchdog window of "
                                           << bus_watchdog_.shortestWindowMs()
                                           << " ms. Something stalls the control loop or holds the bus.");
  }

  // Snapshot *before* the error early-returns below, otherwise the diagnostics topic replays the
  // last healthy snapshot exactly when a fault appears.
  if (++health_snapshot_counter_ >= health_snapshot_divisor_) {
    health_snapshot_counter_ = 0;
    diagnostics_->snapshotHealth(time, allJointsDesireTorque(), mode_switch_failed_, last_successful_read_time_,
                                 longest_bus_silence_ms_, overlap_skipped_cycles_);
    longest_bus_silence_ms_ = 0.0;
  }

  // Only return error after exceeding the consecutive error threshold
  if (!groups_.state_read.isOk()) {
    DXL_LOG_ERROR("Read manager lost connection after " << groups_.state_read.getErrorCount() << " consecutive errors");
    return hardware_interface::return_type::ERROR;
  }

  if (!isHardwareOk()) {
    DXL_LOG_ERROR("Hardware error on " << describeHardwareErrors() << ".");
    return hardware_interface::return_type::ERROR;
  }

  // A reboot clears torque_enable, and a blip shorter than the read error threshold shows nothing else.
  // Only while active: activation derives the torque afresh, so an inactive reboot does no harm.
  if (read_ok_this_cycle && lifecycle_phase_ == LifecyclePhase::Active) {
    for (const auto& [name, joint] : joints_) {
      if (joint.torque_desired && joint.torque_observed == TorqueState::Off) {
        DXL_LOG_ERROR("Joint '" << name
                                << "' lost its torque without a request, so its motor rebooted. Was the power cut?");
        return hardware_interface::return_type::ERROR;
      }
    }
  }

  // Only publish on a successful bus read. Otherwise actuator_state.current is stale, and
  // publishing it with a fresh stamp would mislead subscribers.
  if (read_ok_this_cycle) {
    joint_state_publishers_.publishRead(time);
  }

  for (auto& [name, joint] : joints_) {
    // Only on a successful read. A failed one leaves the actuator values frozen, and a transmission
    // that watches them for a 2pi position reset is entitled to assume consecutive calls carry
    // readings that actually happened.
    if (read_ok_this_cycle && joint.state_transmission) {
      joint.stateToJoint();
    }

    // Only after a SUCCESSFUL read: a failed one would derive goals from stale or zeroed state.
    if (!first_read_successful_ && read_ok_this_cycle) {
      joint.resetGoalState();
    }
    joint.updateMimicJointStates();
  }

  if (read_ok_this_cycle) {
    first_read_successful_ = true;
  }

  publishStateHandles();
  return hardware_interface::return_type::OK;
}

bool DynamixelHardwareInterface::throttleDue(std::atomic<std::chrono::steady_clock::rep>& last_logged,
                                             const std::chrono::nanoseconds period)
{
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto last = last_logged.load(std::memory_order_relaxed);
  if (last != 0 && now - last < period.count()) {
    return false;
  }
  last_logged.store(now, std::memory_order_relaxed);
  return true;
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

bool DynamixelHardwareInterface::allJointsDesireTorque() const
{
  return std::all_of(joints_.begin(), joints_.end(), [](const auto& entry) { return entry.second.torque_desired; });
}

bool DynamixelHardwareInterface::isHardwareOk() const
{
  return std::none_of(joints_.begin(), joints_.end(),
                      [](const auto& entry) { return entry.second.dynamixel->hardware_error_status != OK; });
}

std::string DynamixelHardwareInterface::describeHardwareErrors() const
{
  std::string description;
  for (const auto& name : getJointsWithHardwareError()) {
    description += (description.empty() ? "" : ", ") + name + " (" +
                   hardwareErrorToString(joints_.at(name).dynamixel->hardware_error_status) + ")";
  }
  return description;
}

std::vector<std::string> DynamixelHardwareInterface::commandedJoints(const std::vector<std::string>& names) const
{
  std::vector<std::string> commanded;
  for (const auto& name : names) {
    if (!joints_.at(name).getActiveCommandInterfaces().empty()) {
      commanded.push_back(name);
    }
  }
  return commanded;
}

void DynamixelHardwareInterface::publishStateHandles()
{
  for (auto& [name, joint] : joints_) {
    for (auto& [interface_name, handle] : joint.state_handles) {
      if (!handle->set_value(joint.joint_state.current.at(interface_name))) {
        reportBusyHandle(handle->get_name());
      }
    }
    for (auto& [mimic_name, mimic_state] : joint.mimic_joint_states) {
      for (auto& [interface_name, handle] : mimic_state.state_handles) {
        if (!handle->set_value(mimic_state.current.at(interface_name))) {
          reportBusyHandle(handle->get_name());
        }
      }
    }
  }
}

bool DynamixelHardwareInterface::takeCommandHandles(Joint& joint)
{
  bool taken = true;
  for (const auto& interface_name : joint.getActiveCommandInterfaces()) {
    const auto& handle = joint.command_handles.at(interface_name);
    const auto command = handle->get_optional();
    if (!command) {
      reportBusyHandle(handle->get_name());
      taken = false;
      continue;
    }
    joint.joint_state.goal.at(interface_name) = *command;
  }
  return taken;
}

bool DynamixelHardwareInterface::seedCommandHandles(const std::vector<std::string>& joint_names)
{
  for (const auto& name : joint_names) {
    Joint& joint = joints_.at(name);
    for (const auto& interface_name : joint.getActiveCommandInterfaces()) {
      if (!joint.command_handles.at(interface_name)->set_value(joint.joint_state.goal.at(interface_name), true)) {
        DXL_LOG_ERROR("Could not write the reset goal into command interface '" << name << "/" << interface_name
                                                                                << "'.");
        return false;
      }
    }
  }
  return true;
}

void DynamixelHardwareInterface::reportBusyHandle(const std::string& handle_name)
{
  ++busy_handle_count_;
  if (throttleDue(last_busy_handle_log_, std::chrono::seconds(5))) {
    DXL_LOG_WARN("Interface '" << handle_name << "' was locked by another thread when the control loop needed it ("
                               << busy_handle_count_ << " times so far). That cycle used the previous value.");
  }
}

hardware_interface::return_type DynamixelHardwareInterface::write(const rclcpp::Time& time,
                                                                  const rclcpp::Duration& /*period*/)
{
  const std::unique_lock<BusMutex> bus(dynamixel_comm_mutex_, std::try_to_lock);
  if (!bus.owns_lock()) {
    noteOverlapSkip("write()");
    return hardware_interface::return_type::OK;
  }

  if (mode_switch_failed_) {
    DXL_LOG_ERROR("A control mode switch failed, not writing commands.");
    return hardware_interface::return_type::ERROR;
  }

  for (auto& [name, joint] : joints_) {
    const bool command_taken = takeCommandHandles(joint);
    if (joint.command_transmission) {
      joint.goalToActuator();
    }
    joint.holdOnMissingCommand(command_taken);
    joint.limitCaps();
  }

  if (!first_read_successful_) {
    return hardware_interface::return_type::OK;
  }

  // A latch (-1) drops every goal and a 0 is an arm that never landed, so both are repaired alike
  if (bus_watchdog_.armIntended() && !bus_watchdog_.readsArmed()) {
    // Throttled together: this runs at the control rate for as long as the cause is there.
    const bool log_due = throttleDue(last_watchdog_trip_log_, std::chrono::seconds(5));
    if (log_due) {
      const std::vector<std::string> tripped = bus_watchdog_.trippedJoints();
      DXL_LOG_WARN("Bus watchdog is not armed on joint(s) "
                   << iterableToString(bus_watchdog_.unarmedJoints())
                   << (tripped.empty() ? "" : ", tripped on " + iterableToString(tripped))
                   << "; clearing and re-arming. If this repeats, goal writes are not reaching the motors "
                      "within the window: raise bus_watchdog_cycles, or find what stalls the control loop.");
    }
    // Not a chained ||, a failed clear must not skip the arm that restores the intent guarding this repair
    const bool cleared = bus_watchdog_.clear();
    const bool armed = bus_watchdog_.arm();
    if ((!cleared || !armed) && log_due) {
      DXL_LOG_ERROR("Failed to transmit the bus watchdog clear/re-arm. A latched motor keeps "
                    "rejecting goal writes, and an unarmed one keeps taking them unprotected.");
    }
  }

  const bool write_ok_this_cycle = groups_.control_write.write();
  if (!write_ok_this_cycle && throttleDue(last_write_failure_log_, std::chrono::seconds(1))) {
    DXL_LOG_WARN("Sync write failed (" << groups_.control_write.lastError() << "), "
                                       << groups_.control_write.getErrorCount() << " consecutive errors.");
  }

  if (!groups_.control_write.isOk()) {
    DXL_LOG_ERROR("Write manager lost connection after " << groups_.control_write.getErrorCount()
                                                         << " consecutive errors");
    return hardware_interface::return_type::ERROR;
  }

  joint_state_publishers_.publishGoal(time);
  if (write_ok_this_cycle) {
    joint_state_publishers_.publishWrite(time);
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
      DXL_LOG_ERROR("Transmission '" << transmission_info.name << "' of type '" << transmission_info.type
                                     << "' not supported.");
      return false;
    }
    // Only exactly one joint and actuator per transmission
    if (transmission_info.joints.size() != 1 || transmission_info.actuators.size() != 1) {
      DXL_LOG_ERROR("Only transmissions with exactly one joint and actuator are supported");
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
        // One offset per joint: the state side detects jumps and persists, the command side maps.
        auto pair = adjustable_offset_transmission_loader.loadPair(transmission_info);
        state_transmission = std::move(pair.state);
        command_transmission = std::move(pair.command);
      }
    }
    catch (const transmission_interface::TransmissionInterfaceException& exc) {
      DXL_LOG_ERROR("Error while loading " << transmission_info.name << ": " << exc.what());
      return false;
    }
    // A loader reports a configuration it cannot read by returning null, not by throwing. The
    // interfaces are exported through these transmissions, so a null one is dereferenced there.
    if (!state_transmission || !command_transmission) {
      DXL_LOG_ERROR("Transmission '" << transmission_info.name
                                     << "' could not be loaded. See the loader's error above.");
      return false;
    }

    const auto joint = joints_.find(transmission_info.joints.front().name);
    if (joint == joints_.end()) {
      DXL_LOG_ERROR("Unknown joint '" << transmission_info.joints.front().name << "' in transmission interface");
      return false;
    }
    // Passed through unscaled, a joint velocity cap would land on the motor off by the reduction
    const auto& declared = joint->second.getAvailableCommandInterfaces();
    if (std::find(declared.begin(), declared.end(), MAX_VELOCITY_INTERFACE) != declared.end()) {
      DXL_LOG_ERROR("Joint '" << joint->first << "' declares max_velocity behind a transmission, which cannot map it.");
      return false;
    }
    joint->second.state_transmission = state_transmission;
    joint->second.command_transmission = command_transmission;
  }
  return true;
}

std::set<std::string> DynamixelHardwareInterface::configuredStateInterfaceUnion() const
{
  std::set<std::string> names;
  for (const auto& [name, joint] : joints_) {
    names.insert(joint.getAvailableStateInterfaces().begin(), joint.getAvailableStateInterfaces().end());
  }
  return names;
}

bool DynamixelHardwareInterface::armWatchdogVerified(const std::set<Dynamixel*>* targets)
{
  // A dropped status packet is ordinary on this bus, which is why ping retries three times and the
  // goal reset five. Failing on the first one costs far more than the cause warrants: it rolls
  // back a torque-on every motor has already confirmed, or aborts an activation the spawner then
  // repeats over the whole bus.
  for (int attempt = 1; attempt <= watchdog_arm_retries_; ++attempt) {
    if (armWatchdogVerifiedOnce(targets)) {
      return true;
    }
    DXL_LOG_WARN("Could not arm and verify the bus watchdog (attempt " << attempt << " of " << watchdog_arm_retries_
                                                                       << ").");
  }
  DXL_LOG_ERROR("Failed to arm and verify the bus watchdog after " << watchdog_arm_retries_ << " attempts.");
  return false;
}

bool DynamixelHardwareInterface::armWatchdogVerifiedOnce(const std::set<Dynamixel*>* targets)
{
  // The clear and the arm stay whole-bus: both are one packet for every capable motor, and leaving
  // one out would leave it unprotected. Only the read-back is scoped, so a motor the caller is not
  // answering for cannot fail an arm the targets confirmed. That matches the goal reset, which
  // reads only its targets for the same reason.
  const auto readBack = [this, targets]() {
    return targets == nullptr ? groups_.state_read.read() : groups_.state_read.readSelected(*targets);
  };

  // Both halves are needed: a sync write carries no status, and an already-armed motor reads the
  // right value whether or not this write reached it. So the register must be seen at zero first.
  if (!bus_watchdog_.clear() || !readBack()) {
    DXL_LOG_WARN("Could not clear the bus watchdog before arming it.");
    return false;
  }
  const std::vector<std::string> uncleared = bus_watchdog_.unclearedJoints(targets);
  if (!uncleared.empty()) {
    DXL_LOG_WARN("Bus watchdog of joint(s) " << iterableToString(uncleared)
                                             << " would not clear, so an arm cannot be proven to land.");
    return false;
  }
  if (!bus_watchdog_.arm() || !readBack()) {
    DXL_LOG_WARN("Could not read back the bus watchdog after arming it.");
    return false;
  }
  const std::vector<std::string> unarmed = bus_watchdog_.unarmedJoints(targets);
  if (!unarmed.empty()) {
    DXL_LOG_WARN("Bus watchdog did not arm for joint(s) " << iterableToString(unarmed)
                                                          << "; their goal writes would run unprotected.");
    return false;
  }
  return true;
}

void DynamixelHardwareInterface::updateLeds()
{
  // lifecycle_phase_, not lifecycle_state_: this runs on the executor thread (services)
  // where the base class's plain state member races the controller manager thread's transitions.
  updateLeds(lifecycle_phase_.load() == LifecyclePhase::Active);
}

void DynamixelHardwareInterface::updateLeds(const bool active)
{
  led_manager_.update(active, getJointsWithHardwareError());
}
}  // namespace dynamixel_ros_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_ros_control::DynamixelHardwareInterface, hardware_interface::SystemInterface)
