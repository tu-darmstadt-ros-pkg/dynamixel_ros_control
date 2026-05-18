#include "dynamixel_ros_control/dynamixel_diagnostics.hpp"

#include "dynamixel_ros_control/diagnostic_state.hpp"
#include "dynamixel_ros_control/log.hpp"

#include <optional>
#include <utility>

namespace dynamixel_ros_control {

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;

namespace {

// Sync managers default to error_threshold_=25; not exposed via a public getter. If/when the
// threshold becomes configurable, expose it through SyncRead/WriteManager and read it here.
constexpr unsigned int DEFAULT_ERROR_THRESHOLD = 25;

KeyValue kv(std::string key, std::string value)
{
  KeyValue p;
  p.key = std::move(key);
  p.value = std::move(value);
  return p;
}

// Read a register by name as a raw int32. Returns std::nullopt if unavailable or the read fails.
// Manifest construction only — not RT-safe.
std::optional<int32_t> readInt(const Dynamixel& dxl, const std::string& register_name)
{
  if (!dxl.registerAvailable(register_name)) {
    return std::nullopt;
  }
  int32_t value = 0;
  if (!dxl.readRegister(register_name, value)) {
    return std::nullopt;
  }
  return value;
}

std::string join(const std::vector<std::string>& strs, const std::string& sep = ",")
{
  std::string out;
  for (size_t i = 0; i < strs.size(); ++i) {
    if (i > 0)
      out += sep;
    out += strs[i];
  }
  return out;
}

}  // namespace

DynamixelDiagnostics::DynamixelDiagnostics(rclcpp::Node::SharedPtr node, std::string hw_iface_name,
                                           const std::unordered_map<std::string, Joint>& joints,
                                           const std::vector<std::string>& joint_names, DynamixelDriver& driver,
                                           const SyncReadManager& read_manager, const SyncWriteManager& write_manager)
    : node_(std::move(node)),
      hw_iface_name_(std::move(hw_iface_name)),
      joints_(joints),
      joint_names_(joint_names),
      driver_(driver),
      read_manager_(read_manager),
      write_manager_(write_manager)
{
  rt_manifest_pub_ = std::make_shared<realtime_tools::RealtimePublisher<DiagnosticArray>>(
      node_->create_publisher<DiagnosticArray>("~/manifest", rclcpp::QoS(1).transient_local()));
  rt_health_pub_ = std::make_shared<realtime_tools::RealtimePublisher<DiagnosticArray>>(
      node_->create_publisher<DiagnosticArray>("~/health", rclcpp::QoS(10)));
}

void DynamixelDiagnostics::publishManifest(const rclcpp::Time& stamp)
{
  if (!rt_manifest_pub_->trylock()) {
    DXL_LOG_WARN("Manifest publisher busy; skipping publish.");
    return;
  }
  auto& msg = rt_manifest_pub_->msg_;
  msg.header.stamp = stamp;
  msg.status.clear();

  // Bus-level status.
  DiagnosticStatus bus;
  bus.level = DiagnosticStatus::OK;
  bus.name = hw_iface_name_ + "/bus";
  bus.message = "Manifest";
  bus.hardware_id = hw_iface_name_;
  bus.values.push_back(kv("hardware_interface_name", hw_iface_name_));
  bus.values.push_back(kv("num_joints", std::to_string(joint_names_.size())));
  msg.status.push_back(bus);

  // Per joint: read live EEPROM state so the manifest reflects what's actually on the motor.
  for (const auto& joint_name : joint_names_) {
    const auto& joint = joints_.at(joint_name);
    const Dynamixel& dxl = *joint.dynamixel;

    DiagnosticStatus js;
    js.level = DiagnosticStatus::OK;
    js.name = hw_iface_name_ + "/" + joint_name;
    js.message = "Manifest";
    js.hardware_id = "id=" + std::to_string(dxl.getIdInt()) + " model=" + std::to_string(dxl.getModelNumber());
    js.values.push_back(kv("motor_id", std::to_string(dxl.getIdInt())));
    js.values.push_back(kv("model_number", std::to_string(dxl.getModelNumber())));

    if (auto v = readInt(dxl, "version_of_firmware")) {
      js.values.push_back(kv("firmware_version", std::to_string(*v)));
    } else {
      js.values.push_back(kv("firmware_version", "unknown"));
    }

    if (auto v = readInt(dxl, DXL_REGISTER_CONTROL_MODE)) {
      js.values.push_back(kv("operating_mode", controlModeToString(static_cast<ControlMode>(*v))));
      js.values.push_back(kv("operating_mode_raw", std::to_string(*v)));
    }
    if (auto v = readInt(dxl, "drive_mode")) {
      js.values.push_back(kv("drive_mode", std::to_string(*v)));
    }

    auto add_int_if = [&](const char* reg) {
      if (auto v = readInt(dxl, reg)) {
        js.values.push_back(kv(reg, std::to_string(*v)));
      }
    };
    add_int_if("current_limit");
    add_int_if("velocity_limit");
    add_int_if("min_position_limit");
    add_int_if("max_position_limit");
    add_int_if("return_delay_time");
    add_int_if("bus_watchdog");

    js.values.push_back(kv("command_interfaces", join(joint.getAvailableCommandInterfaces())));
    js.values.push_back(kv("state_interfaces", join(joint.getAvailableStateInterfaces())));

    msg.status.push_back(js);
  }

  rt_manifest_pub_->unlockAndPublish();
}

void DynamixelDiagnostics::startHealthTimer()
{
  if (health_timer_) {
    return;
  }
  health_timer_ = node_->create_wall_timer(HEALTH_PERIOD, [this]() { publishHealth(); });
}

void DynamixelDiagnostics::stopHealthTimer()
{
  if (health_timer_) {
    health_timer_->cancel();
    health_timer_.reset();
  }
}

void DynamixelDiagnostics::snapshotHealth(const rclcpp::Time& stamp, bool e_stop_active, bool torque_globally_desired,
                                          bool mode_switch_failed, const rclcpp::Time& last_successful_read)
{
  std::unique_lock<std::mutex> lock(snapshot_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    // The 5 Hz consumer holds the lock; the timer will publish the previous snapshot. Fine.
    return;
  }

  snapshot_.stamp = stamp;
  snapshot_.last_successful_read = last_successful_read;
  snapshot_.e_stop_active = e_stop_active;
  snapshot_.torque_globally_desired = torque_globally_desired;
  snapshot_.mode_switch_failed = mode_switch_failed;
  snapshot_.read_consecutive_errors = read_manager_.getErrorCount();
  snapshot_.write_consecutive_errors = write_manager_.getErrorCount();

  // First call sizes the vector; subsequent calls reuse the existing slots (no heap traffic).
  if (snapshot_.joints.size() != joint_names_.size()) {
    snapshot_.joints.resize(joint_names_.size());
  }
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto& joint = joints_.at(joint_names_[i]);
    auto& js = snapshot_.joints[i];
    js.joint_name = joint_names_[i];
    js.motor_id = joint.dynamixel->getId();
    js.torqued = joint.torque;
    js.hardware_error_status = joint.dynamixel->hardware_error_status;
    js.operating_mode = joint.getControlMode();
  }

  snapshot_valid_ = true;
}

void DynamixelDiagnostics::publishHealth()
{
  if (!snapshot_valid_.load()) {
    return;
  }
  HealthSnapshot snap;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snap = snapshot_;
  }

  if (!rt_health_pub_->trylock()) {
    return;
  }
  auto& msg = rt_health_pub_->msg_;
  msg.header.stamp = snap.stamp;
  msg.status.clear();

  // Aggregate level computation. Joint-level errors -> ERROR; thresholds exceeded -> ERROR;
  // e-stop or non-zero error counts below threshold -> WARN; else OK.
  uint8_t bus_level = DiagnosticStatus::OK;
  if (snap.read_consecutive_errors >= DEFAULT_ERROR_THRESHOLD ||
      snap.write_consecutive_errors >= DEFAULT_ERROR_THRESHOLD) {
    bus_level = DiagnosticStatus::ERROR;
  } else if (snap.e_stop_active || snap.read_consecutive_errors > 0 || snap.write_consecutive_errors > 0) {
    bus_level = DiagnosticStatus::WARN;
  }

  DiagnosticStatus bus;
  bus.level = bus_level;
  bus.name = hw_iface_name_ + "/bus";
  bus.message = (bus_level == DiagnosticStatus::OK)   ? "OK" :
                (bus_level == DiagnosticStatus::WARN) ? "Degraded" :
                                                        "Errored";
  bus.hardware_id = hw_iface_name_;
  bus.values.push_back(kv("e_stop_active", snap.e_stop_active ? "true" : "false"));
  bus.values.push_back(kv("torque_globally_desired", snap.torque_globally_desired ? "true" : "false"));
  bus.values.push_back(kv("mode_switch_failed", snap.mode_switch_failed ? "true" : "false"));
  bus.values.push_back(kv("read_consecutive_errors", std::to_string(snap.read_consecutive_errors)));
  bus.values.push_back(kv("read_error_threshold", std::to_string(DEFAULT_ERROR_THRESHOLD)));
  bus.values.push_back(kv("write_consecutive_errors", std::to_string(snap.write_consecutive_errors)));
  bus.values.push_back(kv("write_error_threshold", std::to_string(DEFAULT_ERROR_THRESHOLD)));
  bus.values.push_back(kv("last_successful_read", std::to_string(snap.last_successful_read.seconds())));
  msg.status.push_back(bus);

  for (const auto& js : snap.joints) {
    DiagnosticStatus s;
    s.name = hw_iface_name_ + "/" + js.joint_name;
    s.hardware_id = "id=" + std::to_string(static_cast<unsigned>(js.motor_id));
    if (js.hardware_error_status != 0) {
      s.level = DiagnosticStatus::ERROR;
      s.message = DiagnosticState::hardwareErrorToString(js.hardware_error_status);
    } else {
      s.level = DiagnosticStatus::OK;
      s.message = "OK";
    }
    s.values.push_back(kv("motor_id", std::to_string(static_cast<unsigned>(js.motor_id))));
    s.values.push_back(kv("torqued", js.torqued ? "true" : "false"));
    s.values.push_back(kv("hardware_error_status", std::to_string(js.hardware_error_status)));
    s.values.push_back(kv("hardware_error_decoded", DiagnosticState::hardwareErrorToString(js.hardware_error_status)));
    s.values.push_back(kv("operating_mode", controlModeToString(js.operating_mode)));
    msg.status.push_back(s);
  }

  rt_health_pub_->unlockAndPublish();
}

}  // namespace dynamixel_ros_control
