#include "dynamixel_ros_control/dynamixel_diagnostics.hpp"

#include "dynamixel_ros_control/diagnostic_state.hpp"
#include "dynamixel_ros_control/log.hpp"

#include <array>
#include <optional>
#include <sstream>
#include <utility>

namespace dynamixel_ros_control {

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;

namespace {

// Sync managers default to error_threshold_=25; not exposed via a public getter. If/when the
// threshold becomes configurable, expose it through SyncRead/WriteManager and read it here.
constexpr unsigned int DEFAULT_ERROR_THRESHOLD = 25;
constexpr uint8_t TARGET_POINTER_DUMP_ID = 18;

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

std::string joinAddresses(const std::vector<uint16_t>& addresses)
{
  std::ostringstream ss;
  for (size_t i = 0; i < addresses.size(); ++i) {
    if (i > 0) {
      ss << ",";
    }
    ss << addresses[i];
  }
  return ss.str();
}

std::vector<uint16_t> expectedAddresses(const Dynamixel& dxl, const std::string& register_name,
                                        const uint8_t data_length)
{
  std::vector<uint16_t> addresses;
  const auto& item = dxl.getItem(register_name);
  addresses.reserve(data_length);
  for (uint8_t i = 0; i < data_length; ++i) {
    addresses.push_back(item.address() + i);
  }
  return addresses;
}

template <typename DebugEntry>
void logIndirectPointerEntries(const Dynamixel& dxl, const std::string& joint_name, const std::string& phase,
                               const char* path, const std::vector<DebugEntry>& entries)
{
  for (const auto& entry : entries) {
    std::vector<uint16_t> actual_addresses;
    uint16_t indirect_address = 0;
    uint16_t indirect_data_address = 0;
    if (!dxl.readIndirectAddressTargets(entry.indirect_index, entry.data_length, indirect_address,
                                        indirect_data_address, actual_addresses)) {
      DXL_LOG_WARN("[INDIRECT_PTR:" << phase << "] joint '" << joint_name << "' id=" << dxl.getIdInt() << " path="
                                    << path << " reg=" << entry.register_name << " failed to read raw pointer window.");
      continue;
    }

    const auto expected = expectedAddresses(dxl, entry.register_name, entry.data_length);
    DXL_LOG_DEBUG("[INDIRECT_PTR:" << phase << "] joint '" << joint_name << "' id=" << dxl.getIdInt()
                                   << " path=" << path << " reg=" << entry.register_name
                                   << " idx=" << entry.indirect_index << " ptr_addr=" << indirect_address
                                   << " data_addr=" << indirect_data_address << " expected=[" << joinAddresses(expected)
                                   << "] actual=[" << joinAddresses(actual_addresses) << "]");
  }
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
  // `~/diagnostics` (volatile, 1 Hz) is the conventional diagnostic topic name. We keep the
  // private namespace so multi-robot deployments (each robot in its own namespace) don't
  // collide on a single global /diagnostics topic; a per-robot aggregator can still pick it up.
  rt_health_pub_ = std::make_shared<realtime_tools::RealtimePublisher<DiagnosticArray>>(
      node_->create_publisher<DiagnosticArray>("~/diagnostics", rclcpp::QoS(10)));

  // Size + name the per-joint snapshot slots once. snapshotHealth() only updates the runtime
  // fields after this; joint_name is never reassigned, so the snapshot path stays alloc-free
  // regardless of name length / SSO threshold.
  snapshot_.joints.resize(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    snapshot_.joints[i].joint_name = joint_names_[i];
  }
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

  DiagnosticStatus bus;
  bus.level = DiagnosticStatus::OK;
  bus.name = hw_iface_name_ + "/bus";
  bus.message = "Manifest";
  bus.hardware_id = hw_iface_name_;
  bus.values.push_back(kv("hardware_interface_name", hw_iface_name_));
  bus.values.push_back(kv("num_joints", std::to_string(joint_names_.size())));
  msg.status.push_back(bus);

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

    // Live readback. Diagnostics publishes operating_mode_desired (cached intent) — disagreement
    // between the two implies a mode-switch write silently failed.
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

    // Limits/config, homing offset, motion-profile setpoints, and control gains. add_int_if() skips
    // any register a model does not declare (e.g. older PROExt/RH/H42-20-S300-R lack some gains).
    constexpr std::array kManifestRegisters{"current_limit",        "velocity_limit",      "acceleration_limit",
                                            "min_position_limit",   "max_position_limit",  "return_delay_time",
                                            "bus_watchdog",         "homing_offset",       "profile_acceleration",
                                            "profile_velocity",     "velocity_i_gain",     "velocity_p_gain",
                                            "position_d_gain",      "position_i_gain",     "position_p_gain",
                                            "feedforward_2nd_gain", "feedforward_1st_gain"};
    for (const char* reg : kManifestRegisters) {
      add_int_if(reg);
    }

    js.values.push_back(kv("command_interfaces", join(joint.getAvailableCommandInterfaces())));
    js.values.push_back(kv("state_interfaces", join(joint.getAvailableStateInterfaces())));

    msg.status.push_back(js);
  }

  rt_manifest_pub_->unlockAndPublish();
}

void DynamixelDiagnostics::logManifest(const std::string& phase) const
{
  static const std::vector<std::string> kReadRegisters{DXL_REGISTER_POSITION, DXL_REGISTER_VELOCITY,
                                                       DXL_REGISTER_EFFORT, DXL_REGISTER_HARDWARE_ERROR};
  static const std::vector<std::string> kWriteRegisters{DXL_REGISTER_CMD_POSITION, DXL_REGISTER_CMD_VELOCITY,
                                                        DXL_REGISTER_CMD_EFFORT, DXL_REGISTER_CMD_TORQUE};
  // Direct register reads (NOT via the indirect-address sync window). If the indirect pointers
  // are stale/misaligned, these direct values will disagree with the sync-read values logged the
  // same cycle — that disagreement is the smoking gun for indirect-address corruption.
  for (const auto& joint_name : joint_names_) {
    const Dynamixel& dxl = *joints_.at(joint_name).dynamixel;
    std::ostringstream ss;
    ss << "[MANIFEST:" << phase << "] joint '" << joint_name << "' id=" << dxl.getIdInt()
       << " model=" << dxl.getModelNumber();

    auto add = [&](const char* label, const char* reg) {
      if (auto v = readInt(dxl, reg)) {
        ss << " " << label << "=" << *v;
      }
    };
    if (auto v = readInt(dxl, DXL_REGISTER_CONTROL_MODE)) {
      ss << " operating_mode=" << *v;
    }
    // Direct reads of exactly the registers implicated in the gripper sweep: where the motor
    // thinks it is, where it is told to go, the homing reference, and the position clamp.
    add("present_position", "present_position");
    add("goal_position", "goal_position");
    add("present_velocity", "present_velocity");
    add("goal_velocity", "goal_velocity");
    add("present_current", "present_current");
    add("goal_current", "goal_current");
    add("homing_offset", "homing_offset");
    add("min_position_limit", "min_position_limit");
    add("max_position_limit", "max_position_limit");
    add("torque_enable", "torque_enable");
    DXL_LOG_DEBUG(ss.str());

    if (dxl.getId() != TARGET_POINTER_DUMP_ID) {
      continue;
    }

    logIndirectPointerEntries(dxl, joint_name, phase, "read",
                              read_manager_.getIndirectDebugEntries(dxl, kReadRegisters));
    logIndirectPointerEntries(dxl, joint_name, phase, "write",
                              write_manager_.getIndirectDebugEntries(dxl, kWriteRegisters));
  }
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
    // The consumer holds the lock; the timer will publish the previous snapshot. Fine.
    return;
  }

  snapshot_.stamp = stamp;
  snapshot_.last_successful_read = last_successful_read;
  snapshot_.e_stop_active = e_stop_active;
  snapshot_.torque_globally_desired = torque_globally_desired;
  snapshot_.mode_switch_failed = mode_switch_failed;
  snapshot_.read_consecutive_errors = read_manager_.getErrorCount();
  snapshot_.write_consecutive_errors = write_manager_.getErrorCount();

  // Slots and joint_name are populated once in the ctor — only update runtime fields here.
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto& joint = joints_.at(joint_names_[i]);
    auto& js = snapshot_.joints[i];
    js.motor_id = joint.dynamixel->getId();
    js.torque_desired = joint.torque;
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

  // Bus level is the max severity of: bus signals (e-stop, mode-switch fail, error counters)
  // and any per-joint hardware fault. Otherwise the rollup row can read OK while individual
  // joints are publishing ERROR — misleading any consumer that doesn't iterate joint statuses.
  uint8_t bus_level = DiagnosticStatus::OK;
  if (snap.read_consecutive_errors >= DEFAULT_ERROR_THRESHOLD ||
      snap.write_consecutive_errors >= DEFAULT_ERROR_THRESHOLD || snap.mode_switch_failed) {
    bus_level = DiagnosticStatus::ERROR;
  } else if (snap.e_stop_active || snap.read_consecutive_errors > 0 || snap.write_consecutive_errors > 0) {
    bus_level = DiagnosticStatus::WARN;
  }
  for (const auto& js : snap.joints) {
    if (js.hardware_error_status != 0) {
      bus_level = DiagnosticStatus::ERROR;
      break;
    }
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
      s.message = hardwareErrorToString(js.hardware_error_status);
    } else {
      s.level = DiagnosticStatus::OK;
      s.message = "OK";
    }
    s.values.push_back(kv("motor_id", std::to_string(static_cast<unsigned>(js.motor_id))));
    s.values.push_back(kv("torque_desired", js.torque_desired ? "true" : "false"));
    s.values.push_back(kv("hardware_error_status", std::to_string(js.hardware_error_status)));
    s.values.push_back(kv("hardware_error_decoded", hardwareErrorToString(js.hardware_error_status)));
    s.values.push_back(kv("operating_mode_desired", controlModeToString(js.operating_mode)));
    msg.status.push_back(s);
  }

  rt_health_pub_->unlockAndPublish();
}

}  // namespace dynamixel_ros_control
