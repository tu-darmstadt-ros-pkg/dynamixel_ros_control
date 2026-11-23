#include "dynamixel_ros_control/log.hpp"

#include <algorithm>
#include <limits>
#include <optional>
#include <tuple>

#include <dynamixel_ros_control/sync_groups.hpp>

namespace dynamixel_ros_control {

namespace {

/// @brief The first pair of interface names in @p interface_names that resolve to one register.
///
/// `effort` and `current` both read present_current, because Dynamixels measure current and never
/// torque. A group would then carry two entries per motor for one register, and its count check
/// would blame the group rather than the robot description.
template <typename Names, typename Resolver>
std::optional<std::tuple<std::string, std::string, std::string>> findRegisterCollision(const Names& interface_names,
                                                                                       const Resolver& to_register)
{
  std::unordered_map<std::string, std::string> claimed_by;
  for (const auto& interface_name : interface_names) {
    std::string register_name = to_register(interface_name);
    auto [it, inserted] = claimed_by.try_emplace(register_name, interface_name);
    if (!inserted) {
      return std::make_tuple(it->second, interface_name, std::move(register_name));
    }
  }
  return std::nullopt;
}

}  // namespace

bool SyncGroups::configure(std::unordered_map<std::string, Joint>& joints, DynamixelDriver& driver,
                           const std::set<std::string>& state_interfaces, const std::set<Dynamixel*>& watchdog_capable)
{
  joints_ = &joints;
  driver_ = &driver;
  return setUpStateRead(state_interfaces) && setUpTorqueWrite() && setUpControlWrite() && setUpGoalRead() &&
         setUpLedWrite() && setUpWatchdogWrite(watchdog_capable);
}

bool SyncGroups::rewriteIndirectAddresses(const std::set<Dynamixel*>& motors)
{
  return state_read.rewriteIndirectAddresses(motors) && goal_read.rewriteIndirectAddresses(motors) &&
         control_write.rewriteIndirectAddresses(motors) && torque_write.rewriteIndirectAddresses(motors) &&
         watchdog_write.rewriteIndirectAddresses(motors) && led_write.rewriteIndirectAddresses(motors);
}

bool SyncGroups::initReadGroup(SyncReadManager& group, const char* label, const RegisterCollector& collect)
{
  group = SyncReadManager();
  std::unordered_map<std::string, DxlValueMappingList> mappings;
  for (auto& [name, joint] : *joints_) {
    collect(joint, mappings);
    // The group needs one value per motor, so a joint without a register in it stays out
    Dynamixel* dxl = joint.dynamixel.get();
    if (std::any_of(mappings.begin(), mappings.end(), [dxl](const auto& entry) {
          return std::any_of(entry.second.begin(), entry.second.end(),
                             [dxl](const auto& mapping) { return mapping.first == dxl; });
        })) {
      group.addDynamixel(dxl);
    }
  }
  for (const auto& [register_name, values] : mappings) {
    // Only some models have bus_watchdog. torque_enable is one byte and universal, so it fills the
    // slot on the others and keeps the group's address range uniform.
    const std::string padding =
        register_name == DXL_REGISTER_BUS_WATCHDOG ? std::string(DXL_REGISTER_CMD_TORQUE) : std::string();
    if (!group.addRegister(register_name, values, padding)) {
      DXL_LOG_ERROR("Failed to add register '" << register_name << "' to the " << label << " group.");
      return false;
    }
  }
  return group.init(*driver_);
}

bool SyncGroups::setUpStateRead(const std::set<std::string>& state_interfaces)
{
  // Every motor is read for every state interface, so two names meaning one register collide even
  // when different joints declare them
  if (const auto collision = findRegisterCollision(state_interfaces, [this](const std::string& interface_name) {
        return joints_->begin()->second.stateInterfaceToRegisterName(interface_name);
      })) {
    const auto& [first, second, register_name] = *collision;
    DXL_LOG_ERROR("The state interfaces '" << first << "' and '" << second << "' both read register '" << register_name
                                           << "'. Every motor on this hardware component is read for the union of "
                                              "every joint's state interfaces, so declare only one of them.");
    return false;
  }
  // torque_enable and bus_watchdog ride along, because both are needed every cycle
  return initReadGroup(
      state_read, "state",
      [&state_interfaces](Joint& joint, std::unordered_map<std::string, DxlValueMappingList>& mappings) {
        for (const auto& interface_name : state_interfaces) {
          mappings[joint.stateInterfaceToRegisterName(interface_name)].emplace_back(
              joint.dynamixel.get(), DxlValue(&joint.getActuatorState().current[interface_name]));
        }
        mappings[DXL_REGISTER_HARDWARE_ERROR].emplace_back(joint.dynamixel.get(),
                                                           DxlValue(&joint.dynamixel->hardware_error_status));
        mappings[DXL_REGISTER_CMD_TORQUE].emplace_back(joint.dynamixel.get(), DxlValue(&joint.torque_read_back));
        mappings[DXL_REGISTER_BUS_WATCHDOG].emplace_back(joint.dynamixel.get(),
                                                         DxlValue(&joint.dynamixel->bus_watchdog_value));
      });
}

bool SyncGroups::setUpGoalRead()
{
  return initReadGroup(goal_read, "goal",
                       [](Joint& joint, std::unordered_map<std::string, DxlValueMappingList>& mappings) {
                         for (const auto& cmd_interface : joint.getAvailableCommandInterfaces()) {
                           // NaN until the first read, so an unread goal never verifies by accident
                           joint.read_goal_values[cmd_interface] = std::numeric_limits<double>::quiet_NaN();
                           mappings[joint.commandInterfaceToRegisterName(cmd_interface)].emplace_back(
                               joint.dynamixel.get(),
                               DxlValue(&joint.read_goal_values.at(cmd_interface), joint.commandRatio(cmd_interface)));
                         }
                       });
}

bool SyncGroups::setUpTorqueWrite()
{
  torque_write = SyncWriteManager();
  for (auto& [name, joint] : *joints_) {
    if (!torque_write.addRegister(*joint.dynamixel, DXL_REGISTER_CMD_TORQUE, joint.torque)) {
      DXL_LOG_ERROR("Joint '" << name << "' could not bind the torque register.");
      return false;
    }
  }
  return torque_write.init(*driver_);
}

bool SyncGroups::setUpControlWrite()
{
  control_write = SyncWriteManager();
  for (auto& [name, joint] : *joints_) {
    // Command interfaces are claimed per joint, so they collide per joint
    if (const auto collision =
            findRegisterCollision(joint.getAvailableCommandInterfaces(), [&joint](const std::string& interface_name) {
              return joint.commandInterfaceToRegisterName(interface_name);
            })) {
      const auto& [first, second, register_name] = *collision;
      DXL_LOG_ERROR("Joint '" << name << "' declares the command interfaces '" << first << "' and '" << second
                              << "', which both write register '" << register_name
                              << "'. Declare only one of them for this joint.");
      return false;
    }
    State& actuator = joint.getActuatorState();
    for (const auto& interface_name : joint.getAvailableCommandInterfaces()) {
      const std::string register_name = joint.commandInterfaceToRegisterName(interface_name);
      // A missing binding would leave the interface claimable and never written
      if (!control_write.addRegister(*joint.dynamixel, register_name, actuator.goal.at(interface_name),
                                     joint.commandRatio(interface_name))) {
        DXL_LOG_ERROR("Joint '" << name << "' declares the command interface '" << interface_name
                                << "', which maps to register '" << register_name
                                << "'. That register cannot be written.");
        return false;
      }
    }
  }
  return control_write.init(*driver_);
}

bool SyncGroups::setUpLedWrite()
{
  led_write = SyncWriteManager();
  for (auto& [name, joint] : *joints_) {
    // All three RGB registers or none, since a sync write needs one layout on every motor
    if (!joint.dynamixel->registerAvailable(DXL_REGISTER_LED_RED) ||
        !joint.dynamixel->registerAvailable(DXL_REGISTER_LED_GREEN) ||
        !joint.dynamixel->registerAvailable(DXL_REGISTER_LED_BLUE)) {
      continue;
    }
    if (!led_write.addRegister(*joint.dynamixel, DXL_REGISTER_LED_RED, joint.led_state.red) ||
        !led_write.addRegister(*joint.dynamixel, DXL_REGISTER_LED_GREEN, joint.led_state.green) ||
        !led_write.addRegister(*joint.dynamixel, DXL_REGISTER_LED_BLUE, joint.led_state.blue)) {
      DXL_LOG_ERROR("Joint '" << name << "' could not bind its LED registers.");
      return false;
    }
  }
  return led_write.init(*driver_);
}

bool SyncGroups::setUpWatchdogWrite(const std::set<Dynamixel*>& capable)
{
  watchdog_write = SyncWriteManager();
  for (auto& [name, joint] : *joints_) {
    if (capable.count(joint.dynamixel.get()) == 0) {
      continue;
    }
    // Raw binding: the tick count is the register value
    if (!watchdog_write.addRegister(*joint.dynamixel, DXL_REGISTER_BUS_WATCHDOG, joint.bus_watchdog_command)) {
      DXL_LOG_ERROR("Joint '" << name << "' could not bind the bus watchdog register.");
      return false;
    }
  }
  return watchdog_write.init(*driver_);
}

}  // namespace dynamixel_ros_control
