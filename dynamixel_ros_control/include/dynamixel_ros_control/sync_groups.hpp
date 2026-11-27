#ifndef DYNAMIXEL_ROS_CONTROL_SYNC_GROUPS_H
#define DYNAMIXEL_ROS_CONTROL_SYNC_GROUPS_H

#include <functional>
#include <set>
#include <string>
#include <unordered_map>

#include <dynamixel_ros_control/joint.hpp>
#include <dynamixel_ros_control/sync_read_manager.hpp>
#include <dynamixel_ros_control/sync_write_manager.hpp>

namespace dynamixel_ros_control {

/**
 * @brief The sync groups of one bus.
 *
 * Every group takes the next slots of the driver's one indirect address counter, so configure()
 * builds them in a fixed order and a new group goes last. On P-series that block is EEPROM.
 */
class SyncGroups
{
public:
  /// @param state_interfaces Read from every motor, so the state group's byte layout is uniform
  /// @param watchdog_capable The motors that have a bus watchdog register
  [[nodiscard]] bool configure(std::unordered_map<std::string, Joint>& joints, DynamixelDriver& driver,
                               const std::set<std::string>& state_interfaces,
                               const std::set<Dynamixel*>& watchdog_capable);

  /// @brief Rewrite the indirect address pointers of every group for @p motors, which a reboot
  /// wiped on RAM-resident models. A group left out would keep using stale slots without any
  /// packet-level failure.
  [[nodiscard]] bool rewriteIndirectAddresses(const std::set<Dynamixel*>& motors);

  SyncReadManager state_read;  ///< State, hardware status, torque and bus watchdog, every cycle
  SyncReadManager goal_read;   ///< Goal registers, to verify what was written
  SyncWriteManager control_write;
  SyncWriteManager torque_write;
  SyncWriteManager led_write;
  SyncWriteManager watchdog_write;  ///< Only the motors that have the register

private:
  [[nodiscard]] const rclcpp::Logger& dxlLogger() const
  {
    return driver_ != nullptr ? driver_->dxlLogger() : dynamixel_ros_control::dxlLogger();
  }
  using RegisterCollector = std::function<void(Joint&, std::unordered_map<std::string, DxlValueMappingList>&)>;
  bool initReadGroup(SyncReadManager& group, const char* label, const RegisterCollector& collect);
  bool setUpStateRead(const std::set<std::string>& state_interfaces);
  bool setUpGoalRead();
  bool setUpTorqueWrite();
  bool setUpControlWrite();
  bool setUpLedWrite();
  bool setUpWatchdogWrite(const std::set<Dynamixel*>& capable);

  std::unordered_map<std::string, Joint>* joints_{nullptr};
  DynamixelDriver* driver_{nullptr};
};

}  // namespace dynamixel_ros_control

#endif
