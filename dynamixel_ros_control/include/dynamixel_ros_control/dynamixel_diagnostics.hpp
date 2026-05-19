#ifndef DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_DIAGNOSTICS_H
#define DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_DIAGNOSTICS_H

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include "dynamixel.hpp"
#include "dynamixel_driver.hpp"
#include "joint.hpp"
#include "sync_read_manager.hpp"
#include "sync_write_manager.hpp"

namespace dynamixel_ros_control {

/// @brief Per-joint snapshot copied from read() into the diagnostics buffer. No allocations after init.
struct JointHealthSnapshot
{
  std::string joint_name;
  uint8_t motor_id{0};
  bool torque_desired{false};  ///< User-requested torque state (joint.torque). NOT a hardware
                               ///< readback — setTorque() flips this before the bus write, so it
                               ///< reflects intent even if the write later fails or retries.
  int32_t hardware_error_status{0};
  ControlMode operating_mode{UNDEFINED};
};

/// @brief Bus-wide diagnostics snapshot. Populated each `read()`, consumed by the 1 Hz timer.
struct HealthSnapshot
{
  rclcpp::Time stamp;
  rclcpp::Time last_successful_read;
  bool e_stop_active{false};
  bool torque_globally_desired{false};
  bool mode_switch_failed{false};
  unsigned int read_consecutive_errors{0};
  unsigned int write_consecutive_errors{0};
  std::vector<JointHealthSnapshot> joints;
};

/**
 * @brief Owns the `~/manifest` and `~/diagnostics` publishers for `DynamixelHardwareInterface`.
 *
 * `~/manifest` is a `diagnostic_msgs::msg::DiagnosticArray` published with transient_local
 * QoS on `on_configure` and after a successful `reboot()`. It carries one bus-level status
 * plus one per joint with static info (motor id, model, firmware, live EEPROM config, declared
 * interfaces).
 *
 * `~/diagnostics` is a `diagnostic_msgs::msg::DiagnosticArray` published at 1 Hz from a timer
 * — the conventional rate and message type for the diagnostics ecosystem (rqt_robot_monitor,
 * diagnostic_aggregator). The topic is private-namespaced because each robot runs in its own
 * namespace; the standard tooling can still pick it up via an aggregator scoped to that robot.
 * Snapshots are recorded in `read()` (lock-free try_lock; skipped on contention) and the timer
 * publishes the latest. Per joint: torque, decoded hardware error, current operating mode.
 * Bus-wide: e-stop, error counters and thresholds, last successful read, mode-switch flag.
 */
class DynamixelDiagnostics
{
public:
  DynamixelDiagnostics(rclcpp::Node::SharedPtr node, std::string hw_iface_name,
                       const std::unordered_map<std::string, Joint>& joints,
                       const std::vector<std::string>& joint_names, DynamixelDriver& driver,
                       const SyncReadManager& read_manager, const SyncWriteManager& write_manager);

  /// @brief Read live EEPROM state and publish the manifest. Not RT-safe — call from lifecycle
  /// hooks or service handlers, not from `read()`/`write()`.
  void publishManifest(const rclcpp::Time& stamp);

  /// @brief Start the 1 Hz diagnostics publish timer. Idempotent.
  void startHealthTimer();

  /// @brief Stop the diagnostics publish timer. Idempotent.
  void stopHealthTimer();

  /// @brief Record a snapshot of runtime state. Safe to call from `read()`; uses try_lock.
  /// On contention the snapshot is skipped (the timer publishes the previous one).
  void snapshotHealth(const rclcpp::Time& stamp, bool e_stop_active, bool torque_globally_desired,
                      bool mode_switch_failed, const rclcpp::Time& last_successful_read);

private:
  void publishHealth();

  rclcpp::Node::SharedPtr node_;
  std::string hw_iface_name_;
  const std::unordered_map<std::string, Joint>& joints_;
  const std::vector<std::string>& joint_names_;
  DynamixelDriver& driver_;
  const SyncReadManager& read_manager_;
  const SyncWriteManager& write_manager_;

  std::shared_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>> rt_manifest_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>> rt_health_pub_;

  std::mutex snapshot_mutex_;
  HealthSnapshot snapshot_;
  std::atomic<bool> snapshot_valid_{false};

  rclcpp::TimerBase::SharedPtr health_timer_;
  static constexpr std::chrono::milliseconds HEALTH_PERIOD{1000};  // 1 Hz, matches the diagnostics convention
};

}  // namespace dynamixel_ros_control

#endif
