#ifndef DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_DIAGNOSTICS_H
#define DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_DIAGNOSTICS_H

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include "dynamixel.hpp"
#include "joint.hpp"
#include "sync_read_manager.hpp"
#include "sync_write_manager.hpp"

namespace dynamixel_ros_control {

/// @brief Per-joint snapshot copied from read() into the diagnostics buffer. No allocations after init.
struct JointHealthSnapshot
{
  std::string joint_name;
  uint8_t motor_id{0};
  bool torque_desired{false};  ///< Intent for this joint (Joint::torque_desired), not a hardware readback
  TorqueState torque_observed{TorqueState::Unknown};  ///< torque_enable as last read from the motor
  int32_t hardware_error_status{0};
  ControlMode operating_mode{UNDEFINED};  ///< Software cache (intent), not a hardware readback.
};

/// @brief Bus-wide diagnostics snapshot. Populated each `read()`, consumed by the 1 Hz timer.
struct HealthSnapshot
{
  rclcpp::Time stamp;
  rclcpp::Time last_successful_read;
  bool torque_globally_desired{false};
  bool mode_switch_failed{false};
  unsigned int read_consecutive_errors{0};
  unsigned int write_consecutive_errors{0};
  /// The longest silence between two bus transactions since the previous snapshot.
  double longest_bus_silence_ms{0.0};
  /// read() and write() calls skipped because a mode switch or service held the bus
  uint64_t overlap_skipped_cycles{0};
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
 * namespace. The standard tooling can still pick it up via an aggregator scoped to that robot.
 * Snapshots are recorded in `read()` (lock-free try_lock, skipped on contention) and the timer
 * publishes the latest. Per joint: torque, decoded hardware error, current operating mode.
 * Bus-wide: error counters and thresholds, last successful read, mode-switch flag.
 */
class DynamixelDiagnostics
{
public:
  DynamixelDiagnostics(rclcpp::Node::SharedPtr node, std::string hw_iface_name,
                       const std::unordered_map<std::string, Joint>& joints,
                       const std::vector<std::string>& joint_names, const SyncReadManager& read_manager,
                       const SyncWriteManager& write_manager);

  /// @brief Read live EEPROM state and publish the manifest. Not RT-safe — call from lifecycle
  /// hooks or service handlers, not from `read()`/`write()`.
  void publishManifest(const rclcpp::Time& stamp);

  /// @brief The USB latency timer the next manifest reports on its bus status, nullopt for a port without one.
  void setUsbLatencyTimer(std::optional<int> latency_ms)
  {
    usb_latency_timer_ms_ = latency_ms;
  }

  /// @brief Start the 1 Hz diagnostics publish timer. Idempotent.
  void startHealthTimer();

  /// @brief Stop the diagnostics publish timer and wait for a publish it is running. Idempotent.
  void stopHealthTimer();

  /// @brief Publish the current snapshot.
  ///
  /// The timer's own callback, and callable directly for the last message before the timer stops:
  /// a component going down on a fault would otherwise leave the last *healthy* sample standing as
  /// its final word, and consumers would see silence where the error belongs.
  void publishHealth();

  /// @brief Record a snapshot of runtime state. Safe to call from `read()`. Uses try_lock.
  /// On contention the snapshot is skipped (the timer publishes the previous one).
  void snapshotHealth(const rclcpp::Time& stamp, bool torque_globally_desired, bool mode_switch_failed,
                      const rclcpp::Time& last_successful_read, double longest_bus_silence_ms,
                      uint64_t overlap_skipped_cycles);

private:
  void publishHealthLocked();

  rclcpp::Node::SharedPtr node_;
  std::string hw_iface_name_;
  const std::unordered_map<std::string, Joint>& joints_;
  const std::vector<std::string>& joint_names_;
  const SyncReadManager& read_manager_;
  const SyncWriteManager& write_manager_;

  // Plain publishers, since both publish off the RT thread and a dropped last word or manifest is never sent again
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr manifest_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_pub_;
  diagnostic_msgs::msg::DiagnosticArray manifest_msg_;
  diagnostic_msgs::msg::DiagnosticArray health_msg_;
  std::optional<int> usb_latency_timer_ms_;

  std::mutex snapshot_mutex_;
  HealthSnapshot snapshot_;
  std::atomic<bool> snapshot_valid_{false};

  rclcpp::TimerBase::SharedPtr health_timer_;
  /// publishHealth() runs on the timer (executor) and from on_error (controller manager), and
  /// both fill health_msg_
  std::mutex publish_mutex_;
  bool health_timer_running_{false};                               ///< Under publish_mutex_
  static constexpr std::chrono::milliseconds HEALTH_PERIOD{1000};  // 1 Hz, matches the diagnostics convention
};

}  // namespace dynamixel_ros_control

#endif
