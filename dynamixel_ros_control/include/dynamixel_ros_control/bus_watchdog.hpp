#ifndef DYNAMIXEL_ROS_CONTROL_BUS_WATCHDOG_H
#define DYNAMIXEL_ROS_CONTROL_BUS_WATCHDOG_H

#include <string>
#include <unordered_map>
#include <vector>

#include <dynamixel_ros_control/joint.hpp>
#include <dynamixel_ros_control/sync_write_manager.hpp>

namespace dynamixel_ros_control {

/// Control cycles without communication before the watchdog trips, when the parameter is unset.
constexpr double DEFAULT_BUS_WATCHDOG_CYCLES = 4.0;

/// Shortest window that survives ordinary scheduling jitter, in control cycles.
constexpr double MIN_BUS_WATCHDOG_WINDOW_CYCLES = 3.0;

/**
 * @brief Raw register ticks for a window of @p cycles control periods.
 *
 * Rounds up, so the window is never shorter than asked for, and clamps in floating point before
 * narrowing. Converting first is undefined for a value outside int's range, which an
 * operator-supplied cycle count can reach.
 *
 * @param cycles Requested window in control cycles. 0 disables the watchdog and yields 0 ticks.
 * @param dt_ms Control period in milliseconds.
 * @param ms_per_tick Register resolution, which differs per motor model.
 */
[[nodiscard]] int32_t busWatchdogTicks(double cycles, double dt_ms, double ms_per_tick);

/**
 * @brief The bus watchdog of every joint on one bus: how long its window is, and whether it is set.
 *
 * The motor stops itself when no packet arrives within the window, which protects a torqued joint
 * against a control loop that has died. A trip latches the register at -1 and every goal write is
 * silently discarded until a 0 is written, so the register is read cyclically (in the state read
 * group) and cleared before it can swallow a command.
 */
class BusWatchdog
{
public:
  /// @param logger The logger of the bus
  void init(std::unordered_map<std::string, Joint>* joints, SyncWriteManager* write_manager,
            const rclcpp::Logger& logger = dynamixel_ros_control::dxlLogger());

  /**
   * @brief Derive each joint's window from the control rate and record which motors have the register.
   * @param cycles Window in control cycles. 0 disables the watchdog on every joint.
   * @param rw_rate Control rate in Hz. 0 leaves every window at zero, as none can be derived.
   * @return false if @p cycles is not a usable window, which is a configuration error.
   */
  [[nodiscard]] bool configure(double cycles, unsigned int rw_rate);

  /// @brief Write 0 to every capable motor, clearing a latch and stopping the window.
  [[nodiscard]] bool clear();

  /// @brief Write each joint's window to every capable motor.
  [[nodiscard]] bool arm();

  /// @brief Whether the last clear/arm asked for the window to be running.
  ///
  /// Intent, not observation: it records what was last transmitted, whether or not the packet
  /// landed. Paired with readsArmed(), it lets the control loop notice a watchdog that should be
  /// running and is not. That is a trip, or an arm that never arrived.
  [[nodiscard]] bool armIntended() const;

  /// @brief Whether every capable joint's last read showed the window it was armed with.
  ///
  /// The allocation-free counterpart of unarmedJoints(), for the per-cycle check. The vector is
  /// only worth building once there is something to report.
  [[nodiscard]] bool readsArmed() const;

  /// @brief Joints with no armed window, because their motor lacks the register or the watchdog is disabled.
  [[nodiscard]] std::vector<std::string> unprotectedJoints() const;

  /// @brief Joints whose last read showed a latched trip.
  [[nodiscard]] std::vector<std::string> trippedJoints() const;

  /// @brief Joints whose last read did not show the window they were armed with.
  [[nodiscard]] std::vector<std::string> unarmedJoints(const std::set<Dynamixel*>* only = nullptr) const;

  /// @brief Joints whose last read did not show a cleared watchdog.
  [[nodiscard]] std::vector<std::string> unclearedJoints(const std::set<Dynamixel*>* only = nullptr) const;

  [[nodiscard]] const std::set<Dynamixel*>& capableMotors() const;

  /// @brief The shortest configured window on this bus in ms, 0 if no motor has one.
  [[nodiscard]] double shortestWindowMs() const;

  /// @brief Whether a bus silence of `silence_ms` used more than half of an armed window.
  [[nodiscard]] bool silenceThreatensWindow(double silence_ms) const;

private:
  [[nodiscard]] const rclcpp::Logger& dxlLogger() const
  {
    return logger_;
  }
  [[nodiscard]] bool writeAll(bool armed);
  [[nodiscard]] std::vector<std::string> jointsNotReading(bool armed, const std::set<Dynamixel*>* only) const;

  std::set<Dynamixel*> capable_;
  /// What the last clear/arm asked for. See armIntended().
  bool arm_intended_{false};
  double shortest_window_ms_{0.0};
  std::unordered_map<std::string, Joint>* joints_{nullptr};
  SyncWriteManager* write_manager_{nullptr};
  rclcpp::Logger logger_{dynamixel_ros_control::dxlLogger()};
};

}  // namespace dynamixel_ros_control

#endif
