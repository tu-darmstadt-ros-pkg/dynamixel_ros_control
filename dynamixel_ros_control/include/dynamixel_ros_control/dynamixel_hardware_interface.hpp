#ifndef DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_HARDWARE_INTERFACE_H
#define DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_HARDWARE_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <atomic>
#include <filesystem>
#include <map>
#include <mutex>
#include <set>
#include <thread>

#include "bus_watchdog.hpp"
#include "log.hpp"
#include "dynamixel_diagnostics.hpp"
#include "joint.hpp"
#include "joint_state_publisher_set.hpp"
#include "led_manager.hpp"
#include "sync_groups.hpp"

#include <hardware_interface/system_interface.hpp>
#include <hector_transmission_interface/adjustable_offset_manager.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace dynamixel_ros_control {

/// @brief Per-motor outcome of a torque change, each classified by its own read-back.
struct TorqueChangeResult
{
  std::set<std::string> confirmed;  ///< Read back in the requested state
  std::set<std::string> unknown;    ///< The motor did not answer
  std::set<std::string> rejected;   ///< Read back not in the requested state

  [[nodiscard]] bool fullSuccess() const
  {
    return unknown.empty() && rejected.empty();
  }
};

/// @brief ros2_control SystemInterface for one bus of Dynamixel Protocol 2.0 motors.
class DynamixelHardwareInterface : public hardware_interface::SystemInterface
{
public:
  /// @param sysfs_root where the USB latency timer of the port is looked up, moved only by tests
  explicit DynamixelHardwareInterface(std::filesystem::path sysfs_root = "/sys")
      : sysfs_root_(std::move(sysfs_root))
  {}
  ~DynamixelHardwareInterface() override;

  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& param) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  /// @brief Mark the component unconfigured. The port stays open and the sync managers keep their
  /// state, so a later on_configure re-establishes them rather than building from nothing.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;

  /// @brief Refuse a switch the joints cannot carry out, without bus traffic: an unknown interface,
  /// torque_off together with a motion interface on one joint, or a pure current mode.
  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) override;

  /// The reason a switch cannot be carried out, empty if it can. Pure logic, no bus traffic.
  [[nodiscard]] std::string switchRefusal(const std::vector<std::string>& start_interfaces,
                                          const std::vector<std::string>& stop_interfaces) const;

  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) override;

  /// @brief read() and write() calls skipped because a mode switch or service held the bus
  uint64_t overlapSkippedCycles() const
  {
    return overlap_skipped_cycles_;
  }

  /// @brief Callers blocked waiting for the bus, which read() and write() yield to
  [[nodiscard]] int busWaiters() const
  {
    return dynamixel_comm_mutex_.waiting();
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  /// @brief Stop free-running joints, and secure the hardware after a fault or a failed mode switch.
  CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  [[nodiscard]] const rclcpp::Logger& dxlLogger() const
  {
    return logger_;
  }

  // on_init steps, in order.
  [[nodiscard]] bool loadHardwareParameters();  ///< Hardware params and driver setup.
  [[nodiscard]] bool loadJoints();              ///< Build joints_ / joint_names_ and mimic links.
  [[nodiscard]] bool setUpRosInterfaces(const std::string& hardware_name);  ///< Node, executor, services, publishers.

  bool loadTransmissionConfiguration();

  /// Warns about a USB latency timer above USB_LATENCY_TIMER_TARGET_MS and hands it to the manifest
  void checkUsbLatencyTimer();

  /// Zeroes the goal of every torqued velocity or current joint of @p joint_names and transmits it
  /// once. Caller owns the bus.
  /// @return whether the stop was transmitted
  [[nodiscard]] bool stopFreeRunningJoints(const std::vector<std::string>& joint_names);

  /// read(): copies the joint states into the exported state handles, without waiting for a lock.
  void publishStateHandles();
  /// write(): takes the commands of @p joint's active interfaces from their handles, without
  /// waiting for a lock. A busy handle leaves the previous command in place for this cycle.
  /// @return whether every handle could be taken
  bool takeCommandHandles(Joint& joint);
  /// perform: writes the goals just reset for @p joint_names into the handles of their active
  /// interfaces, so a controller that starts without a command of its own holds instead of
  /// inheriting what an earlier one left there.
  [[nodiscard]] bool seedCommandHandles(const std::vector<std::string>& joint_names);
  /// Counts a handle that was busy when read() or write() wanted it, and reports it throttled.
  void reportBusyHandle(const std::string& handle_name);

  /// @brief The union of every joint's state interfaces, which every joint exports and every motor
  /// is read for, since one sync read fetches one address range from all of them.
  [[nodiscard]] std::set<std::string> configuredStateInterfaceUnion() const;

  /// @brief Arm the bus watchdog and prove by read-back that it landed.
  /// @param targets The motors whose proof counts, nullptr for all. The arm always goes to the
  /// whole bus, so a motor outside a subset torque-on cannot roll it back.
  bool armWatchdogVerified(const std::set<Dynamixel*>* targets = nullptr);
  bool armWatchdogVerifiedOnce(const std::set<Dynamixel*>* targets = nullptr);

  bool isHardwareOk() const;
  /// @brief The joints with a hardware error and what each reports, for a log line.
  [[nodiscard]] std::string describeHardwareErrors() const;

  std::vector<std::string> getJointsWithHardwareError() const;

  /// @brief Asks every motor for its hardware error status. Recovery needs the bus's answer, and
  /// the cache is empty in a process that has not completed a read.
  std::vector<std::string> readJointsFlaggingHardwareError();

  /// @brief Whether every joint's torque intent is on. Caller owns the bus.
  [[nodiscard]] bool allJointsDesireTorque() const;

  /// @brief Reboot motors with hardware errors and restore torque state. Caller holds the bus and
  /// has checked that the component is unconfigured.
  bool rebootLocked();

  /// @brief Enable or disable torque on exactly the named joints. An empty set is a no-op and
  /// never means "all". Torque-off is best-effort per motor. Torque-on rolls back to torque-off
  /// if it cannot be fully verified. See TorqueChangeResult. Arms no watchdog. Caller owns the bus.
  TorqueChangeResult setTorqueForJoints(bool do_enable, const std::set<std::string>& targets);

  /// @brief setTorqueForJoints() over every joint of this interface. Caller owns the bus.
  TorqueChangeResult setTorqueForAll(bool do_enable);

  /// @brief The joints of @p names a controller commands, in order. Caller owns the bus.
  std::vector<std::string> commandedJoints(const std::vector<std::string>& names) const;

  /// @brief Joints whose torque_desired equals the given value. Caller owns the bus.
  std::set<std::string> jointsDesiringTorque(bool desired) const;

  /// @brief Write the torque to @p motors and verify each by read-back, retrying the unconfirmed
  /// ones. Caller owns the bus.
  TorqueChangeResult writeAndVerifyTorque(bool do_enable, const std::set<Dynamixel*>& motors);

  /// @brief Undo a torque-on that cannot be vouched for: de-energise the whole requested set, clear
  /// its intent, and report nothing as confirmed. Caller owns the bus.
  void rollbackTorqueOn(const std::vector<std::string>& requested_names, const std::set<Dynamixel*>& requested,
                        TorqueChangeResult& result);

  /// @brief The one way to observe torque. Reads torque_enable for @p motors and commits it to
  /// Joint::torque_observed. A motor that did not answer is Unknown, never assumed off. Caller owns
  /// the bus.
  /// @return the motors that answered
  std::set<Dynamixel*> readTorqueStates(const std::set<Dynamixel*>& motors);

  /// @brief Reset the goals to hold the present state and verify them by read-back.
  bool resetGoalStateAndVerify(const std::vector<std::string>& joints, int retries);
  bool resetGoalStateAndVerifyOnce(const std::vector<std::string>& joints);

  /// @brief What securing achieved for every joint. Keyed by outcome, so isSecured() alone decides
  /// which outcomes are safe.
  struct SecuringReport
  {
    std::map<SecuringOutcome, std::set<std::string>> by_outcome;
    std::set<std::string> unresolved;  ///< No verdict yet. The next pass tries again.

    [[nodiscard]] const std::set<std::string>& of(SecuringOutcome outcome) const
    {
      static const std::set<std::string> none;
      const auto it = by_outcome.find(outcome);
      return it != by_outcome.end() ? it->second : none;
    }

    [[nodiscard]] bool fullySecured() const
    {
      return unresolved.empty() && std::all_of(by_outcome.begin(), by_outcome.end(), [](const auto& entry) {
               return entry.second.empty() || isSecured(entry.first);
             });
    }

    [[nodiscard]] size_t securedCount() const
    {
      size_t count = 0;
      for (const auto& [outcome, names] : by_outcome) {
        if (isSecured(outcome)) {
          count += names.size();
        }
      }
      return count;
    }
  };

  SecuringReport buildSecuringReport() const;

  bool verifyGoal(const std::string& name, Joint& joint, const std::string& interface_name);

  /// @brief Whether a goal read back matches the one written. The firmware rejects a goal outside
  /// its limits and never clamps it, so any other value is an older goal.
  static bool goalReadBackMatches(double target, double read_value);

  void updateLeds();
  /// @brief For a transition that lights the LEDs of the state it enters before committing it
  void updateLeds(bool active);

  /// @brief The lifecycle state for executor-thread code, since the base class's is a plain member.
  /// During a transition it holds the pessimistic side, e.g. Unconfigured while on_error runs.
  enum class LifecyclePhase : uint8_t
  {
    Unconfigured,
    Inactive,
    Active
  };

  /// @brief One pass of the securing pipeline over the joints not yet secured. Each step acts on
  /// every joint still in play at once. A joint that fails a step gets a statusless torque-off.
  /// Caller owns the bus.
  SecuringReport securingPass();
  // The steps of securingPass(), in order
  struct SecuringPass;
  SecuringPass beginSecuringPass();
  void observeForSecuring(SecuringPass& pass);
  void planSecuring(SecuringPass& pass);
  void switchModesForHold(SecuringPass& pass);
  void writeHoldGoals(SecuringPass& pass);
  void restoreHoldTorque(SecuringPass& pass);
  void classifySecured(SecuringPass& pass);
  void torqueOffFailed(SecuringPass& pass);
  /// @brief Repeats securingPass() until every joint has a verdict. Caller owns the bus.
  SecuringReport secureHardware();

  std::unordered_map<std::string, Joint> joints_;
  std::vector<std::string> joint_names_;
  std::filesystem::path sysfs_root_;
  DynamixelDriver driver_;

  SyncGroups groups_;
  BusWatchdog bus_watchdog_;
  rclcpp::Time last_successful_read_time_;
  /// A successful read anchored every goal since on_configure. write() sends nothing until then.
  std::atomic<bool> first_read_successful_{false};

  LedManager led_manager_;

  /// A child of the shared logger, so the debug parameter of one component leaves the others alone
  rclcpp::Logger logger_{dynamixel_ros_control::dxlLogger()};

  bool torque_on_startup_{false};
  bool torque_off_on_shutdown_{false};
  bool allow_velocity_without_watchdog_{false};

  /// Written only by the on_* callbacks
  std::atomic<LifecyclePhase> lifecycle_phase_{LifecyclePhase::Unconfigured};
  std::atomic<bool> mode_switch_failed_{false};
  /// Divides the control rate down to the rate the diagnostics timer publishes at
  int health_snapshot_counter_{0};
  int health_snapshot_divisor_{1};

  /// @brief Whether a throttled message is due, and claim the slot if so. Per instance, unlike the
  /// static of RCLCPP_*_THROTTLE, and on the steady clock, which unlike ROS time runs before /clock.
  static bool throttleDue(std::atomic<std::chrono::steady_clock::rep>& last_logged, std::chrono::nanoseconds period);

  // Last emission of each throttled report
  std::atomic<std::chrono::steady_clock::rep> last_securing_timing_log_{0};
  std::atomic<std::chrono::steady_clock::rep> last_unsecured_log_{0};
  std::atomic<std::chrono::steady_clock::rep> last_watchdog_trip_log_{0};
  /// Handles read() or write() found locked by another thread
  uint64_t busy_handle_count_{0};
  /// The longest bus silence since the last health snapshot
  double longest_bus_silence_ms_{0.0};
  std::atomic<std::chrono::steady_clock::rep> last_bus_silence_log_{0};
  std::atomic<std::chrono::steady_clock::rep> last_busy_handle_log_{0};
  std::atomic<std::chrono::steady_clock::rep> last_read_failure_log_{0};
  std::atomic<std::chrono::steady_clock::rep> last_write_failure_log_{0};
  std::atomic<std::chrono::steady_clock::rep> last_overlap_log_{0};

  // A dropped status packet is ordinary on a shared bus, so every bus transaction gets retried
  constexpr static int max_reset_and_verify_retries_ = 5;
  constexpr static int torque_verify_retries_ = 5;
  constexpr static int watchdog_arm_retries_ = 3;
  /// Of every goal read-back check, in the interface's own units
  constexpr static double goal_verify_tolerance_ = 1e-2;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reboot_service_;

  std::unique_ptr<DynamixelDiagnostics> diagnostics_;

  rclcpp::executors::MultiThreadedExecutor::SharedPtr exe_;
  std::thread exe_thread_;
  std::atomic<bool> executor_stop_requested_{false};
  std::atomic<bool> executor_stopped_{false};
  /// The bus. read() and write() only try it and skip their cycle when it is taken. Every other
  /// caller waits for it, and while one waits the loop skips too, so it waits for at most the one
  /// read() or write() in flight. A lock() that throws or relocks on its own thread leaves the loop
  /// skipping for good, so no bus holder may call read(), write() or lock() again.
  class BusMutex
  {
  public:
    void lock()
    {
      ++waiting_;
      mutex_.lock();
      --waiting_;
    }
    void unlock()
    {
      mutex_.unlock();
    }
    bool try_lock()
    {
      return waiting_ == 0 && mutex_.try_lock();
    }
    int waiting() const
    {
      return waiting_;
    }

  private:
    std::mutex mutex_;
    std::atomic<int> waiting_{0};
  };
  BusMutex dynamixel_comm_mutex_;
  std::atomic<uint64_t> overlap_skipped_cycles_{0};
  void noteOverlapSkip(const char* call);

  std::shared_ptr<hector_transmission_interface::AdjustableOffsetManager> offset_manager_;

  JointStatePublisherSet joint_state_publishers_;
};

}  // namespace dynamixel_ros_control
#endif
