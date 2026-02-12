#ifndef DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_HARDWARE_INTERFACE_H
#define DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_HARDWARE_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <mutex>

#include "joint.hpp"
#include "sync_read_manager.hpp"
#include "sync_write_manager.hpp"

#include <hardware_interface/system_interface.hpp>
#include <transmission_interface/transmission.hpp>
#include <hector_transmission_interface_msgs/srv/adjust_transmission_offsets.hpp>
#include <hector_transmission_interface/adjustable_offset_manager.hpp>
#include <controller_orchestrator/controller_orchestrator.hpp>
#include <hector_transmission_interface/adjustable_offset_transmission_loader.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace dynamixel_ros_control {

/**
 * @brief ROS 2 hardware interface for Dynamixel motors.
 *
 * Implements the ros2_control SystemInterface for Dynamixel protocol 2.0 motors.
 * Features include synchronized read/write, automatic control mode switching,
 * software E-Stop, LED status indicators, and hardware error recovery.
 */
class DynamixelHardwareInterface : public hardware_interface::SystemInterface
{
public:
  ~DynamixelHardwareInterface() override;

  /// @brief Load parameters and initialize ROS interfaces (services, subscriptions).
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& param) override;

  /// @brief Connect to hardware and set up sync read/write managers.
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  /// @brief Disconnect from hardware and clean up resources.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  /// @brief Export state interfaces (position, velocity, effort, etc.) for all joints.
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;

  /// @brief Export command interfaces (position, velocity, etc.) for all joints.
  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;

  /// @brief Switch control modes when controllers are loaded/unloaded.
  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) override;

  /// @brief Enable torque and set initial goal positions.
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  /// @brief Disable torque (if configured) and update LED state.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  /// @brief Handle hardware errors by activating E-Stop and optionally rebooting.
  CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

  /// @brief Read state from all motors via SyncRead.
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  /// @brief Write commands to all motors via SyncWrite.
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  /// @brief Load transmission configurations from URDF.
  bool loadTransmissionConfiguration();

  /// @brief Process command interface start/stop requests.
  bool processCommandInterfaceUpdates(const std::vector<std::string>& interface_updates, bool stopping);

  // Sync manager setup
  bool setUpStateAndStatusReadManager();  ///< Configure SyncRead for state and hardware status.
  bool setUpCmdReadManager();             ///< Configure SyncRead for command verification.
  bool setUpTorqueWriteManager();         ///< Configure SyncWrite for torque enable.
  bool setUpControlWriteManager();        ///< Configure SyncWrite for control commands.
  bool setUpLEDWriteManager();            ///< Configure SyncWrite for LED colors.

  /// @brief Check if all motors report no hardware errors.
  bool isHardwareOk() const;

  /// @brief Get names of joints that have hardware errors.
  std::vector<std::string> getJointsWithHardwareError() const;

  /// @brief Reboot motors with hardware errors and restore torque state.
  bool reboot();

  /// @brief Enable or disable torque on all motors.
  bool setTorque(bool do_enable, bool skip_controller_unloading = false, int retries = 5, bool direct_write = false);

  /// @brief Enable or disable the software E-Stop.
  bool setEStop(bool do_enable);

  /// @brief Reset goal positions to current positions and verify they were written correctly.
  bool resetGoalStateAndVerify(const std::vector<std::string>& joints);

  bool resetGoalStateAndVerify(const std::vector<std::string>& joints, int retries);

  /// @brief Deactivate all controllers using this hardware interface.
  bool deactivateControllers() const;

  // LED management
  void updateColorLED(std::string new_state = "");                            ///< Update LEDs based on current state.
  void setColorLED(const int& red, const int& green, const int& blue);        ///< Set LED color for all motors.
  void setColorLED(const std::string& color);                                 ///< Set LED color by name.
  void setJointLED(const std::string& joint_name, const std::string& color);  ///< Set LED color for a specific motor.
  void updateErrorLEDs();                                                     ///< Set red LED for motors with errors.

  /// @brief Activate E-Stop: switch to position mode and hold current positions.
  bool activateEStop();

  // Joint configuration
  std::unordered_map<std::string, Joint> joints_;
  std::vector<std::string> joint_names_;
  DynamixelDriver driver_;

  // Sync managers for efficient bulk communication
  SyncReadManager read_manager_;      ///< Reads state and hardware status from all motors.
  SyncReadManager cmd_read_manager_;  ///< Reads command registers for verification.
  rclcpp::Time last_successful_read_time_;
  bool first_read_successful_{false};  ///< Ensures read() succeeds before write() sends commands.

  SyncWriteManager control_write_manager_;  ///< Writes position/velocity/effort commands.
  SyncWriteManager torque_write_manager_;   ///< Writes torque enable register.
  SyncWriteManager led_write_manager_;      ///< Writes LED color registers.

  // Configuration parameters (from URDF)
  bool debug_{false};
  bool torque_on_startup_{false};
  bool torque_off_on_shutdown_{false};

  // Runtime state
  std::atomic<bool> is_torqued_{false};     ///< Current torque state of motors.
  bool desired_torque_state_{false};        ///< User's desired torque state (restored after reboot).
  std::atomic<bool> e_stop_active_{false};  ///< True if software E-Stop is engaged.
  bool mode_switch_failed_{false};          ///< True if control mode switch failed.

  constexpr static int max_reset_and_verify_retries_ = 5;  ///< Max retries for resetting goal state and verifying.

  // ROS interfaces
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_torque_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reboot_service_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr soft_e_stop_subscription_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr exe_;
  std::thread exe_thread_;
  std::mutex dynamixel_comm_mutex_;  ///< Protects all Dynamixel communication.
  std::shared_ptr<controller_orchestrator::ControllerOrchestrator> controller_orchestrator_;
  std::shared_ptr<hector_transmission_interface::AdjustableOffsetManager> offset_manager_;
};

}  // namespace dynamixel_ros_control
#endif
