#ifndef DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_HARDWARE_INTERFACE_H
#define DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_HARDWARE_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <mutex>

#include "joint.hpp"
#include "sync_read_manager.hpp"
#include "sync_write_manager.hpp"

#include <hardware_interface/system_interface.hpp>
#include <transmission_interface/transmission.hpp>
#include <hector_transmission_interface_msgs/srv/adjust_transmission_offsets.hpp>
#include <controller_orchestrator/controller_orchestrator.hpp>
#include <std_msgs/msg/bool.hpp>
namespace dynamixel_ros_control {

class DynamixelHardwareInterface : public hardware_interface::SystemInterface
{
public:
  ~DynamixelHardwareInterface() override;
  /**
   * Load all parameters from hardware info
   * @param hardware_info
   * @return
   */
  CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;
  /**
   * Connect to hardware
   * @param previous_state
   * @return
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * Do opposite of on_configure
   * @param previous_state
   * @return
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;

  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  bool loadTransmissionConfiguration();
  bool processCommandInterfaceUpdates(const std::vector<std::string>& interface_updates, bool stopping);
  bool setUpStateAndStatusReadManager();
  bool setUpCmdReadManager();
  bool setUpTorqueWriteManager();
  bool setUpControlWriteManager();

  bool isHardwareOk() const;
  bool reboot() const;

  bool setTorque(bool do_enable, bool skip_controller_unloading = false, int retries = 5, bool direct_write = false);
  bool setEStop(bool do_enable);
  bool resetGoalStateAndVerify();
  bool unloadControllers() const;
  void updateColorLED(std::string new_state = "");
  void setColorLED(const int& red, const int& green, const int& blue);
  void setColorLED(const std::string& color);
  void adjustTransmissionOffsetsCallback(
      const std::shared_ptr<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets::Request> request,
      const std::shared_ptr<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets::Response> response);
  bool activateEStop();

  std::unordered_map<std::string, Joint> joints_;
  DynamixelDriver driver_;

  // Read
  SyncReadManager read_manager_;
  SyncReadManager cmd_read_manager_;
  rclcpp::Time last_successful_read_time_;
  bool first_read_successful_{false};

  // Write
  SyncWriteManager control_write_manager_;
  SyncWriteManager torque_write_manager_;

  // Parameters
  bool debug_{false};
  bool torque_on_startup_{false};
  bool torque_off_on_shutdown_{false};
  bool reboot_on_hardware_error_{false};

  // variables
  bool is_torqued_{false};
  std::atomic<bool> e_stop_active_{false};  // true if e-stop is active
  bool mode_switch_failed_{false};

  // ROS interface
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_torque_service_;
  rclcpp::Service<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets>::SharedPtr adjust_offset_service_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr soft_e_stop_subscription_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr exe_;
  std::thread exe_thread_;
  std::mutex dynamixel_comm_mutex_;
  std::shared_ptr<controller_orchestrator::ControllerOrchestrator> controller_orchestrator_;
};

}  // namespace dynamixel_ros_control
#endif
