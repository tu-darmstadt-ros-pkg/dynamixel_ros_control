#ifndef DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_HARDWARE_INTERFACE_H
#define DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_HARDWARE_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "joint.hpp"
#include "sync_read_manager.hpp"
#include "sync_write_manager.hpp"

#include <hardware_interface/system_interface.hpp>

namespace dynamixel_ros_control {

class DynamixelHardwareInterface : public hardware_interface::SystemInterface
{
public:
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

  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>&,
                                                              const std::vector<std::string>&) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  /**
   * Load dynamixels from the configuration
   * @return success
   */
  bool setControlMode();
  void setTorque();

  std::vector<Joint> joints_;
  DynamixelDriver driver_;

  // Read
  SyncReadManager read_manager_;
  SyncReadManager status_read_manager_;
  rclcpp::Time last_successful_read_time_;
  bool first_read_successful_{false};

  // Write
  SyncWriteManager control_write_manager_;
  SyncWriteManager torque_write_manager_;

  // Parameters
  bool debug_{false};
  bool torque_on_startup_{false};
  bool torque_off_on_shutdown_{false};

  // ROS interface
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_torque_service_;
};

}  // namespace dynamixel_ros_control
#endif