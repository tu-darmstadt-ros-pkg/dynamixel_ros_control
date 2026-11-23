#ifndef DYNAMIXEL_ROS_CONTROL_LED_MANAGER_HPP
#define DYNAMIXEL_ROS_CONTROL_LED_MANAGER_HPP

#include <string>
#include <unordered_map>
#include <vector>

#include <dynamixel_ros_control/joint.hpp>
#include <dynamixel_ros_control/sync_write_manager.hpp>

namespace dynamixel_ros_control {

/**
 * @brief Drives the motor status LEDs.
 *
 * Color scheme, evaluated per joint: pink = unconfigured/inactive, green = active + torque verified
 * off (safe to touch), blue = active + torque verified on, yellow = active + torque unverifiable
 * (assume live), red = hardware error.
 */
class LedManager
{
public:
  struct Color
  {
    double red;
    double green;
    double blue;
  };

  /// @brief Both pointers must outlive this manager (they are members of the hardware interface).
  /// @param logger The logger of the bus
  void init(std::unordered_map<std::string, Joint>* joints, SyncWriteManager* led_write_manager,
            const rclcpp::Logger& logger = dynamixel_ros_control::dxlLogger());

  /// @brief Set every LED from the interface state and each joint's observed torque, override error
  /// joints with red, and write.
  void update(bool active, const std::vector<std::string>& joints_with_error);

  /// @brief Set red LEDs for the given joints. Does NOT write — call writeToHardware().
  void setErrorLeds(const std::vector<std::string>& joints_with_error);

  /// @brief Write the LED state of every joint in a single indirect-addressed sync write.
  /// Caller must hold the comm mutex (or be in a lifecycle callback that owns the bus).
  bool writeToHardware();

private:
  [[nodiscard]] const rclcpp::Logger& dxlLogger() const
  {
    return logger_;
  }
  void setAll(const Color& color);
  void setJoint(const std::string& joint_name, const Color& color);

  std::unordered_map<std::string, Joint>* joints_{nullptr};
  SyncWriteManager* led_write_manager_{nullptr};
  rclcpp::Logger logger_{dynamixel_ros_control::dxlLogger()};
};

}  // namespace dynamixel_ros_control

#endif  // DYNAMIXEL_ROS_CONTROL_LED_MANAGER_HPP
