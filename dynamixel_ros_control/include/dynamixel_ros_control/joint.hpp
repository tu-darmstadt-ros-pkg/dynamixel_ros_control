#ifndef DYNAMIXEL_ROS_CONTROL_JOINT_H
#define DYNAMIXEL_ROS_CONTROL_JOINT_H

#include <dynamixel_ros_control/dynamixel.hpp>
#include <dynamixel_ros_control/dynamixel_driver.hpp>
#include <hardware_interface/hardware_info.hpp>

namespace dynamixel_ros_control {

class Joint
{
public:
  Joint() = default;

  bool loadConfiguration(DynamixelDriver& driver, const hardware_interface::ComponentInfo& info);

  std::string name;
  std::shared_ptr<Dynamixel> dynamixel;

  bool torque{false};
  std::unordered_map<std::string, double> current_state;
  std::unordered_map<std::string, double> goal_state;

  double mounting_offset{0.0};
  double offset{0.0};

  double estop_position{0.0};

  [[nodiscard]] ControlMode getControlMode() const;

  [[nodiscard]] bool isPositionControlled() const;
  [[nodiscard]] bool isVelocityControlled() const;
  [[nodiscard]] bool isEffortControlled() const;
  [[nodiscard]] const std::vector<std::string>& getAvailableStateInterfaces() const;
  [[nodiscard]] const std::vector<std::string>& getAvailableCommandInterfaces() const;

  bool addActiveCommandInterface(const std::string& interface_name);
  bool removeActiveCommandInterface(const std::string& interface_name);
  bool updateControlMode();
  [[nodiscard]] const std::vector<std::string>& getActiveCommandInterfaces() const;

private:
  ControlMode getControlModeFromInterfaces(const std::vector<std::string>& interfaces) const;

  ControlMode control_mode_{UNDEFINED};
  std::vector<std::string> active_command_interfaces_;

  // Parameters
  ControlMode preferred_position_control_mode_;
  std::vector<std::string> state_interfaces_;
  std::vector<std::string> command_interfaces_;
};

}  // namespace dynamixel_ros_control

#endif
