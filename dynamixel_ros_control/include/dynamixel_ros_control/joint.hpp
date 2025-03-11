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

  bool loadConfiguration(DynamixelDriver& driver, const hardware_interface::ComponentInfo& info,
                         const std::unordered_map<std::string, std::string>& state_interface_to_register,
                         const std::unordered_map<std::string, std::string>& command_interface_to_register);
  bool connect();
  void reset();

  [[nodiscard]] ControlMode getControlMode() const;

  [[nodiscard]] bool isPositionControlled() const;
  [[nodiscard]] bool isVelocityControlled() const;
  [[nodiscard]] bool isEffortControlled() const;
  [[nodiscard]] const std::vector<std::string>& getAvailableStateInterfaces() const;
  [[nodiscard]] const std::vector<std::string>& getAvailableCommandInterfaces() const;

  [[nodiscard]] const std::vector<std::string>& getActiveCommandInterfaces() const;
  bool addActiveCommandInterface(const std::string& interface_name);
  bool removeActiveCommandInterface(const std::string& interface_name);
  bool updateControlMode();
  bool controlModeChanged();
  void resetControlModeChanged();

  std::string stateInterfaceToRegisterName(const std::string& interface_name) const;
  std::string commandInterfaceToRegisterName(const std::string& interface_name) const;

  void resetGoalState(const std::string& interface_name);
  void resetGoalState();

  // Parameters
  std::string name;
  std::shared_ptr<Dynamixel> dynamixel;
  double mounting_offset{0.0};
  double offset{0.0};

  // Active state
  bool torque{false};
  std::unordered_map<std::string, double> current_state;
  std::unordered_map<std::string, double> goal_state;
  double estop_position{0.0};

private:
  ControlMode getControlModeFromInterfaces(const std::vector<std::string>& interfaces) const;

  // Active state
  ControlMode control_mode_{UNDEFINED};
  bool control_mode_changed_{false};
  std::vector<std::string> active_command_interfaces_;

  // Parameters
  ControlMode preferred_position_control_mode_{POSITION};
  std::vector<std::string> state_interfaces_;
  std::vector<std::string> command_interfaces_;
  std::unordered_map<std::string, std::string> command_interface_to_register_;
  std::unordered_map<std::string, std::string> state_interface_to_register_;
};

}  // namespace dynamixel_ros_control

#endif
