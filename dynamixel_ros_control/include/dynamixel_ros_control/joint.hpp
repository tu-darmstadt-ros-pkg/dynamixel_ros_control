#ifndef DYNAMIXEL_ROS_CONTROL_JOINT_H
#define DYNAMIXEL_ROS_CONTROL_JOINT_H

#include <optional>

#include <dynamixel_ros_control/dynamixel.hpp>
#include <dynamixel_ros_control/dynamixel_driver.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <transmission_interface/transmission.hpp>
namespace dynamixel_ros_control {

struct State
{
  std::unordered_map<std::string, double> current;
  std::unordered_map<std::string, double> goal;
};

/// (joint, actuator) values of the interfaces a transmission does not map. They pass unchanged,
/// right for current, a motor quantity whatever the reduction.
using PassThrough = std::vector<std::pair<double*, double*>>;

/// @brief The colour bound into the LED sync write, already clamped and rounded to the register's 0 to 255.
struct LED_State
{
  double red{0};
  double green{0};
  double blue{0};
};

struct MimicState
{
  std::unordered_map<std::string, double> current;
  /// Exported to the controller manager. Only read() copies into them.
  std::unordered_map<std::string, hardware_interface::StateInterface::SharedPtr> state_handles;
  double offset;
  double multiplier;
};

/// @brief Torque of one motor as last proven by a read-back. A sync write carries no status, so
/// Unknown is ordinary, and every safety decision treats it as On. See torqueMayBeLive().
enum class TorqueState
{
  Off,     ///< Read back as de-energised.
  On,      ///< Read back as energised.
  Unknown  ///< Not verifiable (the motor did not answer). Treat as potentially energised.
};

/// @brief How securing left one joint. The first three are safe because the joint is under control,
/// the fourth because it is proven de-energised. The last three are not safe.
enum class SecuringOutcome : uint8_t
{
  Holding,                  ///< Verified holding at its current position, torque read back ON.
  ZeroCommanded,            ///< Verified commanded to zero velocity/current.
  ParkedTorqueOff,          ///< Hold goal verified, left de-energised because that is how it started.
  TorqueOffConfirmed,       ///< Torque-off verified by read-back.
  TorqueOffRejected,        ///< Torque-off read back as NOT applied: the motor is proven energised.
  TorqueOffSentUnverified,  ///< Statusless torque-off transmitted, not verifiable.
  TransmitFailed,           ///< Not even the packet could be sent.
};

/// @brief Whether an outcome leaves the joint in a state the interface can vouch for. No default, so
/// -Wswitch flags a new outcome here instead of letting it inherit an answer.
[[nodiscard]] inline bool isSecured(const SecuringOutcome outcome)
{
  switch (outcome) {
    case SecuringOutcome::Holding:
    case SecuringOutcome::ZeroCommanded:
    case SecuringOutcome::ParkedTorqueOff:
    case SecuringOutcome::TorqueOffConfirmed:
      return true;
    case SecuringOutcome::TorqueOffRejected:
    case SecuringOutcome::TorqueOffSentUnverified:
    case SecuringOutcome::TransmitFailed:
      return false;
  }
  return false;
}

/// @brief Whether a motor in this state must be assumed to be capable of moving.
inline bool torqueMayBeLive(const TorqueState state)
{
  return state != TorqueState::Off;
}

class Joint
{
public:
  [[nodiscard]] bool
  loadConfiguration(DynamixelDriver& driver, const hardware_interface::ComponentInfo& info,
                    const std::unordered_map<std::string, std::string>& state_interface_to_register,
                    const std::unordered_map<std::string, std::string>& command_interface_to_register,
                    const std::unordered_map<std::string, std::string>& interface_to_register_limits);
  [[nodiscard]] bool connect();

  /// Tell the state transmission's jump detector where this motor's position lands after a power
  /// cycle, so a 2pi jump elsewhere is seen for the real move it is.
  [[nodiscard]] bool setupJumpResetWindow();
  void reset();

  [[nodiscard]] ControlMode getControlMode() const;

  [[nodiscard]] const std::vector<std::string>& getAvailableStateInterfaces() const;
  [[nodiscard]] const std::vector<std::string>& getAvailableCommandInterfaces() const;
  [[nodiscard]] const std::vector<std::string>& getActiveCommandInterfaces() const;
  /// Whether the joint exports <joint>/torque_off, a command mode that switches the motor off.
  [[nodiscard]] bool exportsTorqueOff() const
  {
    return exports_torque_off_;
  }
  [[nodiscard]] bool torqueOffClaimed() const
  {
    return torque_off_claimed_;
  }
  void setTorqueOffClaimed(bool claimed)
  {
    torque_off_claimed_ = claimed;
  }

  [[nodiscard]] bool keepsGoalOnControllerChange() const
  {
    return keeps_goal_on_controller_change_;
  }

  [[nodiscard]] bool addActiveCommandInterface(const std::string& interface_name);
  /// Idempotent: securing may already have cleared the interface a controller now releases
  void removeActiveCommandInterface(const std::string& interface_name);
  void clearActiveCommandInterfaces();
  /// @brief The mode this joint must be switched to, or nullopt if the cached one fits. Without an
  /// active command interface every position mode fits. Only meaningful after readControlMode().
  [[nodiscard]] std::optional<ControlMode> getRequiredControlModeChange() const;

  /// @brief Write the mode getRequiredControlModeChange() asks for, if any. See writeControlMode().
  [[nodiscard]] bool updateControlMode(bool leave_torque_off);
  /**
   * @brief Write @p mode and adopt it, together with the torque it leaves.
   *
   * The register only accepts a write without torque, so torque goes off first. It comes back
   * afterwards unless @p leave_torque_off, for a caller that verifies a safe goal before anything
   * can move. control_mode_ changes only once the motor accepted the mode, because every goal reset
   * derives from it.
   * @param torque_proven_off For a caller that read the motor limp in the same bus sequence, which
   * skips the torque-off. An older reading could miss a motor energised since.
   */
  [[nodiscard]] bool writeControlMode(ControlMode mode, bool leave_torque_off, bool torque_proven_off = false);

  [[nodiscard]] bool readControlMode();

  std::string stateInterfaceToRegisterName(const std::string& interface_name) const;
  std::string commandInterfaceToRegisterName(const std::string& interface_name) const;
  std::string interfaceToLimitRegisterName(const std::string& interface_name) const;

  void resetGoalState(const std::string& interface_name);
  void resetGoalState();
  /// Holds the joint while the interface that sets its mode carries NaN. Position modes keep the
  /// position of the first NaN, velocity mode stops. On the actuator goals, so a changed offset does
  /// not move a held joint.
  /// @param command_taken false when a busy handle left the previous goal, which may be the hold
  void holdOnMissingCommand(bool command_taken);
  /// Clamps the caps on the actuator goals to their EEPROM limit, and a NaN to it
  void limitCaps();
  /// The unit ratio to write and read @p interface_name with, 0 for its register's own
  [[nodiscard]] double commandRatio(const std::string& interface_name) const;
  void setupMimicJoint(const std::string& joint_name, double offset, double multiplier);
  void updateMimicJointStates();

  /// Map through the transmissions and their pass through. Only for a joint with transmissions.
  void stateToJoint();
  void goalToActuator();
  void goalToJoint();

  /// The state the motors read and write, which differs from joint_state behind a transmission
  State& getActuatorState();

  ControlMode getPreferredPositionControlMode() const
  {
    return preferred_position_control_mode_;
  }

  std::string name;
  std::shared_ptr<Dynamixel> dynamixel;
  /// Bus watchdog window in register ticks, 0 if disabled or not supported by this motor
  int32_t bus_watchdog_ticks{0};
  /// What the next watchdog sync write sends, 0 to clear or bus_watchdog_ticks to arm
  int32_t bus_watchdog_command{0};

  /// What the torque sync write asks for, which says nothing about what the motor did with it
  bool torque{false};
  /// Where the state read decodes torque_enable. Stale unless this motor just answered, so only
  /// commitTorqueReadBack() turns it into torque_observed.
  bool torque_read_back{false};
  /// The only admissible answer to "is this motor live?"
  TorqueState torque_observed{TorqueState::Unknown};
  /// Operator intent, restored after a reboot
  bool torque_desired{false};
  /// What securing has learned about this joint since the last deliberate torque change. A retry
  /// must not re-derive any of it from a bus that securing itself has changed since.
  struct SecuringEpisode
  {
    std::optional<bool> torque_was_on;  ///< Energised when the episode began
    bool de_energised{false};           ///< Securing took the torque to change the mode
    /// Set once securing reached a verdict, so later passes skip the joint
    std::optional<SecuringOutcome> outcome;
    int attempts{0};    ///< Passes spent in this episode, the current one included
    int unanswered{0};  ///< Consecutive passes without an answer to the first read
    void reset()
    {
      *this = SecuringEpisode{};
    }
  };
  SecuringEpisode securing;

  /// @brief Commit torque_read_back as the observed torque. Only after this motor answered a read.
  void commitTorqueReadBack()
  {
    torque_observed = torque_read_back ? TorqueState::On : TorqueState::Off;
  }

  State joint_state;
  State actuator_state;  ///< Only used behind a transmission
  /// Only read() publishes the states, write() takes the commands and perform seeds them. Everything
  /// else works on joint_state, so a controller never reads a value written without the handle's lock.
  std::unordered_map<std::string, hardware_interface::StateInterface::SharedPtr> state_handles;
  std::unordered_map<std::string, hardware_interface::CommandInterface::SharedPtr> command_handles;
  std::unordered_map<std::string, double> read_goal_values;  ///< Goal registers as read back
  std::shared_ptr<transmission_interface::Transmission> state_transmission;
  std::shared_ptr<transmission_interface::Transmission> command_transmission;
  PassThrough state_pass_through;
  PassThrough command_pass_through;
  std::unordered_map<std::string, MimicState> mimic_joint_states;

  LED_State led_state;

private:
  [[nodiscard]] const rclcpp::Logger& dxlLogger() const
  {
    return dynamixel ? dynamixel->dxlLogger() : dynamixel_ros_control::dxlLogger();
  }
  ControlMode getControlModeFromInterfaces(const std::vector<std::string>& interfaces) const;
  bool initDefaultGoalValues();
  [[nodiscard]] bool checkVelocityBasedProfile() const;

  struct Cap
  {
    std::string interface_name;
    double floor;
    double ceiling;  ///< The EEPROM limit
  };

  ControlMode control_mode_{UNDEFINED};
  std::vector<std::string> active_command_interfaces_;
  std::unordered_map<std::string, double> default_goal_values_;
  std::vector<Cap> caps_;
  /// Actuator position latched by holdOnMissingCommand() until a command arrives or a goal reset.
  std::optional<double> missing_command_hold_;
  /// The last finite actuator position goal, for joints that keep their goal.
  std::optional<double> last_position_goal_;
  /// do_not_reset_on_ctrl_change: a controller change and a missing command keep the last goal
  /// instead of holding where the joint stands, so a gripper keeps its grasp.
  bool keeps_goal_on_controller_change_{false};
  bool exports_torque_off_{false};
  bool torque_off_claimed_{false};

  ControlMode preferred_position_control_mode_{POSITION};
  std::vector<std::string> state_interfaces_;
  std::vector<std::string> command_interfaces_;
  std::unordered_map<std::string, std::string> command_interface_to_register_;
  std::unordered_map<std::string, std::string> state_interface_to_register_;
  std::unordered_map<std::string, std::string> interface_to_register_limits_;
};

}  // namespace dynamixel_ros_control

#endif
