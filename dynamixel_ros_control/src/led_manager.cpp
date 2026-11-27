#include "dynamixel_ros_control/log.hpp"

#include <dynamixel_ros_control/led_manager.hpp>

#include <algorithm>
#include <cmath>

namespace dynamixel_ros_control {

namespace {

/// @brief One colour component as the register holds it.
///
/// An LED register is a single 0-255 byte, and the sync write's own conversion only clamps to int32
/// range. An out-of-range component would wrap and show an unrelated colour. Clamped here, where
/// the value is set, rather than in a pass before each write that another writer could skip.
double toByte(const double component)
{
  return static_cast<double>(std::clamp(std::lround(component), 0L, 255L));
}

void apply(Joint& joint, const LedManager::Color& color)
{
  joint.led_state.red = toByte(color.red);
  joint.led_state.green = toByte(color.green);
  joint.led_state.blue = toByte(color.blue);
}

constexpr LedManager::Color COLOR_PINK{255, 175, 193};  // unconfigured / inactive
constexpr LedManager::Color COLOR_GREEN{0, 255, 0};     // active, torque verified off (safe to touch)
constexpr LedManager::Color COLOR_BLUE{0, 0, 255};      // active, torque verified on
constexpr LedManager::Color COLOR_YELLOW{255, 255, 0};  // active, torque UNVERIFIABLE, assume live
constexpr LedManager::Color COLOR_RED{255, 0, 0};       // hardware error
}  // namespace

void LedManager::init(std::unordered_map<std::string, Joint>* joints, SyncWriteManager* led_write_manager,
                      const rclcpp::Logger& logger)
{
  logger_ = logger;
  joints_ = joints;
  led_write_manager_ = led_write_manager;
}

void LedManager::update(const bool active, const std::vector<std::string>& joints_with_error)
{
  // Unconfigured and inactive share a colour: what they have in common is that nothing is
  // commanding the joint, which is the only distinction the LEDs can carry at a glance.
  if (!active) {
    setAll(COLOR_PINK);
  } else {
    // Per joint, because torque is per joint: after a partial torque-off the arm really is green
    // in one place and blue in another, and a single colour for the whole bus had to lie about one
    // of them. Green is a claim that the motor is safe to touch, so only a verified Off earns it -
    // an unverifiable motor gets its own colour rather than borrowing that claim.
    for (auto& [name, joint] : *joints_) {
      switch (joint.torque_observed) {
        case TorqueState::Off:
          setJoint(name, COLOR_GREEN);
          break;
        case TorqueState::On:
          setJoint(name, COLOR_BLUE);
          break;
        case TorqueState::Unknown:
          setJoint(name, COLOR_YELLOW);
          break;
      }
    }
  }
  setErrorLeds(joints_with_error);

  if (!writeToHardware()) {
    DXL_LOG_ERROR("Failed to write LED colors for at least one joint.");
  }
}

bool LedManager::writeToHardware()
{
  // One indirect-addressed sync write updates every joint's LEDs in a single bus transaction, which
  // also works across a mixed bus (models with different LED register addresses map to the same
  // indirect data addresses). The values are already register-ready. See toByte().
  return led_write_manager_->write();
}

void LedManager::setErrorLeds(const std::vector<std::string>& joints_with_error)
{
  for (const auto& joint_name : joints_with_error) {
    setJoint(joint_name, COLOR_RED);
  }
}

void LedManager::setAll(const Color& color)
{
  for (auto& [name, joint] : *joints_) {
    apply(joint, color);
  }
}

void LedManager::setJoint(const std::string& joint_name, const Color& color)
{
  const auto it = joints_->find(joint_name);
  if (it == joints_->end()) {
    DXL_LOG_ERROR("Joint '" << joint_name << "' not found when setting LED");
    return;
  }
  apply(it->second, color);
}

}  // namespace dynamixel_ros_control
