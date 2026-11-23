// Hardware-error diagnosis and reboot recovery for DynamixelHardwareInterface.
//
// A reboot wipes RAM, so recovery also restores the indirect-address pointers, the initial
// register values and each joint's torque intent. Any of those can fail with the arm energised.

#include "dynamixel_ros_control/common.hpp"
#include "dynamixel_ros_control/log.hpp"

#include <algorithm>
#include <iterator>
#include <thread>

#include <dynamixel_ros_control/dynamixel_hardware_interface.hpp>

namespace dynamixel_ros_control {

std::vector<std::string> DynamixelHardwareInterface::readJointsFlaggingHardwareError()
{
  std::set<Dynamixel*> connected;
  for (auto& [name, joint] : joints_) {
    if (!joint.dynamixel->isConnected()) {
      // Nothing to reboot: a motor that never answered a ping has no control table, and whatever
      // stops it answering (wrong id, no power, wrong port) a reboot instruction cannot reach it.
      DXL_LOG_WARN("[ID " << joint.dynamixel->getIdInt() << "] Joint '" << name
                          << "' has never connected. Skipping it.");
      continue;
    }
    connected.insert(joint.dynamixel.get());
  }

  // The state group carries hardware_error_status, so one group read asks every motor at once. A
  // reboot cuts a motor's torque, so a single dropped status packet must not decide it: a motor
  // that stays silent gets one more read. Before configuration has built the group, each motor is
  // asked directly instead.
  std::set<Dynamixel*> answered;
  if (groups_.state_read.isInitialized()) {
    answered = groups_.state_read.readWhoeverAnswers(connected);
    std::set<Dynamixel*> silent;
    std::set_difference(connected.begin(), connected.end(), answered.begin(), answered.end(),
                        std::inserter(silent, silent.end()));
    if (!silent.empty()) {
      const std::set<Dynamixel*> answered_on_retry = groups_.state_read.readWhoeverAnswers(silent);
      answered.insert(answered_on_retry.begin(), answered_on_retry.end());
    }
  } else {
    for (Dynamixel* dxl : connected) {
      int32_t status = OK;
      if (dxl->readRegister(DXL_REGISTER_HARDWARE_ERROR, status)) {
        dxl->hardware_error_status = status;
        answered.insert(dxl);
      }
    }
  }

  std::vector<std::string> joints_with_error;
  for (auto& [name, joint] : joints_) {
    if (connected.count(joint.dynamixel.get()) == 0) {
      continue;
    }
    if (answered.count(joint.dynamixel.get()) == 0) {
      // A motor that cannot be asked is not a motor that is healthy, so it is rebooted too: the
      // reboot instruction needs no answer, and a motor that came back is read again afterwards.
      DXL_LOG_WARN("[ID " << joint.dynamixel->getIdInt() << "] Could not read the hardware error status of joint '"
                          << name << "'. Rebooting it as well.");
      joints_with_error.push_back(name);
      continue;
    }
    const int32_t status = joint.dynamixel->hardware_error_status;
    if (status != OK) {
      DXL_LOG_WARN("Joint '" << name << "' reports hardware error: " << hardwareErrorToString(status)
                             << ", which no state read of this process had seen yet.");
      joints_with_error.push_back(name);
    }
  }
  return joints_with_error;
}

bool DynamixelHardwareInterface::rebootLocked()
{
  if (std::none_of(joints_.begin(), joints_.end(),
                   [](const auto& entry) { return entry.second.dynamixel->isConnected(); })) {
    DXL_LOG_ERROR("No motor on this bus has ever connected, so a reboot has nothing to address. "
                  "The port or the motor ids need fixing first.");
    return false;
  }
  std::vector<std::string> joints_to_reboot = getJointsWithHardwareError();
  if (joints_to_reboot.empty()) {
    // A process started against an errored motor may not have read its error yet, so ask the motors
    joints_to_reboot = readJointsFlaggingHardwareError();
  }
  if (joints_to_reboot.empty()) {
    return true;  // nothing to do. Do not disturb the armed watchdogs
  }

  // Cleared first: the sequence below sleeps and runs long register sequences, so an armed
  // watchdog on a torqued motor would trip and latch.
  if (!bus_watchdog_.clear()) {
    DXL_LOG_WARN("Failed to clear the bus watchdog before a reboot.");
  }

  std::vector<std::string> rebooted_joints;
  std::set<Dynamixel*> rebooted_dxls;
  for (const auto& name : joints_to_reboot) {
    auto& joint = joints_.at(name);
    if (!joint.dynamixel->reboot()) {
      DXL_LOG_ERROR("Dynamixel '" << name << "' reboot failed.");
      return false;
    }
    rebooted_joints.push_back(name);
    rebooted_dxls.insert(joint.dynamixel.get());
  }

  // Steady time, since a paused sim clock would hold the bus forever
  std::this_thread::sleep_for(REBOOT_WAIT);

  // Only motors that hold the pointer registers in RAM (X-series) need them rewritten. On
  // P-/PRO-series they live in EEPROM and survive the reboot.
  std::set<Dynamixel*> dxls_needing_rewrite;
  for (Dynamixel* dxl : rebooted_dxls) {
    if (dxl->indirectAddressesInRam()) {
      dxls_needing_rewrite.insert(dxl);
    }
  }
  if (!dxls_needing_rewrite.empty()) {
    DXL_LOG_DEBUG("Rewriting indirect address mappings for " << dxls_needing_rewrite.size() << " of "
                                                             << rebooted_dxls.size()
                                                             << " rebooted motors (RAM-resident pointers).");
    if (!groups_.rewriteIndirectAddresses(dxls_needing_rewrite)) {
      DXL_LOG_ERROR("Failed to restore indirect address mappings after reboot.");
      return false;
    }
  }
  // Fatal, not a warning: these writes are the only thing putting the configured limits, drive
  // mode and profiles back, and the torque restore below would re-energise on factory defaults.
  for (const auto& name : rebooted_joints) {
    if (!joints_.at(name).dynamixel->writeInitialValues()) {
      DXL_LOG_ERROR("Failed to restore initial register values for joint '"
                    << name << "' after reboot. Not restoring torque with unintended settings.");
      return false;
    }
  }

  if (!groups_.state_read.read() || !groups_.state_read.isOk()) {
    DXL_LOG_WARN("Failed to read hardware status after reboot, retrying...");
    std::this_thread::sleep_for(REBOOT_WAIT);
    if (!groups_.state_read.read() || !groups_.state_read.isOk()) {
      DXL_LOG_ERROR("Failed to read hardware status after reboot.");
      return false;
    }
  }
  if (!isHardwareOk()) {
    DXL_LOG_ERROR("Hardware still reports errors after reboot: " << describeHardwareErrors() << ".");
    return false;
  }

  // The error the last securing pass answered is resolved
  for (auto& [name, joint] : joints_) {
    joint.securing.reset();
  }

  // Restore each joint's own desired torque state. A reboot leaves every motor de-energised.
  const std::set<std::string> want_on = jointsDesiringTorque(true);
  const std::set<std::string> want_off = jointsDesiringTorque(false);
  DXL_LOG_INFO("Restoring torque after reboot: ON for " << want_on.size() << " joint(s), OFF for " << want_off.size()
                                                        << ".");
  const bool on_ok = setTorqueForJoints(true, want_on).fullSuccess();
  // Locals rather than a chained ||: short-circuiting would skip the torque-OFF pass whenever the
  // ON pass failed, which is when de-energising the rest matters most.
  const bool off_ok = setTorqueForJoints(false, want_off).fullSuccess();
  if (!on_ok || !off_ok) {
    DXL_LOG_ERROR("Failed to restore torque state after reboot.");
    return false;
  }

  updateLeds();
  // Re-published because a reboot may have changed EEPROM state, e.g. drive_mode via writeInitialValues
  diagnostics_->publishManifest(node_->now());
  return true;
}
}  // namespace dynamixel_ros_control
