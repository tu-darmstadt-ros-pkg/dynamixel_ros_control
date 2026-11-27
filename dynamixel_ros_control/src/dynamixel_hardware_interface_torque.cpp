// Torque switching and goal reset for DynamixelHardwareInterface.
//
// A sync write carries no status, so nothing treats "requested" as "applied": every change is
// verified per motor, and a torque-on that cannot be fully verified rolls the whole set back.

#include "dynamixel_ros_control/common.hpp"
#include "dynamixel_ros_control/log.hpp"

#include <algorithm>
#include <cmath>

#include <dynamixel_ros_control/dynamixel_hardware_interface.hpp>

namespace dynamixel_ros_control {

std::set<Dynamixel*> DynamixelHardwareInterface::readTorqueStates(const std::set<Dynamixel*>& motors)
{
  // torque_enable is part of the state group, so this is the same transaction the control loop
  // already makes. readWhoeverAnswers() falls back to per-motor probes once one goes silent.
  const std::set<Dynamixel*> answered = groups_.state_read.readWhoeverAnswers(motors);
  for (auto& [name, joint] : joints_) {
    Dynamixel* dxl = joint.dynamixel.get();
    if (motors.find(dxl) == motors.end()) {
      continue;
    }
    if (answered.find(dxl) == answered.end()) {
      // torque_read_back still holds whatever the last successful read left there. A motor that
      // stayed silent is unknown, not assumed off. Being wrong here means calling a live motor safe.
      joint.torque_observed = TorqueState::Unknown;
      continue;
    }
    joint.commitTorqueReadBack();
  }
  return answered;
}

TorqueChangeResult DynamixelHardwareInterface::writeAndVerifyTorque(const bool do_enable,
                                                                    const std::set<Dynamixel*>& motors)
{
  TorqueChangeResult result;
  std::set<Dynamixel*> pending = motors;

  // Read-back is the only source of truth, and readTorqueStates() records it as seen: a motor is
  // confirmed only when it answered *and* holds the requested value.
  const auto confirmed = [do_enable](const Joint& joint) {
    return joint.torque_observed == (do_enable ? TorqueState::On : TorqueState::Off);
  };

  for (int attempt = 0; attempt < torque_verify_retries_ && !pending.empty(); ++attempt) {
    // Set the desired value on the pending motors and transmit (transmit-only: this can reach a
    // motor whose status path is dead, which is the point for an emergency torque-off).
    for (auto& [name, joint] : joints_) {
      if (pending.find(joint.dynamixel.get()) != pending.end()) {
        joint.torque = do_enable;  // also the value the torque sync write encodes
      }
    }
    (void) groups_.torque_write.writeSelected(pending);

    // Verify the whole pending set in one read. Unanswered or wrong-value motors are retried.
    (void) readTorqueStates(pending);
    std::set<Dynamixel*> still_pending;
    for (auto& [name, joint] : joints_) {
      Dynamixel* dxl = joint.dynamixel.get();
      if (pending.find(dxl) == pending.end()) {
        continue;
      }
      if (confirmed(joint)) {
        result.confirmed.insert(name);
      } else {
        still_pending.insert(dxl);
      }
    }
    pending = std::move(still_pending);
  }

  // One final read, not a reuse of what the loop last saw: every motor here failed that
  // observation, so reusing it would turn a single missed read-back into a rollback of the set.
  const std::set<Dynamixel*> answered = readTorqueStates(pending);
  for (auto& [name, joint] : joints_) {
    Dynamixel* dxl = joint.dynamixel.get();
    if (pending.find(dxl) == pending.end()) {
      continue;
    }
    if (answered.find(dxl) == answered.end()) {
      result.unknown.insert(name);
    } else {
      (confirmed(joint) ? result.confirmed : result.rejected).insert(name);
    }
  }
  return result;
}

void DynamixelHardwareInterface::rollbackTorqueOn(const std::vector<std::string>& requested_names,
                                                  const std::set<Dynamixel*>& requested, TorqueChangeResult& result)
{
  // writeAndVerifyTorque() records what each motor actually read back, so torque_observed reflects
  // the rollback rather than an optimistic assumption.
  (void) writeAndVerifyTorque(false, requested);
  // The enable failed, so intent stays off for every requested joint. A reboot must not retry it
  // behind the operator's back.
  for (const auto& name : requested_names) {
    Joint& joint = joints_.at(name);
    joint.torque_desired = false;
    joint.securing.reset();
  }
  // The requested ON state was not achieved and the motors were just turned back off, so nothing is
  // confirmed ON. Move them to rejected so the report never claims energised motors.
  result.rejected.insert(result.confirmed.begin(), result.confirmed.end());
  result.confirmed.clear();
  updateLeds();
}

std::set<std::string> DynamixelHardwareInterface::jointsDesiringTorque(const bool desired) const
{
  std::set<std::string> names;
  for (const auto& [name, joint] : joints_) {
    if (joint.torque_desired == desired) {
      names.insert(name);
    }
  }
  return names;
}

TorqueChangeResult DynamixelHardwareInterface::setTorqueForAll(const bool do_enable)
{
  return setTorqueForJoints(do_enable, std::set<std::string>(joint_names_.begin(), joint_names_.end()));
}

TorqueChangeResult DynamixelHardwareInterface::setTorqueForJoints(const bool do_enable,
                                                                  const std::set<std::string>& targets)
{
  // An empty target set means nothing, never everything: a caller computing its targets must not
  // address the whole arm by passing none.
  if (targets.empty()) {
    return {};
  }

  std::set<Dynamixel*> requested;
  std::vector<std::string> requested_names;
  for (auto& [name, joint] : joints_) {
    if (targets.count(name) != 0) {
      requested.insert(joint.dynamixel.get());
      requested_names.push_back(name);
    }
  }
  if (requested_names.size() != targets.size()) {
    // Any unknown name rejects the whole request: dropping it silently would have fullSuccess()
    // report a change that never reached the joints the caller named.
    std::set<std::string> unknown = targets;
    for (const auto& name : requested_names) {
      unknown.erase(name);
    }
    DXL_LOG_ERROR("Torque change requested for joints " << iterableToString(targets) << ", of which "
                                                        << iterableToString(unknown)
                                                        << " do not belong to this interface. No torque was changed.");
    TorqueChangeResult unknown_joints;
    unknown_joints.rejected = targets;
    return unknown_joints;
  }

  // Skip the change when every requested motor already holds the desired state. A failed read
  // falls through to the write, so one silent motor blocks nothing.
  {
    const bool all_reads_ok = readTorqueStates(requested).size() == requested.size();
    bool all_in_desired_state = true;
    for (const auto& [name, joint] : joints_) {
      if (targets.count(name) == 0) {
        continue;
      }
      all_in_desired_state &= (joint.torque_observed == (do_enable ? TorqueState::On : TorqueState::Off));
    }
    if (all_reads_ok && all_in_desired_state) {
      DXL_LOG_INFO("All requested joints already have torque " << (do_enable ? "ENABLED" : "DISABLED") << ". Skipping.");
      for (const auto& name : requested_names) {
        Joint& joint = joints_.at(name);
        joint.torque_desired = do_enable;
        joint.securing.reset();
      }
      // The torque may have changed outside this call, as by the early torque off of a mode switch
      updateLeds();
      TorqueChangeResult already;
      already.confirmed = std::set<std::string>(requested_names.begin(), requested_names.end());
      return already;
    }
  }

  DXL_LOG_INFO((do_enable ? "Enabling" : "Disabling") << " motor torque.");

  // Fresh state and safe goals before energising, so a joint does not jump to a stale goal
  if (do_enable && !resetGoalStateAndVerify(requested_names, max_reset_and_verify_retries_)) {
    DXL_LOG_ERROR("Aborting torque-on: could not reset and verify goal state.");
    // Nothing was written, so rejected rather than unknown, which would suggest a comm failure
    TorqueChangeResult not_written;
    not_written.rejected = std::set<std::string>(requested_names.begin(), requested_names.end());
    return not_written;
  }

  TorqueChangeResult result = writeAndVerifyTorque(do_enable, requested);
  if (do_enable && !result.fullSuccess()) {
    // A partly energised set is a hazard, so the whole set goes back off
    DXL_LOG_ERROR("Torque-on incomplete (confirmed " << result.confirmed.size() << "/" << requested.size()
                                                     << "); rolling back to torque-off.");
    rollbackTorqueOn(requested_names, requested, result);
    return result;
  }
  for (const auto& name : requested_names) {
    Joint& joint = joints_.at(name);
    // Also after an unverified torque-off, so no reboot energises a joint sent limp
    joint.torque_desired = do_enable;
    // A deliberate torque change ends what the securing episode observed
    joint.securing.reset();
  }
  // An unverified torque-off leaves the motor Unknown, which still counts as live
  updateLeds();
  return result;
}

bool DynamixelHardwareInterface::goalReadBackMatches(const double target, const double read_value)
{
  if (!std::isfinite(read_value)) {
    return false;
  }
  // The firmware rejects a goal above its limit and never clamps it, so the tolerance only covers rounding
  return std::abs(read_value - target) <= goal_verify_tolerance_;
}

bool DynamixelHardwareInterface::resetGoalStateAndVerify(const std::vector<std::string>& joints, const int retries)
{
  int counter = 0;
  while (counter < retries) {
    const Stopwatch attempt_timer;
    if (resetGoalStateAndVerifyOnce(joints))
      return true;
    counter++;
    DXL_LOG_WARN("Failed to reset goal state and verify for all joints (attempt took "
                 << static_cast<int>(attempt_timer.elapsedMs()) << " ms). Retrying... (" << counter << " of " << retries
                 << ")");
  }
  return false;
}

bool DynamixelHardwareInterface::resetGoalStateAndVerifyOnce(const std::vector<std::string>& joints)
{
  // Every transaction below addresses exactly the joints being reset. A motor outside that set is
  // neither asked to answer nor written to, so an unreachable one cannot block a reset for the
  // rest. That is usually the joint an operator named in ignore_joints. When the set is every
  // joint, the cyclic groups do it in one transaction and keep feeding the
  // consecutive-error counters that the connection-lost detection reads.
  std::set<Dynamixel*> targets;
  for (const auto& name : joints) {
    targets.insert(joints_.at(name).dynamixel.get());
  }
  const bool resetting_all = targets.size() == joints_.size();

  const auto read_state = [&] {
    return resetting_all ? groups_.state_read.read() && groups_.state_read.isOk() :
                           groups_.state_read.readSelected(targets);
  };
  const auto read_goals = [&] {
    return resetting_all ? groups_.goal_read.read() && groups_.goal_read.isOk() :
                           groups_.goal_read.readSelected(targets);
  };

  // Read current values (positions, velocities, etc.) before enabling torque
  if (!read_state()) {
    DXL_LOG_ERROR("[resetGoalStateAndVerify] Failed to read current values from actuators.");
    return false;
  }
  if (!isHardwareOk()) {
    DXL_LOG_ERROR("[resetGoalStateAndVerify] Hardware error on " << describeHardwareErrors() << ".");
    return false;
  }

  // reset goal state -> goal position = current position, goal velocity = 0, etc.
  for (const auto& name : joints) {
    Joint& joint = joints_.at(name);
    if (joint.state_transmission) {
      joint.stateToJoint();
    }
    joint.resetGoalState();
  }

  // Write goal values
  const bool written = resetting_all ? groups_.control_write.write() && groups_.control_write.isOk() :
                                       groups_.control_write.writeSelected(targets);
  if (!written) {
    DXL_LOG_ERROR("[resetGoalStateAndVerify] Failed to write reset goal values.");
    return false;
  }

  // Re-read goal values for verification
  if (!read_goals()) {
    DXL_LOG_ERROR("[resetGoalStateAndVerify] Failed to re-read goal.");
    return false;
  }

  // The rule lives in goalReadBackMatches(), shared with the emergency hold, so a goal is judged
  // the same way in both places.
  for (const auto& name : joints) {
    auto& joint = joints_.at(name);
    for (const auto& interface_name : joint.getAvailableCommandInterfaces()) {
      if (joint.read_goal_values.count(interface_name) == 0) {
        DXL_LOG_ERROR("[resetGoalStateAndVerify]  Cannot verify cmd values from motor " << name << "!");
        return false;
      }
      const auto& interface_value = joint.read_goal_values.at(interface_name);
      if (!goalReadBackMatches(joint.getActuatorState().goal[interface_name], interface_value)) {
        DXL_LOG_ERROR("[resetGoalStateAndVerify] Joint '"
                      << name << "' goal of interface " << interface_name << " does not match read goal value. "
                      << "(Target Goal Value: " << joint.getActuatorState().goal[interface_name]
                      << ", Read Goal Value: " << interface_value);
        return false;
      }
    }
  }
  return true;
}
}  // namespace dynamixel_ros_control
