// Securing the hardware when on_error gives up the component: bus work over many motors that can
// partly fail and is only ever proven by read-back.

#include "dynamixel_ros_control/common.hpp"
#include "dynamixel_ros_control/log.hpp"

#include <algorithm>
#include <cmath>

#include <dynamixel_ros_control/dynamixel_hardware_interface.hpp>

namespace dynamixel_ros_control {

namespace {

/// Securing passes a joint gets before it is handed the statusless torque-off. More than one
/// because a dropped status packet is ordinary on the degraded bus securing runs over.
constexpr int SECURING_ATTEMPTS = 3;

/// Passes a motor may go unanswered before securing treats it as gone. Retrying a silent motor
/// spends a pipeline of serial timeouts to reach the same conclusion.
constexpr int UNANSWERED_ATTEMPTS_BEFORE_GIVING_UP = 2;

}  // namespace

bool DynamixelHardwareInterface::verifyGoal(const std::string& name, Joint& joint, const std::string& interface_name)
{
  const auto read_it = joint.read_goal_values.find(interface_name);
  const double target = joint.getActuatorState().goal[interface_name];
  if (read_it == joint.read_goal_values.end() || !goalReadBackMatches(target, read_it->second)) {
    DXL_LOG_DEBUG("Emergency goal (" << interface_name << ") for joint '" << name << "' did not verify.");
    return false;
  }
  return true;
}

DynamixelHardwareInterface::SecuringReport DynamixelHardwareInterface::buildSecuringReport() const
{
  SecuringReport report;
  for (const auto& [name, joint] : joints_) {
    if (joint.securing.outcome.has_value()) {
      report.by_outcome[*joint.securing.outcome].insert(name);
    } else {
      report.unresolved.insert(name);
    }
  }
  return report;
}

DynamixelHardwareInterface::SecuringReport DynamixelHardwareInterface::secureHardware()
{
  // Repeated until every joint has a verdict: silence only counts once it has persisted
  SecuringReport report = securingPass();
  for (int attempt = 1; attempt < SECURING_ATTEMPTS && !report.unresolved.empty(); ++attempt) {
    report = securingPass();
  }
  return report;
}

/// The joints one pass still works on, and those it gave up on during this pass
struct DynamixelHardwareInterface::SecuringPass
{
  /// What one joint needs in order to count as secured, decided from its own fresh registers
  struct Plan
  {
    std::string name;
    Joint* joint{nullptr};
    bool torque_was_on{false};
    /// Hold at the current position (the preferred outcome), rather than command zero in place
    bool hold_position{false};
    ControlMode hold_mode{UNDEFINED};  ///< Only meaningful with hold_position
    std::string verify_interface;      ///< The goal whose read-back proves the hold landed
  };
  std::map<Dynamixel*, Plan> pending;
  std::map<Dynamixel*, Plan> failed;

  [[nodiscard]] std::set<Dynamixel*> motors() const
  {
    std::set<Dynamixel*> motors;
    for (const auto& [dxl, plan] : pending) {
      motors.insert(dxl);
    }
    return motors;
  }

  void giveUp(Dynamixel* dxl)
  {
    const auto it = pending.find(dxl);
    if (it != pending.end()) {
      failed.emplace(dxl, std::move(it->second));
      pending.erase(it);
    }
  }

  void dropUnanswered(const std::set<Dynamixel*>& answered)
  {
    for (Dynamixel* dxl : motors()) {
      if (answered.count(dxl) == 0) {
        giveUp(dxl);
      }
    }
  }
};

DynamixelHardwareInterface::SecuringReport DynamixelHardwareInterface::securingPass()
{
  const Stopwatch securing_timer;
  const CommStats stats_at_start = driver_.getCommStats();

  SecuringPass pass = beginSecuringPass();
  observeForSecuring(pass);
  planSecuring(pass);
  switchModesForHold(pass);
  writeHoldGoals(pass);
  restoreHoldTorque(pass);
  classifySecured(pass);
  torqueOffFailed(pass);

  // No controller is active once the component is given up
  for (auto& [name, joint] : joints_) {
    joint.clearActiveCommandInterfaces();
  }
  if (!pass.pending.empty() || !pass.failed.empty()) {
    updateLeds();
  }

  if (throttleDue(last_securing_timing_log_, std::chrono::seconds(5))) {
    DXL_LOG_INFO("[timing] securing pass took " << static_cast<int>(securing_timer.elapsedMs()) << " ms for "
                                                << pass.pending.size() + pass.failed.size() << " joint(s) ("
                                                << driver_.getCommStats().delta(stats_at_start).toString() << ")");
  }

  // The one place that decides whether the hardware counts as secured
  const SecuringReport report = buildSecuringReport();
  if (!report.fullySecured() && throttleDue(last_unsecured_log_, std::chrono::seconds(5))) {
    DXL_LOG_ERROR("Securing has not fully secured the hardware. Refused the torque-off and still "
                  "energised: "
                  << iterableToString(report.of(SecuringOutcome::TorqueOffRejected)) << "; unverified torque-off: "
                  << iterableToString(report.of(SecuringOutcome::TorqueOffSentUnverified))
                  << "; unreachable: " << iterableToString(report.of(SecuringOutcome::TransmitFailed))
                  << "; still retrying: " << iterableToString(report.unresolved));
  }
  return report;
}

DynamixelHardwareInterface::SecuringPass DynamixelHardwareInterface::beginSecuringPass()
{
  // Every joint not yet verified safe. One that is not proven safe may still be live.
  SecuringPass pass;
  for (auto& [name, joint] : joints_) {
    if (joint.securing.outcome.has_value()) {
      if (isSecured(*joint.securing.outcome)) {
        continue;
      }
      // A fresh budget for this attempt. What the episode observed survives, a retry must not
      // re-derive it from a bus that has since changed.
      joint.securing.attempts = 0;
      joint.securing.unanswered = 0;
    }
    joint.securing.outcome.reset();
    ++joint.securing.attempts;
    SecuringPass::Plan plan;
    plan.name = name;
    plan.joint = &joint;
    pass.pending.emplace(joint.dynamixel.get(), std::move(plan));
  }
  return pass;
}

void DynamixelHardwareInterface::observeForSecuring(SecuringPass& pass)
{
  // Fresh state, and proof of which motors still answer
  const std::set<Dynamixel*> answered = groups_.state_read.readWhoeverAnswers(pass.motors());
  for (auto& [dxl, plan] : pass.pending) {
    plan.joint->securing.unanswered = answered.count(dxl) != 0 ? 0 : plan.joint->securing.unanswered + 1;
  }
  pass.dropUnanswered(answered);

  // A latched watchdog discards every goal write, so the hold goals would fail to verify and hand
  // the torque-off to joints that could have held. Withholding goals is what trips it, so a trip
  // here is ordinary. Cleared, not re-armed, since nothing feeds it once the component is given up.
  if (!bus_watchdog_.trippedJoints().empty() && !bus_watchdog_.clear()) {
    DXL_LOG_DEBUG("Could not clear a tripped bus watchdog before writing the emergency hold goals.");
  }

  pass.dropUnanswered(readTorqueStates(pass.motors()));
  for (auto& [dxl, plan] : pass.pending) {
    Joint& joint = *plan.joint;
    if (!joint.securing.torque_was_on.has_value()) {
      joint.securing.torque_was_on = joint.torque_observed == TorqueState::On;
    }
    plan.torque_was_on = *joint.securing.torque_was_on;
  }
}

void DynamixelHardwareInterface::planSecuring(SecuringPass& pass)
{
  for (Dynamixel* dxl : pass.motors()) {
    SecuringPass::Plan& plan = pass.pending.at(dxl);
    Joint& joint = *plan.joint;

    // Read fresh rather than trusting the cache: this runs on paths where a mode write just failed.
    // operating_mode lives in EEPROM, which indirect addresses cannot reach.
    if (!joint.readControlMode()) {
      pass.giveUp(dxl);
      continue;
    }

    const auto& available = joint.getAvailableCommandInterfaces();
    if (std::find(available.begin(), available.end(), hardware_interface::HW_IF_POSITION) != available.end()) {
      // Preferred: hold station, with position the sole active interface so the goal reset holds
      joint.clearActiveCommandInterfaces();
      (void) joint.addActiveCommandInterface(hardware_interface::HW_IF_POSITION);
      plan.hold_mode = joint.getPreferredPositionControlMode();
      plan.hold_position = true;
      plan.verify_interface = hardware_interface::HW_IF_POSITION;
      continue;
    }

    // Otherwise zero in the mode the motor is in. An allowlist, because resetGoalState()
    // understands only these and would silently do nothing for e.g. PWM.
    const char* zero_interface = nullptr;
    switch (joint.getControlMode()) {
      case VELOCITY:
        zero_interface = hardware_interface::HW_IF_VELOCITY;
        break;
      case CURRENT:
        zero_interface = hardware_interface::HW_IF_CURRENT;
        break;
      default:
        break;
    }
    if (zero_interface == nullptr || std::find(available.begin(), available.end(), zero_interface) == available.end()) {
      DXL_LOG_DEBUG("Joint '" << plan.name << "' has no position interface and no zero command for mode "
                              << joint.getControlMode() << ". Falling back to torque-off.");
      pass.giveUp(dxl);
      continue;
    }
    plan.hold_position = false;
    plan.verify_interface = zero_interface;
  }
}

void DynamixelHardwareInterface::switchModesForHold(SecuringPass& pass)
{
  // The mode register only accepts writes without torque, so only the joints whose mode must change
  // go limp, and only once that is proven. A joint that goes limp under gravity falls.
  std::set<Dynamixel*> to_switch_mode;
  for (auto& [dxl, plan] : pass.pending) {
    if (plan.hold_position && plan.joint->getControlMode() != plan.hold_mode) {
      plan.joint->torque = false;
      plan.joint->securing.de_energised = true;
      to_switch_mode.insert(dxl);
    }
  }
  if (to_switch_mode.empty()) {
    return;
  }
  if (!groups_.torque_write.writeSelected(to_switch_mode)) {
    DXL_LOG_DEBUG("Emergency torque-off could not be transmitted; falling back to a per-motor "
                  "torque-off for every joint that was to change mode.");
    for (Dynamixel* dxl : to_switch_mode) {
      pass.giveUp(dxl);
    }
    return;
  }
  (void) readTorqueStates(to_switch_mode);
  for (Dynamixel* dxl : to_switch_mode) {
    // Unknown may be live, so one rule covers the motor that reports torque on and the silent one
    const auto it = pass.pending.find(dxl);
    if (it != pass.pending.end() && torqueMayBeLive(it->second.joint->torque_observed)) {
      DXL_LOG_DEBUG("Joint '" << it->second.name << "' could not be proven de-energised, so its mode "
                              << "cannot be changed safely.");
      pass.giveUp(dxl);
    }
  }

  // Addressed rather than sync-written: the mode register answers with a status error while
  // torqued, and knowing which joint was refused is the whole value of the report. Proven limp just
  // above, so no second torque-off that a lost status packet could turn into a lost hold.
  for (Dynamixel* dxl : to_switch_mode) {
    const auto it = pass.pending.find(dxl);
    if (it != pass.pending.end() && !it->second.joint->writeControlMode(it->second.hold_mode, true, true)) {
      DXL_LOG_DEBUG("Failed to switch joint '" << it->second.name << "' to position mode during emergency hold.");
      pass.giveUp(dxl);
    }
  }
}

void DynamixelHardwareInterface::writeHoldGoals(SecuringPass& pass)
{
  // Read after the mode changes, which can transform the representation, and hold from that reading
  pass.dropUnanswered(groups_.state_read.readWhoeverAnswers(pass.motors()));
  for (auto& [dxl, plan] : pass.pending) {
    Joint& joint = *plan.joint;
    if (joint.state_transmission) {
      joint.stateToJoint();
    }
    joint.resetGoalState();  // hold position, or 0 for velocity/current, per the mode just read
  }

  if (!pass.pending.empty() && !groups_.control_write.writeSelected(pass.motors())) {
    DXL_LOG_DEBUG("Emergency hold goals could not be transmitted; falling back to torque-off for all of them.");
    for (Dynamixel* dxl : pass.motors()) {
      pass.giveUp(dxl);
    }
  }

  // The read is one transaction, the judgement stays per joint
  pass.dropUnanswered(groups_.goal_read.readWhoeverAnswers(pass.motors()));
  for (Dynamixel* dxl : pass.motors()) {
    const SecuringPass::Plan& plan = pass.pending.at(dxl);
    if (!verifyGoal(plan.name, *plan.joint, plan.verify_interface)) {
      pass.giveUp(dxl);
    }
  }
}

void DynamixelHardwareInterface::restoreHoldTorque(SecuringPass& pass)
{
  // Only where torque was and only for joints this episode de-energised. Rewriting torque a joint
  // already holds resets its observation to Unknown.
  std::set<Dynamixel*> to_energise;
  for (auto& [dxl, plan] : pass.pending) {
    if (plan.hold_position && plan.torque_was_on && plan.joint->securing.de_energised) {
      plan.joint->torque = true;
      to_energise.insert(dxl);
    }
  }
  if (to_energise.empty()) {
    return;
  }
  if (!groups_.torque_write.writeSelected(to_energise)) {
    for (Dynamixel* dxl : to_energise) {
      pass.giveUp(dxl);
    }
    return;
  }
  for (Dynamixel* dxl : to_energise) {
    pass.pending.at(dxl).joint->torque_observed = TorqueState::Unknown;  // statusless, verified next
  }
}

void DynamixelHardwareInterface::classifySecured(SecuringPass& pass)
{
  // Every joint that must end up holding is read back, not only the ones just re-energised: a
  // motor can de-energise itself mid-pass through an overload shutdown.
  std::set<Dynamixel*> to_verify_torque;
  for (const auto& [dxl, plan] : pass.pending) {
    if (plan.hold_position && plan.torque_was_on) {
      to_verify_torque.insert(dxl);
    }
  }
  (void) readTorqueStates(to_verify_torque);
  for (Dynamixel* dxl : pass.motors()) {
    const SecuringPass::Plan& plan = pass.pending.at(dxl);
    if (!plan.hold_position) {
      plan.joint->securing.outcome = SecuringOutcome::ZeroCommanded;
      continue;
    }
    // Started de-energised: it keeps its verified hold goal but resists nothing, so it is parked
    if (!plan.torque_was_on) {
      plan.joint->securing.outcome = SecuringOutcome::ParkedTorqueOff;
      continue;
    }
    switch (plan.joint->torque_observed) {
      case TorqueState::On:
        plan.joint->securing.outcome = SecuringOutcome::Holding;
        break;
      case TorqueState::Off:
        DXL_LOG_DEBUG("Joint '" << plan.name << "' did not re-energise for its emergency hold and would go limp.");
        pass.giveUp(dxl);
        break;
      case TorqueState::Unknown:
        DXL_LOG_DEBUG("Could not read back torque for joint '"
                      << plan.name << "' after writing its emergency hold goal, so it cannot be reported as holding.");
        pass.giveUp(dxl);
        break;
    }
  }
}

void DynamixelHardwareInterface::torqueOffFailed(SecuringPass& pass)
{
  // A failed joint keeps its budget until it runs out, then gets the statusless torque-off. One
  // motor at a time, which separates unreachable from reached-and-wrong.
  for (auto& [dxl, plan] : pass.failed) {
    Joint& joint = *plan.joint;
    const bool out_of_attempts = joint.securing.unanswered >= UNANSWERED_ATTEMPTS_BEFORE_GIVING_UP ||
                                 joint.securing.attempts >= SECURING_ATTEMPTS;
    if (!out_of_attempts) {
      continue;  // no verdict yet. The next pass tries again
    }
    joint.torque = false;
    if (!groups_.torque_write.writeSelected({dxl})) {
      joint.securing.outcome = SecuringOutcome::TransmitFailed;
      joint.torque_observed = TorqueState::Unknown;
      continue;
    }
    // The motor answered and is off, answered and is still energised, or did not answer. A
    // read-back that proves it live must not read as unverified, which would send the operator
    // looking for a communication fault on a motor that talks and refuses.
    bool torqued = true;
    if (dxl->readRegister(DXL_REGISTER_CMD_TORQUE, torqued)) {
      joint.torque_observed = torqued ? TorqueState::On : TorqueState::Off;
      joint.securing.outcome = torqued ? SecuringOutcome::TorqueOffRejected : SecuringOutcome::TorqueOffConfirmed;
    } else {
      joint.torque_observed = TorqueState::Unknown;
      joint.securing.outcome = SecuringOutcome::TorqueOffSentUnverified;
    }
  }
}

}  // namespace dynamixel_ros_control
