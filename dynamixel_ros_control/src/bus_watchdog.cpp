#include "dynamixel_ros_control/log.hpp"

#include <algorithm>
#include <cmath>
#include <set>

#include <dynamixel_ros_control/bus_watchdog.hpp>

namespace dynamixel_ros_control {

int32_t busWatchdogTicks(const double cycles, const double dt_ms, const double ms_per_tick)
{
  if (cycles == 0.0 || dt_ms <= 0.0 || ms_per_tick <= 0.0) {
    return 0;
  }
  const double ticks =
      std::clamp(std::ceil(dt_ms * cycles / ms_per_tick), static_cast<double>(DXL_BUS_WATCHDOG_MIN_TICKS),
                 static_cast<double>(DXL_BUS_WATCHDOG_MAX_TICKS));
  return static_cast<int32_t>(ticks);
}

void BusWatchdog::init(std::unordered_map<std::string, Joint>* joints, SyncWriteManager* write_manager,
                       const rclcpp::Logger& logger)
{
  logger_ = logger;
  joints_ = joints;
  write_manager_ = write_manager;
}

const std::set<Dynamixel*>& BusWatchdog::capableMotors() const
{
  return capable_;
}

double BusWatchdog::shortestWindowMs() const
{
  return shortest_window_ms_;
}

bool BusWatchdog::silenceThreatensWindow(const double silence_ms) const
{
  // A window of 0 is a disabled watchdog, which arm() still records as intended
  return arm_intended_ && shortest_window_ms_ > 0.0 && silence_ms > 0.5 * shortest_window_ms_;
}

bool BusWatchdog::configure(const double cycles, const unsigned int rw_rate)
{
  capable_.clear();
  arm_intended_ = false;  // nothing has been armed on this configuration yet
  shortest_window_ms_ = 0.0;
  for (auto& [name, joint] : *joints_) {
    joint.bus_watchdog_ticks = 0;
    if (joint.dynamixel->registerAvailable(DXL_REGISTER_BUS_WATCHDOG)) {
      capable_.insert(joint.dynamixel.get());
    }
  }

  if (cycles < 0.0 || (cycles != 0.0 && cycles < MIN_BUS_WATCHDOG_WINDOW_CYCLES)) {
    DXL_LOG_ERROR("bus_watchdog_cycles=" << cycles << " is invalid. Use 0 to disable the watchdog, or at "
                                         << "least " << MIN_BUS_WATCHDOG_WINDOW_CYCLES
                                         << " cycles: a shorter window trips on ordinary scheduling jitter and stops "
                                            "the motor until the next cycle notices.");
    return false;
  }
  if (cycles == 0.0) {
    DXL_LOG_INFO("Bus watchdog disabled by configuration (bus_watchdog_cycles=0).");
    return true;
  }
  if (rw_rate == 0) {
    // The window is derived from the control period, so without one there is nothing to arm. A
    // configuration that asked for a watchdog and would silently get none is refused instead: the
    // motors would take goals with nothing left to stop them if the goals stopped coming.
    DXL_LOG_ERROR("bus_watchdog_cycles is set to "
                  << cycles << " but the control rate is 0, so no watchdog window can be derived. "
                  << "The motors would run unprotected.");
    return false;
  }

  const double dt_ms = 1000.0 / static_cast<double>(rw_rate);
  for (auto& [name, joint] : *joints_) {
    if (capable_.count(joint.dynamixel.get()) == 0) {
      continue;
    }
    const double ms_per_tick = joint.dynamixel->getItem(DXL_REGISTER_BUS_WATCHDOG).dxlValueToUnitRatio();
    joint.bus_watchdog_ticks = busWatchdogTicks(cycles, dt_ms, ms_per_tick);
    const double window_ms = joint.bus_watchdog_ticks * ms_per_tick;

    // Ticks round up, so the only way the window lands short is the register's own maximum. Refuse
    // rather than arm below the jitter floor, where the motor trips on a healthy bus.
    if (window_ms < MIN_BUS_WATCHDOG_WINDOW_CYCLES * dt_ms) {
      DXL_LOG_ERROR("Bus watchdog for joint '" << name << "' cannot cover " << MIN_BUS_WATCHDOG_WINDOW_CYCLES
                                               << " control cycles (" << MIN_BUS_WATCHDOG_WINDOW_CYCLES * dt_ms
                                               << " ms): the register saturates at " << DXL_BUS_WATCHDOG_MAX_TICKS
                                               << " ticks of " << ms_per_tick << " ms, giving at most " << window_ms
                                               << " ms. Raise the control rate, or set bus_watchdog_cycles=0 to run "
                                               << "without the watchdog on this bus.");
      return false;
    }
    if (shortest_window_ms_ == 0.0 || window_ms < shortest_window_ms_) {
      shortest_window_ms_ = window_ms;
    }
    DXL_LOG_INFO("Bus watchdog for joint '" << name << "' configured to " << joint.bus_watchdog_ticks << " ticks ("
                                            << window_ms << " ms, dt=" << dt_ms << " ms). Armed on activation.");
  }
  // Older PRO motors have no watchdog, so velocity mode is refused on them unless explicitly allowed
  if (const auto unprotected = unprotectedJoints(); !unprotected.empty()) {
    DXL_LOG_WARN("bus_watchdog_cycles is " << cycles << ", but the motors of " << iterableToString(unprotected)
                                           << " have no bus watchdog. Nothing stops them if the bus goes silent.");
  }
  return true;
}

bool BusWatchdog::writeAll(const bool armed)
{
  // Recorded before the transmit, and regardless of its result. It says what the window is meant
  // to be doing from here on. A failed transmit leaves that unachieved, and the control loop's
  // convergence check repairs it.
  arm_intended_ = armed;
  if (capable_.empty()) {
    return true;  // no motor on this bus has the register
  }
  for (auto& [name, joint] : *joints_) {
    if (capable_.count(joint.dynamixel.get()) != 0) {
      joint.bus_watchdog_command = armed ? joint.bus_watchdog_ticks : 0;
    }
  }
  // The cyclic group holds exactly the capable motors and nothing else writes through it, so the
  // allocation-free path is available here. writeSelected() would build a temporary group and
  // re-add every parameter, and it exists to keep a cyclic caller's error counter undisturbed.
  // That does not apply to a manager with no cyclic caller.
  return write_manager_->write();
}

bool BusWatchdog::clear()
{
  return writeAll(false);
}

bool BusWatchdog::arm()
{
  return writeAll(true);
}

bool BusWatchdog::armIntended() const
{
  return arm_intended_;
}

bool BusWatchdog::readsArmed() const
{
  for (const auto& [name, joint] : *joints_) {
    if (joint.bus_watchdog_ticks > 0 && joint.dynamixel->bus_watchdog_value != joint.bus_watchdog_ticks) {
      return false;
    }
  }
  return true;
}

std::vector<std::string> BusWatchdog::unprotectedJoints() const
{
  std::vector<std::string> names;
  for (const auto& [name, joint] : *joints_) {
    if (joint.bus_watchdog_ticks <= 0) {
      names.push_back(name);
    }
  }
  std::sort(names.begin(), names.end());
  return names;
}

std::vector<std::string> BusWatchdog::trippedJoints() const
{
  std::vector<std::string> names;
  for (const auto& [name, joint] : *joints_) {
    if (joint.bus_watchdog_ticks > 0 && joint.dynamixel->bus_watchdog_value < 0) {
      names.push_back(name);
    }
  }
  std::sort(names.begin(), names.end());
  return names;
}

std::vector<std::string> BusWatchdog::jointsNotReading(const bool armed, const std::set<Dynamixel*>* only) const
{
  std::vector<std::string> names;
  for (const auto& [name, joint] : *joints_) {
    if (only != nullptr && only->count(joint.dynamixel.get()) == 0) {
      continue;  // not part of what the caller is answering for
    }
    if (joint.bus_watchdog_ticks <= 0) {
      continue;  // disabled for this joint, or unsupported by this model
    }
    const int32_t expected = armed ? joint.bus_watchdog_ticks : 0;
    if (joint.dynamixel->bus_watchdog_value != expected) {
      names.push_back(name);
    }
  }
  std::sort(names.begin(), names.end());
  return names;
}

std::vector<std::string> BusWatchdog::unarmedJoints(const std::set<Dynamixel*>* only) const
{
  return jointsNotReading(true, only);
}

std::vector<std::string> BusWatchdog::unclearedJoints(const std::set<Dynamixel*>* only) const
{
  return jointsNotReading(false, only);
}

}  // namespace dynamixel_ros_control
