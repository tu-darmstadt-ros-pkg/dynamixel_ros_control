#include <dynamixel_ros_control/mock_dynamixel.hpp>

#include <ament_index_cpp/get_package_share_path.hpp>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

namespace dynamixel_ros_control {

namespace {

// The kinematic model, not datasheet values
constexpr double AT_GOAL_POSITION = 1e-4;         // rad, closer is at the goal
constexpr double AT_REST_VELOCITY = 1e-3;         // rad/s, slower is at rest and not moving
constexpr double AMPERE_PER_ACCELERATION = 0.1;   // A per rad/s^2, the current a profile draws
constexpr double ACCELERATION_PER_AMPERE = 10.0;  // rad/s^2 per A in current mode

// The trimmed fields of an "a | b | c" control table line
std::vector<std::string> splitTableLine(const std::string& line)
{
  std::vector<std::string> parts;
  std::stringstream ss(line);
  std::string segment;
  while (std::getline(ss, segment, '|')) {
    const size_t start = segment.find_first_not_of(' ');
    const size_t end = segment.find_last_not_of(' ');
    if (start != std::string::npos) {
      parts.push_back(segment.substr(start, end - start + 1));
    } else {
      parts.push_back("");
    }
  }
  return parts;
}

/// The profile, else the limit, 0 leaving either unset
double profileOrLimit(const double profile, const double limit, const double unset)
{
  if (profile > 0.0) {
    return profile;
  }
  return limit > 0.0 ? limit : unset;
}

/// Series whose Profile Velocity may not exceed Velocity Limit (P series and RH-P12-RN(A) e-Manuals).
/// X and MX bound only the goal velocity by it.
const std::set<std::string> PROFILE_VELOCITY_BOUNDED_SERIES = {
    "PH", "PM", "PH42-020-S300-R", "PM42-010-S260-R", "H42-20-S300-RA", "RH_P12_RNA"};

/// Series whose Goal Current (the PRO's Goal Torque) of 0 leaves the cap at the limit register (PRO
/// H54-200-S500-R e-Manual). The others hold no current at 0.
const std::set<std::string> ZERO_GOAL_CURRENT_UNCAPPED_SERIES = {"PROExt"};

}  // namespace

MockDynamixel::MockDynamixel(const uint8_t id, const uint16_t model_number)
    : id_(id), model_number_(model_number)
{
  // Covers the control table of every model
  memory_.resize(2048, 0);
  loadModelSpecs(model_number);
  write2Byte(getAddress("model_number"), model_number);
  write1Byte(getAddress("id"), id);
  write1Byte(getAddress("version_of_firmware"), 1);
  resetIndirectPointers();

  // The factory Shutdown bits PH (52) and PRO (58) agree on, which cover every error the tests
  // inject to shut a motor down
  constexpr uint8_t DEFAULT_SHUTDOWN = ERROR_OVERHEATING | ERROR_ELECTRICAL_SHOCK | ERROR_OVERLOAD;
  if (shutdown_addr_ > 0) {
    write1Byte(shutdown_addr_, DEFAULT_SHUTDOWN);
  }
}

void MockDynamixel::update(const double dt)
{
  if (dt <= 0)
    return;

  // The whole step, as tests read the kinematic state from their own thread
  const std::lock_guard<std::recursive_mutex> lock(memory_mutex_);

  bool torque_enabled = false;
  if (torque_enable_addr_ > 0) {
    torque_enabled = read1Byte(torque_enable_addr_) == 1;
  }

  if (torque_enabled && bus_watchdog_addr_ > 0 && MockDynamixelManager::instance().busWatchdogTimed()) {
    // A positive value is a running window, -1 a latched trip
    const auto ticks = static_cast<int8_t>(read1Byte(bus_watchdog_addr_));
    bus_silence_s_ += dt;
    if (ticks > 0 && bus_silence_s_ * 1000.0 > ticks * perTick("bus_watchdog")) {
      tripBusWatchdog();
      ++timed_bus_watchdog_trips_;
    }
  }

  if (!torque_enabled) {
    // The encoder reports a limp motor too, one turned by hand or back from a power cut
    current_velocity_ = 0.0;
    current_current_ = 0.0;
    writePresentValues();
    return;
  }

  OperatingMode mode = OperatingMode::POSITION_CONTROL;
  if (operating_mode_addr_ > 0) {
    mode = static_cast<OperatingMode>(read1Byte(operating_mode_addr_));
  }

  switch (mode) {
    case OperatingMode::POSITION_CONTROL:
      updatePositionMode(dt, true);
      break;
    case OperatingMode::EXTENDED_POSITION_CONTROL:
    case OperatingMode::CURRENT_BASED_POSITION_CONTROL:
      // Multi turn, without the position limits
      updatePositionMode(dt, false);
      break;
    case OperatingMode::VELOCITY_CONTROL:
      updateVelocityMode(dt);
      break;
    case OperatingMode::CURRENT_CONTROL:
      updateCurrentMode(dt);
      break;
    default:
      // PWM has no model, it moves as position mode
      updatePositionMode(dt, true);
      break;
  }

  updateMovingFlag();

  writePresentValues();
}

ServoCommand MockDynamixel::followPlant(const double dt, const double position, const double velocity,
                                        const double current)
{
  const std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  // The kinematic state lives in the motor's frame, the register adds homing to it
  const double homing = getHomingOffset() * rad_per_tick_;
  const double measured = position - homing;
  update(dt);
  ServoCommand command;
  command.torque_on = torque_enable_addr_ > 0 && read1Byte(torque_enable_addr_) == 1;
  if (!command.torque_on) {
    current_position_ = measured;
    current_velocity_ = velocity;
  } else if (operating_mode_addr_ > 0 &&
             static_cast<OperatingMode>(read1Byte(operating_mode_addr_)) == OperatingMode::VELOCITY_CONTROL) {
    // A blocked joint must not wind up a position error that snaps back once it is free
    current_position_ = measured;
  }
  writePresentValues(measured, velocity, current);
  command.position = current_position_ + homing;
  command.velocity = current_velocity_;
  command.current_limit = std::numeric_limits<double>::infinity();
  if (current_limit_addr_ > 0) {
    command.current_limit = read2Byte(current_limit_addr_) * ampere_per_tick_;
  }
  if (goal_current_addr_ > 0) {
    const double goal = std::abs(static_cast<int16_t>(read2Byte(goal_current_addr_))) * ampere_per_tick_;
    if (goal > 0.0 || !zero_goal_current_uncapped_) {
      command.current_limit = std::min(command.current_limit, goal);
    }
  }
  return command;
}

void MockDynamixel::setFactoryValue(const std::string& register_name, const double value)
{
  const double per_tick = perTick(register_name);
  if (per_tick == 0.0) {
    throw std::invalid_argument("Model " + std::to_string(model_number_) + " has no register '" + register_name +
                                "' with a unit conversion");
  }
  const auto ticks = static_cast<uint32_t>(std::lround(value / per_tick));
  const uint16_t address = getAddress(register_name);
  switch (getLength(register_name)) {
    case 1:
      write1Byte(address, static_cast<uint8_t>(ticks));
      break;
    case 2:
      write2Byte(address, static_cast<uint16_t>(ticks));
      break;
    default:
      write4Byte(address, ticks);
      break;
  }
}

void MockDynamixel::loadModelSpecs(const uint16_t model_number)
{
  const std::string devices = ament_index_cpp::get_package_share_path("dynamixel_ros_control").string() + "/devices/";
  try {
    const YAML::Node series = YAML::LoadFile(devices + "model_list.yaml")[model_number];
    if (!series) {
      throw std::runtime_error("it is not in devices/model_list.yaml");
    }
    const YAML::Node model_node = YAML::LoadFile(devices + "models/" + series.as<std::string>() + ".yaml");
    profile_velocity_bounded_ = PROFILE_VELOCITY_BOUNDED_SERIES.count(series.as<std::string>()) != 0;
    zero_goal_current_uncapped_ = ZERO_GOAL_CURRENT_UNCAPPED_SERIES.count(series.as<std::string>()) != 0;

    if (model_node["unit_conversions"]) {
      auto conversions = model_node["unit_conversions"];
      if (conversions["rad"])
        rad_per_tick_ = conversions["rad"].as<double>();
      if (conversions["rad_per_s"])
        rad_per_s_per_tick_ = conversions["rad_per_s"].as<double>();
      if (conversions["A"])
        ampere_per_tick_ = conversions["A"].as<double>();
      for (const auto& conversion : conversions) {
        unit_conversions_[conversion.first.as<std::string>()] = conversion.second.as<double>();
      }
    }

    auto control_table = model_node["control_table"];
    for (const auto& item : control_table) {
      // address | name | length | access | memory | unit
      const std::vector<std::string> parts = splitTableLine(item.as<std::string>());

      if (parts.size() >= 3) {
        int addr = std::stoi(parts[0]);
        std::string name = parts[1];
        int len = std::stoi(parts[2]);

        address_map_[name] = addr;
        length_map_[name] = len;
        if (parts.size() >= 6) {
          unit_map_[name] = parts[5];
        }

        if (parts.size() >= 5 && parts[4] == "RAM") {
          ram_registers_.emplace_back(static_cast<uint16_t>(addr), static_cast<uint8_t>(len));
        } else if (parts.size() >= 5 && parts[4] == "EEPROM") {
          eeprom_registers_.emplace_back(static_cast<uint16_t>(addr), static_cast<uint8_t>(len));
        }
      }
    }

    static const std::pair<const char*, uint16_t MockDynamixel::*> CACHED[] = {
        {"torque_enable", &MockDynamixel::torque_enable_addr_},
        {"operating_mode", &MockDynamixel::operating_mode_addr_},
        {"goal_position", &MockDynamixel::goal_position_addr_},
        {"goal_velocity", &MockDynamixel::goal_velocity_addr_},
        {"goal_torque", &MockDynamixel::goal_current_addr_},
        {"goal_current", &MockDynamixel::goal_current_addr_},
        {"goal_pwm", &MockDynamixel::goal_pwm_addr_},
        {"present_position", &MockDynamixel::present_position_addr_},
        {"present_velocity", &MockDynamixel::present_velocity_addr_},
        {"present_current", &MockDynamixel::present_current_addr_},
        {"current_limit", &MockDynamixel::current_limit_addr_},
        {"velocity_limit", &MockDynamixel::velocity_limit_addr_},
        {"acceleration_limit", &MockDynamixel::acceleration_limit_addr_},
        {"hardware_error_status", &MockDynamixel::hardware_error_addr_},
        {"moving", &MockDynamixel::moving_addr_},
        {"profile_velocity", &MockDynamixel::profile_velocity_addr_},
        {"profile_acceleration", &MockDynamixel::profile_acceleration_addr_},
        {"max_position_limit", &MockDynamixel::max_position_limit_addr_},
        {"min_position_limit", &MockDynamixel::min_position_limit_addr_},
        {"homing_offset", &MockDynamixel::homing_offset_addr_},
        {"led_red", &MockDynamixel::led_red_addr_},
        {"led_green", &MockDynamixel::led_green_addr_},
        {"led_blue", &MockDynamixel::led_blue_addr_},
        {"bus_watchdog", &MockDynamixel::bus_watchdog_addr_},
        {"drive_mode", &MockDynamixel::drive_mode_addr_},
        {"shutdown", &MockDynamixel::shutdown_addr_}};
    for (const auto& [name, member] : CACHED) {
      if (const auto it = address_map_.find(name); it != address_map_.end()) {
        this->*member = it->second;
      }
    }

    if (model_node["indirect_addresses"]) {
      auto indirect = model_node["indirect_addresses"];
      if (indirect.size() > 1) {
        throw std::runtime_error("it declares " + std::to_string(indirect.size()) +
                                 " indirect address blocks and the mock models one");
      }
      if (indirect.size() > 0) {
        const std::vector<std::string> parts = splitTableLine(indirect[0].as<std::string>());
        if (parts.size() >= 4) {
          indirect_address_start_ = std::stoi(parts[0]);
          indirect_data_start_ = std::stoi(parts[1]);
          indirect_count_ = std::stoi(parts[2]);
          indirect_in_ram_ = (parts[3] == "RAM");
          if (!indirect_in_ram_) {
            // Not in the control table, but the EEPROM refusal covers them
            eeprom_registers_.emplace_back(indirect_address_start_,
                                           static_cast<uint8_t>(std::min<uint16_t>(indirect_count_ * 2, 255)));
          }
        }
      }
    }
  }
  catch (const std::exception& e) {
    throw std::runtime_error("Cannot simulate model " + std::to_string(model_number) + ", " + e.what());
  }
}

uint8_t MockDynamixel::read1Byte(const uint16_t address) const
{
  std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  uint16_t real_addr = resolveAddress(address);
  if (static_cast<size_t>(real_addr) >= memory_.size())
    return 0;
  return memory_[real_addr];
}

uint16_t MockDynamixel::read2Byte(const uint16_t address) const
{
  std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  return static_cast<uint16_t>(read1Byte(address)) | (static_cast<uint16_t>(read1Byte(address + 1)) << 8);
}

uint32_t MockDynamixel::read4Byte(const uint16_t address) const
{
  std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  return static_cast<uint32_t>(read1Byte(address)) | (static_cast<uint32_t>(read1Byte(address + 1)) << 8) |
         (static_cast<uint32_t>(read1Byte(address + 2)) << 16) | (static_cast<uint32_t>(read1Byte(address + 3)) << 24);
}

int32_t MockDynamixel::read4ByteSigned(const uint16_t address) const
{
  return static_cast<int32_t>(read4Byte(address));
}

void MockDynamixel::setRegisterWriteFault(const std::string& register_name, const RegisterWriteFault fault)
{
  std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  const uint16_t address = getAddress(register_name);
  if (address == 0) {
    return;  // register not present on this model, nothing to fault
  }
  const uint8_t length = std::max<uint8_t>(getLength(register_name), 1);
  for (uint8_t i = 0; i < length; ++i) {
    register_write_faults_[static_cast<uint16_t>(address + i)] = fault;
  }
}

void MockDynamixel::clearRegisterWriteFaults()
{
  std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  register_write_faults_.clear();
}
void MockDynamixel::setRegisterReadFault(const std::string& register_name)
{
  setRegisterReadFaultCount(register_name, PERSISTENT_READ_FAULT);
}

void MockDynamixel::setRegisterReadFaultCount(const std::string& register_name, const int count)
{
  std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  const uint16_t address = getAddress(register_name);
  if (address == 0) {
    return;  // register not present on this model, nothing to fault
  }
  if (count == 0) {
    return;  // nothing to fail
  }
  // Each byte with the count, as one read spends a use of each
  const uint8_t length = std::max<uint8_t>(getLength(register_name), 1);
  for (uint8_t i = 0; i < length; ++i) {
    register_read_faults_[static_cast<uint16_t>(address + i)] = count;
  }
}

bool MockDynamixel::consumeReadFault(const uint16_t address) const
{
  const auto it = register_read_faults_.find(address);
  if (it == register_read_faults_.end()) {
    return false;
  }
  if (it->second != PERSISTENT_READ_FAULT && --it->second <= 0) {
    register_read_faults_.erase(it);
  }
  return true;
}
void MockDynamixel::setRejectEepromWritesWhileTorqued(const bool reject)
{
  reject_eeprom_writes_while_torqued_ = reject;
}

uint8_t MockDynamixel::statusError(const uint8_t error) const
{
  return error | (hardware_error_ != 0 ? PACKET_ERROR_ALERT : 0);
}

uint8_t MockDynamixel::writeRefusal(const uint16_t address, const uint8_t* data, const uint16_t length) const
{
  const auto pointers_end = static_cast<uint16_t>(indirect_address_start_ + 2 * indirect_count_);
  for (uint16_t i = 0; i + 1 < length; ++i) {
    const auto at = static_cast<uint16_t>(address + i);
    if (at >= indirect_address_start_ && at < pointers_end && (at - indirect_address_start_) % 2 == 0 &&
        pointerOutOfRange(static_cast<uint16_t>(data[i] | (data[i + 1] << 8)))) {
      return PACKET_ERROR_DATA_RANGE;
    }
  }
  const bool torqued =
      reject_eeprom_writes_while_torqued_ && torque_enable_addr_ > 0 && memory_[torque_enable_addr_] != 0;
  for (uint16_t i = 0; i < length; ++i) {
    const auto at = static_cast<uint16_t>(address + i);
    if (torqued && std::any_of(eeprom_registers_.begin(), eeprom_registers_.end(), [at](const auto& eeprom) {
          return at >= eeprom.first && at < eeprom.first + eeprom.second;
        })) {
      return PACKET_ERROR_ACCESS;
    }
    if (busWatchdogLatched() && isGoalRegisterAddress(resolveAddress(at))) {
      return PACKET_ERROR_DATA_RANGE;
    }
  }
  // A limit of 0 is a mock without factory values, where the profile stays free
  const auto exceeds = [&](const uint16_t profile, const uint16_t limit) {
    if (profile == 0 || limit == 0 || isTimeBasedProfile() || read4ByteSigned(limit) <= 0) {
      return false;
    }
    uint8_t bytes[4];
    bool written = false;
    for (uint16_t k = 0; k < 4; ++k) {
      bytes[k] = memory_[profile + k];
      for (uint16_t i = 0; i < length; ++i) {
        if (resolveAddress(static_cast<uint16_t>(address + i)) == profile + k) {
          bytes[k] = data[i];
          written = true;
        }
      }
    }
    const auto value =
        static_cast<int32_t>(bytes[0] | (bytes[1] << 8) | (bytes[2] << 16) | (static_cast<uint32_t>(bytes[3]) << 24));
    return written && value > read4ByteSigned(limit);
  };
  if (exceeds(profile_acceleration_addr_, acceleration_limit_addr_) ||
      (profile_velocity_bounded_ && exceeds(profile_velocity_addr_, velocity_limit_addr_))) {
    return PACKET_ERROR_DATA_LIMIT;
  }
  return 0;
}

bool MockDynamixel::pointerOutOfRange(const uint16_t target) const
{
  const auto ram_area = ramArea();
  return ram_area && (target < ram_area->first || target >= ram_area->second);
}

std::optional<std::pair<uint16_t, uint16_t>> MockDynamixel::ramArea() const
{
  if (ram_registers_.empty()) {
    return std::nullopt;
  }
  uint16_t begin = std::numeric_limits<uint16_t>::max();
  uint16_t end = 0;
  for (const auto& [address, length] : ram_registers_) {
    begin = std::min(begin, address);
    end = std::max(end, static_cast<uint16_t>(address + std::max<uint8_t>(length, 1)));
  }
  return std::make_pair(begin, end);
}

void MockDynamixel::setRegisterReadHook(const std::string& register_name, std::function<void()> hook)
{
  std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  const uint16_t address = getAddress(register_name);
  if (address == 0) {
    return;  // register not present on this model, nothing to hook
  }
  const uint8_t length = std::max<uint8_t>(getLength(register_name), 1);
  for (uint8_t i = 0; i < length; ++i) {
    register_read_hooks_[static_cast<uint16_t>(address + i)] = hook;
  }
}

void MockDynamixel::clearRegisterReadHooks()
{
  std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  register_read_hooks_.clear();
}

void MockDynamixel::tripBusWatchdog()
{
  std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  if (bus_watchdog_addr_ == 0 || static_cast<size_t>(bus_watchdog_addr_) >= memory_.size()) {
    return;  // model without the register
  }
  // The firmware stops the motor: rate and force goals to zero, the position goal where it stands
  if (goal_velocity_addr_ > 0) {
    write4Byte(goal_velocity_addr_, 0);
  }
  if (goal_current_addr_ > 0) {
    write2Byte(goal_current_addr_, 0);
  }
  if (goal_position_addr_ > 0 && present_position_addr_ > 0) {
    write4Byte(goal_position_addr_, read4Byte(present_position_addr_));
  }
  memory_[bus_watchdog_addr_] = 0xFF;  // -1: the latched value
}

int MockDynamixel::timedBusWatchdogTrips() const
{
  std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  return timed_bus_watchdog_trips_;
}

void MockDynamixel::hearInstruction(const bool is_read)
{
  if (is_read && !MockDynamixelManager::instance().busWatchdogFedByReads()) {
    return;
  }
  std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  bus_silence_s_ = 0.0;
}

bool MockDynamixel::busWatchdogLatched() const
{
  std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  if (bus_watchdog_addr_ == 0 || static_cast<size_t>(bus_watchdog_addr_) >= memory_.size()) {
    return false;
  }
  return static_cast<int8_t>(memory_[bus_watchdog_addr_]) < 0;
}

bool MockDynamixel::isGoalRegisterAddress(const uint16_t address) const
{
  for (const auto& [start, name] :
       {std::make_pair(goal_position_addr_, "goal_position"), std::make_pair(goal_velocity_addr_, "goal_velocity"),
        std::make_pair(goal_current_addr_, "goal_current"), std::make_pair(goal_pwm_addr_, "goal_pwm")}) {
    if (start == 0) {
      continue;
    }
    const uint8_t length = std::max<uint8_t>(getLength(name), 1);
    if (address >= start && address < start + length) {
      return true;
    }
  }
  return false;
}

void MockDynamixel::startWriteTrace()
{
  std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  write_trace_.clear();
  write_trace_enabled_ = true;
}

void MockDynamixel::stopWriteTrace()
{
  std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  write_trace_enabled_ = false;
}

namespace {
/// Matches the write of `value` to the register's own address.
auto writeOfValue(const uint8_t value)
{
  return [value](const MockDynamixel::TracedWrite& write, const uint16_t address, uint8_t) {
    return write.address == address && write.value == value;
  };
}
/// Matches a write to any byte of the register, whatever value it carried.
bool writeOfAnyByte(const MockDynamixel::TracedWrite& write, const uint16_t address, const uint8_t length)
{
  return write.address >= address && write.address < address + length;
}
}  // namespace

int MockDynamixel::findWriteIndex(const std::string& register_name, const TraceSearch direction,
                                  const std::function<bool(const TracedWrite&, uint16_t, uint8_t)>& matches) const
{
  std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  const uint16_t address = getAddress(register_name);
  if (address == 0) {
    return -1;
  }
  const uint8_t length = std::max<uint8_t>(getLength(register_name), 1);
  const int size = static_cast<int>(write_trace_.size());
  for (int n = 0; n < size; ++n) {
    const int i = direction == TraceSearch::LAST ? size - 1 - n : n;
    if (matches(write_trace_[i], address, length)) {
      return i;
    }
  }
  return -1;
}

int MockDynamixel::lastWriteIndex(const std::string& register_name, const uint8_t value) const
{
  return findWriteIndex(register_name, TraceSearch::LAST, writeOfValue(value));
}

int MockDynamixel::firstWriteIndex(const std::string& register_name, const uint8_t value) const
{
  return findWriteIndex(register_name, TraceSearch::FIRST, writeOfValue(value));
}

int MockDynamixel::lastWriteIndex(const std::string& register_name) const
{
  return findWriteIndex(register_name, TraceSearch::LAST, writeOfAnyByte);
}

int MockDynamixel::firstWriteIndex(const std::string& register_name) const
{
  return findWriteIndex(register_name, TraceSearch::FIRST, writeOfAnyByte);
}

void MockDynamixel::write1Byte(const uint16_t address, const uint8_t data)
{
  std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  store(resolveAddress(address), data);
}

void MockDynamixel::store(const uint16_t address, const uint8_t data)
{
  if (address < memory_.size()) {
    memory_[address] = data;
  }
  if (write_trace_enabled_) {
    write_trace_.push_back(TracedWrite{address, data});
  }
}

void MockDynamixel::write2Byte(const uint16_t address, const uint16_t data)
{
  std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  write1Byte(address, data & 0xFF);
  write1Byte(address + 1, (data >> 8) & 0xFF);
}

void MockDynamixel::write4Byte(const uint16_t address, const uint32_t data)
{
  std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  write1Byte(address, data & 0xFF);
  write1Byte(address + 1, (data >> 8) & 0xFF);
  write1Byte(address + 2, (data >> 16) & 0xFF);
  write1Byte(address + 3, (data >> 24) & 0xFF);
}

void MockDynamixel::setHardwareError(const uint8_t error_code)
{
  std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  hardware_error_ = error_code;
  if (hardware_error_addr_ > 0) {
    write1Byte(hardware_error_addr_, error_code);
  }
  // "Torque Enable is cleared to 0 and the motor's output becomes 0."
  if (shutdownLatched() && torque_enable_addr_ > 0) {
    memory_[torque_enable_addr_] = 0;
  }
}

bool MockDynamixel::shutdownLatched() const
{
  return shutdown_addr_ > 0 && (hardware_error_.load() & memory_[shutdown_addr_]) != 0;
}

void MockDynamixel::powerCycle()
{
  std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  resetRAMRegisters();
  // A signed single turn. The e-Manual does not say which, so the interface must accept either.
  current_position_ -= 2.0 * M_PI * std::floor((current_position_ + M_PI) / (2.0 * M_PI));
  current_velocity_ = 0.0;
  writePresentValues();  // a booted motor never reports a zeroed register
}

void MockDynamixel::resetRAMRegisters()
{
  std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  for (const auto& [addr, len] : ram_registers_) {
    for (uint8_t i = 0; i < len; ++i) {
      if (static_cast<size_t>(addr + i) < memory_.size()) {
        memory_[addr + i] = 0;
      }
    }
  }
  if (indirect_count_ > 0) {
    // Pointers in EEPROM (P and PRO series) survive, the data slots are RAM in every series
    if (indirect_in_ram_) {
      resetIndirectPointers();
    }
    for (size_t i = 0; i < indirect_count_; ++i) {
      const size_t addr = static_cast<size_t>(indirect_data_start_) + i;
      if (addr < memory_.size()) {
        memory_[addr] = 0;
      }
    }
  }
}

void MockDynamixel::resetIndirectPointers()
{
  for (size_t i = 0; i < indirect_count_; ++i) {
    const size_t addr = static_cast<size_t>(indirect_address_start_) + 2 * i;
    const auto target = static_cast<uint16_t>(indirect_data_start_ + i);
    if (addr + 1 < memory_.size()) {
      memory_[addr] = static_cast<uint8_t>(target & 0xFF);
      memory_[addr + 1] = static_cast<uint8_t>(target >> 8);
    }
  }
}

double MockDynamixel::getJointPosition() const
{
  const std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  int32_t homing_offset_ticks = (homing_offset_addr_ > 0) ? read4ByteSigned(homing_offset_addr_) : 0;
  double homing_offset_rad = static_cast<double>(homing_offset_ticks) * rad_per_tick_;
  return current_position_ + homing_offset_rad;
}

bool MockDynamixel::isTimeBasedProfile() const
{
  if (drive_mode_addr_ == 0)
    return false;
  std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  return (memory_[drive_mode_addr_] & 0x04) != 0;
}

double MockDynamixel::getPositionModeVelocityLimit() const
{
  // The profile counts in the unit of the limit (e-Manual)
  const double limit = velocity_limit_addr_ > 0 ? read4ByteSigned(velocity_limit_addr_) * rad_per_s_per_tick_ : 0.0;
  const double profile = profile_velocity_addr_ > 0 && !isTimeBasedProfile() ?
                             read4ByteSigned(profile_velocity_addr_) * rad_per_s_per_tick_ :
                             0.0;
  return profileOrLimit(profile, limit, DEFAULT_VELOCITY_LIMIT);
}

std::pair<double, double> MockDynamixel::getPositionLimits() const
{
  constexpr double NONE = std::numeric_limits<double>::infinity();
  if (min_position_limit_addr_ == 0 || max_position_limit_addr_ == 0) {
    return {-NONE, NONE};
  }
  const double min_position = read4ByteSigned(min_position_limit_addr_) * rad_per_tick_;
  const double max_position = read4ByteSigned(max_position_limit_addr_) * rad_per_tick_;
  // Both 0 is a motor whose limits were never set
  if (min_position == 0.0 && max_position == 0.0) {
    return {-NONE, NONE};
  }
  return {min_position, max_position};
}

double MockDynamixel::getAccelerationLimit() const
{
  // The profile counts in the unit of the limit (e-Manual)
  const auto conversion = unit_conversions_.find("rad_per_s2");
  const double per_tick = conversion == unit_conversions_.end() ? 0.0 : conversion->second;
  const double limit = acceleration_limit_addr_ > 0 ? read4ByteSigned(acceleration_limit_addr_) * per_tick : 0.0;
  const double profile = profile_acceleration_addr_ > 0 && !isTimeBasedProfile() ?
                             read4ByteSigned(profile_acceleration_addr_) * per_tick :
                             0.0;
  return profileOrLimit(profile, limit, DEFAULT_ACCELERATION_LIMIT);
}

double MockDynamixel::perTick(const std::string& register_name) const
{
  const auto unit = unit_map_.find(register_name);
  const auto conversion = unit == unit_map_.end() ? unit_conversions_.end() : unit_conversions_.find(unit->second);
  return conversion == unit_conversions_.end() ? 0.0 : conversion->second;
}

void MockDynamixel::updatePositionMode(const double dt, const bool enforce_limits)
{
  if (goal_position_addr_ == 0 || present_position_addr_ == 0)
    return;

  // The goal register includes the homing offset, as present_position does
  int32_t goal_ticks = read4ByteSigned(goal_position_addr_);
  int32_t homing_offset_ticks = (homing_offset_addr_ > 0) ? read4ByteSigned(homing_offset_addr_) : 0;
  double goal_pos = static_cast<double>(goal_ticks - homing_offset_ticks) * rad_per_tick_;

  constexpr double NONE = std::numeric_limits<double>::infinity();
  const auto [min_pos, max_pos] = enforce_limits ? getPositionLimits() : std::make_pair(-NONE, NONE);
  goal_pos = std::clamp(goal_pos, min_pos, max_pos);

  double vel_limit = getPositionModeVelocityLimit();
  double acc_limit = getAccelerationLimit();

  double pos_error = goal_pos - current_position_;

  // Trapezoidal profile
  double stopping_distance = (current_velocity_ * current_velocity_) / (2.0 * acc_limit);

  double desired_velocity = 0.0;
  if (std::abs(pos_error) > AT_GOAL_POSITION) {
    double direction = (pos_error > 0) ? 1.0 : -1.0;

    if (std::abs(pos_error) <= stopping_distance) {
      desired_velocity = direction * std::sqrt(2.0 * acc_limit * std::abs(pos_error));
    } else {
      desired_velocity = direction * vel_limit;
    }
    // A step at the braking speed would pass the goal and hunt around it
    desired_velocity = std::clamp(desired_velocity, -std::abs(pos_error) / dt, std::abs(pos_error) / dt);
  }

  double vel_error = desired_velocity - current_velocity_;
  double max_vel_change = acc_limit * dt;

  if (std::abs(vel_error) > max_vel_change) {
    current_acceleration_ = (vel_error > 0 ? 1.0 : -1.0) * acc_limit;
    current_velocity_ += current_acceleration_ * dt;
  } else {
    current_velocity_ = desired_velocity;
    current_acceleration_ = vel_error / dt;
  }

  current_velocity_ = std::clamp(current_velocity_, -vel_limit, vel_limit);

  current_position_ += current_velocity_ * dt;

  if (current_position_ < min_pos || current_position_ > max_pos) {
    current_position_ = std::clamp(current_position_, min_pos, max_pos);
    current_velocity_ = 0.0;
  }

  if (std::abs(current_position_ - goal_pos) < AT_GOAL_POSITION && std::abs(current_velocity_) < AT_REST_VELOCITY) {
    current_position_ = goal_pos;
    current_velocity_ = 0.0;
    current_acceleration_ = 0.0;
  }

  current_current_ = std::abs(current_acceleration_) * AMPERE_PER_ACCELERATION;
}

void MockDynamixel::updateVelocityMode(const double dt)
{
  if (goal_velocity_addr_ == 0)
    return;

  int32_t goal_vel_ticks = read4ByteSigned(goal_velocity_addr_);
  double goal_velocity = static_cast<double>(goal_vel_ticks) * rad_per_s_per_tick_;

  double acc_limit = getAccelerationLimit();

  double vel_error = goal_velocity - current_velocity_;
  double max_vel_change = acc_limit * dt;

  if (std::abs(vel_error) > max_vel_change) {
    current_acceleration_ = (vel_error > 0 ? 1.0 : -1.0) * acc_limit;
    current_velocity_ += current_acceleration_ * dt;
  } else {
    current_velocity_ = goal_velocity;
    current_acceleration_ = 0.0;
  }

  current_position_ += current_velocity_ * dt;

  current_current_ = std::abs(current_acceleration_) * AMPERE_PER_ACCELERATION;
}

void MockDynamixel::updateCurrentMode(const double dt)
{
  if (goal_current_addr_ == 0)
    return;

  int16_t goal_current_ticks = static_cast<int16_t>(read2Byte(goal_current_addr_));
  current_current_ = static_cast<double>(goal_current_ticks) * ampere_per_tick_;

  current_acceleration_ = current_current_ * ACCELERATION_PER_AMPERE;

  current_velocity_ += current_acceleration_ * dt;
  current_position_ += current_velocity_ * dt;
}

void MockDynamixel::updateMovingFlag()
{
  if (moving_addr_ > 0) {
    bool moving = std::abs(current_velocity_) > AT_REST_VELOCITY;
    write1Byte(moving_addr_, moving ? 1 : 0);
  }
}

void MockDynamixel::writePresentValues()
{
  writePresentValues(current_position_, current_velocity_, current_current_);
}

void MockDynamixel::writePresentValues(const double position, const double velocity, const double current)
{
  int32_t homing_offset_ticks = 0;
  if (homing_offset_addr_ > 0) {
    homing_offset_ticks = read4ByteSigned(homing_offset_addr_);
  }

  if (present_position_addr_ > 0) {
    int32_t pos_ticks = static_cast<int32_t>(position / rad_per_tick_);
    int32_t present_ticks = pos_ticks + homing_offset_ticks;
    write4Byte(present_position_addr_, static_cast<uint32_t>(present_ticks));
  }

  if (present_velocity_addr_ > 0) {
    int32_t vel_ticks = static_cast<int32_t>(velocity / rad_per_s_per_tick_);
    write4Byte(present_velocity_addr_, static_cast<uint32_t>(vel_ticks));
  }

  if (present_current_addr_ > 0) {
    int16_t current_ticks = static_cast<int16_t>(current / ampere_per_tick_);
    write2Byte(present_current_addr_, static_cast<uint16_t>(current_ticks));
  }
}

uint16_t MockDynamixel::resolveAddress(const uint16_t address) const
{
  if (indirect_count_ > 0 && address >= indirect_data_start_ && address < indirect_data_start_ + indirect_count_) {

    uint16_t index = address - indirect_data_start_;
    uint16_t pointer_addr = indirect_address_start_ + (index * 2);

    if (static_cast<size_t>(pointer_addr + 1) < memory_.size()) {
      uint16_t target = memory_[pointer_addr] | (memory_[pointer_addr + 1] << 8);
      return target;
    }
  }
  return address;
}

std::optional<MockStatus> MockDynamixel::answerPing()
{
  if (!respondsToStatus()) {
    return std::nullopt;
  }
  hearInstruction(true);
  const uint16_t model_number = read2Byte(getAddress("model_number"));
  return MockStatus{statusError(0),
                    {static_cast<uint8_t>(model_number & 0xFF), static_cast<uint8_t>(model_number >> 8),
                     read1Byte(getAddress("version_of_firmware"))}};
}

std::optional<MockStatus> MockDynamixel::answerReboot()
{
  // A motor that only lost its status path reboots, and the master sees nothing happen
  if (receivesWrites()) {
    ++reboot_count_;
    powerCycle();  // a Reboot Instruction resets the position like a power cut does
    clearHardwareError();
  }
  if (!respondsToStatus()) {
    return std::nullopt;
  }
  return MockStatus{statusError(0), {}};
}

std::optional<MockStatus> MockDynamixel::answerRead(const uint16_t address, const uint16_t length)
{
  if (!respondsToStatus()) {
    return std::nullopt;
  }
  hearInstruction(true);
  std::function<void()> hook;
  {
    const std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
    // Every faulted byte spends a use, so a counted fault heals with its register
    bool faulted = false;
    for (uint16_t i = 0; i < length; ++i) {
      const uint16_t target = resolveAddress(static_cast<uint16_t>(address + i));
      faulted = consumeReadFault(target) || faulted;
      const auto it = register_read_hooks_.find(target);
      if (!hook && it != register_read_hooks_.end()) {
        hook = it->second;
      }
    }
    if (faulted) {
      return std::nullopt;
    }
  }
  // Test code that may wait for the reading thread, so without the memory lock
  if (hook) {
    hook();
  }
  const std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  MockStatus status{statusError(0), {}};
  for (uint16_t i = 0; i < length; ++i) {
    status.params.push_back(read1Byte(static_cast<uint16_t>(address + i)));
  }
  return status;
}

std::optional<MockStatus> MockDynamixel::answerWrite(const uint16_t address, const uint8_t* data, const uint16_t length)
{
  if (!receivesWrites()) {
    return std::nullopt;
  }
  hearInstruction(false);
  const std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
  if (const uint8_t refusal = writeRefusal(address, data, length); refusal != 0) {
    return respondsToStatus() ? std::optional<MockStatus>(MockStatus{statusError(refusal), {}}) : std::nullopt;
  }
  std::vector<uint16_t> targets;
  for (uint16_t i = 0; i < length; ++i) {
    targets.push_back(resolveAddress(static_cast<uint16_t>(address + i)));
  }
  if (std::any_of(targets.begin(), targets.end(), [this](const uint16_t target) {
        const auto fault = register_write_faults_.find(target);
        return fault != register_write_faults_.end() && fault->second == RegisterWriteFault::TIMEOUT;
      })) {
    return std::nullopt;
  }
  if (bus_watchdog_addr_ > 0 && std::find(targets.begin(), targets.end(), bus_watchdog_addr_) != targets.end()) {
    bus_watchdog_write_count_.fetch_add(1, std::memory_order_relaxed);
  }
  for (uint16_t i = 0; i < length; ++i) {
    // After a shutdown "REBOOT is the only method to reset Torque Enable to 1". The manual does not
    // say how the status answers, so the write is acknowledged and not applied.
    const bool locked = targets[i] == torque_enable_addr_ && data[i] != 0 && shutdownLatched();
    if (register_write_faults_.count(targets[i]) == 0 && !locked) {
      store(targets[i], data[i]);
    }
  }
  if (!respondsToStatus()) {
    return std::nullopt;
  }
  return MockStatus{statusError(0), {}};
}

// ============================================================================
// MockDynamixelManager
// ============================================================================

namespace {

class MockSdk : public Sdk
{
public:
  std::shared_ptr<PortHandler> portHandler(const std::string& port_name) const override
  {
    return std::make_shared<MockPortHandler>(port_name.c_str());
  }
  std::shared_ptr<PacketHandler> packetHandler() const override
  {
    return std::make_shared<MockPacketHandler>();
  }
  std::shared_ptr<GroupSyncRead> syncRead(const std::shared_ptr<PortHandler>& port,
                                          const std::shared_ptr<PacketHandler>& packet_handler, const uint16_t address,
                                          const uint16_t length) const override
  {
    return std::make_shared<MockGroupSyncRead>(port, packet_handler, address, length);
  }
  std::shared_ptr<GroupSyncWrite> syncWrite(const std::shared_ptr<PortHandler>& port,
                                            const std::shared_ptr<PacketHandler>& packet_handler,
                                            const uint16_t address, const uint16_t length) const override
  {
    return std::make_shared<MockGroupSyncWrite>(port, packet_handler, address, length);
  }
};

}  // namespace

MockDynamixelManager::MockDynamixelManager()
{
  static const MockSdk sdk;
  setDummySdk(&sdk);
}

void MockDynamixelManager::addMotor(const uint8_t id, const uint16_t model_number)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (motors_.find(id) == motors_.end()) {
    motors_.emplace(id, std::make_shared<MockDynamixel>(id, model_number));
  }
}

void MockDynamixelManager::removeMotor(const uint8_t id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  motors_.erase(id);
}

std::shared_ptr<MockDynamixel> MockDynamixelManager::getMotor(const uint8_t id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = motors_.find(id);
  return it == motors_.end() ? nullptr : it->second;
}

std::vector<uint8_t> MockDynamixelManager::getConnectedIds()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<uint8_t> ids;
  for (auto const& [id, motor] : motors_) {
    ids.push_back(id);
  }
  return ids;
}

void MockDynamixelManager::update(const double dt)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& [id, motor] : motors_) {
    motor->update(dt);
  }
}

void MockDynamixelManager::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  motors_.clear();
  sync_write_tx_failures_ = 0;
  packets_until_cut_ = -1;
  bus_watchdog_timed_ = false;
  bus_watchdog_fed_by_reads_ = true;
}

void MockDynamixelManager::setGlobalCommunicationError(const bool enabled)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& [id, motor] : motors_) {
    motor->setCommunicationFault(enabled ? CommunicationFault::DISCONNECTED : CommunicationFault::NONE);
  }
}

void MockDynamixelManager::cutBusAfterPackets(const int count)
{
  std::lock_guard<std::mutex> lock(mutex_);
  packets_until_cut_ = count;
}

void MockDynamixelManager::countPacket()
{
  packets_.fetch_add(1, std::memory_order_relaxed);
  bool cut = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (packets_until_cut_ >= 0) {
      cut = packets_until_cut_-- == 0;
    }
  }
  if (cut) {
    setGlobalCommunicationError(true);
  }
}

void MockDynamixelManager::setSyncWriteTransmissionFailure(const bool enabled)
{
  sync_write_tx_failures_ = enabled ? -1 : 0;
}

void MockDynamixelManager::failNextSyncWriteTransmissions(const int count)
{
  sync_write_tx_failures_ = count;
}

bool MockDynamixelManager::takeSyncWriteTransmissionFailure()
{
  int remaining = sync_write_tx_failures_;
  while (remaining != 0) {
    if (remaining < 0 || sync_write_tx_failures_.compare_exchange_weak(remaining, remaining - 1)) {
      return true;
    }
  }
  return false;
}

void MockDynamixelManager::setBusWatchdogTiming(const bool enabled, const bool fed_by_reads)
{
  bus_watchdog_fed_by_reads_ = fed_by_reads;
  bus_watchdog_timed_ = enabled;
}

bool MockDynamixelManager::busWatchdogTimed() const
{
  return bus_watchdog_timed_;
}

bool MockDynamixelManager::busWatchdogFedByReads() const
{
  return bus_watchdog_fed_by_reads_;
}

// ============================================================================
// MockPacketHandler
// ============================================================================

namespace {

int answer(const std::optional<MockStatus>& status, uint8_t* error)
{
  if (!status) {
    return COMM_RX_TIMEOUT;
  }
  if (error) {
    *error = status->error;
  }
  return COMM_SUCCESS;
}

}  // namespace

int MockPacketHandler::ping(PortHandler& /*port*/, const uint8_t id, uint16_t* model_number, uint8_t* error)
{
  MockDynamixelManager::instance().countPacket();
  const auto motor = MockDynamixelManager::instance().getMotor(id);
  const auto status = motor ? motor->answerPing() : std::nullopt;
  if (status && model_number) {
    *model_number = static_cast<uint16_t>(status->params[0] | (status->params[1] << 8));
  }
  return answer(status, error);
}

int MockPacketHandler::reboot(PortHandler& /*port*/, const uint8_t id, uint8_t* error)
{
  MockDynamixelManager::instance().countPacket();
  const auto motor = MockDynamixelManager::instance().getMotor(id);
  return answer(motor ? motor->answerReboot() : std::nullopt, error);
}

template <typename T>
int MockPacketHandler::readTxRx(const uint8_t id, const uint16_t address, T* data, uint8_t* error)
{
  MockDynamixelManager::instance().countPacket();
  const auto motor = MockDynamixelManager::instance().getMotor(id);
  const auto status = motor ? motor->answerRead(address, sizeof(T)) : std::nullopt;
  if (status && data) {
    *data = 0;
    for (size_t i = 0; i < sizeof(T); ++i) {
      *data |= static_cast<T>(static_cast<T>(status->params[i]) << (8 * i));
    }
  }
  return answer(status, error);
}

template <typename T>
int MockPacketHandler::writeTxRx(const uint8_t id, const uint16_t address, const T data, uint8_t* error)
{
  MockDynamixelManager::instance().countPacket();
  uint8_t bytes[sizeof(T)];
  for (size_t i = 0; i < sizeof(T); ++i) {
    bytes[i] = static_cast<uint8_t>((data >> (8 * i)) & 0xFF);
  }
  const auto motor = MockDynamixelManager::instance().getMotor(id);
  return answer(motor ? motor->answerWrite(address, bytes, sizeof(T)) : std::nullopt, error);
}

int MockPacketHandler::read1ByteTxRx(PortHandler& /*port*/, const uint8_t id, const uint16_t address, uint8_t* data,
                                     uint8_t* error)
{
  return readTxRx(id, address, data, error);
}

int MockPacketHandler::read2ByteTxRx(PortHandler& /*port*/, const uint8_t id, const uint16_t address, uint16_t* data,
                                     uint8_t* error)
{
  return readTxRx(id, address, data, error);
}

int MockPacketHandler::read4ByteTxRx(PortHandler& /*port*/, const uint8_t id, const uint16_t address, uint32_t* data,
                                     uint8_t* error)
{
  return readTxRx(id, address, data, error);
}

int MockPacketHandler::write1ByteTxRx(PortHandler& /*port*/, const uint8_t id, const uint16_t address,
                                      const uint8_t data, uint8_t* error)
{
  return writeTxRx(id, address, data, error);
}

int MockPacketHandler::write2ByteTxRx(PortHandler& /*port*/, const uint8_t id, const uint16_t address,
                                      const uint16_t data, uint8_t* error)
{
  return writeTxRx(id, address, data, error);
}

int MockPacketHandler::write4ByteTxRx(PortHandler& /*port*/, const uint8_t id, const uint16_t address,
                                      const uint32_t data, uint8_t* error)
{
  return writeTxRx(id, address, data, error);
}

const char* MockPacketHandler::getTxRxResult(const int result)
{
  switch (result) {
    case COMM_SUCCESS:
      return "COMM_SUCCESS";
    case COMM_TX_FAIL:
      return "COMM_TX_FAIL";
    case COMM_RX_FAIL:
      return "COMM_RX_FAIL";
    case COMM_TX_ERROR:
      return "COMM_TX_ERROR";
    case COMM_RX_TIMEOUT:
      return "COMM_RX_TIMEOUT";
    default:
      return "COMM_ERROR";
  }
}

const char* MockPacketHandler::getRxPacketError(const uint8_t error)
{
  if (error == 0)
    return "";
  // A motor with a hardware error still answers a refused write with its error, so name both
  const bool alert = (error & PACKET_ERROR_ALERT) != 0;
  switch (error & static_cast<uint8_t>(~PACKET_ERROR_ALERT)) {
    case 0:
      return "Hardware error alert";
    case PACKET_ERROR_DATA_LIMIT:
      return alert ? "Data Limit Error (with hardware error alert)" : "Data Limit Error";
    case PACKET_ERROR_DATA_RANGE:
      return alert ? "Data Range Error (with hardware error alert)" : "Data Range Error";
    case PACKET_ERROR_ACCESS:
      return alert ? "Access Error (with hardware error alert)" : "Access Error";
    default:
      return alert ? "Unknown Error (with hardware error alert)" : "Unknown Error";
  }
}

// ============================================================================
// MockGroupSyncRead
// ============================================================================

int MockGroupSyncRead::txRxPacket()
{
  MockDynamixelManager::instance().countPacket();
  // The SDK group fails as a whole and keeps no data when one motor stays silent
  data_map_.clear();
  bool all_answered = true;
  for (const uint8_t id : ids_) {
    const auto motor = MockDynamixelManager::instance().getMotor(id);
    auto status = motor ? motor->answerRead(start_address_, data_length_) : std::nullopt;
    if (status) {
      data_map_[id] = std::move(status->params);
    } else {
      all_answered = false;
    }
  }
  if (!all_answered) {
    data_map_.clear();
    return COMM_RX_TIMEOUT;
  }
  return COMM_SUCCESS;
}

bool MockGroupSyncRead::isAvailable(const uint8_t id, const uint16_t address, const uint16_t data_length)
{
  return data_map_.find(id) != data_map_.end() && address >= start_address_ &&
         address + data_length <= start_address_ + data_length_;
}

uint32_t MockGroupSyncRead::getData(const uint8_t id, const uint16_t address, const uint16_t data_length)
{
  if (!isAvailable(id, address, data_length)) {
    return 0;
  }
  const auto& data = data_map_.at(id);
  uint32_t value = 0;
  for (uint16_t i = 0; i < data_length && i < 4; ++i) {
    value |= static_cast<uint32_t>(data[address - start_address_ + i]) << (8 * i);
  }
  return value;
}

// ============================================================================
// MockGroupSyncWrite
// ============================================================================

bool MockGroupSyncWrite::addParam(const uint8_t id, uint8_t* data)
{
  // As the SDK group, which is what surfaces two joints on one motor id
  if (params_.find(id) != params_.end()) {
    return false;
  }
  std::vector<uint8_t> d;
  d.assign(data, data + data_length_);
  params_[id] = d;
  return true;
}

bool MockGroupSyncWrite::changeParam(const uint8_t id, uint8_t* data)
{
  if (params_.find(id) == params_.end())
    return false;
  std::vector<uint8_t> d;
  d.assign(data, data + data_length_);
  params_[id] = d;
  return true;
}

int MockGroupSyncWrite::txPacket()
{
  if (MockDynamixelManager::instance().takeSyncWriteTransmissionFailure()) {
    // Nothing reaches the bus, as on a serial write error
    return COMM_TX_FAIL;
  }
  MockDynamixelManager::instance().countPacket();
  for (const auto& [id, data] : params_) {
    if (const auto motor = MockDynamixelManager::instance().getMotor(id)) {
      (void) motor->answerWrite(start_address_, data.data(), data_length_);
    }
  }
  return COMM_SUCCESS;
}

}  // namespace dynamixel_ros_control
