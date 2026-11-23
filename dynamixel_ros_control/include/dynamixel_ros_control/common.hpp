#ifndef DYNAMIXEL_ROS_CONTROL_COMMON_H
#define DYNAMIXEL_ROS_CONTROL_COMMON_H
#include <string>
#include <set>
#include <unordered_map>
#include <algorithm>
#include <sstream>
#include <vector>
#include <cstdint>
#include <chrono>
#include <limits>
#include <stdexcept>

namespace dynamixel_ros_control {

// Timing constants for motor communication
constexpr std::chrono::milliseconds REBOOT_WAIT{500};  // Wait for motor to restart after reboot.

/// Consecutive failed bus transactions after which a sync manager reports the connection lost.
/// Shared so the diagnostics topic reports the threshold the managers actually apply.
constexpr unsigned int COMM_ERROR_THRESHOLD = 25;

/// The command interface whose claim switches a joint's torque off. A controller that holds it
/// writes no value: the claim itself is the command.
constexpr char TORQUE_OFF_INTERFACE[] = "torque_off";

/// Runtime caps beside a main interface, never deciding the mode. Unclaimed or NaN they hold the
/// EEPROM limit, and a value above it is clamped to it.
constexpr char MAX_CURRENT_INTERFACE[] = "max_current";
constexpr char MAX_VELOCITY_INTERFACE[] = "max_velocity";

/// @brief Parse a decimal integer from a control-table field and check it fits @p T.
///
/// std::stoi throws out_of_range as well as invalid_argument, and a bare static_cast narrows
/// silently. A data_length of 256 would become 0, and the register would be read as zero bytes.
/// Both are configuration errors in a shipped YAML, so both must be refused rather than absorbed.
/// @return false, leaving @p out untouched, if @p text is not an integer, carries trailing
/// characters, or does not fit.
template <typename T>
[[nodiscard]] bool parseIntegerInto(const std::string& text, T& out)
{
  try {
    size_t parsed = 0;
    const long long value = std::stoll(text, &parsed);
    // stoll stops at the first character it cannot use and reports success for what it did read.
    // A remainder means the field was only partly understood, so "4junk" must not become 4.
    if (text.find_first_not_of(" \t\n\r\f\v", parsed) != std::string::npos) {
      return false;
    }
    if (value < static_cast<long long>(std::numeric_limits<T>::min()) ||
        value > static_cast<long long>(std::numeric_limits<T>::max())) {
      return false;
    }
    out = static_cast<T>(value);
    return true;
  }
  catch (const std::exception&) {
    return false;
  }
}

using ParameterMap = std::unordered_map<std::string, std::string>;

template <typename T>
[[nodiscard]] bool getParameter(const ParameterMap& map, const std::string& param_name, T& value);

/// An absent parameter takes @p default_value. A present one that does not parse fails.
template <typename T>
[[nodiscard]] bool getOptionalParameter(const ParameterMap& map, const std::string& param_name, T& value,
                                        const T& default_value)
{
  if (map.count(param_name) == 0) {
    value = default_value;
    return true;
  }
  return getParameter(map, param_name, value);
}

template <typename Iterable>
std::string iterableToString(const Iterable& container)
{
  std::stringstream ss;
  ss << "[";

  auto it = container.begin();
  if (it != container.end()) {
    ss << *it;  // First element (avoid trailing comma)
    ++it;
  }

  while (it != container.end()) {
    ss << ", " << *it;
    ++it;
  }

  ss << "]";
  return ss.str();
}

template <typename Map>
std::string mapToString(const Map& m, const std::string& separator = ", ", const std::string& keyValueSeparator = ": ")
{
  std::ostringstream oss;
  bool first = true;

  for (const auto& [key, value] : m) {
    if (!first) {
      oss << separator;
    }
    first = false;
    oss << key << keyValueSeparator << value;
  }

  return oss.str();
}

/**
 * @brief Simple monotonic stopwatch for startup/phase timing diagnostics.
 */
class Stopwatch
{
public:
  Stopwatch()
      : start_(std::chrono::steady_clock::now())
  {}

  void reset()
  {
    start_ = std::chrono::steady_clock::now();
  }

  [[nodiscard]] double elapsedMs() const
  {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_).count();
  }

private:
  std::chrono::steady_clock::time_point start_;
};

inline std::string& removeWhitespace(std::string& s)
{
  const auto end_pos = std::remove(s.begin(), s.end(), ' ');
  s.erase(end_pos, s.end());
  return s;
}

bool splitFullInterfaceName(const std::string& full_name, std::string& joint_name, std::string& interface_name);

/// The names in @p parameters that are neither in @p known nor start with @p known_prefix (if not empty),
/// in alphabetical order.
std::vector<std::string> unknownParameters(const ParameterMap& parameters, const std::set<std::string>& known,
                                           const std::string& known_prefix = "");

/**
 * @brief Load the interface-name -> register-name translation tables from a YAML file.
 *
 * The file must be a map with the sequences "state_interfaces", "command_interfaces" and "limits",
 * each entry a map with "interface_name" and "register_name". Any deviation — unreadable file,
 * missing section, entry that is not a map, missing key — fails with a logged error rather than
 * returning partially filled maps: an empty translation table would let configuration continue and
 * fail much later with a far less obvious message.
 */
bool loadInterfaceRegisterNameTranslation(const std::string& path,
                                          std::unordered_map<std::string, std::string>& state_interface_to_register,
                                          std::unordered_map<std::string, std::string>& command_interface_to_register,
                                          std::unordered_map<std::string, std::string>& interface_to_register_limits);

}  // namespace dynamixel_ros_control

#endif
