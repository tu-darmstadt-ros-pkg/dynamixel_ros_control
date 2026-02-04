#ifndef DYNAMIXEL_ROS_CONTROL_COMMON_H
#define DYNAMIXEL_ROS_CONTROL_COMMON_H
#include <string>
#include <unordered_map>
#include <algorithm>
#include <sstream>
#include <vector>
#include <cstdint>

namespace dynamixel_ros_control {

// Timing constants for motor communication
constexpr int64_t REBOOT_WAIT_NS = 500'000'000;  // 500ms - Wait for motor to restart after reboot

// Default model number for dummy/mock motors (Dynamixel PH series)
constexpr uint16_t DEFAULT_MOCK_MODEL_NUMBER = 2020;

// LED color values
constexpr int COLOR_PINK_VALUES[] = {255, 175, 193};
constexpr int COLOR_GREEN_VALUES[] = {0, 255, 0};
constexpr int COLOR_BLUE_VALUES[] = {0, 0, 255};
constexpr int COLOR_ORANGE_VALUES[] = {255, 165, 0};
constexpr int COLOR_RED_VALUES[] = {255, 0, 0};

// LED color names
constexpr char COLOR_PINK[] = "pink";
constexpr char COLOR_GREEN[] = "green";
constexpr char COLOR_BLUE[] = "blue";
constexpr char COLOR_ORANGE[] = "orange";
constexpr char COLOR_RED[] = "red";

using ParameterMap = std::unordered_map<std::string, std::string>;

template <typename T>
bool getParameter(const ParameterMap& map, const std::string& param_name, T& value);

template <typename T>
bool getParameter(const ParameterMap& map, const std::string& param_name, T& value, const T& default_value)
{
  const bool success = getParameter(map, param_name, value);
  if (!success) {
    value = default_value;
  }
  return success;
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

inline std::string& removeWhitespace(std::string& s)
{
  const auto end_pos = std::remove(s.begin(), s.end(), ' ');
  s.erase(end_pos, s.end());
  return s;
}

bool splitFullInterfaceName(const std::string& full_name, std::string& joint_name, std::string& interface_name);

// template<typename T>
// void split(const std::string &s, char delim, T result) {
//     std::stringstream ss(s);
//     std::string item;
//     while (std::getline(ss, item, delim)) {
//         *(result++) = item;
//     }
// }

// std::vector<std::string> split(const std::string &s, char delim) {
//     std::vector<std::string> elems;
//     split(s, delim, std::back_inserter(elems));
//     return elems;
// }

}  // namespace dynamixel_ros_control

#endif
