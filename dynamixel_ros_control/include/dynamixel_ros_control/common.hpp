#ifndef DYNAMIXEL_ROS_CONTROL_COMMON_H
#define DYNAMIXEL_ROS_CONTROL_COMMON_H
#include <string>
#include <unordered_map>
#include <algorithm>
#include <sstream>
#include <vector>

namespace dynamixel_ros_control {

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
    ss << *it; // First element (avoid trailing comma)
    ++it;
  }

  while (it != container.end()) {
    ss << ", " << *it;
    ++it;
  }

  ss << "]";
  return ss.str();
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
