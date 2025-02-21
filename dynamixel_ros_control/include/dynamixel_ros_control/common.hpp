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

template <typename T>
std::string vectorToString(const std::vector<T>& vec)
{
  std::stringstream ss;
  ss << "[";
  for (const T& element : vec) {
    ss << element << ", ";
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
