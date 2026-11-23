#include "dynamixel_ros_control/log.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <dynamixel_ros_control/common.hpp>
#include <yaml-cpp/yaml.h>

namespace dynamixel_ros_control {

template <>
bool getParameter<std::string>(const ParameterMap& map, const std::string& param_name, std::string& value)
{
  const auto it = map.find(param_name);
  if (it == map.end()) {
    return false;
  }
  value = it->second;
  return true;
}

namespace {

/// @brief Whether everything from `pos` on is whitespace.
///
/// std::stod / std::stoi stop at the first character they cannot use and report success, so
/// "5x", "3 cycles" and "0.1;" would all silently become numbers. A parameter that does not
/// wholly parse is operator error, and guessing at its intent is worse than refusing it.
bool onlyTrailingWhitespace(const std::string& text, const size_t pos)
{
  return std::all_of(text.begin() + static_cast<std::ptrdiff_t>(pos), text.end(),
                     [](const unsigned char c) { return std::isspace(c) != 0; });
}

/// @brief Parse a whole parameter string with @p parse, or explain why it is not a number.
///
/// The two numeric specialisations differ only in which conversion they call and what they call
/// the value. The rules that make a parameter trustworthy (it must parse, and nothing but
/// whitespace may follow it) live here, so they are not kept in step in each of them.
template <typename T, typename Parse>
bool parseWholeParameter(const ParameterMap& map, const std::string& param_name, const char* type_noun,
                         const Parse& parse, T& value)
{
  std::string parameter_string;
  if (!getParameter<std::string>(map, param_name, parameter_string)) {
    return false;
  }

  T parsed;
  size_t consumed = 0;
  try {
    parsed = parse(parameter_string, &consumed);
  }
  catch (const std::exception& e) {
    DXL_LOG_ERROR("Failed to convert parameter '" << param_name << "' to " << type_noun << ": " << e.what());
    return false;
  }
  if (!onlyTrailingWhitespace(parameter_string, consumed)) {
    DXL_LOG_ERROR("Parameter '" << param_name << "' = '" << parameter_string << "' is not " << type_noun
                                << "; it has trailing characters.");
    return false;
  }
  value = parsed;
  return true;
}

}  // namespace

template <>
bool getParameter<double>(const ParameterMap& map, const std::string& param_name, double& value)
{
  double parsed;
  if (!parseWholeParameter(
          map, param_name, "a number", [](const std::string& text, size_t* pos) { return std::stod(text, pos); },
          parsed)) {
    return false;
  }
  // std::stod happily returns nan and inf, and every ordinary range check downstream passes for
  // nan (all comparisons are false). A "nan" timeout would sail through its own validation.
  if (!std::isfinite(parsed)) {
    DXL_LOG_ERROR("Parameter '" << param_name << "' is not a finite number.");
    return false;
  }
  value = parsed;
  return true;
}

template <>
bool getParameter<int>(const ParameterMap& map, const std::string& param_name, int& value)
{
  return parseWholeParameter(
      map, param_name, "an integer", [](const std::string& text, size_t* pos) { return std::stoi(text, pos); }, value);
}

template <>
bool getParameter<bool>(const ParameterMap& map, const std::string& param_name, bool& value)
{
  std::string parameter_string;
  if (!getParameter<std::string>(map, param_name, parameter_string)) {
    return false;
  }
  // Surrounding whitespace is ignored, as the numeric parsers do
  const auto is_space = [](const unsigned char c) { return std::isspace(c) != 0; };
  parameter_string.erase(std::find_if_not(parameter_string.rbegin(), parameter_string.rend(), is_space).base(),
                         parameter_string.end());
  parameter_string.erase(parameter_string.begin(),
                         std::find_if_not(parameter_string.begin(), parameter_string.end(), is_space));
  std::transform(parameter_string.begin(), parameter_string.end(), parameter_string.begin(),
                 [](const unsigned char c) { return std::tolower(c); });

  if (parameter_string == "true" || parameter_string == "1") {
    value = true;
    return true;
  }
  if (parameter_string == "false" || parameter_string == "0") {
    value = false;
    return true;
  }
  DXL_LOG_ERROR("Failed to convert parameter '" << param_name << "' to a bool.");
  return false;
}

template <>
bool getParameter<uint8_t>(const ParameterMap& map, const std::string& param_name, uint8_t& value)
{
  int param_int;
  if (!getParameter<int>(map, param_name, param_int)) {
    return false;
  }

  if (param_int < std::numeric_limits<uint8_t>::min() || param_int > std::numeric_limits<uint8_t>::max()) {
    DXL_LOG_ERROR("Failed to convert parameter '" << param_name << "' to an uint8: Exceeded bounds.");
    return false;
  }
  value = param_int;
  return true;
}

bool splitFullInterfaceName(const std::string& full_name, std::string& joint_name, std::string& interface_name)
{
  const auto it = std::find(full_name.begin(), full_name.end(), '/');
  if (it == full_name.end()) {
    return false;
  }
  joint_name = full_name.substr(0, it - full_name.begin());
  interface_name = full_name.substr(it - full_name.begin() + 1, full_name.end() - it);
  return true;
}

namespace {

bool loadInterfaceRegisterTranslationMap(const YAML::Node& node, const char* section_name,
                                         std::unordered_map<std::string, std::string>& conversion_map)
{
  if (!node.IsDefined() || node.IsNull()) {
    DXL_LOG_ERROR("Section '" << section_name << "' is missing from the interface translation file.");
    return false;
  }
  if (!node.IsSequence()) {
    DXL_LOG_ERROR("Section '" << section_name << "' is not a sequence.");
    return false;
  }
  conversion_map.clear();
  for (const auto& entry : node) {
    if (!entry.IsMap()) {
      DXL_LOG_ERROR("Entry in section '" << section_name << "' is not a map.");
      return false;
    }
    // .as<>() throws on a missing or non-scalar key. The caller's catch turns that into a clean
    // failure instead of an exception escaping on_init into the controller manager.
    const auto interface_name = entry["interface_name"].as<std::string>();
    const auto register_name = entry["register_name"].as<std::string>();
    conversion_map.emplace(interface_name, register_name);
  }
  return true;
}

}  // namespace

bool loadInterfaceRegisterNameTranslation(const std::string& path,
                                          std::unordered_map<std::string, std::string>& state_interface_to_register,
                                          std::unordered_map<std::string, std::string>& command_interface_to_register,
                                          std::unordered_map<std::string, std::string>& interface_to_register_limits)
{
  YAML::Node config;
  try {
    config = YAML::LoadFile(path);
    if (!config.IsMap()) {
      DXL_LOG_ERROR("Interface translation file '" << path << "' is not a map (wrong format).");
      return false;
    }
    return loadInterfaceRegisterTranslationMap(config["state_interfaces"], "state_interfaces",
                                               state_interface_to_register) &&
           loadInterfaceRegisterTranslationMap(config["command_interfaces"], "command_interfaces",
                                               command_interface_to_register) &&
           loadInterfaceRegisterTranslationMap(config["limits"], "limits", interface_to_register_limits);
  }
  catch (const YAML::BadFile&) {
    DXL_LOG_ERROR("Failed to read interface to register name translation at '" << path << "'. Does the file exist?");
    return false;
  }
  catch (const YAML::Exception& e) {
    DXL_LOG_ERROR("Malformed interface translation file '" << path << "': " << e.what());
    return false;
  }
}

std::vector<std::string> unknownParameters(const ParameterMap& parameters, const std::set<std::string>& known,
                                           const std::string& known_prefix)
{
  std::vector<std::string> unknown;
  for (const auto& [name, value] : parameters) {
    if (known.count(name) == 0 && (known_prefix.empty() || name.rfind(known_prefix, 0) != 0)) {
      unknown.push_back(name);
    }
  }
  std::sort(unknown.begin(), unknown.end());
  return unknown;
}

}  // namespace dynamixel_ros_control
