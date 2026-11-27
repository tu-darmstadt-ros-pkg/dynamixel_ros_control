#ifndef DYNAMIXEL_ROS_CONTROL_LOG_H
#define DYNAMIXEL_ROS_CONTROL_LOG_H

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rcutils/logging.h>

namespace dynamixel_ros_control {

constexpr char DXL_LOGGER_NAME[] = "dynamixel_ros_control";

/// The logger of code that belongs to no bus. A class on a bus declares its own dxlLogger(), which
/// DXL_LOG finds first, so each hardware component logs and sets its level separately.
inline const rclcpp::Logger& dxlLogger()
{
  static const rclcpp::Logger logger = rclcpp::get_logger(DXL_LOGGER_NAME);
  return logger;
}

}  // namespace dynamixel_ros_control

// RCLCPP_LOG_STREAM formats the message before it checks the level
#define DXL_LOG(level, msg)                                                                                            \
  do {                                                                                                                 \
    const rclcpp::Logger& dxl_log_logger = dxlLogger();                                                                \
    if (rcutils_logging_logger_is_enabled_for(dxl_log_logger.get_name(), RCUTILS_LOG_SEVERITY_##level)) {              \
      RCLCPP_##level##_STREAM(dxl_log_logger, msg);                                                                    \
    }                                                                                                                  \
  } while (0)

#define DXL_LOG_DEBUG(msg) DXL_LOG(DEBUG, msg)
#define DXL_LOG_INFO(msg) DXL_LOG(INFO, msg)
#define DXL_LOG_WARN(msg) DXL_LOG(WARN, msg)
#define DXL_LOG_ERROR(msg) DXL_LOG(ERROR, msg)
#define DXL_LOG_FATAL(msg) DXL_LOG(FATAL, msg)

#endif  // DYNAMIXEL_ROS_CONTROL_LOG_H
