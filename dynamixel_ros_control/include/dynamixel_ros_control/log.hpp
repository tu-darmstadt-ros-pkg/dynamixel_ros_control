#ifndef DYNAMIXEL_ROS_CONTROL_LOG_H
#define DYNAMIXEL_ROS_CONTROL_LOG_H

#include <rclcpp/logging.hpp>

constexpr char DXL_LOGGER_NAME[] = "dynamixel_ros_control";
#define DXL_LOG(level, msg) RCLCPP_##level##_STREAM(rclcpp::get_logger(DXL_LOGGER_NAME), msg)

#define DXL_LOG_DEBUG(msg) DXL_LOG(DEBUG, msg)
#define DXL_LOG_INFO(msg) DXL_LOG(INFO, msg)
#define DXL_LOG_WARN(msg) DXL_LOG(WARN, msg)
#define DXL_LOG_ERROR(msg) DXL_LOG(ERROR, msg)
#define DXL_LOG_FATAL(msg) DXL_LOG(FATAL, msg)

#endif //DYNAMIXEL_ROS_CONTROL_LOG_H
