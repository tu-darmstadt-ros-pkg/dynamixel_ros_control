#ifndef DYNAMIXEL_ROS_CONTROL_LOG_H
#define DYNAMIXEL_ROS_CONTROL_LOG_H

#include <rclcpp/logging.hpp>

#define DXL_LOG(level, msg) RCLCPP_##level##_STREAM(rclcpp::get_logger("dynamixel_ros_control"), msg)

#define DXL_LOG_DEBUG(msg) DXL_LOG(DEBUG, msg)
#define DXL_LOG_INFO(msg) DXL_LOG(INFO, msg)
#define DXL_LOG_WARN(msg) DXL_LOG(WARN, msg)
#define DXL_LOG_ERROR(msg) DXL_LOG(ERROR, msg)
#define DXL_LOG_FATAL(msg) DXL_LOG(FATAL, msg)

#endif //DYNAMIXEL_ROS_CONTROL_LOG_H
