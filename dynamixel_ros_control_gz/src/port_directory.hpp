#ifndef DYNAMIXEL_ROS_CONTROL_GZ_PORT_DIRECTORY_HPP
#define DYNAMIXEL_ROS_CONTROL_GZ_PORT_DIRECTORY_HPP

#include <filesystem>

namespace dynamixel_ros_control_gz {

/// @brief Creates the directory the port links live in, 0700. The fallback lives under /tmp, so it
/// refuses a directory another user could swap a link in. Every directory on the way must belong to
/// this user or root, and the last one must be writable by this user only.
/// @throws std::runtime_error naming the directory that is refused
void securePortDirectory(const std::filesystem::path& directory);

}  // namespace dynamixel_ros_control_gz

#endif  // DYNAMIXEL_ROS_CONTROL_GZ_PORT_DIRECTORY_HPP
