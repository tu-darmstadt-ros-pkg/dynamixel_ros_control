#ifndef DYNAMIXEL_ROS_CONTROL_DIAGNOSTIC_STATE_H
#define DYNAMIXEL_ROS_CONTROL_DIAGNOSTIC_STATE_H

#include <cstdint>
#include <string>

namespace dynamixel_ros_control {

/// @brief Decode a hardware-error-status bitfield (Dynamixel error byte) into a comma-
/// separated human-readable string. Returns "ok" when no bits are set.
std::string hardwareErrorToString(int32_t error_status);

}  // namespace dynamixel_ros_control

#endif
