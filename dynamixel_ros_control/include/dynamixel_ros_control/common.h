#ifndef DYNAMIXEL_ROS_CONTROL__COMMON_H
#define DYNAMIXEL_ROS_CONTROL__COMMON_H

#include <ros/ros.h>

namespace dynamixel_ros_control {

template <typename T>
bool loadRequiredParameter(const ros::NodeHandle& nh, std::string name, T& value) {
  if (!nh.getParam(name, value)) {
    ROS_ERROR_STREAM("Failed to load required parameter '" << nh.getNamespace() << "/" << name << "'.");
    return false;
  }
  return true;
}

}

#endif
