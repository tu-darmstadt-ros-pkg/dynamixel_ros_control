#ifndef DYNAMIXEL_ROS_CONTROL_COMMON_H
#define DYNAMIXEL_ROS_CONTROL_COMMON_H

#include <ros/ros.h>

const std::string PACKAGE_NAME = "dynamixel_ros_control";

namespace dynamixel_ros_control {

template <typename T>
bool loadRequiredParameter(const ros::NodeHandle& nh, std::string name, T& value) {
  if (!nh.getParam(name, value)) {
    ROS_ERROR_STREAM("Failed to load required parameter '" << nh.getNamespace() << "/" << name << "'.");
    return false;
  }
  return true;
}

inline std::string &removeWhitespace(std::string &s) {
  std::string::iterator end_pos = std::remove(s.begin(), s.end(), ' ');
  s.erase(end_pos, s.end());
  return s;
}

//template<typename T>
//void split(const std::string &s, char delim, T result) {
//    std::stringstream ss(s);
//    std::string item;
//    while (std::getline(ss, item, delim)) {
//        *(result++) = item;
//    }
//}

//std::vector<std::string> split(const std::string &s, char delim) {
//    std::vector<std::string> elems;
//    split(s, delim, std::back_inserter(elems));
//    return elems;
//}

}

#endif
