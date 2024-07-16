#ifndef DYNAMIXEL_ROS_CONTROL_JOINT_H
#define DYNAMIXEL_ROS_CONTROL_JOINT_H

#include <dynamixel_ros_control/dynamixel.h>

#include <dynamixel_ros_control/dynamixel_driver.h>

namespace dynamixel_ros_control {

struct State
{
  State() : position(0), velocity(0), effort(0), torque(false) {}
  double position;
  double velocity;
  double effort;
  bool torque;
};

class Joint {
public:
  Joint(std::string joint_name, dynamixel_ros_control::DynamixelDriver& driver);
  Joint(std::string _name, uint8_t id, dynamixel_ros_control::DynamixelDriver& driver);

  bool initFromNh(const ros::NodeHandle& nh);
  bool initDxl();

  std::string name;
  Dynamixel dynamixel;

  State current_state;
  State goal_state;

  double mounting_offset;
  double offset;

  double estop_position;

  ControlMode getControlMode() const;
  bool setControlMode(const ControlMode& value);

  bool isPositionControlled() const;
  bool isVelocityControlled() const;
  bool isEffortControlled() const;

private:
  ros::NodeHandle nh_;
  ControlMode control_mode;

};

}

#endif
