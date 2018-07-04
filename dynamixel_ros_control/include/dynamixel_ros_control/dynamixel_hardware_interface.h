#ifndef DYNAMIXEL_ROS_CONTROL__DYNAMIXEL_HARDWARE_INTERFACE_H
#define DYNAMIXEL_ROS_CONTROL__DYNAMIXEL_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <std_msgs/Bool.h>

#include <dynamixel_ros_control/dynamixel_driver.h>

namespace dynamixel_ros_control {

class DynamixelHardwareInterface : public hardware_interface::RobotHW
{
public:
  DynamixelHardwareInterface(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~DynamixelHardwareInterface();

  bool init();
  void read();
  void write();

private:
  bool loadConfig();

  void setTorque(bool enabled);
  void setTorque(std_msgs::BoolConstPtr enabled);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  bool debug_;

  bool first_cycle_;

  boost::shared_ptr<dynamixel_ros_control::DynamixelDriver> driver_;

  // Read
  hardware_interface::JointStateInterface jnt_state_interface_;

  // Write
  hardware_interface::PositionJointInterface jnt_pos_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;
  hardware_interface::EffortJointInterface jnt_eff_interface_;

  bool torque_off_on_shutdown_;

  int joint_count_;

  std::vector<std::string> joint_names_;


  // subscriber
  ros::Subscriber set_torque_sub_;
};

}

#endif
