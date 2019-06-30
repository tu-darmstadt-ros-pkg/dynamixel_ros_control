#ifndef DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_HARDWARE_INTERFACE_H
#define DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

#include <dynamixel_ros_control/dynamixel_driver.h>
#include <dynamixel_ros_control/sync_read_manager.h>
#include <dynamixel_ros_control/sync_write_manager.h>

#include <dynamixel_ros_control/joint.h>

namespace dynamixel_ros_control {

class DynamixelHardwareInterface : public hardware_interface::RobotHW
{
public:
  DynamixelHardwareInterface(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~DynamixelHardwareInterface() override;

  bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh) override;
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;

  ros::Time getLastReadTime() const;

  bool resetRequired() const;
  void clearResetRequired();

private:
  bool loadDynamixels(const ros::NodeHandle& nh);
  void writeInitialValues(const ros::NodeHandle& nh);
  void writeControlMode();
  void setTorque(bool enabled);
  void setTorque(const std_msgs::BoolConstPtr& enabled);
  bool rebootCb(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response);
  Joint* getJointByName(std::string name);

  void estopCb(const std_msgs::BoolConstPtr& bool_ptr);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  bool initialized_;
  bool first_cycle_;
  bool estop_;
  bool reset_required_;

  dynamixel_ros_control::DynamixelDriver driver_;

  // Read
  hardware_interface::JointStateInterface jnt_state_interface_;
  SyncReadManager read_manager_;
  SyncReadManager status_read_manager_;

  // Write
  hardware_interface::PositionJointInterface jnt_pos_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;
  hardware_interface::EffortJointInterface jnt_eff_interface_;
  SyncWriteManager control_write_manager_;
  SyncWriteManager torque_write_manager_;

  // Configuration
  bool debug_;
  bool torque_on_startup_;
  bool torque_off_on_shutdown_;
  bool reset_controllers_after_estop_;

  bool read_position_;
  bool read_velocity_;
  bool read_effort_;

  std::vector<Joint>::size_type time_sync_joint_idx_;

  std::vector<Joint> joints_;

  // Subscribers
  ros::Subscriber estop_sub_;
  ros::Subscriber set_torque_sub_;
  ros::ServiceServer reboot_server_;
};

}

#endif
