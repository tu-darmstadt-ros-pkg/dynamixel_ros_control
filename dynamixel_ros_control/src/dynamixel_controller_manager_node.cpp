#include <dynamixel_ros_control/dynamixel_hardware_interface.h>
#include <ros/callback_queue.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_controller_manager");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Initialize hardware interface
  dynamixel_ros_control::DynamixelHardwareInterface hw(nh, pnh);
  if (!hw.init(nh, pnh))
  {
    ROS_ERROR_STREAM("Failed to initialize hardware interface.");
    return 0;
  }

  // Create separate queue, because otherwise CM will freeze
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);
  ros::AsyncSpinner spinner(1, &queue);
  spinner.start();
  controller_manager::ControllerManager cm(&hw, nh);

  // Start control loop
  ros::Time previous_time = ros::Time::now();
  bool first_update = true;
  double control_rate = pnh.param("control_rate", 25);
  ros::Rate rate(control_rate);
  while (ros::ok())
  {
    ros::Time now = ros::Time::now();
    ros::Duration period;
    if (now < previous_time) {
      ROS_ERROR_STREAM("Jump in time detected.");
      period = ros::Duration(1.0/control_rate); // Fallback: Set to expected period
    } else {
      period = ros::Time::now() - previous_time;
    }
    previous_time = now;
    hw.read(now, period);
    if (first_update) {
      first_update = false;
    } else {
      cm.update(now, period);
    }
    hw.write(now, period);
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
