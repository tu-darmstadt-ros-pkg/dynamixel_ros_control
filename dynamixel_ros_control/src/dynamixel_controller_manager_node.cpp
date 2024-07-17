#include <dynamixel_ros_control/dynamixel_hardware_interface.h>
#include <ros/callback_queue.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_controller_manager");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Create separate queue, because otherwise CM will freeze
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);
  ros::AsyncSpinner spinner(1, &queue);
  spinner.start();

  // Initialize hardware interface
  dynamixel_ros_control::DynamixelHardwareInterface hw;
  if (!hw.init(nh, pnh))
  {
    ROS_ERROR_STREAM("Failed to initialize hardware interface.");
    return 0;
  }
  // Initialize controller manager
  controller_manager::ControllerManager cm(&hw, nh);

  // Start control loop
  ros::Time previous_read_time = ros::Time::now();
  bool first_update = true;
  double control_rate = pnh.param("control_rate", 25);
  ros::Rate rate(control_rate);
  ros::Duration expected_period(1.0/control_rate);
  while (ros::ok()) {
    // Compute period and detect jumps in time
    ros::Time now = ros::Time::now();
    ros::Duration period;
    if (now.toSec() > previous_read_time.toSec() + 1.5*expected_period.toSec()) {
      ros::Duration diff = now - previous_read_time;
      ROS_ERROR_STREAM("Jump in time detected (+" << diff.toSec() << " s).");
    }
    if (now < previous_read_time) {
      ros::Duration diff = previous_read_time - now;
      ROS_ERROR_STREAM("Time moved backwards (-" << diff.toSec() << " s).");
      rate.reset();
      period = expected_period; // Fallback: Set to expected period
    } else {
      period = ros::Time::now() - previous_read_time;
    }

    hw.read(now, period);
    ros::Time read_time = hw.getLastReadTime();
    ros::Duration read_period = read_time - previous_read_time;
    previous_read_time = read_time;

    if (first_update) {
      first_update = false;
    } else {
      cm.update(read_time, read_period, hw.resetRequired());
      if (hw.resetRequired()) {
        hw.clearResetRequired();
      }
    }
    hw.write(read_time, read_period);
    if (!rate.sleep()) {
      ROS_WARN_STREAM("Desired rate was not met.");
    }
    ros::spinOnce();
  }
  return 0;
}
