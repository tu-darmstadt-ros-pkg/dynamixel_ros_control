// #include <dynamixel_workbench_ros_control/dynamixel_hardware_interface.h>
// #include <ros/callback_queue.h>
// #include <controller_manager/controller_manager.h>

int main(int argc, char** argv)
{
//   ros::init(argc, argv, "dynamixel_controller_manager");
//   ros::NodeHandle pnh("~");
// 
//   // Load dynamixels
//   dynamixel_workbench_ros_control::DynamixelHardwareInterface hw;
//   if (!hw.init(pnh))
//   {
//     ROS_ERROR_STREAM("Failed to initialize hardware interface.");
//     return 0;
//   }
// 
//   // Create separate queue, because otherwise CM will freeze
//   ros::NodeHandle nh;
//   ros::CallbackQueue queue;
//   nh.setCallbackQueue(&queue);
//   ros::AsyncSpinner spinner(1, &queue);
//   spinner.start();
//   controller_manager::ControllerManager cm(&hw, nh);
// 
//   // Start control loop
//   ros::Time current_time = ros::Time::now();
//   bool first_update = true;
//   ros::Rate rate(pnh.param("control_loop_hz", 25));
//   while (ros::ok())
//   {
//     hw.read();
//     ros::Duration period = ros::Time::now() - current_time;
//     current_time = ros::Time::now();
//     if (first_update) {
//       first_update = false;
//     } else {
//       cm.update(current_time, period);
//     }
//     hw.write();
//     rate.sleep();
//     ros::spinOnce();
//   }
  return 0;
}
