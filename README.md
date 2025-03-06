# dynamixel_ros_control
_dynamixel_ros_control_ is a [ROS2](https://www.ros.org/) driver for [Robotis Dynamixel](http://www.robotis.us/dynamixel/) actuators. It is based on the  [ros2_control](https://control.ros.org/rolling/index.html) framework and implements a hardware interface.

**Main features:**
* Support for all protocol 2.0 Dynamixel models
* Support for mixed chains with different models in the same chain
* Synchronized, efficient reading and writing of registers
* Provides state interfaces for any readable register (position, velocity, current, ...)
* Provides command interfaces for any writable register (position, velocity, current, ...)
* Automatic switching of control mode during runtime
* Automatic reconnection in case of errors
