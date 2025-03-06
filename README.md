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

## Installation
Install dynamixel_ros_control from source by cloning this repository into your ros2 workspace. The dependencies can be installed using [rosdep](http://wiki.ros.org/rosdep). Go into the dynamixel_ros_control folder and execute
```
rosdep install --from-paths . --ignore-src -r -y
```
Afterwards, build your workspace.

## Getting started

A demo configuration can be started by launching:
```
ros2 launch dynamixel_ros_control controller_manager.launch.yaml port_name:=/dev/ttyUSB0 id:=1 baud_rate:=57600
```
This will open the device `/dev/ttyUSB0` with baud rate `57600` for the dynamixel motor id `1` with a position controller.

Send a position command with:
```
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "{data: ["1"]}"
```
Show the joint state with:
```
ros2 topic echo /joint_states
```
You can configure other controllers by using [rqt_controller_manager](https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/userdoc.html#rqt-controller-manager).

## Configuration
The motors are configured in the ros2_control tag of the robot description. Example:

```xml
    <xacro:macro name="ros2_control_test_config">
        <ros2_control name="hardware_interface" type="system">
            <hardware>
                <plugin>dynamixel_ros_control/DynamixelHardwareInterface</plugin>
                <param name="port_name">/dev/ttyUSB0</param>       <!-- path to USB serial converter -->
                <param name="baud_rate">57600</param>              <!-- baud rate of the dynamixel motors -->
                <param name="torque_on_startup">true</param>       <!-- enable motor torque on startup -->
                <param name="torque_off_on_shutdown">false</param> <!-- disable motor torque on shutdown -->
            </hardware>

            <joint name="joint_1">
                <param name="id">1</param>  <!-- ID of the dynamixel -->
                <param name="position_control_mode">extended_position</param> <!-- control mode used for the position interface (default: position) -->
                <command_interface name="position"/>
                <command_interface name="velocity"/>
                <state_interface name="position"/>
                <state_interface name="velocity"/>
                <state_interface name="effort"/>
            </joint>
        </ros2_control>
    </xacro:macro>
```

All configured command and state interfaces are made available to controllers. The mapping between interface and register name is given in [interface_to_register_names.yaml](dynamixel_ros_control/devices/interface_to_register_names.yaml). You can also directly use register names, e.g.:
```
<state_interface name="present_input_voltage"/>
```
will make the input voltage available to controllers as a state interface.


