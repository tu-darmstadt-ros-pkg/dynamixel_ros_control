# dynamixel_ros_control

_dynamixel_ros_control_ is a [ROS2](https://www.ros.org/) driver
for [Robotis Dynamixel](http://www.robotis.us/dynamixel/) actuators. It is based on
the  [ros2_control](https://control.ros.org/rolling/index.html) framework and implements a hardware interface.

**Main features:**

* Support for all protocol 2.0 Dynamixel models
* Support for mixed chains with different models in the same chain
* Automatic conversion of all registers to SI units
* Synchronized, efficient reading and writing of registers
* Provides state interfaces for **all** readable registers (position, velocity, input voltage, ...)
* Provides command interfaces for **all** writable registers (position, velocity, LEDs, ...)
* Automatic switching of control mode during runtime
* Automatic reconnection in case of errors

## Installation

Install dynamixel_ros_control from source by cloning this repository into your ros2 workspace. The dependencies can be
installed using [rosdep](http://wiki.ros.org/rosdep). Go into the dynamixel_ros_control folder and execute

```
rosdep install --from-paths . --ignore-src -r -y
```

Afterward, build your workspace.

## Getting started

A demo configuration can be started by launching:

```
ros2 launch dynamixel_ros_control controller_manager.launch.yaml port_name:=/dev/ttyUSB0 id:=1 baud_rate:=57600
```

This will open the device `/dev/ttyUSB0` with baud rate `57600` for the dynamixel motor id `1` with a position
controller.

Send a position command with:

```
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "{data: ["1"]}"
```

Show the joint state with:

```
ros2 topic echo /joint_states
```

You can configure other controllers by
using [rqt_controller_manager](https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/userdoc.html#rqt-controller-manager).

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
            <param name="position_control_mode">extended_position
            </param> <!-- control mode used for the position interface (default: position) -->
            <param name="registers.velocity_limit">2.0</param> <!-- set a register to some initial value -->
            <command_interface name="position"/>
            <command_interface name="velocity"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
        </joint>
        <joint name="mimic_joint_1"> <!-- example of a mimic joint -->
            <state_interface name="position"/>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
        </joint>
    </ros2_control>
</xacro:macro>
```

All configured command and state interfaces are made available to controllers. The mapping between interface and
register name is given
in [interface_to_register_names.yaml](dynamixel_ros_control/devices/interface_to_register_names.yaml). You can also
directly use register names, e.g.:

```
<state_interface name="present_input_voltage"/>
```

will make the input voltage available to controllers as a state interface.

## Advanced features

### Automatic conversion to SI units

All register values are converted from dynamixel counts to SI units automatically. This means, registers can be read and
written using SI units without additional conversions required. Some examples are _goal_position_ and
_max_position_limit_ in radians, _velocity_limit_ in radians per second or _present_temperature_ in celsius.

### Enabling/Disabling Torque

Torque can be toggled via service calls, e.g.:

```bash
ros2 service call /<hardware_interface>/set_torque std_srvs/srv/SetBool "{data: <true|false>}"
```

The hardware interface automatically updates the **goal position register** of each motor to the current position before
enabling torque.
This prevents the motors from abruptly moving toward a previously commanded goal position.
Active Controllers will be deactivated when torque is turned off to prevent unexpected behavior when torque is turned
back on.
For example a position controller would otherwise try to move the motor to the last commanded position when re-enabling
torque.

### Software E-Stop

The hardware interface provides a software emergency‑stop by subscribing to the <hardware_interface_name>/soft_e_stop
topic (std_msgs/msg/Bool). Whenever it receives a true message, all motion commands are suppressed, and the joints hold
their current positions. Once a false message is published, normal control behavior resumes.

⚠️ Note the software E-Stop can only be activated if torque is enabled and all joints have a position command interface.

### LED Status Indicators

The onboard LED reflects the current state of the hardware interface:

* **🔴 Red** – Hardware interface is **inactive** or **unconfigured**
* **🟠 Orange** - The software E‑Stop is engaged. All motion commands are suppressed, ensuring the robot cannot move
* **🔵 Blue** – Hardware interface is **active**, and motors are **torqued on** (controllers can command the joints)
* **🟢 Green** – Hardware interface is **active**, but motors are **torqued off** (safe for manual movement)

### Mimic Joints

If a mimicked joint is part of the controlled joints, the hardware interface can create a state interfaces for
all of its mimic joints. To enable this feature, add the mimic joints to the ros2_control section and add their state
interfaces (see an example above).
Their position and velocity are computed from the mimicked joint using the URDF-defined multiplier and offset.

### Transmission Support

#### SimpleTransmission

For joints with fixed gear reduction and offsets, define them directly in the URDF. See
the [official ROS 2 documentation](https://docs.ros.org/en/ros2_packages/jazzy/api/transmission_interface/generated/classtransmission__interface_1_1SimpleTransmission.html)
for configuration details.

#### AdjustableOffsetTransmission

Supports runtime calibration of joint offsets. Useful for flippers or joints that require manual calibration.

* Offsets can be adjusted **per joint** using external joint position measurements.
* Automatically deactivates all active controllers before adjusting any transmission offsets.
* New offsets are saved persistently and automatically restored after reboot.

To calibrate joints, call:

```bash
ros2 service call /<hardware_interface>/adjust_transmission_offsets \
  hector_transmission_interface_msgs/srv/AdjustTransmissionOffsets '
external_joint_measurements:
  name: ["joint_1", "joint_2",]
  position: [0.0, 0.0]
'
```

## Contribution

Feel free to contribute to this project by opening an issue or a pull request.

### Adding support for new models

The driver contains a database of control tables of supported models. If a control table of a specific model is missing,
it needs to be added for the driver to work. The association of model number to control table is provided
in [model_list.yaml](dynamixel_ros_control/devices/model_list.yaml) . If the control table of the missing model matches
an existing control table, simply add an entry here. If the control table is different, a new name has to be chosen and
a new control table file has to be placed
in [dynamixel_ros_control/devices/models/](dynamixel_ros_control/devices/models/) . Make sure to use the correct
conversion ratios from Dynamixel value counts to SI units. It is advised to use a high precision to prevent large
rounding errors.
