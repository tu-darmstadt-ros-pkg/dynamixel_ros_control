# dynamixel_ros_control
_dynamixel_ros_control_ is a [ROS](https://www.ros.org/) driver for [Robotis Dynamixel](http://www.robotis.us/dynamixel/) actuators. It is based on the  [ros_control](http://wiki.ros.org/ros_control) framework and implements a hardware interface.

**Main features:**
* Support for all protocol 2.0 Dynamixel models
* Support for mixed chains with different models in the same chain
* Synchronized, efficient reading and writing of registers
* Provides position, velocity and effort interfaces, even mixed in the same chain
* Automatic reconnection in case of errors

## Installation
Install dynamixel_ros_control from source by cloning this repository into your catkin workspace. Most dependencies can be installed using [rosdep](http://wiki.ros.org/rosdep). Go into the dynamixel_ros_control folder and execute
```
rosdep install --from-paths . --ignore-src -r -y
```
[cuckoo_time_translator](https://github.com/ethz-asl/cuckoo_time_translator) is not available as a binary and has to be built from source. It is available on Github, so just clone it into your catkin workspace:
```
git clone https://github.com/ethz-asl/cuckoo_time_translator.git
```



## Getting started
The servo chain has to be configured using a yaml configuration file. Considering a simple chain with two Dynamixel motors a minimal configuration file could look like this:

_test_config.yaml:_
```
control_rate: 20.0 # Update cycles per second (Hz)
torque_on_startup: true # Enable torque on startup
torque_off_on_shutdown: true # Disable torque on shutdown
dynamixels:
  port_info:
    port_name: /dev/ttyUSB0
    baud_rate: 3000000
  read_values:
    read_position: true
    read_velocity: false
    read_effort: false
  control_mode: position
  device_info:
    joint_0:
      id: 0
    joint_1:
      id: 1
```
The joint names in ROS are _joint_0_ and _joint_1_. As the control mode is position, a position interface is exposed. This can be used to start a _JointPositionController_, which needs to be defined first:

_test_controllers.yaml_
```
joint_state_controller:
  type: joint_state_controller/JointStateController
  publish_rate: 50 
joint_0_position_controller:
  type: "position_controllers/JointPositionController"
  joint: joint_0
joint_1_position_controller:
  type: "position_controllers/JointPositionController"
  joint: joint_1
```
The _JointStateController_ publishes joint states of all joints on `/joint_states`. Because we are only reading position values from the servos, only the position is reported. The joint controllers offer a topic interface to set joint goal positions.

The following launch file now starts the controller manager and the previously defined controllers. Make sure to update the package name with the name of your package where the respective configuration files are located.
```
<?xml version="1.0"?>
<launch>
  <group ns="arm_control">
    <rosparam file="$(find dynamixel_ros_control)/config/test_controllers.yaml" command="load"/>

    <remap from="joint_states" to="/joint_states" />
    <node pkg="dynamixel_ros_control" type="dynamixel_controller_manager_node" name="controller_manager_node" output="screen" respawn="false" clear_params="true">
      <rosparam file="$(find dynamixel_ros_control)/config/test_config.yaml" command="load" />
    </node>
  
    <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false" output="screen"
          args="joint_state_controller 
                joint_0_position_controller
                joint_1_position_controller">    
    </node>
  </group>
</launch>
```
## Advanced features
### Soft E-Stop
The controller manager subscribes to the topic `~estop` of type `std_msgs/Bool`. If `true` is send, the soft e-stop will be activated. In case of position control, the current position will be hold. In case of velocity or effort control, zero commands will be sent. The soft e-stop function is deactivated by publishing `false`. All controllers are reset to prevent jerking motions before restoring control. If this behavior is not desired, set `~reset_controllers_after_estop` to `false`.

### Changing control modes
The control mode is set with the parameter `~dynamixels/control_mode`. Supported values are:

| Control Mode | Interface |
|-|-|
| effort | Effort |
| velocity | Velocity |
| position | Position |
|extended position | Position |
| current_based_position | Position |

For more details about each control mode, refer to the [dynamixel documentation](http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode). 
The control mode is automatically set during startup. If this behavior is not desired, set the parameter `~write_control_mode` to `false`. Changing the control mode requires the servo to be untorqued.

It is also possible to set individual control modes for each servo. They overwrite the default control mode, e.g.:
```
dynamixels:
  control_mode: velocity # default mode for all servos
  device_info:
    joint_0: # joint_0 is position controlled
      id: 0
      control_mode: position
    joint_1: # joint_1 is velocity controlled
      id: 1
```

### Automatic conversion to SI units
All register values are converted from dynamixel counts to SI units automatically. This means, registers can be read and written using SI units without additional conversions required. Some examples are _goal_position_ and _max_position_limit_ in radians, _velocity_limit_ in radians per second or _present_temperature_ in celsius.
### Writing registers at startup
It can be convenient to write certain registers at each startup, especially if the field is non-persistent (RAM). Some use-cases are:
* Writing joint limits into _max_position_limit_ and _min_position_limit_
* Setting a maximum current in _current_based_position_ control by writing _goal_torque_
* Enabling external ports by writing to _external_port_mod_1_

Note: It is not advised to set _torque_enable_ here. Please use the parameter `~torque_on_startup` instead.

The controller manager provides the parameter `~dynamixels/write_registers` for register writes on startup, e.g.:
```
dynamixels:
  write_registers:
    joint_0:
      external_port_mod_1: 1
      max_position_limit: 3.14 # Radians
      min_position_limit: -3.14 # Radians
    joint_1:
      goal_current: 0.05 # Ampere
```
Valid register names can be found in the respective control table in the folder `dynamixel_ros_control/devices/models`. 

### Writing registers during run time
It is also possible to write registers during run time with a service interface on `~write_register`. It is only possible to write a single value at a time, writing is not synchronized across servos. The service definition is as follows:
```
uint8 DOUBLE_VALUE=0 # Write a double value
uint8 BOOL_VALUE=1   # Write a bool value
uint8 INT_VALUE=2    # Write a signed integer value

# Set either joint_name or id. If both is set, joint_name is used
string joint_name # Name of target joint 
uint8 id # ID of target dynamixel
string register_name # Register name to be written (see control table)

int32 value_type # 0 double value, 1 bool value, 2 signed integer value
float64 dvalue # Double value
bool bvalue   # Bool value
int32 ivalue  # Signed integer value
---
```
You can either write a double value (SI unit), a bool value or an integer value (raw dynamixel count). Which value is written is decided by `value_type` with 0 for a double, 1 for a bool and 2 for a signed integer.
Valid register names can be found in the respective control table in the folder `dynamixel_ros_control/devices/models`. 
### Rebooting servos in error state
The dynamixel will protect itself by shutting down in case that a dangerous situation occurs during operation (over-heating, over-load, ...). The error state can only be cleared by power-cycling or rebooting the motor. A reboot of servos in an error state can be initiated by calling the service `~reboot_if_error_state` of type `std_srvs/Empty`. The respective error state will be read from `hardware_error_status` and printed to the console.

### Time translation
By default, servo states (position, velocity, effort) are time-stamped by receive time. As the Dynamixel converter board is typically attached by USB, this can lead to noticeable jitter in the time stamps (for an example, see [here](https://github.com/ethz-asl/cuckoo_time_translator/wiki#usage-examples-ros-sensor-drivers)). 
To address this issue, this driver implements time translation using [cuckoo_time_translator](https://github.com/ethz-asl/cuckoo_time_translator). The feature can only be used with Dynamixels that provide a realtime clock, e.g. the XM-series. This is indicated by the field `realtime_tick` in the control table.
Time translation is enabled by setting the parameter `~time_translator/device_time/filter_algo` to one of the following values:

| Value | Algorithm |
|-|-|
| 0 | Receive Time only (default) |
| 1 | Convex Hull |
| 2 | Kalman Filter |

The parameter `~time_translator/device_time/switch_time` should be set to `150`.

## Parameters, Topics and Services
### Parameters
| Parameter | Type | Description | Default |
|-|-|-|-|
| ~debug | bool | Enable debug messages | false |
| ~control_rate | int | Frequency in Hz of the update cycle | 25 |
| ~torque_on_startup | bool | Enable motor torque on startup | false |
| ~torque_off_on_shutdown | bool | Disable motor torque on shutdown | false |
|~reset_controllers_after_estop | bool | If true, running controllers are reset after the soft estop has been released. See chapter "Soft E-Stop". | true |
| ~write_control_mode | bool | If true, the control mode specified in `~devices/control_mode` will be written to each servo | true |
| ~dynamixels/port_info/port_name | string | Device path of the serial usb adapter | |
| ~dynamixels/port_info/baud_rate | int | Baud rate of the connection. Has to match the setting in each Dynamixel |
| ~dynamixels/read_values/read_position | bool | Read the current position of each motor in every cycle | true
| ~dynamixels/read_values/read_velocity | bool | Read the current velocity of each motor in every cycle | false
| ~dynamixels/read_values/read_effort | bool | Read the current effort of each motor in every cycle | false
| ~dynamixels/control_mode | string | Set the control / operation mode of the chain. See "Changing control modes" chapter | "position" |
| ~dynamixels/device_info | list | List of joint names and their respective ids. See "Getting started" for examples | |
| ~dynamixels/write_registers | list | List of joint names and registers to write during startup. See chapter "Writing registers at startup" | |
| ~time_translator/device_time/filter_algo | enum | Time translation algorithm. 0: Receive Time, 1: Convex Hull, 2: Kalman Filter| 0
| ~time_translator/device_time/switch_time| double | Time after which switch to a pending clock filter and at the same time start a new pending filter (use 0 to disable switching). Recommended value: 150 | 36000 |

### Subscriptions
| Topic | Type | Description |
|-|-|-|
| ~estop | std_msgs/Bool | If true is received, the soft estop is triggered and the chain will halt operation. See chapter "Soft E-Stop" |
| ~set_torque | std_msgs/Bool | Enables / disables motor torque |

### Service servers
| Service | Type | Descriptions |
|-|-|-|
 | ~reboot_if_error_state | std_srvs/Empty | Reboot all Dynamixels in an error state. See chapter "Rebooting servos in error state" |
 | ~write_register | dynamixel_ros_control_msgs/WriteRegister | Writes one register for one Dynamixel. See chapter "Writing registers during run time". |

## Contribution
Feel free to contribute to this project by opening an issue or a pull request.
### Adding support for new models
The driver contains a database of control tables of supported models. If a control table of a specific model is missing, it needs to be added for the driver to work. The association of model number to control table is provided in `dynamixel_ros_control/devices/model_list.yaml` . If the control table of the missing model matches an existing control table, simply add an entry here. If the control table is different, a new name has to be chosen and a new control table file has to be placed in `dynamixel_ros_control/devices/models/` . Make sure to use the correct conversion ratios from Dynamixel value counts to SI units. It is advised to use a high precision to prevent large rounding errors.

## Planned features
* [diagnostics](http://wiki.ros.org/diagnostics) support
