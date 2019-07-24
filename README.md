# dynamixel_ros_control
_dynamixel_ros_control_ is a [ROS](https://www.ros.org/) driver for [Robotis Dynamixel](http://www.robotis.us/dynamixel/) actuators. It is based on the  [ros_control](http://wiki.ros.org/ros_control) framework and implements a hardware interface.

**Main features:**
* Support for all protocol 2.0 Dynamixel models
* Support for mixed chains with different models in the same chain
* Synchronized, efficient reading and writing of registers
* Provides position, velocity and effort interfaces, even mixed in the same chain
* Automatic reconnection in case of errors

## Getting started
Install dynamixel_ros_control by cloning the repository into your catkin workspace and build it.
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
  device_info: # read/write all devices with one packet
    joint_0:
      id: 0
    joint_1:
      id: 1
```
The servos are exposed to ROS as _joint_0_ and _joint_1_. As the control mode is position, a position interface is exposed. This can be used to start a _JointPositionController_, which needs to be defined first:

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
The joint state controller publishes joint states on `/joint_states`. Because we are only reading position values, only the position is reported. The joint controllers expose a topic interface to set joint goal positions.

The following launch file now starts the controller manager and the previously defined controllers:
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

### Changing control modes
The control mode is set with the parameter `~control_mode`. Supported values are:

| Control Mode | Interface |
|-|-|
| effort | Effort |
| velocity | Velocity |
| position | Position |
|extended position | Position |
| current_based_position | Position |

For more details about each control mode, refer to the [dynamixel documentation](http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode). 
The control mode is automatically set during startup. Changing the control mode requires the servo to be untorqued.

It is also possible to set individual control modes for each servo. They overwrite the default control mode, e.g.:
```
  control_mode: velocity # default mode for all servos
  device_info:
    joint_0: # joint_0 is position controlled
      id: 0
      control_mode: position
    joint_1: # joint_1 is velocity controlled
	  id: 1
```

### Automatic conversion to SI units
### Writing registers at startup
### Writing registers during run time
### Rebooting servos in error state

## Parameters and Topics

## Contribution
Feel free to contribute to this project by opening an issue or a pull request.
### Adding support for new models
The driver contains a database of control tables of supported models. If a control table of a specific model is missing, it needs to be added for the driver to work. The association of model number to control table is provided in `dynamixel_ros_control/devices/model_list.yaml` . If the control table of the missing model matches an existing control table, simply add an entry here. If the control table is different, a new name has to be chosen and a new control table file has to be placed in `dynamixel_ros_control/devices/models/` . Make sure to use the correct conversion ratios from Dynamixel value counts to SI units. It is advised to use a high precision to prevent large rounding errors.

## Planned features
* [diagnostics](http://wiki.ros.org/diagnostics) support
