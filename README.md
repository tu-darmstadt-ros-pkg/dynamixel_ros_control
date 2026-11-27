# dynamixel_ros_control

> **Requires a controller manager that handles the e-stop.** The controllers and the hardware
> interface have no e-stop of their own. `hector_controller_manager` stops them on a hard or soft
> e-stop when both e-stop topics are configured (`estop_topics`). Under the stock `ros2_control`
> controller manager nothing stops them on an e-stop.

![Lint](https://github.com/tu-darmstadt-ros-pkg/dynamixel_ros_control/actions/workflows/lint_build_test.yaml/badge.svg)

_dynamixel_ros_control_ is a [ROS2](https://www.ros.org/) driver for [Robotis Dynamixel](http://www.robotis.us/dynamixel/) actuators. It is based on the [ros2_control](https://control.ros.org/rolling/index.html) framework and implements a hardware interface.


## Features

* Protocol 2.0 Dynamixel support, with control tables shipped for the X (XM, XMExt), MX 2.0,
  P (PH, PM), PRO (H42, H54, L54, M42, M54) and RH series. A model outside those is recognised
  by model number and reports which control table is missing and how to add it.
* Support for mixed chains with different models in the same chain
* Automatic conversion of all registers to SI units
* Synchronized, efficient reading and writing of registers
* Provides state interfaces for **all** readable registers (position, velocity, input voltage, ...)
* Provides the command interfaces position, velocity, current and effort (both goal current), the caps max_current and max_velocity, plus torque_off. on_init refuses any other
* Automatic switching of control mode during runtime
* Securing with automatic position hold when the component faults
* Hardware error detection with per-motor LED indication
* Reboot service for error recovery
* Diagnostics topics (`~/manifest` for static motor info, `~/diagnostics` at 1 Hz for runtime state)

## Installation

### Prerequisites

* ROS 2 (Jazzy or later recommended)
* [vcstool](https://github.com/dirk-thomas/vcstool) (`sudo apt install python3-vcstool`)
* [rosdep](http://wiki.ros.org/rosdep) (`sudo apt install python3-rosdep`)

### Building from Source

1. **Create a workspace** (or use an existing one):

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. **Clone the repository**:

   ```bash
   git clone https://github.com/tu-darmstadt-ros-pkg/dynamixel_ros_control.git
   ```

3. **Import dependencies using vcs**:

   The package has dependencies that are not available via rosdep. Use `vcs import` to clone them.
   Run this recursively to also fetch dependencies of dependencies:

   ```bash
   cd ~/ros2_ws/src
   vcs import --recursive < dynamixel_ros_control/dependencies.repos
   ```
4. **Install rosdep dependencies**:

   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

5. **Build the workspace**:

   ```bash
   cd ~/ros2_ws
   colcon build
   ```

6. **Source the workspace**:

   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

### Running Tests

To build and run the test suite:

```bash
colcon build --packages-select dynamixel_ros_control --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select dynamixel_ros_control
colcon test-result --verbose
```

See [dynamixel_ros_control/test/README.md](dynamixel_ros_control/test/README.md) for detailed test documentation.

## Getting started

A demo configuration can be started by launching:

```
ros2 launch dynamixel_ros_control controller_manager.launch.yaml port_name:=/dev/ttyUSB0 id:=1 baud_rate:=57600
```

This opens the device `/dev/ttyUSB0` with baud rate `57600` for the Dynamixel motor with id `1` and
starts a trajectory controller for it.

### Bench testing

The same launch takes a few arguments meant for working on a desk rather than a robot:

| Argument | Default | Effect |
|---|---|---|
| `torque_on_startup` | `true` | Pass `false` to bring the bus up limp. A joint gets torque when a controller of it starts, so `joint_1` is limp only until the launch's trajectory controller starts, about a second later. It does not give you a motor to turn by hand. |
| `id2` | `0` | ID of a second motor to put on the bus, `0` for none. |
| `velocity_limit` | `1.0` | `velocity_limit` register written to both joints, rad/s. |
| `current_limit` | `0` | `current_limit` register written to both joints, A. `0` leaves the motor's own value alone. |

A failed first activation, for example with a motor unplugged, is not retried by the launch. Retry it with
`ros2 control set_hardware_component_state hardware_interface active`.

Start on the [emulated bus](#emulated-bus): it exercises the description, the controller manager,
the spawners and every controller against simulated motors that move, so a config or wiring mistake
surfaces before a motor is on the bus:

```
ros2 run dynamixel_ros_control dynamixel_bus_emulator $XDG_RUNTIME_DIR/dxl/ttyUSB0 1:2020
ros2 launch dynamixel_ros_control controller_manager.launch.yaml port_name:=$XDG_RUNTIME_DIR/dxl/ttyUSB0 baud_rate:=57600 id:=1
```

Then move to hardware:

```
ros2 launch dynamixel_ros_control controller_manager.launch.yaml \
  port_name:=/dev/ttyUSB0 baud_rate:=57600 id:=1 torque_on_startup:=false current_limit:=1.5
```

`id2` makes the multi-motor paths reachable, such as the sync read/write groups,
the `bus_watchdog`/`torque_enable` padding on a mixed bus, per-joint torque, and securing a bus where one motor answers and another does not. A
second motor of a *different* model covers more of that than a matching pair. Nothing
commands the second joint; it is energised and read, but no controller claims it.

Both joints must declare the same command interfaces. Every joint rides in one sync
write group and the group needs a single byte layout, so a mismatch fails `on_configure`
with a layout error rather than starting. Their control *modes* may differ freely, and
the bench config deliberately puts `joint_2` in plain position mode against `joint_1`'s
`extended_position`.

Move the joint to 1 rad in two seconds with:

```
ros2 topic pub --once /trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint_1], points: [{positions: [1.0], time_from_start: {sec: 2}}]}"
```

Show the joint state with:

```
ros2 topic echo /joint_states
```

You can configure other controllers by
using [rqt_controller_manager](https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/userdoc.html#rqt-controller-manager).

### Emulated bus

`dynamixel_bus_emulator` serves simulated motors as Protocol 2.0 packets on a pseudo terminal, so
the plugin runs unchanged, through the real SDK and its serial port handling. It takes the port to create and the motors as
`<id>:<model number>`.

```
ros2 run dynamixel_ros_control dynamixel_bus_emulator $XDG_RUNTIME_DIR/dxl/ttyUSB0 1:2020 2:54024
```

Then point `port_name` at `$XDG_RUNTIME_DIR/dxl/ttyUSB0`. Keep the path short, because the SDK holds
at most 99 characters. The motors are the mock motors of the package's tests. They move
kinematically and count the bus watchdog in wall time. One emulator serves one bus. In tests,
`MockBusServer` does the same in process and injects wire faults (corrupt, truncated, dropped and
late status packets), bus power cuts and unplugging.

### Simulated buses in Gazebo

`dynamixel_ros_control_gz` serves the same servos in gz sim, one `DynamixelBusSystem` per bus, so a
robot's own launch, plugin and controllers run against physics:

```xml
<plugin filename="dynamixel_ros_control_gz-system" name="dynamixel_ros_control_gz::DynamixelBusSystem">
  <port>/run/user/1000/dxl_sim/athena/ttyUSB_manipulator_arm</port>
  <servo joint="arm_joint_1" id="11" model="2020" reduction="1.0" offset="-0.785" initial="0.0"/>
</plugin>
```

`reduction` and `offset` repeat the transmission of the ros2_control description and must match it,
since gz does not see that block. Derive both from one xacro property, as athena_description does.
`initial` is the start position. The servo constants are datasheet values in
`dynamixel_ros_control_gz/config/servo_models.yaml`. `gz.msgs.Boolean` true on
`/<model>/dynamixel/power_cut` cuts the power of every bus of the model, false boots them. The servo
is a velocity servo under a current limit, not the firmware's cascaded PID, so tracking lag and
stiffness are qualitative. See the sources of `dynamixel_ros_control_gz` for the mimic followers,
the port directory rules and the remaining limits of the model.

## Configuration

The motors are configured in the ros2_control tag of the robot description. Example:

```xml
<xacro:macro name="ros2_control_test_config">
    <ros2_control name="hardware_interface" type="system">
        <hardware>
            <plugin>dynamixel_ros_control/DynamixelHardwareInterface</plugin>
            <param name="port_name">/dev/ttyUSB0</param>             <!-- path to USB serial converter -->
            <param name="baud_rate">57600</param>                    <!-- baud rate of the dynamixel motors -->
            <param name="torque_on_startup">true</param>             <!-- enable motor torque on startup -->
            <param name="torque_off_on_shutdown">false</param>       <!-- disable motor torque on shutdown -->
            <param name="debug">false</param>                        <!-- raise this component's log level to DEBUG -->
        </hardware>

        <joint name="joint_1">
            <param name="id">1</param>  <!-- ID of the dynamixel -->
            <param name="position_control_mode">extended_position</param> <!-- mode of the position interface (default: position) -->
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
in [interface_to_register_names.yaml](dynamixel_ros_control/devices/interface_to_register_names.yaml). A state
interface can also name a register directly, e.g.:

```
<state_interface name="present_input_voltage"/>
```

will make the input voltage available to controllers as a state interface. Command interfaces are limited to position,
velocity, current, effort, max_current, max_velocity and torque_off, since only these have a safe goal to reset to.

### Parameters

Hardware parameters, in `<hardware>`. An unknown name fails `on_init`, and so does a value that does
not parse. A flag takes `true`, `false`, `1` or `0`, in any case and with surrounding whitespace.

| Parameter | Default | Effect |
|---|---|---|
| `port_name` | required | Serial device of the bus |
| `baud_rate` | required | Baud rate of every motor on the bus, in bit/s |
| `torque_on_startup` | `false` | Energise every motor on activation. Without it the bus comes up limp and a joint gets torque when a motion interface of it starts |
| `torque_off_on_shutdown` | `false` | Turn torque off on deactivation |
| `bus_watchdog_cycles` | `4` | Bus watchdog window in control cycles. `0` disables it, values below 3 are refused (see [Bus Watchdog](#bus-watchdog)) |
| `allow_velocity_without_watchdog` | `false` | Permit velocity mode on joints that have no armed bus watchdog |
| `use_dummy` | `false` | Talk to the motors simulated in this process instead of the port. The package's tests, the emulator and the gz host simulate motors in process, in `ros2_control_node` on_init fails. To simulate a bus for the plugin, use the [emulator](#emulated-bus) |
| `debug` | `false` | Log this hardware component at DEBUG level. Its logger is `dynamixel_ros_control.<component name>` |
| `publish_goal_joint_states`, `publish_read_joint_states`, `publish_write_joint_states` | `false` | Debug topics, see [Joint State Publishers](#joint-state-publishers) |

Joint parameters, in `<joint>`:

| Parameter | Default | Effect |
|---|---|---|
| `id` | required | Motor id on the bus |
| `position_control_mode` | `position` | Mode a position claim selects: `position`, `extended_position` or `current_based_position` |
| `do_not_reset_on_ctrl_change` | `false` | Keep the goal across controller changes and missing (NaN) commands, so a gripper keeps squeezing an object. Activation and torque coming back still reset it to the present position |
| `model_number` | `2020` | Model the tests simulate for the joint, ignored by the plugin |
| `registers.<name>` | | Register value written on configure and after a reboot. SI units where the register has one, raw counts otherwise |

## Advanced Features

### Automatic conversion to SI units

Registers carrying a unit are converted from dynamixel counts to SI units automatically. This means, registers can be
read and written using SI units without additional conversions required. Some examples are _goal_position_ and
_max_position_limit_ in radians, _velocity_limit_ in radians per second or _present_temperature_ in celsius.

Acceleration on the older PRO models is the exception: those tables carry a placeholder ratio of 1, because ROBOTIS
specifies their acceleration at the core motor and the factor to the output shaft is model-dependent.

### Torque off as a command mode

A joint that declares `<command_interface name="torque_off"/>` can be made limp by a controller that
claims that interface. The claim itself is the command, no value is ever written to it:

* the mode switch that starts the claim turns the motor's torque off, verified;
* ending the claim alone leaves the joint limp. A controller manager's hold after a fault, which only
  stops interfaces, therefore never brings torque back;
* torque comes back when any other command interface of a limp joint starts, a cap included, so a
  controller that claims `max_current` beside position to set a limit energises the joint too. That
  mode switch sets the goal to the present position and turns torque on last, so the joint holds
  where it was left. To give torque back, end the claim and start the next controller in one switch.

A joint cannot be torque off and commanded at once. Such a switch is refused before anything changes.
Other joints on the same bus keep their torque. A starting command interface energises any limp
joint this way, also one without `torque_off`, for example after an activation with
`torque_on_startup` false. A torque-off claim does not survive a fault or a deactivation. The next
activation applies `torque_on_startup`.

### Current mode

A claim that would put a joint in current mode (current or effort) is refused in
`prepare_command_mode_switch`. A joint in current mode cannot be held once its current command stops,
and until that hold exists the mode is not supported.

### Caps

The main interface a controller claims, `position`, `velocity` or `current`, selects the operating
mode, and only one of them per joint. `max_current` and `max_velocity` limit it and never select a
mode. `prepare_command_mode_switch` refuses

* two main interfaces on one joint, so a limit beside position goes through a cap,
* a cap without a main interface, and `max_velocity` beside anything but `position`,
* `max_current` in a mode in which the model does not limit the current by Goal Current. The model
  YAML lists those modes in `current_cap_modes`: velocity and both position modes on P and PRO,
  only `current_based_position` on X and RH. A model without the list refuses it in every mode.

| Cap | Register | Unit |
|---|---|---|
| `max_current` | Goal Current (RAM) | A |
| `max_velocity` | Profile Velocity (RAM) | rad/s |

`max_velocity` needs a velocity-based profile. Configure fails on a joint that declares it with
bit 2 of `drive_mode` set, in which Profile Velocity is a duration, and on a model without Profile
Velocity, such as the older PRO series. `on_init` refuses it behind a transmission, which would hand
the motor a joint velocity off by the reduction.

A cap above the EEPROM limit (`current_limit`, `velocity_limit`) is clamped to it, and below one
tick raised to one tick, since 0 lifts the cap on PRO Goal Torque and on Profile Velocity. While no
controller claims it, after activation and while it carries NaN, a cap is the EEPROM limit read on
configure. A joint with `do_not_reset_on_ctrl_change` keeps a released cap like its goal, so a
gripper keeps its grip force. The EEPROM limits change only through `registers.*` on configure.
Configure fails when the EEPROM limit of a declared cap cannot be read, since the cap's own register
may still hold the cap of an earlier session. `on_init` refuses a `registers.*` value for a cap's
register (`goal_current`, `profile_velocity`), which the cap would overwrite every cycle.

### Motion Profiles

Dynamixel motors have a built-in trajectory profile generator that shapes motion through controlled acceleration, cruising, and deceleration phases. Without profile configuration, the motor jumps to each new goal position as fast as possible, which causes jerky, staccato-like motion when streaming position commands at high rates (e.g., 50 Hz).

Profile registers can be configured per joint using the `registers.*` URDF parameter mechanism. Their values are written as raw register units (no SI conversion), and their meaning depends on the configured `drive_mode`: velocity-based by default, or time-based when bit 2 of `drive_mode` is set.

For **time-based profiles** (recommended for streaming controllers), set `drive_mode` bit 2 to enable time-based profiling. In this mode, `profile_velocity` specifies the total profile duration in milliseconds and `profile_acceleration` specifies the acceleration time in milliseconds:

```xml
<joint name="arm_joint_1">
  <param name="id">11</param>
  <param name="registers.drive_mode">4</param>
  <param name="registers.profile_velocity">20</param>
  <param name="registers.profile_acceleration">8</param>
  ...
</joint>
```

This configures a 20 ms profile duration (matching a 50 Hz command rate) with 8 ms acceleration ramp.

In velocity-based mode (the default), `profile_velocity` and `profile_acceleration` are raw tick values; consult your motor's control table for the unit (typically 0.229 rev/min and 214.577 rev/min² for X-series motors).

> **Note:** Not all Dynamixel models support profile registers. The older PRO series (H42-20-S300-R, H54-200-S500-R) and the original RH-P12-RN use `goal_acceleration` instead. The hardware interface logs a warning if a configured register is not available on the motor model.

All `registers.*` values are automatically restored after a hardware error reboot, so profile configuration is not lost during recovery.

### E-Stop

The interface has no e-stop of its own. A soft e-stop is the controller manager's: it stops the
controllers, and the joints hold their present position. A joint stopped in velocity mode is put
back into its position mode for that, which drops its torque for the moment of the mode write. A
hard e-stop cuts the motor power, so the reads
fail and the component faults like any lost bus. A cut shorter than the read error threshold
reboots a motor without enough failed reads. While active, a read that finds a motor's torque off
although it should be on reports that reboot as an error, so the component faults all the same.

Known limitation: such a short cut goes unnoticed on a joint that is limp, since its torque is off
either way. The only trace is a single `Bus watchdog is not armed ... clearing and re-arming`
warning, and none on a model without the watchdog. Until the next configure the rebooted motor has
lost its `registers.*` values, and an X series motor also its indirect addresses, so its joint
state is wrong and the next torque-on fails. Bring the component to unconfigured and activate it
again to recover.

### Controller manager requirements

`hector_controller_manager` never runs `read()` or `write()` during a lifecycle callback or a mode
switch. The stock `ros2_control` controller manager does for every switch that is not requested
with `activate_asap`: it runs `perform_command_mode_switch` on its service thread while the control
thread keeps calling `read()` and `write()`. The two never drive the bus at once. `read()` and
`write()` only try the bus lock and never wait. One that finds the bus taken by a switch or a
service skips its cycle with a throttled warning and counts it as `overlap_skipped_cycles` in the
health diagnostics. A switch or a service waits for at most the `read()` or `write()` in flight.
Use hcm, or request switches with `activate_asap`, to avoid the skips.

### LED Status Indicators

The onboard LED reflects the current state of the hardware interface:

| Color | Meaning |
|-------|---------|
| 🟣 **Pink** | Hardware interface is **inactive** or **unconfigured** |
| 🔵 **Blue** | Hardware interface is **active**, motors are **torqued on** (normal operation) |
| 🟢 **Green** | Hardware interface is **active**, motors are **torqued off** (safe for manual movement) |
| 🟡 **Yellow** | Hardware interface is **active**, the motor's torque could not be read back. Treat it as **torqued on** |
| 🔴 **Red** | **Hardware error** detected on this specific motor |

> **Note:** When a hardware error occurs, the affected motor shows red while the other motors keep their torque color.

### Hardware Error Handling

When a Dynamixel motor reports a hardware error (overload, overheating, etc.):

1. `read()` returns ERROR, and the controller manager stops every controller of the component
2. `on_error` stops joints in velocity or current mode and secures the motors (hold or torque off)
3. The affected motor's LED turns **red**
4. The component ends **unconfigured**: no read and no write reach it any more

To recover without restarting the controller manager:

1. Call the **reboot service** (below) to clear the motor's hardware error.
2. Bring the component back with `ros2 control set_hardware_component_state <component> active`,
   which configures and activates it again.
3. Start the controllers again.

A fault recovery resets the gripper goal to its present position, so a held grasp is released.

### Reboot Service

Motors can be rebooted to recover from hardware errors:

```bash
ros2 service call /<hardware_interface>_node/reboot std_srvs/srv/Trigger
```

The reboot service:
* Is refused unless the component is unconfigured, as it is after a fault: a reboot cuts torque and wipes RAM
* Only reboots motors that have hardware errors (not all motors)
* Waits for motors to come back online
* Verifies the hardware error is cleared
* Restores the previous torque state

### Diagnostics

The hardware interface publishes two `diagnostic_msgs/DiagnosticArray` topics:

* `/<hardware_interface>_node/manifest`: transient_local QoS, published once on `on_configure` and after a successful reboot. Its bus status carries the USB latency timer of the port (see [Slow communication](#slow-communication)). Carries static per-motor info read live from EEPROM: model number, firmware version, current `operating_mode` / `drive_mode`, configured limits, `homing_offset`, motion-profile setpoints (`profile_velocity` / `profile_acceleration`), control gains, and declared interfaces. Registers a model does not support are omitted.
* `/<hardware_interface>_node/diagnostics`: 1 Hz, runtime state. Bus-level: mode switch failure, error counters, last successful read. Per joint: `torque_desired`, decoded `hardware_error_status`, `operating_mode_desired`. Compatible with `rqt_robot_monitor` and `diagnostic_aggregator`.

Note that `operating_mode_desired` (and `torque_desired`) on the diagnostics topic reflects the driver's cached intent; the manifest's `operating_mode` is a live hardware readback.

### Joint State Publishers

Optional `sensor_msgs/JointState` topics, off by default. Enable per hardware interface via URDF hardware parameters:

| Parameter | Topic | Content |
|---|---|---|
| `publish_goal_joint_states` | `/<hardware_interface>_node/goal_joint_states` | Joint-space goal (controller intent, pre-transmission) |
| `publish_read_joint_states` | `/<hardware_interface>_node/read_joint_states` | Actuator-space state (pre state-transmission) |
| `publish_write_joint_states` | `/<hardware_interface>_node/write_joint_states` | Actuator-space goal (post command-transmission, only published on a successful bus write) |

Intended for debugging the transmission and goal-vs-state pipeline.

### Bus Watchdog

The hardware interface computes the Dynamixel [Bus Watchdog](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#bus-watchdog98) window during `on_configure` and arms it at the end of `on_activate`, so the register reads 0 until then. The bus watchdog is a safety feature that stops the motor if communication is lost, preventing uncontrolled movement.

The watchdog timeout is set to a multiple of the control loop period (derived from the `<update_rate>` in the URDF `<ros2_control>` tag). The multiple defaults to **4** and can be configured via the `bus_watchdog_cycles` hardware parameter. For example, with an update rate of 50 Hz (20 ms period) and the default of 4 cycles, the watchdog timeout is set to 80 ms. If the motor does not receive any communication within that time, it stops and enters a watchdog error state.

A control mode switch disarms the watchdog of the whole bus while it verifies goals and modes, which can take longer than the window. Before that it writes a zero goal to every torqued velocity or current joint whose interfaces the switch starts or stops, and sends the torque off to every joint the switch makes limp, so a bus lost during the switch cannot leave such a joint running. A velocity joint the switch does not touch, for example an arm moving in velocity while a gripper controller of the same component is switched, keeps its speed without watchdog protection until the switch ends, so it runs on if the PC dies in that window. The switch arms the watchdog again when it ends on an active component, also when it fails, since `write()` sends nothing after a failed switch. A velocity joint with `do_not_reset_on_ctrl_change` is deliberately left running through a switch that touches it, and is equally unprotected during the switch.

Set `bus_watchdog_cycles` to **0** to disable the bus watchdog entirely (the register is written as `0`, the Dynamixel value that turns the feature off). Any other value below **3**, including a negative one, is refused and fails `on_configure`, because a window that short trips on ordinary scheduling jitter and stops the motor until the next cycle notices.

```xml
<ros2_control name="..." type="system">
  <hardware>
    <plugin>dynamixel_ros_control/DynamixelHardwareInterface</plugin>
    <param name="bus_watchdog_cycles">4</param>
    <!-- ... -->
  </hardware>
</ros2_control>
```

> **Note:** Not all Dynamixel models support the bus watchdog. The older PRO series (non-A variants such as H42-20-S300-R, H54-200-S500-R) and the original RH-P12-RN do not have this register. The hardware interface skips these models with a warning on configure that names their joints. Supported models include all X-series (XM, XL, XC, XH, XD, XW), MX 2.0 series, P-series (PH, PM), PRO+ A-variants (e.g. H42-20-S300-R(A)), and RH-P12-RN(A).

A joint without an armed watchdog, because its model has no register or `bus_watchdog_cycles` is 0, keeps running at its last goal when the bus goes silent. Velocity mode is therefore refused on such a joint in `prepare_command_mode_switch`. Position modes, also with a cap claimed beside position, stay allowed, since the motor holds its last position. Set the hardware parameter `allow_velocity_without_watchdog` to `true` to permit velocity mode anyway. Configure then warns and names the affected joints.

### Mimic Joints

If a mimicked joint is part of the controlled joints, the hardware interface can create state interfaces for
all of its mimic joints. To enable this feature, add the mimic joints to the ros2_control section and add their state
interfaces (see an example above).
Their position is the mimicked joint's times the URDF multiplier plus the offset. Velocity, effort and current are
the mimicked joint's times the multiplier.

### Transmission Support

An interface the transmission does not map, such as `current` or `max_current`, passes through
unchanged. So `current` is the motor's own current, sign included, and on a joint with a negative
reduction a current command acts reversed to the joint. That is harmless while current mode is
refused. `effort` reads and writes the same register in amperes but goes through the transmission,
so a SimpleTransmission divides an effort command by the reduction and multiplies the effort state by
it. `max_current` is a limit in motor amperes.

#### SimpleTransmission

For joints with fixed gear reduction and offsets, define them directly in the URDF. See
the [official ROS 2 documentation](https://docs.ros.org/en/ros2_packages/jazzy/api/transmission_interface/generated/classtransmission__interface_1_1SimpleTransmission.html)
for configuration details.

#### AdjustableOffsetTransmission

Supports runtime calibration of joint offsets. Useful for flippers or joints that require manual calibration.

* Offsets can be adjusted **per joint** using external joint position measurements.
* A joint a controller commands is refused, stop that controller first. The others are adjusted between two
  control cycles, so a held joint keeps its actuator goal. A calibration waits for a running switch or lifecycle
  transition, and may cost the control loop one skipped cycle, counted as `overlap_skipped_cycles`.
* New offsets are saved persistently and automatically restored after reboot.
* **Automatic 2π jump correction**: When an actuator briefly loses power and its position resets by a multiple of 2π,
  the transmission detects the jump and compensates the offset automatically. The command transmission is synchronized
  to keep goal positions consistent. See the
  [hector_transmission_interface README](https://github.com/tu-darmstadt-ros-pkg/hector_transmission_interface) for
  details.

To calibrate joints, call:

```bash
ros2 service call /<hardware_interface>_node/adjust_transmission_offsets \
  hector_transmission_interface_msgs/srv/AdjustTransmissionOffsets '
external_joint_measurements:
  name: ["joint_1", "joint_2",]
  position: [0.0, 0.0]
'
```

## Troubleshooting

### Slow communication

You can inspect read/write performance via the controller manager statistics topic. Look for the hardware interface’s
reported read and write durations.
If communication appears slow or irregular, check the following:

* **USB latency timer**
    * Many USB–TTL adapters buffer data by default (often 16 ms).
    * Verify with:
      ```bash
      cat /sys/bus/usb-serial/devices/<ttyUSB0>/latency_timer
      ```
      It should show **1 ms**. Set it with a udev rule, `ATTR{latency_timer}="1"`.
    * `on_configure` resolves `port_name` to its tty, reads this file and warns when the value is
      above 1 ms. The manifest's bus status reports it as `usb_latency_timer_ms`, at WARN level
      when it is too high. A port without the file, such as a pty or a ttyACM adapter, is skipped.
      A file that holds no number is reported with a warning.
    * The DynamixelSDK assumes 16 ms in its packet timeout whatever the adapter is set to, so a
      transaction on a dead bus always waits about 34 ms.
* **Baud rate**
    * Ensure the configured baud rate matches all motors.
    * Higher baud rates (e.g. 3 M or 4.5 M) significantly reduce cycle times.

* **Return Delay Time (per motor)**
    * Each Dynamixel adds this delay before replying.
    * Keep it **near 0** for fastest SyncRead performance

### Slow startup

`on_configure`, `on_activate`, the control mode switch and the securing pass log their phase
durations with a `[timing]` prefix. A cold start writes the indirect address mappings and initial
register values to EEPROM and takes longer than a warm start.

### Configure fails with "indirect address layout ... does not match"

The indirect address mappings live in EEPROM, which a motor only writes while its torque is off.
With `torque_off_on_shutdown: false` the motors stay energised across a restart. If the interface
list or the driver version changed since the last run, the stored layout no longer matches, and
configuration refuses to write it rather than turning the motor off under load. Power-cycle the
actuators, or turn their torque off, then configure again.

### Stack smashing after a late status packet

A late status longer than the expected answer overflows the receive buffers of the SDK's write,
ping and reboot in DynamixelSDK a8d35d5 and earlier, and can kill the controller manager on the
robot. `test_sdk_late_status` stays red, aborting with "stack smashing detected", until the SDK has
the fix of branch `fix/rx_buffer_overflow`.

## Testing

The package ships an integration test suite (gtest + a mock Dynamixel layer)
that exercises the hardware interface without real motors:

```bash
colcon test --packages-select dynamixel_ros_control
```

`benchmark_hw_offset_service_call` measures how long an offset service call holds the bus. It needs
wall time, so it runs only when configured with `-DAMENT_RUN_PERFORMANCE_TESTS=ON`, or directly as
`ROS_AUTOMATIC_DISCOVERY_RANGE=OFF build/dynamixel_ros_control/benchmark_hw_offset_service_call`, which keeps its calls off a running robot.

### Sanitizers

The interface serves its services from its own executor
thread while the controller manager drives `read()`/`write()` from another, so
most of what keeps it safe is a mutex discipline and a handful of atomics. A
race there does not change what a test observes, both orderings leave the same
registers behind. That is why the suite is also run under sanitizers:

```bash
tools/run_sanitizers.sh          # ASan+UBSan, then TSan, whole suite
tools/run_sanitizers.sh tsan     # one of them
tools/run_sanitizers.sh --help   # subsets, regex filters, reusing a build tree
```

Each mode builds into its own `build-<mode>` / `install-<mode>` beside the
workspace's normal ones, so switching back needs no rebuild. Test timeouts are
auto-scaled (×5 under ASan, ×10 under TSan).

Only this package is instrumented; rclcpp and the DDS implementation are not,
so reports naming only their symbols are suppressed in
[`tools/sanitizers/`](tools/sanitizers/). A report with this package's own
symbols on both stacks is real, do not suppress it. TSan costs roughly ten times
the wall clock, so these runs are manual.

## Contribution

Feel free to contribute to this project by opening an issue or a pull request.

### Adding support for new models

The driver contains a database of control tables of supported models. If a control table of a specific model is missing,
it needs to be added for the driver to work. The association of model number to control table is provided
in [model_list.yaml](dynamixel_ros_control/devices/model_list.yaml). If the control table of the missing model matches
an existing control table, simply add an entry here. If the control table is different, a new name has to be chosen and
a new control table file has to be placed
in [dynamixel_ros_control/devices/models/](dynamixel_ros_control/devices/models/). Make sure to use the correct
conversion ratios from Dynamixel value counts to SI units. It is advised to use a high precision to prevent large
rounding errors.
