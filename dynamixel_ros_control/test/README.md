# Tests

The tests run the plugin against mock motors, so no hardware is needed. Most of them load the test
robot in `config/athena.urdf`: `athena_arm_interface` (arm joints on ids 11 to 17, the gripper on 18)
and `athena_flipper_interface` (flippers on ids 1 to 4, reductions of plus or minus 2). The motors are
PH54 (model 2020) unless a test changes the model, every motor joint exports `torque_off`, and both
components start with `torque_on_startup` true.

## Mock motors

`MockDynamixel` (`include/dynamixel_ros_control/mock_dynamixel.hpp`) is one motor with the control
table of its model from `devices/`, a kinematic profile, the firmware rules this package relies on
(mode writes only without torque, goal limits, bus watchdog latch, reboot, indirect addresses) and
fault injection for registers, status packets and hardware errors. `MockDynamixelManager` holds the
motors of the process and steps them. They live in `libdynamixel_ros_control_bus_sim`, which the
plugin does not link. The fixtures create the motors of every joint with `use_dummy` (`model_number`
picks the model), and the plugin talks to them through a mock packet handler. `MockBusServer` serves
them over a pty to the real SDK instead.

## Fixtures

- `HardwareInterfaceTest` (`test_hardware_interface_common.hpp`) runs a controller manager with the
  test robot and its controllers in a thread. For behaviour that needs real controllers, services
  and timing.
- `DirectHardwareTest` (`direct_hardware_fixture.hpp`) drives one component directly, one `read()`,
  `write()` and mock step per simulated 20 ms cycle. Deterministic and fast, and it reaches states a
  controller manager cannot provoke.
- `serveOverWire()` (`wire_bus.hpp`) moves a direct component onto a `MockBusServer`, so the same
  test runs through the SDK and a pty.

Every test redirects `HOME` to a temporary directory, so no calibration offsets leak in. Tests with
a ROS node lease their own `ROS_DOMAIN_ID` and can run in parallel.

## Test binaries

**Below the plugin.** `test_dynamixel_driver` (driver, `Dynamixel` and `Joint`: register access,
value encoding, initial values) and `test_sync_managers` (the sync read and write managers), both on
the simulated bus through `DriverTest` (`driver_fixture.hpp`).

**Mock and tables.** `test_mock_dynamixel` (the mock motor), `test_control_table_validation` (every
shipped control table loads and has no overlapping registers), `test_translation_loading`,
`test_bus_watchdog_ticks`, `test_diagnostics_health`.

**Bus simulation.** `test_protocol2_framing` (packets and CRC), `test_mock_bus_server` (the server
on the wire, its faults and power cuts), `test_servo_law` (servo on an external plant, as in gz),
`test_bus_emulator` (the `dynamixel_bus_emulator` executable), `test_hw_wire_faults` (the plugin
through the SDK under corrupt, truncated, dropped and late status packets), `test_hw_contract`
(the hector_controller_manager plugin contract, directly and over the wire).

**SDK.** `test_sdk_late_status` checks a DynamixelSDK bug, not this package. It aborts with "stack
smashing detected" until the SDK has the fix of branch `fix/rx_buffer_overflow`.

**Plugin, direct.** Lifecycle and claims (`test_hw_lifecycle_claims`, `test_hw_switch_refusal`,
`test_hw_unknown_parameters`, `test_hw_port_name`, `test_hw_executor_teardown`,
`test_hw_bus_ownership`), torque and goals (`test_hw_torque_off_mode`, `test_hw_keep_goal`,
`test_hw_nan_hold`), faults (`test_hw_on_error`, `test_hw_securing`, `test_hw_power_blip`), bus
watchdog (`test_hw_bus_watchdog_timing`, `test_hw_watchdog_unsupported`), transmissions
(`test_hw_transmission`, `test_hw_calibration_while_active`).

**Plugin, with a controller manager.** Normal use (`test_hw_normal_usage`, `test_hw_control_mode`,
`test_hw_command_handles`, `test_hw_profile`, `test_hw_mimic`), torque (`test_hw_torque`,
`test_hw_per_joint_torque`, `test_hw_partial_torque`, `test_hw_warm_start`, `test_hw_safety`),
faults and recovery (`test_hw_hardware_error`, `test_hw_communication_error`,
`test_hw_securing_order`, `test_hw_reboot_indirect`, `test_hw_reboot_unconnected`,
`test_hw_strict_init`), lifecycle (`test_hw_lifecycle`, `test_hw_combined_state`,
`test_hw_deactivate_current`), reporting (`test_hw_led`, `test_hw_manifest`, `test_hw_health`,
`test_hw_realtime_publishers`), bus watchdog (`test_hw_bus_watchdog`).

**Plugin, asynchronous under hector_controller_manager.** `test_hw_hcm_async` runs the arm as an
asynchronous component at 100 Hz with the mock's watchdog timing on, and holds the owner thread in
`write()` for whole control cycles: stalls below, at and above the 40 ms watchdog window, a hang
past the hang timeout, a slow owner, the e-stop, controller stops, deactivation and a lost bus
during a held write, and the caps.

**Gazebo** (`dynamixel_ros_control_gz/test`). `test_gz_bus.py` runs the plugin and a controller
manager against servos in a headless gz world, `test_bus_watchdog` checks that the simulated bus
watchdog counts sim time, `test_port_directory` and `test_wait_for_ports` cover the port handling.

## Running

```bash
colcon test --packages-select dynamixel_ros_control dynamixel_ros_control_gz
colcon test-result --verbose
```

`--ctest-args -R <regex>` selects binaries. For a single case run the binary with a filter:

```bash
./build/dynamixel_ros_control/test_hw_keep_goal --gtest_filter='*TorqueBack*'
```

`tools/run_sanitizers.sh` runs the suite under ASan, UBSan and TSan, `scripts/coverage.sh` writes a
coverage report to `build/dynamixel_ros_control/coverage/html/index.html`.
