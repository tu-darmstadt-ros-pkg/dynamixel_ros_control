# Dynamixel ROS Control Tests

## Overview

This directory contains comprehensive tests for the Dynamixel ROS Control hardware interface. The tests verify the correct behavior of motor communication, control modes, safety features (e-stop, torque control), and integration with the ROS 2 controller manager.

## Test Infrastructure

### Mock Dynamixel Interface Layer

The test suite uses a sophisticated mocking layer that simulates real Dynamixel actuator behavior without requiring physical hardware. This allows for rapid, repeatable, and deterministic testing.

#### Key Components

**MockDynamixel Class** (`mock_dynamixel.hpp`)
- Simulates a single Dynamixel servo motor
- Implements register read/write operations matching the PH series (model 2020) protocol
- Models realistic physics including trapezoidal velocity profiles for smooth motion
- Supports multiple control modes: Position Mode, Velocity Mode, Current Mode, and Extended Position Mode
- Enforces position limits (min/max position) in normal position mode, ignored in extended mode
- Simulates LED state tracking (useful for safety status indication)
- Supports indirect addressing for batch register operations

**MockDynamixelManager Class** (`mock_dynamixel.hpp`)
- Singleton manager that maintains a collection of mock motors
- Handles motor creation and lifecycle management
- Provides global communication error injection for stress testing
- Implements the mock serial communication protocol
- Routes Dynamixel protocol packets to appropriate motors

#### Actuator Physics Model

All mock motors currently simulate **PH Series (model 2020)** behavior:
- **Unit Conversions**: Uses PH series tick-to-radian conversions loaded from YAML control tables
- **Velocity Profiles**: Motors accelerate smoothly to commanded velocity using trapezoidal profiles, preventing unrealistic instantaneous changes
- **Position Limits**: In position mode, motors respect min/max position limits; extended position mode ignores these limits
- **Torque Control**: Torque enable register controls whether motors respond to commands
- **LED Colors**: LED state reflects motor status:
  - Blue: Active and torque enabled (normal operation)
  - Green: Torque disabled (safe to touch)
  - Orange: E-stop active (emergency stop engaged)
  - Red: Hardware error detected on this motor
  - Pink: Hardware interface inactive or unconfigured

### Test Configuration

#### URDF Setup
The test uses a robot URDF configuration with two hardware interfaces:
- **athena_arm_interface**: 7 arm joints (motor IDs 11-17) + gripper (ID 18)
  - SimpleTransmission with 1:1 ratio (no mechanical reduction)
  - Joints: arm_joint_1 through arm_joint_7, gripper_servo_joint

- **athena_flipper_interface**: 4 flipper motors (IDs 1-4)
  - AdjustableOffsetTransmission with ±2.0 mechanical reduction
  - Flippers: flipper_fl, flipper_fr, flipper_bl, flipper_br
  - Transmissions allow for offset adjustment for calibration

All motors are configured with:
- `use_dummy: true` - Enables mock motor usage
- `torque_on_startup: true` - Motors start with torque enabled
- Position/velocity/trajectory controllers

#### Motor IDs Reference
```
Arm Joint Mapping:
  arm_joint_1 → Motor ID 11
  arm_joint_2 → Motor ID 12
  arm_joint_3 → Motor ID 13
  arm_joint_4 → Motor ID 14
  arm_joint_5 → Motor ID 15
  arm_joint_6 → Motor ID 16
  arm_joint_7 → Motor ID 17
  gripper_servo_joint → Motor ID 18

Flipper Mapping:
  flipper_fl (front-left, reduction: -2.0) → Motor ID 1
  flipper_fr (front-right, reduction: 2.0) → Motor ID 2
  flipper_bl (back-left, reduction: 2.0) → Motor ID 3
  flipper_br (back-right, reduction: -2.0) → Motor ID 4
```

## Test Files

### `test_mock_dynamixel.cpp`

Unit tests for the mock motor implementation itself. Run with:
```bash
colcon test --packages-select dynamixel_ros_control --ctest-args -R "MockDynamixelTest"
```

**Tests:**
| Test Name | Description |
|-----------|-------------|
| `PingMotor` | Verify ping response from mock motor |
| `PingNonExistentMotor` | Verify ping fails for non-existent motor |
| `ReadIdRegister` | Read motor ID register |
| `ReadModelNumber` | Read model number register |
| `WriteAndRead1Byte` | Write and read back 1-byte register |
| `WriteAndRead4Byte` | Write and read back 4-byte register |
| `PositionModePhysicsBasic` | Basic position mode physics simulation |
| `PositionModeTorqueDisabledNoMovement` | No movement when torque disabled |
| `PositionModeAddressValidation` | Validate address bounds checking |
| `VelocityModePhysicsBasic` | Basic velocity mode physics simulation |
| `VelocityModeReverseDirection` | Velocity mode with negative velocity |
| `CurrentModeBasic` | Current/torque mode operation |
| `SyncReadMultipleMotors` | Bulk read from multiple motors |
| `SyncWriteMultipleMotors` | Bulk write to multiple motors |
| `InjectHardwareError` | Hardware error injection and detection |
| `MultipleHardwareErrors` | Multiple simultaneous hardware errors |
| `ClearHardwareErrorOnReboot` | Reboot clears hardware errors |
| `CommunicationErrorOnPing` | Communication error during ping |
| `CommunicationErrorOnReadWrite` | Communication error during read/write |
| `CommunicationErrorRecovery` | Recovery after communication error cleared |
| `GlobalCommunicationError` | System-wide communication error |
| `ManagerReset` | Reset all motors via manager |
| `RemoveMotor` | Remove motor from manager |
| `GetConnectedIds` | Query connected motor IDs |
| `DirectMotorAccess` | Direct motor object access |
| `AddressLookup` | Address name-to-offset lookup |
| `TestIsolation1` / `TestIsolation2` | Verify test isolation between runs |
| `PositionLimitsEnforcedInPositionMode` | Position limits in position mode |
| `PositionLimitsEnforcedNegativeDirection` | Position limits in negative direction |
| `ExtendedPositionModeIgnoresLimits` | Extended mode ignores limits |
| `LedAccessors` | LED getter/setter methods |
| `LedViaRegisterWrite` | LED control via register writes |
| `HomingOffsetAppliedToPresentPosition` | Homing offset affects position reading |
| `HomingOffsetNegative` | Negative homing offset |
| `ProfileVelocityLimitsMovementSpeed` | Profile velocity limits speed |
| `ProfileVelocityZeroUsesDefault` | Zero profile velocity uses default |
| `OverheatingErrorInjection` | Overheating error injection |
| `IndirectAddressingOneByte` | Indirect addressing for 1-byte |
| `IndirectAddressingMultiByteContiguous` | Indirect addressing contiguous bytes |
| `IndirectAddressingScattered` | Indirect addressing scattered bytes |

### Hardware Interface Integration Tests

Integration tests for the hardware interface with the ROS 2 controller manager. The tests are organized into multiple files by category.

Run all hardware interface tests with:
```bash
colcon test --packages-select dynamixel_ros_control --ctest-args -R "test_hw_"
```

Run a specific test file:
```bash
colcon test --packages-select dynamixel_ros_control --ctest-args -R "test_hw_estop"
```

## Test Files and Test Cases

### `test_hw_normal_usage.cpp`

Normal operation and basic functionality tests.

| Test Name | Description |
|-----------|-------------|
| `NormalUsage_ArmPositionMode` | Load arm position controller, send commands, verify motors reach targets |
| `NormalUsage_FlipperVelocityMode` | Load flipper velocity controller, verify transmission ratios applied |
| `NormalUsage_ControllerSwitch_ArmPositionToVelocity` | Switch from position to velocity controller mid-operation |
| `NormalUsage_SimultaneousMovement` | Arm, flipper, and gripper move independently and simultaneously |
| `RapidControllerSwitch_StressTest` | Rapidly switch controllers without crashes |
| `Gripper_PositionControl` | Position control of gripper servo (ID 18) |
| `MotorLimits_PositionLimitRespected` | Motors stop at configured position limits |
| `StateInterface_PositionVelocityConsistent` | Position and velocity state interfaces are consistent |
| `SimultaneousOperations_ArmAndFlipperIndependent` | Arm and flipper interfaces operate independently |

### `test_hw_estop.cpp`

E-stop (emergency stop) safety feature tests.

| Test Name | Description |
|-----------|-------------|
| `EStop_StopsMovement` | E-stop halts all motors, deactivates controllers, sets orange LED |
| `EStop_MultipleCommandsBlocked` | E-stop blocks all command types (position, velocity) |
| `EStop_VelocityControllerSwitchesToPositionMode` | Activating e-stop with velocity controller switches motors to position mode |
| `EStop_GoalPositionWriteFailurePreventsActivation` | E-stop activation proceeds even if goal position write fails (motor positions remain stable) |
| `EStop_ReactivationMaintainsPositionModeUntilControllerLoaded` | After e-stop release, motors stay in position mode until controller activates |
| `EStop_CannotActivateWhenTorqueOff` | E-stop cannot be activated when torque is disabled |
| `EStop_DeactivationRetriesControllerDeactivation` | E-stop deactivation retries controller deactivation if controllers are still active |
| `EStop_RemainsActiveWhenControllerDeactivationFails` | E-stop remains active if controller deactivation fails during release |

### `test_hw_torque.cpp`

Torque enable/disable functionality tests.

| Test Name | Description |
|-----------|-------------|
| `Torque_DisableTorqueChangesLEDToGreen` | Disabling torque sets LED green (safe to touch) |
| `Torque_EnableTorqueChangesLEDToBlue` | Enabling torque sets LED blue (active) |
| `Torque_CommandsNotExecutedWhenTorqueOff` | Motors don't move when torque is disabled |
| `Torque_OnStartupVerification` | Verifies torque_on_startup parameter enables torque on hardware activation |
| `Torque_GoalPositionUpdatedBeforeReEnable` | Motors stay at previous goal position after torque cycle (goal persists in motor) |
| `Torque_GoalVelocityZeroBeforeReEnable` | Motors stop moving after velocity controller is deactivated via torque disable |
| `Torque_DeactivatesControllersOnDisable` | Disabling torque deactivates active controllers |

### `test_hw_led.cpp`

LED status indication tests.

| Test Name | Description |
|-----------|-------------|
| `LED_BlueWhenActiveAndTorqueOn` | Blue LED when hardware active and torque enabled |
| `LED_GreenWhenTorqueOff` | Green LED when torque disabled |
| `LED_OrangeWhenEStopActive` | Orange LED when e-stop engaged |
| `LED_PinkWhenHardwareInterfaceInactive` | Pink LED when hardware interface is deactivated (inactive state) |
| `LED_BluAfterReactivation` | LED returns to blue after hardware interface reactivation |
| `MockMotor_VerifyPhysicsSimulation` | Verify mock motor physics simulation accuracy |

### `test_hw_transmission.cpp`

Transmission ratio and calibration offset tests.

| Test Name | Description |
|-----------|-------------|
| `Transmission_FlipperVelocityReduction` | Flipper velocity scaled by transmission ratio (±2.0x) |
| `Transmission_FlipperPositionReduction` | Flipper position scaled by transmission ratio |
| `TransmissionOffset_AdjustFlipperOffset` | Runtime calibration offset adjustment via service |
| `TransmissionOffset_JointPositionMatchesExternalMeasurement` | **CRITICAL**: Joint position equals external measurement value after offset calibration |
| `TransmissionOffset_ResetToZero` | Transmission offsets can be reset to zero |

### `test_hw_communication_error.cpp`

Communication error handling and reboot service tests.

| Test Name | Description |
|-----------|-------------|
| `CommunicationError_TemporaryErrorRecovery` | System recovers after transient communication error |
| `CommunicationError_GlobalErrorBlocksOperation` | Global communication error blocks all operations |
| `EdgeCase_ControllerActivationWithCommunicationErrors` | Controller activation behavior under communication errors |
| `RebootService_ResetsMotors` | Reboot service resets motor state |
| `RebootService_OnlyRebootsFaultyMotors` | Reboot is only called for motors with hardware errors |
| `RebootService_RestoresTorqueOnAndBlueLED` | Reboot restores torque ON state (LED pink due to HW deactivation after error) |
| `RebootService_RestoresTorqueOffAndGreenLED` | Reboot restores torque OFF state (LED pink due to HW deactivation after error) |
| `RebootService_NoRebootWhenNoErrors` | No reboots occur when no motors have hardware errors |

### `test_hw_hardware_error.cpp`

Hardware error detection, LED indication, and recovery tests.

| Test Name | Description |
|-----------|-------------|
| `HardwareError_LEDTurnsRedOnError` | Motor with hardware error gets red LED, others get orange (e-stop) |
| `HardwareError_EStopActivatedOnError` | E-stop is automatically activated when hardware error detected |
| `HardwareError_RebootServiceClearsErrorAndReleasesEStop` | Reboot service clears hardware error and releases e-stop |
| `HardwareError_MultipleMotorsWithErrors` | Multiple motors with errors all get red LEDs |
| `HardwareError_HardwareInterfaceDoesNotCrash` | Hardware interface continues running after error (on_error returns SUCCESS) |
| `HardwareError_RebootOnlyAffectedMotors` | Reboot only reboots motors with hardware errors, not all motors |

### `test_hw_safety.cpp`

Safety-critical behavior tests.

| Test Name | Description |
|-----------|-------------|
| `Safety_TorqueEnableFailsWhenGoalWriteFails` | Torque enable fails if goal position write fails (prevents jerky motion) |
| `Safety_NoMovementOnFailedTorqueEnable` | No motor movement occurs on failed torque enable |

### `test_hw_lifecycle.cpp`

Hardware interface lifecycle and service availability tests.

| Test Name | Description |
|-----------|-------------|
| `Lifecycle_TorqueServiceAvailableWhenActive` | Torque service is available when hardware interface is active |
| `Lifecycle_CalibrationServiceAvailableWhenActive` | Calibration offset adjustment service is available when active |
| `Lifecycle_EStopTopicSubscribedWhenActive` | E-stop topic subscription is active when hardware interface is active |

### `test_hw_combined_state.cpp`

Tests for combined/interacting states (e-stop + torque, calibration during special states).

| Test Name | Description |
|-----------|-------------|
| `CombinedState_EStopWhileTorqueOff` | E-stop while torque already off |
| `CombinedState_TorqueOffWhileEStopActive` | Disable torque while e-stop active |
| `CombinedState_CalibrationWhileEStopActive` | Calibration service behavior during e-stop |
| `CombinedState_CalibrationWhileTorqueOff` | Calibration service behavior with torque off |
| `CombinedState_NoSuddenMovementOnAnyStateTransition` | No sudden movement during any state transition |

## Running the Tests

### Run All Tests
```bash
colcon test --packages-select dynamixel_ros_control
colcon test-result --verbose
```

### Run Specific Test Suite
```bash
# Mock Dynamixel unit tests
colcon test --packages-select dynamixel_ros_control --ctest-args -R "MockDynamixelTest"

# All Hardware Interface integration tests
colcon test --packages-select dynamixel_ros_control --ctest-args -R "test_hw_"

# Specific test file (e.g., hardware error tests)
colcon test --packages-select dynamixel_ros_control --ctest-args -R "test_hw_hardware_error"

# E-stop tests only
colcon test --packages-select dynamixel_ros_control --ctest-args -R "test_hw_estop"
```

### Run Specific Test
```bash
colcon test --packages-select dynamixel_ros_control --ctest-args -R "HardwareError_LEDTurnsRedOnError"
```

### Run with Verbose Output
```bash
colcon test --packages-select dynamixel_ros_control --ctest-args -V
```

### Generate Coverage Report
```bash
# Prerequisites: sudo apt install lcov

# Generate HTML coverage report
./src/dynamixel_ros_control/dynamixel_ros_control/scripts/coverage.sh

# Generate and open in browser
./src/dynamixel_ros_control/dynamixel_ros_control/scripts/coverage.sh --open
```

The coverage report is generated at `build/dynamixel_ros_control/coverage/html/index.html`.

## Test Fixture Setup

The `HardwareInterfaceTest` fixture (inherits from `HectorTestFixture`) handles:

1. **Environment Isolation**: Redirects HOME to temp directory to prevent loading persistent calibration offsets
2. **URDF Loading**: Loads test robot description with mock motor configuration
3. **Controller Manager**: Spawns ROS 2 controller manager in separate thread
4. **Controller Configuration**: Loads position, velocity, and trajectory controllers
5. **Mock Motor Initialization**: Creates mock motors matching URDF configuration
6. **Service Clients**: Sets up clients for motor control services (set_torque, reboot, etc.)
7. **Cleanup**: Properly shuts down controller manager and executor threads

## CI Robustness

The tests are designed for CI environments with the following considerations:

**Robust patterns used:**
- Timeout-based polling for controller state changes (`waitForControllerState`)
- Timeout-based polling for hardware interface activation (`waitForHardwareInterfacesActive`)
- Environment isolation via HOME redirection prevents flaky behavior from persistent state
- Mock motors provide deterministic behavior without hardware timing variations
- Service calls use configurable timeouts

**Timing considerations:**
- Fixed sleep durations are used for physics simulation settling (e.g., motor reaching position)
- These sleeps use generous timeouts (1-3 seconds) to accommodate slow CI runners
- The mock physics runs faster than real-time, so timing is generally reliable

**Recommendations for flaky CI environments:**
- If tests timeout, increase the global timeout in CMakeLists.txt test configuration
- The test fixture includes 10-second timeouts for service discovery
- Controller state polling uses 50ms intervals with configurable total timeout

## Design Principles

### Safety First
- E-stop has highest priority and cannot be overridden
- Torque enable/disable is explicit and safe
- Invalid state transitions fail rather than silently succeed
- Hardware errors are propagated and visible

### Realistic Behavior
- Motor physics use smooth acceleration profiles
- Position/velocity limits are enforced like real hardware
- Communication is synchronous with proper timeouts
- LED status reflects actual hardware state

### Testability
- Mock layer allows error injection at any point
- Deterministic behavior enables reproducible tests
- No real hardware dependencies
- Environment isolation ensures clean state per test

### Maintainability
- Clear test organization by feature area
- Descriptive test names following `Category_Behavior` convention
- Reusable helper methods in fixture
- Comprehensive coverage of success and failure paths

## Related Documentation

- Motor Register Map: See `config/PH.yaml`
- Hardware Interface Implementation: See `src/dynamixel_hardware_interface.cpp`
- Mock Motor Implementation: See `include/dynamixel_ros_control/mock_dynamixel.hpp`
