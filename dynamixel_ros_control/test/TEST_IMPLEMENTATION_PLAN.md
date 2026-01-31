# Hardware Interface Test Implementation Plan

## 1. Analysis of Current Implementation

### 1.1 Mock Dynamixel Implementation - Status: GOOD

The mock implementation in [mock_dynamixel.hpp](include/dynamixel_ros_control/mock_dynamixel.hpp) is well-designed:

**Strengths:**
- Realistic physics simulation with trapezoidal velocity profiles
- Support for multiple control modes (position, velocity, current, extended position)
- Position limits enforcement (respected in position mode, ignored in extended mode)
- Error injection capabilities (hardware errors, communication errors)
- Indirect addressing support for batch operations
- LED state tracking
- Homing offset support
- Unit conversions loaded from YAML control tables

**Minor Issues to Address:**
1. No `model_number` parameter in test URDF joint config - motors default to model 2020 (PH series)
2. The physics simulation uses simplified acceleration model - adequate for testing but not 100% accurate

### 1.2 Current Test File Issues

**test_mock_dynamixel.cpp** - Well structured, covers:
- Basic communication (ping, read, write)
- Position/velocity/current mode physics
- Sync read/write operations
- Hardware/communication error injection
- Position limits
- LED control
- Homing offset
- Indirect addressing

**test_hardware_interface.cpp** - Has issues:
1. **Complex manual controller loading** - The test manually loads/configures/activates controllers which is error-prone
2. **Missing motor ID verification** - Uses ID 11 (correct per URDF), but doesn't verify mock motors are created
3. **Timing issues** - Uses fixed 2s sleep which may not be sufficient for physics to settle
4. **Unit conversion mismatch** - Uses PH series tick ratio but URDF doesn't specify model_number

### 1.3 Test Configuration Analysis

**URDF Configuration:**
- Two hardware interfaces: `athena_arm_interface`, `athena_flipper_interface`
- `use_dummy: true` enables mock motors
- Arm joints: IDs 11-17 (arm_joint_1 to arm_joint_7) + ID 18 (gripper_servo_joint)
- Flipper joints: IDs 1-4 with transmissions (mechanical_reduction: ±2.0)
- Transmissions: SimpleTransmission for arm, AdjustableOffsetTransmission for flippers

**Controllers Configuration:**
- Position/velocity/trajectory controllers for arm, flippers, gripper
- All use JointGroupPositionController or JointGroupVelocityController (standard)

---

## 2. Test Implementation Plan

### 2.1 Test Fixture Improvements

The current fixture needs these improvements:

```cpp
class HardwareInterfaceTest : public HectorTestFixture {
protected:
    // Helper methods to add
    void waitForControllerState(const std::string& controller_name,
                                const std::string& expected_state,
                                std::chrono::seconds timeout = 10s);

    void waitForMotorPosition(uint8_t motor_id, double target_pos,
                              double tolerance = 0.05,
                              std::chrono::seconds timeout = 5s);

    void waitForMotorVelocity(uint8_t motor_id, double target_vel,
                              double tolerance = 0.1,
                              std::chrono::seconds timeout = 5s);

    bool loadAndActivateController(const std::string& controller_name);
    bool deactivateController(const std::string& controller_name);
    bool switchControllers(const std::vector<std::string>& activate,
                          const std::vector<std::string>& deactivate);

    // LED verification
    void verifyLEDColor(uint8_t motor_id, uint8_t r, uint8_t g, uint8_t b);
    void verifyAllLEDsColor(const std::vector<uint8_t>& motor_ids,
                            uint8_t r, uint8_t g, uint8_t b);

    // Service clients
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_torque_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reboot_client_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr e_stop_pub_;
};
```

### 2.2 Test Cases - Normal Usage

#### Test 1: Arm Position Mode
```cpp
TEST_F(HardwareInterfaceTest, NormalUsage_ArmPositionMode) {
    // 1. Load and activate arm_position_controller
    ASSERT_TRUE(loadAndActivateController("arm_position_controller"));
    waitForControllerState("arm_position_controller", "active");

    // 2. Verify all arm motors exist (IDs 11-17)
    for (uint8_t id = 11; id <= 17; ++id) {
        auto motor = MockDynamixelManager::instance().getMotor(id);
        ASSERT_NE(motor, nullptr) << "Motor ID " << (int)id << " not found";
    }

    // 3. Publish position command
    auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/arm_position_controller/commands", 10);
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data = {0.5, 0.3, 0.2, 0.1, -0.1, -0.2, -0.3};  // 7 joints

    waitForPublisherReady(pub);
    pub->publish(cmd);

    // 4. Verify each motor reaches its target position
    //    Note: Transmission ratio is 1.0 for arm joints
    for (size_t i = 0; i < 7; ++i) {
        uint8_t motor_id = 11 + i;
        waitForMotorPosition(motor_id, cmd.data[i], 0.05, 5s);

        auto motor = MockDynamixelManager::instance().getMotor(motor_id);
        EXPECT_NEAR(motor->getCurrentPosition(), cmd.data[i], 0.05)
            << "Motor " << (int)motor_id << " did not reach target";
    }
}
```

#### Test 2: Flipper Velocity Mode with Transmission
```cpp
TEST_F(HardwareInterfaceTest, NormalUsage_FlipperVelocityMode) {
    // 1. Load and activate flipper_velocity_controller
    ASSERT_TRUE(loadAndActivateController("flipper_velocity_controller"));
    waitForControllerState("flipper_velocity_controller", "active");

    // 2. Flipper motor IDs: 1-4, with transmissions:
    //    flipper_fl: reduction=-2.0 (motor moves 2x speed, opposite direction)
    //    flipper_fr: reduction=2.0  (motor moves 2x speed, same direction)
    //    flipper_bl: reduction=2.0
    //    flipper_br: reduction=-2.0

    // 3. Publish velocity command for joints
    auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/flipper_velocity_controller/commands", 10);
    std_msgs::msg::Float64MultiArray cmd;
    double joint_velocity = 1.0;  // rad/s for joint
    cmd.data = {joint_velocity, joint_velocity, joint_velocity, joint_velocity};

    waitForPublisherReady(pub);
    pub->publish(cmd);

    // 4. Wait for motors to reach velocity
    std::this_thread::sleep_for(1s);

    // 5. Verify actuator velocities (2x joint velocity due to transmission)
    // flipper_fl: motor velocity = joint_velocity * (-2.0) = -2.0 rad/s
    // flipper_fr: motor velocity = joint_velocity * (2.0)  = 2.0 rad/s
    double expected_actuator_velocity = joint_velocity * 2.0;

    struct FlipperConfig {
        uint8_t id;
        double reduction;
    };
    std::vector<FlipperConfig> flippers = {
        {1, -2.0},  // FL
        {2, 2.0},   // FR
        {3, 2.0},   // BL
        {4, -2.0}   // BR
    };

    for (const auto& flipper : flippers) {
        auto motor = MockDynamixelManager::instance().getMotor(flipper.id);
        ASSERT_NE(motor, nullptr);

        double expected = joint_velocity * flipper.reduction;
        EXPECT_NEAR(motor->getCurrentVelocity(), expected, 0.2)
            << "Flipper motor " << (int)flipper.id
            << " velocity mismatch. Expected: " << expected
            << ", Got: " << motor->getCurrentVelocity();
    }
}
```

#### Test 3: Controller Switch (Arm Position to Velocity)
```cpp
TEST_F(HardwareInterfaceTest, NormalUsage_ControllerSwitch_ArmPositionToVelocity) {
    // 1. Start with position controller active
    ASSERT_TRUE(loadAndActivateController("arm_position_controller"));
    waitForControllerState("arm_position_controller", "active");

    // 2. Move to a known position
    publishArmPositionCommand({0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
    std::this_thread::sleep_for(3s);

    // 3. Load velocity controller (not active yet)
    loadController("arm_velocity_controller");
    configureController("arm_velocity_controller");

    // 4. Switch controllers
    ASSERT_TRUE(switchControllers({"arm_velocity_controller"},
                                   {"arm_position_controller"}));

    // 5. Verify position controller is inactive
    waitForControllerState("arm_position_controller", "inactive");

    // 6. Verify velocity controller is active
    waitForControllerState("arm_velocity_controller", "active");

    // 7. Send velocity command and verify movement
    publishArmVelocityCommand({0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5});

    // Record positions before
    std::vector<double> positions_before;
    for (uint8_t id = 11; id <= 17; ++id) {
        auto motor = MockDynamixelManager::instance().getMotor(id);
        positions_before.push_back(motor->getCurrentPosition());
    }

    std::this_thread::sleep_for(1s);

    // Verify positions changed (motor is moving)
    for (size_t i = 0; i < 7; ++i) {
        auto motor = MockDynamixelManager::instance().getMotor(11 + i);
        EXPECT_GT(motor->getCurrentPosition(), positions_before[i] + 0.1)
            << "Motor " << (11 + i) << " should have moved";
    }
}
```

#### Test 4: Simultaneous Arm, Flipper, Gripper Movement
```cpp
TEST_F(HardwareInterfaceTest, NormalUsage_SimultaneousMovement) {
    // 1. Activate all controllers
    ASSERT_TRUE(loadAndActivateController("arm_position_controller"));
    ASSERT_TRUE(loadAndActivateController("flipper_velocity_controller"));
    ASSERT_TRUE(loadAndActivateController("gripper_position_controller"));

    // 2. Record initial positions
    auto getArmPositions = [&]() {
        std::vector<double> pos;
        for (uint8_t id = 11; id <= 17; ++id) {
            pos.push_back(MockDynamixelManager::instance().getMotor(id)->getCurrentPosition());
        }
        return pos;
    };

    auto arm_initial = getArmPositions();
    auto gripper_motor = MockDynamixelManager::instance().getMotor(18);
    double gripper_initial = gripper_motor->getCurrentPosition();

    // 3. Send commands to all
    publishArmPositionCommand({0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
    publishFlipperVelocityCommand({1.0, 1.0, 1.0, 1.0});
    publishGripperPositionCommand({0.5});

    // 4. Wait for movement
    std::this_thread::sleep_for(3s);

    // 5. Verify all moved
    auto arm_final = getArmPositions();
    for (size_t i = 0; i < 7; ++i) {
        EXPECT_NE(arm_final[i], arm_initial[i]) << "Arm joint " << i << " didn't move";
    }

    // Flippers should be moving (check velocity)
    for (uint8_t id = 1; id <= 4; ++id) {
        auto motor = MockDynamixelManager::instance().getMotor(id);
        EXPECT_NE(motor->getCurrentVelocity(), 0.0) << "Flipper " << (int)id << " not moving";
    }

    // Gripper should have moved
    EXPECT_NE(gripper_motor->getCurrentPosition(), gripper_initial);
}
```

### 2.3 Test Cases - E-Stop

#### Test 5: E-Stop Stops Movement and Sets Orange LED
```cpp
TEST_F(HardwareInterfaceTest, EStop_StopsMovementAndSetsOrangeLED) {
    // 1. Start arm controller and begin movement
    ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

    // Send command to move far away (will take time)
    publishArmPositionCommand({2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0});

    // 2. Wait for movement to start
    std::this_thread::sleep_for(500ms);

    // Verify motors are moving
    for (uint8_t id = 11; id <= 17; ++id) {
        auto motor = MockDynamixelManager::instance().getMotor(id);
        EXPECT_NE(motor->getCurrentVelocity(), 0.0) << "Motor " << (int)id << " should be moving";
    }

    // 3. Activate E-Stop
    auto e_stop_msg = std_msgs::msg::Bool();
    e_stop_msg.data = true;
    e_stop_pub_->publish(e_stop_msg);

    // 4. Wait for E-Stop to take effect
    std::this_thread::sleep_for(1s);

    // 5. Verify controller is deactivated
    waitForControllerState("arm_position_controller", "inactive", 5s);

    // 6. Verify motors stopped
    for (uint8_t id = 11; id <= 17; ++id) {
        auto motor = MockDynamixelManager::instance().getMotor(id);
        EXPECT_NEAR(motor->getCurrentVelocity(), 0.0, 0.01)
            << "Motor " << (int)id << " should have stopped";
    }

    // 7. Verify LED is orange (255, 165, 0)
    verifyAllLEDsColor({11, 12, 13, 14, 15, 16, 17}, 255, 165, 0);
}
```

#### Test 6: E-Stop Prevents Movement Even if Controller Reloaded
```cpp
TEST_F(HardwareInterfaceTest, EStop_PreventsMovementWhenControllerReloaded) {
    // 1. Activate E-Stop first
    auto e_stop_msg = std_msgs::msg::Bool();
    e_stop_msg.data = true;
    e_stop_pub_->publish(e_stop_msg);
    std::this_thread::sleep_for(500ms);

    // 2. Try to load and activate controller
    loadController("arm_position_controller");
    configureController("arm_position_controller");

    // Attempt to activate (should fail or commands should not execute)
    auto switch_result = switchControllers({"arm_position_controller"}, {});

    // 3. Send position command
    publishArmPositionCommand({1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

    // 4. Wait
    std::this_thread::sleep_for(2s);

    // 5. Motors should NOT have moved (e-stop prevents write)
    for (uint8_t id = 11; id <= 17; ++id) {
        auto motor = MockDynamixelManager::instance().getMotor(id);
        EXPECT_NEAR(motor->getCurrentPosition(), 0.0, 0.05)
            << "Motor " << (int)id << " should not move with e-stop active";
    }

    // 6. Deactivate E-Stop
    e_stop_msg.data = false;
    e_stop_pub_->publish(e_stop_msg);
    std::this_thread::sleep_for(500ms);

    // 7. Now movement should work
    ASSERT_TRUE(loadAndActivateController("arm_position_controller"));
    publishArmPositionCommand({0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
    std::this_thread::sleep_for(3s);

    for (uint8_t id = 11; id <= 17; ++id) {
        auto motor = MockDynamixelManager::instance().getMotor(id);
        EXPECT_NEAR(motor->getCurrentPosition(), 0.5, 0.1)
            << "Motor " << (int)id << " should have moved after e-stop cleared";
    }
}
```

### 2.4 Test Cases - Torque Control

#### Test 7: set_torque_on_startup Parameter
```cpp
TEST_F(HardwareInterfaceTest, Torque_OnStartupParameter) {
    // The URDF has torque_on_startup: true
    // After hardware interface activation, torque should be enabled

    // Verify torque is enabled on all motors
    std::this_thread::sleep_for(1s);  // Wait for activation

    for (uint8_t id = 11; id <= 17; ++id) {
        auto motor = MockDynamixelManager::instance().getMotor(id);
        EXPECT_EQ(motor->read1Byte(512), 1)  // torque_enable register
            << "Motor " << (int)id << " should have torque enabled on startup";
    }

    // LED should be blue (normal mode, can move)
    verifyAllLEDsColor({11, 12, 13, 14, 15, 16, 17}, 0, 0, 255);
}
```

#### Test 8: set_torque Service Deactivates Controllers
```cpp
TEST_F(HardwareInterfaceTest, Torque_SetTorqueDeactivatesControllers) {
    // 1. Activate controller
    ASSERT_TRUE(loadAndActivateController("arm_position_controller"));
    waitForControllerState("arm_position_controller", "active");

    // 2. Call set_torque(false) service
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;

    auto future = set_torque_client_->async_send_request(request);
    ASSERT_EQ(future.wait_for(5s), std::future_status::ready);
    EXPECT_TRUE(future.get()->success);

    // 3. Verify controller was deactivated
    waitForControllerState("arm_position_controller", "inactive", 5s);

    // 4. Verify torque is disabled
    for (uint8_t id = 11; id <= 17; ++id) {
        auto motor = MockDynamixelManager::instance().getMotor(id);
        EXPECT_EQ(motor->read1Byte(512), 0);
    }

    // 5. LED should be green (safe to touch)
    verifyAllLEDsColor({11, 12, 13, 14, 15, 16, 17}, 0, 255, 0);
}
```

#### Test 9: Torque Not Activated on Goal Position Setting Failure (CRITICAL)
```cpp
TEST_F(HardwareInterfaceTest, Torque_NotActivatedOnGoalPositionFailure) {
    // 1. Disable torque first
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;
    auto future = set_torque_client_->async_send_request(request);
    ASSERT_EQ(future.wait_for(5s), std::future_status::ready);

    // 2. Inject communication error on motor 11 (goal position write will fail)
    auto motor = MockDynamixelManager::instance().getMotor(11);
    motor->setCommunicationError(true);

    // 3. Try to enable torque
    request->data = true;
    future = set_torque_client_->async_send_request(request);
    auto response = future.get();

    // 4. Should fail because goal position reset fails
    EXPECT_FALSE(response->success);

    // 5. Verify torque is still disabled on all motors
    for (uint8_t id = 11; id <= 17; ++id) {
        auto m = MockDynamixelManager::instance().getMotor(id);
        EXPECT_EQ(m->read1Byte(512), 0)
            << "Motor " << (int)id << " should NOT have torque enabled after failure";
    }

    // 6. Clear error and verify we can now enable torque
    motor->setCommunicationError(false);
    request->data = true;
    future = set_torque_client_->async_send_request(request);
    response = future.get();
    EXPECT_TRUE(response->success);
}
```

#### Test 10: Commands Written but Not Executed When Torque Off
```cpp
TEST_F(HardwareInterfaceTest, Torque_CommandsNotExecutedWhenTorqueOff) {
    // 1. Disable torque
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;
    auto future = set_torque_client_->async_send_request(request);
    ASSERT_EQ(future.wait_for(5s), std::future_status::ready);

    // 2. Record initial positions
    std::vector<double> initial_positions;
    for (uint8_t id = 11; id <= 17; ++id) {
        auto motor = MockDynamixelManager::instance().getMotor(id);
        initial_positions.push_back(motor->getCurrentPosition());
    }

    // 3. Try to activate controller and send commands
    // (This should work at the controller level but motor shouldn't move)
    ASSERT_TRUE(loadAndActivateController("arm_position_controller"));
    publishArmPositionCommand({1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

    // 4. Wait for would-be movement time
    std::this_thread::sleep_for(3s);

    // 5. Verify motors did NOT move (torque disabled in MockDynamixel::update)
    for (size_t i = 0; i < 7; ++i) {
        auto motor = MockDynamixelManager::instance().getMotor(11 + i);
        EXPECT_NEAR(motor->getCurrentPosition(), initial_positions[i], 0.01)
            << "Motor " << (11 + i) << " should NOT move with torque off";
    }
}
```

#### Test 11: Goal Values Updated Before Torque Re-activation
```cpp
TEST_F(HardwareInterfaceTest, Torque_GoalValuesUpdatedBeforeReactivation) {
    // 1. With torque on, move arm to a position
    ASSERT_TRUE(loadAndActivateController("arm_position_controller"));
    publishArmPositionCommand({0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
    std::this_thread::sleep_for(3s);

    // 2. Disable torque
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;
    auto future = set_torque_client_->async_send_request(request);
    ASSERT_TRUE(future.get()->success);

    // 3. Manually move motor positions (simulating external force)
    for (uint8_t id = 11; id <= 17; ++id) {
        auto motor = MockDynamixelManager::instance().getMotor(id);
        // Directly modify position in memory (as if externally moved)
        // This simulates the motor being moved while torque is off
        motor->write4Byte(motor->getAddress("present_position"),
                          static_cast<uint32_t>(100000));  // ~2 rad
    }

    // 4. Re-enable torque
    request->data = true;
    future = set_torque_client_->async_send_request(request);
    ASSERT_TRUE(future.get()->success);

    // 5. Verify goal_position was updated to current position (not old goal)
    //    This prevents sudden jumps when torque is re-enabled
    for (uint8_t id = 11; id <= 17; ++id) {
        auto motor = MockDynamixelManager::instance().getMotor(id);
        int32_t goal_pos = motor->read4ByteSigned(motor->getAddress("goal_position"));
        int32_t present_pos = motor->read4ByteSigned(motor->getAddress("present_position"));
        EXPECT_NEAR(goal_pos, present_pos, 100)  // Within ~0.002 rad
            << "Goal should match present position after torque re-enable";
    }
}
```

### 2.5 Test Cases - Transmission

#### Test 12: Flipper Transmission Offset Reset
```cpp
TEST_F(HardwareInterfaceTest, Transmission_FlipperOffsetReset) {
    // This tests the AdjustableOffsetManager functionality

    // 1. Activate flipper controller
    ASSERT_TRUE(loadAndActivateController("flipper_position_controller"));

    // 2. Get initial offset value
    // (Offsets are: FL=0.684, FR=-1.177, BL=-0.949, BR=-0.77)

    // 3. Call adjust_transmission_offsets service with external measurement
    // (This would need the hector_transmission_interface service client)

    // 4. Verify:
    //    - Controllers were deactivated during adjustment
    //    - Joint position now equals external_measurement_value
    //    - Controllers can be re-activated

    // Note: Full implementation requires hector_transmission_interface service client
    GTEST_SKIP() << "Requires hector_transmission_interface service client";
}
```

### 2.6 Test Cases - LED Colors

#### Test 13: LED Color States
```cpp
TEST_F(HardwareInterfaceTest, LED_ColorStates) {
    // Test LED colors for different states:
    // - Blue: Normal mode, can move (active + torque on)
    // - Green: Safe to touch (active + torque off)
    // - Red: Hardware interface not active
    // - Orange: E-Stop active

    // --- State: Active + Torque On (Blue) ---
    std::this_thread::sleep_for(1s);
    verifyAllLEDsColor({11, 12, 13, 14, 15, 16, 17}, 0, 0, 255);

    // --- State: Active + Torque Off (Green) ---
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;
    set_torque_client_->async_send_request(request).get();
    std::this_thread::sleep_for(500ms);
    verifyAllLEDsColor({11, 12, 13, 14, 15, 16, 17}, 0, 255, 0);

    // Re-enable torque for next test
    request->data = true;
    set_torque_client_->async_send_request(request).get();
    std::this_thread::sleep_for(500ms);

    // --- State: E-Stop Active (Orange) ---
    auto e_stop_msg = std_msgs::msg::Bool();
    e_stop_msg.data = true;
    e_stop_pub_->publish(e_stop_msg);
    std::this_thread::sleep_for(1s);
    verifyAllLEDsColor({11, 12, 13, 14, 15, 16, 17}, 255, 165, 0);

    // --- State: Inactive (Red) ---
    // Note: Testing inactive state requires deactivating hardware interface
    // which is complex in this test setup
}
```

### 2.7 Test Cases - Quick Start Issues

#### Test 14: No Successful Read Before Controller Start
```cpp
TEST_F(HardwareInterfaceTest, QuickStart_NoReadBeforeControllerStart) {
    // This tests the edge case where a controller is loaded before
    // the first successful read() has occurred.
    // The hardware interface should reject the controller activation.

    // The current implementation checks first_read_successful_ in
    // perform_command_mode_switch() and returns ERROR if false.

    // To test this, we'd need to:
    // 1. Inject communication errors to prevent read from succeeding
    // 2. Try to activate a controller
    // 3. Verify it fails

    // Set all motors to have communication errors
    for (uint8_t id = 11; id <= 17; ++id) {
        auto motor = MockDynamixelManager::instance().getMotor(id);
        if (motor) motor->setCommunicationError(true);
    }

    // Wait a bit for read to fail
    std::this_thread::sleep_for(500ms);

    // Try to activate controller - should fail
    loadController("arm_position_controller");
    configureController("arm_position_controller");

    // This should fail because first_read_successful_ is false
    auto request = std::make_shared<SwitchController::Request>();
    request->activate_controllers = {"arm_position_controller"};
    request->strictness = SwitchController::Request::STRICT;

    auto result = hector_testing_utils::call_service<SwitchController>(
        switch_client_->get(), request, *executor_,
        hector_testing_utils::ServiceCallOptions{.service_timeout = 5s});

    EXPECT_FALSE(result->ok) << "Controller activation should fail without successful read";

    // Clear errors
    for (uint8_t id = 11; id <= 17; ++id) {
        auto motor = MockDynamixelManager::instance().getMotor(id);
        if (motor) motor->setCommunicationError(false);
    }
}
```

---

## 3. Implementation Recommendations

### 3.1 Fix Test URDF

Add `model_number` parameter to joints in the test URDF to ensure correct unit conversions:

```xml
<joint name="arm_joint_1">
  <param name="id">11</param>
  <param name="model_number">2020</param>  <!-- Add this -->
  ...
</joint>
```

### 3.2 Improve Test Fixture

Add these helper methods to reduce code duplication:

```cpp
// Wait helpers with polling
void waitForControllerState(const std::string& name, const std::string& state,
                            std::chrono::seconds timeout);
void waitForPublisherReady(auto& publisher, std::chrono::seconds timeout = 5s);

// Command publishers
void publishArmPositionCommand(const std::vector<double>& positions);
void publishArmVelocityCommand(const std::vector<double>& velocities);
void publishFlipperPositionCommand(const std::vector<double>& positions);
void publishFlipperVelocityCommand(const std::vector<double>& velocities);
void publishGripperPositionCommand(const std::vector<double>& positions);

// LED verification
void verifyAllLEDsColor(const std::vector<uint8_t>& motor_ids, uint8_t r, uint8_t g, uint8_t b);
```

### 3.3 Test Categories

Organize tests into Google Test categories:

```cpp
// Normal usage tests
TEST_F(HardwareInterfaceTest, NormalUsage_ArmPositionMode)
TEST_F(HardwareInterfaceTest, NormalUsage_FlipperVelocityMode)
TEST_F(HardwareInterfaceTest, NormalUsage_ControllerSwitch)
TEST_F(HardwareInterfaceTest, NormalUsage_SimultaneousMovement)

// E-Stop tests
TEST_F(HardwareInterfaceTest, EStop_StopsMovement)
TEST_F(HardwareInterfaceTest, EStop_SetsOrangeLED)
TEST_F(HardwareInterfaceTest, EStop_PreventsMovementAfterControllerReload)

// Torque tests
TEST_F(HardwareInterfaceTest, Torque_OnStartupParameter)
TEST_F(HardwareInterfaceTest, Torque_OffOnShutdownParameter)
TEST_F(HardwareInterfaceTest, Torque_ServiceDeactivatesControllers)
TEST_F(HardwareInterfaceTest, Torque_NotActivatedOnGoalPositionFailure)
TEST_F(HardwareInterfaceTest, Torque_CommandsNotExecutedWhenOff)
TEST_F(HardwareInterfaceTest, Torque_GoalValuesUpdatedBeforeReactivation)

// Transmission tests
TEST_F(HardwareInterfaceTest, Transmission_FlipperOffsetReset)

// LED tests
TEST_F(HardwareInterfaceTest, LED_BlueWhenActiveAndTorqueOn)
TEST_F(HardwareInterfaceTest, LED_GreenWhenTorqueOff)
TEST_F(HardwareInterfaceTest, LED_RedWhenInactive)
TEST_F(HardwareInterfaceTest, LED_OrangeWhenEStopActive)

// Edge case tests
TEST_F(HardwareInterfaceTest, EdgeCase_NoReadBeforeControllerStart)
```

---

## 4. Estimated Test Count

| Category | Test Count |
|----------|-----------|
| Normal Usage | 5 |
| E-Stop | 3 |
| Torque | 6 |
| Transmission | 2 |
| LED | 4 |
| Edge Cases | 2 |
| **Total** | **22** |

---

## 5. Next Steps

1. **Update test URDF** - Add `model_number` parameters
2. **Implement helper methods** - Add fixture helpers
3. **Implement tests in order**:
   - Start with Normal Usage (validates basic setup)
   - Then E-Stop tests
   - Then Torque tests
   - Then LED tests
   - Finally Edge Cases
4. **Add service clients** to fixture for set_torque and reboot services
5. **Run and iterate** - Fix any issues found during testing
