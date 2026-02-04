#include <gtest/gtest.h>
#include <dynamixel_ros_control/dynamixel_driver.hpp>
#include <dynamixel_ros_control/mock_dynamixel.hpp>
#include <cmath>

using namespace dynamixel_ros_control;

// PH-Series (model 2020) register addresses (from PH.yaml)
constexpr uint16_t ADDR_MODEL_NUMBER = 0;
constexpr uint16_t ADDR_ID = 7;
constexpr uint16_t ADDR_OPERATING_MODE = 11;
constexpr uint16_t ADDR_VELOCITY_LIMIT = 44;
constexpr uint16_t ADDR_TORQUE_ENABLE = 512;
constexpr uint16_t ADDR_GOAL_POSITION = 564;
constexpr uint16_t ADDR_GOAL_VELOCITY = 552;
constexpr uint16_t ADDR_PRESENT_POSITION = 580;
constexpr uint16_t ADDR_PRESENT_VELOCITY = 576;
constexpr uint16_t ADDR_HARDWARE_ERROR = 518;

// Model number for PH series (PH54-200-S500-R)
constexpr uint16_t MODEL_PH = 2020;

/**
 * @brief Test fixture that resets the MockDynamixelManager between tests
 */
class MockDynamixelTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Reset the singleton manager before each test
    MockDynamixelManager::instance().reset();
  }

  void TearDown() override
  {
    // Clean up after each test
    MockDynamixelManager::instance().reset();
  }
};

// ============================================================================
// Basic Communication Tests
// ============================================================================

TEST_F(MockDynamixelTest, PingMotor)
{
  DynamixelDriver driver;
  ASSERT_TRUE(driver.init("/dev/ttyUSB0", 57600, true));

  driver.addDummyMotor(1, MODEL_PH);

  uint16_t model_number;
  EXPECT_TRUE(driver.ping(1, model_number));
  EXPECT_EQ(model_number, MODEL_PH);
}

TEST_F(MockDynamixelTest, PingNonExistentMotor)
{
  DynamixelDriver driver;
  ASSERT_TRUE(driver.init("/dev/ttyUSB0", 57600, true));

  // Don't add any motor
  uint16_t model_number;
  EXPECT_FALSE(driver.ping(99, model_number));
}

TEST_F(MockDynamixelTest, ReadIdRegister)
{
  DynamixelDriver driver;
  ASSERT_TRUE(driver.init("/dev/ttyUSB0", 57600, true));

  driver.addDummyMotor(42, MODEL_PH);

  int32_t val;
  EXPECT_TRUE(driver.readRegister(42, ADDR_ID, 1, val));
  EXPECT_EQ(val, 42);
}

TEST_F(MockDynamixelTest, ReadModelNumber)
{
  DynamixelDriver driver;
  ASSERT_TRUE(driver.init("/dev/ttyUSB0", 57600, true));

  driver.addDummyMotor(1, MODEL_PH);

  int32_t val;
  EXPECT_TRUE(driver.readRegister(1, ADDR_MODEL_NUMBER, 2, val));
  EXPECT_EQ(val, MODEL_PH);
}

TEST_F(MockDynamixelTest, WriteAndRead1Byte)
{
  DynamixelDriver driver;
  ASSERT_TRUE(driver.init("/dev/ttyUSB0", 57600, true));

  driver.addDummyMotor(1, MODEL_PH);

  // Write torque enable
  EXPECT_TRUE(driver.writeRegister(1, ADDR_TORQUE_ENABLE, 1, 1));

  int32_t val;
  EXPECT_TRUE(driver.readRegister(1, ADDR_TORQUE_ENABLE, 1, val));
  EXPECT_EQ(val, 1);
}

TEST_F(MockDynamixelTest, WriteAndRead4Byte)
{
  DynamixelDriver driver;
  ASSERT_TRUE(driver.init("/dev/ttyUSB0", 57600, true));

  driver.addDummyMotor(1, MODEL_PH);

  int32_t target_pos = 100000;
  EXPECT_TRUE(driver.writeRegister(1, ADDR_GOAL_POSITION, 4, target_pos));

  int32_t val;
  EXPECT_TRUE(driver.readRegister(1, ADDR_GOAL_POSITION, 4, val));
  EXPECT_EQ(val, target_pos);
}

// ============================================================================
// Position Mode Physics Tests
// ============================================================================

TEST_F(MockDynamixelTest, PositionModePhysicsBasic)
{
  // Test that position mode physics simulation moves motor toward goal
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  auto motor = MockDynamixelManager::instance().getMotor(1);
  ASSERT_NE(motor, nullptr);

  // Verify addresses are cached correctly
  EXPECT_EQ(motor->getAddress("torque_enable"), ADDR_TORQUE_ENABLE);
  EXPECT_EQ(motor->getAddress("goal_position"), ADDR_GOAL_POSITION);
  EXPECT_GT(motor->getAddress("profile_velocity"), 0);  // Should be 560
  EXPECT_GT(motor->getAddress("present_position"), 0);  // Should be 580

  // Set position mode (operating_mode = 3)
  motor->write1Byte(ADDR_OPERATING_MODE, 3);

  // Enable torque
  motor->write1Byte(ADDR_TORQUE_ENABLE, 1);

  // Set profile_velocity (the primary velocity limit for position mode)
  motor->write4Byte(560, 10000);  // profile_velocity

  // Set goal position (positive direction)
  int32_t target_pos = 100000;  // Large goal position
  motor->write4Byte(ADDR_GOAL_POSITION, static_cast<uint32_t>(target_pos));

  // Verify initial position is 0
  EXPECT_NEAR(motor->getCurrentPosition(), 0.0, 0.001);

  // Simulate for significant time (1000 iterations like working tests)
  for (int i = 0; i < 1000; ++i) {
    motor->update(0.01);  // 10 seconds total
  }

  // Motor should have moved toward goal (position > 0)
  EXPECT_GT(motor->getCurrentPosition(), 0.0);
}

TEST_F(MockDynamixelTest, PositionModeTorqueDisabledNoMovement)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  auto motor = MockDynamixelManager::instance().getMotor(1);
  ASSERT_NE(motor, nullptr);

  // Torque is OFF by default
  EXPECT_EQ(motor->read1Byte(ADDR_TORQUE_ENABLE), 0);

  // Set goal position
  motor->write4Byte(ADDR_GOAL_POSITION, 100000);

  // Simulate
  for (int i = 0; i < 100; ++i) {
    motor->update(0.01);
  }

  // Should not have moved (torque disabled)
  EXPECT_NEAR(motor->getCurrentPosition(), 0.0, 0.001);
}

TEST_F(MockDynamixelTest, PositionModeAddressValidation)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  auto motor = MockDynamixelManager::instance().getMotor(1);
  ASSERT_NE(motor, nullptr);

  // Verify all required addresses are loaded from YAML
  EXPECT_GT(motor->getAddress("torque_enable"), 0);
  EXPECT_GT(motor->getAddress("goal_position"), 0);
  EXPECT_GT(motor->getAddress("present_position"), 0);
  EXPECT_GT(motor->getAddress("operating_mode"), 0);

  // Set position mode and verify register write
  motor->write1Byte(ADDR_OPERATING_MODE, 3);
  EXPECT_EQ(motor->read1Byte(ADDR_OPERATING_MODE), 3);

  // Enable torque and verify
  motor->write1Byte(ADDR_TORQUE_ENABLE, 1);
  EXPECT_EQ(motor->read1Byte(ADDR_TORQUE_ENABLE), 1);

  // Set goal and verify
  motor->write4Byte(ADDR_GOAL_POSITION, 50000);
  EXPECT_EQ(motor->read4Byte(ADDR_GOAL_POSITION), 50000u);
}

// ============================================================================
// Velocity Mode Physics Tests
// ============================================================================

TEST_F(MockDynamixelTest, VelocityModePhysicsBasic)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  auto motor = MockDynamixelManager::instance().getMotor(1);
  ASSERT_NE(motor, nullptr);

  // Set velocity control mode (operating_mode = 1)
  motor->write1Byte(ADDR_OPERATING_MODE, 1);

  // Enable torque
  motor->write1Byte(ADDR_TORQUE_ENABLE, 1);

  // Set goal velocity (positive direction)
  int32_t goal_vel_ticks = 10000;
  motor->write4Byte(ADDR_GOAL_VELOCITY, static_cast<uint32_t>(goal_vel_ticks));

  // Record initial state
  double initial_pos = motor->getCurrentPosition();

  // Simulate
  for (int i = 0; i < 100; ++i) {
    motor->update(0.01);
  }

  // Position should have changed (motor is rotating)
  double final_pos = motor->getCurrentPosition();
  EXPECT_GT(final_pos, initial_pos);

  // Velocity should be positive (matching goal direction)
  EXPECT_GT(motor->getCurrentVelocity(), 0.0);
}

TEST_F(MockDynamixelTest, VelocityModeReverseDirection)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  auto motor = MockDynamixelManager::instance().getMotor(1);
  ASSERT_NE(motor, nullptr);

  // Set velocity control mode
  motor->write1Byte(ADDR_OPERATING_MODE, 1);
  motor->write1Byte(ADDR_TORQUE_ENABLE, 1);

  // Set negative goal velocity (using 2's complement for signed value)
  int32_t goal_vel = -10000;
  motor->write4Byte(ADDR_GOAL_VELOCITY, static_cast<uint32_t>(goal_vel));

  // Simulate
  double initial_pos = motor->getCurrentPosition();
  for (int i = 0; i < 100; ++i) {
    motor->update(0.01);
  }

  // Position should decrease (negative direction)
  double final_pos = motor->getCurrentPosition();
  EXPECT_LT(final_pos, initial_pos);

  // Velocity should be negative
  EXPECT_LT(motor->getCurrentVelocity(), 0.0);
}

// ============================================================================
// Current/Torque Mode Physics Tests
// ============================================================================

TEST_F(MockDynamixelTest, CurrentModeBasic)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  auto motor = MockDynamixelManager::instance().getMotor(1);
  ASSERT_NE(motor, nullptr);

  // Set current control mode (operating_mode = 0)
  motor->write1Byte(ADDR_OPERATING_MODE, 0);
  motor->write1Byte(ADDR_TORQUE_ENABLE, 1);

  // Write goal current (use goal_torque address from PH.yaml = 550)
  uint16_t goal_current_addr = motor->getAddress("goal_torque");
  if (goal_current_addr == 0) {
    goal_current_addr = 550;  // Fallback to PH series address
  }
  motor->write2Byte(goal_current_addr, 1000);

  // Simulate
  for (int i = 0; i < 100; ++i) {
    motor->update(0.01);
  }

  // In current mode, current causes acceleration
  // Velocity should have increased from 0
  double velocity = motor->getCurrentVelocity();
  EXPECT_GT(std::abs(velocity), 0.0);
}

// ============================================================================
// Sync Read/Write Tests
// ============================================================================

TEST_F(MockDynamixelTest, SyncReadMultipleMotors)
{
  DynamixelDriver driver;
  ASSERT_TRUE(driver.init("/dev/ttyUSB0", 57600, true));

  // Add multiple motors
  driver.addDummyMotor(1, MODEL_PH);
  driver.addDummyMotor(2, MODEL_PH);
  driver.addDummyMotor(3, MODEL_PH);

  // Write distinct positions to each
  ASSERT_TRUE(driver.writeRegister(1, ADDR_GOAL_POSITION, 4, 1000));
  ASSERT_TRUE(driver.writeRegister(2, ADDR_GOAL_POSITION, 4, 2000));
  ASSERT_TRUE(driver.writeRegister(3, ADDR_GOAL_POSITION, 4, 3000));

  // Create sync read for goal_position
  auto sync_read = driver.setSyncRead(ADDR_GOAL_POSITION, 4);
  ASSERT_NE(sync_read, nullptr);

  sync_read->addParam(1);
  sync_read->addParam(2);
  sync_read->addParam(3);

  EXPECT_EQ(sync_read->txRxPacket(), COMM_SUCCESS);

  // Verify each motor's data
  EXPECT_TRUE(sync_read->isAvailable(1, ADDR_GOAL_POSITION, 4));
  EXPECT_TRUE(sync_read->isAvailable(2, ADDR_GOAL_POSITION, 4));
  EXPECT_TRUE(sync_read->isAvailable(3, ADDR_GOAL_POSITION, 4));

  EXPECT_EQ(sync_read->getData(1, ADDR_GOAL_POSITION, 4), 1000u);
  EXPECT_EQ(sync_read->getData(2, ADDR_GOAL_POSITION, 4), 2000u);
  EXPECT_EQ(sync_read->getData(3, ADDR_GOAL_POSITION, 4), 3000u);
}

TEST_F(MockDynamixelTest, SyncWriteMultipleMotors)
{
  DynamixelDriver driver;
  ASSERT_TRUE(driver.init("/dev/ttyUSB0", 57600, true));

  driver.addDummyMotor(1, MODEL_PH);
  driver.addDummyMotor(2, MODEL_PH);

  // Create sync write for torque enable
  auto sync_write = driver.setSyncWrite(ADDR_TORQUE_ENABLE, 1);
  ASSERT_NE(sync_write, nullptr);

  uint8_t torque_on = 1;
  sync_write->addParam(1, &torque_on);
  sync_write->addParam(2, &torque_on);

  EXPECT_EQ(sync_write->txPacket(), COMM_SUCCESS);

  // Verify both motors have torque enabled
  int32_t val1, val2;
  ASSERT_TRUE(driver.readRegister(1, ADDR_TORQUE_ENABLE, 1, val1));
  ASSERT_TRUE(driver.readRegister(2, ADDR_TORQUE_ENABLE, 1, val2));

  EXPECT_EQ(val1, 1);
  EXPECT_EQ(val2, 1);
}

// ============================================================================
// Hardware Error Tests
// ============================================================================

TEST_F(MockDynamixelTest, InjectHardwareError)
{
  // Use direct mock access since driver fails on hardware error (by design)
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  auto motor = MockDynamixelManager::instance().getMotor(1);
  ASSERT_NE(motor, nullptr);

  // Inject overheating error
  motor->setHardwareError(ERROR_OVERHEATING);

  // Read hardware error register directly from mock
  EXPECT_EQ(motor->getHardwareError(), ERROR_OVERHEATING);
  EXPECT_EQ(motor->read1Byte(ADDR_HARDWARE_ERROR), ERROR_OVERHEATING);
}

TEST_F(MockDynamixelTest, MultipleHardwareErrors)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  auto motor = MockDynamixelManager::instance().getMotor(1);
  ASSERT_NE(motor, nullptr);

  // Inject multiple errors (bitmap)
  uint8_t errors = ERROR_OVERHEATING | ERROR_OVERLOAD;
  motor->setHardwareError(errors);

  EXPECT_EQ(motor->getHardwareError(), errors);
  EXPECT_EQ(motor->read1Byte(ADDR_HARDWARE_ERROR), errors);
}

TEST_F(MockDynamixelTest, ClearHardwareErrorOnReboot)
{
  DynamixelDriver driver;
  ASSERT_TRUE(driver.init("/dev/ttyUSB0", 57600, true));

  driver.addDummyMotor(1, MODEL_PH);

  auto motor = MockDynamixelManager::instance().getMotor(1);
  motor->setHardwareError(ERROR_OVERLOAD);

  // Verify error is set (directly via mock)
  EXPECT_EQ(motor->getHardwareError(), ERROR_OVERLOAD);
  EXPECT_EQ(motor->read1Byte(ADDR_HARDWARE_ERROR), ERROR_OVERLOAD);

  // Reboot should clear the error
  EXPECT_TRUE(driver.reboot(1));

  EXPECT_EQ(motor->getHardwareError(), 0);
  EXPECT_EQ(motor->read1Byte(ADDR_HARDWARE_ERROR), 0);
}

// ============================================================================
// Communication Error Tests
// ============================================================================

TEST_F(MockDynamixelTest, CommunicationErrorOnPing)
{
  DynamixelDriver driver;
  ASSERT_TRUE(driver.init("/dev/ttyUSB0", 57600, true));

  driver.addDummyMotor(1, MODEL_PH);

  // Enable communication error
  auto motor = MockDynamixelManager::instance().getMotor(1);
  motor->setCommunicationError(true);

  // Ping should fail
  uint16_t model_number;
  EXPECT_FALSE(driver.ping(1, model_number));
}

TEST_F(MockDynamixelTest, CommunicationErrorOnReadWrite)
{
  DynamixelDriver driver;
  ASSERT_TRUE(driver.init("/dev/ttyUSB0", 57600, true));

  driver.addDummyMotor(1, MODEL_PH);

  auto motor = MockDynamixelManager::instance().getMotor(1);
  motor->setCommunicationError(true);

  // Read should fail
  int32_t val;
  EXPECT_FALSE(driver.readRegister(1, ADDR_TORQUE_ENABLE, 1, val));

  // Write should fail
  EXPECT_FALSE(driver.writeRegister(1, ADDR_TORQUE_ENABLE, 1, 1));
}

TEST_F(MockDynamixelTest, CommunicationErrorRecovery)
{
  DynamixelDriver driver;
  ASSERT_TRUE(driver.init("/dev/ttyUSB0", 57600, true));

  driver.addDummyMotor(1, MODEL_PH);

  auto motor = MockDynamixelManager::instance().getMotor(1);

  // Enable error
  motor->setCommunicationError(true);
  EXPECT_FALSE(driver.ping(1));

  // Clear error
  motor->setCommunicationError(false);
  EXPECT_TRUE(driver.ping(1));
}

TEST_F(MockDynamixelTest, GlobalCommunicationError)
{
  DynamixelDriver driver;
  ASSERT_TRUE(driver.init("/dev/ttyUSB0", 57600, true));

  driver.addDummyMotor(1, MODEL_PH);
  driver.addDummyMotor(2, MODEL_PH);

  // Set global communication error
  MockDynamixelManager::instance().setGlobalCommunicationError(true);

  EXPECT_FALSE(driver.ping(1));
  EXPECT_FALSE(driver.ping(2));

  // Clear
  MockDynamixelManager::instance().setGlobalCommunicationError(false);

  EXPECT_TRUE(driver.ping(1));
  EXPECT_TRUE(driver.ping(2));
}

// ============================================================================
// Manager Tests
// ============================================================================

TEST_F(MockDynamixelTest, ManagerReset)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  MockDynamixelManager::instance().addMotor(2, MODEL_PH);

  EXPECT_NE(MockDynamixelManager::instance().getMotor(1), nullptr);
  EXPECT_NE(MockDynamixelManager::instance().getMotor(2), nullptr);

  MockDynamixelManager::instance().reset();

  EXPECT_EQ(MockDynamixelManager::instance().getMotor(1), nullptr);
  EXPECT_EQ(MockDynamixelManager::instance().getMotor(2), nullptr);
}

TEST_F(MockDynamixelTest, RemoveMotor)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  MockDynamixelManager::instance().addMotor(2, MODEL_PH);

  MockDynamixelManager::instance().removeMotor(1);

  EXPECT_EQ(MockDynamixelManager::instance().getMotor(1), nullptr);
  EXPECT_NE(MockDynamixelManager::instance().getMotor(2), nullptr);
}

TEST_F(MockDynamixelTest, GetConnectedIds)
{
  MockDynamixelManager::instance().addMotor(5, MODEL_PH);
  MockDynamixelManager::instance().addMotor(10, MODEL_PH);
  MockDynamixelManager::instance().addMotor(15, MODEL_PH);

  auto ids = MockDynamixelManager::instance().getConnectedIds();

  EXPECT_EQ(ids.size(), 3u);
  EXPECT_NE(std::find(ids.begin(), ids.end(), 5), ids.end());
  EXPECT_NE(std::find(ids.begin(), ids.end(), 10), ids.end());
  EXPECT_NE(std::find(ids.begin(), ids.end(), 15), ids.end());
}

// ============================================================================
// Direct Motor Access Tests
// ============================================================================

TEST_F(MockDynamixelTest, DirectMotorAccess)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  auto motor = MockDynamixelManager::instance().getMotor(1);
  ASSERT_NE(motor, nullptr);

  EXPECT_EQ(motor->getId(), 1);
  EXPECT_EQ(motor->getModelNumber(), MODEL_PH);
}

TEST_F(MockDynamixelTest, AddressLookup)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  auto motor = MockDynamixelManager::instance().getMotor(1);
  ASSERT_NE(motor, nullptr);

  // Verify address lookup works
  EXPECT_EQ(motor->getAddress("torque_enable"), ADDR_TORQUE_ENABLE);
  EXPECT_EQ(motor->getAddress("goal_position"), ADDR_GOAL_POSITION);
  EXPECT_EQ(motor->getAddress("present_position"), ADDR_PRESENT_POSITION);
  EXPECT_EQ(motor->getAddress("hardware_error_status"), ADDR_HARDWARE_ERROR);

  // Unknown register should return 0
  EXPECT_EQ(motor->getAddress("nonexistent_register"), 0);
}

// ============================================================================
// Test Isolation
// ============================================================================

TEST_F(MockDynamixelTest, TestIsolation1)
{
  // This test adds motor ID 1
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  EXPECT_NE(MockDynamixelManager::instance().getMotor(1), nullptr);
}

TEST_F(MockDynamixelTest, TestIsolation2)
{
  // This test should not see motor from previous test (due to reset in SetUp)
  EXPECT_EQ(MockDynamixelManager::instance().getMotor(1), nullptr);

  // Add motor 1 fresh
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  EXPECT_NE(MockDynamixelManager::instance().getMotor(1), nullptr);
}

// ============================================================================
// Position Limits Tests
// ============================================================================

// PH Series position limit addresses
constexpr uint16_t ADDR_MAX_POSITION_LIMIT = 48;
constexpr uint16_t ADDR_MIN_POSITION_LIMIT = 52;

TEST_F(MockDynamixelTest, PositionLimitsEnforcedInPositionMode)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  auto motor = MockDynamixelManager::instance().getMotor(1);
  ASSERT_NE(motor, nullptr);

  // Set position mode (operating_mode = 3)
  motor->write1Byte(ADDR_OPERATING_MODE, 3);
  motor->write1Byte(ADDR_TORQUE_ENABLE, 1);

  // Set position limits (in ticks)
  // PH series: rad_per_tick = 0.00000625911
  // Set limits at approximately +/- 1 rad (160000 ticks each way)
  int32_t max_limit = 160000;
  int32_t min_limit = -160000;
  motor->write4Byte(ADDR_MAX_POSITION_LIMIT, static_cast<uint32_t>(max_limit));
  motor->write4Byte(ADDR_MIN_POSITION_LIMIT, static_cast<uint32_t>(min_limit));

  // Set velocity/acceleration for movement
  motor->write4Byte(560, 10000);  // profile_velocity

  // Try to move beyond maximum position limit
  int32_t beyond_max = 500000;  // Way beyond max_limit
  motor->write4Byte(ADDR_GOAL_POSITION, static_cast<uint32_t>(beyond_max));

  // Simulate for a long time
  for (int i = 0; i < 1000; ++i) {
    motor->update(0.01);  // 10 seconds
  }

  // Motor should stop at or before max_limit
  double max_limit_rad = static_cast<double>(max_limit) * 0.00000625911;
  EXPECT_LE(motor->getCurrentPosition(), max_limit_rad + 0.001);
}

TEST_F(MockDynamixelTest, PositionLimitsEnforcedNegativeDirection)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  auto motor = MockDynamixelManager::instance().getMotor(1);
  ASSERT_NE(motor, nullptr);

  // Position mode
  motor->write1Byte(ADDR_OPERATING_MODE, 3);
  motor->write1Byte(ADDR_TORQUE_ENABLE, 1);

  // Set position limits
  int32_t max_limit = 160000;
  int32_t min_limit = -160000;
  motor->write4Byte(ADDR_MAX_POSITION_LIMIT, static_cast<uint32_t>(max_limit));
  motor->write4Byte(ADDR_MIN_POSITION_LIMIT, static_cast<uint32_t>(min_limit));

  motor->write4Byte(560, 10000);  // profile_velocity

  // Try to move beyond minimum position limit (negative)
  int32_t beyond_min = -500000;
  motor->write4Byte(ADDR_GOAL_POSITION, static_cast<uint32_t>(beyond_min));

  // Simulate
  for (int i = 0; i < 1000; ++i) {
    motor->update(0.01);
  }

  // Motor should stop at or after min_limit
  double min_limit_rad = static_cast<double>(min_limit) * 0.00000625911;
  EXPECT_GE(motor->getCurrentPosition(), min_limit_rad - 0.001);
}

TEST_F(MockDynamixelTest, ExtendedPositionModeIgnoresLimits)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  auto motor = MockDynamixelManager::instance().getMotor(1);
  ASSERT_NE(motor, nullptr);

  // Set EXTENDED position mode (operating_mode = 4) - multi-turn
  motor->write1Byte(ADDR_OPERATING_MODE, 4);
  motor->write1Byte(ADDR_TORQUE_ENABLE, 1);

  // Set position limits (these should be IGNORED in extended mode)
  int32_t max_limit = 160000;
  int32_t min_limit = -160000;
  motor->write4Byte(ADDR_MAX_POSITION_LIMIT, static_cast<uint32_t>(max_limit));
  motor->write4Byte(ADDR_MIN_POSITION_LIMIT, static_cast<uint32_t>(min_limit));

  motor->write4Byte(560, 10000);  // profile_velocity

  // Set goal well beyond the "limit"
  int32_t beyond_max = 500000;
  motor->write4Byte(ADDR_GOAL_POSITION, static_cast<uint32_t>(beyond_max));

  // Simulate for a very long time
  for (int i = 0; i < 2000; ++i) {
    motor->update(0.01);  // 20 seconds
  }

  // In extended mode, motor should have moved past the limit
  double max_limit_rad = static_cast<double>(max_limit) * 0.00000625911;
  EXPECT_GT(motor->getCurrentPosition(), max_limit_rad);
}

// ============================================================================
// LED Control Tests
// ============================================================================

constexpr uint16_t ADDR_LED_RED = 513;
constexpr uint16_t ADDR_LED_GREEN = 514;
constexpr uint16_t ADDR_LED_BLUE = 515;

TEST_F(MockDynamixelTest, LedAccessors)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  auto motor = MockDynamixelManager::instance().getMotor(1);
  ASSERT_NE(motor, nullptr);

  // Initially LEDs should be off
  EXPECT_EQ(motor->getLedRed(), 0);
  EXPECT_EQ(motor->getLedGreen(), 0);
  EXPECT_EQ(motor->getLedBlue(), 0);

  // Set LED colors using accessors
  motor->setLedRed(255);
  motor->setLedGreen(128);
  motor->setLedBlue(64);

  // Verify via accessors
  EXPECT_EQ(motor->getLedRed(), 255);
  EXPECT_EQ(motor->getLedGreen(), 128);
  EXPECT_EQ(motor->getLedBlue(), 64);

  // Verify via direct register read
  EXPECT_EQ(motor->read1Byte(ADDR_LED_RED), 255);
  EXPECT_EQ(motor->read1Byte(ADDR_LED_GREEN), 128);
  EXPECT_EQ(motor->read1Byte(ADDR_LED_BLUE), 64);
}

TEST_F(MockDynamixelTest, LedViaRegisterWrite)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  auto motor = MockDynamixelManager::instance().getMotor(1);
  ASSERT_NE(motor, nullptr);

  // Write LED values directly to registers
  motor->write1Byte(ADDR_LED_RED, 100);
  motor->write1Byte(ADDR_LED_GREEN, 150);
  motor->write1Byte(ADDR_LED_BLUE, 200);

  // Read back via accessors
  EXPECT_EQ(motor->getLedRed(), 100);
  EXPECT_EQ(motor->getLedGreen(), 150);
  EXPECT_EQ(motor->getLedBlue(), 200);
}

// ============================================================================
// Homing Offset Tests
// ============================================================================

constexpr uint16_t ADDR_HOMING_OFFSET = 20;

TEST_F(MockDynamixelTest, HomingOffsetAppliedToPresentPosition)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  auto motor = MockDynamixelManager::instance().getMotor(1);
  ASSERT_NE(motor, nullptr);

  // Set homing offset (in ticks)
  int32_t homing_offset = 10000;
  motor->setHomingOffset(homing_offset);

  // Verify it was written
  EXPECT_EQ(motor->getHomingOffset(), homing_offset);

  // Enable torque and set a goal position
  // Goal Position is in the same coordinate system as Present Position (includes homing offset)
  // So to keep the motor at raw position 0, we need to set Goal = homing_offset
  motor->write1Byte(ADDR_OPERATING_MODE, 3);
  motor->write1Byte(ADDR_TORQUE_ENABLE, 1);
  motor->write4Byte(560, 10000);                         // profile_velocity
  motor->write4Byte(ADDR_GOAL_POSITION, homing_offset);  // Goal = homing_offset to stay at raw 0

  // Motor's actual position is 0, and present_position = actual + homing_offset
  motor->update(0.01);

  // Present position = actual position + homing offset
  int32_t present_pos = static_cast<int32_t>(motor->read4Byte(ADDR_PRESENT_POSITION));
  // Actual position is 0, so present = 0 + homing_offset = homing_offset
  EXPECT_EQ(present_pos, homing_offset);
}

TEST_F(MockDynamixelTest, HomingOffsetNegative)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  auto motor = MockDynamixelManager::instance().getMotor(1);
  ASSERT_NE(motor, nullptr);

  // Set negative homing offset
  int32_t homing_offset = -5000;
  motor->setHomingOffset(homing_offset);

  EXPECT_EQ(motor->getHomingOffset(), homing_offset);

  // Simulate one update to write present values
  // Set goal = homing_offset to keep motor at raw position 0
  motor->write1Byte(ADDR_OPERATING_MODE, 3);
  motor->write1Byte(ADDR_TORQUE_ENABLE, 1);
  motor->write4Byte(ADDR_GOAL_POSITION, homing_offset);  // Goal = homing_offset to stay at raw 0
  motor->update(0.001);

  // Present position should be actual (0) + homing_offset (-5000) = -5000
  int32_t present_pos = static_cast<int32_t>(motor->read4Byte(ADDR_PRESENT_POSITION));
  EXPECT_EQ(present_pos, homing_offset);
}

// ============================================================================
// Profile Velocity as Dynamic Limit Tests
// ============================================================================

constexpr uint16_t ADDR_PROFILE_VELOCITY = 560;

TEST_F(MockDynamixelTest, ProfileVelocityLimitsMovementSpeed)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  auto motor = MockDynamixelManager::instance().getMotor(1);
  ASSERT_NE(motor, nullptr);

  // Position mode
  motor->write1Byte(ADDR_OPERATING_MODE, 3);
  motor->write1Byte(ADDR_TORQUE_ENABLE, 1);

  // Set a slow profile velocity
  motor->write4Byte(ADDR_PROFILE_VELOCITY, 1000);  // Slow (~1 rad/s)

  // Set a goal far away (10,000,000 ticks approx 62 rads)
  motor->write4Byte(ADDR_GOAL_POSITION, 10000000);

  // Simulate for short time (2 seconds)
  // At 1 rad/s, should move ~2 rads
  for (int i = 0; i < 200; ++i) {
    motor->update(0.01);
  }
  double slow_position = motor->getCurrentPosition();

  // Reset and try with faster profile velocity
  MockDynamixelManager::instance().reset();
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  motor = MockDynamixelManager::instance().getMotor(1);

  motor->write1Byte(ADDR_OPERATING_MODE, 3);
  motor->write1Byte(ADDR_TORQUE_ENABLE, 1);
  motor->write4Byte(ADDR_PROFILE_VELOCITY, 50000);  // Fast (~50 rad/s)
  motor->write4Byte(ADDR_GOAL_POSITION, 10000000);

  // Simulate for same time
  // At 50 rad/s (accelerating), should move much further
  for (int i = 0; i < 200; ++i) {
    motor->update(0.01);
  }
  double fast_position = motor->getCurrentPosition();

  // Motor with faster profile_velocity should have traveled farther
  EXPECT_GT(fast_position, slow_position);
}

TEST_F(MockDynamixelTest, ProfileVelocityZeroUsesDefault)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  auto motor = MockDynamixelManager::instance().getMotor(1);
  ASSERT_NE(motor, nullptr);

  motor->write1Byte(ADDR_OPERATING_MODE, 3);
  motor->write1Byte(ADDR_TORQUE_ENABLE, 1);

  // Set profile_velocity to 0 (should use default or velocity_limit)
  motor->write4Byte(ADDR_PROFILE_VELOCITY, 0);
  motor->write4Byte(ADDR_VELOCITY_LIMIT, 5000);

  motor->write4Byte(ADDR_GOAL_POSITION, 100000);

  // Should still move (uses velocity_limit as fallback)
  for (int i = 0; i < 1000; ++i) {
    motor->update(0.01);
  }

  EXPECT_GT(motor->getCurrentPosition(), 0.0);
}

// ============================================================================
// Hardware Error Simulation Tests
// ============================================================================

TEST_F(MockDynamixelTest, OverheatingErrorInjection)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  auto motor = MockDynamixelManager::instance().getMotor(1);
  ASSERT_NE(motor, nullptr);

  // No errors initially
  EXPECT_EQ(motor->getHardwareError(), 0);

  // Inject overheating error
  motor->setHardwareError(ERROR_OVERHEATING);

  EXPECT_EQ(motor->getHardwareError(), ERROR_OVERHEATING);
  EXPECT_EQ(motor->read1Byte(ADDR_HARDWARE_ERROR), ERROR_OVERHEATING);

  // Reboot clears error
  motor->clearHardwareError();
  EXPECT_EQ(motor->getHardwareError(), 0);
}

// ============================================================================
// Indirect Addressing Tests
// ============================================================================

// PH Series Indirect Addresses
constexpr uint16_t ADDR_INDIRECT_ADDRESS_START = 168;
constexpr uint16_t ADDR_INDIRECT_DATA_START = 634;

TEST_F(MockDynamixelTest, IndirectAddressingOneByte)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  auto motor = MockDynamixelManager::instance().getMotor(1);
  ASSERT_NE(motor, nullptr);

  // Map Indirect Data 1 (634) to Torque Enable (512)
  // Indirect Address 1 is at 168.
  motor->write2Byte(ADDR_INDIRECT_ADDRESS_START, ADDR_TORQUE_ENABLE);

  // Write to Indirect Data 1
  motor->write1Byte(ADDR_INDIRECT_DATA_START, 1);

  // Check real address
  EXPECT_EQ(motor->read1Byte(ADDR_TORQUE_ENABLE), 1);

  // Read from Indirect Data 1
  EXPECT_EQ(motor->read1Byte(ADDR_INDIRECT_DATA_START), 1);

  // Disable via Indirect Data
  motor->write1Byte(ADDR_INDIRECT_DATA_START, 0);
  EXPECT_EQ(motor->read1Byte(ADDR_TORQUE_ENABLE), 0);
}

TEST_F(MockDynamixelTest, IndirectAddressingMultiByteContiguous)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  auto motor = MockDynamixelManager::instance().getMotor(1);
  ASSERT_NE(motor, nullptr);

  // Map Indirect Data 1..4 (634-637) to Goal Position (564-567)
  // Goal Position is 4 bytes at 564.

  // Set pointers
  for (int i = 0; i < 4; ++i) {
    motor->write2Byte(ADDR_INDIRECT_ADDRESS_START + (i * 2), ADDR_GOAL_POSITION + i);
  }

  // Write 4-byte value to Indirect Data Start
  int32_t target_pos = 123456;
  motor->write4Byte(ADDR_INDIRECT_DATA_START, static_cast<uint32_t>(target_pos));

  // Verify real address received the value
  int32_t real_pos = static_cast<int32_t>(motor->read4Byte(ADDR_GOAL_POSITION));
  EXPECT_EQ(real_pos, target_pos);

  // Verify reading from Indirect Data works
  int32_t indirect_read = static_cast<int32_t>(motor->read4Byte(ADDR_INDIRECT_DATA_START));
  EXPECT_EQ(indirect_read, target_pos);
}

TEST_F(MockDynamixelTest, IndirectAddressingScattered)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  auto motor = MockDynamixelManager::instance().getMotor(1);
  ASSERT_NE(motor, nullptr);

  // Map scattered registers:
  // IndData 1 -> LED Red (513)
  // IndData 2 -> LED Green (514)
  // IndData 3 -> Torque Enable (512)

  motor->write2Byte(ADDR_INDIRECT_ADDRESS_START, 513);      // Addr 168 -> 513
  motor->write2Byte(ADDR_INDIRECT_ADDRESS_START + 2, 514);  // Addr 170 -> 514
  motor->write2Byte(ADDR_INDIRECT_ADDRESS_START + 4, 512);  // Addr 172 -> 512

  // Create a 4-byte packet: [Red=1, Green=1, Torque=1, Dummy=0]
  // Byte 0: 0x01 -> IndData 1 -> Red
  // Byte 1: 0x01 -> IndData 2 -> Green
  // Byte 2: 0x01 -> IndData 3 -> Torque
  // Byte 3: 0x00 -> IndData 4 -> Unmapped (points to 0)

  uint32_t packet = 0x00010101;
  motor->write4Byte(ADDR_INDIRECT_DATA_START, packet);

  // Check results
  EXPECT_EQ(motor->read1Byte(513), 1);  // Red
  EXPECT_EQ(motor->read1Byte(514), 1);  // Green
  EXPECT_EQ(motor->read1Byte(512), 1);  // Torque

  // Check reading scattered data
  // Change Green manually
  motor->write1Byte(514, 0);

  // Read 4 bytes from Indirect Data
  // Should get: [1, 0, 1, (model_lsb)]
  // Model Number of PH is 2020 (0x07E4). LSB is 0xE4 (228).

  uint32_t read_packet = motor->read4Byte(ADDR_INDIRECT_DATA_START);
  uint8_t byte0 = read_packet & 0xFF;
  uint8_t byte1 = (read_packet >> 8) & 0xFF;
  uint8_t byte2 = (read_packet >> 16) & 0xFF;
  uint8_t byte3 = (read_packet >> 24) & 0xFF;

  EXPECT_EQ(byte0, 1);  // Red
  EXPECT_EQ(byte1, 0);  // Green
  EXPECT_EQ(byte2, 1);  // Torque
  // Byte 3 reads from address 0 (Model LSB).
  // Since we wrote 0 to it via the packet, and Mock allows overwriting, it should be 0.
  EXPECT_EQ(byte3, 0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
