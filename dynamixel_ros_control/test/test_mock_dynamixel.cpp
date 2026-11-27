// The mock motor: its kinematic model, its firmware rules and its fault injection

#include <gtest/gtest.h>

#include <dynamixel_ros_control/mock_dynamixel.hpp>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace dynamixel_ros_control {
namespace {

constexpr uint16_t MODEL_PH = 2020;  // PH54-200-S500-R, indirect pointers in EEPROM
constexpr uint16_t MODEL_XM = 1020;  // XM430-W350, indirect pointers in RAM
constexpr uint16_t MODEL_PRO_H54 = 54024;

// The indirect blocks of devices/models/PH.yaml and XM.yaml, which getAddress() does not name
constexpr uint16_t PH_POINTER_1 = 168;
constexpr uint16_t XM_POINTER_1 = 578;
constexpr uint16_t DATA_1 = 634;

class MockDynamixelTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    MockDynamixelManager::instance().reset();
  }

  void TearDown() override
  {
    MockDynamixelManager::instance().reset();
  }

  static std::shared_ptr<MockDynamixel> motor(const uint8_t id = 1, const uint16_t model = MODEL_PH)
  {
    MockDynamixelManager::instance().addMotor(id, model);
    return MockDynamixelManager::instance().getMotor(id);
  }

  /// A motor with torque on in @p mode
  static std::shared_ptr<MockDynamixel> energised(const OperatingMode mode, const uint8_t id = 1,
                                                  const uint16_t model = MODEL_PH)
  {
    auto m = motor(id, model);
    m->write1Byte(m->getAddress("operating_mode"), static_cast<uint8_t>(mode));
    m->write1Byte(m->getAddress("torque_enable"), 1);
    return m;
  }

  static void run(MockDynamixel& m, const int steps, const double dt = 0.01)
  {
    for (int i = 0; i < steps; ++i) {
      m.update(dt);
    }
  }

  MockPortHandler port_{"mock"};
  MockPacketHandler handler_;
  uint8_t error_{0};
};

/// +1 and -1, for the rules that hold in both directions
class MockDynamixelSignTest : public MockDynamixelTest, public ::testing::WithParamInterface<int>
{
};

INSTANTIATE_TEST_SUITE_P(BothDirections, MockDynamixelSignTest, ::testing::Values(1, -1));

// ============================================================================
// Kinematic model
// ============================================================================

TEST_F(MockDynamixelTest, PositionModeMovesTowardTheGoal)
{
  const auto m = energised(OperatingMode::POSITION_CONTROL);
  m->write4Byte(m->getAddress("profile_velocity"), 10000);
  m->write4Byte(m->getAddress("goal_position"), 100000);
  run(*m, 1000);
  EXPECT_GT(m->getCurrentPosition(), 0.0);
}

// A goal latched at the present position of a fast motor, as a NaN hold does. At a 20 ms step the
// braking speed of the last stretch must not carry the motor past the goal.
TEST_F(MockDynamixelTest, PositionModeComesToRestAtAGoalSetWhileMoving)
{
  const auto m = energised(OperatingMode::POSITION_CONTROL);
  m->write4Byte(m->getAddress("goal_position"), 1000000);
  run(*m, 10, 0.02);
  ASSERT_GT(m->getCurrentVelocity(), 1.0);
  m->write4Byte(m->getAddress("goal_position"), m->read4Byte(m->getAddress("present_position")));
  run(*m, 100, 0.02);
  const double rest = m->getCurrentPosition();
  m->update(0.02);
  EXPECT_EQ(m->getCurrentPosition(), rest);
  EXPECT_EQ(m->getCurrentVelocity(), 0.0);
}

TEST_F(MockDynamixelTest, ALimpMotorDoesNotMove)
{
  const auto m = motor();
  EXPECT_EQ(m->read1Byte(m->getAddress("torque_enable")), 0) << "torque is off at boot";
  m->write4Byte(m->getAddress("goal_position"), 100000);
  run(*m, 100);
  EXPECT_NEAR(m->getCurrentPosition(), 0.0, 0.001);
}

TEST_P(MockDynamixelSignTest, VelocityModeTurnsInTheDirectionOfTheGoal)
{
  const auto m = energised(OperatingMode::VELOCITY_CONTROL);
  m->write4Byte(m->getAddress("goal_velocity"), static_cast<uint32_t>(GetParam() * 10000));
  run(*m, 100);
  EXPECT_GT(GetParam() * m->getCurrentPosition(), 0.0);
  EXPECT_GT(GetParam() * m->getCurrentVelocity(), 0.0);
}

TEST_F(MockDynamixelTest, CurrentModeAccelerates)
{
  const auto m = energised(OperatingMode::CURRENT_CONTROL);
  m->write2Byte(m->getAddress("goal_current"), 1000);
  run(*m, 100);
  EXPECT_GT(std::abs(m->getCurrentVelocity()), 0.0);
}

TEST_P(MockDynamixelSignTest, PositionModeStopsAtThePositionLimit)
{
  const auto m = energised(OperatingMode::POSITION_CONTROL);
  m->setFactoryValue("max_position_limit", 1.0);
  m->setFactoryValue("min_position_limit", -1.0);
  m->write4Byte(m->getAddress("profile_velocity"), 10000);
  m->setFactoryValue("goal_position", GetParam() * 3.0);
  run(*m, 1000);
  EXPECT_LE(GetParam() * m->getCurrentPosition(), 1.0 + 1e-3);
}

TEST_F(MockDynamixelTest, ExtendedPositionModeIgnoresThePositionLimits)
{
  const auto m = energised(OperatingMode::EXTENDED_POSITION_CONTROL);
  m->setFactoryValue("max_position_limit", 1.0);
  m->setFactoryValue("min_position_limit", -1.0);
  m->write4Byte(m->getAddress("profile_velocity"), 10000);
  m->setFactoryValue("goal_position", 3.0);
  run(*m, 2000);
  EXPECT_GT(m->getCurrentPosition(), 1.0);
}

TEST_P(MockDynamixelSignTest, HomingOffsetShiftsThePresentPosition)
{
  const auto m = energised(OperatingMode::POSITION_CONTROL);
  const int32_t offset = GetParam() * 10000;
  m->setHomingOffset(offset);
  EXPECT_EQ(m->getHomingOffset(), offset);
  // The goal is in the frame of the present position, so this goal holds the motor at 0
  m->write4Byte(m->getAddress("goal_position"), static_cast<uint32_t>(offset));
  m->update(0.01);
  EXPECT_EQ(static_cast<int32_t>(m->read4Byte(m->getAddress("present_position"))), offset);
}

TEST_F(MockDynamixelTest, AFasterProfileVelocityTravelsFarther)
{
  double travelled[2] = {};
  for (const int i : {0, 1}) {
    const auto m = energised(OperatingMode::POSITION_CONTROL, static_cast<uint8_t>(i + 1));
    m->write4Byte(m->getAddress("profile_velocity"), i == 0 ? 1000 : 50000);
    m->write4Byte(m->getAddress("goal_position"), 10000000);
    run(*m, 200);
    travelled[i] = m->getCurrentPosition();
  }
  EXPECT_GT(travelled[1], travelled[0]);
}

TEST_F(MockDynamixelTest, WithoutAProfileVelocityTheLimitApplies)
{
  const auto m = energised(OperatingMode::POSITION_CONTROL);
  m->write4Byte(m->getAddress("profile_velocity"), 0);
  m->write4Byte(m->getAddress("velocity_limit"), 5000);
  m->write4Byte(m->getAddress("goal_position"), 100000);
  run(*m, 1000);
  EXPECT_GT(m->getCurrentPosition(), 0.0);
}

TEST_F(MockDynamixelTest, TheAccelerationLimitIsInItsRegisterUnit)
{
  const auto m = energised(OperatingMode::VELOCITY_CONTROL);
  m->setFactoryValue("acceleration_limit", 2.0);
  m->setFactoryValue("goal_velocity", 3.0);
  m->update(0.1);
  EXPECT_NEAR(m->getCurrentVelocity(), 0.2, 1e-3) << "2 rad/s^2 for 0.1 s";
}

// The profile in its unit moves the motor inside the limits
TEST_F(MockDynamixelTest, TheProfileMovesTheMotorInsideTheLimits)
{
  const auto m = energised(OperatingMode::EXTENDED_POSITION_CONTROL);
  m->setFactoryValue("acceleration_limit", 2.0);
  m->setFactoryValue("velocity_limit", 1.0);
  // Half the limits, in their unit
  m->write4Byte(m->getAddress("profile_acceleration"), m->read4Byte(m->getAddress("acceleration_limit")) / 2);
  m->write4Byte(m->getAddress("profile_velocity"), m->read4Byte(m->getAddress("velocity_limit")) / 2);
  m->setFactoryValue("goal_position", 100.0);
  m->update(0.1);
  EXPECT_NEAR(m->getCurrentVelocity(), 0.1, 1e-3) << "accelerates at the profile";
  run(*m, 100, 0.1);
  EXPECT_NEAR(m->getCurrentVelocity(), 0.5, 1e-3) << "cruises at the profile";
}

// P series: "Profile Velocity(560) cannot exceed Velocity Limit(44)", the same for the acceleration,
// a write over the limit gets a Data Limit Error
TEST_F(MockDynamixelTest, APSeriesRefusesAProfileAboveItsLimit)
{
  const auto m = motor();
  for (const auto& [profile, limit] : {std::pair<const char*, const char*>{"profile_velocity", "velocity_limit"},
                                       {"profile_acceleration", "acceleration_limit"}}) {
    m->write4Byte(m->getAddress(limit), 100);
    ASSERT_EQ(handler_.write4ByteTxRx(port_, 1, m->getAddress(profile), 100, &error_), COMM_SUCCESS);
    EXPECT_EQ(error_, 0) << profile << " at its limit was refused";
    ASSERT_EQ(handler_.write4ByteTxRx(port_, 1, m->getAddress(profile), 101, &error_), COMM_SUCCESS);
    EXPECT_EQ(error_, PACKET_ERROR_DATA_LIMIT) << profile;
    EXPECT_EQ(m->read4Byte(m->getAddress(profile)), 100U) << profile << " over its limit was applied";

    MockGroupSyncWrite group(nullptr, nullptr, m->getAddress(profile), 4);
    uint8_t value[4] = {101, 0, 0, 0};
    ASSERT_TRUE(group.addParam(1, value));
    EXPECT_EQ(group.txPacket(), COMM_SUCCESS);
    EXPECT_EQ(m->read4Byte(m->getAddress(profile)), 100U) << profile << " over its limit landed by sync write";
  }
}

// X series: Profile Velocity ranges over 0 to 32767, the velocity limit bounds only the goal velocity
TEST_F(MockDynamixelTest, AnXSeriesTakesAProfileVelocityAboveTheVelocityLimit)
{
  const auto m = energised(OperatingMode::EXTENDED_POSITION_CONTROL, 1, MODEL_XM);
  m->setFactoryValue("velocity_limit", 1.0);
  const uint32_t profile = 2 * m->read4Byte(m->getAddress("velocity_limit"));
  ASSERT_EQ(handler_.write4ByteTxRx(port_, 1, m->getAddress("profile_velocity"), profile, &error_), COMM_SUCCESS);
  EXPECT_EQ(error_, 0);
  EXPECT_EQ(m->read4Byte(m->getAddress("profile_velocity")), profile);
  m->setFactoryValue("goal_position", 100.0);
  run(*m, 1000);
  EXPECT_NEAR(m->getCurrentVelocity(), 2.0, 0.03) << "moves at the profile, 0.024 rad/s a tick";
}

// ============================================================================
// Bus watchdog
// ============================================================================

// The firmware stops the motor on a trip: rate and force goals go to zero, the position goal stays
// where the motor stands
TEST_F(MockDynamixelTest, BusWatchdogTripHaltsATurningMotor)
{
  const auto m = energised(OperatingMode::VELOCITY_CONTROL);
  m->write4Byte(m->getAddress("goal_velocity"), 10000);
  run(*m, 100);
  ASSERT_GT(m->getCurrentVelocity(), 0.0);
  m->tripBusWatchdog();
  run(*m, 100);
  EXPECT_NEAR(m->getCurrentVelocity(), 0.0, 1e-3);
}

// The PH manual: the goal registers are read-only while latched, a write gets a Data Range Error
TEST_F(MockDynamixelTest, LatchedBusWatchdogAnswersAnAddressedGoalWriteWithARangeError)
{
  const auto m = motor();
  const uint16_t goal = m->getAddress("goal_position");
  m->write4Byte(goal, 100);
  m->tripBusWatchdog();
  const uint32_t goal_after_trip = m->read4Byte(goal);

  EXPECT_EQ(handler_.write4ByteTxRx(port_, 1, goal, 5000, &error_), COMM_SUCCESS);
  EXPECT_EQ(error_, PACKET_ERROR_DATA_RANGE);
  EXPECT_EQ(m->read4Byte(goal), goal_after_trip) << "the refused goal was applied";

  // Clearing the watchdog register lifts the latch
  EXPECT_EQ(handler_.write1ByteTxRx(port_, 1, m->getAddress("bus_watchdog"), 0, &error_), COMM_SUCCESS);
  EXPECT_EQ(error_, 0);
  EXPECT_EQ(handler_.write4ByteTxRx(port_, 1, goal, 5000, &error_), COMM_SUCCESS);
  EXPECT_EQ(error_, 0);
  EXPECT_EQ(m->read4Byte(goal), 5000U);
}

// Writes always restart the window, reads only if the manager says so
TEST_F(MockDynamixelTest, TimedWatchdogTripsAfterAWindowWithoutInstructions)
{
  MockDynamixelManager::instance().setBusWatchdogTiming(true, false);
  const auto m = motor();
  const uint16_t goal = m->getAddress("goal_position");
  ASSERT_EQ(handler_.write1ByteTxRx(port_, 1, m->getAddress("torque_enable"), 1, &error_), COMM_SUCCESS);
  ASSERT_EQ(handler_.write1ByteTxRx(port_, 1, m->getAddress("bus_watchdog"), 4, &error_), COMM_SUCCESS);  // 80 ms

  for (int i = 0; i < 10; ++i) {
    MockDynamixelManager::instance().update(0.02);
    ASSERT_EQ(handler_.write4ByteTxRx(port_, 1, goal, 100, &error_), COMM_SUCCESS);
  }
  EXPECT_FALSE(m->busWatchdogLatched()) << "a write every cycle feeds it";

  uint32_t value = 0;
  for (int i = 0; i < 4; ++i) {
    MockDynamixelManager::instance().update(0.02);
    ASSERT_EQ(handler_.read4ByteTxRx(port_, 1, goal, &value, &error_), COMM_SUCCESS);
  }
  MockDynamixelManager::instance().update(0.02);
  EXPECT_TRUE(m->busWatchdogLatched()) << "reads fed it although configured not to";
}

TEST_F(MockDynamixelTest, TimedWatchdogCanBeFedByReads)
{
  MockDynamixelManager::instance().setBusWatchdogTiming(true, true);
  const auto m = motor();
  ASSERT_EQ(handler_.write1ByteTxRx(port_, 1, m->getAddress("torque_enable"), 1, &error_), COMM_SUCCESS);
  ASSERT_EQ(handler_.write1ByteTxRx(port_, 1, m->getAddress("bus_watchdog"), 4, &error_), COMM_SUCCESS);

  uint32_t value = 0;
  for (int i = 0; i < 20; ++i) {
    MockDynamixelManager::instance().update(0.02);
    ASSERT_EQ(handler_.read4ByteTxRx(port_, 1, m->getAddress("goal_position"), &value, &error_), COMM_SUCCESS);
  }
  EXPECT_FALSE(m->busWatchdogLatched());
  for (int i = 0; i < 5; ++i) {
    MockDynamixelManager::instance().update(0.02);
  }
  EXPECT_TRUE(m->busWatchdogLatched()) << "silence longer than the window trips it";
}

TEST_F(MockDynamixelTest, TheWatchdogAccessorReadsTheRegister)
{
  const auto m = motor();
  EXPECT_EQ(m->getBusWatchdog(), 0);
  ASSERT_EQ(handler_.write1ByteTxRx(port_, 1, m->getAddress("bus_watchdog"), 4, &error_), COMM_SUCCESS);
  EXPECT_EQ(m->getBusWatchdog(), 4);
  uint8_t value = 0;
  ASSERT_EQ(handler_.read1ByteTxRx(port_, 1, m->getAddress("bus_watchdog"), &value, &error_), COMM_SUCCESS);
  EXPECT_EQ(value, 4);
}

// ============================================================================
// Hardware errors and reboot
// ============================================================================

TEST_F(MockDynamixelTest, AHardwareErrorShowsInItsRegisterUntilCleared)
{
  const auto m = motor();
  const uint16_t status = m->getAddress("hardware_error_status");
  EXPECT_EQ(m->getHardwareError(), 0);
  m->setHardwareError(ERROR_OVERHEATING | ERROR_OVERLOAD);
  EXPECT_EQ(m->getHardwareError(), ERROR_OVERHEATING | ERROR_OVERLOAD);
  EXPECT_EQ(m->read1Byte(status), ERROR_OVERHEATING | ERROR_OVERLOAD);
  m->clearHardwareError();
  EXPECT_EQ(m->getHardwareError(), 0);
}

TEST_F(MockDynamixelTest, ARebootClearsTheHardwareError)
{
  const auto m = motor();
  m->setHardwareError(ERROR_OVERLOAD);
  ASSERT_EQ(handler_.reboot(port_, 1, &error_), COMM_SUCCESS);
  EXPECT_EQ(m->getHardwareError(), 0);
  EXPECT_EQ(m->read1Byte(m->getAddress("hardware_error_status")), 0);
}

// The manual: a shutdown clears Torque Enable, and "REBOOT is the only method to reset Torque
// Enable to 1". Input voltage is not among the default Shutdown bits.
TEST_F(MockDynamixelTest, ShutdownErrorClearsTorqueUntilAReboot)
{
  const auto m = motor();
  const uint16_t torque = m->getAddress("torque_enable");
  m->write1Byte(torque, 1);
  m->setHardwareError(ERROR_INPUT_VOLTAGE);
  EXPECT_EQ(m->read1Byte(torque), 1) << "an error outside the Shutdown bits cut torque";

  m->setHardwareError(ERROR_OVERLOAD);
  EXPECT_EQ(m->read1Byte(torque), 0) << "a shutdown error kept torque";
  EXPECT_EQ(handler_.write1ByteTxRx(port_, 1, torque, 1, &error_), COMM_SUCCESS);
  EXPECT_EQ(m->read1Byte(torque), 0) << "torque came back without a reboot";

  ASSERT_EQ(handler_.reboot(port_, 1, &error_), COMM_SUCCESS);
  EXPECT_EQ(handler_.write1ByteTxRx(port_, 1, torque, 1, &error_), COMM_SUCCESS);
  EXPECT_EQ(m->read1Byte(torque), 1);
}

// A motor that lost only its status path still receives the reboot, while the master sees nothing
TEST_F(MockDynamixelTest, RebootIsAppliedByAMotorThatCannotAnswer)
{
  const auto m = motor();
  m->setCommunicationFault(CommunicationFault::DROP_STATUS_PACKETS);
  EXPECT_EQ(handler_.reboot(port_, 1, &error_), COMM_RX_TIMEOUT);
  EXPECT_EQ(m->getRebootCount(), 1);

  m->setCommunicationFault(CommunicationFault::DISCONNECTED);
  EXPECT_EQ(handler_.reboot(port_, 1, &error_), COMM_RX_TIMEOUT);
  EXPECT_EQ(m->getRebootCount(), 1) << "a disconnected motor received the reboot";
}

struct PointerReboot
{
  uint16_t model;
  uint16_t pointer;
  bool survives;
};

class IndirectPointerRebootTest : public MockDynamixelTest, public ::testing::WithParamInterface<PointerReboot>
{
};

INSTANTIATE_TEST_SUITE_P(RamAndEeprom, IndirectPointerRebootTest,
                         ::testing::Values(PointerReboot{MODEL_XM, XM_POINTER_1, false},
                                           PointerReboot{MODEL_PH, PH_POINTER_1, true}));

// X series pointers live in RAM and a reboot sets them back to their own slot, P series pointers
// live in EEPROM and survive. The interface rewrites only the former.
TEST_P(IndirectPointerRebootTest, ARebootKeepsOnlyEepromPointers)
{
  const auto m = motor(1, GetParam().model);
  const uint16_t torque = m->getAddress("torque_enable");
  m->write2Byte(GetParam().pointer, torque);
  ASSERT_EQ(handler_.reboot(port_, 1, &error_), COMM_SUCCESS);
  EXPECT_EQ(m->read2Byte(GetParam().pointer), GetParam().survives ? torque : DATA_1);
}

// ============================================================================
// Firmware write rules
// ============================================================================

// The status names the refusal: Access Error, a write to EEPROM while torque is enabled
TEST_F(MockDynamixelTest, EepromWriteWhileTorquedIsRefused)
{
  const auto m = motor();
  const uint16_t velocity_limit = m->getAddress("velocity_limit");
  ASSERT_EQ(handler_.write4ByteTxRx(port_, 1, velocity_limit, 1000, &error_), COMM_SUCCESS);
  EXPECT_EQ(error_, 0) << "a limp motor refused an EEPROM write";

  m->write1Byte(m->getAddress("torque_enable"), 1);
  ASSERT_EQ(handler_.write4ByteTxRx(port_, 1, velocity_limit, 2000, &error_), COMM_SUCCESS);
  EXPECT_EQ(error_, PACKET_ERROR_ACCESS);
  EXPECT_EQ(m->read4Byte(velocity_limit), 1000U) << "the refused value was applied";

  ASSERT_EQ(handler_.write4ByteTxRx(port_, 1, m->getAddress("goal_position"), 123, &error_), COMM_SUCCESS);
  EXPECT_EQ(error_, 0) << "a RAM write was refused";
}

// A sync write carries no status, so a motor drops what it refuses an addressed write
TEST_F(MockDynamixelTest, ASyncWriteToEepromOfATorquedMotorIsDropped)
{
  const auto m = motor();
  const uint16_t velocity_limit = m->getAddress("velocity_limit");
  m->write4Byte(velocity_limit, 1000);
  m->write1Byte(m->getAddress("torque_enable"), 1);
  MockGroupSyncWrite group(nullptr, nullptr, velocity_limit, 4);
  uint8_t value[4] = {0xD0, 0x07, 0, 0};
  ASSERT_TRUE(group.addParam(1, value));
  EXPECT_EQ(group.txPacket(), COMM_SUCCESS);
  EXPECT_EQ(m->read4Byte(velocity_limit), 1000U);
}

// Indirect pointers may only address the RAM area. A group over an EEPROM register would work
// against a mock that stores any pointer and fail on_configure on the robot.
TEST_F(MockDynamixelTest, AnIndirectPointerIsBoundedByTheRamArea)
{
  const auto m = motor();
  ASSERT_EQ(handler_.write2ByteTxRx(port_, 1, PH_POINTER_1, m->getAddress("operating_mode"), &error_), COMM_SUCCESS);
  EXPECT_EQ(error_, PACKET_ERROR_DATA_RANGE) << "a pointer into EEPROM was accepted";
  EXPECT_EQ(m->read2Byte(PH_POINTER_1), DATA_1);

  // The area, not register membership: 520 is in a gap between listed PH registers
  ASSERT_EQ(handler_.write2ByteTxRx(port_, 1, PH_POINTER_1, 520, &error_), COMM_SUCCESS);
  EXPECT_EQ(error_, 0) << "a pointer the motor accepts was refused";

  ASSERT_EQ(handler_.write2ByteTxRx(port_, 1, PH_POINTER_1, m->getAddress("torque_enable"), &error_), COMM_SUCCESS);
  EXPECT_EQ(error_, 0);
  m->write1Byte(DATA_1, 1);
  EXPECT_EQ(m->read1Byte(m->getAddress("torque_enable")), 1) << "the accepted pointer does not alias its register";
}

TEST_F(MockDynamixelTest, ASyncWriteOfAnIndirectPointerOutsideRamIsDropped)
{
  const auto m = motor();
  MockGroupSyncWrite group(nullptr, nullptr, PH_POINTER_1, 2);
  uint8_t pointer[2] = {static_cast<uint8_t>(m->getAddress("operating_mode")), 0};
  ASSERT_TRUE(group.addParam(1, pointer));
  EXPECT_EQ(group.txPacket(), COMM_SUCCESS);
  EXPECT_EQ(m->read2Byte(PH_POINTER_1), DATA_1);
}

TEST_F(MockDynamixelTest, IndirectSlotsReachTheirRegisters)
{
  const auto m = motor();
  const uint16_t torque = m->getAddress("torque_enable");
  const uint16_t goal = m->getAddress("goal_position");
  m->write2Byte(PH_POINTER_1, torque);
  m->write1Byte(DATA_1, 1);
  EXPECT_EQ(m->read1Byte(torque), 1);
  EXPECT_EQ(m->read1Byte(DATA_1), 1);
  m->write1Byte(DATA_1, 0);
  EXPECT_EQ(m->read1Byte(torque), 0);

  // Four slots over a four byte register
  for (uint16_t i = 0; i < 4; ++i) {
    m->write2Byte(static_cast<uint16_t>(PH_POINTER_1 + 2 * i), static_cast<uint16_t>(goal + i));
  }
  m->write4Byte(DATA_1, 123456);
  EXPECT_EQ(m->read4Byte(goal), 123456U);
  EXPECT_EQ(m->read4Byte(DATA_1), 123456U);
}

TEST_F(MockDynamixelTest, ScatteredIndirectSlotsReachTheirRegisters)
{
  const auto m = motor();
  const uint16_t red = m->getAddress("led_red");
  const uint16_t green = m->getAddress("led_green");
  const uint16_t torque = m->getAddress("torque_enable");
  m->write2Byte(PH_POINTER_1, red);
  m->write2Byte(PH_POINTER_1 + 2, green);
  m->write2Byte(PH_POINTER_1 + 4, torque);
  // The fourth slot is unmapped and holds its own byte
  m->write4Byte(DATA_1, 0x2A010101);
  EXPECT_EQ(m->read1Byte(red), 1);
  EXPECT_EQ(m->read1Byte(green), 1);
  EXPECT_EQ(m->read1Byte(torque), 1);

  m->write1Byte(green, 0);
  EXPECT_EQ(m->read4Byte(DATA_1), 0x2A010001U);
  EXPECT_EQ(m->read2Byte(m->getAddress("model_number")), MODEL_PH) << "the unmapped slot wrote the model number";
}

// ============================================================================
// Fault injection
// ============================================================================

TEST_F(MockDynamixelTest, ADisconnectedMotorNeitherAnswersNorApplies)
{
  const auto m = motor();
  const uint16_t torque = m->getAddress("torque_enable");
  m->setCommunicationFault(CommunicationFault::DISCONNECTED);
  uint16_t model_number = 0;
  uint8_t value = 0;
  EXPECT_EQ(handler_.ping(port_, 1, &model_number, &error_), COMM_RX_TIMEOUT);
  EXPECT_EQ(handler_.read1ByteTxRx(port_, 1, torque, &value, &error_), COMM_RX_TIMEOUT);
  EXPECT_EQ(handler_.write1ByteTxRx(port_, 1, torque, 1, &error_), COMM_RX_TIMEOUT);
  EXPECT_EQ(m->read1Byte(torque), 0);

  m->setCommunicationFault(CommunicationFault::NONE);
  EXPECT_EQ(handler_.ping(port_, 1, &model_number, &error_), COMM_SUCCESS) << "the motor did not recover";
  EXPECT_EQ(model_number, MODEL_PH);
}

TEST_F(MockDynamixelTest, AGlobalCommunicationErrorSilencesEveryMotor)
{
  motor(1);
  motor(2);
  uint16_t model_number = 0;
  MockDynamixelManager::instance().setGlobalCommunicationError(true);
  EXPECT_EQ(handler_.ping(port_, 1, &model_number, &error_), COMM_RX_TIMEOUT);
  EXPECT_EQ(handler_.ping(port_, 2, &model_number, &error_), COMM_RX_TIMEOUT);
  MockDynamixelManager::instance().setGlobalCommunicationError(false);
  EXPECT_EQ(handler_.ping(port_, 1, &model_number, &error_), COMM_SUCCESS);
  EXPECT_EQ(handler_.ping(port_, 2, &model_number, &error_), COMM_SUCCESS);
}

// A counted fault heals, so it tells a caller that retries from one that gives up on the first miss
TEST_F(MockDynamixelTest, ACountedReadFaultHealsAfterItsCount)
{
  const auto m = motor();
  const uint16_t torque = m->getAddress("torque_enable");
  m->write1Byte(torque, 1);
  m->setRegisterReadFaultCount("torque_enable", 2);
  uint8_t value = 0;
  EXPECT_EQ(handler_.read1ByteTxRx(port_, 1, torque, &value, &error_), COMM_RX_TIMEOUT);
  EXPECT_EQ(handler_.read1ByteTxRx(port_, 1, torque, &value, &error_), COMM_RX_TIMEOUT);
  EXPECT_EQ(handler_.read1ByteTxRx(port_, 1, torque, &value, &error_), COMM_SUCCESS) << "the fault was not spent";
  EXPECT_EQ(value, 1);
}

TEST_F(MockDynamixelTest, APersistentReadFaultNeverHeals)
{
  const auto m = motor();
  m->setRegisterReadFault("torque_enable");
  uint8_t value = 0;
  for (int attempt = 0; attempt < 5; ++attempt) {
    EXPECT_EQ(handler_.read1ByteTxRx(port_, 1, m->getAddress("torque_enable"), &value, &error_), COMM_RX_TIMEOUT)
        << "attempt " << attempt;
  }
}

// One read spends one use of every byte, or the remaining bytes stay faulted forever
TEST_F(MockDynamixelTest, ACountedReadFaultOnAMultiByteRegisterHealsAsAWhole)
{
  const auto m = motor();
  m->setRegisterReadFaultCount("goal_position", 1);
  uint32_t value = 0;
  EXPECT_EQ(handler_.read4ByteTxRx(port_, 1, m->getAddress("goal_position"), &value, &error_), COMM_RX_TIMEOUT);
  EXPECT_EQ(handler_.read4ByteTxRx(port_, 1, m->getAddress("goal_position"), &value, &error_), COMM_SUCCESS);
}

// A motor resolves an addressed read of an indirect slot as it does a sync read
TEST_F(MockDynamixelTest, AnAddressedReadOfAnIndirectSlotFailsWithItsRegister)
{
  const auto m = motor();
  m->write2Byte(PH_POINTER_1, m->getAddress("torque_enable"));
  m->setRegisterReadFault("torque_enable");
  uint8_t value = 0;
  EXPECT_EQ(handler_.read1ByteTxRx(port_, 1, DATA_1, &value, &error_), COMM_RX_TIMEOUT);
}

// A hook reports a read that happened, so it must not fire for a motor that did not answer
TEST_F(MockDynamixelTest, AReadHookDoesNotFireForAMotorThatDidNotAnswer)
{
  const auto m = motor();
  m->write2Byte(PH_POINTER_1, m->getAddress("torque_enable"));
  int fired = 0;
  m->setRegisterReadHook("torque_enable", [&fired]() { ++fired; });
  MockGroupSyncRead group(nullptr, nullptr, DATA_1, 1);
  ASSERT_TRUE(group.addParam(1));

  m->setCommunicationFault(CommunicationFault::DISCONNECTED);
  EXPECT_NE(group.txRxPacket(), COMM_SUCCESS);
  EXPECT_EQ(fired, 0) << "fired for a silent motor";

  m->setCommunicationFault(CommunicationFault::NONE);
  m->setRegisterReadFault("torque_enable");
  EXPECT_NE(group.txRxPacket(), COMM_SUCCESS);
  EXPECT_EQ(fired, 0) << "fired for a faulted register";
}

// The error field carries an error number and an alert bit, which occur together
TEST_F(MockDynamixelTest, PacketErrorStringKeepsTheErrorNumberAndTheAlertApart)
{
  EXPECT_STREQ(handler_.getRxPacketError(0), "");
  EXPECT_STREQ(handler_.getRxPacketError(PACKET_ERROR_ALERT), "Hardware error alert");
  EXPECT_STREQ(handler_.getRxPacketError(PACKET_ERROR_DATA_RANGE), "Data Range Error");
  EXPECT_STREQ(handler_.getRxPacketError(PACKET_ERROR_DATA_RANGE | PACKET_ERROR_ALERT),
               "Data Range Error (with hardware error alert)");
}

// ============================================================================
// Sync groups
// ============================================================================

TEST_F(MockDynamixelTest, ASyncReadAnswersForEveryMotor)
{
  for (const uint8_t id : {1, 2, 3}) {
    const auto m = motor(id);
    m->write4Byte(m->getAddress("goal_position"), id * 1000U);
  }
  const uint16_t goal = MockDynamixelManager::instance().getMotor(1)->getAddress("goal_position");
  MockGroupSyncRead group(nullptr, nullptr, goal, 4);
  for (const uint8_t id : {1, 2, 3}) {
    ASSERT_TRUE(group.addParam(id));
  }
  ASSERT_EQ(group.txRxPacket(), COMM_SUCCESS);
  for (const uint8_t id : {1, 2, 3}) {
    EXPECT_TRUE(group.isAvailable(id, goal, 4));
    EXPECT_EQ(group.getData(id, goal, 4), id * 1000U);
  }
}

// As the SDK group, which aborts at the first missing status packet
TEST_F(MockDynamixelTest, ASyncReadFailsWholeWhenOneMotorStaysSilent)
{
  for (const uint8_t id : {1, 2}) {
    motor(id);
  }
  const uint16_t present = MockDynamixelManager::instance().getMotor(1)->getAddress("present_position");
  MockDynamixelManager::instance().getMotor(2)->setCommunicationFault(CommunicationFault::DROP_STATUS_PACKETS);
  MockGroupSyncRead group(nullptr, nullptr, present, 4);
  ASSERT_TRUE(group.addParam(1));
  ASSERT_TRUE(group.addParam(2));
  EXPECT_NE(group.txRxPacket(), COMM_SUCCESS);
  EXPECT_FALSE(group.isAvailable(1, present, 4)) << "data of a partial read was kept";
  EXPECT_FALSE(group.isAvailable(2, present, 4));
}

// getData must not sign-extend a value with the top bit set
TEST_F(MockDynamixelTest, ASyncReadKeepsTheTopBitOfAFourByteValue)
{
  const auto m = motor();
  const uint16_t present = m->getAddress("present_position");
  m->write4Byte(present, 0xFF000000U);
  MockGroupSyncRead group(nullptr, nullptr, present, 4);
  ASSERT_TRUE(group.addParam(1));
  ASSERT_EQ(group.txRxPacket(), COMM_SUCCESS);
  EXPECT_EQ(group.getData(1, present, 4), 0xFF000000U);
}

TEST_F(MockDynamixelTest, ASyncWriteReachesEveryMotor)
{
  motor(1);
  motor(2);
  const uint16_t torque = MockDynamixelManager::instance().getMotor(1)->getAddress("torque_enable");
  MockGroupSyncWrite group(nullptr, nullptr, torque, 1);
  uint8_t on = 1;
  ASSERT_TRUE(group.addParam(1, &on));
  ASSERT_TRUE(group.addParam(2, &on));
  EXPECT_EQ(group.txPacket(), COMM_SUCCESS);
  EXPECT_EQ(MockDynamixelManager::instance().getMotor(1)->read1Byte(torque), 1);
  EXPECT_EQ(MockDynamixelManager::instance().getMotor(2)->read1Byte(torque), 1);
}

// A sync write is transmit only, so a motor that cannot answer still applies it
TEST_F(MockDynamixelTest, ASyncWriteReachesAMotorThatCannotAnswerButNotADisconnectedOne)
{
  const auto m = motor();
  const uint16_t torque = m->getAddress("torque_enable");
  MockGroupSyncWrite group(nullptr, nullptr, torque, 1);
  uint8_t off = 0;
  ASSERT_TRUE(group.addParam(1, &off));

  m->write1Byte(torque, 1);
  m->setCommunicationFault(CommunicationFault::DROP_STATUS_PACKETS);
  EXPECT_EQ(group.txPacket(), COMM_SUCCESS);
  EXPECT_EQ(m->read1Byte(torque), 0);

  m->write1Byte(torque, 1);
  m->setCommunicationFault(CommunicationFault::DISCONNECTED);
  EXPECT_EQ(group.txPacket(), COMM_SUCCESS);
  EXPECT_EQ(m->read1Byte(torque), 1);
}

// ============================================================================
// Manager and accessors
// ============================================================================

TEST_F(MockDynamixelTest, TheManagerAddsRemovesAndResetsMotors)
{
  for (const uint8_t id : {5, 10, 15}) {
    motor(id);
  }
  EXPECT_EQ(MockDynamixelManager::instance().getConnectedIds(), (std::vector<uint8_t>{5, 10, 15}));
  MockDynamixelManager::instance().removeMotor(5);
  EXPECT_EQ(MockDynamixelManager::instance().getMotor(5), nullptr);
  EXPECT_NE(MockDynamixelManager::instance().getMotor(10), nullptr);
  MockDynamixelManager::instance().reset();
  EXPECT_TRUE(MockDynamixelManager::instance().getConnectedIds().empty());
}

TEST_F(MockDynamixelTest, AMotorKnowsItsIdentity)
{
  const auto m = motor(42);
  EXPECT_EQ(m->getId(), 42);
  EXPECT_EQ(m->getModelNumber(), MODEL_PH);
  EXPECT_EQ(m->read1Byte(m->getAddress("id")), 42);
  EXPECT_EQ(m->read2Byte(m->getAddress("model_number")), MODEL_PH);
  EXPECT_EQ(m->getAddress("nonexistent_register"), 0) << "an unknown register";
}

// A motor without its control table would answer pings and ignore every register
TEST_F(MockDynamixelTest, AModelWithoutAControlTableIsRefused)
{
  EXPECT_THROW(MockDynamixelManager::instance().addMotor(1, 7), std::runtime_error) << "not in model_list.yaml";
  EXPECT_THROW(MockDynamixelManager::instance().addMotor(1, 12), std::runtime_error) << "AX, no table in devices/";
  EXPECT_EQ(MockDynamixelManager::instance().getMotor(1), nullptr);
}

TEST_F(MockDynamixelTest, TheLedAccessorsReadAndWriteTheRegisters)
{
  const auto m = motor();
  EXPECT_EQ(m->getLedRed(), 0);
  EXPECT_EQ(m->getLedGreen(), 0);
  EXPECT_EQ(m->getLedBlue(), 0);
  m->setLedRed(255);
  m->setLedGreen(128);
  m->setLedBlue(64);
  EXPECT_EQ(m->read1Byte(m->getAddress("led_red")), 255);
  EXPECT_EQ(m->read1Byte(m->getAddress("led_green")), 128);
  EXPECT_EQ(m->read1Byte(m->getAddress("led_blue")), 64);
  m->write1Byte(m->getAddress("led_red"), 100);
  m->write1Byte(m->getAddress("led_green"), 150);
  m->write1Byte(m->getAddress("led_blue"), 200);
  EXPECT_EQ(m->getLedRed(), 100);
  EXPECT_EQ(m->getLedGreen(), 150);
  EXPECT_EQ(m->getLedBlue(), 200);
}

// ============================================================================
// Plant mode: the motor drives a joint of a physics engine
// ============================================================================

TEST_F(MockDynamixelTest, LimpServoReferenceFollowsThePlantAndTorqueOnStartsThere)
{
  const auto m = motor();
  m->write1Byte(m->getAddress("operating_mode"), static_cast<uint8_t>(OperatingMode::EXTENDED_POSITION_CONTROL));
  auto command = m->followPlant(0.01, 1.0, 0.5, 0.0);
  EXPECT_FALSE(command.torque_on);
  EXPECT_NEAR(command.position, 1.0, 1e-9);
  EXPECT_NEAR(command.velocity, 0.5, 1e-9);

  // The plugin anchors the goal at the present position before it enables torque
  m->followPlant(0.01, 1.0, 0.0, 0.0);
  m->write4Byte(m->getAddress("goal_position"), m->read4Byte(m->getAddress("present_position")));
  m->write1Byte(m->getAddress("torque_enable"), 1);
  command = m->followPlant(0.01, 1.0, 0.0, 0.0);
  EXPECT_TRUE(command.torque_on);
  EXPECT_NEAR(command.position, 1.0, 1e-3);
}

// The plant hands over what the register reports, homing included, and the reference comes back in
// that frame, so a torque on does not turn arm_joint_1 by its homing offset of pi
TEST_F(MockDynamixelTest, PlantPositionIsInTheFrameOfThePresentPositionRegister)
{
  const auto m = motor();
  m->setFactoryValue("homing_offset", M_PI);
  m->write1Byte(m->getAddress("operating_mode"), static_cast<uint8_t>(OperatingMode::EXTENDED_POSITION_CONTROL));
  m->followPlant(0.01, 2.0, 0.0, 0.0);
  m->setFactoryValue("goal_position", 2.0);
  EXPECT_NEAR(static_cast<int32_t>(m->read4Byte(m->getAddress("present_position"))),
              static_cast<int32_t>(m->read4Byte(m->getAddress("goal_position"))), 1);
  EXPECT_NEAR(m->getCurrentPosition(), 2.0 - M_PI, 1e-4);

  m->write4Byte(m->getAddress("goal_position"), m->read4Byte(m->getAddress("present_position")));
  m->write1Byte(m->getAddress("torque_enable"), 1);
  EXPECT_NEAR(m->followPlant(0.01, 2.0, 0.0, 0.0).position, 2.0, 1e-4);
}

// A blocked or current limited joint must not hold back the profile, or the servo stops pulling
TEST_F(MockDynamixelTest, PositionModeReferenceAdvancesTowardTheGoalWhileThePlantStands)
{
  const auto m = energised(OperatingMode::EXTENDED_POSITION_CONTROL);
  m->setFactoryValue("goal_position", 1.0);
  ServoCommand command;
  for (int i = 0; i < 10; ++i) {
    command = m->followPlant(0.01, 0.0, 0.0, 0.0);
  }
  EXPECT_GT(command.position, 0.05);
  EXPECT_LT(command.position, 1.0);
  EXPECT_GT(command.velocity, 0.0);
}

TEST_F(MockDynamixelTest, PlantMeasurementReachesThePresentRegistersInModelUnits)
{
  const auto m = motor(1, MODEL_PRO_H54);
  m->followPlant(0.01, 1.0, 0.5, 2.0);
  // The per tick values of devices/models/PROExt.yaml
  EXPECT_NEAR(static_cast<int32_t>(m->read4Byte(m->getAddress("present_position"))), 1.0 / 0.00001251822, 1);
  EXPECT_NEAR(static_cast<int32_t>(m->read4Byte(m->getAddress("present_velocity"))), 0.5 / 0.00020863735, 1);
  EXPECT_NEAR(static_cast<int16_t>(m->read2Byte(m->getAddress("present_current"))), 2.0 / 0.01611328125, 1);
}

TEST_F(MockDynamixelTest, VelocityModeReferenceDoesNotWindUpOnABlockedJoint)
{
  const auto m = energised(OperatingMode::VELOCITY_CONTROL);
  m->setFactoryValue("goal_velocity", 1.0);
  ServoCommand command;
  for (int i = 0; i < 100; ++i) {
    command = m->followPlant(0.01, 0.0, 0.0, 0.0);
  }
  EXPECT_NEAR(command.velocity, 1.0, 1e-3);
  EXPECT_NEAR(command.position, 0.0, 1e-9);
}

TEST_F(MockDynamixelTest, PlantCurrentLimitIsTheLowerOfCurrentLimitAndGoalCurrent)
{
  const auto m = motor();
  m->setFactoryValue("current_limit", 2.0);
  m->setFactoryValue("goal_current", -0.5);
  EXPECT_NEAR(m->followPlant(0.01, 0.0, 0.0, 0.0).current_limit, 0.5, 1e-9);
  m->setFactoryValue("goal_current", 3.0);
  EXPECT_NEAR(m->followPlant(0.01, 0.0, 0.0, 0.0).current_limit, 2.0, 1e-9);
}

TEST_F(MockDynamixelTest, OnlyAProGoalTorqueOfZeroLeavesTheTorqueLimitAsTheCap)
{
  // Goal Current is RAM, so it boots at 0
  const auto pro = motor(1, MODEL_PRO_H54);
  pro->setFactoryValue("current_limit", 9.99);
  EXPECT_NEAR(pro->followPlant(0.01, 0.0, 0.0, 0.0).current_limit, 9.99, 0.02);
  pro->setFactoryValue("goal_current", 1.0);
  EXPECT_NEAR(pro->followPlant(0.01, 0.0, 0.0, 0.0).current_limit, 1.0, 0.02);
  const auto ph = motor(2);
  ph->setFactoryValue("current_limit", 2.0);
  EXPECT_NEAR(ph->followPlant(0.01, 0.0, 0.0, 0.0).current_limit, 0.0, 1e-9);
}

TEST_F(MockDynamixelTest, FactoryValuesAreWrittenInTheRegisterUnit)
{
  const auto m = motor(1, MODEL_PRO_H54);
  // Velocity first, so a current limit written wider than its register would show
  m->setFactoryValue("velocity_limit", 3.0);
  m->setFactoryValue("current_limit", 22.0);
  EXPECT_EQ(m->read2Byte(m->getAddress("current_limit")), std::lround(22.0 / 0.01611328125));
  EXPECT_EQ(m->read4Byte(m->getAddress("velocity_limit")), std::lround(3.0 / 0.00020863735));
  EXPECT_THROW(m->setFactoryValue("no_such_register", 1.0), std::invalid_argument);
}

}  // namespace
}  // namespace dynamixel_ros_control
