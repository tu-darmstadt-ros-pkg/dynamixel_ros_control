// DynamixelDriver, Dynamixel and Joint on the simulated bus: register access, value encoding,
// initial values and the bus watchdog clear

#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <thread>
#include <utility>

#include <dynamixel_ros_control/common.hpp>
#include <dynamixel_ros_control/joint.hpp>

#include "driver_fixture.hpp"

namespace dynamixel_ros_control::test {
namespace {

constexpr uint16_t MODEL_RH = 35073;  // has no bus_watchdog register

TEST_F(DriverTest, APingFindsTheMotorAndItsModel)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  uint16_t model_number = 0;
  int comm_result = COMM_TX_FAIL;
  EXPECT_TRUE(driver_.ping(1, model_number, comm_result));
  EXPECT_EQ(model_number, MODEL_PH);
  EXPECT_EQ(comm_result, COMM_SUCCESS);
  EXPECT_FALSE(driver_.ping(99, model_number, comm_result)) << "a motor that is not on the bus answered";
  EXPECT_EQ(comm_result, COMM_RX_TIMEOUT);
}

TEST_F(DriverTest, WritesAndReadsRegistersOfEveryWidth)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  const auto m = motor();
  int32_t value = 0;
  EXPECT_TRUE(driver_.writeRegister(1, m->getAddress("torque_enable"), 1, 1));
  EXPECT_TRUE(driver_.readRegister(1, m->getAddress("torque_enable"), 1, value, false));
  EXPECT_EQ(value, 1);
  EXPECT_TRUE(driver_.writeRegister(1, m->getAddress("goal_position"), 4, 100000));
  EXPECT_TRUE(driver_.readRegister(1, m->getAddress("goal_position"), 4, value, true));
  EXPECT_EQ(value, 100000);
}

TEST_F(DriverTest, AnUnansweredTransactionFails)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  motor()->setCommunicationFault(CommunicationFault::DISCONNECTED);
  int32_t value = 0;
  EXPECT_FALSE(driver_.readRegister(1, motor()->getAddress("torque_enable"), 1, value, false));
  EXPECT_FALSE(driver_.writeRegister(1, motor()->getAddress("torque_enable"), 1, 1));
}

// The longest gap between two transactions is what a bus watchdog sees
TEST_F(DriverTest, TheDriverReportsTheLongestSilenceOnTheBus)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  uint16_t model_number = 0;
  int comm_result = COMM_TX_FAIL;
  ASSERT_TRUE(driver_.ping(1, model_number, comm_result));
  (void) driver_.takeLongestSilenceMs();

  ASSERT_TRUE(driver_.ping(1, model_number, comm_result));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  ASSERT_TRUE(driver_.ping(1, model_number, comm_result));
  ASSERT_TRUE(driver_.ping(1, model_number, comm_result));
  const double longest = driver_.takeLongestSilenceMs();
  EXPECT_GE(longest, 45.0);
  EXPECT_LT(longest, 1000.0);
  EXPECT_LT(driver_.takeLongestSilenceMs(), 1.0) << "taking the value did not start a new measurement";
}

// ============================================================================
// Values that do not fit their register
// ============================================================================

// The driver sends the low bytes, so a value too wide for the register would arrive reduced modulo
// its width and be accepted: a temperature limit of 300 becomes 44
TEST_F(DriverTest, AValueTooWideForTheRegisterIsRefused)
{
  Dynamixel& dxl = connected();
  const uint16_t limit = motor()->getAddress("temperature_limit");
  ASSERT_TRUE(dxl.writeRegister("temperature_limit", static_cast<int32_t>(80)));
  EXPECT_FALSE(dxl.writeRegister("temperature_limit", static_cast<int32_t>(300)));
  EXPECT_FALSE(dxl.writeRegister("temperature_limit", static_cast<int32_t>(-200)));
  EXPECT_EQ(motor()->read1Byte(limit), 80);

  // The register's own span: it is unsigned, so -1 would arrive as 255
  EXPECT_TRUE(dxl.writeRegister("temperature_limit", static_cast<int32_t>(255)));
  EXPECT_TRUE(dxl.writeRegister("temperature_limit", static_cast<int32_t>(0)));
  EXPECT_FALSE(dxl.writeRegister("temperature_limit", static_cast<int32_t>(-1)));
  EXPECT_FALSE(dxl.writeRegister("temperature_limit", static_cast<int32_t>(-128)));
}

// A signed register loses the top half of the unsigned range: 65535 would arrive as -1
TEST_F(DriverTest, ASignedRegisterKeepsItsNegativesAndRefusesTheUnsignedHalf)
{
  Dynamixel& dxl = connected();
  ASSERT_TRUE(dxl.getItem(DXL_REGISTER_CMD_EFFORT).is_signed());
  EXPECT_TRUE(dxl.writeRegister(DXL_REGISTER_CMD_EFFORT, static_cast<int32_t>(-32768)));
  EXPECT_TRUE(dxl.writeRegister(DXL_REGISTER_CMD_EFFORT, static_cast<int32_t>(32767)));
  EXPECT_FALSE(dxl.writeRegister(DXL_REGISTER_CMD_EFFORT, static_cast<int32_t>(65535)));
  EXPECT_FALSE(dxl.writeRegister(DXL_REGISTER_CMD_EFFORT, static_cast<int32_t>(32768)));
}

// Clamped into INT32_MAX ticks, an impossible position would become the far end of the travel
TEST_F(DriverTest, AFiniteValueWithNoRawEquivalentIsRefused)
{
  Dynamixel& dxl = connected();
  const uint16_t goal = motor()->getAddress(DXL_REGISTER_CMD_POSITION);
  ASSERT_TRUE(dxl.writeRegister(DXL_REGISTER_CMD_POSITION, 0.5));
  const uint32_t before = motor()->read4Byte(goal);
  EXPECT_FALSE(dxl.writeRegister(DXL_REGISTER_CMD_POSITION, 1e30));
  EXPECT_FALSE(dxl.writeRegister(DXL_REGISTER_CMD_POSITION, -1e30));
  EXPECT_EQ(motor()->read4Byte(goal), before);
}

TEST_F(DriverTest, ANonFiniteValueIsRefused)
{
  Dynamixel& dxl = connected();
  const uint16_t goal = motor()->getAddress(DXL_REGISTER_CMD_POSITION);
  ASSERT_TRUE(dxl.writeRegister(DXL_REGISTER_CMD_POSITION, 0.5));
  const uint32_t before = motor()->read4Byte(goal);
  ASSERT_NE(before, 0U) << "test premise: a silent zero would change the register";
  for (const double bad : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity()}) {
    EXPECT_FALSE(dxl.writeRegister(DXL_REGISTER_CMD_POSITION, bad)) << bad;
    EXPECT_EQ(motor()->read4Byte(goal), before) << bad;
  }
}

// false is also a legitimate reading, so a non bool register read as bool has to fail
TEST_F(DriverTest, ReadingANonBoolRegisterAsBoolFails)
{
  Dynamixel& dxl = connected();
  bool value = false;
  EXPECT_TRUE(dxl.readRegister(DXL_REGISTER_CMD_TORQUE, value));
  EXPECT_FALSE(dxl.readRegister("temperature_limit", value));
}

// A raw 254 in an unsigned byte is 254, which the manifest and a state interface carry
TEST_F(DriverTest, UnsignedRegistersDecodeUnsigned)
{
  Dynamixel& dxl = connected();
  for (const auto& [name, raw] :
       {std::pair<const char*, uint16_t>{"return_delay_time", 254}, {"led_blue", 255}, {"position_p_gain", 40000}}) {
    if (motor()->getLength(name) == 1) {
      motor()->write1Byte(motor()->getAddress(name), static_cast<uint8_t>(raw));
    } else {
      motor()->write2Byte(motor()->getAddress(name), raw);
    }
    int32_t value = 0;
    ASSERT_TRUE(dxl.readRegister(name, value));
    EXPECT_EQ(value, raw) << name;
  }
}

TEST_F(DriverTest, SignedRegistersStillDecodeSigned)
{
  Dynamixel& dxl = connected();
  motor()->write2Byte(motor()->getAddress(DXL_REGISTER_EFFORT), static_cast<uint16_t>(-25));
  int32_t value = 0;
  ASSERT_TRUE(dxl.readRegister(DXL_REGISTER_EFFORT, value));
  EXPECT_EQ(value, -25);
  // clearBusWatchdog() reads a trip as -1
  motor()->write1Byte(motor()->getAddress(DXL_REGISTER_BUS_WATCHDOG), static_cast<uint8_t>(-1));
  ASSERT_TRUE(dxl.readRegister(DXL_REGISTER_BUS_WATCHDOG, value));
  EXPECT_EQ(value, -1);
}

// ============================================================================
// Read-only registers and initial values
// ============================================================================

// The motor would answer with an instruction error, so the message names the mistake instead
TEST_F(DriverTest, AWriteToAReadOnlyRegisterIsRefused)
{
  Dynamixel& dxl = connected();
  ASSERT_FALSE(dxl.getItem(DXL_REGISTER_POSITION).writable());
  EXPECT_FALSE(dxl.writeRegister(DXL_REGISTER_POSITION, static_cast<int32_t>(1234)));
  EXPECT_TRUE(dxl.writeRegister(DXL_REGISTER_CMD_POSITION, static_cast<int32_t>(1234)));
}

// The motor refuses with an Access Error, which the write reports
TEST_F(DriverTest, AnEepromWriteToATorquedMotorFails)
{
  Dynamixel& dxl = connected();
  EXPECT_TRUE(dxl.writeRegister("velocity_limit", static_cast<int32_t>(1000)));
  ASSERT_TRUE(dxl.writeRegister(DXL_REGISTER_CMD_TORQUE, true));
  EXPECT_FALSE(dxl.writeRegister("velocity_limit", static_cast<int32_t>(2000)));
  int32_t value = 0;
  ASSERT_TRUE(dxl.readRegister("velocity_limit", value));
  EXPECT_EQ(value, 1000);
}

// A register the model lacks is skipped, the others still land
TEST_F(DriverTest, InitialValuesSkipAnUnavailableRegister)
{
  Dynamixel& dxl = connected();
  dxl.setInitialRegisterValues({{"led_blue", "200"}, {"this_register_does_not_exist_on_any_model", "42"}});
  EXPECT_TRUE(dxl.writeInitialValues());
  EXPECT_EQ(motor()->read1Byte(motor()->getAddress("led_blue")), 200);
}

// The URDF names registers as strings, so an initial value can name a telemetry register
TEST_F(DriverTest, AnInitialValueForAReadOnlyRegisterFailsConfiguration)
{
  Dynamixel& dxl = connected();
  dxl.setInitialRegisterValues({{std::string(DXL_REGISTER_POSITION), "1234"}});
  EXPECT_FALSE(dxl.writeInitialValues());
}

// The write is skipped where the register already holds the value. That must not decide whether a
// configuration is legal, or a URDF is valid depending on where the arm stands.
TEST_F(DriverTest, AReadOnlyInitialValueIsRefusedEvenWhenItMatches)
{
  Dynamixel& dxl = connected();
  int32_t present_position = 0;
  ASSERT_TRUE(dxl.readRegister(DXL_REGISTER_POSITION, present_position));
  dxl.setInitialRegisterValues({{std::string(DXL_REGISTER_POSITION), std::to_string(present_position)}});
  EXPECT_FALSE(dxl.writeInitialValues());
}

// 300 & 0xFF is 44, so a masked comparison against a register holding 44 would skip the write
TEST_F(DriverTest, AnOverWideInitialValueIsRefusedEvenWhenItsTruncationMatches)
{
  Dynamixel& dxl = connected();
  motor()->write1Byte(motor()->getAddress("led_blue"), 44);
  dxl.setInitialRegisterValues({{"led_blue", "300"}});
  EXPECT_FALSE(dxl.writeInitialValues());
}

// homing_offset moves the frame of goal_position, so torque comes back only on a goal anchored at
// the present position
TEST_F(DriverTest, TorqueStaysOffWhenTheGoalCannotBeAnchoredAfterAFrameChange)
{
  Dynamixel& dxl = connected();
  ASSERT_TRUE(dxl.writeRegister(DXL_REGISTER_CMD_TORQUE, true));
  motor()->setRegisterReadFault(DXL_REGISTER_POSITION);
  dxl.setInitialRegisterValues({{std::string(DXL_REGISTER_HOMING_OFFSET), "0.5"}});
  EXPECT_FALSE(dxl.writeInitialValues());
  bool torque = true;
  ASSERT_TRUE(dxl.readRegister(DXL_REGISTER_CMD_TORQUE, torque));
  EXPECT_FALSE(torque) << "energised against a stale goal";
}

// One dropped status packet costs a transaction, not the attempt
TEST_F(DriverTest, TheGoalIsAnchoredAfterATransientPositionReadFailure)
{
  Dynamixel& dxl = connected();
  ASSERT_TRUE(dxl.writeRegister(DXL_REGISTER_CMD_POSITION, static_cast<int32_t>(99999)));
  int32_t present_position = 0;
  ASSERT_TRUE(dxl.readRegister(DXL_REGISTER_POSITION, present_position));
  ASSERT_NE(present_position, 99999) << "test premise: the stale goal is not the anchor";
  ASSERT_TRUE(dxl.writeRegister(DXL_REGISTER_CMD_TORQUE, true));
  motor()->setRegisterReadFaultCount(DXL_REGISTER_POSITION, 1);

  dxl.setInitialRegisterValues({{std::string(DXL_REGISTER_HOMING_OFFSET), "0.5"}});
  EXPECT_TRUE(dxl.writeInitialValues());
  int32_t goal = 0;
  ASSERT_TRUE(dxl.readRegister(DXL_REGISTER_CMD_POSITION, goal));
  EXPECT_EQ(goal, present_position);
  bool torque = false;
  ASSERT_TRUE(dxl.readRegister(DXL_REGISTER_CMD_TORQUE, torque));
  EXPECT_TRUE(torque);
}

// current_limit leaves the goal frame alone, so the goal still is where the motor held
TEST_F(DriverTest, TorqueIsRestoredAfterAnOrdinaryEepromWriteDespiteAReadFailure)
{
  Dynamixel& dxl = connected();
  ASSERT_TRUE(dxl.writeRegister(DXL_REGISTER_CMD_TORQUE, true));
  motor()->setRegisterReadFault(DXL_REGISTER_POSITION);
  dxl.setInitialRegisterValues({{"current_limit", "1.0"}});
  EXPECT_TRUE(dxl.writeInitialValues());
  bool torque = false;
  ASSERT_TRUE(dxl.readRegister(DXL_REGISTER_CMD_TORQUE, torque));
  EXPECT_TRUE(torque);
}

// ============================================================================
// Bus watchdog clear
// ============================================================================

TEST_F(DriverTest, ClearingTheBusWatchdogReportsALatchedTrip)
{
  Dynamixel& dxl = connected();
  motor()->tripBusWatchdog();
  bool was_tripped = false;
  EXPECT_EQ(dxl.clearBusWatchdog(&was_tripped), WatchdogResult::SUCCESS);
  EXPECT_TRUE(was_tripped);
  EXPECT_EQ(motor()->getBusWatchdog(), 0);

  was_tripped = true;
  EXPECT_EQ(dxl.clearBusWatchdog(&was_tripped), WatchdogResult::SUCCESS);
  EXPECT_FALSE(was_tripped) << "a healthy watchdog reported a trip";
}

// A model without the register is a valid mixed bus, not a failure
TEST_F(DriverTest, TheBusWatchdogIsUnsupportedOnAModelWithoutTheRegister)
{
  Dynamixel& dxl = connected(1, MODEL_RH);
  ASSERT_FALSE(dxl.registerAvailable(DXL_REGISTER_BUS_WATCHDOG));
  EXPECT_EQ(dxl.clearBusWatchdog(), WatchdogResult::UNSUPPORTED);
}

// ============================================================================
// Joint configuration
// ============================================================================

// loadConfiguration() runs in on_init, where an exception reads as a crashed plugin
TEST_F(DriverTest, AnUnknownPositionControlModeFailsConfigurationWithoutThrowing)
{
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  hardware_interface::ComponentInfo info;
  info.name = "joint_with_bogus_mode";
  info.parameters["id"] = "1";
  info.parameters["position_control_mode"] = "not_a_control_mode";
  Joint joint;
  bool loaded = true;
  EXPECT_NO_THROW({ loaded = joint.loadConfiguration(driver_, info, {}, {}, {}); });
  EXPECT_FALSE(loaded);
}

// Zero is a position, so a calibration before the first read would compute an offset against a pose
// the joint was never in
TEST(JointTest, AResetLeavesTheStateUnreadRatherThanZero)
{
  Joint joint;
  joint.joint_state.current["position"] = 1.0;
  joint.actuator_state.current["position"] = 1.0;
  joint.setupMimicJoint("mimic_joint", 0.0, 1.0);
  joint.mimic_joint_states["mimic_joint"].current["position"] = 1.0;
  joint.reset();
  EXPECT_TRUE(std::isnan(joint.joint_state.current.at("position")));
  EXPECT_TRUE(std::isnan(joint.actuator_state.current.at("position")));
  EXPECT_TRUE(std::isnan(joint.mimic_joint_states.at("mimic_joint").current.at("position")));
}

}  // namespace
}  // namespace dynamixel_ros_control::test
