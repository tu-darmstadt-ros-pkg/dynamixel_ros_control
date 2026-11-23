// SyncReadManager and SyncWriteManager on the simulated bus

#include <dynamixel_ros_control/common.hpp>
#include <dynamixel_ros_control/sync_read_manager.hpp>
#include <dynamixel_ros_control/sync_write_manager.hpp>

#include "driver_fixture.hpp"

namespace dynamixel_ros_control::test {
namespace {

using SyncManagerTest = DriverTest;

// ============================================================================
// SyncWriteManager
// ============================================================================

// On PH 65.535 A is exactly 65535 ticks, which the motor reads as -1
TEST_F(SyncManagerTest, ACyclicWriteHoldsACommandThatWouldArriveWithItsSignFlipped)
{
  Dynamixel& dxl = connected();
  const uint16_t goal = motor()->getAddress(DXL_REGISTER_CMD_EFFORT);
  double goal_current = 0.02;
  SyncWriteManager manager;
  ASSERT_TRUE(manager.addRegister(dxl, DXL_REGISTER_CMD_EFFORT, goal_current));
  ASSERT_TRUE(manager.init(driver_));
  ASSERT_TRUE(manager.write());
  ASSERT_EQ(motor()->read2Byte(goal), 20);

  goal_current = 65.535;
  EXPECT_TRUE(manager.write()) << "the write goes out with the last good value";
  EXPECT_EQ(motor()->read2Byte(goal), 20) << "the sign flipping command was sent";

  goal_current = 0.03;
  EXPECT_TRUE(manager.write());
  EXPECT_EQ(motor()->read2Byte(goal), 30);
}

// It holds, like a NaN, instead of jumping to INT32_MAX ticks
TEST_F(SyncManagerTest, ACyclicWriteHoldsAFiniteCommandWithNoRawEquivalent)
{
  Dynamixel& dxl = connected();
  const uint16_t goal = motor()->getAddress(DXL_REGISTER_CMD_POSITION);
  double goal_position = 0.5;
  SyncWriteManager manager;
  ASSERT_TRUE(manager.addRegister(dxl, DXL_REGISTER_CMD_POSITION, goal_position));
  ASSERT_TRUE(manager.init(driver_));
  ASSERT_TRUE(manager.write());
  const uint32_t held = motor()->read4Byte(goal);
  ASSERT_NE(held, 0U);
  goal_position = 1e30;
  EXPECT_TRUE(manager.write());
  EXPECT_EQ(motor()->read4Byte(goal), held);
}

// A sync write gets no status, so a read only register in the group would be dropped by every motor
// silently. The refusal has to reach the caller, and init() has to refuse the incomplete group.
TEST_F(SyncManagerTest, ASyncWriteGroupRefusesAReadOnlyRegister)
{
  Dynamixel& dxl = connected();
  double value = 0.0;
  SyncWriteManager manager;
  EXPECT_FALSE(manager.addRegister(dxl, DXL_REGISTER_POSITION, value));
  EXPECT_FALSE(manager.init(driver_));

  SyncWriteManager writable;
  EXPECT_TRUE(writable.addRegister(dxl, DXL_REGISTER_CMD_POSITION, value));
  EXPECT_TRUE(writable.init(driver_));
}

// Two joints on one motor id meet only here, where the second is refused and the group would
// command the first alone
TEST_F(SyncManagerTest, ASyncWriteGroupWithTwoJointsOnOneIdFailsToInitialise)
{
  Dynamixel& first = connected();
  Dynamixel second(1, driver_);
  ASSERT_TRUE(second.connect());
  double first_goal = 0.0;
  double second_goal = 0.0;
  SyncWriteManager manager;
  ASSERT_TRUE(manager.addRegister(first, "goal_position", first_goal));
  ASSERT_TRUE(manager.addRegister(second, "goal_position", second_goal));
  EXPECT_FALSE(manager.init(driver_));
  EXPECT_FALSE(manager.write()) << "an incomplete group wrote";
}

TEST_F(SyncManagerTest, AWriteOfSelectedMotorsLeavesTheOthersAlone)
{
  Dynamixel& dxl1 = connected(1);
  Dynamixel& dxl2 = connected(2);
  double goal1 = 0.0;
  double goal2 = 0.0;
  SyncWriteManager manager;
  ASSERT_TRUE(manager.addRegister(dxl1, "goal_position", goal1));
  ASSERT_TRUE(manager.addRegister(dxl2, "goal_position", goal2));
  ASSERT_TRUE(manager.init(driver_));
  const uint16_t goal = motor(1)->getAddress("goal_position");
  const uint32_t before = motor(2)->read4Byte(goal);

  goal1 = 1.0;
  goal2 = 1.0;
  EXPECT_TRUE(manager.writeSelected({&dxl1}));
  EXPECT_NE(motor(1)->read4Byte(goal), 0U);
  EXPECT_EQ(motor(2)->read4Byte(goal), before) << "an unselected motor was written";
}

// A sync write is transmit only, so an unverifiable torque off still reaches a silent motor
TEST_F(SyncManagerTest, AWriteOfSelectedMotorsReachesAMotorThatCannotAnswer)
{
  Dynamixel& dxl = connected();
  double goal_position = 0.0;
  SyncWriteManager manager;
  ASSERT_TRUE(manager.addRegister(dxl, "goal_position", goal_position));
  ASSERT_TRUE(manager.init(driver_));
  const uint16_t goal = motor()->getAddress("goal_position");

  motor()->setCommunicationFault(CommunicationFault::DROP_STATUS_PACKETS);
  goal_position = 1.0;
  EXPECT_TRUE(manager.writeSelected({&dxl}));
  EXPECT_NE(motor()->read4Byte(goal), 0U);

  const uint32_t before = motor()->read4Byte(goal);
  motor()->setCommunicationFault(CommunicationFault::DISCONNECTED);
  goal_position = -1.0;
  EXPECT_TRUE(manager.writeSelected({&dxl})) << "a transmit only write failed";
  EXPECT_EQ(motor()->read4Byte(goal), before) << "a disconnected motor applied it";
}

// A reboot wipes the pointers of a RAM pointer model, and a group left out of the rewrite keeps
// writing into slots that point nowhere, with every packet a success
TEST_F(SyncManagerTest, RewritingIndirectAddressesRestoresWritesAfterAReboot)
{
  Dynamixel& dxl = connected(1, MODEL_XM);
  ASSERT_TRUE(dxl.indirectAddressesInRam());
  SyncWriteManager manager;
  int32_t ticks = 4;
  ASSERT_TRUE(manager.addRegister(dxl, DXL_REGISTER_BUS_WATCHDOG, ticks));
  ASSERT_TRUE(manager.init(driver_));
  ASSERT_TRUE(manager.writeSelected({&dxl}));
  ASSERT_EQ(motor()->getBusWatchdog(), 4);

  motor()->resetRAMRegisters();
  ticks = 7;
  ASSERT_TRUE(manager.writeSelected({&dxl}));
  ASSERT_NE(motor()->getBusWatchdog(), 7) << "test premise: wiped pointers lose the write";

  ASSERT_TRUE(manager.rewriteIndirectAddresses({&dxl}));
  ASSERT_TRUE(manager.writeSelected({&dxl}));
  EXPECT_EQ(motor()->getBusWatchdog(), 7);
}

// ============================================================================
// SyncReadManager
// ============================================================================

// A sync read reaches its registers through indirect pointers, which may only address RAM
TEST_F(SyncManagerTest, ASyncReadGroupIsBuiltOnlyOverRamRegisters)
{
  Dynamixel& dxl = connected();
  int32_t mode = 0;
  SyncReadManager eeprom;
  eeprom.addDynamixel(&dxl);
  ASSERT_TRUE(eeprom.addRegister("operating_mode", {{&dxl, DxlValue(&mode)}}));
  EXPECT_FALSE(eeprom.init(driver_)) << "a group over an EEPROM register came up";

  bool torqued = false;
  SyncReadManager ram;
  ram.addDynamixel(&dxl);
  ASSERT_TRUE(ram.addRegister(DXL_REGISTER_CMD_TORQUE, {{&dxl, DxlValue(&torqued)}}));
  ASSERT_TRUE(ram.init(driver_));
  ASSERT_TRUE(dxl.writeRegister(DXL_REGISTER_CMD_TORQUE, true));
  ASSERT_TRUE(ram.read());
  EXPECT_TRUE(torqued);
}

// A state interface over an unsigned register is decoded here every cycle
TEST_F(SyncManagerTest, ASyncReadDecodesUnsignedRegistersUnsigned)
{
  Dynamixel& dxl = connected();
  motor()->write1Byte(motor()->getAddress("led_blue"), 255);
  double led_blue = 0.0;
  SyncReadManager manager;
  manager.addDynamixel(&dxl);
  ASSERT_TRUE(manager.addRegister("led_blue", {{&dxl, DxlValue(&led_blue)}}));
  ASSERT_TRUE(manager.init(driver_));
  ASSERT_TRUE(manager.read());
  EXPECT_DOUBLE_EQ(led_blue, 255.0);
}

// On a mixed bus a register only some models have still reaches the others
TEST_F(SyncManagerTest, AddingARegisterFailsWhenOneModelLacksIt)
{
  Dynamixel& ph = connected(1, MODEL_PH);
  Dynamixel& xm = connected(2, MODEL_XM);
  ASSERT_TRUE(ph.registerAvailable("external_port_data_1"));
  ASSERT_FALSE(xm.registerAvailable("external_port_data_1"));
  SyncReadManager manager;
  manager.addDynamixel(&ph);
  manager.addDynamixel(&xm);
  double ph_value = 0.0;
  double xm_value = 0.0;
  bool added = true;
  EXPECT_NO_THROW({
    added = manager.addRegister("external_port_data_1", {{&ph, DxlValue(&ph_value)}, {&xm, DxlValue(&xm_value)}});
  });
  EXPECT_FALSE(added);
}

// During a partial bus loss the group read answers for nobody, so motors are read one at a time.
// That must not move the consecutive error count the interface decides on.
TEST_F(SyncManagerTest, ASelectiveReadAnswersForTheRespondersWhenTheGroupFails)
{
  Dynamixel& dxl1 = connected(1);
  Dynamixel& dxl2 = connected(2);
  double pos1 = 0.0;
  double pos2 = 0.0;
  SyncReadManager manager;
  manager.addDynamixel(&dxl1);
  manager.addDynamixel(&dxl2);
  ASSERT_TRUE(manager.addRegister("present_position", {{&dxl1, DxlValue(&pos1)}, {&dxl2, DxlValue(&pos2)}}));
  ASSERT_TRUE(manager.init(driver_));
  EXPECT_TRUE(manager.read());
  const unsigned int errors_before = manager.getErrorCount();

  motor(2)->setCommunicationFault(CommunicationFault::DROP_STATUS_PACKETS);
  EXPECT_FALSE(manager.read());
  EXPECT_TRUE(manager.readSelected({&dxl1}));
  EXPECT_FALSE(manager.readSelected({&dxl2}));
  EXPECT_FALSE(manager.readSelected({&dxl1, &dxl2}));
  EXPECT_EQ(manager.getErrorCount(), errors_before + 1) << "selective reads moved the cyclic error count";
}

// A motor the manager was not told about is no member, so asking for it is no failure
TEST_F(SyncManagerTest, ASelectiveReadIgnoresUnregisteredMotors)
{
  Dynamixel& dxl1 = connected(1);
  Dynamixel& dxl2 = connected(2);
  double pos1 = 0.0;
  SyncReadManager manager;
  manager.addDynamixel(&dxl1);
  ASSERT_TRUE(manager.addRegister("present_position", {{&dxl1, DxlValue(&pos1)}}));
  ASSERT_TRUE(manager.init(driver_));
  EXPECT_TRUE(manager.readSelected({&dxl2}));
  EXPECT_TRUE(manager.readSelected({&dxl1, &dxl2}));
}

// Callers commit the read back values of the motors in the set, and for torque_enable a default
// false would report a live motor as limp
TEST_F(SyncManagerTest, AManagerReportsNobodyAnsweringWhatItDoesNotRead)
{
  Dynamixel& dxl = connected(1);
  SyncReadManager unconfigured;
  unconfigured.addDynamixel(&dxl);
  EXPECT_TRUE(unconfigured.readWhoeverAnswers({&dxl}).empty()) << "a group with no registers answered";

  Dynamixel stranger(2, driver_);
  bool torqued = false;
  SyncReadManager configured;
  configured.addDynamixel(&dxl);
  ASSERT_TRUE(configured.addRegister(DXL_REGISTER_CMD_TORQUE, {{&dxl, DxlValue(&torqued)}}));
  ASSERT_TRUE(configured.init(driver_));
  EXPECT_TRUE(configured.readWhoeverAnswers({&stranger}).empty()) << "a motor outside the group answered";
}

}  // namespace
}  // namespace dynamixel_ros_control::test
