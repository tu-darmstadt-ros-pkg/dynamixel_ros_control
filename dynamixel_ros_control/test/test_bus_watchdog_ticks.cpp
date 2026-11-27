// The watchdog window is a safety parameter that reaches the motor as a whole number of register
// ticks, so the arithmetic that gets it there matters: rounding the wrong way shortens the window,
// and the register only accepts 1..127. None of that needs a motor, a bus or a controller manager.

#include <gtest/gtest.h>

#include <dynamixel_ros_control/bus_watchdog.hpp>
#include <dynamixel_ros_control/dynamixel_driver.hpp>
#include <dynamixel_ros_control/mock_dynamixel.hpp>
#include <dynamixel_ros_control/sync_read_manager.hpp>
#include <dynamixel_ros_control/sync_write_manager.hpp>

#include <mutex>
#include <unordered_map>

namespace dynamixel_ros_control::test {

// 20 ms control period at 50 Hz, and the ms_20 register resolution the PH series uses.
constexpr double DT_MS_50HZ = 20.0;
constexpr double MS_PER_TICK_PH = 20.0;

// Zero is the documented way to disable the watchdog. It must not become "one tick", which arms
// the shortest possible window instead of disabling anything.
TEST(BusWatchdogTicks, ZeroCyclesDisablesRatherThanArmingTheShortestWindow)
{
  EXPECT_EQ(busWatchdogTicks(0.0, DT_MS_50HZ, MS_PER_TICK_PH), 0);
}

// The default configuration the hardware ships with, and the value test_hw_bus_watchdog asserts
// against the mock: 4 cycles * 20 ms = 80 ms, at 20 ms per tick, is 4 ticks.
TEST(BusWatchdogTicks, DefaultWindowMatchesTheConfiguredCycleCount)
{
  EXPECT_EQ(busWatchdogTicks(DEFAULT_BUS_WATCHDOG_CYCLES, DT_MS_50HZ, MS_PER_TICK_PH), 4);
}

// Rounding is up, never to nearest. A window rounded down is shorter than the operator asked for,
// which is the direction that makes the motor trip on a bus that is actually healthy.
TEST(BusWatchdogTicks, RoundsUpSoTheWindowIsNeverShorterThanRequested)
{
  // 3.1 cycles * 20 ms = 62 ms. At 20 ms per tick that is 3.1 ticks, which must become 4, not 3.
  EXPECT_EQ(busWatchdogTicks(3.1, DT_MS_50HZ, MS_PER_TICK_PH), 4);
  // Exact multiples must not be pushed up a tick by the same rule.
  EXPECT_EQ(busWatchdogTicks(3.0, DT_MS_50HZ, MS_PER_TICK_PH), 3);
}

// The register is one signed byte: 0 disables, 1..127 select a window. A long window or a fine
// resolution must saturate at 127 rather than wrap into the negative range, which the motor reads
// as a latched trip.
TEST(BusWatchdogTicks, SaturatesAtTheRegisterMaximum)
{
  EXPECT_EQ(busWatchdogTicks(1000.0, DT_MS_50HZ, MS_PER_TICK_PH), DXL_BUS_WATCHDOG_MAX_TICKS);
  EXPECT_EQ(busWatchdogTicks(DEFAULT_BUS_WATCHDOG_CYCLES, DT_MS_50HZ, 0.001), DXL_BUS_WATCHDOG_MAX_TICKS);
}

// A cycle count large enough to overflow int must saturate too. Narrowing such a value before
// clamping is undefined behaviour, and an operator can put any number in the URDF.
TEST(BusWatchdogTicks, AnAbsurdCycleCountSaturatesInsteadOfOverflowing)
{
  EXPECT_EQ(busWatchdogTicks(1e18, DT_MS_50HZ, MS_PER_TICK_PH), DXL_BUS_WATCHDOG_MAX_TICKS);
}

// A window that quantises below one tick must still arm one, not zero: zero means "disabled", and
// silently disabling a watchdog the operator asked for is the one outcome worse than a short one.
TEST(BusWatchdogTicks, NeverQuantisesDownToDisabled)
{
  EXPECT_EQ(busWatchdogTicks(MIN_BUS_WATCHDOG_WINDOW_CYCLES, 0.001, 1000.0), DXL_BUS_WATCHDOG_MIN_TICKS);
}

// A control rate or resolution of zero yields no window rather than a division by zero.
TEST(BusWatchdogTicks, DegenerateInputsYieldNoWindow)
{
  EXPECT_EQ(busWatchdogTicks(DEFAULT_BUS_WATCHDOG_CYCLES, 0.0, MS_PER_TICK_PH), 0);
  EXPECT_EQ(busWatchdogTicks(DEFAULT_BUS_WATCHDOG_CYCLES, DT_MS_50HZ, 0.0), 0);
}

// ============================================================================
// configure(): the window a joint ends up with, not just the arithmetic
// ============================================================================

// PH54-200-S500-R, whose bus_watchdog register counts in 20 ms ticks.
constexpr uint16_t MODEL_PH = 2020;

// Builds one joint on a mock bus, which is all configure() needs: it reads the model's tick
// resolution from the control table and writes back a tick count, without touching the bus.
class BusWatchdogConfigureTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    MockDynamixelManager::instance().reset();
    ASSERT_TRUE(driver_.init("/dev/ttyUSB0", 57600, true));
    MockDynamixelManager::instance().addMotor(1, MODEL_PH);

    Joint joint;
    joint.name = "joint";
    joint.dynamixel = std::make_shared<Dynamixel>(1, driver_);
    ASSERT_TRUE(joint.dynamixel->connect());
    joints_.emplace(joint.name, std::move(joint));

    watchdog_.init(&joints_, &write_manager_);
  }

  void TearDown() override
  {
    MockDynamixelManager::instance().reset();
  }

  int32_t ticks() const
  {
    return joints_.at("joint").bus_watchdog_ticks;
  }

  DynamixelDriver driver_;
  std::unordered_map<std::string, Joint> joints_;
  std::mutex comm_mutex_;
  SyncReadManager read_manager_;
  SyncWriteManager write_manager_;
  BusWatchdog watchdog_;
};

// The ordinary case: a window the register can express is configured and kept.
TEST_F(BusWatchdogConfigureTest, AWindowTheRegisterCanExpressIsConfigured)
{
  EXPECT_TRUE(watchdog_.configure(DEFAULT_BUS_WATCHDOG_CYCLES, 50));
  EXPECT_EQ(ticks(), 4);  // 4 cycles * 20 ms, at 20 ms per tick
}

// At a control rate this low the minimum window (3 cycles = 3 s) exceeds what 127 ticks of 20 ms
// can express (2.54 s). The rounding fallback recomputes the same saturated value, so accepting it
// would arm a window below the jitter floor while the log claims the default was substituted. That
// is the "arm frozen, goals silently ignored" failure the minimum exists to prevent.
TEST_F(BusWatchdogConfigureTest, AWindowBoundedByTheRegisterCapIsRefused)
{
  EXPECT_FALSE(watchdog_.configure(DEFAULT_BUS_WATCHDOG_CYCLES, 1))
      << "a window the register cannot express must fail configuration rather than be armed short";
}

// The documented opt-out stays available at any rate: 0 disables the watchdog outright.
TEST_F(BusWatchdogConfigureTest, DisablingStaysPossibleAtAnyRate)
{
  EXPECT_TRUE(watchdog_.configure(0.0, 1));
  EXPECT_EQ(ticks(), 0);
}

// ============================================================================
// clear() / arm() over a real mock bus
// ============================================================================
//
// The register is one byte the motor latches at -1 on a trip and only a 0 clears. Everything the
// watchdog does is those two writes, so each one has to reach every capable motor, and the cyclic
// read has to report the result.
class BusWatchdogBusTest : public BusWatchdogConfigureTest
{
protected:
  void SetUp() override
  {
    BusWatchdogConfigureTest::SetUp();
    ASSERT_TRUE(watchdog_.configure(DEFAULT_BUS_WATCHDOG_CYCLES, 50));
    ASSERT_TRUE(setUpManagers());
  }

  // The sync managers the watchdog drives. configure() alone does not touch the bus.
  bool setUpManagers()
  {
    Joint& joint = joints_.at("joint");
    read_manager_ = SyncReadManager();
    read_manager_.addDynamixel(joint.dynamixel.get());
    DxlValueMappingList mappings{{joint.dynamixel.get(), DxlValue(&joint.dynamixel->bus_watchdog_value)}};
    if (!read_manager_.addRegister(DXL_REGISTER_BUS_WATCHDOG, mappings) || !read_manager_.init(driver_)) {
      return false;
    }
    write_manager_ = SyncWriteManager();
    if (!write_manager_.addRegister(*joint.dynamixel, DXL_REGISTER_BUS_WATCHDOG, joint.bus_watchdog_command)) {
      return false;
    }
    return write_manager_.init(driver_);
  }

  int32_t registerValue() const
  {
    return MockDynamixelManager::instance().getMotor(1)->getBusWatchdog();
  }

  // The control loop refreshes the bound value from the bus.
  void refresh()
  {
    ASSERT_TRUE(read_manager_.read());
  }
};

// The control loop warns about a bus silence that used more than half of an armed window
TEST_F(BusWatchdogBusTest, ASilenceThreatensOnlyAnArmedWindow)
{
  const double window_ms = watchdog_.shortestWindowMs();
  ASSERT_GT(window_ms, 0.0);
  EXPECT_FALSE(watchdog_.silenceThreatensWindow(window_ms)) << "nothing is armed yet";

  ASSERT_TRUE(watchdog_.arm());
  EXPECT_TRUE(watchdog_.silenceThreatensWindow(0.6 * window_ms));
  EXPECT_FALSE(watchdog_.silenceThreatensWindow(0.4 * window_ms));
}

// arm() of a disabled watchdog still records the intent, and every silence exceeds half of 0 ms
TEST_F(BusWatchdogBusTest, ASilenceThreatensNoDisabledWatchdog)
{
  ASSERT_TRUE(watchdog_.configure(0.0, 50));
  ASSERT_TRUE(watchdog_.arm());
  ASSERT_TRUE(watchdog_.armIntended());
  EXPECT_FALSE(watchdog_.silenceThreatensWindow(20.0));
}

// arm() puts the configured window in the register, clear() takes it back out.
TEST_F(BusWatchdogBusTest, ArmWritesTheWindowAndClearWritesZero)
{
  ASSERT_TRUE(watchdog_.arm());
  EXPECT_EQ(registerValue(), 4);

  ASSERT_TRUE(watchdog_.clear());
  EXPECT_EQ(registerValue(), 0);
}

// Both are a single sync write, whatever the state they are leaving. A per-motor fallback here
// would run on the real-time thread, since write() clears and re-arms inline on a trip.
TEST_F(BusWatchdogBusTest, ClearAndArmAreOneTransactionEach)
{
  ASSERT_TRUE(watchdog_.arm());

  const CommStats before = driver_.getCommStats();
  ASSERT_TRUE(watchdog_.clear());
  ASSERT_TRUE(watchdog_.arm());
  const CommStats delta = driver_.getCommStats().delta(before);

  EXPECT_EQ(delta.group_writes, 2u) << "clear+arm cost more than one transaction each";
  EXPECT_EQ(delta.group_reads, 0u) << "the cyclic read reports the result; neither may read for itself";
}

// A latch is only visible once the cyclic read has run: trippedJoints() reports what was read, and
// must not report a trip from a stale value nobody refreshed.
TEST_F(BusWatchdogBusTest, ATripIsReportedOnlyAfterTheReadThatSawIt)
{
  ASSERT_TRUE(watchdog_.arm());
  refresh();
  ASSERT_TRUE(watchdog_.trippedJoints().empty());

  MockDynamixelManager::instance().getMotor(1)->tripBusWatchdog();
  EXPECT_TRUE(watchdog_.trippedJoints().empty()) << "a trip was reported before any read observed it";

  refresh();
  EXPECT_EQ(watchdog_.trippedJoints(), std::vector<std::string>{"joint"});
}

// The recovery write() performs: clear the latch, then re-arm. Both must land, or the goal writes
// that follow are silently discarded by the motor.
TEST_F(BusWatchdogBusTest, ClearThenArmRecoversALatchedTrip)
{
  ASSERT_TRUE(watchdog_.arm());
  MockDynamixelManager::instance().getMotor(1)->tripBusWatchdog();
  refresh();
  ASSERT_FALSE(watchdog_.trippedJoints().empty());

  ASSERT_TRUE(watchdog_.clear());
  ASSERT_TRUE(watchdog_.arm());
  refresh();

  EXPECT_TRUE(watchdog_.trippedJoints().empty());
  EXPECT_TRUE(watchdog_.unarmedJoints().empty()) << "the re-arm did not restore the window";
}

// A joint whose window did not land is not armed, however healthy the bus looks. on_activate
// checks this before it reports success.
TEST_F(BusWatchdogBusTest, UnarmedIsReportedUntilTheWindowIsReadBack)
{
  refresh();
  EXPECT_EQ(watchdog_.unarmedJoints(), std::vector<std::string>{"joint"});

  ASSERT_TRUE(watchdog_.arm());
  refresh();
  EXPECT_TRUE(watchdog_.unarmedJoints().empty());
}

// A disabled watchdog has nothing to report, whatever the register happens to hold. The joint's
// window is zero, so no read of it can accuse the control loop of starving it.
TEST_F(BusWatchdogBusTest, ADisabledWatchdogNeverReportsATrip)
{
  ASSERT_TRUE(watchdog_.configure(0.0, 50));
  MockDynamixelManager::instance().getMotor(1)->tripBusWatchdog();
  refresh();

  EXPECT_TRUE(watchdog_.trippedJoints().empty());
  EXPECT_TRUE(watchdog_.unarmedJoints().empty());
}

// ============================================================================
// Mixed buses
// ============================================================================

// H42-20-S300-R has no bus_watchdog register.
constexpr uint16_t MODEL_WITHOUT_WATCHDOG = 51200;

// One motor with the register and one without. The sync write covers only the capable one. The
// other is not written, and it cannot fail the group.
TEST(BusWatchdogMixedBus, OnlyCapableMotorsAreArmed)
{
  MockDynamixelManager::instance().reset();
  DynamixelDriver driver;
  ASSERT_TRUE(driver.init("/dev/ttyUSB0", 57600, true));
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  MockDynamixelManager::instance().addMotor(2, MODEL_WITHOUT_WATCHDOG);

  std::unordered_map<std::string, Joint> joints;
  for (uint8_t id : {1, 2}) {
    Joint joint;
    joint.name = "joint_" + std::to_string(id);
    joint.dynamixel = std::make_shared<Dynamixel>(id, driver);
    ASSERT_TRUE(joint.dynamixel->connect());
    joints.emplace(joint.name, std::move(joint));
  }

  SyncWriteManager write_manager;
  BusWatchdog watchdog;
  watchdog.init(&joints, &write_manager);
  ASSERT_TRUE(watchdog.configure(DEFAULT_BUS_WATCHDOG_CYCLES, 50));

  EXPECT_EQ(watchdog.capableMotors().size(), 1u) << "the motor without the register joined the watchdog group";
  EXPECT_EQ(joints.at("joint_2").bus_watchdog_ticks, 0) << "a window was configured for a motor that cannot hold one";

  for (auto& [name, joint] : joints) {
    if (watchdog.capableMotors().count(joint.dynamixel.get()) != 0) {
      ASSERT_TRUE(write_manager.addRegister(*joint.dynamixel, DXL_REGISTER_BUS_WATCHDOG, joint.bus_watchdog_command));
    }
  }
  ASSERT_TRUE(write_manager.init(driver));

  EXPECT_TRUE(watchdog.arm());
  EXPECT_EQ(MockDynamixelManager::instance().getMotor(1)->getBusWatchdog(), 4);
  EXPECT_TRUE(watchdog.trippedJoints().empty()) << "a motor with no watchdog register reported a trip";

  MockDynamixelManager::instance().reset();
}

// The watchdog register rides in the cyclic state read, which every motor on the bus takes part in.
// A motor without the register maps a padding register into its slot instead, and the bytes that
// come back belong to that register. They are not a watchdog value and must not be stored as one.
// Storing them would make a torque_enable of 1 read as an armed window, or a 0 as a disarmed one.
TEST(BusWatchdogMixedBus, PaddedMotorsDoNotDecodeTheirPaddingAsAWatchdogValue)
{
  MockDynamixelManager::instance().reset();
  DynamixelDriver driver;
  ASSERT_TRUE(driver.init("/dev/ttyUSB0", 57600, true));
  MockDynamixelManager::instance().addMotor(1, MODEL_PH);
  MockDynamixelManager::instance().addMotor(2, MODEL_WITHOUT_WATCHDOG);

  std::vector<std::shared_ptr<Dynamixel>> motors;
  SyncReadManager read_manager;
  for (uint8_t id : {1, 2}) {
    auto dxl = std::make_shared<Dynamixel>(id, driver);
    ASSERT_TRUE(dxl->connect());
    read_manager.addDynamixel(dxl.get());
    motors.push_back(dxl);
  }

  DxlValueMappingList mappings;
  for (const auto& dxl : motors) {
    mappings.emplace_back(dxl.get(), DxlValue(&dxl->bus_watchdog_value));
  }
  ASSERT_TRUE(read_manager.addRegister(DXL_REGISTER_BUS_WATCHDOG, mappings, DXL_REGISTER_CMD_TORQUE))
      << "A motor without the register must still be able to join the group via its padding register";
  ASSERT_TRUE(read_manager.init(driver));

  // The padded motor's slot points at torque_enable, so give it a value a watchdog reading would
  // be mistaken for.
  auto padded = MockDynamixelManager::instance().getMotor(2);
  ASSERT_NE(padded, nullptr);
  padded->write1Byte(padded->getAddress("torque_enable"), 1);

  constexpr int32_t SENTINEL = -99;
  motors[1]->bus_watchdog_value = SENTINEL;
  ASSERT_TRUE(read_manager.read());

  EXPECT_EQ(motors[1]->bus_watchdog_value, SENTINEL)
      << "The padded motor's torque_enable byte was decoded into its bus_watchdog value. It has no "
         "watchdog register, so any value read for it is a fiction the trip check would act on.";

  MockDynamixelManager::instance().reset();
}

}  // namespace dynamixel_ros_control::test
