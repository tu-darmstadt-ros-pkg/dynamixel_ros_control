// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include <tuple>

#include "direct_hardware_fixture.hpp"

namespace dynamixel_ros_control::test {

// ============================================================================
// Bus Watchdog Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, BusWatchdog_ConfiguredOnStartup)
{
  // The hardware interface computes the bus watchdog timeout during on_configure and arms the
  // register at the end of on_activate (the fixture auto-activates).
  // With update_rate=50 Hz, dt=20 ms, watchdog = 4 * 20 ms = 80 ms.
  // The bus_watchdog register uses ms_20 unit (20 ms per tick), so the
  // expected register value is 80 / 20 = 4 ticks.
  constexpr uint8_t EXPECTED_WATCHDOG_TICKS = 4;

  // Check all arm motors
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr) << "Motor ID " << static_cast<int>(id) << " not found";
    EXPECT_EQ(motor->getBusWatchdog(), EXPECTED_WATCHDOG_TICKS)
        << "Bus watchdog not correctly configured for arm motor " << static_cast<int>(id);
  }

  // Check gripper motor
  auto gripper = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(GRIPPER_ID);
  ASSERT_NE(gripper, nullptr) << "Gripper motor not found";
  EXPECT_EQ(gripper->getBusWatchdog(), EXPECTED_WATCHDOG_TICKS)
      << "Bus watchdog not correctly configured for gripper motor";

  // Check all flipper motors
  for (uint8_t id : FLIPPER_MOTOR_IDS) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr) << "Motor ID " << static_cast<int>(id) << " not found";
    EXPECT_EQ(motor->getBusWatchdog(), EXPECTED_WATCHDOG_TICKS)
        << "Bus watchdog not correctly configured for flipper motor " << static_cast<int>(id);
  }
}

// Deactivation stops read()/write(), so nothing feeds the watchdog from that point on. Leaving it
// armed guarantees a trip a few cycles later, and a trip latches at -1 and rejects every goal write
// until cleared. The motor ends up in the same physical state either way. It is stationary because
// nothing commands it, so the latch buys no safety and only leaves the arm in a state the reboot
// service and the diagnostics both have to work around.
TEST_F(HardwareInterfaceTest, BusWatchdog_DisarmedOnDeactivation)
{
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr) << "Motor ID " << static_cast<int>(id) << " not found";
    ASSERT_NE(motor->getBusWatchdog(), 0) << "Test premise: the watchdog should be armed while active";
  }

  ASSERT_TRUE(
      setHardwareComponentState("athena_arm_interface", lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive"))
      << "Deactivation should succeed";

  std::this_thread::sleep_for(300ms);

  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr);
    EXPECT_EQ(motor->getBusWatchdog(), 0)
        << "Motor " << static_cast<int>(id)
        << " left the interface with its watchdog still armed, so it will trip and latch with "
           "nothing left to feed or clear it.";
  }
}

TEST_F(HardwareInterfaceTest, BusWatchdog_LatchedErrorClearedOnReactivation)
{
  // A watchdog that trips while torqued latches its register at -1 (0xFF) and silently rejects
  // goal writes until 0 is written. This happens in practice when traffic stops while torque is
  // still on (process killed, deactivated interface). Reactivation must clear the latch and re-arm
  // the watchdog. A latch that survives leaves the arm frozen while reporting itself "active".
  // 1. Deactivate the hardware interface
  ASSERT_TRUE(
      setHardwareComponentState("athena_arm_interface", lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive"))
      << "Deactivation should succeed";

  // 2. Inject a tripped/latched watchdog into all arm motors (register value -1 = 0xFF)
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr) << "Motor ID " << static_cast<int>(id) << " not found";
    motor->write1Byte(motor->getAddress("bus_watchdog"), 0xFF);
  }

  // 3. Reactivate — must succeed despite the latched watchdog
  ASSERT_TRUE(
      setHardwareComponentState("athena_arm_interface", lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active"))
      << "Reactivation should succeed despite latched watchdog";

  std::this_thread::sleep_for(300ms);

  // 4. The latch must be cleared and the watchdog re-armed with the configured value
  constexpr uint8_t EXPECTED_WATCHDOG_TICKS = 4;
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr);
    EXPECT_EQ(motor->getBusWatchdog(), EXPECTED_WATCHDOG_TICKS)
        << "Bus watchdog not re-armed after reactivation for motor " << static_cast<int>(id);
  }
}

TEST_F(HardwareInterfaceTest, BusWatchdog_TripDuringOperationIsRearmed)
{
  // The bus watchdog register is part of the cyclic sync read. If it trips mid-operation
  // (register latches at -1, e.g. after a control-loop stall), read() must detect it and
  // clear + re-arm within a few control cycles — without any lifecycle transition.
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr) << "Motor ID " << static_cast<int>(id) << " not found";
    motor->write1Byte(motor->getAddress("bus_watchdog"), 0xFF);  // -1 = tripped/latched
  }

  // A couple of 50 Hz control cycles are plenty for detection + re-arm.
  std::this_thread::sleep_for(500ms);

  constexpr uint8_t EXPECTED_WATCHDOG_TICKS = 4;
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr);
    EXPECT_EQ(motor->getBusWatchdog(), EXPECTED_WATCHDOG_TICKS)
        << "Tripped bus watchdog not re-armed by the cyclic monitoring for motor " << static_cast<int>(id);
  }
}

// A clear that does not reach the motors must not cost the intent to be armed. The repair runs on
// every control cycle and is gated on that intent, so losing it leaves a latched motor rejecting
// goal writes with nothing left in the process that knows it should not be.
TEST_F(HardwareInterfaceTest, BusWatchdog_FailedClearStillLeavesTheRepairArmed)
{
  auto& manager = dynamixel_ros_control::MockDynamixelManager::instance();
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = manager.getMotor(id);
    ASSERT_NE(motor, nullptr);
    // Latched, which is what makes the control loop attempt the clear-and-re-arm at all.
    motor->write1Byte(motor->getAddress("bus_watchdog"), 0xFF);
  }
  // And nothing it sends reaches the bus, so the clear fails. Kept well short of the consecutive
  // error count that declares the connection lost, since that would take the component down and
  // stop the control loop that does the repairing.
  manager.setSyncWriteTransmissionFailure(true);
  std::this_thread::sleep_for(150ms);

  // The bus recovers.
  manager.setSyncWriteTransmissionFailure(false);
  std::this_thread::sleep_for(500ms);

  constexpr uint8_t EXPECTED_WATCHDOG_TICKS = 4;
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = manager.getMotor(id);
    ASSERT_NE(motor, nullptr);
    EXPECT_EQ(motor->getBusWatchdog(), EXPECTED_WATCHDOG_TICKS)
        << "Motor " << static_cast<int>(id)
        << " was left latched: the failed clear dropped the intent to be armed, and the repair that "
           "keys on it never ran again.";
  }
}

// ============================================================================
// bus_watchdog_cycles parameter tests
//
// The watchdog timeout is computed as
//   ticks = clamp(ceil(dt_ms * cycles / ms_per_tick), MIN_TICKS=1, MAX_TICKS=127)
// where dt_ms = 1000 / rw_rate. The test rig runs the arm interface at 50 Hz (dt_ms = 20) and the
// PH-series bus_watchdog register uses the ms_20 unit (ms_per_tick = 20), so on the arm motors the
// formula collapses to ticks = clamp(ceil(cycles), 1, 127). The default cycles is 4 (-> 4 ticks),
// which BusWatchdog_ConfiguredOnStartup already covers. These tests inject a non-default
// bus_watchdog_cycles hardware parameter into the arm interface and verify the override, the upper
// clamp, the disable behaviour (cycles == 0 -> 0 ticks), and that invalid values are refused.
// ============================================================================

// Fixture that injects a configurable bus_watchdog_cycles parameter into the arm <hardware> block.
class BusWatchdogCyclesTest : public HardwareInterfaceTest
{
protected:
  // The literal text written into the URDF for <param name="bus_watchdog_cycles">. Set by each test
  // before SetUp() runs. A string (not a number) so the invalid "0"/"-1" cases can be exercised
  // exactly as a user would configure them.
  std::string bus_watchdog_cycles_param_ = "8";

  std::string transformUrdf(std::string urdf) override
  {
    // Anchor on the arm interface's unique port_name so only the arm <hardware> block is touched.
    const std::string anchor = "<param name=\"port_name\">/dev/ttyUSB_manipulator_arm</param>";
    const auto pos = urdf.find(anchor);
    if (pos == std::string::npos) {
      ADD_FAILURE() << "Could not find arm interface anchor to inject bus_watchdog_cycles";
      return urdf;
    }
    const std::string injected =
        anchor + "\n      <param name=\"bus_watchdog_cycles\">" + bus_watchdog_cycles_param_ + "</param>";
    urdf.replace(pos, anchor.size(), injected);
    return urdf;
  }

  // Convenience: assert every arm + gripper motor holds the expected watchdog tick value.
  void expectArmWatchdogTicks(uint8_t expected_ticks)
  {
    for (uint8_t id : ARM_MOTOR_IDS) {
      auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
      ASSERT_NE(motor, nullptr) << "Motor ID " << static_cast<int>(id) << " not found";
      EXPECT_EQ(motor->getBusWatchdog(), expected_ticks)
          << "Unexpected watchdog ticks for arm motor " << static_cast<int>(id);
    }
    auto gripper = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(GRIPPER_ID);
    ASSERT_NE(gripper, nullptr) << "Gripper motor not found";
    EXPECT_EQ(gripper->getBusWatchdog(), expected_ticks) << "Unexpected watchdog ticks for gripper motor";
  }
};

// (1) A non-default bus_watchdog_cycles must override the default and change the computed ticks.
// cycles = 8 -> ceil(20 * 8 / 20) = 8 ticks, distinct from the default of 4.
TEST_F(BusWatchdogCyclesTest, OverridesDefaultMultiplier)
{
  // bus_watchdog_cycles_param_ defaults to "8" above. Assert the resulting ticks.
  constexpr uint8_t EXPECTED_TICKS = 8;
  expectArmWatchdogTicks(EXPECTED_TICKS);
}

// A large value must be clamped to MAX_TICKS (127) rather than overflowing the register.
class BusWatchdogCyclesClampTest : public BusWatchdogCyclesTest
{
protected:
  void SetUp() override
  {
    bus_watchdog_cycles_param_ = "1000";  // ceil(1000) = 1000, clamped to 127.
    BusWatchdogCyclesTest::SetUp();
  }
};

TEST_F(BusWatchdogCyclesClampTest, ClampsToMaxTicks)
{
  constexpr uint8_t EXPECTED_TICKS = 127;  // DXL_BUS_WATCHDOG_MAX_TICKS
  expectArmWatchdogTicks(EXPECTED_TICKS);
}

// (2) A zero bus_watchdog_cycles disables the watchdog: the register is written as 0, NOT clamped
// up to MIN_TICKS and NOT replaced with the default. This is the documented way to turn the feature
// off.
class BusWatchdogCyclesZeroTest : public BusWatchdogCyclesTest
{
protected:
  void SetUp() override
  {
    bus_watchdog_cycles_param_ = "0";
    BusWatchdogCyclesTest::SetUp();
  }
};

TEST_F(BusWatchdogCyclesZeroTest, ZeroDisablesWatchdog)
{
  constexpr uint8_t DISABLED_TICKS = 0;  // 0 = watchdog disabled on Dynamixel hardware
  expectArmWatchdogTicks(DISABLED_TICKS);
}

class BusWatchdogCyclesHugeTest : public BusWatchdogCyclesTest
{
protected:
  void SetUp() override
  {
    // Finite and positive, so it is a legitimate request. ceil() of it overflows int, so the clamp
    // has to happen in floating point before the narrowing conversion.
    bus_watchdog_cycles_param_ = "1e300";
    BusWatchdogCyclesTest::SetUp();
  }
};

TEST_F(BusWatchdogCyclesHugeTest, HugeValueClampsInsteadOfOverflowing)
{
  expectArmWatchdogTicks(127);
}

// An invalid bus_watchdog_cycles is refused, not replaced by a window nobody configured. Below three
// cycles a window trips on scheduling jitter, nan passes every range check, and "8x" is not 8.
class BusWatchdogCyclesRefusalTest : public DirectHardwareTest
{
protected:
  hardware_interface::CallbackReturn configureWith(const std::string& value)
  {
    hardware_.reset();
    MockDynamixelManager::instance().reset();
    initComponent("athena_arm_interface", [&value](std::string urdf) {
      const std::string anchor = "<param name=\"port_name\">/dev/ttyUSB_manipulator_arm</param>";
      const auto pos = urdf.find(anchor);
      EXPECT_NE(pos, std::string::npos);
      return urdf.insert(pos + anchor.size(), "<param name=\"bus_watchdog_cycles\">" + value + "</param>");
    });
    EXPECT_EQ(hardware_->init(params_), hardware_interface::CallbackReturn::SUCCESS) << value;
    std::ignore = hardware_->on_export_state_interfaces();
    std::ignore = hardware_->on_export_command_interfaces();
    return hardware_->on_configure(rclcpp_lifecycle::State());
  }
};

TEST_F(BusWatchdogCyclesRefusalTest, AnInvalidValueFailsConfiguration)
{
  // The same transform with a valid value, so a refusal below is the value's doing
  EXPECT_EQ(configureWith("4"), hardware_interface::CallbackReturn::SUCCESS);
  for (const std::string value : {"-5", "1", "nan", "inf", "8x"}) {
    EXPECT_EQ(configureWith(value), hardware_interface::CallbackReturn::FAILURE)
        << "bus_watchdog_cycles=" << value << " was accepted";
  }
  hardware_.reset();
}

// ============================================================================
// Watchdog re-arm failures must not be reported as success
// ============================================================================

// Ending a torque_off claim arms the watchdog after the torque-ON. If that fails, the motors are
// live with no comm-loss protection, so rolling back to torque-off is the only honest outcome.
TEST_F(HardwareInterfaceTest, TorqueOnRollsBackWhenTheWatchdogCannotBeRearmed)
{
  std::this_thread::sleep_for(400ms);

  ASSERT_TRUE(limp()) << "Test premise: the arm must start de-energised";
  std::this_thread::sleep_for(300ms);

  // One joint can no longer have its watchdog written, so the arm at the end of the torque change
  // fails while the torque writes themselves all succeed.
  auto faulted = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_3_ID);
  ASSERT_NE(faulted, nullptr);
  faulted->setRegisterWriteFault("bus_watchdog", RegisterWriteFault::TIMEOUT);

  EXPECT_FALSE(releaseLimp()) << "A torque-on whose watchdog could not be re-armed was reported as successful.";
  std::this_thread::sleep_for(300ms);

  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr);
    EXPECT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 0)
        << "Motor " << static_cast<int>(id)
        << " was left energised after a torque-on that could not verify its bus watchdog.";
  }
}

// Activation energises the motors and then arms the watchdog. If that arm fails, activation aborts
// with the motors already live and nothing running to feed them. That is the same hazard the
// torque_off release above rolls back for. Returning ERROR without de-energising leaves a joint whose
// watchdog did arm to trip and freeze a few cycles later, and the one whose arm failed live and
// unprotected.
TEST_F(HardwareInterfaceTest, ActivationRollsBackTorqueWhenTheWatchdogCannotBeRearmed)
{
  std::this_thread::sleep_for(400ms);

  const auto transition = [&](uint8_t state_id, const std::string& label) {
    return setHardwareComponentState("athena_arm_interface", state_id, label);
  };

  ASSERT_TRUE(transition(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive"));
  std::this_thread::sleep_for(300ms);

  // From here the arm at the end of activation cannot be verified on this joint.
  auto faulted = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_3_ID);
  ASSERT_NE(faulted, nullptr);
  faulted->setRegisterWriteFault("bus_watchdog", RegisterWriteFault::TIMEOUT);

  transition(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  std::this_thread::sleep_for(500ms);

  EXPECT_NE(hardwareComponentState("athena_arm_interface"), "active")
      << "Activation succeeded although the bus watchdog could not be armed.";
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr);
    EXPECT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 0)
        << "Motor " << static_cast<int>(id)
        << " was left energised by an activation that aborted because it could not arm the watchdog.";
  }
}

// A single dropped status packet must not be enough to fail an activation whose registers are in
// fact correct: activation judges everything it does by read-back, so one lost answer would
// otherwise abort deterministically on a bus that is merely a little noisy.
//
// The injected faults reach the goal reset, which reads the whole state group (bus_watchdog
// included) and retries five times. They are spent there, well before the watchdog's own
// clear/verify/arm/verify runs. So this pins activation's tolerance as a whole, not that sequence
// in particular: nothing here can place a fault on one of its two read-backs, because the number of
// state-group reads before it is not a property any test should depend on. Its retry is covered by
// construction instead, since armWatchdogVerified() repeats armWatchdogVerifiedOnce().
TEST_F(HardwareInterfaceTest, ActivationSurvivesADroppedStateReadWhileArming)
{
  std::this_thread::sleep_for(400ms);

  const auto transition = [&](uint8_t state_id, const std::string& label) {
    return setHardwareComponentState("athena_arm_interface", state_id, label);
  };

  ASSERT_TRUE(transition(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive"));
  std::this_thread::sleep_for(300ms);

  // Two transactions, which is one lost observation: a group read plus the per-motor probe that
  // follows it. Everything after them sees a healthy bus, so the assertion is that activation
  // absorbed the miss rather than aborting on it.
  auto flaky = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_3_ID);
  ASSERT_NE(flaky, nullptr);
  flaky->setRegisterReadFaultCount("bus_watchdog", 2);

  transition(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  std::this_thread::sleep_for(500ms);

  EXPECT_EQ(hardwareComponentState("athena_arm_interface"), "active")
      << "Activation was aborted by a single dropped state read.";
}

// The watchdog poll is skipped in a cycle whose state read failed. It would be a second round trip
// on a bus that just did not answer the first. This pins the part that matters either way: the poll
// resumes once reads recover, so a trip that latched during the outage is still noticed and cleared.
//
// It does not distinguish whether the failed cycles counted toward the poll interval. That only
// shows on a bus that keeps failing intermittently for longer than the interval, and the mock's
// read faults are a one-shot count rather than a repeating pattern, so the outage here ends well
// inside the observation window either way.
TEST_F(HardwareInterfaceTest, WatchdogPollRecoversALatchAfterAReadOutage)
{
  std::this_thread::sleep_for(400ms);

  auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_3_ID);
  ASSERT_NE(motor, nullptr);

  // Fail enough state reads to cover a full poll interval, but stay well under the consecutive-error
  // threshold that would take the component down.
  motor->setRegisterReadFaultCount("present_position", 12);

  motor->resetBusWatchdogWriteCount();
  const uint64_t writes_before = motor->getBusWatchdogWriteCount();

  // A trip the poll is supposed to notice and clear. Nothing else re-arms it cyclically.
  motor->tripBusWatchdog();
  std::this_thread::sleep_for(1500ms);

  EXPECT_GT(motor->getBusWatchdogWriteCount(), writes_before)
      << "The watchdog poll never ran after the reads recovered, so a latched trip went unnoticed - "
         "the failed cycles must still count toward the poll interval.";
  EXPECT_FALSE(motor->busWatchdogLatched())
      << "The latched trip (-1) was never cleared, so goal writes stay silently rejected.";
  EXPECT_EQ(motor->getBusWatchdog(), 4) << "The cleared watchdog was not armed again.";
}

// The mirror case: a mode switch is strict about the watchdog of a motor that stays energised and
// fails, but the torque off it asked for must still reach every motor.
TEST_F(HardwareInterfaceTest, TorqueOffStillDeEnergisesWhenTheWatchdogCannotBeRearmed)
{
  std::this_thread::sleep_for(400ms);

  auto faulted = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(GRIPPER_ID);
  ASSERT_NE(faulted, nullptr);
  faulted->setRegisterWriteFault("bus_watchdog", RegisterWriteFault::TIMEOUT);

  EXPECT_FALSE(limp("arm_joints_torque_off_controller"))
      << "A mode switch is strict about the watchdog, as activation is.";
  std::this_thread::sleep_for(300ms);

  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr);
    EXPECT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 0);
  }
}

// Only energised motors need comm-loss protection, so a watchdog that cannot be re-armed on a motor
// the switch leaves limp fails nothing
TEST_F(HardwareInterfaceTest, AnUnarmableWatchdogOfALimpMotorDoesNotFailTheTorqueOff)
{
  std::this_thread::sleep_for(400ms);

  auto faulted = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_3_ID);
  ASSERT_NE(faulted, nullptr);
  faulted->setRegisterWriteFault("bus_watchdog", RegisterWriteFault::TIMEOUT);

  EXPECT_TRUE(limp("arm_joints_torque_off_controller"));
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(faulted->read1Byte(faulted->getAddress("torque_enable")), 0);
}

// ============================================================================
// Mixed-bus watchdog monitoring
// ============================================================================
// A Protocol 2.0 sync read needs a uniform register set across its members, so the watchdog has its
// own read group holding only bus_watchdog and only the motors that have it. A bus carrying mixed
// models is monitored through that group.
//
// H42-20-S300-R has every goal and present register the URDF needs but no bus_watchdog, so pointing
// the gripper at it makes the arm bus mixed. These tests check that the watchdog-capable motors are
// still monitored and re-armed, and that the poll costs no register writes while nothing is wrong.
class MixedBusRearmTrafficTest : public HardwareInterfaceTest
{
protected:
  // Model 51200 = H42-20-S300-R: no bus_watchdog register.
  static constexpr const char* WATCHDOGLESS_MODEL = "51200";

  // When non-empty, also injects bus_watchdog_cycles into the arm <hardware> block.
  std::string bus_watchdog_cycles_param_;

  std::string transformUrdf(std::string urdf) override
  {
    // Anchor on the gripper's id param so only that joint's model changes.
    const std::string anchor = "<param name=\"id\">18</param>";
    const auto pos = urdf.find(anchor);
    if (pos == std::string::npos) {
      ADD_FAILURE() << "Could not find gripper id anchor to inject model_number";
      return urdf;
    }
    const std::string injected =
        anchor + "\n      <param name=\"model_number\">" + std::string(WATCHDOGLESS_MODEL) + "</param>";
    urdf.replace(pos, anchor.size(), injected);

    if (!bus_watchdog_cycles_param_.empty()) {
      const std::string hw_anchor = "<param name=\"port_name\">/dev/ttyUSB_manipulator_arm</param>";
      const auto hw_pos = urdf.find(hw_anchor);
      if (hw_pos == std::string::npos) {
        ADD_FAILURE() << "Could not find arm interface anchor to inject bus_watchdog_cycles";
        return urdf;
      }
      const std::string hw_injected =
          hw_anchor + "\n      <param name=\"bus_watchdog_cycles\">" + bus_watchdog_cycles_param_ + "</param>";
      urdf.replace(hw_pos, hw_anchor.size(), hw_injected);
    }
    return urdf;
  }

  static uint64_t totalArmWatchdogWrites()
  {
    uint64_t total = 0;
    for (uint8_t id : ARM_MOTOR_IDS) {
      auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
      if (motor)
        total += motor->getBusWatchdogWriteCount();
    }
    return total;
  }

  static void resetArmWatchdogWrites()
  {
    for (uint8_t id : ARM_MOTOR_IDS) {
      auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
      if (motor)
        motor->resetBusWatchdogWriteCount();
    }
  }
};

// Sanity: the injected model really produced a mixed bus, otherwise the traffic test below would
// measure a code path that does not run.
TEST_F(MixedBusRearmTrafficTest, GripperHasNoWatchdogRegister)
{
  auto gripper = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(GRIPPER_ID);
  ASSERT_NE(gripper, nullptr);
  EXPECT_EQ(gripper->getAddress("bus_watchdog"), 0) << "Gripper should have no bus_watchdog register";

  auto arm = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(arm, nullptr);
  EXPECT_NE(arm->getAddress("bus_watchdog"), 0) << "Arm motors should still have the register";

  // The traffic test below is only meaningful if activation actually completed and armed the
  // watchdog. Otherwise a zero measurement would be vacuous.
  EXPECT_EQ(arm->getBusWatchdog(), 4) << "Arm watchdog not armed - interface did not reach ACTIVE";
  EXPECT_EQ(arm->read1Byte(arm->getAddress("torque_enable")), 1) << "Torque not on";
}

// The periodic poll reads the watchdog register and only writes it when it finds a latch. Nothing
// is latched here, so a healthy bus must cost zero watchdog writes. A poll that re-armed on every
// tick would be indistinguishable in the register's end state and would flood a shared bus.
TEST_F(MixedBusRearmTrafficTest, NoRearmTrafficDuringHealthyOperation)
{
  std::this_thread::sleep_for(500ms);  // let the loop settle after activation
  resetArmWatchdogWrites();
  std::this_thread::sleep_for(1s);  // ~50 write cycles at 50 Hz

  EXPECT_EQ(totalArmWatchdogWrites(), 0u)
      << "The watchdog poll wrote the register during healthy operation; it should only read.";
}

// A torque_off switch holds the comm mutex across reads and retries that can outlast a watchdog
// window, so the mode switch suspends the watchdog for the whole change and re-arms it while ACTIVE.
// Without that, a torque cycle leaves the arm frozen behind a latched or disarmed watchdog.
TEST_F(MixedBusRearmTrafficTest, TorqueOffCycleLeavesWatchdogArmed)
{
  std::this_thread::sleep_for(300ms);

  ASSERT_TRUE(limp()) << "Torque-off on a healthy (if mixed) bus should succeed";
  std::this_thread::sleep_for(150ms);
  ASSERT_TRUE(releaseLimp()) << "Torque-on should succeed and re-arm the watchdog";
  std::this_thread::sleep_for(150ms);

  // The arm motors (which have the register) must hold the configured value: not 0 left by the
  // suspension, and not -1 left by a trip during the change.
  constexpr uint8_t EXPECTED_WATCHDOG_TICKS = 4;
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr);
    EXPECT_EQ(motor->getBusWatchdog(), EXPECTED_WATCHDOG_TICKS)
        << "Watchdog not correctly armed after a torque_off cycle for motor " << static_cast<int>(id);
  }
}

// ============================================================================
// Latch recovery
// ============================================================================
//
// A trip latches the register at -1 and the motor then silently rejects every goal write until
// something writes 0. Packets are still acknowledged, so nothing in the protocol reveals it.
// Detection has to be an observation, so the watchdog has its own read group covering exactly the
// motors that have the register.
//
// The mock latches on demand and enforces the goal-write rejection, so both halves ("the latch is
// cleared" and "goal writes land again") are real assertions.

// Uniform bus: every arm motor has the register.
TEST_F(HardwareInterfaceTest, LatchedWatchdogIsClearedAndGoalWritesResume)
{
  std::this_thread::sleep_for(400ms);

  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));
  std::this_thread::sleep_for(300ms);

  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor, nullptr);
  ASSERT_EQ(motor->getBusWatchdog(), 4) << "Test premise: the watchdog should be armed";

  const double start_position = motor->getJointPosition();

  // Latch it, exactly as a stall past the window would.
  motor->tripBusWatchdog();
  ASSERT_TRUE(motor->busWatchdogLatched()) << "Test premise: the mock should now be latched";

  // Recovery must happen on its own, within a poll interval or two.
  bool cleared = false;
  for (int i = 0; i < 40 && !cleared; ++i) {
    std::this_thread::sleep_for(50ms);
    cleared = !motor->busWatchdogLatched();
  }
  EXPECT_TRUE(cleared) << "The latched bus watchdog was never cleared. Goal writes stay silently "
                          "rejected and the joint is frozen with no error anywhere.";
  EXPECT_EQ(motor->getBusWatchdog(), 4) << "The watchdog was cleared but not re-armed, leaving the motor "
                                           "without comm-loss protection.";

  // Clearing the latch has to make the motor accept commands again.
  auto cmd_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);
  ASSERT_TRUE(waitForSubscribers(cmd_pub, 5s));
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = std::vector<double>(ARM_MOTOR_IDS.size(), start_position + 0.4);
  cmd_pub->publish(cmd);
  std::this_thread::sleep_for(1200ms);

  EXPECT_GT(std::abs(motor->getJointPosition() - start_position), 0.05)
      << "Goal writes are still being rejected after the watchdog recovery.";
}

// The same on a mixed bus, where one motor has no watchdog register.
TEST_F(MixedBusRearmTrafficTest, LatchedWatchdogIsClearedOnAMixedBus)
{
  std::this_thread::sleep_for(400ms);

  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor, nullptr);
  ASSERT_EQ(motor->getBusWatchdog(), 4) << "Test premise: the watchdog should be armed";
  auto gripper = MockDynamixelManager::instance().getMotor(GRIPPER_ID);
  ASSERT_NE(gripper, nullptr);
  ASSERT_EQ(gripper->getAddress("bus_watchdog"), 0) << "Test premise: the bus must be mixed";

  motor->tripBusWatchdog();
  ASSERT_TRUE(motor->busWatchdogLatched());

  bool cleared = false;
  for (int i = 0; i < 40 && !cleared; ++i) {
    std::this_thread::sleep_for(50ms);
    cleared = !motor->busWatchdogLatched();
  }
  EXPECT_TRUE(cleared) << "A latched watchdog on a mixed bus was never cleared. One watchdog-less motor "
                          "must not disable trip detection for every other motor on the bus.";
  EXPECT_EQ(motor->getBusWatchdog(), 4) << "Cleared but not re-armed";
}

// A control-mode switch produces a stall long enough to trip the watchdog. It holds the comm mutex
// across several goal reset+verify rounds while read()/write() skip. Whether the suspension
// prevents the trip or the poll recovers it, the joint must not be left latched.
TEST_F(HardwareInterfaceTest, ModeSwitchDoesNotLeaveAWatchdogLatched)
{
  std::this_thread::sleep_for(400ms);

  ASSERT_TRUE(loadAndActivateController("arm_velocity_controller"));
  std::this_thread::sleep_for(300ms);
  ASSERT_TRUE(loadController("arm_position_controller"));
  ASSERT_TRUE(switch_controllers({"arm_position_controller"}, {"arm_velocity_controller"}));
  std::this_thread::sleep_for(600ms);

  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr);
    EXPECT_FALSE(motor->busWatchdogLatched())
        << "Motor " << static_cast<int>(id) << " was left with a latched watchdog after a control mode switch.";
    EXPECT_EQ(motor->getBusWatchdog(), 4)
        << "Motor " << static_cast<int>(id) << " was left disarmed after a control mode switch.";
  }
}

// The other half of the same rule: while goals ARE flowing, a trip is unexpected and must still be
// cleared and re-armed. Moving the poll into write() must not cost that recovery.
TEST_F(HardwareInterfaceTest, WatchdogStillRecoversWhileGoalsAreFlowing)
{
  std::this_thread::sleep_for(400ms);

  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));
  std::this_thread::sleep_for(300ms);

  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor, nullptr);
  ASSERT_EQ(motor->getBusWatchdog(), 4) << "Test premise: the watchdog should be armed";

  motor->tripBusWatchdog();
  ASSERT_TRUE(motor->busWatchdogLatched());

  bool cleared = false;
  for (int i = 0; i < 40 && !cleared; ++i) {
    std::this_thread::sleep_for(50ms);
    cleared = !motor->busWatchdogLatched();
  }
  EXPECT_TRUE(cleared) << "A trip taken while the loop was delivering goals was never cleared, so goal "
                          "writes stay silently rejected.";
  EXPECT_EQ(motor->getBusWatchdog(), 4) << "Cleared but not re-armed.";
}

// The other way an armed watchdog stops being armed, and the one no latch marks: a clear and an arm
// are two separate statusless sync writes, so a lost arm packet leaves the register at 0 with the
// interface believing it protected. That reads as "disarmed", not as "tripped", so a recovery keyed
// on the latch alone does not notice it. The motors stay torqued with nothing to stop them if the
// control loop dies. Injected as the state such a loss leaves behind.
TEST_F(HardwareInterfaceTest, WatchdogIsReArmedAfterAnArmThatNeverLanded)
{
  std::this_thread::sleep_for(400ms);

  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));
  std::this_thread::sleep_for(300ms);

  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor, nullptr);
  ASSERT_EQ(motor->getBusWatchdog(), 4) << "Test premise: the watchdog should be armed";

  motor->write1Byte(motor->getAddress("bus_watchdog"), 0);
  ASSERT_EQ(motor->getBusWatchdog(), 0) << "Test premise: the window should now be stopped";
  ASSERT_FALSE(motor->busWatchdogLatched()) << "Test premise: a lost arm leaves no latch to find";

  bool re_armed = false;
  for (int i = 0; i < 40 && !re_armed; ++i) {
    std::this_thread::sleep_for(50ms);
    re_armed = motor->getBusWatchdog() == 4;
  }
  EXPECT_TRUE(re_armed) << "The watchdog was left disarmed while the interface kept delivering goals as if it "
                           "were protected. Nothing latches in this state, so only comparing the register "
                           "against what was last armed can catch it.";
}

}  // namespace dynamixel_ros_control::test
