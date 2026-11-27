#include "direct_hardware_fixture.hpp"

namespace dynamixel_ros_control::test {

// The bus watchdog with the mock's timing model on, in simulated time, so a trip is a real silence
// and never scheduling jitter. Reads do not feed it here, the strictest reading of the e-manual. Each
// test covers a way the watchdog used to trip on a healthy robot.
class BusWatchdogTimingTest : public DirectHardwareTest
{
protected:
  void SetUp() override
  {
    DirectHardwareTest::SetUp();
    MockDynamixelManager::instance().setBusWatchdogTiming(true, false);
  }

  /// Simulated time passing without a single packet, as while the control loop stalls.
  static void silence(int cycle_count)
  {
    for (int i = 0; i < cycle_count; ++i) {
      MockDynamixelManager::instance().update(kPeriod);
    }
  }

  static bool anyLatched()
  {
    for (const auto id : MockDynamixelManager::instance().getConnectedIds()) {
      if (MockDynamixelManager::instance().getMotor(id)->busWatchdogLatched()) {
        return true;
      }
    }
    return false;
  }

  /// Whether any motor tripped, also if the driver has repaired it since.
  static bool anyTripped()
  {
    for (const auto id : MockDynamixelManager::instance().getConnectedIds()) {
      if (MockDynamixelManager::instance().getMotor(id)->timedBusWatchdogTrips() > 0) {
        return true;
      }
    }
    return false;
  }
};

TEST_F(BusWatchdogTimingTest, SteadyControlNeverTrips)
{
  startComponent("athena_flipper_interface");
  startInterface("flipper_fl_joint/velocity", 0.5);
  cycles(500);
  EXPECT_FALSE(anyTripped());
}

TEST_F(BusWatchdogTimingTest, SkippedCyclesShorterThanTheWindowNeverTrip)
{
  startComponent("athena_flipper_interface");
  cycles(10);
  silence(3);  // the window is four cycles
  cycles(10);
  EXPECT_FALSE(anyTripped());
}

// A trip stops the motor and latches its goal registers. The control loop must clear and re-arm it,
// or every later goal is silently discarded.
TEST_F(BusWatchdogTimingTest, AStalledLoopTripsAndTheNextCyclesRepairIt)
{
  startComponent("athena_flipper_interface");
  cycles(10);
  silence(6);
  ASSERT_TRUE(anyLatched()) << "precondition: six silent cycles exceed the window";
  cycles(3);
  EXPECT_FALSE(anyLatched());
  EXPECT_TRUE(MockDynamixelManager::instance()
                  .getMotor(FLIPPER_FL_ID)
                  ->read1Byte(MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID)->getAddress("bus_watchdog")) > 0)
      << "the watchdog must be armed again, not just cleared";
}

// A mode switch blocks the control loop of its component for as long as it runs, which with
// verification and retries can exceed the window. The switch clears the watchdog first.
TEST_F(BusWatchdogTimingTest, AModeSwitchLongerThanTheWindowNeverTrips)
{
  startComponent("athena_flipper_interface");
  cycles(10);
  auto motor = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  bool stalled = false;
  motor->setRegisterReadHook("operating_mode", [&stalled]() {
    if (!stalled) {
      stalled = true;
      silence(10);
    }
  });
  ASSERT_EQ(hardware_->perform_command_mode_switch({"flipper_fl_joint/velocity"}, {}),
            hardware_interface::return_type::OK);
  motor->clearRegisterReadHooks();
  ASSERT_TRUE(stalled) << "precondition: the switch must have read the operating mode";
  cycles(10);
  EXPECT_FALSE(anyTripped());
}

// The switch disarms the watchdog for its verify rounds. A bus lost right after that must not
// leave a velocity joint turning with nothing left to stop it.
TEST_F(BusWatchdogTimingTest, ABusLostDuringAStoppingSwitchStillStopsAVelocityJoint)
{
  startComponent("athena_flipper_interface");
  startInterface("flipper_fl_joint/velocity", 0.5);
  cycles(20);
  const auto motor = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  ASSERT_GT(std::abs(motor->getCurrentVelocity()), 0.05) << "precondition: the joint turns";

  MockDynamixelManager::instance().cutBusAfterPackets(1);
  EXPECT_EQ(hardware_->perform_command_mode_switch({}, {"flipper_fl_joint/velocity"}),
            hardware_interface::return_type::ERROR);
  silence(20);
  EXPECT_LT(std::abs(motor->getCurrentVelocity()), 0.01) << "the joint kept turning after the bus was lost";
}

// A joint going limp gets no goal before its torque off, so the torque off itself has to come
// before the clear
TEST_F(BusWatchdogTimingTest, ABusLostWhileAVelocityJointGoesLimpStillStopsIt)
{
  startComponent("athena_flipper_interface");
  startInterface("flipper_fl_joint/velocity", 0.5);
  cycles(20);
  const auto motor = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  ASSERT_GT(std::abs(motor->getCurrentVelocity()), 0.05) << "precondition: the joint turns";

  MockDynamixelManager::instance().cutBusAfterPackets(1);
  EXPECT_EQ(hardware_->perform_command_mode_switch({"flipper_fl_joint/torque_off"}, {"flipper_fl_joint/velocity"}),
            hardware_interface::return_type::ERROR);
  silence(20);
  EXPECT_LT(std::abs(motor->getCurrentVelocity()), 0.01) << "the joint kept turning after the bus was lost";
}

// Every packet of a verified torque off goes to the joint going limp, so a slow one ahead of the
// clear would starve the other running joints of their watchdog feed
TEST_F(BusWatchdogTimingTest, ASlowTorqueOffDoesNotTripAnotherRunningJoint)
{
  startComponent("athena_flipper_interface");
  startInterface("flipper_fl_joint/velocity", 0.5);
  startInterface("flipper_fr_joint/velocity", 0.5);
  cycles(20);
  const auto motor = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  const auto other = MockDynamixelManager::instance().getMotor(FLIPPER_FR_ID);
  bool stalled = false;
  motor->setRegisterReadHook("torque_enable", [&stalled]() {
    if (!stalled) {
      stalled = true;
      silence(10);
    }
  });
  EXPECT_EQ(hardware_->perform_command_mode_switch({"flipper_fl_joint/torque_off"}, {"flipper_fl_joint/velocity"}),
            hardware_interface::return_type::OK);
  motor->clearRegisterReadHooks();
  ASSERT_TRUE(stalled) << "precondition: the switch must have read the torque back";
  EXPECT_EQ(other->timedBusWatchdogTrips(), 0);
}

// A joint that did not go limp may still turn, and write() sends nothing after a failed switch, so
// the watchdog must be armed to stop it
TEST_F(BusWatchdogTimingTest, AFailedTorqueOffLeavesTheWatchdogArmed)
{
  startComponent("athena_flipper_interface");
  startInterface("flipper_fl_joint/velocity", 0.5);
  cycles(20);
  const auto motor = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  ASSERT_GT(std::abs(motor->getCurrentVelocity()), 0.05) << "precondition: the joint turns";

  motor->setRegisterWriteFault("torque_enable", RegisterWriteFault::DROP);
  EXPECT_EQ(hardware_->perform_command_mode_switch({"flipper_fl_joint/torque_off"}, {"flipper_fl_joint/velocity"}),
            hardware_interface::return_type::ERROR);
  motor->clearRegisterWriteFaults();
  EXPECT_GT(motor->getBusWatchdog(), 0);
  EXPECT_EQ(cycle(), hardware_interface::return_type::ERROR);
  silence(20);
  EXPECT_LT(std::abs(motor->getCurrentVelocity()), 0.01) << "the joint kept turning after the failed switch";
}

// Nothing feeds a watchdog on an inactive component, so a failed switch there must not arm it
TEST_F(BusWatchdogTimingTest, AFailedSwitchOnAnInactiveComponentLeavesTheWatchdogDisarmed)
{
  startComponent("athena_flipper_interface");
  ASSERT_EQ(hardware_->on_deactivate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  const auto motor = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  ASSERT_EQ(motor->getBusWatchdog(), 0) << "precondition: deactivation disarms";

  motor->setRegisterWriteFault("torque_enable", RegisterWriteFault::DROP);
  EXPECT_EQ(hardware_->perform_command_mode_switch({"flipper_fl_joint/torque_off"}, {}),
            hardware_interface::return_type::ERROR);
  motor->clearRegisterWriteFaults();
  EXPECT_EQ(motor->getBusWatchdog(), 0) << "an inactive component got an armed watchdog nothing feeds";
}

// Any failure after the clear leaves write() silent, so the watchdog must protect the running joints again
TEST_F(BusWatchdogTimingTest, AFailedControlModeWriteLeavesTheWatchdogArmed)
{
  startComponent("athena_flipper_interface");
  startInterface("flipper_fr_joint/velocity", 0.5);
  cycles(20);
  const auto motor = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  const auto running = MockDynamixelManager::instance().getMotor(FLIPPER_FR_ID);
  ASSERT_GT(std::abs(running->getCurrentVelocity()), 0.05) << "precondition: the joint turns";

  motor->setRegisterWriteFault("operating_mode", RegisterWriteFault::TIMEOUT);
  EXPECT_EQ(hardware_->perform_command_mode_switch({"flipper_fl_joint/velocity"}, {}),
            hardware_interface::return_type::ERROR);
  motor->clearRegisterWriteFaults();
  EXPECT_GT(running->getBusWatchdog(), 0) << "the failed switch left the watchdog cleared";
  silence(20);
  EXPECT_LT(std::abs(running->getCurrentVelocity()), 0.01) << "a joint kept turning after the failed switch";
}

// The early stop goes out before the torque off, so the one packet that still reaches the bus stops
// the joint that would otherwise keep its speed
TEST_F(BusWatchdogTimingTest, TheEarlyStopIsTheFirstPacketOfASwitch)
{
  startComponent("athena_flipper_interface");
  startInterface("flipper_fl_joint/velocity", 0.5);
  startInterface("flipper_fr_joint/velocity", 0.5);
  cycles(20);
  const auto stopped = MockDynamixelManager::instance().getMotor(FLIPPER_FR_ID);
  ASSERT_NE(stopped->read4Byte(stopped->getAddress("goal_velocity")), 0U) << "precondition: the joint turns";

  MockDynamixelManager::instance().cutBusAfterPackets(1);
  EXPECT_EQ(hardware_->perform_command_mode_switch({"flipper_fl_joint/torque_off"},
                                                   {"flipper_fl_joint/velocity", "flipper_fr_joint/velocity"}),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(stopped->read4Byte(stopped->getAddress("goal_velocity")), 0U);
}

// A lost early stop costs nothing but the head start, the reset of the switch still stops the joint
TEST_F(BusWatchdogTimingTest, ALostEarlyStopDoesNotFailTheSwitch)
{
  startComponent("athena_flipper_interface");
  startInterface("flipper_fl_joint/velocity", 0.5);
  cycles(20);
  const auto motor = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  ASSERT_GT(std::abs(motor->getCurrentVelocity()), 0.05) << "precondition: the joint turns";

  MockDynamixelManager::instance().failNextSyncWriteTransmissions(1);
  ASSERT_EQ(hardware_->perform_command_mode_switch({}, {"flipper_fl_joint/velocity"}),
            hardware_interface::return_type::OK);
  cycles(20);
  EXPECT_LT(std::abs(motor->getCurrentVelocity()), 0.01);
  EXPECT_FALSE(anyTripped());
}

// The early stop is for the joints the switch hands over, a joint whose controller keeps running is left alone
TEST_F(BusWatchdogTimingTest, AStoppingSwitchHoldsTheJointRearmsAndLeavesOthersRunning)
{
  startComponent("athena_flipper_interface");
  startInterface("flipper_fl_joint/velocity", 0.5);
  startInterface("flipper_fr_joint/velocity", 0.5);
  cycles(20);
  const auto stopped = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  const auto running = MockDynamixelManager::instance().getMotor(FLIPPER_FR_ID);
  ASSERT_GT(std::abs(stopped->getCurrentVelocity()), 0.05) << "precondition: the joint turns";

  ASSERT_EQ(hardware_->perform_command_mode_switch({}, {"flipper_fl_joint/velocity"}),
            hardware_interface::return_type::OK);
  EXPECT_NE(running->read4Byte(running->getAddress("goal_velocity")), 0U) << "the switch stopped an untouched joint";
  cycles(20);
  const double held_at = stopped->getJointPosition();
  cycles(20);
  EXPECT_LT(std::abs(stopped->getCurrentVelocity()), 0.01);
  EXPECT_NEAR(stopped->getJointPosition(), held_at, 1e-3);
  EXPECT_GT(std::abs(running->getCurrentVelocity()), 0.05);
  EXPECT_GT(stopped->getBusWatchdog(), 0) << "the switch left the watchdog disarmed";
  EXPECT_FALSE(anyTripped());
}

// A deactivated component is still read but never written, and its motors may keep their torque.
TEST_F(BusWatchdogTimingTest, ADeactivatedComponentNeverTrips)
{
  startComponent("athena_flipper_interface");
  cycles(10);
  ASSERT_EQ(hardware_->on_deactivate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(kPeriod);
  for (int i = 0; i < 50; ++i) {
    time_ += period;
    ASSERT_EQ(hardware_->read(time_, period), hardware_interface::return_type::OK);
    MockDynamixelManager::instance().update(kPeriod);
  }
  EXPECT_FALSE(anyTripped());
}

// A stop that never reached a free running joint leaves the watchdog armed, the only thing left
// that halts it
TEST_F(BusWatchdogTimingTest, DeactivationKeepsTheWatchdogArmedWhenTheStopIsLost)
{
  startComponent("athena_flipper_interface");
  startInterface("flipper_fl_joint/velocity", 0.5);
  cycles(20);
  const auto motor = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  ASSERT_GT(std::abs(motor->getCurrentVelocity()), 0.05) << "precondition: the joint turns";

  MockDynamixelManager::instance().failNextSyncWriteTransmissions(1);
  ASSERT_EQ(hardware_->on_deactivate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  silence(20);
  EXPECT_LT(std::abs(motor->getCurrentVelocity()), 0.01) << "the joint kept turning after its stop was lost";
}

// Configuration ends long before the first cycle, for example while a second component configures.
TEST_F(BusWatchdogTimingTest, ConfigurationLeavesTheWatchdogDisarmed)
{
  initComponent("athena_flipper_interface");
  ASSERT_EQ(hardware_->init(params_), hardware_interface::CallbackReturn::SUCCESS);
  hardware_->on_export_state_interfaces();
  hardware_->on_export_command_interfaces();
  ASSERT_EQ(hardware_->on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  configured_ = true;
  // Torqued motors, as after a warm start or a torque request, so an armed window would run.
  for (const auto id : MockDynamixelManager::instance().getConnectedIds()) {
    auto motor = MockDynamixelManager::instance().getMotor(id);
    motor->write1Byte(motor->getAddress("torque_enable"), 1);
  }
  silence(50);
  ASSERT_EQ(hardware_->on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  cycles(10);
  EXPECT_FALSE(anyTripped());
}

}  // namespace dynamixel_ros_control::test
