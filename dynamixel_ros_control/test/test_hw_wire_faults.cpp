// Faults of the wire and of bus power, seen by the plugin through the real SDK

#include "wire_bus.hpp"

namespace dynamixel_ros_control::test {

using WireFaultTest = WireBusTest;

TEST_F(WireFaultTest, CorruptStatusesFaultTheComponentAtTheThreshold)
{
  startComponent("athena_arm_interface");
  server_->injectWireFault(WireFault::CORRUPT, PERSISTENT_WIRE_FAULT);
  for (unsigned int i = 1; i < COMM_ERROR_THRESHOLD; ++i) {
    ASSERT_EQ(cycle(), hardware_interface::return_type::OK) << "cycle " << i << " is below the threshold";
  }
  EXPECT_EQ(cycle(), hardware_interface::return_type::ERROR);
}

// Each burst stays below the threshold, together they exceed it
TEST_F(WireFaultTest, SeparateBurstsOfDroppedStatusesAreRiddenOut)
{
  startComponent("athena_arm_interface");
  for (int burst = 0; burst < 2; ++burst) {
    server_->injectWireFault(WireFault::DROP, PERSISTENT_WIRE_FAULT);
    cycles(15);
    server_->clearWireFault();
    cycles(5);
  }
}

// A late status answers the next read of its motor, so the plugin takes a reading one cycle old
TEST_F(WireFaultTest, ALateStatusIsTakenForTheNextReading)
{
  startComponent("athena_arm_interface");
  const double before = state("arm_joint_1/position");
  // Every status of one sync read, seven arm joints and the gripper
  server_->injectWireFault(WireFault::DELAY, 8);
  ASSERT_EQ(cycle(), hardware_interface::return_type::OK);
  MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID)->moveByHand(0.5);
  ASSERT_EQ(cycle(), hardware_interface::return_type::OK);
  EXPECT_NEAR(state("arm_joint_1/position"), before, 1e-3) << "the late status was not taken for the reading";
  ASSERT_EQ(cycle(), hardware_interface::return_type::OK);
  EXPECT_GT(std::abs(state("arm_joint_1/position") - before), 0.3) << "the reading stayed stale";
}

// The USB adapter goes away. The port refuses every packet, and on_configure fails until it is back.
class UnplugTest : public WireFaultTest
{
protected:
  void SetUp() override
  {
    WireFaultTest::SetUp();
    char dir[] = "/tmp/dxl_unplug_XXXXXX";
    link_dir_ = mkdtemp(dir);
    startComponent("athena_arm_interface");
  }

  void TearDown() override
  {
    WireFaultTest::TearDown();
    server_.reset();
    std::filesystem::remove_all(link_dir_);
  }

  void adjustInfo(hardware_interface::HardwareInfo& info) override
  {
    server_ = serveOverWire(info, linkPath());
  }

  std::string linkPath() const
  {
    return link_dir_ + "/ttyUSB_arm";
  }

  std::string link_dir_;
};

TEST_F(UnplugTest, AReplugRecoversThroughOnError)
{
  server_.reset();
  int cycles_until_fault = 1;
  while (cycle() == hardware_interface::return_type::OK) {
    ASSERT_LE(++cycles_until_fault, static_cast<int>(COMM_ERROR_THRESHOLD) + 1) << "the lost port was never reported";
  }
  EXPECT_EQ(hardware_->on_error(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(hardware_->on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::FAILURE)
      << "configured without a port";

  std::vector<uint8_t> ids = ARM_MOTOR_IDS;
  ids.push_back(GRIPPER_ID);
  server_ = std::make_unique<MockBusServer>(ids, linkPath());
  ASSERT_EQ(hardware_->on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware_->on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  cycles(10);
  for (const uint8_t id : ids) {
    const auto motor = MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 1U) << "motor " << static_cast<int>(id);
  }
}

// A hardware error selected in the Shutdown register cuts torque. Recovery is the reboot service
// while unconfigured, then activation, all of it over the wire.
class WireShutdownTest : public HardwareInterfaceTest
{
protected:
  void SetUp() override
  {
    std::vector<uint8_t> ids = ARM_MOTOR_IDS;
    ids.push_back(GRIPPER_ID);
    for (const uint8_t id : ids) {
      MockDynamixelManager::instance().addMotor(id, DEFAULT_MOCK_MODEL_NUMBER);
    }
    server_ = std::make_unique<MockBusServer>(ids);
    HardwareInterfaceTest::SetUp();
  }

  std::string transformUrdf(std::string urdf) override
  {
    const auto begin = urdf.find("<ros2_control name=\"athena_arm_interface\"");
    const auto end = urdf.find("</ros2_control>", begin);
    std::string arm = urdf.substr(begin, end - begin);
    for (const auto& [from, to] : {std::pair<std::string, std::string>{"<param name=\"use_dummy\">true</param>",
                                                                       "<param name=\"use_dummy\">false</param>"},
                                   {"<param name=\"port_name\">/dev/ttyUSB_manipulator_arm</param>",
                                    "<param name=\"port_name\">" + server_->portName() + "</param>"}}) {
      const auto pos = arm.find(from);
      EXPECT_NE(pos, std::string::npos) << from;
      arm.replace(pos, from.size(), to);
    }
    return urdf.replace(begin, end - begin, arm);
  }

  std::unique_ptr<MockBusServer> server_;
};

TEST_F(WireShutdownTest, AShutdownIsRecoveredByARebootOverTheWire)
{
  ASSERT_TRUE(waitForHardwareInterfacesActive(10s));
  const auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  const uint16_t torque_enable = motor->getAddress("torque_enable");
  motor->setHardwareError(ERROR_OVERLOAD);
  ASSERT_EQ(motor->read1Byte(torque_enable), 0U) << "test premise: overload is in the Shutdown register";

  ASSERT_TRUE(waitForComponentState("athena_arm_interface", "unconfigured"));

  const auto response = rebootMotors();
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success) << response->message;
  EXPECT_GT(motor->getRebootCount(), 0);
  EXPECT_EQ(motor->getHardwareError(), 0U);

  ASSERT_TRUE(
      setHardwareComponentState("athena_arm_interface", lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active"));
  EXPECT_EQ(hardwareComponentState("athena_arm_interface"), "active");
  EXPECT_EQ(motor->read1Byte(torque_enable), 1U) << "torque did not come back after the reboot";
}

// athena's hard e-stop cuts power to every actuator. They come back limp with their goals reset, and
// the plugin must neither drive them on nor jump to a zeroed goal. PRO H54 has no bus watchdog.
class BusPowerCutTest : public WireFaultTest
{
protected:
  void SetUp() override
  {
    WireFaultTest::SetUp();
    startComponent("athena_flipper_interface");
    for (const uint8_t id : FLIPPER_MOTOR_IDS) {
      motors_.push_back(MockDynamixelManager::instance().getMotor(id));
    }
  }

  void adjustInfo(hardware_interface::HardwareInfo& info) override
  {
    for (auto& joint : info.joints) {
      joint.parameters["model_number"] = "54024";  // athena's PRO H54
    }
    WireFaultTest::adjustInfo(info);
    // Away from zero, where a goal reset by the power cut would pull them
    double offset = 0.2;
    for (const uint8_t id : FLIPPER_MOTOR_IDS) {
      MockDynamixelManager::instance().getMotor(id)->moveByHand(offset);
      offset += 0.2;
    }
  }

  /// Configures and activates again, as hcm does once on_error is through
  void recover()
  {
    for (const auto& motor : motors_) {
      motor->startWriteTrace();
    }
    ASSERT_EQ(hardware_->on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(hardware_->on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
    cycles(25);
  }

  void expectHoldingWhereTheyWere(const std::vector<double>& positions)
  {
    for (size_t i = 0; i < motors_.size(); ++i) {
      EXPECT_EQ(motors_[i]->read1Byte(motors_[i]->getAddress("torque_enable")), 1U) << "motor " << i;
      // The goal register came back zeroed, and a motor energised on it runs there
      EXPECT_LT(motors_[i]->firstWriteIndex("goal_position"), motors_[i]->firstWriteIndex("torque_enable", 1))
          << "motor " << i << " was energised before its goal was reset";
      EXPECT_NEAR(motors_[i]->getCurrentPosition(), positions[i], 0.01) << "motor " << i << " moved";
    }
  }

  std::vector<double> positions() const
  {
    std::vector<double> result;
    for (const auto& motor : motors_) {
      result.push_back(motor->getCurrentPosition());
    }
    return result;
  }

  std::vector<std::shared_ptr<MockDynamixel>> motors_;
};

TEST_F(BusPowerCutTest, ALongCutFaultsTheComponentAndItRecoversHolding)
{
  const auto before = positions();
  server_->setPower(false);
  unsigned int cycles_until_fault = 1;
  while (cycle() == hardware_interface::return_type::OK) {
    ASSERT_LT(++cycles_until_fault, COMM_ERROR_THRESHOLD + 1) << "the dead bus was never reported";
  }
  EXPECT_EQ(hardware_->on_error(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  server_->setPower(true);
  ASSERT_EQ(motors_[0]->read1Byte(motors_[0]->getAddress("torque_enable")), 0U);
  recover();
  expectHoldingWhereTheyWere(before);
}

TEST_F(BusPowerCutTest, AShortCutIsCaughtByTheLostTorqueAndRecoversHolding)
{
  const auto before = positions();
  server_->setPower(false);
  cycles(3);
  server_->setPower(true);
  EXPECT_EQ(cycle(), hardware_interface::return_type::ERROR) << "motors that rebooted were driven on";
  EXPECT_EQ(hardware_->on_error(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  recover();
  expectHoldingWhereTheyWere(before);
}

}  // namespace dynamixel_ros_control::test
