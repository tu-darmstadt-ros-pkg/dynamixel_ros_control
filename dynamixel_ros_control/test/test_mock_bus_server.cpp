// The bus server against the real SDK and PortHandlerLinux, over a pty

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iterator>
#include <memory>
#include <thread>

#include <dynamixel_ros_control/mock_bus_server.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>

namespace dynamixel_ros_control {
namespace {

constexpr uint16_t PH54 = 2020;
constexpr uint16_t PRO_H54 = 54024;

MockDynamixelManager& manager()
{
  return MockDynamixelManager::instance();
}

class MockBusServerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    manager().addMotor(1, PH54);
    manager().addMotor(2, PH54);
    manager().addMotor(4, PRO_H54);
    manager().addMotor(3, PH54);  // on another bus
    server_ = std::make_unique<MockBusServer>(std::vector<uint8_t>{1, 2, 4});
    port_ = dynamixel::PortHandler::getPortHandler(server_->portName().c_str());
    ASSERT_TRUE(port_->openPort());
    ASSERT_TRUE(port_->setBaudRate(1000000));
    goal_position_ = manager().getMotor(1)->getAddress("goal_position");
  }

  void TearDown() override
  {
    port_->closePort();
    delete port_;
    server_.reset();
    manager().reset();
  }

  std::unique_ptr<MockBusServer> server_;
  dynamixel::PortHandler* port_{nullptr};
  dynamixel::PacketHandler* packet_{dynamixel::PacketHandler::getPacketHandler(2.0)};
  uint16_t goal_position_{0};
  uint8_t error_{0};
};

TEST_F(MockBusServerTest, APingAnswersWithTheModelNumber)
{
  uint16_t model_number = 0;
  ASSERT_EQ(packet_->ping(port_, 1, &model_number, &error_), COMM_SUCCESS);
  EXPECT_EQ(model_number, PH54);
  ASSERT_EQ(packet_->ping(port_, 4, &model_number, &error_), COMM_SUCCESS);
  EXPECT_EQ(model_number, PRO_H54);
}

TEST_F(MockBusServerTest, AMotorOfAnotherBusStaysSilent)
{
  uint16_t model_number = 0;
  EXPECT_EQ(packet_->ping(port_, 3, &model_number, &error_), COMM_RX_TIMEOUT);
  EXPECT_EQ(packet_->ping(port_, 5, &model_number, &error_), COMM_RX_TIMEOUT);
}

TEST_F(MockBusServerTest, AddressedReadsAndWritesReachTheRegisters)
{
  const auto motor = manager().getMotor(1);
  ASSERT_EQ(packet_->write4ByteTxRx(port_, 1, goal_position_, 123456, &error_), COMM_SUCCESS);
  EXPECT_EQ(motor->read4Byte(goal_position_), 123456U);
  const uint16_t goal_current = motor->getAddress("goal_current");
  ASSERT_EQ(packet_->write2ByteTxRx(port_, 1, goal_current, 300, &error_), COMM_SUCCESS);
  const uint16_t led_red = motor->getAddress("led_red");
  ASSERT_EQ(packet_->write1ByteTxRx(port_, 1, led_red, 7, &error_), COMM_SUCCESS);

  uint32_t position = 0;
  uint16_t current = 0;
  uint8_t led = 0;
  ASSERT_EQ(packet_->read4ByteTxRx(port_, 1, goal_position_, &position, &error_), COMM_SUCCESS);
  ASSERT_EQ(packet_->read2ByteTxRx(port_, 1, goal_current, &current, &error_), COMM_SUCCESS);
  ASSERT_EQ(packet_->read1ByteTxRx(port_, 1, led_red, &led, &error_), COMM_SUCCESS);
  EXPECT_EQ(position, 123456U);
  EXPECT_EQ(current, 300U);
  EXPECT_EQ(led, 7U);
  EXPECT_EQ(error_, 0U);
}

// Stuffing is where the server and the SDK must agree, in both directions
TEST_F(MockBusServerTest, AHeaderInsideAValueSurvivesStuffing)
{
  constexpr uint32_t value = 0xFDFDFFFF;  // FF FF FD FD, which reads as stuffed unless stuffed again
  ASSERT_EQ(packet_->write4ByteTxRx(port_, 1, goal_position_, value, &error_), COMM_SUCCESS);
  EXPECT_EQ(manager().getMotor(1)->read4Byte(goal_position_), value);
  uint32_t read_back = 0;
  ASSERT_EQ(packet_->read4ByteTxRx(port_, 1, goal_position_, &read_back, &error_), COMM_SUCCESS);
  EXPECT_EQ(read_back, value);
}

TEST_F(MockBusServerTest, TheMocksFirmwareRulesAnswerOnTheWire)
{
  const auto motor = manager().getMotor(1);
  ASSERT_EQ(packet_->write1ByteTxRx(port_, 1, motor->getAddress("torque_enable"), 1, &error_), COMM_SUCCESS);
  ASSERT_EQ(packet_->write1ByteTxRx(port_, 1, motor->getAddress("operating_mode"), 1, &error_), COMM_SUCCESS);
  EXPECT_EQ(error_, PACKET_ERROR_ACCESS) << "an EEPROM write while torqued must be refused";
}

TEST_F(MockBusServerTest, ASilentMotorStaysSilentOnAnUnservedLength)
{
  manager().getMotor(1)->setCommunicationFault(CommunicationFault::DROP_STATUS_PACKETS);
  uint8_t data[3] = {};
  EXPECT_EQ(packet_->readTxRx(port_, 1, goal_position_, 3, data, &error_), COMM_RX_TIMEOUT);
}

TEST_F(MockBusServerTest, ASilentMotorTimesOutButAppliesTheWrite)
{
  manager().getMotor(1)->setCommunicationFault(CommunicationFault::DROP_STATUS_PACKETS);
  EXPECT_EQ(packet_->write4ByteTxRx(port_, 1, goal_position_, 42, &error_), COMM_RX_TIMEOUT);
  EXPECT_EQ(manager().getMotor(1)->read4Byte(goal_position_), 42U);
}

TEST_F(MockBusServerTest, ARebootResetsTheRam)
{
  const auto motor = manager().getMotor(1);
  ASSERT_EQ(packet_->write1ByteTxRx(port_, 1, motor->getAddress("torque_enable"), 1, &error_), COMM_SUCCESS);
  ASSERT_EQ(packet_->reboot(port_, 1, &error_), COMM_SUCCESS);
  EXPECT_EQ(motor->getRebootCount(), 1);
  EXPECT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 0U);
}

TEST_F(MockBusServerTest, ASyncReadAnswersForEveryMotor)
{
  manager().getMotor(1)->moveByHand(0.5);
  manager().getMotor(2)->moveByHand(-0.25);
  const uint16_t present_position = manager().getMotor(1)->getAddress("present_position");
  dynamixel::GroupSyncRead group(port_, packet_, present_position, 4);
  ASSERT_TRUE(group.addParam(1));
  ASSERT_TRUE(group.addParam(2));
  ASSERT_EQ(group.txRxPacket(), COMM_SUCCESS);
  for (const uint8_t id : {1, 2}) {
    ASSERT_TRUE(group.isAvailable(id, present_position, 4));
    EXPECT_EQ(group.getData(id, present_position, 4), manager().getMotor(id)->read4Byte(present_position));
  }
  EXPECT_NE(group.getData(1, present_position, 4), group.getData(2, present_position, 4));
}

TEST_F(MockBusServerTest, ASyncReadFailsWhenOneMotorIsSilent)
{
  manager().getMotor(2)->setCommunicationFault(CommunicationFault::DROP_STATUS_PACKETS);
  dynamixel::GroupSyncRead group(port_, packet_, goal_position_, 4);
  ASSERT_TRUE(group.addParam(1));
  ASSERT_TRUE(group.addParam(2));
  EXPECT_EQ(group.txRxPacket(), COMM_RX_TIMEOUT);
}

TEST_F(MockBusServerTest, ASyncWriteReachesEveryMotor)
{
  dynamixel::GroupSyncWrite group(port_, packet_, goal_position_, 4);
  uint8_t first[4] = {0x10, 0x20, 0x00, 0x00};
  uint8_t second[4] = {0xFF, 0xFF, 0xFD, 0x00};
  ASSERT_TRUE(group.addParam(1, first));
  ASSERT_TRUE(group.addParam(2, second));
  ASSERT_EQ(group.txPacket(), COMM_SUCCESS);
  server_->waitUntilIdle();
  EXPECT_EQ(manager().getMotor(1)->read4Byte(goal_position_), 0x2010U);
  EXPECT_EQ(manager().getMotor(2)->read4Byte(goal_position_), 0x00FDFFFFU);
}

// A servo answers a sync read from one snapshot, while the emulator's physics rewrites registers
TEST_F(MockBusServerTest, ASyncReadNeverSeesAHalfWrittenRegister)
{
  constexpr uint32_t first = 0x00FFFFFF;
  constexpr uint32_t second = 0x01000000;  // differs from first in every byte
  const auto motor = manager().getMotor(1);
  motor->write4Byte(goal_position_, first);
  std::atomic<bool> stop{false};
  std::thread writer([&] {
    for (bool flip = false; !stop; flip = !flip) {
      motor->write4Byte(goal_position_, flip ? second : first);
      std::this_thread::yield();
    }
  });
  dynamixel::GroupSyncRead group(port_, packet_, goal_position_, 4);
  ASSERT_TRUE(group.addParam(1));
  int torn = 0;
  int answered = 0;
  // The writer hogs the unfair mutex, so a read may time out, which is no tear
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
  while (answered < 200 && std::chrono::steady_clock::now() < deadline) {
    if (group.txRxPacket() == COMM_SUCCESS) {
      ++answered;
      const uint32_t value = group.getData(1, goal_position_, 4);
      torn += value != first && value != second ? 1 : 0;
    }
  }
  stop = true;
  writer.join();
  EXPECT_EQ(answered, 200) << "too few reads answered to judge";
  EXPECT_EQ(torn, 0) << "sync reads returned values that were never in the register";
}

// A servo applies a sync write in one go after its CRC check
TEST_F(MockBusServerTest, ASyncWriteLandsWhole)
{
  uint8_t first[4] = {0xFF, 0xFF, 0xFF, 0x00};
  uint8_t second[4] = {0x00, 0x00, 0x00, 0x01};
  const auto motor = manager().getMotor(1);
  motor->write4Byte(goal_position_, 0x00FFFFFF);
  std::atomic<bool> stop{false};
  std::atomic<int> torn{0};
  std::thread reader([&] {
    while (!stop) {
      const uint32_t value = motor->read4Byte(goal_position_);
      torn += value != 0x00FFFFFFU && value != 0x01000000U ? 1 : 0;
    }
  });
  dynamixel::GroupSyncWrite group(port_, packet_, goal_position_, 4);
  ASSERT_TRUE(group.addParam(1, first));
  for (int i = 0; i < 500; ++i) {
    if (!group.changeParam(1, i % 2 == 0 ? second : first) || group.txPacket() != COMM_SUCCESS) {
      ADD_FAILURE() << "sync write " << i << " failed";
      break;
    }
    server_->waitUntilIdle();
  }
  stop = true;
  reader.join();
  EXPECT_EQ(torn, 0) << "a reader saw a sync write half applied";
}

// The firmware rules of an addressed write hold for a sync write, which drops what they refuse
TEST_F(MockBusServerTest, ASyncWriteToEepromOfATorquedMotorIsDropped)
{
  const auto motor = manager().getMotor(1);
  const uint16_t velocity_limit = motor->getAddress("velocity_limit");
  motor->write4Byte(velocity_limit, 1000);
  motor->write1Byte(motor->getAddress("torque_enable"), 1);
  dynamixel::GroupSyncWrite sync_write(port_, packet_, velocity_limit, 4);
  uint8_t value[4] = {0xD0, 0x07, 0, 0};
  ASSERT_TRUE(sync_write.addParam(1, value));
  ASSERT_EQ(sync_write.txPacket(), COMM_SUCCESS);
  server_->waitUntilIdle();
  EXPECT_EQ(motor->read4Byte(velocity_limit), 1000U);
}

TEST_F(MockBusServerTest, ASyncWriteOfAnIndirectPointerOutsideRamIsDropped)
{
  constexpr uint16_t PH_POINTER_1 = 168;  // devices/models/PH.yaml
  const auto motor = manager().getMotor(1);
  const uint16_t before = motor->read2Byte(PH_POINTER_1);
  dynamixel::GroupSyncWrite sync_write(port_, packet_, PH_POINTER_1, 2);
  uint8_t pointer[2] = {static_cast<uint8_t>(motor->getAddress("operating_mode")), 0};
  ASSERT_TRUE(sync_write.addParam(1, pointer));
  ASSERT_EQ(sync_write.txPacket(), COMM_SUCCESS);
  server_->waitUntilIdle();
  EXPECT_EQ(motor->read2Byte(PH_POINTER_1), before);
}

TEST_F(MockBusServerTest, AnInstructionTheBusDoesNotServeGetsAnError)
{
  ASSERT_EQ(packet_->factoryReset(port_, 1, 0x01, &error_), COMM_SUCCESS);
  EXPECT_EQ(error_, PACKET_ERROR_INSTRUCTION);
  uint8_t data[3] = {};
  ASSERT_EQ(packet_->readTxRx(port_, 1, goal_position_, 3, data, &error_), COMM_SUCCESS);
  EXPECT_EQ(error_, PACKET_ERROR_DATA_LENGTH);
}

TEST_F(MockBusServerTest, ACorruptStatusReadsAsCorruptOnce)
{
  uint32_t value = 0;
  server_->injectWireFault(WireFault::CORRUPT);
  EXPECT_EQ(packet_->read4ByteTxRx(port_, 1, goal_position_, &value, &error_), COMM_RX_CORRUPT);
  EXPECT_EQ(packet_->read4ByteTxRx(port_, 1, goal_position_, &value, &error_), COMM_SUCCESS);
}

TEST_F(MockBusServerTest, ATruncatedStatusReadsAsCorrupt)
{
  uint32_t value = 0;
  server_->injectWireFault(WireFault::TRUNCATE);
  EXPECT_EQ(packet_->read4ByteTxRx(port_, 1, goal_position_, &value, &error_), COMM_RX_CORRUPT);
}

TEST_F(MockBusServerTest, ADroppedStatusTimesOutAfterTheWriteLanded)
{
  server_->injectWireFault(WireFault::DROP);
  EXPECT_EQ(packet_->write4ByteTxRx(port_, 1, goal_position_, 42, &error_), COMM_RX_TIMEOUT);
  EXPECT_EQ(manager().getMotor(1)->read4Byte(goal_position_), 42U);
}

// The SDK matches a status by id alone, so a late one answers the next read of the same motor
TEST_F(MockBusServerTest, ADelayedStatusIsTakenForTheNextAnswer)
{
  const auto motor = manager().getMotor(1);
  motor->write4Byte(goal_position_, 111);
  uint32_t value = 0;
  server_->injectWireFault(WireFault::DELAY);
  EXPECT_EQ(packet_->read4ByteTxRx(port_, 1, goal_position_, &value, &error_), COMM_RX_TIMEOUT);
  motor->write4Byte(goal_position_, 222);
  ASSERT_EQ(packet_->read4ByteTxRx(port_, 1, goal_position_, &value, &error_), COMM_SUCCESS);
  EXPECT_EQ(value, 111U) << "the late status was not taken for the answer";
}

TEST_F(MockBusServerTest, AFaultForNoPacketIsNoFault)
{
  uint32_t value = 0;
  server_->injectWireFault(WireFault::CORRUPT, 0);
  EXPECT_EQ(packet_->read4ByteTxRx(port_, 1, goal_position_, &value, &error_), COMM_SUCCESS);
}

TEST_F(MockBusServerTest, APersistentFaultLastsUntilCleared)
{
  uint32_t value = 0;
  server_->injectWireFault(WireFault::CORRUPT, PERSISTENT_WIRE_FAULT);
  EXPECT_EQ(packet_->read4ByteTxRx(port_, 1, goal_position_, &value, &error_), COMM_RX_CORRUPT);
  EXPECT_EQ(packet_->read4ByteTxRx(port_, 2, goal_position_, &value, &error_), COMM_RX_CORRUPT);
  server_->clearWireFault();
  EXPECT_EQ(packet_->read4ByteTxRx(port_, 1, goal_position_, &value, &error_), COMM_SUCCESS);
}

TEST_F(MockBusServerTest, PowerComesBackWithTheRamReset)
{
  const auto motor = manager().getMotor(1);
  const uint16_t torque_enable = motor->getAddress("torque_enable");
  ASSERT_EQ(packet_->write1ByteTxRx(port_, 1, torque_enable, 1, &error_), COMM_SUCCESS);
  ASSERT_EQ(packet_->write4ByteTxRx(port_, 1, goal_position_, 4096, &error_), COMM_SUCCESS);

  server_->setPower(false);
  uint16_t model_number = 0;
  EXPECT_EQ(packet_->ping(port_, 1, &model_number, &error_), COMM_RX_TIMEOUT);
  EXPECT_EQ(packet_->ping(port_, 2, &model_number, &error_), COMM_RX_TIMEOUT);

  server_->setPower(true);
  EXPECT_EQ(packet_->ping(port_, 1, &model_number, &error_), COMM_SUCCESS);
  EXPECT_EQ(motor->read1Byte(torque_enable), 0U);
  EXPECT_EQ(motor->read4Byte(goal_position_), 0U);
}

// In gz a torque on that reached an unpowered servo would drive its joint
TEST_F(MockBusServerTest, AnUnpoweredMotorIgnoresASyncWrite)
{
  const auto motor = manager().getMotor(1);
  const uint16_t torque_enable = motor->getAddress("torque_enable");
  server_->setPower(false);
  dynamixel::GroupSyncWrite group(port_, packet_, torque_enable, 1);
  uint8_t on = 1;
  ASSERT_TRUE(group.addParam(1, &on));
  ASSERT_EQ(group.txPacket(), COMM_SUCCESS);
  server_->waitUntilIdle();
  EXPECT_EQ(motor->read1Byte(torque_enable), 0U);
}

// The plugin reads the error register itself, so only the server's own test sees the alert bit
TEST_F(MockBusServerTest, ASyncReadStatusCarriesTheAlertBit)
{
  manager().getMotor(2)->setHardwareError(ERROR_OVERLOAD);
  dynamixel::GroupSyncRead group(port_, packet_, goal_position_, 4);
  ASSERT_TRUE(group.addParam(1));
  ASSERT_TRUE(group.addParam(2));
  ASSERT_EQ(group.txRxPacket(), COMM_SUCCESS);
  uint8_t error = 0;
  ASSERT_TRUE(group.getError(1, &error));
  EXPECT_EQ(error, 0U);
  ASSERT_TRUE(group.getError(2, &error));
  EXPECT_EQ(error, PACKET_ERROR_ALERT);
}

TEST_F(MockBusServerTest, AnUnpoweredMotorGoesLimp)
{
  const auto motor = manager().getMotor(1);
  motor->write1Byte(motor->getAddress("operating_mode"), static_cast<uint8_t>(OperatingMode::VELOCITY_CONTROL));
  motor->write1Byte(motor->getAddress("torque_enable"), 1);
  motor->write4Byte(motor->getAddress("goal_velocity"), 1000);
  manager().update(0.1);
  ASSERT_GT(std::abs(motor->getCurrentVelocity()), 0.1) << "test premise: the motor turns";

  server_->setPower(false);
  const double cut_at = motor->getCurrentPosition();
  manager().update(0.1);
  EXPECT_EQ(motor->getCurrentPosition(), cut_at) << "an unpowered motor kept turning";
}

// The gz host builds one server per bus, so a failed one must not leak its pty
TEST(MockBusServerConstruction, AFailedLinkClosesThePty)
{
  const auto open_fds = []() {
    return std::distance(std::filesystem::directory_iterator("/proc/self/fd"), std::filesystem::directory_iterator());
  };
  const auto before = open_fds();
  EXPECT_THROW(MockBusServer({1}, "/nonexistent_dynamixel_test_dir/bus"), std::filesystem::filesystem_error);
  EXPECT_EQ(open_fds(), before) << "the pty of a server whose link failed stayed open";
}

}  // namespace
}  // namespace dynamixel_ros_control
