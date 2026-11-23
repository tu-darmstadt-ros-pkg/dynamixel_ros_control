// A regression test of the DynamixelSDK, not of this package. It stays red, by aborting with
// "stack smashing detected", as long as the SDK in use lacks the fix of fix/rx_buffer_overflow.

#include <gtest/gtest.h>

#include <memory>

#include <dynamixel_ros_control/mock_bus_server.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>

namespace dynamixel_ros_control {
namespace {

constexpr uint16_t PH54 = 2020;

// The SDK received the answers to write, ping and reboot into stack buffers of 11 or 14 bytes and
// compared the id only after the whole packet was in, so a late longer status smashed the stack
TEST(SdkLateStatus, ALateLongStatusLeavesWritePingAndRebootIntact)
{
  auto& manager = MockDynamixelManager::instance();
  manager.addMotor(1, PH54);
  manager.addMotor(2, PH54);
  MockBusServer server({1, 2});
  auto* port = dynamixel::PortHandler::getPortHandler(server.portName().c_str());
  auto* packet = dynamixel::PacketHandler::getPacketHandler(2.0);
  ASSERT_TRUE(port->openPort());
  const uint16_t goal_position = manager.getMotor(1)->getAddress("goal_position");
  uint8_t error = 0;

  const auto late_sync_read = [&] {
    dynamixel::GroupSyncRead group(port, packet, goal_position, 32);
    ASSERT_TRUE(group.addParam(2));
    server.injectWireFault(WireFault::DELAY);
    EXPECT_EQ(group.txRxPacket(), COMM_RX_TIMEOUT);
  };
  late_sync_read();
  EXPECT_EQ(packet->write4ByteTxRx(port, 1, goal_position, 7, &error), COMM_SUCCESS);
  EXPECT_EQ(manager.getMotor(1)->read4Byte(goal_position), 7U);
  late_sync_read();
  uint16_t model_number = 0;
  EXPECT_EQ(packet->ping(port, 1, &model_number, &error), COMM_SUCCESS);
  EXPECT_EQ(model_number, PH54);
  late_sync_read();
  EXPECT_EQ(packet->reboot(port, 1, &error), COMM_SUCCESS);
  EXPECT_EQ(manager.getMotor(1)->getRebootCount(), 1);

  port->closePort();
  delete port;
  manager.reset();
}

}  // namespace
}  // namespace dynamixel_ros_control
