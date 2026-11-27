#include <gtest/gtest.h>

#include <dynamixel_ros_control/mock_bus_server.hpp>

namespace dynamixel_ros_control::protocol2 {

// The ping example of the Protocol 2.0 e-Manual, instruction and status
TEST(Protocol2Framing, TheCrcMatchesTheManualsPingInstruction)
{
  const std::vector<uint8_t> ping{0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01};
  EXPECT_EQ(crc(ping.data(), ping.size()), 0x4E19);
}

TEST(Protocol2Framing, AStatusPacketMatchesTheManualsPingStatus)
{
  const std::vector<uint8_t> expected{0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00,
                                      0x55, 0x00, 0x06, 0x04, 0x26, 0x65, 0x5D};
  EXPECT_EQ(statusPacket(1, 0, {0x06, 0x04, 0x26}), expected);
}

TEST(Protocol2Framing, AHeaderInsideTheBodyIsStuffedAndRestored)
{
  const std::vector<uint8_t> body{0x55, 0x00, 0xFF, 0xFF, 0xFD, 0x00, 0xFF, 0xFF, 0xFD};
  const std::vector<uint8_t> stuffed{0x55, 0x00, 0xFF, 0xFF, 0xFD, 0xFD, 0x00, 0xFF, 0xFF, 0xFD, 0xFD};
  EXPECT_EQ(stuff(body), stuffed);
  EXPECT_EQ(unstuff(stuffed), body);
}

TEST(Protocol2Framing, BytesThatAreNoHeaderAreLeftAlone)
{
  const std::vector<uint8_t> body{0x55, 0xFF, 0xFD, 0xFF, 0xFF, 0x00, 0xFD};
  EXPECT_EQ(stuff(body), body);
  EXPECT_EQ(unstuff(body), body);
}

}  // namespace dynamixel_ros_control::protocol2
