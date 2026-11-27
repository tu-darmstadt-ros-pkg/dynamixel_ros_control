// read() and write() run on the realtime thread, where an allocation can block on the allocator.
// Its own binary, because it replaces the global operator new.
// C malloc, which rcutils and rcl use, is not counted, and only the healthy path runs.

#include <cstdlib>
#include <limits>
#include <memory>
#include <new>
#include <vector>

#include <dynamixel_sdk/dynamixel_sdk.h>

#include "wire_bus.hpp"

namespace {
// Per thread, so the executor, the DDS threads and the bus server count nothing
thread_local bool counting = false;
thread_local size_t allocations = 0;

void* allocate(std::size_t size, const std::size_t alignment = 0)
{
  if (counting) {
    ++allocations;
  }
  size = size == 0 ? 1 : size;
  if (alignment == 0) {
    return std::malloc(size);
  }
  // aligned_alloc takes only multiples of the alignment
  return std::aligned_alloc(alignment, (size + alignment - 1) / alignment * alignment);
}

void* allocateOrThrow(const std::size_t size, const std::size_t alignment = 0)
{
  if (void* memory = allocate(size, alignment)) {
    return memory;
  }
  throw std::bad_alloc();
}
}  // namespace

// Every form, since a sanitizer runtime exports its own and they do not forward to each other.
// Out of line, so the compiler pairs these with each other and not with malloc and free.
[[gnu::noinline]] void* operator new(const std::size_t size)
{
  return allocateOrThrow(size);
}
[[gnu::noinline]] void* operator new[](const std::size_t size)
{
  return allocateOrThrow(size);
}
[[gnu::noinline]] void* operator new(const std::size_t size, const std::nothrow_t& /*tag*/) noexcept
{
  return allocate(size);
}
[[gnu::noinline]] void* operator new[](const std::size_t size, const std::nothrow_t& /*tag*/) noexcept
{
  return allocate(size);
}
[[gnu::noinline]] void* operator new(const std::size_t size, const std::align_val_t alignment)
{
  return allocateOrThrow(size, static_cast<std::size_t>(alignment));
}
[[gnu::noinline]] void* operator new[](const std::size_t size, const std::align_val_t alignment)
{
  return allocateOrThrow(size, static_cast<std::size_t>(alignment));
}
[[gnu::noinline]] void* operator new(const std::size_t size, const std::align_val_t alignment,
                                     const std::nothrow_t& /*tag*/) noexcept
{
  return allocate(size, static_cast<std::size_t>(alignment));
}
[[gnu::noinline]] void* operator new[](const std::size_t size, const std::align_val_t alignment,
                                       const std::nothrow_t& /*tag*/) noexcept
{
  return allocate(size, static_cast<std::size_t>(alignment));
}

[[gnu::noinline]] void operator delete(void* memory) noexcept
{
  std::free(memory);
}
[[gnu::noinline]] void operator delete[](void* memory) noexcept
{
  std::free(memory);
}
[[gnu::noinline]] void operator delete(void* memory, std::size_t /*size*/) noexcept
{
  std::free(memory);
}
[[gnu::noinline]] void operator delete[](void* memory, std::size_t /*size*/) noexcept
{
  std::free(memory);
}
[[gnu::noinline]] void operator delete(void* memory, const std::nothrow_t& /*tag*/) noexcept
{
  std::free(memory);
}
[[gnu::noinline]] void operator delete[](void* memory, const std::nothrow_t& /*tag*/) noexcept
{
  std::free(memory);
}
[[gnu::noinline]] void operator delete(void* memory, std::align_val_t /*alignment*/) noexcept
{
  std::free(memory);
}
[[gnu::noinline]] void operator delete[](void* memory, std::align_val_t /*alignment*/) noexcept
{
  std::free(memory);
}
[[gnu::noinline]] void operator delete(void* memory, std::size_t /*size*/, std::align_val_t /*alignment*/) noexcept
{
  std::free(memory);
}
[[gnu::noinline]] void operator delete[](void* memory, std::size_t /*size*/, std::align_val_t /*alignment*/) noexcept
{
  std::free(memory);
}
[[gnu::noinline]] void operator delete(void* memory, std::align_val_t /*alignment*/,
                                       const std::nothrow_t& /*tag*/) noexcept
{
  std::free(memory);
}
[[gnu::noinline]] void operator delete[](void* memory, std::align_val_t /*alignment*/,
                                         const std::nothrow_t& /*tag*/) noexcept
{
  std::free(memory);
}

namespace dynamixel_ros_control::test {

class AllocationTest : public WireBusTest
{
protected:
  /// The operator new calls of @p call alone
  template <typename Call>
  static size_t allocationsOf(const Call& call)
  {
    allocations = 0;
    counting = true;
    call();
    counting = false;
    return allocations;
  }

  /// What the SDK's own group read and group write of @p motors cost, on a bus of their own, so that
  /// only what read() and write() add on top is held against them
  struct SdkBaseline
  {
    size_t read{0};
    size_t write{0};
  };

  static SdkBaseline measureSdk(const size_t motors)
  {
    std::vector<uint8_t> ids;
    for (size_t i = 0; i < motors; ++i) {
      ids.push_back(static_cast<uint8_t>(BASELINE_FIRST_ID + i));
      MockDynamixelManager::instance().addMotor(ids.back(), DEFAULT_MOCK_MODEL_NUMBER);
    }
    const auto motor = MockDynamixelManager::instance().getMotor(ids.front());
    const uint16_t present_position = motor->getAddress("present_position");
    const uint16_t goal_position = motor->getAddress("goal_position");
    MockBusServer server(ids);
    const std::unique_ptr<dynamixel::PortHandler> port(
        dynamixel::PortHandler::getPortHandler(server.portName().c_str()));
    dynamixel::PacketHandler* packet = dynamixel::PacketHandler::getPacketHandler(2.0);
    EXPECT_TRUE(port->openPort());
    EXPECT_TRUE(port->setBaudRate(1000000));

    dynamixel::GroupSyncRead group_read(port.get(), packet, present_position, 4);
    dynamixel::GroupSyncWrite group_write(port.get(), packet, goal_position, 4);
    uint8_t payload[4] = {0, 0, 0, 0};
    for (const uint8_t id : ids) {
      EXPECT_TRUE(group_read.addParam(id));
      EXPECT_TRUE(group_write.addParam(id, payload));
    }
    // Warm-up, like the cycles before the counted ones
    EXPECT_EQ(group_read.txRxPacket(), COMM_SUCCESS);
    EXPECT_EQ(group_write.txPacket(), COMM_SUCCESS);
    server.waitUntilIdle();

    SdkBaseline baseline;
    baseline.read = allocationsOf([&] { EXPECT_EQ(group_read.txRxPacket(), COMM_SUCCESS); });
    baseline.write = allocationsOf([&] {
      for (const uint8_t id : ids) {
        EXPECT_TRUE(group_write.changeParam(id, payload));
      }
      EXPECT_EQ(group_write.txPacket(), COMM_SUCCESS);
    });
    server.waitUntilIdle();
    port->closePort();
    for (const uint8_t id : ids) {
      MockDynamixelManager::instance().removeMotor(id);
    }
    return baseline;
  }

  static constexpr int CYCLES = 100;
  static constexpr uint8_t BASELINE_FIRST_ID = 101;
};

// Over the wire, so the SDK's own code paths run, and long enough to cover the diagnostics
// snapshot. read() makes one group read and write() one group write of the four flipper motors.
TEST_F(AllocationTest, ReadAllocatesNothingBeyondTheSdk)
{
  const SdkBaseline sdk = measureSdk(FLIPPER_MOTOR_IDS.size());
  RecordProperty("sdk_read_allocations", static_cast<int>(sdk.read));
  RecordProperty("sdk_write_allocations", static_cast<int>(sdk.write));
  startComponent("athena_flipper_interface");
  startInterface("flipper_fl_joint/position", 0.0);
  cycles(10);
  for (int i = 0; i < CYCLES; ++i) {
    const rclcpp::Duration period = rclcpp::Duration::from_seconds(kPeriod);
    time_ += period;
    EXPECT_EQ(allocationsOf([&] { ASSERT_EQ(hardware_->read(time_, period), hardware_interface::return_type::OK); }),
              sdk.read)
        << "cycle " << i;
    ASSERT_EQ(hardware_->write(time_, period), hardware_interface::return_type::OK);
    step();
  }
}

TEST_F(AllocationTest, WriteAllocatesNothingBeyondTheSdk)
{
  const SdkBaseline sdk = measureSdk(FLIPPER_MOTOR_IDS.size());
  RecordProperty("sdk_read_allocations", static_cast<int>(sdk.read));
  RecordProperty("sdk_write_allocations", static_cast<int>(sdk.write));
  startComponent("athena_flipper_interface");
  startInterface("flipper_fl_joint/position", 0.0);
  cycles(10);
  for (int i = 0; i < CYCLES; ++i) {
    const rclcpp::Duration period = rclcpp::Duration::from_seconds(kPeriod);
    time_ += period;
    ASSERT_EQ(hardware_->read(time_, period), hardware_interface::return_type::OK);
    EXPECT_EQ(allocationsOf([&] { ASSERT_EQ(hardware_->write(time_, period), hardware_interface::return_type::OK); }),
              sdk.write)
        << "cycle " << i;
    step();
  }
}

// The caps are clamped in write(), a NaN one included
TEST_F(AllocationTest, WriteWithCapsAllocatesNothingBeyondTheSdk)
{
  const SdkBaseline sdk = measureSdk(FLIPPER_MOTOR_IDS.size());
  startComponent("athena_flipper_interface", [](std::string urdf) {
    const std::string current = "<command_interface name=\"current\"/>";
    for (auto pos = urdf.find(current); pos != std::string::npos; pos = urdf.find(current, pos)) {
      urdf.replace(pos, current.size(), "<command_interface name=\"max_current\"/>");
    }
    return urdf;
  });
  ASSERT_EQ(hardware_->perform_command_mode_switch({"flipper_fl_joint/position", "flipper_fl_joint/max_current"}, {}),
            hardware_interface::return_type::OK);
  setCommand("flipper_fl_joint/position", state("flipper_fl_joint/position"));
  cycles(10);
  for (int i = 0; i < CYCLES; ++i) {
    setCommand("flipper_fl_joint/max_current", i % 2 == 0 ? std::numeric_limits<double>::quiet_NaN() : 100.0);
    const rclcpp::Duration period = rclcpp::Duration::from_seconds(kPeriod);
    time_ += period;
    ASSERT_EQ(hardware_->read(time_, period), hardware_interface::return_type::OK);
    EXPECT_EQ(allocationsOf([&] { ASSERT_EQ(hardware_->write(time_, period), hardware_interface::return_type::OK); }),
              sdk.write)
        << "cycle " << i;
    step();
  }
}

}  // namespace dynamixel_ros_control::test
