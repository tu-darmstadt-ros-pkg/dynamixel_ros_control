#ifndef DYNAMIXEL_ROS_CONTROL_TEST_DRIVER_FIXTURE_HPP
#define DYNAMIXEL_ROS_CONTROL_TEST_DRIVER_FIXTURE_HPP

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <dynamixel_ros_control/dynamixel.hpp>
#include <dynamixel_ros_control/dynamixel_driver.hpp>
#include <dynamixel_ros_control/mock_dynamixel.hpp>

namespace dynamixel_ros_control::test {

constexpr uint16_t MODEL_PH = 2020;  // PH54-200-S500-R
constexpr uint16_t MODEL_XM = 1020;  // XM430-W350, indirect pointers in RAM

/// A driver on the simulated bus, for the code above the SDK
class DriverTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    MockDynamixelManager::instance().reset();
    ASSERT_TRUE(driver_.init("/dev/ttyUSB0", 57600, true));
  }

  void TearDown() override
  {
    dynamixels_.clear();
    MockDynamixelManager::instance().reset();
  }

  /// A motor of @p model on the bus, connected
  Dynamixel& connected(const uint8_t id = 1, const uint16_t model = MODEL_PH)
  {
    MockDynamixelManager::instance().addMotor(id, model);
    auto& dxl = *dynamixels_.emplace_back(std::make_unique<Dynamixel>(id, driver_));
    EXPECT_TRUE(dxl.connect());
    return dxl;
  }

  static std::shared_ptr<MockDynamixel> motor(const uint8_t id = 1)
  {
    return MockDynamixelManager::instance().getMotor(id);
  }

  DynamixelDriver driver_;
  std::vector<std::unique_ptr<Dynamixel>> dynamixels_;
};

}  // namespace dynamixel_ros_control::test

#endif
