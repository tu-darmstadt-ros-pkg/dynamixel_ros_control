// The plugin alone, as ros2_control_node loads it, with no motor simulated in the process

#include <gtest/gtest.h>

#include <dynamixel_ros_control/dynamixel_driver.hpp>

TEST(DummyWithoutSimulationTest, UseDummyFailsWhereNoMotorIsSimulated)
{
  dynamixel_ros_control::DynamixelDriver driver;
  EXPECT_FALSE(driver.init("/dev/ttyUSB0", 57600, true));
}
