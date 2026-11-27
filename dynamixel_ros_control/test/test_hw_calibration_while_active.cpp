#include <atomic>
#include <chrono>
#include <cmath>

#include <hector_transmission_interface_msgs/srv/adjust_transmission_offsets.hpp>
#include <hector_transmission_interface_msgs/srv/flip_by_pi.hpp>

#include "direct_hardware_fixture.hpp"

namespace dynamixel_ros_control::test {

// read() and write() map with the transmission offsets, so a calibration changes them only while it
// holds the bus. A held joint keeps its actuator goal, a joint a controller commands is refused.
class CalibrationWhileActiveTest : public DirectHardwareTest
{
protected:
  using AdjustOffsets = hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets;
  using FlipByPi = hector_transmission_interface_msgs::srv::FlipByPi;
  static constexpr const char* kService = "/athena_flipper_interface_node/adjust_transmission_offsets";
  static constexpr const char* kFlipService = "/athena_flipper_interface_node/flip_by_pi";

  void SetUp() override
  {
    DirectHardwareTest::SetUp();
    startComponent("athena_flipper_interface");
    motor_ = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
    ASSERT_NE(motor_, nullptr);
  }

  static std::shared_ptr<AdjustOffsets::Request> measure(double position)
  {
    auto request = std::make_shared<AdjustOffsets::Request>();
    request->external_joint_measurements.name = {"flipper_fl_joint"};
    request->external_joint_measurements.position = {position};
    return request;
  }

  static std::shared_ptr<FlipByPi::Request> flip()
  {
    auto request = std::make_shared<FlipByPi::Request>();
    request->joint_names = {"flipper_fl_joint"};
    return request;
  }

  /// Whether the joint moved from @p reference by a half turn
  [[nodiscard]] bool halfTurnFrom(double reference) const
  {
    return std::abs(std::abs(std::remainder(jointPosition() - reference, 2 * M_PI)) - M_PI) < 1e-3;
  }

  [[nodiscard]] double jointPosition() const
  {
    return states_.at("flipper_fl_joint/position")->get_optional().value();
  }

  [[nodiscard]] int32_t goalRegister() const
  {
    return motor_->read4ByteSigned(motor_->getAddress("goal_position"));
  }

  std::shared_ptr<MockDynamixel> motor_;
};

TEST_F(CalibrationWhileActiveTest, AHeldJointIsCalibratedAndKeepsItsActuatorGoal)
{
  cycles(5);
  const int32_t goal_before = goalRegister();
  const double target = jointPosition() + 0.7;

  const auto response = callWhileCycling<AdjustOffsets>(kService, measure(target));
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success) << response->message;

  cycles(5);
  EXPECT_NEAR(jointPosition(), target, 1e-3) << "the new offset was not applied";
  EXPECT_EQ(goalRegister(), goal_before) << "calibrating a held joint moved its actuator goal";
}

TEST_F(CalibrationWhileActiveTest, AHeldJointIsFlippedWhileTheLoopCycles)
{
  cycles(5);
  const double before = jointPosition();

  const auto response = callWhileCycling<FlipByPi>(kFlipService, flip());
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success) << response->message;

  cycles(2);
  EXPECT_TRUE(halfTurnFrom(before)) << "the flip did not turn the joint by pi, it is at " << jointPosition();
}

TEST_F(CalibrationWhileActiveTest, AJointAControllerCommandsIsRefused)
{
  cycles(5);
  startInterface("flipper_fl_joint/position", jointPosition());  // holds where it is
  cycles(5);
  const double before = jointPosition();

  const auto response = callWhileCycling<AdjustOffsets>(kService, measure(before + 0.7));
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("a controller commands it"), std::string::npos) << response->message;
  cycles(2);
  EXPECT_NEAR(jointPosition(), before, 1e-3);
}

TEST_F(CalibrationWhileActiveTest, ACalibrationWaitsForARunningRead)
{
  cycles(5);
  const double before = jointPosition();
  auto node = std::make_shared<rclcpp::Node>("calibration_while_active_client");
  auto client = node->create_client<AdjustOffsets>(kService);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(10)));
  std::atomic<bool> started{false};
  bool answered_during_read = true;
  std::shared_future<AdjustOffsets::Response::SharedPtr> response;
  motor_->setRegisterReadHook("present_position", [&]() {
    if (started.exchange(true)) {
      return;
    }
    response = client->async_send_request(measure(before + 0.7)).future.share();
    answered_during_read = rclcpp::spin_until_future_complete(node, response, std::chrono::milliseconds(500)) ==
                           rclcpp::FutureReturnCode::SUCCESS;
  });
  ASSERT_EQ(cycle(), hardware_interface::return_type::OK);
  motor_->clearRegisterReadHooks();
  ASSERT_TRUE(started) << "precondition: read() reads the present position";
  EXPECT_FALSE(answered_during_read) << "the offset changed while read() mapped with it";

  // Nothing else has to run for it
  ASSERT_EQ(rclcpp::spin_until_future_complete(node, response, std::chrono::seconds(10)),
            rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(response.get()->success) << response.get()->message;
  cycles(2);
  EXPECT_NEAR(jointPosition(), before + 0.7, 1e-3) << "the new offset was not applied";
}

// An adjustment holds the bus, so a packet in it would stall the loop for a bus round trip. Nothing
// cycles during the calls, so every packet sent meanwhile would be theirs. A hold without packets
// is not bounded here, only measured by benchmark_hw_offset_service_call.
TEST_F(CalibrationWhileActiveTest, AnOffsetServiceCallSendsNoBusPacket)
{
  auto node = std::make_shared<rclcpp::Node>("calibration_while_active_client");
  auto adjust_client = node->create_client<AdjustOffsets>(kService);
  auto flip_client = node->create_client<FlipByPi>(kFlipService);
  ASSERT_TRUE(adjust_client->wait_for_service(std::chrono::seconds(10)));
  ASSERT_TRUE(flip_client->wait_for_service(std::chrono::seconds(10)));
  const auto call = [&](auto& client, auto request) {
    auto future = client->async_send_request(request);
    ASSERT_EQ(rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(10)),
              rclcpp::FutureReturnCode::SUCCESS);
    const auto response = future.get();
    EXPECT_TRUE(response->success) << response->message;
  };
  const double target = jointPosition() + 0.7;

  const uint64_t packets_before = MockDynamixelManager::instance().packetCount();
  call(adjust_client, measure(target));
  call(flip_client, flip());
  EXPECT_EQ(MockDynamixelManager::instance().packetCount(), packets_before) << "an offset service call used the bus";

  cycles(2);
  EXPECT_GT(MockDynamixelManager::instance().packetCount(), packets_before) << "precondition: the mock counts packets";
  EXPECT_TRUE(halfTurnFrom(target)) << "the adjustment and the flip did not both apply, the joint is at "
                                    << jointPosition();
}

}  // namespace dynamixel_ros_control::test
