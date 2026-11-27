#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <tuple>

#include "direct_hardware_fixture.hpp"
#include "problem_capture.hpp"

namespace dynamixel_ros_control::test {

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;

// Configure reads the latency timer of the adapter behind the port from a fake sysfs, through a
// udev style symlink as on the robot
class LatencyTimerTest : public DirectHardwareTest
{
protected:
  void SetUp() override
  {
    DirectHardwareTest::SetUp();
    root_ = std::getenv("HOME");
    std::filesystem::create_directories(root_ / "dev");
    std::ofstream(root_ / "dev/ttyUSB7").flush();
    std::filesystem::create_symlink(root_ / "dev/ttyUSB7", root_ / "dev/ttyUSB_flippers");
  }

  void adjustInfo(hardware_interface::HardwareInfo& info) override
  {
    info.hardware_parameters["port_name"] = (root_ / "dev/ttyUSB_flippers").string();
  }

  void setLatencyTimer(const int latency_ms) const
  {
    const auto device = root_ / "sys/bus/usb-serial/devices/ttyUSB7";
    std::filesystem::create_directories(device);
    std::ofstream(device / "latency_timer") << latency_ms << "\n";
  }

  void configure()
  {
    initComponent("athena_flipper_interface");
    hardware_ = std::make_unique<DynamixelHardwareInterface>(root_ / "sys");
    ASSERT_EQ(hardware_->init(params_), hardware_interface::CallbackReturn::SUCCESS);
    std::ignore = hardware_->on_export_state_interfaces();
    std::ignore = hardware_->on_export_command_interfaces();
    ASSERT_EQ(hardware_->on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
    configured_ = true;
  }

  /// The bus status of the manifest configure published
  DiagnosticStatus manifestBus()
  {
    auto node = std::make_shared<rclcpp::Node>("latency_timer_test");
    DiagnosticArray::SharedPtr manifest;
    auto subscription = node->create_subscription<DiagnosticArray>(
        "/athena_flipper_interface_node/manifest", rclcpp::QoS(1).transient_local(),
        [&manifest](DiagnosticArray::SharedPtr msg) { manifest = std::move(msg); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    waitUntil([&] {
      executor.spin_some();
      return manifest != nullptr;
    });
    if (!manifest) {
      return DiagnosticStatus();
    }
    for (const auto& status : manifest->status) {
      if (status.name == "athena_flipper_interface/bus") {
        return status;
      }
    }
    ADD_FAILURE() << "the manifest has no bus status";
    return DiagnosticStatus();
  }

  static std::string valueOf(const DiagnosticStatus& status, const std::string& key)
  {
    for (const auto& value : status.values) {
      if (value.key == key) {
        return value.value;
      }
    }
    return "";
  }

  std::filesystem::path root_;
};

TEST_F(LatencyTimerTest, TheAdapterDefaultWarnsAndIsReported)
{
  setLatencyTimer(16);
  const ProblemCapture capture;
  configure();
  EXPECT_FALSE(capture.loggerOf("ttyUSB_flippers' is 16 ms").empty()) << "the slow latency timer was not reported";
  const DiagnosticStatus bus = manifestBus();
  EXPECT_EQ(bus.level, DiagnosticStatus::WARN);
  EXPECT_EQ(valueOf(bus, "usb_latency_timer_ms"), "16");
}

TEST_F(LatencyTimerTest, OneMillisecondIsReportedOk)
{
  setLatencyTimer(1);
  const ProblemCapture capture;
  configure();
  EXPECT_TRUE(capture.loggerOf("latency timer").empty());
  const DiagnosticStatus bus = manifestBus();
  EXPECT_EQ(bus.level, DiagnosticStatus::OK);
  EXPECT_EQ(valueOf(bus, "usb_latency_timer_ms"), "1");
}

TEST_F(LatencyTimerTest, AnUnreadableValueWarnsWithoutFailing)
{
  const auto device = root_ / "sys/bus/usb-serial/devices/ttyUSB7";
  std::filesystem::create_directories(device);
  std::ofstream(device / "latency_timer") << "garbage\n";
  const ProblemCapture capture;
  configure();
  EXPECT_EQ(capture.loggerOf("Could not read a latency timer"), "dynamixel_ros_control.athena_flipper_interface");
  EXPECT_EQ(valueOf(manifestBus(), "usb_latency_timer_ms"), "");
}

// A pty, a ttyACM or a non FTDI adapter has no latency timer
TEST_F(LatencyTimerTest, APortWithoutALatencyTimerIsSkipped)
{
  const ProblemCapture capture;
  configure();
  EXPECT_TRUE(capture.loggerOf("latency timer").empty());
  const DiagnosticStatus bus = manifestBus();
  EXPECT_EQ(bus.level, DiagnosticStatus::OK);
  EXPECT_EQ(valueOf(bus, "usb_latency_timer_ms"), "");
}

}  // namespace dynamixel_ros_control::test
