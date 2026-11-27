#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <dynamixel_ros_control/mock_dynamixel.hpp>
#include <gz/sim/TestFixture.hh>

namespace fs = std::filesystem;
using dynamixel_ros_control::MockDynamixelManager;

namespace {

std::string readFile(const fs::path& path)
{
  std::ifstream file(path);
  std::stringstream content;
  content << file.rdbuf();
  return content.str();
}

void replaceAll(std::string& text, const std::string& from, const std::string& to)
{
  for (auto at = text.find(from); at != std::string::npos; at = text.find(from, at + to.size())) {
    text.replace(at, from.size(), to);
  }
}

/// The rig in a directory of its own, removed again however the test ends
class Rig
{
public:
  explicit Rig(const std::string& from = "", const std::string& to = "")
  {
    std::string pattern = (fs::temp_directory_path() / "dxl_gz_wd_XXXXXX").string();
    if (mkdtemp(pattern.data()) == nullptr) {
      throw std::runtime_error("mkdtemp failed");
    }
    directory_ = pattern;
    setenv("GZ_PARTITION", directory_.filename().c_str(), 1);
    setenv("GZ_IP", "127.0.0.1", 1);

    std::string robot = readFile(fs::path(TEST_CONFIG_DIR) / "bus_rig.urdf");
    replaceAll(robot, "@PORT@", port().string());
    if (!from.empty()) {
      if (robot.find(from) == std::string::npos) {
        throw std::runtime_error("bus_rig.urdf has no '" + from + "'");
      }
      replaceAll(robot, from, to);
    }
    std::ofstream(directory_ / "bus_rig.urdf") << robot;
    std::string world = readFile(fs::path(TEST_CONFIG_DIR) / "bus_rig.sdf");
    replaceAll(world, "@ROBOT@", (directory_ / "bus_rig.urdf").string());
    std::ofstream(directory_ / "bus_rig.sdf") << world;
  }
  ~Rig()
  {
    fs::remove_all(directory_);
  }
  Rig(const Rig&) = delete;
  Rig& operator=(const Rig&) = delete;

  [[nodiscard]] fs::path world() const
  {
    return directory_ / "bus_rig.sdf";
  }
  [[nodiscard]] fs::path port() const
  {
    return directory_ / "ports" / "bus";
  }

private:
  fs::path directory_;
};

}  // namespace

// A controller manager that stops talking must halt the servos, timed on the sim clock that paces it
TEST(BusWatchdog, TripsAfterItsWindowOfSimTime)
{
  const Rig rig;
  gz::sim::TestFixture fixture(rig.world().string());
  fixture.Finalize();
  fixture.Server()->Run(true, 1, false);

  const auto motor = MockDynamixelManager::instance().getMotor(1);
  ASSERT_NE(motor, nullptr);
  const uint16_t watchdog = motor->getAddress("bus_watchdog");
  ASSERT_GT(watchdog, 0);
  motor->write1Byte(watchdog, 5);  // 100 ms
  motor->write1Byte(motor->getAddress("torque_enable"), 1);

  fixture.Server()->Run(true, 90, false);
  EXPECT_EQ(motor->timedBusWatchdogTrips(), 0) << "tripped before 100 ms of sim time";
  fixture.Server()->Run(true, 20, false);
  EXPECT_EQ(motor->timedBusWatchdogTrips(), 1) << "a silent bus did not trip after 100 ms of sim time";
}

// A servo attribute left out would read as 0, and a reduction of 0 divides by zero
TEST(BusConfiguration, AServoWithoutAReductionOpensNoBus)
{
  const Rig rig("id=\"1\" model=\"2020\" reduction=\"1.0\"", "id=\"1\" model=\"2020\"");
  gz::sim::TestFixture fixture(rig.world().string());
  fixture.Finalize();
  fixture.Server()->Run(true, 1, false);
  EXPECT_FALSE(fs::exists(rig.port())) << "the bus came up with a servo that has no reduction";
}

// 253 to 255 are reserved, 254 being the broadcast id every motor on the bus answers
TEST(BusConfiguration, AReservedServoIdOpensNoBus)
{
  for (const std::string id : {"253", "254"}) {
    const Rig rig("id=\"1\" model=\"2020\"", "id=\"" + id + "\" model=\"2020\"");
    gz::sim::TestFixture fixture(rig.world().string());
    fixture.Finalize();
    fixture.Server()->Run(true, 1, false);
    EXPECT_FALSE(fs::exists(rig.port())) << "the bus came up with a servo on the reserved id " << id;
  }
}

TEST(BusConfiguration, TheHighestServoIdOpensTheBus)
{
  const Rig rig("id=\"1\" model=\"2020\"", "id=\"252\" model=\"2020\"");
  gz::sim::TestFixture fixture(rig.world().string());
  fixture.Finalize();
  fixture.Server()->Run(true, 1, false);
  EXPECT_TRUE(fs::exists(rig.port())) << "a servo on id 252 was refused";
}
