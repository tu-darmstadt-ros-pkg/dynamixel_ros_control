#include <gtest/gtest.h>
#include <spawn.h>
#include <sys/wait.h>

#include <chrono>
#include <csignal>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <dynamixel_sdk/dynamixel_sdk.h>

extern char** environ;

namespace {

class BusEmulatorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char dir[] = "/tmp/dxl_emulator_XXXXXX";
    dir_ = mkdtemp(dir);
    link_ = dir_ + "/bus/ttyUSB_test";
  }

  // A failed assertion returns before the test stops the emulator
  void TearDown() override
  {
    if (pid_ > 0) {
      kill(pid_, SIGKILL);
      waitpid(pid_, nullptr, 0);
    }
    std::filesystem::remove_all(dir_);
  }

  /// Runs the emulator with @p args
  void spawn(std::vector<std::string> args)
  {
    args.insert(args.begin(), BUS_EMULATOR);
    std::vector<char*> argv;
    for (auto& arg : args) {
      argv.push_back(arg.data());
    }
    argv.push_back(nullptr);
    ASSERT_EQ(posix_spawn(&pid_, BUS_EMULATOR, nullptr, nullptr, argv.data(), environ), 0);
  }

  /// The exit code, or -1 after killing an emulator that did not exit in time
  int exitCode()
  {
    int status = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    pid_t done = 0;
    while ((done = waitpid(pid_, &status, WNOHANG)) == 0 && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (done != pid_) {
      ADD_FAILURE() << "the emulator did not exit";
      kill(pid_, SIGKILL);
      waitpid(pid_, nullptr, 0);
      status = -1;
    }
    pid_ = 0;
    return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  }

  std::string dir_;
  std::string link_;
  pid_t pid_{0};
};

TEST_F(BusEmulatorTest, ServesItsMotorsUntilInterrupted)
{
  spawn({link_, "1:2020", "4:54024"});
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (!std::filesystem::exists(link_) && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  const std::unique_ptr<dynamixel::PortHandler> port(dynamixel::PortHandler::getPortHandler(link_.c_str()));
  auto* packet = dynamixel::PacketHandler::getPacketHandler(2.0);
  ASSERT_TRUE(port->openPort());
  uint16_t model_number = 0;
  EXPECT_EQ(packet->ping(port.get(), 1, &model_number), COMM_SUCCESS);
  EXPECT_EQ(model_number, 2020);
  EXPECT_EQ(packet->ping(port.get(), 4, &model_number), COMM_SUCCESS);
  EXPECT_EQ(model_number, 54024);
  port->closePort();

  kill(pid_, SIGINT);
  EXPECT_EQ(exitCode(), 0);
  EXPECT_FALSE(std::filesystem::exists(std::filesystem::symlink_status(link_))) << "the link was left behind";
}

TEST_F(BusEmulatorTest, RefusesAMotorItCannotEmulate)
{
  for (const auto& [motors, reason] :
       {std::pair<std::vector<std::string>, const char*>{{"1:2020", "1:2020"}, "a duplicate id"},
        {{"1:7"}, "a model without a control table"},
        {{"1"}, "no model number"}}) {
    std::vector<std::string> args{link_};
    args.insert(args.end(), motors.begin(), motors.end());
    spawn(args);
    EXPECT_EQ(exitCode(), 2) << reason;
  }
}

}  // namespace
