// Emulates one Dynamixel bus on a pty, so the plugin runs unchanged without the robot.
// Usage: dynamixel_bus_emulator <port link> <id>:<model number>...

#include <charconv>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "dynamixel_ros_control/mock_bus_server.hpp"

namespace {

volatile std::sig_atomic_t running = 1;

void stop(int /*signal*/)
{
  running = 0;
}

/// "<id>:<model number>"
std::optional<std::pair<uint8_t, uint16_t>> parseMotor(const std::string& text)
{
  const char* end = text.data() + text.size();
  unsigned int id = 0;
  unsigned int model_number = 0;
  const auto [colon, id_error] = std::from_chars(text.data(), end, id);
  if (id_error != std::errc{} || colon == end || *colon != ':' || id > 252) {
    return std::nullopt;
  }
  const auto [rest, model_error] = std::from_chars(colon + 1, end, model_number);
  if (model_error != std::errc{} || rest != end || model_number > 0xFFFF) {
    return std::nullopt;
  }
  return std::make_pair(static_cast<uint8_t>(id), static_cast<uint16_t>(model_number));
}

}  // namespace

int main(int argc, char** argv)
{
  using dynamixel_ros_control::MockDynamixelManager;
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <port link> <id>:<model number>..." << std::endl;
    return 2;
  }
  // Before the link exists, so an early signal still removes it
  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);
  auto& manager = MockDynamixelManager::instance();
  std::vector<uint8_t> ids;
  for (int i = 2; i < argc; ++i) {
    const auto motor = parseMotor(argv[i]);
    if (!motor || manager.getMotor(motor->first)) {
      std::cerr << "'" << argv[i] << "' is no new <id>:<model number>." << std::endl;
      return 2;
    }
    try {
      manager.addMotor(motor->first, motor->second);
    }
    catch (const std::runtime_error& error) {
      std::cerr << error.what() << std::endl;
      return 2;
    }
    ids.push_back(motor->first);
  }
  // The firmware counts the watchdog window in real time, and so does the plugin
  manager.setBusWatchdogTiming(true, true);

  const std::filesystem::path link = argv[1];
  if (link.has_parent_path()) {
    std::filesystem::create_directories(link.parent_path());
  }
  dynamixel_ros_control::MockBusServer server(ids, link);
  std::cout << "Serving " << ids.size() << " motors on " << server.portName() << std::endl;

  auto last = std::chrono::steady_clock::now();
  while (running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    const auto now = std::chrono::steady_clock::now();
    manager.update(std::chrono::duration<double>(now - last).count());
    last = now;
  }
  return 0;
}
