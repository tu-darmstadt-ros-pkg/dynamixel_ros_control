#ifndef DYNAMIXEL_ROS_CONTROL_TEST_TEST_HOME_HPP
#define DYNAMIXEL_ROS_CONTROL_TEST_TEST_HOME_HPP

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>

namespace dynamixel_ros_control::test {

/// @brief Points HOME and ROS_LOG_DIR at a fresh directory of the running test while it lives.
///
/// The plugin keeps calibration offsets in $HOME, and one left by an earlier test or by the user
/// moves the motors at startup. The directory lies under DXL_TEST_HOME, which dxl_add_gtest sets in
/// the build tree, and is named after the test, so a test that died leaves it and its logs there to
/// be replaced by the next run rather than in /tmp.
class TestHome
{
public:
  TestHome()
  {
    const char* base = std::getenv("DXL_TEST_HOME");
    if (base == nullptr) {
      throw std::runtime_error("DXL_TEST_HOME is not set. Register the test with dxl_add_gtest.");
    }
    // A benchmark drives the fixtures outside any test
    const auto* test = ::testing::UnitTest::GetInstance()->current_test_info();
    std::string name = test != nullptr ? std::string(test->test_suite_name()) + "." + test->name() : "benchmark";
    // Typed and parameterised tests carry slashes in their names
    std::replace(name.begin(), name.end(), '/', '_');
    directory_ = std::filesystem::path(base) / name;
    std::filesystem::remove_all(directory_);
    std::filesystem::create_directories(directory_);
    redirect("HOME", directory_, original_home_);
    redirect("ROS_LOG_DIR", directory_ / "log", original_log_dir_);
  }

  ~TestHome()
  {
    restore("HOME", original_home_);
    restore("ROS_LOG_DIR", original_log_dir_);
    std::filesystem::remove_all(directory_);
  }

  TestHome(const TestHome&) = delete;
  TestHome& operator=(const TestHome&) = delete;

private:
  static void redirect(const char* name, const std::filesystem::path& value, std::optional<std::string>& original)
  {
    if (const char* current = std::getenv(name)) {
      original = current;
    }
    setenv(name, value.c_str(), 1);
  }

  static void restore(const char* name, const std::optional<std::string>& original)
  {
    if (original) {
      setenv(name, original->c_str(), 1);
    } else {
      unsetenv(name);
    }
  }

  std::filesystem::path directory_;
  std::optional<std::string> original_home_;
  std::optional<std::string> original_log_dir_;
};

}  // namespace dynamixel_ros_control::test

#endif
