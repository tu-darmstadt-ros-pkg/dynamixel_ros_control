// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause
//
// loadInterfaceRegisterNameTranslation() is the first thing on_init does. A malformed file must
// fail cleanly there: an exception escaping into the controller manager, or half-loaded maps that
// let configuration continue, both surface as a far less obvious failure much later.

#include <gtest/gtest.h>

#include <dynamixel_ros_control/common.hpp>

#include <filesystem>
#include <fstream>

namespace dynamixel_ros_control::test {

class TranslationLoadingTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char dir_template[] = "/tmp/dxl_translation_test_XXXXXX";
    dir_ = mkdtemp(dir_template);
  }

  void TearDown() override
  {
    if (!dir_.empty()) {
      std::filesystem::remove_all(dir_);
    }
  }

  /// Write `content` to a file in the temp dir and return its path.
  std::string writeFile(const std::string& content)
  {
    const std::string path = dir_ + "/translation.yaml";
    std::ofstream file(path);
    file << content;
    file.close();
    return path;
  }

  bool load(const std::string& path)
  {
    return loadInterfaceRegisterNameTranslation(path, state_, command_, limits_);
  }

  std::string dir_;
  std::unordered_map<std::string, std::string> state_;
  std::unordered_map<std::string, std::string> command_;
  std::unordered_map<std::string, std::string> limits_;
};

TEST_F(TranslationLoadingTest, LoadsWellFormedFile)
{
  const std::string path = writeFile(R"(
state_interfaces:
  - interface_name: position
    register_name: present_position
  - interface_name: velocity
    register_name: present_velocity
command_interfaces:
  - interface_name: position
    register_name: goal_position
limits:
  - interface_name: velocity
    register_name: velocity_limit
)");
  ASSERT_TRUE(load(path));
  EXPECT_EQ(state_.at("position"), "present_position");
  EXPECT_EQ(state_.at("velocity"), "present_velocity");
  EXPECT_EQ(command_.at("position"), "goal_position");
  EXPECT_EQ(limits_.at("velocity"), "velocity_limit");
}

// The shipped file must keep passing its own loader.
TEST_F(TranslationLoadingTest, LoadsShippedDeviceFile)
{
  const std::string path = std::string(TEST_DEVICES_DIR) + "/interface_to_register_names.yaml";
  ASSERT_TRUE(std::filesystem::exists(path)) << "Shipped translation file missing at " << path;
  ASSERT_TRUE(load(path));
  EXPECT_FALSE(state_.empty());
  EXPECT_FALSE(command_.empty());
}

TEST_F(TranslationLoadingTest, MissingFileFails)
{
  EXPECT_FALSE(load(dir_ + "/does_not_exist.yaml"));
}

// .as<std::string>() throws on a missing key. Escaping on_init as an exception would take down the
// controller manager instead of failing the component's initialization.
TEST_F(TranslationLoadingTest, MissingKeyFailsWithoutThrowing)
{
  const std::string path = writeFile(R"(
state_interfaces:
  - register_name: present_position
command_interfaces: []
limits: []
)");
  EXPECT_NO_THROW({ EXPECT_FALSE(load(path)); });
}

// A section that is not a sequence used to yield an empty map while the load still reported
// success, leaving the interface with a silently empty translation table.
TEST_F(TranslationLoadingTest, NonSequenceSectionFails)
{
  const std::string path = writeFile(R"(
state_interfaces:
  position: present_position
command_interfaces: []
limits: []
)");
  EXPECT_FALSE(load(path));
}

TEST_F(TranslationLoadingTest, NonMapEntryFails)
{
  const std::string path = writeFile(R"(
state_interfaces:
  - just_a_scalar
command_interfaces: []
limits: []
)");
  EXPECT_FALSE(load(path));
}

TEST_F(TranslationLoadingTest, MissingSectionFails)
{
  const std::string path = writeFile(R"(
state_interfaces:
  - interface_name: position
    register_name: present_position
command_interfaces: []
)");
  EXPECT_FALSE(load(path));
}

TEST_F(TranslationLoadingTest, TopLevelSequenceFails)
{
  const std::string path = writeFile(R"(
- interface_name: position
  register_name: present_position
)");
  EXPECT_FALSE(load(path));
}

}  // namespace dynamixel_ros_control::test
