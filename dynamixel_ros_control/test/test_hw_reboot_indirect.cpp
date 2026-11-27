// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

#include <cstring>

namespace dynamixel_ros_control::test {

// ============================================================================
// Indirect-address restoration after a reboot
// ============================================================================
//
// A reboot wipes RAM, including the indirect-address *pointer* registers on models that hold them
// there (X-/MX-series, while P-/PRO-series keep them in EEPROM and survive). Every sync manager
// that uses indirect addressing must have its pointers rewritten afterwards, or it keeps reading
// from and writing to indirect slots that no longer point anywhere. Nothing fails at packet level,
// so the breakage is silent.
//
// These tests exercise a reboot on a RAM-resident model and require the read, command-read and
// control-write paths to keep working afterwards.
//
// led_write_manager_'s membership in the rewrite list cannot be tested with the current device set,
// and that is a hardware fact rather than a fixture gap: across devices/models/*.yaml the
// RGB-capable models (PH, PM, RH, PRO, H42) all keep their indirect pointers in EEPROM, while every
// RAM-resident model (XM, XMExt, MX2) is monochrome. No supported motor can both lose its pointers
// to a reboot and be in the LED group.
//
// Fixture notes, both load-bearing:
//   * The default mock model (2020 = PH54) keeps its indirect pointers in EEPROM, so the rewrite
//     path is skipped entirely for it. The flippers are pointed at XM430-W350 instead.
//   * Every RAM-resident model has only 28 indirect slots, and position+velocity+current across the
//     sync managers needs ~36 — which is why the real arm cannot run on X-series. The
//     flippers are reduced to position-only here, which fits in 24.
class RebootIndirectAddressTest : public HardwareInterfaceTest
{
protected:
  static constexpr const char* RAM_INDIRECT_MODEL = "1020";  // XM430_W350
  static constexpr uint16_t RAM_INDIRECT_MODEL_NUMBER = 1020;

  std::string transformUrdf(std::string urdf) override
  {
    for (const char* joint : {"flipper_fl_joint", "flipper_fr_joint", "flipper_bl_joint", "flipper_br_joint"}) {
      // The ros2_control entry, not the kinematic one — the latter carries a type= attribute.
      const std::string open_tag = "<joint name=\"" + std::string(joint) + "\">";
      const auto start = urdf.find(open_tag);
      if (start == std::string::npos) {
        ADD_FAILURE() << "Could not find ros2_control joint block for " << joint;
        return urdf;
      }
      const auto end = urdf.find("</joint>", start);
      if (end == std::string::npos) {
        ADD_FAILURE() << "Unterminated joint block for " << joint;
        return urdf;
      }

      std::string block = urdf.substr(start, end - start);
      // Drop the interfaces that do not fit in XM's 28 indirect slots.
      for (const char* line : {"<command_interface name=\"velocity\"/>", "<command_interface name=\"current\"/>",
                               "<state_interface name=\"velocity\"/>", "<state_interface name=\"current\"/>"}) {
        for (auto pos = block.find(line); pos != std::string::npos; pos = block.find(line)) {
          block.erase(pos, std::strlen(line));
        }
      }
      block = block.insert(open_tag.size(),
                           "\n      <param name=\"model_number\">" + std::string(RAM_INDIRECT_MODEL) + "</param>");
      urdf.replace(start, end - start, block);
    }
    return urdf;
  }

  // Provoke a reboot of one flipper motor via a hardware error. Returns the service's own verdict.
  bool rebootAfterErrorOn(uint8_t motor_id, bool& service_success)
  {
    auto motor = MockDynamixelManager::instance().getMotor(motor_id);
    if (!motor) {
      return false;
    }
    motor->setHardwareError(ERROR_OVERHEATING);
    std::this_thread::sleep_for(400ms);

    auto reboot_client =
        tester_node_->create_test_client<std_srvs::srv::Trigger>("/athena_flipper_interface_node/reboot");
    if (!reboot_client->wait_for_service(*executor_, 5s)) {
      return false;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto resp = callService<std_srvs::srv::Trigger>(reboot_client, request, 10s);
    if (!resp) {
      return false;
    }
    service_success = resp->success;
    return true;
  }
};

// Sanity: without a RAM-resident model the rewrite path under test is not entered, and everything
// below would pass regardless.
TEST_F(RebootIndirectAddressTest, FlippersUseRamResidentIndirectAddresses)
{
  std::this_thread::sleep_for(300ms);
  for (uint8_t id : FLIPPER_MOTOR_IDS) {
    auto motor = MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr);
    EXPECT_EQ(motor->getModelNumber(), RAM_INDIRECT_MODEL_NUMBER)
        << "Motor " << static_cast<int>(id)
        << " is not the RAM-indirect model; the reboot rewrite path is skipped entirely and the "
           "tests below cannot fail.";
  }
}

// After a reboot on a RAM-resident model, the interface must be able to come back to ACTIVE.
// Activation runs resetGoalStateAndVerify(), which writes every goal register and reads it back:
// if the control-write or command-read pointers were left dangling, the read-back cannot match and
// activation fails. This is the end-to-end guard on the rewrite list.
TEST_F(RebootIndirectAddressTest, RebootKeepsGoalWriteAndVerifyWorking)
{
  std::this_thread::sleep_for(400ms);

  bool reboot_ok = false;
  ASSERT_TRUE(rebootAfterErrorOn(FLIPPER_FL_ID, reboot_ok)) << "Reboot service call failed";
  ASSERT_TRUE(reboot_ok) << "Reboot itself failed";
  std::this_thread::sleep_for(400ms);

  auto motor = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  ASSERT_NE(motor, nullptr);
  ASSERT_GT(motor->getRebootCount(), 0) << "Test premise: the faulted motor should have been rebooted";
  EXPECT_EQ(motor->getHardwareError(), 0) << "Hardware error should have been cleared by the reboot";

  // The hardware error took the component out of ACTIVE. Bringing it back exercises the goal
  // write + read-back over the freshly rewritten indirect mappings.
  for (const auto& [state_id, label] :
       std::vector<std::pair<uint8_t, std::string>>{{lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive"},
                                                    {lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active"}}) {
    EXPECT_TRUE(setHardwareComponentState("athena_flipper_interface", state_id, label))
        << "Could not return the flipper interface to '" << label
        << "' after a reboot. Its indirect-address mappings were probably not restored, so "
           "the goal write-and-verify during activation reads back the wrong registers.";
  }

  // The rebooted motor must follow a command over that restored path.
  std::this_thread::sleep_for(300ms);
  ASSERT_TRUE(loadAndActivateController("flipper_position_controller"));
  std::this_thread::sleep_for(200ms);

  auto cmd_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/flipper_position_controller/commands", 10);
  ASSERT_TRUE(waitForSubscribers(cmd_pub, 5s));

  const double start_position = motor->getJointPosition();
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = std::vector<double>(FLIPPER_MOTOR_IDS.size(), start_position + 0.5);
  cmd_pub->publish(cmd);
  std::this_thread::sleep_for(1s);

  EXPECT_GT(std::abs(motor->getJointPosition() - start_position), 0.05)
      << "The rebooted motor did not follow a position command; the control write manager's "
         "indirect mapping was not restored.";
}

}  // namespace dynamixel_ros_control::test
