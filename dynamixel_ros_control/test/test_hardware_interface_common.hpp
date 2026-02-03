// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <gtest/gtest.h>

#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/load_controller.hpp>
#include <controller_manager_msgs/srv/configure_controller.hpp>
#include <controller_manager_msgs/srv/list_hardware_interfaces.hpp>
#include <controller_manager_msgs/srv/set_hardware_component_state.hpp>
#include <hardware_interface/introspection.hpp>
#include <hector_testing_utils/hector_testing_utils.hpp>
#include <hector_transmission_interface_msgs/srv/adjust_transmission_offsets.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_helpers.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <dynamixel_ros_control/mock_dynamixel.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <mutex>

namespace dynamixel_ros_control::test {

using controller_manager_msgs::srv::ConfigureController;
using controller_manager_msgs::srv::ListControllers;
using controller_manager_msgs::srv::ListHardwareInterfaces;
using controller_manager_msgs::srv::LoadController;
using controller_manager_msgs::srv::SetHardwareComponentState;
using controller_manager_msgs::srv::SwitchController;
using hector_testing_utils::HectorTestFixture;
using namespace std::chrono_literals;

// ============================================================================
// Constants
// ============================================================================

constexpr int kSchedPriority = 50;

// Motor IDs from URDF
constexpr uint8_t ARM_JOINT_1_ID = 11;
constexpr uint8_t ARM_JOINT_2_ID = 12;
constexpr uint8_t ARM_JOINT_3_ID = 13;
constexpr uint8_t ARM_JOINT_4_ID = 14;
constexpr uint8_t ARM_JOINT_5_ID = 15;
constexpr uint8_t ARM_JOINT_6_ID = 16;
constexpr uint8_t ARM_JOINT_7_ID = 17;
constexpr uint8_t GRIPPER_ID = 18;
constexpr uint8_t FLIPPER_FL_ID = 1;
constexpr uint8_t FLIPPER_FR_ID = 2;
constexpr uint8_t FLIPPER_BL_ID = 3;
constexpr uint8_t FLIPPER_BR_ID = 4;

// LED colors from common.hpp
constexpr uint8_t COLOR_PINK_R = 255, COLOR_PINK_G = 175, COLOR_PINK_B = 193;
constexpr uint8_t COLOR_GREEN_R = 0, COLOR_GREEN_G = 255, COLOR_GREEN_B = 0;
constexpr uint8_t COLOR_BLUE_R = 0, COLOR_BLUE_G = 0, COLOR_BLUE_B = 255;
constexpr uint8_t COLOR_ORANGE_R = 255, COLOR_ORANGE_G = 165, COLOR_ORANGE_B = 0;
constexpr uint8_t COLOR_RED_R = 255, COLOR_RED_G = 0, COLOR_RED_B = 0;

// Motor ID vectors for convenient iteration
inline const std::vector<uint8_t> ARM_MOTOR_IDS = {ARM_JOINT_1_ID, ARM_JOINT_2_ID, ARM_JOINT_3_ID, ARM_JOINT_4_ID,
                                                   ARM_JOINT_5_ID, ARM_JOINT_6_ID, ARM_JOINT_7_ID};
inline const std::vector<uint8_t> FLIPPER_MOTOR_IDS = {FLIPPER_FL_ID, FLIPPER_FR_ID, FLIPPER_BL_ID, FLIPPER_BR_ID};

// ============================================================================
// Utility Functions
// ============================================================================

inline std::string load_file(const std::string& path)
{
  std::ifstream file(path);
  if (!file.is_open())
    return "";
  return std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
}

inline std::unordered_map<std::string, std::string> states_from_list(const ListControllers::Response& msg)
{
  std::unordered_map<std::string, std::string> states;
  for (const auto& c : msg.controller) {
    states[c.name] = c.state;
  }
  return states;
}

// ============================================================================
// Test Fixture
// ============================================================================

class HardwareInterfaceTest : public HectorTestFixture
{
protected:
  void SetUp() override
  {
    // Create temporary HOME directory to isolate tests from persistent calibration files
    // The AdjustableOffsetTransmission stores offsets in ~/.ros/dynamic_offset_transmissions/
    // Without isolation, pre-existing offsets can cause motors to travel large distances
    // at startup, leading to measurement errors in timing-sensitive tests.
    char temp_dir[] = "/tmp/dynamixel_test_home_XXXXXX";
    test_home_dir_ = mkdtemp(temp_dir);
    original_home_ = std::getenv("HOME");
    setenv("HOME", test_home_dir_.c_str(), 1);

    HectorTestFixture::SetUp();

    // Config paths
    controllers_yaml_ = std::string(TEST_CONFIG_DIR) + "/controllers.yaml";
    urdf_path_ = std::string(TEST_CONFIG_DIR) + "/athena.urdf";

    const std::string urdf = load_file(urdf_path_);
    ASSERT_FALSE(urdf.empty()) << "Failed to load URDF from " << urdf_path_;

    // Setup CM options
    auto yaml_options = hector_testing_utils::node_options_from_yaml(controllers_yaml_);
    auto cm_options = controller_manager::get_cm_node_options();
    cm_options.arguments(yaml_options.arguments());
    cm_options.automatically_declare_parameters_from_overrides(true);

    // Introspection registry
    INITIALIZE_ROS2_CONTROL_INTROSPECTION_REGISTRY(tester_node_, hardware_interface::DEFAULT_INTROSPECTION_TOPIC,
                                                   hardware_interface::DEFAULT_REGISTRY_KEY);

    // Executor and CM
    // IMPORTANT: Set activate_components=true (3rd param) to auto-activate hardware interfaces
    cm_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    controller_manager_ = std::make_shared<controller_manager::ControllerManager>(cm_executor_, urdf, true,
                                                                                  "controller_manager", "", cm_options);

    cm_executor_->add_node(controller_manager_);

    // Setup update loop
    const bool use_sim_time = controller_manager_->get_parameter_or("use_sim_time", false);
    const int thread_priority = controller_manager_->get_parameter_or<int>("thread_priority", kSchedPriority);

    cm_running_ = true;
    cm_spin_thread_ = std::thread([this]() { cm_executor_->spin(); });

    start_update_loop(use_sim_time, thread_priority);

    // Setup service clients
    list_client_ = tester_node_->create_test_client<ListControllers>("/controller_manager/list_controllers");
    ASSERT_TRUE(list_client_->wait_for_service(*executor_, 10s));

    switch_client_ = tester_node_->create_test_client<SwitchController>("/controller_manager/switch_controller");
    ASSERT_TRUE(switch_client_->wait_for_service(*executor_, 10s));

    load_client_ = tester_node_->create_test_client<LoadController>("/controller_manager/load_controller");
    ASSERT_TRUE(load_client_->wait_for_service(*executor_, 10s));

    config_client_ = tester_node_->create_test_client<ConfigureController>("/controller_manager/configure_controller");
    ASSERT_TRUE(config_client_->wait_for_service(*executor_, 10s));

    list_hw_client_ =
        tester_node_->create_test_client<ListHardwareInterfaces>("/controller_manager/list_hardware_interfaces");
    ASSERT_TRUE(list_hw_client_->wait_for_service(*executor_, 10s));

    // Wait for hardware interfaces to be active
    ASSERT_TRUE(waitForHardwareInterfacesActive(10s)) << "Hardware interfaces failed to become active";
  }

  void TearDown() override
  {
    cm_running_ = false;
    if (cm_update_thread_.joinable())
      cm_update_thread_.join();
    if (cm_executor_)
      cm_executor_->cancel();
    if (cm_spin_thread_.joinable())
      cm_spin_thread_.join();

    controller_manager_.reset();
    cm_executor_.reset();

    dynamixel_ros_control::MockDynamixelManager::instance().reset();

    HectorTestFixture::TearDown();

    // Restore original HOME and clean up temporary directory
    if (!original_home_.empty()) {
      setenv("HOME", original_home_.c_str(), 1);
    }
    if (!test_home_dir_.empty()) {
      std::filesystem::remove_all(test_home_dir_);
    }
  }

  void start_update_loop(bool use_sim_time, int thread_priority)
  {
    cm_update_thread_ = std::thread([this, use_sim_time, thread_priority]() {
      if (!realtime_tools::configure_sched_fifo(thread_priority)) {
        // Warning logged
      }

      controller_manager_->get_clock()->wait_until_started();

      const auto period = std::chrono::nanoseconds(1'000'000'000 / controller_manager_->get_update_rate());
      rclcpp::Time previous_time = controller_manager_->get_trigger_clock()->now();

      std::chrono::steady_clock::time_point next_iteration_time{std::chrono::steady_clock::now()};

      while (cm_running_ && rclcpp::ok()) {
        const auto current_time = controller_manager_->get_trigger_clock()->now();
        const auto measured_period = current_time - previous_time;
        previous_time = current_time;

        // Update mock physics
        double dt = measured_period.seconds();
        if (dt > 0.0) {
          dynamixel_ros_control::MockDynamixelManager::instance().update(dt);
        }

        controller_manager_->read(current_time, measured_period);
        controller_manager_->update(current_time, measured_period);
        controller_manager_->write(current_time, measured_period);

        if (use_sim_time) {
          controller_manager_->get_clock()->sleep_until(current_time + period);
        } else {
          next_iteration_time += period;
          std::this_thread::sleep_until(next_iteration_time);
        }
      }
    });
  }

  // Wait for hardware interfaces to be in active state
  bool waitForHardwareInterfacesActive(std::chrono::seconds timeout)
  {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      auto request = std::make_shared<ListHardwareInterfaces::Request>();
      hector_testing_utils::ServiceCallOptions options;
      options.service_timeout = 5s;
      options.response_timeout = 5s;
      auto resp = hector_testing_utils::call_service<ListHardwareInterfaces>(list_hw_client_->get(), request,
                                                                             *executor_, options);

      if (resp) {
        // Check if command interfaces are available (indicates HW is active)
        bool arm_interfaces_found = false;
        bool flipper_interfaces_found = false;
        for (const auto& iface : resp->command_interfaces) {
          if (iface.name.find("arm_joint_1/position") != std::string::npos) {
            arm_interfaces_found = true;
          }
          if (iface.name.find("flipper_fl_joint/position") != std::string::npos) {
            flipper_interfaces_found = true;
          }
        }
        if (arm_interfaces_found && flipper_interfaces_found) {
          return true;
        }
      }
      std::this_thread::sleep_for(100ms);
      executor_->spin_some();
    }
    return false;
  }

  // Load and activate a controller
  bool loadAndActivateController(const std::string& controller_name, std::chrono::seconds timeout = 10s)
  {
    hector_testing_utils::ServiceCallOptions options;
    options.service_timeout = 5s;
    options.response_timeout = 5s;

    // Load
    auto load_req = std::make_shared<LoadController::Request>();
    load_req->name = controller_name;
    auto load_resp =
        hector_testing_utils::call_service<LoadController>(load_client_->get(), load_req, *executor_, options);
    if (!load_resp || !load_resp->ok) {
      std::cerr << "Failed to load controller: " << controller_name << std::endl;
      return false;
    }

    // Configure
    auto config_req = std::make_shared<ConfigureController::Request>();
    config_req->name = controller_name;
    auto config_resp =
        hector_testing_utils::call_service<ConfigureController>(config_client_->get(), config_req, *executor_, options);
    if (!config_resp || !config_resp->ok) {
      std::cerr << "Failed to configure controller: " << controller_name << std::endl;
      return false;
    }

    // Activate
    auto switch_req = std::make_shared<SwitchController::Request>();
    switch_req->activate_controllers = {controller_name};
    switch_req->strictness = SwitchController::Request::STRICT;
    auto switch_resp =
        hector_testing_utils::call_service<SwitchController>(switch_client_->get(), switch_req, *executor_, options);
    if (!switch_resp || !switch_resp->ok) {
      std::cerr << "Failed to activate controller: " << controller_name << std::endl;
      return false;
    }

    // Wait for active state
    return waitForControllerState(controller_name, "active", timeout);
  }

  bool waitForControllerState(const std::string& controller_name, const std::string& expected_state,
                              std::chrono::seconds timeout = 10s)
  {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      auto resp = list_controllers();
      if (resp) {
        auto states = states_from_list(*resp);
        if (states.count(controller_name) && states[controller_name] == expected_state) {
          return true;
        }
      }
      std::this_thread::sleep_for(50ms);
      executor_->spin_some();
    }
    return false;
  }

  ListControllers::Response::SharedPtr list_controllers()
  {
    auto request = std::make_shared<ListControllers::Request>();
    hector_testing_utils::ServiceCallOptions options;
    options.service_timeout = 5s;
    options.response_timeout = 5s;
    return hector_testing_utils::call_service<ListControllers>(list_client_->get(), request, *executor_, options);
  }

  bool switch_controllers(const std::vector<std::string>& activate, const std::vector<std::string>& deactivate)
  {
    auto request = std::make_shared<SwitchController::Request>();
    request->activate_controllers = activate;
    request->deactivate_controllers = deactivate;
    request->strictness = SwitchController::Request::STRICT;

    hector_testing_utils::ServiceCallOptions options;
    options.service_timeout = 10s;
    options.response_timeout = 10s;
    auto resp =
        hector_testing_utils::call_service<SwitchController>(switch_client_->get(), request, *executor_, options);
    return resp && resp->ok;
  }

  // Helper to verify LED color on a motor
  void verifyLEDColor(uint8_t motor_id, uint8_t expected_r, uint8_t expected_g, uint8_t expected_b)
  {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(motor_id);
    ASSERT_NE(motor, nullptr) << "Motor ID " << (int) motor_id << " not found";
    EXPECT_EQ(motor->getLedRed(), expected_r) << "Motor " << (int) motor_id << " LED Red mismatch";
    EXPECT_EQ(motor->getLedGreen(), expected_g) << "Motor " << (int) motor_id << " LED Green mismatch";
    EXPECT_EQ(motor->getLedBlue(), expected_b) << "Motor " << (int) motor_id << " LED Blue mismatch";
  }

  // Helper to verify all arm motors have same LED color
  void verifyArmLEDColor(uint8_t r, uint8_t g, uint8_t b)
  {
    for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
      verifyLEDColor(id, r, g, b);
    }
  }

  // Publish command and wait for subscriber
  template <typename MsgT>
  void publishAndWait(typename rclcpp::Publisher<MsgT>::SharedPtr pub, const MsgT& msg,
                      std::chrono::seconds sub_timeout = 5s)
  {
    auto deadline = std::chrono::steady_clock::now() + sub_timeout;
    while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
      std::this_thread::sleep_for(50ms);
      executor_->spin_some();
    }
    ASSERT_GT(pub->get_subscription_count(), 0u) << "No subscribers for topic";
    pub->publish(msg);
  }

  // Helper to wait for publisher to have subscribers
  bool waitForSubscribers(const rclcpp::PublisherBase::SharedPtr& pub, std::chrono::seconds timeout = 5s)
  {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
      std::this_thread::sleep_for(50ms);
      executor_->spin_some();
    }
    return pub->get_subscription_count() > 0;
  }

  // Helper to create and wait for E-Stop publisher
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr createEStopPublisher()
  {
    auto estop_pub = tester_node_->create_publisher<std_msgs::msg::Bool>("/soft_e_stop", 10);
    EXPECT_TRUE(waitForSubscribers(estop_pub, 5s)) << "No subscriber for e-stop topic";
    return estop_pub;
  }

  // Helper to create torque service client
  std::shared_ptr<hector_testing_utils::TestClient<std_srvs::srv::SetBool>>
  createTorqueClient(const std::string& interface_name = "athena_arm_interface")
  {
    auto client = tester_node_->create_test_client<std_srvs::srv::SetBool>("/" + interface_name + "/set_torque");
    EXPECT_TRUE(client->wait_for_service(*executor_, 5s)) << "Torque service not available";
    return client;
  }

  // Helper to set torque state
  bool setTorque(const std::shared_ptr<hector_testing_utils::TestClient<std_srvs::srv::SetBool>>& client, bool enable)
  {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enable;

    hector_testing_utils::ServiceCallOptions options;
    options.service_timeout = 5s;
    options.response_timeout = 5s;
    auto resp = hector_testing_utils::call_service<std_srvs::srv::SetBool>(client->get(), request, *executor_, options);
    return resp && resp->success;
  }

  // Member variables
  std::string controllers_yaml_;
  std::string urdf_path_;
  std::string test_home_dir_;
  std::string original_home_;

  std::atomic<bool> cm_running_{false};
  std::shared_ptr<rclcpp::Executor> cm_executor_;
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  std::thread cm_spin_thread_;
  std::thread cm_update_thread_;

  std::shared_ptr<hector_testing_utils::TestClient<ListControllers>> list_client_;
  std::shared_ptr<hector_testing_utils::TestClient<SwitchController>> switch_client_;
  std::shared_ptr<hector_testing_utils::TestClient<LoadController>> load_client_;
  std::shared_ptr<hector_testing_utils::TestClient<ConfigureController>> config_client_;
  std::shared_ptr<hector_testing_utils::TestClient<ListHardwareInterfaces>> list_hw_client_;
};

}  // namespace dynamixel_ros_control::test
