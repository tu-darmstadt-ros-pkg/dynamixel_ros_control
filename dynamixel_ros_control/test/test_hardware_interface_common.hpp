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
#include <controller_manager_msgs/srv/list_hardware_components.hpp>
#include <controller_manager_msgs/srv/set_hardware_component_state.hpp>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/introspection.hpp>
#include <hector_testing_utils/hector_testing_utils.hpp>
#include <hector_transmission_interface_msgs/srv/adjust_transmission_offsets.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_helpers.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <dynamixel_ros_control/mock_dynamixel.hpp>

#include "test_home.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <map>
#include <filesystem>
#include <fstream>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <mutex>
#include <optional>

namespace dynamixel_ros_control::test {

using controller_manager_msgs::srv::ConfigureController;
using controller_manager_msgs::srv::ListControllers;
using controller_manager_msgs::srv::ListHardwareComponents;
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
constexpr uint8_t COLOR_RED_R = 255, COLOR_RED_G = 0, COLOR_RED_B = 0;

// Motor ID vectors for convenient iteration
inline const std::vector<uint8_t> ARM_MOTOR_IDS = {ARM_JOINT_1_ID, ARM_JOINT_2_ID, ARM_JOINT_3_ID, ARM_JOINT_4_ID,
                                                   ARM_JOINT_5_ID, ARM_JOINT_6_ID, ARM_JOINT_7_ID};
inline const std::vector<uint8_t> FLIPPER_MOTOR_IDS = {FLIPPER_FL_ID, FLIPPER_FR_ID, FLIPPER_BL_ID, FLIPPER_BR_ID};

// ============================================================================
// Utility Functions
// ============================================================================

/// @p timeout stretched by the factor the build sets for sanitizers, which slow everything down
constexpr std::chrono::seconds scaled(const std::chrono::seconds timeout)
{
  return timeout * DXL_TEST_TIMEOUT_FACTOR;
}

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

/// Puts the motors of @p info's joints on the simulated bus, unless an earlier step put them there
inline std::vector<uint8_t> addMotorsOf(const hardware_interface::HardwareInfo& info)
{
  std::vector<uint8_t> ids;
  for (const auto& joint : info.joints) {
    const auto id = joint.parameters.find("id");
    if (id == joint.parameters.end()) {
      continue;
    }
    const auto model = joint.parameters.find("model_number");
    ids.push_back(static_cast<uint8_t>(std::stoi(id->second)));
    MockDynamixelManager::instance().addMotor(ids.back(), model == joint.parameters.end() ?
                                                              DEFAULT_MOCK_MODEL_NUMBER :
                                                              static_cast<uint16_t>(std::stoi(model->second)));
  }
  return ids;
}

/// Puts the motors of every component with use_dummy on the simulated bus
inline void addDummyMotors(const std::vector<hardware_interface::HardwareInfo>& components)
{
  for (const auto& info : components) {
    const auto use_dummy = info.hardware_parameters.find("use_dummy");
    if (use_dummy != info.hardware_parameters.end() && use_dummy->second == "true") {
      addMotorsOf(info);
    }
  }
}

// ============================================================================
// Test Fixture
// ============================================================================

class HardwareInterfaceTest : public HectorTestFixture
{
protected:
  void SetUp() override
  {
    test_home_.emplace();
    HectorTestFixture::SetUp();

    // Config paths
    controllers_yaml_ = std::string(TEST_CONFIG_DIR) + "/controllers.yaml";
    urdf_path_ = std::string(TEST_CONFIG_DIR) + "/athena.urdf";

    std::string urdf = load_file(urdf_path_);
    ASSERT_FALSE(urdf.empty()) << "Failed to load URDF from " << urdf_path_;

    // Hook allowing subclasses to mutate the URDF (e.g. inject hardware parameters) before the
    // controller manager is built. Defaults to identity, so existing tests are unaffected.
    urdf = transformUrdf(std::move(urdf));

    // Hook allowing subclasses to put the bus into a state the interface then has to come up
    // against. Motors created here survive, because addMotor() leaves an existing id alone.
    beforeControllerManager();
    addDummyMotors(hardware_interface::parse_control_resources_from_urdf(urdf));

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

    // The controller manager does not forward its own --params-file to controller nodes, so point
    // every configured controller at the YAML via <controller>.params_file.
    const std::string type_suffix = ".type";
    for (const auto& name : controller_manager_->list_parameters({}, 0).names) {
      if (name.size() > type_suffix.size() &&
          name.compare(name.size() - type_suffix.size(), type_suffix.size(), type_suffix) == 0) {
        const std::string controller = name.substr(0, name.size() - type_suffix.size());
        if (controller.find('.') == std::string::npos) {
          controller_manager_->declare_parameter(controller + ".params_file", controllers_yaml_);
        }
      }
    }

    cm_executor_->add_node(controller_manager_);

    // Setup update loop
    const bool use_sim_time = controller_manager_->get_parameter_or("use_sim_time", false);
    const int thread_priority = controller_manager_->get_parameter_or<int>("thread_priority", kSchedPriority);

    cm_running_ = true;
    cm_spin_thread_ = std::thread([this]() { cm_executor_->spin(); });

    // A component that failed on_init leaves interfaces the resource manager cannot read, and the
    // update loop dies on the first one. There is nothing to drive in that case anyway.
    if (run_update_loop_) {
      start_update_loop(use_sim_time, thread_priority);
    }

    // Setup service clients
    list_client_ = tester_node_->create_test_client<ListControllers>("/controller_manager/list_controllers");
    switch_client_ = tester_node_->create_test_client<SwitchController>("/controller_manager/switch_controller");
    load_client_ = tester_node_->create_test_client<LoadController>("/controller_manager/load_controller");
    config_client_ = tester_node_->create_test_client<ConfigureController>("/controller_manager/configure_controller");
    list_hw_client_ =
        tester_node_->create_test_client<ListHardwareInterfaces>("/controller_manager/list_hardware_interfaces");
    list_hw_components_client_ =
        tester_node_->create_test_client<ListHardwareComponents>("/controller_manager/list_hardware_components");

    // A component that refuses the robot description in on_init takes the resource manager with it,
    // so the controller manager waits for a new description instead of offering services. A fixture
    // expecting that clears require_controller_manager_ rather than waiting out every client.
    if (require_controller_manager_) {
      ASSERT_TRUE(list_client_->wait_for_service(*executor_, scaled(10s)));
      ASSERT_TRUE(switch_client_->wait_for_service(*executor_, scaled(10s)));
      ASSERT_TRUE(load_client_->wait_for_service(*executor_, scaled(10s)));
      ASSERT_TRUE(config_client_->wait_for_service(*executor_, scaled(10s)));
      ASSERT_TRUE(list_hw_client_->wait_for_service(*executor_, scaled(10s)));
      ASSERT_TRUE(list_hw_components_client_->wait_for_service(*executor_, scaled(10s)));
    }

    // Wait for hardware interfaces to be active. Subclasses that deliberately break configuration
    // (to assert the interface refuses to come up) clear require_hardware_active_ instead.
    if (require_hardware_active_) {
      ASSERT_TRUE(waitForHardwareInterfacesActive(10s)) << "Hardware interfaces failed to become active";
    }
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
    test_home_.reset();
  }

  // Override point: transform the loaded URDF string before the controller manager is constructed.
  // The base implementation returns it unchanged.
  virtual std::string transformUrdf(std::string urdf)
  {
    return urdf;
  }

  // Override point: prepare the bus before the controller manager is constructed, which is what
  // runs on_init, on_configure and activation. The base implementation does nothing.
  virtual void beforeControllerManager() {}

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

  /// @brief Calls @p client with symmetric timeouts on the fixture executor.
  /// @return the response, or nullptr if the service did not answer in time.
  template <typename Srv>
  typename Srv::Response::SharedPtr callService(const std::shared_ptr<hector_testing_utils::TestClient<Srv>>& client,
                                                const typename Srv::Request::SharedPtr& request,
                                                std::chrono::seconds timeout = 5s)
  {
    hector_testing_utils::ServiceCallOptions options;
    options.service_timeout = scaled(timeout);
    options.response_timeout = scaled(timeout);
    return hector_testing_utils::call_service<Srv>(client->get(), request, *executor_, options);
  }

  /// @brief Drive one hardware component to a lifecycle state.
  /// @return whether the controller manager accepted the transition. A refused transition is a
  ///         result for the caller to judge. A call that goes unanswered fails the test here.
  bool setHardwareComponentState(const std::string& component_name, uint8_t state_id, const std::string& label)
  {
    if (!hw_state_client_) {
      hw_state_client_ = tester_node_->create_test_client<SetHardwareComponentState>("/controller_manager/"
                                                                                     "set_hardware_component_state");
      EXPECT_TRUE(hw_state_client_->wait_for_service(*executor_, scaled(5s)));
    }
    auto request = std::make_shared<SetHardwareComponentState::Request>();
    request->name = component_name;
    request->target_state.id = state_id;
    request->target_state.label = label;
    auto resp = callService<SetHardwareComponentState>(hw_state_client_, request, 10s);
    EXPECT_NE(resp, nullptr) << "set_hardware_component_state('" << label << "') call failed";
    return resp != nullptr && resp->ok;
  }

  /// @brief Call an interface's ~/reboot service.
  /// @return the response, so callers keep the service's own message for their failure text.
  ///         Returns nullptr if the service did not answer.
  std_srvs::srv::Trigger::Response::SharedPtr rebootMotors(const std::string& interface_name = "athena_arm_interface",
                                                           std::chrono::seconds timeout = 10s)
  {
    auto client = tester_node_->create_test_client<std_srvs::srv::Trigger>("/" + interface_name + "_node/reboot");
    EXPECT_TRUE(client->wait_for_service(*executor_, scaled(5s))) << "Reboot service not available";
    return callService<std_srvs::srv::Trigger>(client, std::make_shared<std_srvs::srv::Trigger::Request>(), timeout);
  }

  /// @brief Lifecycle state label of one hardware component, or "" if it is not listed.
  ///
  /// Ask this, not "are the interfaces exported": interfaces are exported during on_init, long
  /// before on_configure runs, so their presence is true even for a component whose configuration
  /// failed outright.
  std::string hardwareComponentState(const std::string& component_name)
  {
    auto request = std::make_shared<ListHardwareComponents::Request>();
    auto resp = callService<ListHardwareComponents>(list_hw_components_client_, request);
    if (!resp) {
      return "";
    }
    for (const auto& component : resp->component) {
      if (component.name == component_name) {
        return component.state.label;
      }
    }
    return "";
  }

  /// Waits until @p component_name reports the lifecycle state @p label, for example after on_error
  bool waitForComponentState(const std::string& component_name, const std::string& label,
                             const std::chrono::seconds timeout = 10s)
  {
    const auto deadline = std::chrono::steady_clock::now() + scaled(timeout);
    while (hardwareComponentState(component_name) != label) {
      if (std::chrono::steady_clock::now() >= deadline) {
        return false;
      }
      std::this_thread::sleep_for(50ms);
    }
    return true;
  }

  // Wait for both hardware components to reach the ACTIVE lifecycle state.
  bool waitForHardwareInterfacesActive(std::chrono::seconds timeout)
  {
    auto deadline = std::chrono::steady_clock::now() + scaled(timeout);
    while (std::chrono::steady_clock::now() < deadline) {
      if (hardwareComponentState("athena_arm_interface") == "active" &&
          hardwareComponentState("athena_flipper_interface") == "active") {
        return true;
      }
      std::this_thread::sleep_for(100ms);
      executor_->spin_some();
    }
    return false;
  }

  // Load and configure a controller without activating it. Use when the activation itself is the
  // thing under test (e.g. a switch that is expected to fail).
  bool loadController(const std::string& controller_name)
  {
    auto load_req = std::make_shared<LoadController::Request>();
    load_req->name = controller_name;
    auto load_resp = callService<LoadController>(load_client_, load_req);
    if (!load_resp || !load_resp->ok) {
      std::cerr << "Failed to load controller: " << controller_name << std::endl;
      return false;
    }

    auto config_req = std::make_shared<ConfigureController::Request>();
    config_req->name = controller_name;
    auto config_resp = callService<ConfigureController>(config_client_, config_req);
    if (!config_resp || !config_resp->ok) {
      std::cerr << "Failed to configure controller: " << controller_name << std::endl;
      return false;
    }
    return true;
  }

  // Load and activate a controller
  bool loadAndActivateController(const std::string& controller_name, std::chrono::seconds timeout = 10s)
  {
    if (!loadController(controller_name)) {
      return false;
    }

    auto switch_req = switchRequest();
    switch_req->activate_controllers = {controller_name};
    auto switch_resp = callService<SwitchController>(switch_client_, switch_req);
    if (!switch_resp || !switch_resp->ok) {
      std::cerr << "Failed to activate controller: " << controller_name << std::endl;
      return false;
    }

    return waitForControllerState(controller_name, "active", timeout);
  }

  bool waitForControllerState(const std::string& controller_name, const std::string& expected_state,
                              std::chrono::seconds timeout = 10s)
  {
    auto deadline = std::chrono::steady_clock::now() + scaled(timeout);
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
    return callService<ListControllers>(list_client_, std::make_shared<ListControllers::Request>());
  }

  /// A STRICT switch request that the controller manager performs on its control thread. Without
  /// activate_asap it runs perform_command_mode_switch on the service thread while the control thread
  /// keeps calling read() and write(), which the plugin requires never to overlap.
  static SwitchController::Request::SharedPtr switchRequest()
  {
    auto request = std::make_shared<SwitchController::Request>();
    request->strictness = SwitchController::Request::STRICT;
    request->activate_asap = true;
    return request;
  }

  bool switch_controllers(const std::vector<std::string>& activate, const std::vector<std::string>& deactivate)
  {
    auto request = switchRequest();
    request->activate_controllers = activate;
    request->deactivate_controllers = deactivate;
    auto resp = callService<SwitchController>(switch_client_, request, 10s);
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

  /// Commands every arm joint to @p target until each is within @p tolerance of it
  bool armReaches(const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& pub, const double target,
                  const double tolerance = 0.1)
  {
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(ARM_MOTOR_IDS.size(), target);
    const auto at_target = [&]() {
      return std::all_of(ARM_MOTOR_IDS.begin(), ARM_MOTOR_IDS.end(), [&](const uint8_t id) {
        return std::abs(MockDynamixelManager::instance().getMotor(id)->getJointPosition() - target) < tolerance;
      });
    };
    const auto deadline = std::chrono::steady_clock::now() + scaled(10s);
    while (!at_target()) {
      if (std::chrono::steady_clock::now() >= deadline) {
        return false;
      }
      pub->publish(cmd);
      std::this_thread::sleep_for(50ms);
    }
    return true;
  }

  // Helper to wait for publisher to have subscribers
  bool waitForSubscribers(const rclcpp::PublisherBase::SharedPtr& pub, std::chrono::seconds timeout = 5s)
  {
    auto deadline = std::chrono::steady_clock::now() + scaled(timeout);
    while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
      std::this_thread::sleep_for(50ms);
      executor_->spin_some();
    }
    return pub->get_subscription_count() > 0;
  }

  /// Makes joints limp the way an operator does, by activating a controller that claims their
  /// torque_off interfaces. A joint cannot be limp and commanded at once, so the controllers
  /// commanding the same joints stop in the same switch.
  bool limp(const std::string& torque_off_controller = "arm_torque_off_controller")
  {
    static const std::map<std::string, std::vector<std::string>> commanded_by = {
        {"arm_torque_off_controller", {"arm_", "gripper_"}},
        {"arm_joints_torque_off_controller", {"arm_"}},
        {"flipper_torque_off_controller", {"flipper_"}},
        {"gripper_torque_off_controller", {"gripper_"}}};
    const auto list = list_controllers();
    if (!list) {
      return false;
    }
    const auto states = states_from_list(*list);
    if (states.count(torque_off_controller) == 0 && !loadController(torque_off_controller)) {
      return false;
    }
    std::vector<std::string> stop;
    for (const auto& [name, state] : states) {
      for (const auto& prefix : commanded_by.at(torque_off_controller)) {
        if (state == "active" && name != torque_off_controller && name.rfind(prefix, 0) == 0) {
          stop.push_back(name);
          break;
        }
      }
    }
    return switch_controllers({torque_off_controller}, stop) &&
           waitForControllerState(torque_off_controller, "active", 5s);
  }

  /// Ends a torque_off claim in one switch with position controllers on its joints, the way an
  /// operator gives torque back: a released claim alone leaves the joints limp.
  bool releaseLimp(const std::string& torque_off_controller = "arm_torque_off_controller")
  {
    static const std::map<std::string, std::vector<std::string>> holders = {
        {"arm_torque_off_controller", {"arm_position_controller", "gripper_position_controller"}},
        {"arm_joints_torque_off_controller", {"arm_position_controller"}},
        {"flipper_torque_off_controller", {"flipper_position_controller"}},
        {"gripper_torque_off_controller", {"gripper_position_controller"}}};
    const auto list = list_controllers();
    if (!list) {
      return false;
    }
    const auto states = states_from_list(*list);
    for (const auto& name : holders.at(torque_off_controller)) {
      if (states.count(name) == 0 && !loadController(name)) {
        return false;
      }
    }
    return switch_controllers(holders.at(torque_off_controller), {torque_off_controller}) &&
           waitForControllerState(torque_off_controller, "inactive", 5s);
  }

  // Cleared by subclasses that expect configuration or activation to fail.
  bool require_hardware_active_{true};

  // Cleared by subclasses that expect the robot description itself to be refused, which leaves the
  // controller manager without services to offer.
  bool require_controller_manager_{true};

  // Cleared by subclasses whose component fails on_init: read()/update()/write() then run against
  // interfaces that were never exported.
  bool run_update_loop_{true};

  // Member variables
  std::string controllers_yaml_;
  std::string urdf_path_;
  std::optional<TestHome> test_home_;

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
  std::shared_ptr<hector_testing_utils::TestClient<ListHardwareComponents>> list_hw_components_client_;
  /// Created on first use by setHardwareComponentState().
  std::shared_ptr<hector_testing_utils::TestClient<SetHardwareComponentState>> hw_state_client_;
};

}  // namespace dynamixel_ros_control::test
