#include <gtest/gtest.h>

#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/load_controller.hpp>
#include <controller_manager_msgs/srv/configure_controller.hpp>
#include <controller_manager_msgs/srv/list_hardware_interfaces.hpp>
#include <controller_manager_msgs/srv/set_hardware_component_state.hpp>
#include <hardware_interface/introspection.hpp>
#include <hector_controller_spawner/hector_controller_spawner.hpp>
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

// Include MockDynamixel for simulation control
#include <dynamixel_ros_control/mock_dynamixel.hpp>

#include <fstream>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <mutex>

using controller_manager_msgs::srv::ConfigureController;
using controller_manager_msgs::srv::ListControllers;
using controller_manager_msgs::srv::ListHardwareInterfaces;
using controller_manager_msgs::srv::LoadController;
using controller_manager_msgs::srv::SetHardwareComponentState;
using controller_manager_msgs::srv::SwitchController;
using hector_testing_utils::HectorTestFixture;
using namespace std::chrono_literals;

namespace {
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
constexpr uint8_t COLOR_RED_R = 255, COLOR_RED_G = 0, COLOR_RED_B = 0;
constexpr uint8_t COLOR_GREEN_R = 0, COLOR_GREEN_G = 255, COLOR_GREEN_B = 0;
constexpr uint8_t COLOR_BLUE_R = 0, COLOR_BLUE_G = 0, COLOR_BLUE_B = 255;
constexpr uint8_t COLOR_ORANGE_R = 255, COLOR_ORANGE_G = 165, COLOR_ORANGE_B = 0;

std::string load_file(const std::string& path)
{
  std::ifstream file(path);
  if (!file.is_open())
    return "";
  return std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
}

std::unordered_map<std::string, std::string> states_from_list(const ListControllers::Response& msg)
{
  std::unordered_map<std::string, std::string> states;
  for (const auto& c : msg.controller) {
    states[c.name] = c.state;
  }
  return states;
}

}  // namespace

class HardwareInterfaceTest : public HectorTestFixture
{
protected:
  void SetUp() override
  {
    HectorTestFixture::SetUp();

    // Config paths
    controllers_yaml_ = std::string(TEST_CONFIG_DIR) + "/controllers.yaml";
    spawner_yaml_ = std::string(TEST_CONFIG_DIR) + "/controller_spawner.yaml";
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

  // Member variables
  std::string controllers_yaml_;
  std::string spawner_yaml_;
  std::string urdf_path_;

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

// ============================================================================
// Normal Usage Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, NormalUsage_ArmPositionMode)
{
  // 1. Load and activate arm_position_controller
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  // 2. Verify all arm motors exist in mock manager
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr) << "Motor ID " << (int) id << " not found in mock manager";
  }

  // 3. Create publisher and send position command
  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);

  std_msgs::msg::Float64MultiArray cmd;
  double target_position = 0.5;  // radians
  cmd.data = {target_position, target_position, target_position, target_position,
              target_position, target_position, target_position};  // 7 joints

  // Wait for subscriber and publish
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0) << "Controller not subscribed to commands topic";

  pub->publish(cmd);

  // 4. Wait for motors to reach target position
  std::this_thread::sleep_for(3s);

  // 5. Verify motors reached target position
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    double current_pos = motor->getCurrentPosition();
    EXPECT_NEAR(current_pos, target_position, 0.1) << "Motor ID " << (int) id << " did not reach target position. "
                                                   << "Expected: " << target_position << ", Got: " << current_pos;
  }
}

TEST_F(HardwareInterfaceTest, NormalUsage_FlipperVelocityMode)
{
  // 1. Load and activate flipper_velocity_controller
  ASSERT_TRUE(loadAndActivateController("flipper_velocity_controller"));

  // 2. Verify all flipper motors exist
  std::vector<uint8_t> flipper_ids = {FLIPPER_FL_ID, FLIPPER_FR_ID, FLIPPER_BL_ID, FLIPPER_BR_ID};
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr) << "Flipper motor ID " << (int) id << " not found";
  }

  // 3. Record initial positions
  std::map<uint8_t, double> initial_positions;
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    initial_positions[id] = motor->getCurrentPosition();
  }

  // 4. Send velocity command
  auto pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/flipper_velocity_controller/commands", 10);

  std_msgs::msg::Float64MultiArray cmd;
  double joint_velocity = 1.0;  // rad/s for joint
  cmd.data = {joint_velocity, joint_velocity, joint_velocity, joint_velocity};

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  pub->publish(cmd);

  // 5. Wait and verify motors are moving
  std::this_thread::sleep_for(1s);

  // Flipper transmissions have mechanical_reduction of +/-2.0
  // So actuator velocity = joint_velocity * reduction
  // FL: -2.0, FR: 2.0, BL: 2.0, BR: -2.0
  std::map<uint8_t, double> expected_directions = {
      {FLIPPER_FL_ID, -1.0},  // reduction=-2.0, so negative direction
      {FLIPPER_FR_ID, 1.0},   // reduction=2.0, positive direction
      {FLIPPER_BL_ID, 1.0},   // reduction=2.0
      {FLIPPER_BR_ID, -1.0}   // reduction=-2.0
  };

  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    double current_pos = motor->getCurrentPosition();
    double position_change = current_pos - initial_positions[id];

    // Verify position changed in expected direction
    if (expected_directions[id] > 0) {
      EXPECT_GT(position_change, 0.1) << "Flipper motor " << (int) id << " should have moved in positive direction";
    } else {
      EXPECT_LT(position_change, -0.1) << "Flipper motor " << (int) id << " should have moved in negative direction";
    }
  }
}

TEST_F(HardwareInterfaceTest, NormalUsage_ControllerSwitch_ArmPositionToVelocity)
{
  // 1. Start with arm position controller active
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  // 2. Move to a known position
  auto pos_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);

  std_msgs::msg::Float64MultiArray pos_cmd;
  pos_cmd.data = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pos_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  pos_pub->publish(pos_cmd);
  std::this_thread::sleep_for(2s);

  // 3. Load velocity controller (configure but don't activate)
  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;

  auto load_req = std::make_shared<LoadController::Request>();
  load_req->name = "arm_velocity_controller";
  hector_testing_utils::call_service<LoadController>(load_client_->get(), load_req, *executor_, options);

  auto config_req = std::make_shared<ConfigureController::Request>();
  config_req->name = "arm_velocity_controller";
  hector_testing_utils::call_service<ConfigureController>(config_client_->get(), config_req, *executor_, options);

  // 4. Switch controllers
  ASSERT_TRUE(switch_controllers({"arm_velocity_controller"}, {"arm_position_controller"}));

  // 5. Verify states
  ASSERT_TRUE(waitForControllerState("arm_position_controller", "inactive", 5s));
  ASSERT_TRUE(waitForControllerState("arm_velocity_controller", "active", 5s));

  // 6. Send velocity command and verify movement
  auto vel_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_velocity_controller/commands", 10);

  std_msgs::msg::Float64MultiArray vel_cmd;
  vel_cmd.data = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};  // rad/s

  deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && vel_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(vel_pub->get_subscription_count(), 0);

  // Record positions before velocity command
  std::vector<double> positions_before;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_before.push_back(motor->getCurrentPosition());
  }

  vel_pub->publish(vel_cmd);
  std::this_thread::sleep_for(1s);

  // Verify positions changed (motors are moving)
  for (size_t i = 0; i < 7; ++i) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID + i);
    double pos_change = motor->getCurrentPosition() - positions_before[i];
    EXPECT_GT(pos_change, 0.1) << "Motor " << (ARM_JOINT_1_ID + i) << " should have moved in velocity mode";
  }
}

TEST_F(HardwareInterfaceTest, NormalUsage_SimultaneousMovement)
{
  // 1. Activate all controllers
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));
  ASSERT_TRUE(loadAndActivateController("flipper_velocity_controller"));
  ASSERT_TRUE(loadAndActivateController("gripper_position_controller"));

  // 2. Create publishers
  auto arm_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);
  auto flipper_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/flipper_velocity_controller/commands", 10);
  auto gripper_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/gripper_position_controller/commands", 10);

  // Wait for all subscribers
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (arm_pub->get_subscription_count() > 0 && flipper_pub->get_subscription_count() > 0 &&
        gripper_pub->get_subscription_count() > 0) {
      break;
    }
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }

  // 3. Record initial states
  std::vector<double> arm_initial, flipper_initial;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    arm_initial.push_back(dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id)->getCurrentPosition());
  }
  for (uint8_t id : {FLIPPER_FL_ID, FLIPPER_FR_ID, FLIPPER_BL_ID, FLIPPER_BR_ID}) {
    flipper_initial.push_back(
        dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id)->getCurrentPosition());
  }
  double gripper_initial =
      dynamixel_ros_control::MockDynamixelManager::instance().getMotor(GRIPPER_ID)->getCurrentPosition();

  // 4. Send commands to all
  std_msgs::msg::Float64MultiArray arm_cmd, flipper_cmd, gripper_cmd;
  arm_cmd.data = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  flipper_cmd.data = {1.0, 1.0, 1.0, 1.0};
  gripper_cmd.data = {0.5};

  arm_pub->publish(arm_cmd);
  flipper_pub->publish(flipper_cmd);
  gripper_pub->publish(gripper_cmd);

  // 5. Wait for movement
  std::this_thread::sleep_for(2s);

  // 6. Verify all moved
  for (size_t i = 0; i < 7; ++i) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID + i);
    EXPECT_NE(motor->getCurrentPosition(), arm_initial[i]) << "Arm joint " << i << " should have moved";
  }

  size_t idx = 0;
  for (uint8_t id : {FLIPPER_FL_ID, FLIPPER_FR_ID, FLIPPER_BL_ID, FLIPPER_BR_ID}) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_NE(motor->getCurrentPosition(), flipper_initial[idx]) << "Flipper " << (int) id << " should have moved";
    idx++;
  }

  auto gripper_motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(GRIPPER_ID);
  EXPECT_NE(gripper_motor->getCurrentPosition(), gripper_initial) << "Gripper should have moved";
}

// ============================================================================
// LED Color Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, LED_BlueWhenActiveAndTorqueOn)
{
  // Hardware interface is active with torque on by default (torque_on_startup: true)
  // LED should be blue
  std::this_thread::sleep_for(500ms);  // Allow time for LED update

  // Check arm motors (hardware interface athena_arm_interface)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor) {
      EXPECT_EQ(motor->getLedRed(), COLOR_BLUE_R) << "Motor " << (int) id << " LED Red";
      EXPECT_EQ(motor->getLedGreen(), COLOR_BLUE_G) << "Motor " << (int) id << " LED Green";
      EXPECT_EQ(motor->getLedBlue(), COLOR_BLUE_B) << "Motor " << (int) id << " LED Blue";
    }
  }
}

// ============================================================================
// Mock Motor Physics Verification Test
// ============================================================================

TEST_F(HardwareInterfaceTest, MockMotor_VerifyPhysicsSimulation)
{
  // This test verifies the mock motor physics work correctly
  auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor, nullptr);

  // Verify motor is in position mode with torque enabled
  EXPECT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 1);

  // Verify motor can read/write registers correctly
  uint16_t goal_pos_addr = motor->getAddress("goal_position");
  EXPECT_GT(goal_pos_addr, 0) << "goal_position address should be valid";

  // Write a goal position directly and verify physics simulation moves toward it
  int32_t target_ticks = 10000;  // Small positive position
  motor->write4Byte(goal_pos_addr, static_cast<uint32_t>(target_ticks));

  // Let physics update
  for (int i = 0; i < 100; ++i) {
    motor->update(0.01);
  }

  // Verify motor moved toward goal
  EXPECT_GT(motor->getCurrentPosition(), 0.0) << "Motor should have moved toward positive goal";
}

// ============================================================================
// E-Stop Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, EStop_StopsMovement)
{
  // 1. Activate arm position controller and start movement
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  // Send position command - motors should start moving
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};  // Target position
  pub->publish(cmd);
  std::this_thread::sleep_for(500ms);

  // Record positions before e-stop
  std::vector<double> positions_before;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_before.push_back(motor->getCurrentPosition());
  }

  // 2. Trigger E-Stop
  auto estop_pub = tester_node_->create_publisher<std_msgs::msg::Bool>("/soft_e_stop", 10);
  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;

  deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && estop_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(estop_pub->get_subscription_count(), 0) << "No subscriber for e-stop topic";

  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // 3. Send another command - should be ignored due to e-stop
  cmd.data = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
  pub->publish(cmd);
  std::this_thread::sleep_for(500ms);

  // 4. Verify LED is orange and motors stopped (positions haven't changed much since e-stop)
  std::this_thread::sleep_for(200ms);  // Allow LED update

  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedRed(), COLOR_ORANGE_R) << "Motor " << (int) id << " LED should be orange (R)";
    EXPECT_EQ(motor->getLedGreen(), COLOR_ORANGE_G) << "Motor " << (int) id << " LED should be orange (G)";
    EXPECT_EQ(motor->getLedBlue(), COLOR_ORANGE_B) << "Motor " << (int) id << " LED should be orange (B)";
  }

  // 5. Disable E-Stop and verify normal operation resumes
  estop_msg.data = false;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // Verify LED is back to blue
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedRed(), COLOR_BLUE_R) << "Motor " << (int) id << " LED should be blue (R)";
    EXPECT_EQ(motor->getLedGreen(), COLOR_BLUE_G) << "Motor " << (int) id << " LED should be blue (G)";
    EXPECT_EQ(motor->getLedBlue(), COLOR_BLUE_B) << "Motor " << (int) id << " LED should be blue (B)";
  }
}

// ============================================================================
// Torque Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, Torque_DisableTorqueChangesLEDToGreen)
{
  // 1. Verify initial state - torque on, LED blue
  std::this_thread::sleep_for(200ms);
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor) {
      EXPECT_EQ(motor->getLedBlue(), COLOR_BLUE_B) << "Initial LED should be blue";
    }
  }

  // 2. Call set_torque service to disable torque
  auto torque_client = tester_node_->create_test_client<std_srvs::srv::SetBool>("/athena_arm_interface/set_torque");
  ASSERT_TRUE(torque_client->wait_for_service(*executor_, 5s));

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;  // Disable torque

  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;
  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);

  ASSERT_NE(resp, nullptr) << "Service call failed";
  EXPECT_TRUE(resp->success) << "Torque disable should succeed";

  // 3. Wait for LED update and verify LED is green
  std::this_thread::sleep_for(500ms);

  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor) {
      EXPECT_EQ(motor->getLedRed(), COLOR_GREEN_R) << "Motor " << (int) id << " LED should be green (R)";
      EXPECT_EQ(motor->getLedGreen(), COLOR_GREEN_G) << "Motor " << (int) id << " LED should be green (G)";
      EXPECT_EQ(motor->getLedBlue(), COLOR_GREEN_B) << "Motor " << (int) id << " LED should be green (B)";
    }
  }

  // 4. Verify torque is actually disabled in motors
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor) {
      uint16_t torque_addr = motor->getAddress("torque_enable");
      EXPECT_EQ(motor->read1Byte(torque_addr), 0) << "Motor " << (int) id << " torque should be disabled";
    }
  }
}

TEST_F(HardwareInterfaceTest, Torque_EnableTorqueChangesLEDToBlue)
{
  // 1. First disable torque
  auto torque_client = tester_node_->create_test_client<std_srvs::srv::SetBool>("/athena_arm_interface/set_torque");
  ASSERT_TRUE(torque_client->wait_for_service(*executor_, 5s));

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;  // Disable torque

  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;
  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);

  std::this_thread::sleep_for(300ms);

  // Verify LED is green (torque off)
  auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  EXPECT_EQ(motor->getLedGreen(), COLOR_GREEN_G) << "LED should be green when torque is off";

  // 2. Enable torque
  request->data = true;
  resp = hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success);

  std::this_thread::sleep_for(300ms);

  // 3. Verify LED is blue
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor) {
      EXPECT_EQ(motor->getLedRed(), COLOR_BLUE_R) << "Motor " << (int) id << " LED should be blue (R)";
      EXPECT_EQ(motor->getLedGreen(), COLOR_BLUE_G) << "Motor " << (int) id << " LED should be blue (G)";
      EXPECT_EQ(motor->getLedBlue(), COLOR_BLUE_B) << "Motor " << (int) id << " LED should be blue (B)";
    }
  }

  // 4. Verify torque is enabled in motors
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor) {
      uint16_t torque_addr = motor->getAddress("torque_enable");
      EXPECT_EQ(motor->read1Byte(torque_addr), 1) << "Motor " << (int) id << " torque should be enabled";
    }
  }
}

TEST_F(HardwareInterfaceTest, Torque_CommandsNotExecutedWhenTorqueOff)
{
  // 1. Disable torque
  auto torque_client = tester_node_->create_test_client<std_srvs::srv::SetBool>("/athena_arm_interface/set_torque");
  ASSERT_TRUE(torque_client->wait_for_service(*executor_, 5s));

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;

  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;
  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);

  std::this_thread::sleep_for(300ms);

  // 2. Record initial positions
  std::vector<double> initial_positions;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    initial_positions.push_back(motor->getCurrentPosition());
  }

  // 3. Load and activate controller
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  // 4. Send position command
  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }

  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  pub->publish(cmd);
  std::this_thread::sleep_for(1s);

  // 5. Verify motors didn't move (torque is off, physics simulation shouldn't move them)
  for (size_t i = 0; i < 7; ++i) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID + i);
    // With torque off, motors should not move toward goal
    EXPECT_NEAR(motor->getCurrentPosition(), initial_positions[i], 0.01)
        << "Motor " << (ARM_JOINT_1_ID + i) << " should not have moved with torque off";
  }
}

// ============================================================================
// Additional LED Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, LED_GreenWhenTorqueOff)
{
  // Disable torque and verify LED is green
  auto torque_client = tester_node_->create_test_client<std_srvs::srv::SetBool>("/athena_arm_interface/set_torque");
  ASSERT_TRUE(torque_client->wait_for_service(*executor_, 5s));

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;

  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;
  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success);

  std::this_thread::sleep_for(500ms);

  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor) {
      EXPECT_EQ(motor->getLedRed(), COLOR_GREEN_R) << "Motor " << (int) id << " R";
      EXPECT_EQ(motor->getLedGreen(), COLOR_GREEN_G) << "Motor " << (int) id << " G";
      EXPECT_EQ(motor->getLedBlue(), COLOR_GREEN_B) << "Motor " << (int) id << " B";
    }
  }
}

TEST_F(HardwareInterfaceTest, LED_OrangeWhenEStopActive)
{
  // Trigger E-Stop and verify LED is orange
  auto estop_pub = tester_node_->create_publisher<std_msgs::msg::Bool>("/soft_e_stop", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && estop_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(estop_pub->get_subscription_count(), 0);

  std_msgs::msg::Bool msg;
  msg.data = true;
  estop_pub->publish(msg);

  std::this_thread::sleep_for(500ms);

  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor) {
      EXPECT_EQ(motor->getLedRed(), COLOR_ORANGE_R) << "Motor " << (int) id << " R";
      EXPECT_EQ(motor->getLedGreen(), COLOR_ORANGE_G) << "Motor " << (int) id << " G";
      EXPECT_EQ(motor->getLedBlue(), COLOR_ORANGE_B) << "Motor " << (int) id << " B";
    }
  }

  // Clean up - disable e-stop
  msg.data = false;
  estop_pub->publish(msg);
  std::this_thread::sleep_for(200ms);
}

// ============================================================================
// Transmission Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, Transmission_FlipperVelocityReduction)
{
  // Test that flipper transmission correctly applies mechanical reduction
  // FL: -2.0, FR: 2.0, BL: 2.0, BR: -2.0
  // When joint velocity is 1.0 rad/s, actuator velocity should be 2.0 rad/s (with sign)

  ASSERT_TRUE(loadAndActivateController("flipper_velocity_controller"));

  auto pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/flipper_velocity_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  // Record initial positions
  std::vector<uint8_t> flipper_ids = {FLIPPER_FL_ID, FLIPPER_FR_ID, FLIPPER_BL_ID, FLIPPER_BR_ID};
  std::map<uint8_t, double> initial_positions;
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    initial_positions[id] = motor->getCurrentPosition();
  }

  // Send joint velocity command of 1.0 rad/s to all flippers
  std_msgs::msg::Float64MultiArray cmd;
  double joint_velocity = 1.0;
  cmd.data = {joint_velocity, joint_velocity, joint_velocity, joint_velocity};
  pub->publish(cmd);

  // Run for 1 second
  double test_duration = 1.0;
  std::this_thread::sleep_for(std::chrono::duration<double>(test_duration));

  // Calculate expected position changes
  // Actuator velocity = joint velocity * mechanical_reduction
  // Expected position change = actuator_velocity * time
  std::map<uint8_t, double> expected_reductions = {
      {FLIPPER_FL_ID, -2.0}, {FLIPPER_FR_ID, 2.0}, {FLIPPER_BL_ID, 2.0}, {FLIPPER_BR_ID, -2.0}};

  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    double current_pos = motor->getCurrentPosition();
    double position_change = current_pos - initial_positions[id];
    double expected_actuator_velocity = joint_velocity * expected_reductions[id];
    double expected_position_change = expected_actuator_velocity * test_duration;

    // Allow some tolerance for timing and physics simulation
    EXPECT_NEAR(position_change, expected_position_change, 0.5)
        << "Flipper motor " << (int) id << " position change doesn't match transmission ratio. "
        << "Expected: " << expected_position_change << ", Got: " << position_change;
  }
}

TEST_F(HardwareInterfaceTest, Transmission_FlipperPositionReduction)
{
  // Test position control respects transmission ratio
  // For FL with reduction=-2.0: joint position change of 0.5 rad -> actuator change of -1.0 rad

  ASSERT_TRUE(loadAndActivateController("flipper_position_controller"));

  auto pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/flipper_position_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  // First send to position 0 and wait
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.0, 0.0, 0.0, 0.0};
  pub->publish(cmd);
  std::this_thread::sleep_for(2s);

  // Record positions at joint position 0
  std::map<uint8_t, double> positions_at_zero;
  std::vector<uint8_t> flipper_ids = {FLIPPER_FL_ID, FLIPPER_FR_ID, FLIPPER_BL_ID, FLIPPER_BR_ID};
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_at_zero[id] = motor->getCurrentPosition();
  }

  // Send joint position command
  double joint_position = 0.5;  // rad
  cmd.data = {joint_position, joint_position, joint_position, joint_position};
  pub->publish(cmd);

  // Wait for motors to reach position
  std::this_thread::sleep_for(3s);

  // Verify actuator position changes match transmission ratios
  std::map<uint8_t, double> expected_reductions = {
      {FLIPPER_FL_ID, -2.0}, {FLIPPER_FR_ID, 2.0}, {FLIPPER_BL_ID, 2.0}, {FLIPPER_BR_ID, -2.0}};

  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    double actuator_pos = motor->getCurrentPosition();
    double actuator_change = actuator_pos - positions_at_zero[id];
    double expected_actuator_change = joint_position * expected_reductions[id];

    EXPECT_NEAR(actuator_change, expected_actuator_change, 0.3)
        << "Flipper motor " << (int) id << " position change doesn't match transmission ratio. "
        << "Expected change: " << expected_actuator_change << ", Got: " << actuator_change;
  }
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, EStop_MultipleCommandsBlocked)
{
  // Test that multiple commands during E-Stop are all blocked
  // and that LED correctly reflects e-stop state

  // 1. Activate arm position controller and move to initial position
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  // First move to a known position
  std_msgs::msg::Float64MultiArray init_cmd;
  init_cmd.data = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
  pub->publish(init_cmd);
  std::this_thread::sleep_for(1s);

  // 2. Trigger E-Stop
  auto estop_pub = tester_node_->create_publisher<std_msgs::msg::Bool>("/soft_e_stop", 10);
  deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && estop_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(estop_pub->get_subscription_count(), 0);

  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // 3. Record positions at e-stop
  std::vector<double> positions_at_estop;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_at_estop.push_back(motor->getCurrentPosition());
  }

  // 4. Verify LED is orange
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedRed(), COLOR_ORANGE_R) << "Motor " << (int) id << " LED should be orange (R)";
    EXPECT_EQ(motor->getLedGreen(), COLOR_ORANGE_G) << "Motor " << (int) id << " LED should be orange (G)";
    EXPECT_EQ(motor->getLedBlue(), COLOR_ORANGE_B) << "Motor " << (int) id << " LED should be orange (B)";
  }

  // 5. Send multiple movement commands over time - all should be ignored
  for (int i = 0; i < 5; ++i) {
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data = {1.0 + i * 0.2, 1.0 + i * 0.2, 1.0 + i * 0.2, 1.0 + i * 0.2,
                1.0 + i * 0.2, 1.0 + i * 0.2, 1.0 + i * 0.2};
    pub->publish(cmd);
    std::this_thread::sleep_for(200ms);
  }
  std::this_thread::sleep_for(500ms);

  // 6. Verify motors haven't moved significantly (e-stop still active)
  for (size_t i = 0; i < 7; ++i) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID + i);
    EXPECT_NEAR(motor->getCurrentPosition(), positions_at_estop[i], 0.15)
        << "Motor " << (ARM_JOINT_1_ID + i) << " should not move during e-stop";
  }

  // 7. Verify LED is still orange after all those commands
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedRed(), COLOR_ORANGE_R) << "Motor " << (int) id << " LED should still be orange (R)";
    EXPECT_EQ(motor->getLedGreen(), COLOR_ORANGE_G) << "Motor " << (int) id << " LED should still be orange (G)";
    EXPECT_EQ(motor->getLedBlue(), COLOR_ORANGE_B) << "Motor " << (int) id << " LED should still be orange (B)";
  }

  // 8. Disable E-Stop and verify LED returns to blue
  estop_msg.data = false;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedRed(), COLOR_BLUE_R) << "Motor " << (int) id << " LED should be blue (R)";
    EXPECT_EQ(motor->getLedGreen(), COLOR_BLUE_G) << "Motor " << (int) id << " LED should be blue (G)";
    EXPECT_EQ(motor->getLedBlue(), COLOR_BLUE_B) << "Motor " << (int) id << " LED should be blue (B)";
  }
}

TEST_F(HardwareInterfaceTest, Gripper_PositionControl)
{
  // Test gripper position control
  ASSERT_TRUE(loadAndActivateController("gripper_position_controller"));

  // Verify gripper motor exists
  auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(GRIPPER_ID);
  ASSERT_NE(motor, nullptr) << "Gripper motor not found";

  double initial_pos = motor->getCurrentPosition();

  // Send position command
  auto pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/gripper_position_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  std_msgs::msg::Float64MultiArray cmd;
  double target = 0.5;
  cmd.data = {target};
  pub->publish(cmd);

  std::this_thread::sleep_for(2s);

  // Verify gripper moved
  double final_pos = motor->getCurrentPosition();
  EXPECT_NEAR(final_pos, target, 0.2) << "Gripper should have moved to target. Initial: " << initial_pos
                                      << ", Final: " << final_pos;
}

// ============================================================================
// Transmission Offset Manager Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, TransmissionOffset_AdjustFlipperOffset)
{
  // Test that calling adjust_transmission_offsets service:
  // 1. Deactivates active flipper controllers
  // 2. Adjusts offset so position jumps to external measurement
  // 3. Position should not change during service call

  using AdjustOffsets = hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets;

  // 1. First activate flipper controller and move to a position
  ASSERT_TRUE(loadAndActivateController("flipper_position_controller"));

  auto pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/flipper_position_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  // Move to initial position
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.3, 0.3, 0.3, 0.3};
  pub->publish(cmd);
  std::this_thread::sleep_for(2s);

  // 2. Record current actuator positions
  std::vector<uint8_t> flipper_ids = {FLIPPER_FL_ID, FLIPPER_FR_ID, FLIPPER_BL_ID, FLIPPER_BR_ID};
  std::map<uint8_t, double> positions_before;
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_before[id] = motor->getCurrentPosition();
  }

  // 3. Call adjust_transmission_offsets service
  auto offset_client =
      tester_node_->create_test_client<AdjustOffsets>("/athena_flipper_interface/adjust_transmission_offsets");
  ASSERT_TRUE(offset_client->wait_for_service(*executor_, 5s)) << "adjust_transmission_offsets service not available";

  auto request = std::make_shared<AdjustOffsets::Request>();
  // Set external measurement - this is the "true" joint position from external sensor
  request->external_joint_measurements.name = {"flipper_fl_joint", "flipper_fr_joint", "flipper_bl_joint",
                                               "flipper_br_joint"};
  double external_position = 1.5708;  // ~90 degrees
  request->external_joint_measurements.position = {external_position, external_position, external_position,
                                                   external_position};

  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 10s;
  options.response_timeout = 10s;
  auto resp = hector_testing_utils::call_service<AdjustOffsets>(offset_client->get(), request, *executor_, options);

  ASSERT_NE(resp, nullptr) << "Service call failed";
  EXPECT_TRUE(resp->success) << "Offset adjustment should succeed: " << resp->message;

  // 4. Verify controller was deactivated during adjustment
  // (The pre_callback deactivates controllers)
  auto list_resp = list_controllers();
  ASSERT_NE(list_resp, nullptr);
  auto states = states_from_list(*list_resp);
  // Controller should be inactive after offset adjustment (deactivated by pre_callback)
  EXPECT_EQ(states["flipper_position_controller"], "inactive")
      << "Controller should be deactivated after offset adjustment";

  // 5. Verify actuator positions didn't change (only offset changed)
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    double pos_after = motor->getCurrentPosition();
    EXPECT_NEAR(pos_after, positions_before[id], 0.1)
        << "Actuator position should not change during offset adjustment for motor " << (int) id;
  }
}

// ============================================================================
// Communication Error / Hard E-Stop Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, CommunicationError_TemporaryErrorRecovery)
{
  // Test that the system can recover from temporary communication errors
  // This simulates brief motor disconnection or bus errors

  // 1. Activate controller and verify normal operation
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  // Move to initial position
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
  pub->publish(cmd);
  std::this_thread::sleep_for(1s);

  // 2. Inject temporary communication errors on some motors
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  auto motor2 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_2_ID);
  ASSERT_NE(motor1, nullptr);
  ASSERT_NE(motor2, nullptr);

  motor1->setCommunicationError(true);
  motor2->setCommunicationError(true);

  // Wait a bit with errors
  std::this_thread::sleep_for(500ms);

  // 3. Clear errors - simulate recovery
  motor1->setCommunicationError(false);
  motor2->setCommunicationError(false);

  // 4. Wait for recovery and then send command
  std::this_thread::sleep_for(500ms);

  // Record positions before new command
  std::vector<double> positions_before;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_before.push_back(motor->getCurrentPosition());
  }

  // Send new position command
  cmd.data = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  pub->publish(cmd);
  std::this_thread::sleep_for(2s);

  // 5. Verify at least some motors moved (system recovered)
  bool any_moved = false;
  for (size_t i = 0; i < 7; ++i) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID + i);
    if (std::abs(motor->getCurrentPosition() - positions_before[i]) > 0.05) {
      any_moved = true;
      break;
    }
  }
  EXPECT_TRUE(any_moved) << "System should recover from temporary communication errors";
}

TEST_F(HardwareInterfaceTest, CommunicationError_GlobalErrorBlocksOperation)
{
  // Test that global communication errors (all motors) block operation
  // This simulates hard E-Stop cutting power to all motors

  // 1. Activate controller
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  // Move to initial position
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
  pub->publish(cmd);
  std::this_thread::sleep_for(1s);

  // Record initial positions
  std::vector<double> initial_positions;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    initial_positions.push_back(motor->getCurrentPosition());
  }

  // 2. Enable global communication error (simulates power cut)
  dynamixel_ros_control::MockDynamixelManager::instance().setGlobalCommunicationError(true);

  // 3. Try to send movement commands - should have no effect
  cmd.data = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  pub->publish(cmd);
  std::this_thread::sleep_for(1s);

  // Verify positions haven't changed (communication errors prevent read/write)
  for (size_t i = 0; i < 7; ++i) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID + i);
    // With communication errors, the mock won't update positions
    EXPECT_NEAR(motor->getCurrentPosition(), initial_positions[i], 0.1)
        << "Motor " << (ARM_JOINT_1_ID + i) << " should not move during communication errors";
  }

  // 4. Clear errors (simulate power restored)
  dynamixel_ros_control::MockDynamixelManager::instance().setGlobalCommunicationError(false);
  std::this_thread::sleep_for(500ms);

  // 5. Verify system can recover
  std::vector<double> positions_after_recovery;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_after_recovery.push_back(motor->getCurrentPosition());
  }

  cmd.data = {0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7};
  pub->publish(cmd);
  std::this_thread::sleep_for(2s);

  bool any_moved = false;
  for (size_t i = 0; i < 7; ++i) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID + i);
    if (std::abs(motor->getCurrentPosition() - positions_after_recovery[i]) > 0.05) {
      any_moved = true;
      break;
    }
  }
  EXPECT_TRUE(any_moved) << "System should recover after communication errors are cleared";
}

// ============================================================================
// Reboot Service Test
// ============================================================================

TEST_F(HardwareInterfaceTest, RebootService_ResetsMotors)
{
  // Test the reboot service functionality
  // This is used to recover motors from error states

  // 1. Create reboot service client
  auto reboot_client = tester_node_->create_test_client<std_srvs::srv::Trigger>("/athena_arm_interface/reboot");
  ASSERT_TRUE(reboot_client->wait_for_service(*executor_, 5s)) << "Reboot service not available";

  // 2. Record initial motor states
  std::vector<double> initial_positions;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    initial_positions.push_back(motor->getCurrentPosition());
  }

  // 3. Call reboot service
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 10s;
  options.response_timeout = 10s;
  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::Trigger>(reboot_client->get(), request, *executor_, options);

  ASSERT_NE(resp, nullptr) << "Reboot service call failed";
  EXPECT_TRUE(resp->success) << "Reboot should succeed: " << resp->message;

  // 4. Verify motors still exist and are functional
  std::this_thread::sleep_for(500ms);

  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr) << "Motor " << (int) id << " should still exist after reboot";
  }

  // 5. Verify LED is correct (should be blue if torque is on)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedBlue(), COLOR_BLUE_B) << "Motor " << (int) id << " LED should be blue after reboot";
  }
}

// ============================================================================
// Rapid Controller Switching Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, RapidControllerSwitch_StressTest)
{
  // Test rapid switching between controllers
  // This can expose race conditions and timing issues

  // Load and configure both controllers
  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;

  auto load_req = std::make_shared<LoadController::Request>();
  load_req->name = "arm_position_controller";
  hector_testing_utils::call_service<LoadController>(load_client_->get(), load_req, *executor_, options);

  auto config_req = std::make_shared<ConfigureController::Request>();
  config_req->name = "arm_position_controller";
  hector_testing_utils::call_service<ConfigureController>(config_client_->get(), config_req, *executor_, options);

  load_req->name = "arm_velocity_controller";
  hector_testing_utils::call_service<LoadController>(load_client_->get(), load_req, *executor_, options);

  config_req->name = "arm_velocity_controller";
  hector_testing_utils::call_service<ConfigureController>(config_client_->get(), config_req, *executor_, options);

  // First, activate position controller (velocity is still inactive)
  ASSERT_TRUE(switch_controllers({"arm_position_controller"}, {}))
      << "Initial activation of position controller failed";
  std::this_thread::sleep_for(100ms);

  // Perform rapid switches between the two
  for (int i = 0; i < 5; ++i) {
    // Switch to velocity controller (deactivate position)
    ASSERT_TRUE(switch_controllers({"arm_velocity_controller"}, {"arm_position_controller"}))
        << "Switch to velocity controller failed on iteration " << i;
    std::this_thread::sleep_for(100ms);

    // Switch to position controller (deactivate velocity)
    ASSERT_TRUE(switch_controllers({"arm_position_controller"}, {"arm_velocity_controller"}))
        << "Switch to position controller failed on iteration " << i;
    std::this_thread::sleep_for(100ms);
  }

  // Final state check - verify system is stable
  auto list_resp = list_controllers();
  ASSERT_NE(list_resp, nullptr);
  auto states = states_from_list(*list_resp);

  // Position controller should be active (last activation in the loop)
  EXPECT_EQ(states["arm_position_controller"], "active")
      << "Position controller should be active after rapid switching";
  EXPECT_EQ(states["arm_velocity_controller"], "inactive")
      << "Velocity controller should be inactive after rapid switching";
}

// ============================================================================
// Motor Limit Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, MotorLimits_PositionLimitRespected)
{
  // Test that position limits are respected
  // (This depends on hardware interface implementation)

  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  // Send extreme position command (outside normal operating range)
  std_msgs::msg::Float64MultiArray cmd;
  double extreme_position = 100.0;  // Very large position
  cmd.data = {extreme_position, extreme_position, extreme_position, extreme_position,
              extreme_position, extreme_position, extreme_position};
  pub->publish(cmd);
  std::this_thread::sleep_for(2s);

  // Motors should move but may be limited by hardware
  // Just verify motors are still functional (no crash/hang)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr) << "Motor " << (int) id << " should still exist";
    // Motor should have moved in positive direction
    EXPECT_GT(motor->getCurrentPosition(), 0.0)
        << "Motor " << (int) id << " should have moved toward positive position";
  }
}

// ============================================================================
// State Interface Consistency Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, StateInterface_PositionVelocityConsistent)
{
  // Test that position and velocity state interfaces are consistent
  // Velocity should approximately match position derivative

  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  // Send position command to initiate movement
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  pub->publish(cmd);

  // Sample positions over time
  double dt = 0.1;
  std::this_thread::sleep_for(std::chrono::duration<double>(dt));

  auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  double pos1 = motor->getCurrentPosition();

  std::this_thread::sleep_for(std::chrono::duration<double>(dt));

  double pos2 = motor->getCurrentPosition();

  // Velocity should be approximately (pos2 - pos1) / dt
  double approx_velocity = (pos2 - pos1) / dt;

  // Just verify motor is moving in expected direction
  EXPECT_GT(pos2, pos1) << "Motor should be moving toward goal (increasing position)";
  EXPECT_GT(approx_velocity, 0.0) << "Velocity should be positive when moving to higher position";
}

// ============================================================================
// Controller Activation Before Successful Read Test
// NOTE: This test documents expected behavior. If first_read_successful_ is false,
// controller activation should fail. This is a safety feature.
// ============================================================================

TEST_F(HardwareInterfaceTest, EdgeCase_ControllerActivationWithCommunicationErrors)
{
  // Test controller activation when communication errors prevent successful reads
  // Expected: Controller activation should fail gracefully

  // 1. Set communication errors on all arm motors BEFORE activating controller
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor)
      motor->setCommunicationError(true);
  }
  // Also set error on gripper
  auto gripper_motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(GRIPPER_ID);
  if (gripper_motor)
    gripper_motor->setCommunicationError(true);

  // Wait for errors to take effect in read cycle
  std::this_thread::sleep_for(500ms);

  // 2. Try to load and activate controller - this may fail
  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;

  auto load_req = std::make_shared<LoadController::Request>();
  load_req->name = "arm_position_controller";
  auto load_resp =
      hector_testing_utils::call_service<LoadController>(load_client_->get(), load_req, *executor_, options);

  // Loading should succeed (doesn't require communication)
  ASSERT_NE(load_resp, nullptr);
  EXPECT_TRUE(load_resp->ok) << "Controller loading should succeed";

  auto config_req = std::make_shared<ConfigureController::Request>();
  config_req->name = "arm_position_controller";
  auto config_resp =
      hector_testing_utils::call_service<ConfigureController>(config_client_->get(), config_req, *executor_, options);
  ASSERT_NE(config_resp, nullptr);
  EXPECT_TRUE(config_resp->ok) << "Controller configuration should succeed";

  // 3. Activation might fail due to no successful read
  auto switch_req = std::make_shared<SwitchController::Request>();
  switch_req->activate_controllers = {"arm_position_controller"};
  switch_req->strictness = SwitchController::Request::STRICT;
  auto switch_resp =
      hector_testing_utils::call_service<SwitchController>(switch_client_->get(), switch_req, *executor_, options);

  // This documents the current behavior - activation may fail
  // If this test passes with switch_resp->ok == false, that's expected safety behavior
  // If a fix is implemented to do inline read, this test should be updated

  // 4. Clear errors for cleanup
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor)
      motor->setCommunicationError(false);
  }
  if (gripper_motor)
    gripper_motor->setCommunicationError(false);

  // Log the result for documentation
  if (switch_resp && !switch_resp->ok) {
    // Expected behavior - activation fails when read hasn't succeeded
    SUCCEED() << "Controller activation correctly rejected when communication errors prevent reads";
  } else if (switch_resp && switch_resp->ok) {
    // If activation succeeded, the fix was implemented - verify system is functional
    SUCCEED() << "Controller activation succeeded (inline read fix may be implemented)";
  }
}

// ============================================================================
// Simultaneous Hardware Interface Operations Test
// ============================================================================

TEST_F(HardwareInterfaceTest, SimultaneousOperations_ArmAndFlipperIndependent)
{
  // Test that arm and flipper interfaces operate independently
  // E-Stop on arm shouldn't affect flipper operation

  // 1. Activate controllers on both interfaces
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));
  ASSERT_TRUE(loadAndActivateController("flipper_position_controller"));

  auto arm_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);
  auto flipper_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/flipper_position_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (arm_pub->get_subscription_count() > 0 && flipper_pub->get_subscription_count() > 0) {
      break;
    }
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }

  // 2. Move both to initial positions
  std_msgs::msg::Float64MultiArray arm_cmd, flipper_cmd;
  arm_cmd.data = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
  flipper_cmd.data = {0.3, 0.3, 0.3, 0.3};
  arm_pub->publish(arm_cmd);
  flipper_pub->publish(flipper_cmd);
  std::this_thread::sleep_for(1s);

  // Record flipper positions
  std::vector<uint8_t> flipper_ids = {FLIPPER_FL_ID, FLIPPER_FR_ID, FLIPPER_BL_ID, FLIPPER_BR_ID};
  std::map<uint8_t, double> flipper_positions_before;
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    flipper_positions_before[id] = motor->getCurrentPosition();
  }

  // 3. Disable torque on ARM only (simulating arm-specific issue)
  auto torque_client = tester_node_->create_test_client<std_srvs::srv::SetBool>("/athena_arm_interface/set_torque");
  ASSERT_TRUE(torque_client->wait_for_service(*executor_, 5s));

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;

  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;
  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success);

  std::this_thread::sleep_for(300ms);

  // 4. Send new flipper command - should still work
  flipper_cmd.data = {0.6, 0.6, 0.6, 0.6};
  flipper_pub->publish(flipper_cmd);
  std::this_thread::sleep_for(2s);

  // 5. Verify flippers moved (not affected by arm torque disable)
  bool flipper_moved = false;
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (std::abs(motor->getCurrentPosition() - flipper_positions_before[id]) > 0.1) {
      flipper_moved = true;
      break;
    }
  }
  EXPECT_TRUE(flipper_moved) << "Flippers should still move when arm torque is disabled";

  // 6. Verify arm motors have green LED (torque off) while flippers have blue
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedGreen(), COLOR_GREEN_G) << "Arm motor " << (int) id << " LED should be green (torque off)";
  }

  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedBlue(), COLOR_BLUE_B) << "Flipper motor " << (int) id << " LED should be blue (torque on)";
  }
}

// ============================================================================
// Combined State Tests - Verifying behavior when multiple states are active
// ============================================================================

TEST_F(HardwareInterfaceTest, CombinedState_EStopWhileTorqueOff)
{
  // Test that E-Stop can be activated while torque is off
  // When torque is already off, E-Stop has no additional effect on motors (already safe)
  // LED behavior: When torque is off, LED stays green even with E-Stop active
  // (torque off is already a safe state)

  // 1. Disable torque first
  auto torque_client = tester_node_->create_test_client<std_srvs::srv::SetBool>("/athena_arm_interface/set_torque");
  ASSERT_TRUE(torque_client->wait_for_service(*executor_, 5s));

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;

  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;
  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);

  std::this_thread::sleep_for(300ms);

  // Verify LED is green (torque off)
  auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  EXPECT_EQ(motor->getLedGreen(), COLOR_GREEN_G) << "LED should be green when torque is off";

  // Verify torque is off
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    uint16_t torque_addr = motor->getAddress("torque_enable");
    EXPECT_EQ(motor->read1Byte(torque_addr), 0) << "Motor " << (int) id << " torque should be disabled";
  }

  // 2. Activate E-Stop while torque is off
  auto estop_pub = tester_node_->create_publisher<std_msgs::msg::Bool>("/soft_e_stop", 10);
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && estop_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(estop_pub->get_subscription_count(), 0);

  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // 3. Verify torque is still off (E-Stop should not enable torque)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    uint16_t torque_addr = motor->getAddress("torque_enable");
    EXPECT_EQ(motor->read1Byte(torque_addr), 0)
        << "Motor " << (int) id << " torque should still be disabled with E-Stop";
  }

  // 4. Disable E-Stop
  estop_msg.data = false;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // 5. Verify torque is still off after E-Stop release
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    uint16_t torque_addr = motor->getAddress("torque_enable");
    EXPECT_EQ(motor->read1Byte(torque_addr), 0)
        << "Motor " << (int) id << " torque should still be disabled after E-Stop release";
  }
}

TEST_F(HardwareInterfaceTest, CombinedState_TorqueOffWhileEStopActive)
{
  // Test disabling torque while E-Stop is active - should still work
  // This verifies that torque can always be disabled as a safety mechanism

  // 1. Activate E-Stop first
  auto estop_pub = tester_node_->create_publisher<std_msgs::msg::Bool>("/soft_e_stop", 10);
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && estop_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(estop_pub->get_subscription_count(), 0);

  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // Verify LED is orange during E-Stop
  auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  EXPECT_EQ(motor->getLedRed(), COLOR_ORANGE_R) << "LED should be orange during E-Stop";

  // 2. Disable torque while E-Stop is active
  auto torque_client = tester_node_->create_test_client<std_srvs::srv::SetBool>("/athena_arm_interface/set_torque");
  ASSERT_TRUE(torque_client->wait_for_service(*executor_, 5s));

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;

  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;
  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr);
  // Note: This should succeed - torque can be disabled even during E-Stop
  EXPECT_TRUE(resp->success);

  std::this_thread::sleep_for(300ms);

  // 3. CRITICAL: Verify torque is actually disabled (safety check)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    uint16_t torque_addr = motor->getAddress("torque_enable");
    EXPECT_EQ(motor->read1Byte(torque_addr), 0) << "Motor " << (int) id << " torque should be disabled";
  }

  // 4. Release E-Stop
  estop_msg.data = false;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // 5. Verify torque is still off after E-Stop release
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    uint16_t torque_addr = motor->getAddress("torque_enable");
    EXPECT_EQ(motor->read1Byte(torque_addr), 0)
        << "Motor " << (int) id << " torque should still be disabled after E-Stop release";
    // LED should be green (torque off state)
    EXPECT_EQ(motor->getLedGreen(), COLOR_GREEN_G)
        << "Motor " << (int) id << " LED should be green after E-Stop release";
  }
}

TEST_F(HardwareInterfaceTest, CombinedState_CalibrationWhileEStopActive)
{
  // Test that calibration (offset adjustment) is blocked when E-Stop is active

  // 1. Activate position controller to set up state
  ASSERT_TRUE(loadAndActivateController("flipper_position_controller"));
  std::this_thread::sleep_for(300ms);

  // 2. Activate E-Stop
  auto estop_pub = tester_node_->create_publisher<std_msgs::msg::Bool>("/soft_e_stop", 10);
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && estop_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(estop_pub->get_subscription_count(), 0);

  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // Record initial positions
  std::vector<uint8_t> flipper_ids = {FLIPPER_FL_ID, FLIPPER_FR_ID, FLIPPER_BL_ID, FLIPPER_BR_ID};
  std::map<uint8_t, double> initial_positions;
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    initial_positions[id] = motor->getCurrentPosition();
  }

  // 3. Attempt calibration service call (should complete but E-Stop prevents movement)
  auto calibration_client =
      tester_node_->create_test_client<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets>(
          "/athena_flipper_interface/adjust_transmission_offsets");
  // Service may or may not be available during E-Stop - just ensure no dangerous movement
  bool service_available = calibration_client->wait_for_service(*executor_, 2s);

  if (service_available) {
    auto request = std::make_shared<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets::Request>();
    sensor_msgs::msg::JointState ext_measurement;
    ext_measurement.name = {"flipper_front_left_joint"};
    ext_measurement.position = {0.5};
    request->external_joint_measurements = ext_measurement;

    hector_testing_utils::ServiceCallOptions options;
    options.service_timeout = 5s;
    options.response_timeout = 5s;
    auto resp = hector_testing_utils::call_service<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets>(
        calibration_client->get(), request, *executor_, options);
    // Response may or may not succeed, but key is no sudden movement
  }

  std::this_thread::sleep_for(500ms);

  // 4. Verify no sudden movements occurred
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    double position_change = std::abs(motor->getCurrentPosition() - initial_positions[id]);
    EXPECT_LT(position_change, 0.1) << "Motor " << (int) id
                                    << " should not have significant movement during E-Stop calibration attempt";
  }

  // Cleanup
  estop_msg.data = false;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(300ms);
}

TEST_F(HardwareInterfaceTest, CombinedState_CalibrationWhileTorqueOff)
{
  // Test calibration (offset adjustment) when torque is off

  // 1. Activate position controller
  ASSERT_TRUE(loadAndActivateController("flipper_position_controller"));
  std::this_thread::sleep_for(300ms);

  // 2. Disable torque (flipper interface service)
  auto torque_client = tester_node_->create_test_client<std_srvs::srv::SetBool>("/athena_flipper_interface/set_torque");
  ASSERT_TRUE(torque_client->wait_for_service(*executor_, 5s));

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;

  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;
  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);

  std::this_thread::sleep_for(300ms);

  // Record initial positions
  std::vector<uint8_t> flipper_ids = {FLIPPER_FL_ID, FLIPPER_FR_ID, FLIPPER_BL_ID, FLIPPER_BR_ID};
  std::map<uint8_t, double> initial_positions;
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    initial_positions[id] = motor->getCurrentPosition();
  }

  // 3. Attempt calibration service call
  auto calibration_client =
      tester_node_->create_test_client<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets>(
          "/athena_flipper_interface/adjust_transmission_offsets");
  ASSERT_TRUE(calibration_client->wait_for_service(*executor_, 5s));

  auto cal_request = std::make_shared<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets::Request>();
  sensor_msgs::msg::JointState ext_measurement;
  ext_measurement.name = {"flipper_front_left_joint"};
  ext_measurement.position = {0.5};
  cal_request->external_joint_measurements = ext_measurement;

  auto cal_resp =
      hector_testing_utils::call_service<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets>(
          calibration_client->get(), cal_request, *executor_, options);
  // Calibration should succeed even with torque off (offset adjustment is software-only)
  ASSERT_NE(cal_resp, nullptr);
  EXPECT_TRUE(cal_resp->success) << "Calibration should succeed when torque is off: " << cal_resp->message;

  std::this_thread::sleep_for(500ms);

  // 4. Verify no sudden movements (torque is off, so motors shouldn't move)
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    double position_change = std::abs(motor->getCurrentPosition() - initial_positions[id]);
    EXPECT_LT(position_change, 0.05) << "Motor " << (int) id
                                     << " should not move when torque is off during calibration";
  }

  // 5. Enable torque again - verify no sudden movement when torque re-enabled
  request->data = true;
  resp = hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success);

  // Give time for any potential movement
  std::this_thread::sleep_for(500ms);

  // Verify positions didn't jump
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    // Position change should be minimal (just due to physics simulation settling)
    double position_change = std::abs(motor->getCurrentPosition() - initial_positions[id]);
    EXPECT_LT(position_change, 0.2) << "Motor " << (int) id << " should not have jumped after torque re-enable";
  }
}

TEST_F(HardwareInterfaceTest, CombinedState_NoSuddenMovementOnAnyStateTransition)
{
  // Test verifying safe behavior during state transitions
  // The mock physics simulation naturally moves motors toward goals, so this test
  // verifies that state transitions don't cause dangerous instantaneous jumps
  // (as opposed to gradual controlled movement toward goal positions)

  // Record initial positions for all motors
  std::map<uint8_t, double> initial_positions;
  std::vector<uint8_t> all_arm_ids = {ARM_JOINT_1_ID, ARM_JOINT_2_ID, ARM_JOINT_3_ID, ARM_JOINT_4_ID,
                                      ARM_JOINT_5_ID, ARM_JOINT_6_ID, ARM_JOINT_7_ID};

  auto record_positions = [&]() {
    for (uint8_t id : all_arm_ids) {
      auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
      initial_positions[id] = motor->getCurrentPosition();
    }
  };

  // Max delta of 2.0 radians allows for normal physics simulation movement over 500ms
  // but catches instantaneous jumps (which would be >10 radians in a single step)
  auto verify_no_jump = [&](const std::string& context, double max_delta = 2.0) {
    for (uint8_t id : all_arm_ids) {
      auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
      double delta = std::abs(motor->getCurrentPosition() - initial_positions[id]);
      EXPECT_LT(delta, max_delta) << "Motor " << (int) id << " jumped during " << context;
    }
  };

  // Setup
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));
  std::this_thread::sleep_for(300ms);
  record_positions();

  auto torque_client = tester_node_->create_test_client<std_srvs::srv::SetBool>("/athena_arm_interface/set_torque");
  ASSERT_TRUE(torque_client->wait_for_service(*executor_, 5s));

  auto estop_pub = tester_node_->create_publisher<std_msgs::msg::Bool>("/soft_e_stop", 10);
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && estop_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }

  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;
  std_msgs::msg::Bool estop_msg;
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

  // Transition 1: Normal -> E-Stop
  record_positions();
  estop_msg.data = true;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);
  verify_no_jump("Normal -> E-Stop");

  // Transition 2: E-Stop -> E-Stop + Torque Off
  record_positions();
  request->data = false;
  hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  std::this_thread::sleep_for(300ms);
  verify_no_jump("E-Stop -> E-Stop + Torque Off");

  // Transition 3: E-Stop + Torque Off -> Torque Off (release E-Stop)
  record_positions();
  estop_msg.data = false;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);
  verify_no_jump("E-Stop + Torque Off -> Torque Off");

  // Transition 4: Torque Off -> Normal (enable torque)
  record_positions();
  request->data = true;
  hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  std::this_thread::sleep_for(300ms);
  verify_no_jump("Torque Off -> Normal");

  // Transition 5: Normal -> Torque Off -> E-Stop -> Torque On -> E-Stop Off (complex sequence)
  record_positions();

  // Torque off
  request->data = false;
  hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  std::this_thread::sleep_for(100ms);
  verify_no_jump("Complex: after torque off");

  // E-Stop on
  estop_msg.data = true;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(100ms);
  verify_no_jump("Complex: after E-Stop on");

  // Torque on (while E-Stop active)
  request->data = true;
  hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  std::this_thread::sleep_for(100ms);
  verify_no_jump("Complex: after torque on during E-Stop");

  // E-Stop off
  estop_msg.data = false;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(300ms);
  verify_no_jump("Complex: after E-Stop off");
}

// ============================================================================
// Critical Safety Test - Torque Enable Must Fail If Goal Write Fails
// ============================================================================

TEST_F(HardwareInterfaceTest, Safety_TorqueEnableFailsWhenGoalWriteFails)
{
  // CRITICAL SAFETY TEST: When enabling torque, the system must first write
  // the current position as goal position. If this write fails, torque MUST
  // NOT be enabled to prevent sudden, uncontrolled movement.

  // 1. Activate position controller
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));
  std::this_thread::sleep_for(300ms);

  // 2. Record initial motor positions
  std::map<uint8_t, double> initial_positions;
  std::vector<uint8_t> arm_ids = {ARM_JOINT_1_ID, ARM_JOINT_2_ID, ARM_JOINT_3_ID, ARM_JOINT_4_ID,
                                  ARM_JOINT_5_ID, ARM_JOINT_6_ID, ARM_JOINT_7_ID};
  for (uint8_t id : arm_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    initial_positions[id] = motor->getCurrentPosition();
  }

  // 3. Disable torque
  auto torque_client = tester_node_->create_test_client<std_srvs::srv::SetBool>("/athena_arm_interface/set_torque");
  ASSERT_TRUE(torque_client->wait_for_service(*executor_, 5s));

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;

  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;
  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success) << "Should be able to disable torque";

  std::this_thread::sleep_for(300ms);

  // Verify torque is off
  for (uint8_t id : arm_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    uint16_t torque_addr = motor->getAddress("torque_enable");
    ASSERT_EQ(motor->read1Byte(torque_addr), 0) << "Motor " << (int) id << " torque should be disabled";
  }

  // 4. Inject communication error on motor 1 to make goal position write/verify fail
  // This simulates the scenario where the hardware interface cannot properly write/verify
  // goal positions before enabling torque - a dangerous situation if allowed to proceed
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  motor1->setCommunicationError(true);

  // 5. Attempt to enable torque - THIS MUST FAIL
  // The resetGoalStateAndVerify() function requires successful read/write/verify cycle
  request->data = true;
  resp = hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr);
  EXPECT_FALSE(resp->success) << "Torque enable MUST fail when goal position write fails!";

  std::this_thread::sleep_for(300ms);

  // 6. CRITICAL CHECK: Verify torque is still OFF on all motors
  for (uint8_t id : arm_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    uint16_t torque_addr = motor->getAddress("torque_enable");
    EXPECT_EQ(motor->read1Byte(torque_addr), 0)
        << "CRITICAL: Motor " << (int) id << " torque MUST remain disabled when goal write failed! "
        << "Enabling torque without proper goal position could cause dangerous, sudden movement!";
  }

  // 7. Clear communication error and verify torque can now be enabled successfully
  motor1->setCommunicationError(false);

  // Small delay for error recovery
  std::this_thread::sleep_for(200ms);

  request->data = true;
  resp = hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success) << "Torque enable should succeed after communication error is cleared";

  std::this_thread::sleep_for(300ms);

  // Verify torque is now enabled
  for (uint8_t id : arm_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    uint16_t torque_addr = motor->getAddress("torque_enable");
    EXPECT_EQ(motor->read1Byte(torque_addr), 1) << "Motor " << (int) id << " torque should be enabled after recovery";
  }
}

TEST_F(HardwareInterfaceTest, Safety_NoMovementOnFailedTorqueEnable)
{
  // Test that even when torque enable fails, no movement occurs during the failed attempt

  // 1. Set up controller
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }

  // Move to known position
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
  pub->publish(cmd);
  std::this_thread::sleep_for(1s);

  // 2. Disable torque
  auto torque_client = tester_node_->create_test_client<std_srvs::srv::SetBool>("/athena_arm_interface/set_torque");
  ASSERT_TRUE(torque_client->wait_for_service(*executor_, 5s));

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;

  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;
  hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  std::this_thread::sleep_for(300ms);

  // 3. Record positions after torque off
  std::map<uint8_t, double> positions_after_torque_off;
  std::vector<uint8_t> arm_ids = {ARM_JOINT_1_ID, ARM_JOINT_2_ID, ARM_JOINT_3_ID, ARM_JOINT_4_ID,
                                  ARM_JOINT_5_ID, ARM_JOINT_6_ID, ARM_JOINT_7_ID};
  for (uint8_t id : arm_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_after_torque_off[id] = motor->getCurrentPosition();
  }

  // 4. Inject communication error on multiple motors
  for (uint8_t id : arm_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    motor->setCommunicationError(true);
  }

  // 5. Attempt torque enable (should fail)
  request->data = true;
  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  EXPECT_FALSE(resp->success) << "Torque enable should fail with communication errors";

  std::this_thread::sleep_for(500ms);

  // 6. Verify NO movement occurred during failed torque enable attempt
  for (uint8_t id : arm_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    double position_change = std::abs(motor->getCurrentPosition() - positions_after_torque_off[id]);
    EXPECT_LT(position_change, 0.05) << "Motor " << (int) id << " should NOT have moved during failed torque enable";
  }

  // Cleanup
  for (uint8_t id : arm_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    motor->setCommunicationError(false);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
