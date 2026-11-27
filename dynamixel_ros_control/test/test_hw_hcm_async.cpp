// The plugin as an asynchronous component of hector_controller_manager at 100 Hz, so the arm's bus
// watchdog window of 4 cycles is 40 ms. The mock counts the watchdog in control cycles, the control
// loop waits for the owner before each tick, and the owner thread is held in write() for whole cycles.

#include <dynamixel_ros_control/dynamixel_hardware_interface.hpp>
#include <hector_controller_manager/controller_manager.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <thread>

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

namespace {

constexpr int kUpdateRate = 100;
constexpr double kPeriod = 1.0 / kUpdateRate;
constexpr double kArmVelocity = 0.3;
constexpr double kStopped = 0.01;
const std::string kArm = "athena_arm_interface";
const std::string kEStopTopic = "/dxl_hcm_async_estop";

/// Holds the owner thread of the arm in write(), before the bus, for a number of control cycles, and
/// lets the control loop wait for the owner, so the mock's bus time never counts a late owner
class OwnerStall
{
public:
  /// Returns the holds so far, for waitHeld()
  uint64_t holdNextWrite(const int cycles)
  {
    next_.store(cycles);
    return holds();
  }

  void holdEveryWrite(const int cycles)
  {
    every_.store(cycles);
  }

  /// The control loop, before it triggers the owner
  void cycleStarts()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    trigger_cycle_ = cycle_;
    reads_at_start_ = reads_;
  }

  /// Owner, at the start of every write(). The ticks of the next @p cycles cycles pass while it is held.
  void beforeWrite()
  {
    int cycles = next_.exchange(0);
    if (cycles == 0) {
      cycles = every_.load();
    }
    if (cycles == 0) {
      return;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    hold_cycle_ = trigger_cycle_;
    hold_until_ = trigger_cycle_ + 1 + static_cast<uint64_t>(cycles);
    ++holds_;
    held_ = true;
    changed_.notify_all();
    changed_.wait(lock, [&] { return cycle_ >= hold_until_ || released_; });
    held_ = false;
    changed_.notify_all();
  }

  /// Owner, at the end of every read(), the last call of its cycle
  void afterRead()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    ++reads_;
    changed_.notify_all();
  }

  /// The control loop, after every cycle. Returns once the owner finished a cycle or stays held, or
  /// after @p timeout when the component is not cycling, as in a mode switch or while unconfigured.
  void cycleDone(const std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ++cycle_;
    changed_.notify_all();
    changed_.wait_for(lock, timeout,
                      [&] { return reads_ > reads_at_start_ || (held_ && cycle_ < hold_until_) || released_; });
  }

  uint64_t holds()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return holds_;
  }

  /// The cycle that triggered the first hold after @p holds_before, nullopt if none comes in time
  std::optional<uint64_t> waitHeld(const uint64_t holds_before, const std::chrono::seconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!changed_.wait_for(lock, timeout, [&] { return holds_ > holds_before; })) {
      return std::nullopt;
    }
    return hold_cycle_;
  }

  bool waitReleased(const std::chrono::seconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return changed_.wait_for(lock, timeout, [&] { return !held_; });
  }

  bool held()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return held_;
  }

  uint64_t cycle()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return cycle_;
  }

  /// Waits until @p cycles more control cycles ran
  void waitCycles(const uint64_t cycles)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const uint64_t until = cycle_ + cycles;
    changed_.wait(lock, [&] { return cycle_ >= until; });
  }

  /// Lets a held owner go for good, before the controller manager shuts down
  void release()
  {
    every_.store(0);
    next_.store(0);
    const std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    changed_.notify_all();
  }

private:
  std::atomic<int> next_{0};
  std::atomic<int> every_{0};
  std::mutex mutex_;
  std::condition_variable changed_;
  uint64_t cycle_ = 0;
  uint64_t trigger_cycle_ = 0;
  uint64_t reads_ = 0;
  uint64_t reads_at_start_ = 0;
  uint64_t holds_ = 0;
  uint64_t hold_cycle_ = 0;
  uint64_t hold_until_ = 0;
  bool held_ = false;
  bool released_ = false;
};

class StalledDynamixel : public DynamixelHardwareInterface
{
public:
  explicit StalledDynamixel(OwnerStall* stall)
      : stall_(stall)
  {}

  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override
  {
    if (stall_ != nullptr) {
      stall_->beforeWrite();
    }
    return DynamixelHardwareInterface::write(time, period);
  }

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override
  {
    const auto result = DynamixelHardwareInterface::read(time, period);
    if (stall_ != nullptr) {
      stall_->afterRead();
    }
    return result;
  }

private:
  OwnerStall* stall_;
};

void replaceOnce(std::string& text, const std::string& from, const std::string& to)
{
  const auto pos = text.find(from);
  ASSERT_NE(pos, std::string::npos) << from;
  text.replace(pos, from.size(), to);
}

/// Both components asynchronous, and EEPROM limits on arm_joint_2 for its cap
std::string asyncRobot()
{
  std::string urdf = load_file(std::string(TEST_CONFIG_DIR) + "/athena.urdf");
  for (const std::string component : {"athena_arm_interface", "athena_flipper_interface"}) {
    replaceOnce(urdf, "<ros2_control name=\"" + component + "\" type=\"system\">",
                "<ros2_control name=\"" + component + "\" type=\"system\" is_async=\"true\">");
  }
  const std::string id = "<param name=\"id\">" + std::to_string(ARM_JOINT_2_ID) + "</param>";
  replaceOnce(
      urdf, id,
      id + "<param name=\"registers.current_limit\">5</param><param name=\"registers.velocity_limit\">2</param>");
  return urdf;
}

}  // namespace

class HcmAsyncTest : public HectorTestFixture
{
protected:
  void SetUp() override
  {
    test_home_.emplace();
    HectorTestFixture::SetUp();

    const std::string urdf = asyncRobot();
    addDummyMotors(hardware_interface::parse_control_resources_from_urdf(urdf));
    // Reads do not feed it, the strictest reading of the e-manual
    MockDynamixelManager::instance().setBusWatchdogTiming(true, false);

    estop_ =
        tester_node_->create_publisher<std_msgs::msg::Bool>(kEStopTopic, rclcpp::QoS(1).reliable().transient_local());
    publishEStop(false);

    const std::string controllers_yaml = std::string(TEST_CONFIG_DIR) + "/controllers.yaml";
    std::vector<rclcpp::Parameter> parameters = {
        {"robot_description", urdf},
        {"update_rate", kUpdateRate},
        {"hardware_hang_timeout_ms", 100},
        {"autostart_cooldown_ms", 0},
        {"estop_topics", std::vector<std::string>{kEStopTopic}},
        {"startup.hardware_active", std::vector<std::string>{kArm}},
        {"hardware_components_initial_state.unconfigured", std::vector<std::string>{"athena_flipper_interface"}},
        {"startup.controllers_loaded", std::vector<std::string>{"arm_velocity_controller", "arm_cap_controller"}},
        {"startup.controllers_active", startupControllers()},
        {"arm_velocity_controller.type", "forward_command_controller/ForwardCommandController"},
        {"arm_velocity_controller.params_file", controllers_yaml},
        {"arm_cap_controller.type", "forward_command_controller/MultiInterfaceForwardCommandController"},
        {"arm_cap_controller.node_options_args",
         std::vector<std::string>{"-p", "joint:=arm_joint_2", "-p", "interface_names:=[position, max_current]"}},
    };
    auto options = hector_controller_manager::get_cm_node_options();
    options.parameter_overrides(parameters);
    cm_ = std::make_shared<hector_controller_manager::ControllerManager>(
        nullptr, "controller_manager", "", options,
        [this](const hardware_interface::HardwareInfo& info)
            -> std::unique_ptr<hardware_interface::HardwareComponentInterface> {
          return std::make_unique<StalledDynamixel>(info.name == kArm ? &stall_ : nullptr);
        });
    cm_executor_ = cm_->executor();
    cm_executor_->add_node(cm_);
    cm_spin_thread_ = std::thread([this] { cm_executor_->spin(); });
    loop_ = std::thread([this] { controlLoop(); });

    ASSERT_TRUE(waitFor([this] { return hardwareState(kArm) == "active"; }, scaled(30s))) << hardwareState(kArm);
    for (const std::string controller : {"arm_velocity_controller", "arm_cap_controller"}) {
      // Configured, so a strict switch can start it
      ASSERT_TRUE(waitFor(
          [&] {
            const std::string state = controllerState(controller);
            return state == "inactive" || state == "active";
          },
          scaled(10s)))
          << controller;
    }
    for (const auto& controller : startupControllers()) {
      ASSERT_TRUE(waitFor([&] { return controllerState(controller) == "active"; }, scaled(10s))) << controller;
    }
  }

  void TearDown() override
  {
    stall_.release();
    if (cm_) {
      cm_->shutdown();
    }
    loop_running_ = false;
    if (loop_.joinable()) {
      loop_.join();
    }
    if (cm_executor_) {
      cm_executor_->cancel();
    }
    if (cm_spin_thread_.joinable()) {
      cm_spin_thread_.join();
    }
    publishers_.clear();
    estop_.reset();
    cm_.reset();
    cm_executor_.reset();
    MockDynamixelManager::instance().reset();
    // rclcpp stays up for the next test, as the controller manager's own tests keep it
    executor_.reset();
    tester_node_.reset();
    test_home_.reset();
  }

  virtual std::vector<std::string> startupControllers() const
  {
    return {};
  }

  /// The mock steps one nominal period per cycle and only after the owner served the cycle before,
  /// so its bus time counts cycles and holds, never a late owner
  void controlLoop()
  {
    const auto period = std::chrono::nanoseconds(1'000'000'000 / kUpdateRate);
    const rclcpp::Duration duration(period);
    auto next = std::chrono::steady_clock::now();
    while (loop_running_) {
      MockDynamixelManager::instance().update(kPeriod);
      stall_.cycleStarts();
      const rclcpp::Time now = cm_->now();
      cm_->read(now, duration);
      cm_->update(now, duration);
      cm_->write(now, duration);
      stall_.cycleDone(50ms);
      next = std::max(next + period, std::chrono::steady_clock::now());
      std::this_thread::sleep_until(next);
    }
  }

  static bool waitFor(const std::function<bool()>& condition, const std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!condition()) {
      if (std::chrono::steady_clock::now() > deadline) {
        return false;
      }
      std::this_thread::sleep_for(1ms);
    }
    return true;
  }

  std::string hardwareState(const std::string& name)
  {
    auto response = std::make_shared<ListHardwareComponents::Response>();
    cm_->list_hardware_components_srv_cb(std::make_shared<ListHardwareComponents::Request>(), response);
    for (const auto& component : response->component) {
      if (component.name == name) {
        return component.state.label;
      }
    }
    return "";
  }

  std::string controllerState(const std::string& name)
  {
    auto response = std::make_shared<ListControllers::Response>();
    cm_->list_controllers_srv_cb(std::make_shared<ListControllers::Request>(), response);
    for (const auto& controller : response->controller) {
      if (controller.name == name) {
        return controller.state;
      }
    }
    return "";
  }

  bool switchControllers(const std::vector<std::string>& activate, const std::vector<std::string>& deactivate)
  {
    auto request = std::make_shared<SwitchController::Request>();
    request->activate_controllers = activate;
    request->deactivate_controllers = deactivate;
    request->strictness = SwitchController::Request::STRICT;
    auto response = std::make_shared<SwitchController::Response>();
    cm_->switch_controller_service_cb(request, response);
    return response->ok;
  }

  bool setHardwareState(const std::string& name, const uint8_t id, const std::string& label)
  {
    auto request = std::make_shared<SetHardwareComponentState::Request>();
    request->name = name;
    request->target_state.id = id;
    request->target_state.label = label;
    auto response = std::make_shared<SetHardwareComponentState::Response>();
    cm_->set_hardware_component_state_srv_cb(request, response);
    return response->ok;
  }

  void command(const std::string& controller, const std::vector<double>& values)
  {
    auto& publisher = publishers_[controller];
    if (!publisher) {
      publisher = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/" + controller + "/commands",
                                                                                   rclcpp::QoS(10).reliable());
    }
    std_msgs::msg::Float64MultiArray message;
    message.data = values;
    publisher->publish(message);
  }

  void publishEStop(const bool engaged)
  {
    std_msgs::msg::Bool message;
    message.data = engaged;
    estop_->publish(message);
  }

  static std::shared_ptr<MockDynamixel> motor(const uint8_t id)
  {
    return MockDynamixelManager::instance().getMotor(id);
  }

  static double armVelocity()
  {
    return motor(ARM_JOINT_1_ID)->getCurrentVelocity();
  }

  static int armTrips()
  {
    int trips = 0;
    for (const auto id : ARM_MOTOR_IDS) {
      trips += motor(id)->timedBusWatchdogTrips();
    }
    return trips;
  }

  static bool armWatchdogArmed()
  {
    for (const auto id : ARM_MOTOR_IDS) {
      if (motor(id)->busWatchdogLatched() || motor(id)->getBusWatchdog() == 0) {
        return false;
      }
    }
    return true;
  }

  /// Turns arm_joint_1 in velocity mode, the other arm joints at rest
  void turnArm()
  {
    if (controllerState("arm_velocity_controller") != "active") {
      ASSERT_TRUE(switchControllers({"arm_velocity_controller"}, {}));
    }
    // The subscription may match after the first message
    ASSERT_TRUE(waitFor(
        [this] {
          command("arm_velocity_controller", {kArmVelocity, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
          return std::abs(armVelocity() - kArmVelocity) < 0.02;
        },
        scaled(10s)))
        << "arm_joint_1 turns at " << armVelocity();
  }

  /// Whether arm_joint_1 stays below kStopped for @p cycles cycles
  bool staysStopped(const uint64_t cycles)
  {
    const uint64_t until = stall_.cycle() + cycles;
    while (stall_.cycle() < until) {
      if (std::abs(armVelocity()) >= kStopped) {
        return false;
      }
      std::this_thread::sleep_for(1ms);
    }
    return true;
  }

  /// The cycles until arm_joint_1 is below kStopped, from @p since
  std::optional<uint64_t> cyclesUntilStopped(const uint64_t since, const std::chrono::seconds timeout)
  {
    if (!waitFor([] { return std::abs(armVelocity()) < kStopped; }, timeout)) {
      return std::nullopt;
    }
    return stall_.cycle() - since;
  }

  std::optional<TestHome> test_home_;
  OwnerStall stall_;
  std::shared_ptr<hector_controller_manager::ControllerManager> cm_;
  std::shared_ptr<rclcpp::Executor> cm_executor_;
  std::thread cm_spin_thread_;
  std::atomic<bool> loop_running_{true};
  std::thread loop_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr> publishers_;
};

// Below and at the window: the last packet before a stall of 3 cycles and the first after it are
// 4 cycles apart, which the window still allows
TEST_F(HcmAsyncTest, OwnerStallsUpToThreeCyclesNeverTripTheArmWatchdog)
{
  turnArm();
  for (int stall = 1; stall <= 3; ++stall) {
    for (int repeat = 0; repeat < 5; ++repeat) {
      const uint64_t holds = stall_.holdNextWrite(stall);
      ASSERT_TRUE(stall_.waitHeld(holds, scaled(5s)));
      ASSERT_TRUE(stall_.waitReleased(scaled(5s)));
      stall_.waitCycles(10);
    }
  }
  EXPECT_EQ(armTrips(), 0);
  EXPECT_NEAR(armVelocity(), kArmVelocity, 0.02);
  EXPECT_EQ(controllerState("arm_velocity_controller"), "active");
  EXPECT_EQ(hardwareState(kArm), "active");
}

// Above the window the motor stops by itself while the owner is held, and the write that follows
// re-arms the watchdog and the joint turns on, all before the hang timeout
TEST_F(HcmAsyncTest, AStallAboveTheWindowStopsTheArmUntilTheNextWrite)
{
  turnArm();
  const uint64_t holds = stall_.holdNextWrite(6);
  const auto held = stall_.waitHeld(holds, scaled(5s));
  ASSERT_TRUE(held);
  const auto stopped = cyclesUntilStopped(*held, scaled(5s));
  ASSERT_TRUE(stopped) << "the watchdog did not stop the arm";
  EXPECT_LE(*stopped, 5U) << "the watchdog stopped the arm only after " << *stopped << " cycles";
  EXPECT_TRUE(stall_.held()) << "the arm stopped only after the owner went on";
  EXPECT_GT(armTrips(), 0);

  ASSERT_TRUE(stall_.waitReleased(scaled(5s)));
  EXPECT_TRUE(waitFor([] { return std::abs(armVelocity() - kArmVelocity) < 0.02; }, scaled(1s)))
      << "the arm did not turn on after the stall";
  EXPECT_TRUE(armWatchdogArmed());
  EXPECT_EQ(controllerState("arm_velocity_controller"), "active");
  EXPECT_EQ(hardwareState(kArm), "active");
}

// A component whose owner hangs past the hang timeout faults and its controllers stop, while the
// watchdog has stopped the arm long before. The late write cannot set it going again for longer
// than the fault handling takes, and the arm comes back by a hardware state request alone.
TEST_F(HcmAsyncTest, AnOwnerHungPastTheHangTimeoutFaultsTheArmAndItStaysStopped)
{
  turnArm();
  const uint64_t holds = stall_.holdNextWrite(30);
  const auto held = stall_.waitHeld(holds, scaled(5s));
  ASSERT_TRUE(held);
  const auto stopped = cyclesUntilStopped(*held, scaled(5s));
  ASSERT_TRUE(stopped);
  EXPECT_LE(*stopped, 5U);
  // Past the hang timeout and still held. The hcm README has the component fault and report a
  // call that outlives its deadline in a transition state, and its controllers stop.
  stall_.waitCycles(15);
  ASSERT_TRUE(stall_.held());
  EXPECT_NE(controllerState("arm_velocity_controller"), "active");
  EXPECT_NE(hardwareState(kArm), "active");

  ASSERT_TRUE(stall_.waitReleased(scaled(5s)));
  EXPECT_TRUE(waitFor([this] { return controllerState("arm_velocity_controller") == "inactive"; }, scaled(5s)));
  const double released_at = motor(ARM_JOINT_1_ID)->getJointPosition();
  EXPECT_TRUE(waitFor([this] { return hardwareState(kArm) != "active"; }, scaled(5s))) << hardwareState(kArm);
  ASSERT_TRUE(waitFor([] { return std::abs(armVelocity()) < kStopped; }, scaled(2s)));
  EXPECT_TRUE(staysStopped(100));
  RecordProperty("travel_after_hang_rad", std::to_string(motor(ARM_JOINT_1_ID)->getJointPosition() - released_at));
  EXPECT_NEAR(motor(ARM_JOINT_1_ID)->getJointPosition(), released_at, 0.01);

  ASSERT_TRUE(setHardwareState(kArm, lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active"));
  EXPECT_TRUE(armWatchdogArmed());
  EXPECT_TRUE(staysStopped(50));
  turnArm();
}

// An owner at a third of the loop rate skips two triggers of three, yet writes often enough for
// the window
TEST_F(HcmAsyncTest, AnOwnerSlowerThanTheLoopKeepsTheArmTurning)
{
  turnArm();
  const double start = motor(ARM_JOINT_1_ID)->getJointPosition();
  const uint64_t since = stall_.cycle();
  stall_.holdEveryWrite(2);
  stall_.waitCycles(200);
  stall_.holdEveryWrite(0);
  const double travel = motor(ARM_JOINT_1_ID)->getJointPosition() - start;
  const double expected = kArmVelocity * static_cast<double>(stall_.cycle() - since) * kPeriod;
  EXPECT_NEAR(travel, expected, 0.1 * expected);
  EXPECT_EQ(armTrips(), 0);
  EXPECT_EQ(controllerState("arm_velocity_controller"), "active");
  EXPECT_EQ(hardwareState(kArm), "active");
}

// The e-stop stops the controller while the owner is held, and the next write after it holds the
// arm. Gaps of the e-stop path beyond the stall would show as extra cycles.
TEST_F(HcmAsyncTest, AnEStopDuringAHeldWriteStopsTheArmRightAfterIt)
{
  turnArm();
  const uint64_t holds = stall_.holdNextWrite(3);
  const auto held = stall_.waitHeld(holds, scaled(5s));
  ASSERT_TRUE(held);
  publishEStop(true);
  const auto stopped = cyclesUntilStopped(*held, scaled(5s));
  ASSERT_TRUE(stopped);
  RecordProperty("estop_to_stop_cycles", std::to_string(*stopped));
  EXPECT_LE(*stopped, 6U) << "the arm stopped " << *stopped << " cycles after the e-stop during a 3 cycle stall";
  EXPECT_EQ(controllerState("arm_velocity_controller"), "inactive");
  EXPECT_TRUE(staysStopped(50));
  EXPECT_EQ(armTrips(), 0);
}

// The release brings the controller back, but never its command from before the e-stop
class HcmAsyncAutostartTest : public HcmAsyncTest
{
protected:
  std::vector<std::string> startupControllers() const override
  {
    return {"arm_velocity_controller"};
  }
};

TEST_F(HcmAsyncAutostartTest, AnEStopReleaseDoesNotRestartTheArmsLastVelocity)
{
  turnArm();
  const uint64_t holds = stall_.holdNextWrite(3);
  ASSERT_TRUE(stall_.waitHeld(holds, scaled(5s)));
  publishEStop(true);
  ASSERT_TRUE(waitFor([this] { return controllerState("arm_velocity_controller") == "inactive"; }, scaled(5s)));
  ASSERT_TRUE(waitFor([] { return std::abs(armVelocity()) < kStopped; }, scaled(5s)));

  publishEStop(false);
  ASSERT_TRUE(waitFor([this] { return controllerState("arm_velocity_controller") == "active"; }, scaled(10s)));
  EXPECT_TRUE(staysStopped(100));
  EXPECT_TRUE(armWatchdogArmed());
}

// A controller stopped while the owner is held hands its joint over to the hold, and a component
// deactivated while the owner is held stops its joints itself
TEST_F(HcmAsyncTest, StoppingTheControllerAndThenTheArmDuringHeldWritesStopsIt)
{
  turnArm();
  const uint64_t holds = stall_.holdNextWrite(3);
  ASSERT_TRUE(stall_.waitHeld(holds, scaled(5s)));
  ASSERT_TRUE(switchControllers({}, {"arm_velocity_controller"}));
  ASSERT_TRUE(waitFor([] { return std::abs(armVelocity()) < kStopped; }, scaled(2s)));
  EXPECT_TRUE(staysStopped(50));

  turnArm();
  ASSERT_TRUE(switchControllers({}, {"arm_velocity_controller"}));
  ASSERT_TRUE(waitFor([] { return std::abs(armVelocity()) < kStopped; }, scaled(2s)));
  const uint64_t holds_again = stall_.holdNextWrite(3);
  ASSERT_TRUE(stall_.waitHeld(holds_again, scaled(5s)));
  EXPECT_TRUE(setHardwareState(kArm, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive"));
  EXPECT_EQ(hardwareState(kArm), "inactive");
  EXPECT_TRUE(staysStopped(100));
  EXPECT_EQ(armTrips(), 0) << "the arm was left to its watchdog";
}

// The bus goes away while the owner is held. Only the watchdog can stop the arm then, and the read
// errors fault the component. Once the bus is back a hardware state request recovers it.
TEST_F(HcmAsyncTest, ABusLostDuringAHeldWriteStopsTheArmAndFaultsTheComponent)
{
  turnArm();
  const uint64_t holds = stall_.holdNextWrite(3);
  const auto held = stall_.waitHeld(holds, scaled(5s));
  ASSERT_TRUE(held);
  MockDynamixelManager::instance().setGlobalCommunicationError(true);
  const auto stopped = cyclesUntilStopped(*held, scaled(5s));
  ASSERT_TRUE(stopped);
  EXPECT_LE(*stopped, 6U);
  EXPECT_TRUE(waitFor([this] { return controllerState("arm_velocity_controller") == "inactive"; }, scaled(5s)));
  EXPECT_TRUE(waitFor([this] { return hardwareState(kArm) != "active"; }, scaled(5s)));
  EXPECT_TRUE(staysStopped(50));

  MockDynamixelManager::instance().setGlobalCommunicationError(false);
  ASSERT_TRUE(setHardwareState(kArm, lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active"));
  EXPECT_TRUE(armWatchdogArmed());
  EXPECT_TRUE(staysStopped(50));
}

// The cap travels the async copy like any command. It is clamped to the EEPROM limit, and the limit
// again once the controller releases it, also while the owner is held at the release
TEST_F(HcmAsyncTest, TheArmCapIsClampedAndItsReleaseDuringAHeldWriteRestoresTheLimit)
{
  const auto arm_joint_2 = motor(ARM_JOINT_2_ID);
  const auto goal_current = [&arm_joint_2] {
    return static_cast<int16_t>(arm_joint_2->read2Byte(arm_joint_2->getAddress("goal_current")));
  };
  ASSERT_TRUE(switchControllers({"arm_cap_controller"}, {}));
  for (const auto& [cap, ticks] : std::vector<std::pair<double, int16_t>>{{1.5, 1500}, {9.0, 5000}, {1.5, 1500}}) {
    EXPECT_TRUE(waitFor(
        [&] {
          command("arm_cap_controller", {arm_joint_2->getJointPosition(), cap});
          return goal_current() == ticks;
        },
        scaled(10s)))
        << "a cap of " << cap << " A wrote " << goal_current();
  }

  const uint64_t holds = stall_.holdNextWrite(3);
  ASSERT_TRUE(stall_.waitHeld(holds, scaled(5s)));
  ASSERT_TRUE(switchControllers({}, {"arm_cap_controller"}));
  EXPECT_TRUE(waitFor([&] { return goal_current() == 5000; }, scaled(2s))) << goal_current();
  EXPECT_EQ(hardwareState(kArm), "active");
}

}  // namespace dynamixel_ros_control::test
