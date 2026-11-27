// One simulated Dynamixel bus. Its mock servos answer the real plugin on a pty and drive gz joints.

#include <gz/common/Console.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include <ament_index_cpp/get_package_share_path.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "dynamixel_ros_control/mock_bus_server.hpp"
#include "dynamixel_ros_control/servo_law.hpp"
#include "port_directory.hpp"

namespace dynamixel_ros_control_gz {

using dynamixel_ros_control::JointDrive;
using dynamixel_ros_control::MimicFollower;
using dynamixel_ros_control::MockBusServer;
using dynamixel_ros_control::MockDynamixel;
using dynamixel_ros_control::MockDynamixelManager;
using dynamixel_ros_control::ServoCommand;
using dynamixel_ros_control::ServoModel;
using dynamixel_ros_control::Transmission;

namespace {

/// A gz joint read and driven along its single axis.
class AxisJoint
{
public:
  AxisJoint(const gz::sim::Entity entity, gz::sim::EntityComponentManager& ecm)
      : joint_(entity)
  {
    const auto axis = joint_.Axis(ecm);
    if (!axis || axis->empty()) {
      throw std::runtime_error("Joint '" + joint_.Name(ecm).value_or("?") + "' has no axis");
    }
    axis_ = (*axis)[0].Xyz();
    prismatic_ = joint_.Type(ecm) == sdf::JointType::PRISMATIC;
    joint_.EnablePositionCheck(ecm);
    joint_.EnableVelocityCheck(ecm);
    // gz fills the wrench only on joints that carry the component
    joint_.EnableTransmittedWrenchCheck(ecm);
  }

  /// False until physics has stepped once.
  [[nodiscard]] bool read(const gz::sim::EntityComponentManager& ecm)
  {
    const auto position = joint_.Position(ecm);
    const auto velocity = joint_.Velocity(ecm);
    const auto wrench = joint_.TransmittedWrench(ecm);
    if (!position || position->empty() || !velocity || velocity->empty() || !wrench || wrench->empty()) {
      return false;
    }
    this->position = (*position)[0];
    this->velocity = (*velocity)[0];
    // What the parent applies to the child, in the joint frame
    const auto& load = prismatic_ ? (*wrench)[0].force() : (*wrench)[0].torque();
    effort = axis_.Dot(gz::math::Vector3d(load.x(), load.y(), load.z()));
    return true;
  }

  void drive(gz::sim::EntityComponentManager& ecm, const JointDrive& drive)
  {
    joint_.SetVelocity(ecm, {drive.velocity});
    joint_.SetEffortLimits(ecm, {gz::math::Vector2d(-drive.effort_limit, drive.effort_limit)});
  }

  void reset(gz::sim::EntityComponentManager& ecm, const double position)
  {
    joint_.ResetPosition(ecm, {position});
  }

  double position = 0.0;
  double velocity = 0.0;
  double effort = 0.0;

private:
  gz::sim::Joint joint_;
  gz::math::Vector3d axis_;
  bool prismatic_ = false;
};

struct Servo
{
  std::shared_ptr<MockDynamixel> motor;
  ServoModel model;
  Transmission transmission;
  std::vector<AxisJoint> joints;         ///< the servo joint, or its mimic followers when it has some
  std::vector<MimicFollower> followers;  ///< beside the servo joint, they carry its load
  std::vector<MimicFollower> distal;     ///< further out, they hold the finger's shape
  std::vector<double> follower_positions;
  std::vector<double> follower_efforts;
  std::vector<double> distal_positions;
  ServoCommand command;
};

/// The model's constants and factory EEPROM values from servo_models.yaml.
ServoModel loadModel(const YAML::Node& models, MockDynamixel& motor, const int model_number)
{
  const YAML::Node entry = models[model_number];
  if (!entry) {
    throw std::runtime_error("servo_models.yaml has no model " + std::to_string(model_number));
  }
  ServoModel model{entry["torque_constant"].as<double>(), entry["friction_torque"].as<double>()};
  model.bandwidth = entry["bandwidth"].as<double>(model.bandwidth);
  // The reported current divides by it
  if (!(model.torque_constant > 0.0)) {
    throw std::runtime_error("servo_models.yaml gives model " + std::to_string(model_number) +
                             " a torque_constant that is not positive");
  }
  for (const auto& value : entry["factory"]) {
    motor.setFactoryValue(value.first.as<std::string>(), value.second.as<double>());
  }
  return model;
}

/// The attribute or child @p key of @p element, which has no default
template <typename T>
T required(const sdf::Element& element, const std::string& key)
{
  const auto [value, found] = element.Get<T>(key, T{});
  if (!found) {
    throw std::runtime_error("<" + element.GetName() + "> has no " + key);
  }
  return value;
}

/// @p value as a @p T, refused if it does not fit
template <typename T>
T inRange(const int value, const std::string& key)
{
  if (value < std::numeric_limits<T>::min() || value > std::numeric_limits<T>::max()) {
    throw std::runtime_error("Servo " + key + " " + std::to_string(value) + " is out of range");
  }
  return static_cast<T>(value);
}

}  // namespace

class DynamixelBusSystem : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
public:
  ~DynamixelBusSystem() override
  {
    server_.reset();
    for (const auto& servo : servos_) {
      MockDynamixelManager::instance().removeMotor(servo.motor->getId());
    }
  }

  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& /*events*/) override
  {
    try {
      configure(gz::sim::Model(entity), *sdf, ecm);
    }
    catch (const std::exception& error) {
      // gz offers no way to fail a model, so the bus stays silent and the plugin cannot connect
      gzerr << "DynamixelBusSystem: " << error.what() << std::endl;
      server_.reset();
    }
  }

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override
  {
    if (info.paused || !server_) {
      return;
    }
    for (auto& servo : servos_) {
      for (auto& joint : servo.joints) {
        if (!joint.read(ecm)) {
          return;
        }
      }
    }
    // Here and not in the callback, so the plant overwrites the position a power cycle leaves within this step
    const bool power = !power_cut_;
    if (power != powered_) {
      server_->setPower(power);
      powered_ = power;
    }
    // Goals written just before take effect within this step, as on a real bus
    server_->waitUntilIdle();
    const double dt = std::chrono::duration<double>(info.dt).count();
    for (auto& servo : servos_) {
      step(servo, dt, ecm);
    }
  }

private:
  // 253 to 255 are reserved, 254 being the broadcast id every motor answers
  static constexpr uint8_t MAX_SERVO_ID = 252;

  void configure(const gz::sim::Model& model, const sdf::Element& sdf, gz::sim::EntityComponentManager& ecm)
  {
    const YAML::Node models = YAML::LoadFile(
        (ament_index_cpp::get_package_share_path("dynamixel_ros_control_gz") / "config/servo_models.yaml").string());
    // Windows count the physics dt, the sim time that paces a controller manager under use_sim_time
    MockDynamixelManager::instance().setBusWatchdogTiming(true, true);
    std::vector<uint8_t> ids;
    for (auto element = sdf.FindElement("servo"); element; element = element->GetNextElement("servo")) {
      const auto id = inRange<uint8_t>(required<int>(*element, "id"), "id");
      if (id > MAX_SERVO_ID) {
        throw std::runtime_error("Servo id " + std::to_string(id) + " is reserved");
      }
      const auto model_number = inRange<uint16_t>(required<int>(*element, "model"), "model");
      const Transmission transmission{required<double>(*element, "reduction"), required<double>(*element, "offset")};
      if (transmission.reduction == 0.0 || !std::isfinite(transmission.reduction)) {
        throw std::runtime_error("Servo id " + std::to_string(id) + " has a reduction that is zero or not finite");
      }
      const auto joint = required<std::string>(*element, "joint");
      const auto initial = required<double>(*element, "initial");
      auto& manager = MockDynamixelManager::instance();
      if (manager.getMotor(id)) {
        throw std::runtime_error("Servo id " + std::to_string(id) + " is on another bus already");
      }
      manager.addMotor(id, model_number);
      ids.push_back(id);
      Servo& servo = servos_.emplace_back();
      servo.motor = manager.getMotor(id);
      servo.model = loadModel(models, *servo.motor, model_number);
      servo.transmission = transmission;
      addJoints(servo, model, joint, initial, ecm);
    }
    const std::filesystem::path port = required<std::string>(sdf, "port");
    securePortDirectory(port.parent_path());
    server_ = std::make_unique<MockBusServer>(ids, port);
    // Under the model name, as the robot namespace a ROS bridge expands its gz topics in
    const std::string topic = "/" + model.Name(ecm) + "/dynamixel/power_cut";
    if (!node_.Subscribe(topic, &DynamixelBusSystem::onPowerCut, this)) {
      throw std::runtime_error("Cannot subscribe to " + topic);
    }
    gzmsg << "DynamixelBusSystem: " << ids.size() << " servos on " << port.string() << ", power cut by " << topic
          << std::endl;
  }

  /// dartsim builds no mimic constraint, so a servo joint that others mimic drives them instead.
  static void addJoints(Servo& servo, const gz::sim::Model& model, const std::string& name, const double initial,
                        gz::sim::EntityComponentManager& ecm)
  {
    const gz::sim::Entity entity = model.JointByName(ecm, name);
    if (entity == gz::sim::kNullEntity) {
      throw std::runtime_error("Model has no joint '" + name + "'");
    }
    std::vector<std::pair<gz::sim::Entity, MimicFollower>> followers;
    for (const gz::sim::Entity candidate : model.Joints(ecm)) {
      const auto axis = gz::sim::Joint(candidate).Axis(ecm);
      const auto mimic = axis && !axis->empty() ? (*axis)[0].Mimic() : std::nullopt;
      if (mimic && mimic->Joint() == name) {
        followers.emplace_back(candidate, MimicFollower{mimic->Multiplier(),
                                                        mimic->Offset() - mimic->Multiplier() * mimic->Reference()});
      }
    }
    const auto parent = gz::sim::Joint(entity).ParentLinkName(ecm);
    const auto distal = std::stable_partition(followers.begin(), followers.end(), [&](const auto& follower) {
      return gz::sim::Joint(follower.first).ParentLinkName(ecm) == parent;
    });
    for (auto follower = followers.begin(); follower != followers.end(); ++follower) {
      servo.joints.emplace_back(follower->first, ecm);
      servo.joints.back().reset(ecm, follower->second.multiplier * initial + follower->second.offset);
      // Without a follower beside the servo joint every follower carries the load
      const bool carries_load = follower < distal || distal == followers.begin();
      (carries_load ? servo.followers : servo.distal).push_back(follower->second);
    }
    if (servo.followers.empty()) {
      servo.joints.emplace_back(entity, ecm);
    }
    gz::sim::Joint(entity).ResetPosition(ecm, {initial});
    servo.follower_positions.resize(servo.followers.size());
    servo.follower_efforts.resize(servo.followers.size());
    servo.distal_positions.resize(servo.distal.size());
  }

  static void step(Servo& servo, const double dt, gz::sim::EntityComponentManager& ecm)
  {
    double position = servo.joints[0].position;
    double velocity = servo.joints[0].velocity;
    double effort = servo.joints[0].effort;
    if (!servo.followers.empty()) {
      for (size_t i = 0; i < servo.followers.size(); ++i) {
        servo.follower_positions[i] = servo.joints[i].position;
        servo.follower_efforts[i] = servo.joints[i].effort;
      }
      for (size_t i = 0; i < servo.distal.size(); ++i) {
        servo.distal_positions[i] = servo.joints[servo.followers.size() + i].position;
      }
      position = servo.followers[0].servoPosition(position);
      velocity /= servo.followers[0].multiplier;
      effort = dynamixel_ros_control::servoTorque(servo.followers, servo.follower_efforts);
    }
    const auto& transmission = servo.transmission;
    // The current of the last command, the one that produced this load
    const double current = dynamixel_ros_control::servoCurrent(servo.command, servo.model, transmission, effort);
    servo.command = servo.motor->followPlant(dt, transmission.actuatorPosition(position),
                                             transmission.actuatorVelocity(velocity), current);
    const JointDrive drive = dynamixel_ros_control::servoDrive(servo.command, servo.model, transmission, position);
    if (servo.followers.empty()) {
      servo.joints[0].drive(ecm, drive);
      return;
    }
    auto drives = dynamixel_ros_control::followerDrives(drive, servo.model.bandwidth, position, servo.followers,
                                                        servo.follower_positions);
    // Fed the commanded speed a distal joint would curl to its limit while its stalled base waits
    const auto distal = dynamixel_ros_control::followerDrives({velocity, drive.effort_limit}, servo.model.bandwidth,
                                                              position, servo.distal, servo.distal_positions);
    drives.insert(drives.end(), distal.begin(), distal.end());
    for (size_t i = 0; i < drives.size(); ++i) {
      servo.joints[i].drive(ecm, drives[i]);
    }
  }

  void onPowerCut(const gz::msgs::Boolean& message)
  {
    power_cut_ = message.data();
  }

  std::vector<Servo> servos_;
  std::unique_ptr<MockBusServer> server_;
  std::atomic<bool> power_cut_{false};
  bool powered_ = true;
  // Last, so its subscription is gone before the state its callback writes
  gz::transport::Node node_;
};

}  // namespace dynamixel_ros_control_gz

GZ_ADD_PLUGIN(dynamixel_ros_control_gz::DynamixelBusSystem, gz::sim::System, gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)
