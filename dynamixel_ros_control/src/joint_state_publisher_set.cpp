#include "dynamixel_ros_control/joint_state_publisher_set.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <limits>

#include "dynamixel_ros_control/common.hpp"

namespace dynamixel_ros_control {

namespace {

template <typename Pick>
inline void fill(sensor_msgs::msg::JointState& msg, const std::vector<const Joint*>& joints, const rclcpp::Time& stamp,
                 Pick pick)
{
  msg.header.stamp = stamp;
  const size_t n = joints.size();
  for (size_t i = 0; i < n; ++i) {
    const auto& m = pick(*joints[i]);
    auto p = m.find(hardware_interface::HW_IF_POSITION);
    msg.position[i] = (p != m.end()) ? p->second : std::numeric_limits<double>::quiet_NaN();
    auto v = m.find(hardware_interface::HW_IF_VELOCITY);
    msg.velocity[i] = (v != m.end()) ? v->second : std::numeric_limits<double>::quiet_NaN();
    // sensor_msgs/JointState only has `effort`. The Dynamixel hardware interface treats
    // HW_IF_EFFORT and HW_IF_CURRENT as alternative names for the same physical quantity, so fall back to HW_IF_CURRENT
    // when a joint exposes the current interface instead of effort.
    auto e = m.find(hardware_interface::HW_IF_EFFORT);
    if (e == m.end()) {
      e = m.find(hardware_interface::HW_IF_CURRENT);
    }
    msg.effort[i] = (e != m.end()) ? e->second : std::numeric_limits<double>::quiet_NaN();
  }
}

}  // namespace

bool JointStatePublisherSet::init(const rclcpp::Node::SharedPtr& node,
                                  const std::unordered_map<std::string, std::string>& hardware_parameters,
                                  const std::unordered_map<std::string, Joint>& joints,
                                  const std::vector<std::string>& joint_names)
{
  bool publish_goal = false;
  bool publish_read = false;
  bool publish_write = false;
  if (!getOptionalParameter(hardware_parameters, "publish_goal_joint_states", publish_goal, false) ||
      !getOptionalParameter(hardware_parameters, "publish_read_joint_states", publish_read, false) ||
      !getOptionalParameter(hardware_parameters, "publish_write_joint_states", publish_write, false)) {
    return false;
  }

  const auto make = [&node, &joint_names](const std::string& topic) {
    auto p = std::make_unique<Publisher>();
    auto pub = node->create_publisher<sensor_msgs::msg::JointState>(topic, rclcpp::SystemDefaultsQoS());
    p->publisher = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(pub);
    auto& m = p->msg;
    m.name.assign(joint_names.begin(), joint_names.end());
    m.position.assign(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
    m.velocity.assign(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
    m.effort.assign(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
    return p;
  };

  if (publish_goal) {
    goal_ = make("~/goal_joint_states");
  }
  if (publish_read) {
    read_ = make("~/read_joint_states");
  }
  if (publish_write) {
    write_ = make("~/write_joint_states");
  }

  if (!goal_ && !read_ && !write_) {
    return true;
  }
  // Assigned, not appended: fill() sizes its loop from this vector but writes into message vectors
  // sized by joint_names, so a second init() that doubled it would run off the end of those.
  joints_.clear();
  joints_.reserve(joint_names.size());
  for (const auto& name : joint_names) {
    joints_.push_back(&joints.at(name));
  }
  return true;
}

void JointStatePublisherSet::publishRead(const rclcpp::Time& stamp)
{
  if (!read_) {
    return;
  }
  fill(read_->msg, joints_, stamp, [](const Joint& j) -> const std::unordered_map<std::string, double>& {
    return j.state_transmission ? j.actuator_state.current : j.joint_state.current;
  });
  read_->publisher->try_publish(read_->msg);
}

void JointStatePublisherSet::publishGoal(const rclcpp::Time& stamp)
{
  if (!goal_) {
    return;
  }
  fill(goal_->msg, joints_, stamp,
       [](const Joint& j) -> const std::unordered_map<std::string, double>& { return j.joint_state.goal; });
  goal_->publisher->try_publish(goal_->msg);
}

void JointStatePublisherSet::publishWrite(const rclcpp::Time& stamp)
{
  if (!write_) {
    return;
  }
  fill(write_->msg, joints_, stamp, [](const Joint& j) -> const std::unordered_map<std::string, double>& {
    return j.command_transmission ? j.actuator_state.goal : j.joint_state.goal;
  });
  write_->publisher->try_publish(write_->msg);
}

}  // namespace dynamixel_ros_control
