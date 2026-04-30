#include "dynamixel_ros_control/joint_state_publisher_set.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <limits>

#include "dynamixel_ros_control/common.hpp"

namespace dynamixel_ros_control {

namespace {

std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>
makeRTPublisher(const rclcpp::Node::SharedPtr& node, const std::string& topic,
                const std::vector<std::string>& joint_names)
{
  auto pub = node->create_publisher<sensor_msgs::msg::JointState>(topic, rclcpp::SystemDefaultsQoS());
  auto rt_pub = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(pub);
  auto& m = rt_pub->msg_;
  m.name.assign(joint_names.begin(), joint_names.end());
  m.position.assign(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  m.velocity.assign(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  m.effort.assign(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  return rt_pub;
}

template <typename Pick>
inline void fill(sensor_msgs::msg::JointState& msg, const std::vector<std::string>& joint_names,
                 const std::unordered_map<std::string, Joint>& joints, const rclcpp::Time& stamp, Pick pick)
{
  msg.header.stamp = stamp;
  const size_t n = joint_names.size();
  for (size_t i = 0; i < n; ++i) {
    const auto& m = pick(joints.at(joint_names[i]));
    auto p = m.find(hardware_interface::HW_IF_POSITION);
    msg.position[i] = (p != m.end()) ? p->second : std::numeric_limits<double>::quiet_NaN();
    auto v = m.find(hardware_interface::HW_IF_VELOCITY);
    msg.velocity[i] = (v != m.end()) ? v->second : std::numeric_limits<double>::quiet_NaN();
    auto e = m.find(hardware_interface::HW_IF_EFFORT);
    msg.effort[i] = (e != m.end()) ? e->second : std::numeric_limits<double>::quiet_NaN();
  }
}

}  // namespace

void JointStatePublisherSet::init(const rclcpp::Node::SharedPtr& node,
                                  const std::unordered_map<std::string, std::string>& hardware_parameters,
                                  const std::vector<std::string>& joint_names)
{
  bool publish_goal = false;
  bool publish_read = false;
  bool publish_write = false;
  getParameter(hardware_parameters, "publish_goal_joint_states", publish_goal, false);
  getParameter(hardware_parameters, "publish_read_joint_states", publish_read, false);
  getParameter(hardware_parameters, "publish_write_joint_states", publish_write, false);

  if (publish_goal) {
    goal_ = makeRTPublisher(node, "~/goal_joint_states", joint_names);
  }
  if (publish_read) {
    read_ = makeRTPublisher(node, "~/read_joint_states", joint_names);
  }
  if (publish_write) {
    write_ = makeRTPublisher(node, "~/write_joint_states", joint_names);
  }
}

void JointStatePublisherSet::publishRead(const rclcpp::Time& stamp,
                                         const std::unordered_map<std::string, Joint>& joints,
                                         const std::vector<std::string>& joint_names)
{
  if (read_ && read_->trylock()) {
    fill(read_->msg_, joint_names, joints, stamp, [](const Joint& j) -> const std::unordered_map<std::string, double>& {
      return j.state_transmission ? j.actuator_state.current : j.joint_state.current;
    });
    read_->unlockAndPublish();
  }
}

void JointStatePublisherSet::publishGoalAndWrite(const rclcpp::Time& stamp,
                                                 const std::unordered_map<std::string, Joint>& joints,
                                                 const std::vector<std::string>& joint_names)
{
  if (goal_ && goal_->trylock()) {
    fill(goal_->msg_, joint_names, joints, stamp,
         [](const Joint& j) -> const std::unordered_map<std::string, double>& { return j.joint_state.goal; });
    goal_->unlockAndPublish();
  }
  if (write_ && write_->trylock()) {
    fill(write_->msg_, joint_names, joints, stamp, [](const Joint& j) -> const std::unordered_map<std::string, double>& {
      return j.command_transmission ? j.actuator_state.goal : j.joint_state.goal;
    });
    write_->unlockAndPublish();
  }
}

}  // namespace dynamixel_ros_control
