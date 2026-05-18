#ifndef DYNAMIXEL_ROS_CONTROL_JOINT_STATE_PUBLISHER_SET_H
#define DYNAMIXEL_ROS_CONTROL_JOINT_STATE_PUBLISHER_SET_H

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "joint.hpp"

namespace dynamixel_ros_control {

// Owns up to three realtime sensor_msgs/JointState publishers exposed by the
// Dynamixel hardware interface:
//   - ~/goal_joint_states  (joint-space goal,    pre-transmission)
//   - ~/read_joint_states  (actuator-space state, before state_transmission applies)
//   - ~/write_joint_states (actuator-space goal,  after command_transmission applies)
//
// Each publisher is independently enabled by a hardware_parameter flag in URDF
// (publish_goal_joint_states / publish_read_joint_states / publish_write_joint_states).
// Disabled publishers stay null and are skipped via a single nullptr check at the
// call site, so the realtime cost when off is one branch.
class JointStatePublisherSet
{
public:
  // Read flags from `hardware_parameters` and create the enabled publishers on `node`.
  // Pre-sizes message vectors with `joint_names` so realtime publishes do no allocations.
  void init(const rclcpp::Node::SharedPtr& node,
            const std::unordered_map<std::string, std::string>& hardware_parameters,
            const std::vector<std::string>& joint_names);

  // Publish actuator-space read state (current values). Use the time of the read cycle
  // for the message stamp. Picks `actuator_state.current` when a state transmission
  // exists, else `joint_state.current` (mirroring Joint::getActuatorState()).
  void publishRead(const rclcpp::Time& stamp, const std::unordered_map<std::string, Joint>& joints,
                   const std::vector<std::string>& joint_names);

  // Publish the joint-space goal (the controller's commanded values, pre-transmission).
  // Safe to call even when the actual bus write failed for the cycle: this reflects
  // controller intent, not what reached the motors.
  void publishGoal(const rclcpp::Time& stamp, const std::unordered_map<std::string, Joint>& joints,
                   const std::vector<std::string>& joint_names);

  // Publish the actuator-space goal (post-transmission). Picks `actuator_state.goal` when
  // a command transmission exists, else `joint_state.goal`. Should only be called on a
  // successful bus write so the published values match what actually reached the motors.
  void publishWrite(const rclcpp::Time& stamp, const std::unordered_map<std::string, Joint>& joints,
                    const std::vector<std::string>& joint_names);

private:
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> goal_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> read_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> write_;
};

}  // namespace dynamixel_ros_control

#endif
