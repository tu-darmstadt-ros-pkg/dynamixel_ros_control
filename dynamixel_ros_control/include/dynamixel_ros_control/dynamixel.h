#ifndef DYNAMIXEL_ROS_CONTROL__DYNAMIXEL_H
#define DYNAMIXEL_ROS_CONTROL__DYNAMIXEL_H

#include <dynamixel_ros_control/control_table_item.h>
#include <map>

namespace dynamixel_ros_control {

struct State
{
  State() : position(0), velocity(0), effort(0) {}
  double position;
  double velocity;
  double effort;
};

class Dynamixel {
public:
  Dynamixel(uint8_t id, uint16_t model_number);
  Dynamixel(uint8_t id, std::string model_name);

  double tickToRad(int32_t tick);
  int32_t radToTick(double rad);

  double tickToVelocity(int32_t tick);
  int32_t velocityToTick(double velocity);

  double tickToTorque(int32_t tick);
  int32_t torqueToTick(double torque);
private:
  uint8_t id_;
  uint16_t model_number_;
  std::string model_name_;

  std::map<std::string, ControlTableItem> ctrl_table_;

  State current_state_;
  State goal_state_;
};

}

#endif
