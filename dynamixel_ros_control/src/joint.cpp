#include <dynamixel_ros_control/joint.h>

namespace dynamixel_ros_control {

Joint::Joint(std::string _name, uint8_t id, uint16_t model_number)
  : name(_name),
    dynamixel(id, model_number),
    mounting_offset(0.0),
    offset(0.0),
    control_mode(POSITION)
{}

}
