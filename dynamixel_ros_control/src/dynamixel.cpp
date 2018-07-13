#include <dynamixel_ros_control/dynamixel.h>

namespace dynamixel_ros_control {

Dynamixel::Dynamixel(uint8_t id, uint16_t model_number)
{
  // from model_number look up series
  // load control table
  std::string series = getSeries();
  loadControlTable(series);
}

Dynamixel::Dynamixel(uint8_t id, std::string model_name)
{
  // convert model_name to model_number
}

double Dynamixel::dxlValuetoPosition(int32_t dxl_value)
{

}

int32_t Dynamixel::positionToDxlValue(double rad)
{

}

double Dynamixel::dxlValueToVelocity(int32_t dxl_value)
{

}

int32_t Dynamixel::velocityToDxlValue(double velocity)
{

}

double Dynamixel::dxlValueToTorque(int32_t dxl_value)
{

}

int32_t Dynamixel::torqueToDxlValue(double torque)
{

}

const ControlTableItem& Dynamixel::getItem(std::string& name)
{
  try {
    return control_table_.at(name);
  } catch (const std::out_of_range& e) {
    ROS_ERROR_STREAM("Could not find register '" << name << "'.");
    throw;
  }

}

std::string Dynamixel::getSeries()
{

}

void Dynamixel::loadControlTable(std::string series)
{

}

uint16_t Dynamixel::getModelNumber() const
{
  return model_number_;
}

uint8_t Dynamixel::getId() const
{
  return id_;
}

}
