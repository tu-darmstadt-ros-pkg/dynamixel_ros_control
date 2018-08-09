#include <dynamixel_ros_control/dynamixel_driver.h>

#include <dynamixel_ros_control/common.h>

namespace dynamixel_ros_control {

DynamixelDriver::DynamixelDriver()
  : next_indirect_address_(0) {}

bool DynamixelDriver::loadDynamixels(const ros::NodeHandle& nh, std::vector<Joint>& joints)
{
  // Get port info
  std::string port_name;
  nh.getParam("port_info/port_name", port_name);
  setPortHandler(port_name);

  int baudrate;
  nh.getParam("port_info/baudrate", baudrate);
  setBaudRate(baudrate);

  float protocol_version;
  nh.getParam("port_info/protocol_version", protocol_version);
  setPacketHandler(protocol_version);

  // Get dxl info
  XmlRpc::XmlRpcValue dxls;
  nh.getParam("device_info", dxls);
  ROS_ASSERT(dxls.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = dxls.begin(); it != dxls.end(); ++it)
  {
    std::string joint_name = (std::string)(it->first);
    ros::NodeHandle dxl_nh(nh, "device_info/" + joint_name);

    int id_int;
    if (!loadRequiredParameter(dxl_nh, "id", id_int)) {
      return false;
    }
    uint8_t id;
    if (id_int < 256) {
      id = static_cast<uint8_t>(id_int);
    } else {
      ROS_ERROR_STREAM("ID " << id_int << " exceeds 256.");
      continue;
    }

    int model_number_config;
    dxl_nh.param("model_number", model_number_config, -1);

    // Ping dynamixel to retrieve model number
    uint16_t model_number_ping;
    if (!ping(id, model_number_ping)) {
      ROS_ERROR_STREAM("Failed to ping motor '" << joint_name << "' with id " << id_int);
      return false;
    } else {
      // Ping successful, add to list
      if (model_number_config != model_number_ping) {
        ROS_WARN_STREAM("Model number in config [" << model_number_config
                        << "] does not match servo model number [" << model_number_ping << "] for joint '"
                        << joint_name << "', ID: " << id_int);
      }

      Joint joint(joint_name, id, model_number_ping);
      dxl_nh.param("mounting_offset", joint.mounting_offset, 0.0);
      dxl_nh.param("offset", joint.offset, 0.0);
      if (!joint.dynamixel.loadControlTable(nh)) {
        return false;
      }
      joints.push_back(joint); // TODO prevent copy?

      std::stringstream ss;
      ss << "Loaded dynamixel:" << std::endl;
      ss << "-- name: " << joint.name << std::endl;
      ss << "-- id: " << static_cast<int>(joint.dynamixel.getId()) << std::endl;
      ss << "-- model number: " << joint.dynamixel.getModelNumber() << std::endl;
      ROS_DEBUG_STREAM(ss.str());
    }
  }
  return true;
}

bool DynamixelDriver::ping(uint8_t id)
{
  uint16_t model_number;
  return ping(id, model_number);
}

bool DynamixelDriver::ping(uint8_t id, uint16_t& model_number)
{
  uint8_t error = 0;
  return packet_handler_->ping(port_handler_, id, &model_number, &error) == COMM_SUCCESS;
}

dynamixel::GroupSyncWrite*DynamixelDriver::setSyncWrite(uint16_t address, uint8_t data_length)
{

}

dynamixel::GroupSyncRead* DynamixelDriver::setSyncRead(uint16_t address, uint8_t data_length)
{

}

bool DynamixelDriver::requestIndirectAddresses(unsigned int data_length, unsigned int& address_start)
{
  address_start = next_indirect_address_;
  next_indirect_address_ += data_length;
  return true;
}

bool DynamixelDriver::setPacketHandler(float protocol_version)
{
  packet_handler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);
  if (!packet_handler_) {
    ROS_ERROR_STREAM("Unsupported protocol version: " << protocol_version);
    return false;
  }
  return true;
}

bool DynamixelDriver::setPortHandler(std::string port_name)
{
  port_handler_ = dynamixel::PortHandler::getPortHandler(port_name.c_str());

  if (port_handler_->openPort())
  {
    ROS_INFO_STREAM("Succeeded to open port " << port_name);
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("Failed to open port " << port_name);
    return false;
  }
}

bool DynamixelDriver::setBaudRate(int baud_rate)
{
  if (port_handler_->setBaudRate(baud_rate))
  {
    ROS_INFO_STREAM("Succeeded to change the baudrate to " << port_handler_->getBaudRate());
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("Failed to change the baudrate to " << port_handler_->getBaudRate());
    return false;
  }
}

}
