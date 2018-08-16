#ifndef DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_DRIVER_H
#define DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_DRIVER_H

#include <ros/ros.h>

#include <dynamixel_sdk/port_handler.h>
#include <dynamixel_sdk/packet_handler.h>
#include <dynamixel_sdk/group_sync_read.h>
#include <dynamixel_sdk/group_sync_write.h>

namespace dynamixel_ros_control {

class DynamixelDriver {
public:
  DynamixelDriver();

  bool init(const ros::NodeHandle& nh);

  bool ping(uint8_t id);
  bool ping(uint8_t id, uint16_t& model_number);
  std::vector<std::pair<uint8_t /*id*/, uint16_t /*model_number*/>> scan();

  bool writeMultiRegister(std::string register_name, uint32_t value);
  bool writeRegister(uint8_t id, uint16_t address, uint8_t data_length, int32_t value);
  bool readRegister(uint8_t id, uint16_t address, uint8_t data_length, int32_t& value_out);

  dynamixel::GroupSyncWrite* setSyncWrite(uint16_t address, uint8_t data_length);

  dynamixel::GroupSyncRead* setSyncRead(uint16_t address, uint8_t data_length);

  bool requestIndirectAddresses(unsigned int data_length, unsigned int& address_start);
private:
  bool setPacketHandler(float protocol_version);
  bool setPortHandler(std::string port_name);
  bool setBaudRate(int baud_rate);

  dynamixel::PacketHandler* packet_handler_;
  dynamixel::PortHandler* port_handler_;

  unsigned int next_indirect_address_;
};

}

#endif
