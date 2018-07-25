#ifndef DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_DRIVER_H
#define DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_DRIVER_H

#include <ros/ros.h>

#include <dynamixel_ros_control/sync_read_manager.h>
#include <dynamixel_ros_control/sync_write_manager.h>
#include <dynamixel_ros_control/joint.h>
#include <dynamixel_sdk/port_handler.h>
#include <dynamixel_sdk/packet_handler.h>
#include <dynamixel_sdk/group_sync_read.h>
#include <dynamixel_sdk/group_sync_write.h>

namespace dynamixel_ros_control {

class DynamixelDriver {
public:
  DynamixelDriver();
  bool loadDynamixels(const ros::NodeHandle& nh, std::vector<Joint>& joints);
  void addDynamixel(const Dynamixel& dxl);

  bool ping(uint8_t id);
  bool ping(uint8_t id, uint16_t& model_number);


  void registerSyncReadManager(SyncReadManager& sync_read);
  void registerSyncWriteManager(SyncReadManager& sync_write);

  bool writeMultiRegister(std::string register_name, uint32_t value);
  bool writeRegister(const Dynamixel& dxl, std::string register_name, uint32_t value);
  bool writeRegister(uint8_t id, uint16_t address, uint8_t data_length, uint32_t value);

  bool readRegister(const Dynamixel& dxl, std::string register_name, uint32_t& value_out);
  bool readRegister(uint8_t id, uint16_t address, uint8_t data_length, uint32_t& value_out);

  dynamixel::GroupSyncWrite* setSyncWrite(uint16_t address, uint8_t data_length);
  dynamixel::GroupSyncWrite* setSyncWrite(std::string register_name);

  dynamixel::GroupSyncRead* setSyncRead(uint16_t address, uint8_t data_length);
  dynamixel::GroupSyncRead* setSyncRead(std::string register_name);

  const std::vector<Dynamixel>& getDynamixels();
private:
  bool setPacketHandler(float protocol_version);
  bool setPortHandler(std::string port_name);
  bool setBaudRate(uint32_t baud_rate);

  dynamixel::PacketHandler* packet_handler_;
  dynamixel::PortHandler* port_handler_;

  unsigned int next_indirect_address_;
  std::vector<SyncReadManager> sync_read_;
  std::vector<SyncWriteManager> sync_write_;

};

}

#endif
