#ifndef DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_DRIVER_H
#define DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_DRIVER_H

#include <rclcpp/rclcpp.hpp>

#include <dynamixel_sdk/port_handler.h>
#include <dynamixel_sdk/packet_handler.h>
#include <dynamixel_sdk/group_sync_read.h>
#include <dynamixel_sdk/group_sync_write.h>

#include <dynamixel_ros_control/control_table.hpp>

namespace dynamixel_ros_control {

std::string interfaceNameToRegister(const std::string& interface_name);

class DynamixelDriver
{
public:
  DynamixelDriver();

  bool init(const std::string& port_name, int baud_rate);

  bool connect();

  ControlTable* loadControlTable(uint16_t model_number);

  [[nodiscard]] bool ping(uint8_t id) const;
  bool ping(uint8_t id, uint16_t& model_number) const;
  bool reboot(uint8_t id) const;
  [[nodiscard]] std::vector<std::pair<uint8_t /*id*/, uint16_t /*model_number*/>> scan() const;

  bool writeRegister(uint8_t id, uint16_t address, uint8_t data_length, int32_t value) const;
  bool readRegister(uint8_t id, uint16_t address, uint8_t data_length, int32_t& value_out) const;

  [[nodiscard]] dynamixel::GroupSyncWrite* setSyncWrite(uint16_t address, uint8_t data_length) const;
  [[nodiscard]] dynamixel::GroupSyncRead* setSyncRead(uint16_t address, uint8_t data_length) const;

  bool requestIndirectAddresses(unsigned int data_length, unsigned int& address_start);

  [[nodiscard]] std::string communicationErrorToString(int comm_result) const;
  [[nodiscard]] std::string packetErrorToString(uint8_t error) const;
private:
  bool loadSeriesMapping();
  ControlTable* readControlTable(std::string series);
  bool setPacketHandler();
  bool setPortHandler(const std::string& port_name);
  bool connectPort();
  bool setBaudRate(int baud_rate) const;

  dynamixel::PacketHandler* packet_handler_;
  dynamixel::PortHandler* port_handler_;

  unsigned int next_indirect_address_;

  std::string package_path_;
  std::map<uint16_t, std::string> model_number_to_series_;
  std::map<std::string, ControlTable> series_to_control_table_;

  // Parameters
  std::string port_name_;
  int baud_rate_;
};

}  // namespace dynamixel_ros_control

#endif
