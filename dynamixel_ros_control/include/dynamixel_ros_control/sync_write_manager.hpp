#ifndef DYNAMIXEL_ROS_CONTROL_SYNC_WRITE_MANAGER_H
#define DYNAMIXEL_ROS_CONTROL_SYNC_WRITE_MANAGER_H

#include <dynamixel_ros_control/dynamixel_driver.hpp>
#include <dynamixel_ros_control/dynamixel.hpp>
#include <dynamixel_sdk/group_sync_write.h>
#include <rclcpp/rclcpp.hpp>

namespace dynamixel_ros_control {

struct WriteEntry {
  Dynamixel* dxl{nullptr};
  std::string register_name;
  double* d_value{nullptr};
  bool* b_value{nullptr};
  double offset{0.0};
};

class SyncWriteManager {
public:
  SyncWriteManager() = default;
  void addRegister(Dynamixel& dxl, const std::string& register_name, double& value, double offset = 0.0);
  void addRegister(Dynamixel& dxl, const std::string& register_name, bool& value);

  bool init(DynamixelDriver& driver);
  bool write();

  [[nodiscard]] bool isOk() const;
  void setErrorThreshold(unsigned int threshold);
private:
  std::vector<WriteEntry>::iterator addEntry(Dynamixel& dxl, const std::string& register_name);
  std::vector<WriteEntry> write_entries_;

  unsigned int indirect_address_index_{0};
  uint16_t indirect_data_address_{0};
  uint8_t data_length_{0};

  DynamixelDriver* driver_{nullptr};
  dynamixel::GroupSyncWrite* sync_write_{nullptr};

  unsigned int subsequent_error_count_{0};
  unsigned int error_threshold_{25};
};

}

#endif
