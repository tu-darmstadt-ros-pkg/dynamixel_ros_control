#ifndef DYNAMIXEL_ROS_CONTROL_SYNC_WRITE_MANAGER_H
#define DYNAMIXEL_ROS_CONTROL_SYNC_WRITE_MANAGER_H

#include <dynamixel_ros_control/dynamixel_driver.h>
#include <dynamixel_ros_control/dynamixel.h>
#include <dynamixel_sdk/group_sync_write.h>

namespace dynamixel_ros_control {

struct WriteEntry {
  WriteEntry()
    : dxl(nullptr), register_name(""), d_value(nullptr), b_value(nullptr), offset(0.0){}
  Dynamixel* dxl;
  std::string register_name;
  double* d_value;
  bool* b_value;
  double offset;
};

class SyncWriteManager {
public:
  SyncWriteManager();
  void addRegister(Dynamixel& dxl, std::string register_name, double& value, double offset = 0.0);
  void addRegister(Dynamixel& dxl, std::string register_name, bool& value);

  bool init(DynamixelDriver& driver);
  bool write();
private:
  std::vector<WriteEntry>::iterator addEntry(Dynamixel& dxl, std::string register_name);
  std::vector<WriteEntry> write_entries_;

  unsigned int indirect_address_index_;
  uint16_t indirect_data_address_;
  uint8_t data_length_;

  dynamixel::GroupSyncWrite* sync_write_;

  unsigned int error_count_;
};

}

#endif
