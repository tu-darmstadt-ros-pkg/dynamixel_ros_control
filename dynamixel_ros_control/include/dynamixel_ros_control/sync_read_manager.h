#ifndef DYNAMIXEL_ROS_CONTROL_SYNC_READ_MANAGER_H
#define DYNAMIXEL_ROS_CONTROL_SYNC_READ_MANAGER_H

#include <dynamixel_ros_control/dynamixel_driver.h>
#include <dynamixel_ros_control/dynamixel.h>
#include <dynamixel_sdk/group_sync_read.h>

namespace dynamixel_ros_control {

struct DxlValue {
  DxlValue(double* _dvalue)  : dvalue(_dvalue) {}
  DxlValue(bool* _bvalue)    : bvalue(_bvalue) {}
  DxlValue(int32_t* _ivalue) : ivalue(_ivalue) {}
  double *dvalue;
  bool *bvalue;
  int32_t *ivalue;
};

using DxlValueMappingList = std::vector<std::pair<Dynamixel*, DxlValue>>;

struct ReadEntry {
  std::string register_name;

  unsigned int indirect_index;
  uint16_t indirect_data_address;
  uint8_t data_length;

  DxlValueMappingList dxl_value_pairs;
  std::vector<double> offsets;
};

class SyncReadManager {
public:
  SyncReadManager();
  // TODO make template?

  void addDynamixel(Dynamixel* dxl);
//  void addRegister(Dynamixel& dxl, std::string register_name, uint32_t* value);
  bool addRegister(std::string register_name, const DxlValueMappingList& dxl_value_pairs, std::vector<double> offsets = {});

  /**
   * @brief init To be called by dynamixel driver.
   * Writes the indirect addresses and sets up the sync read
   * @return
   */
  bool init(DynamixelDriver& driver);
  bool read();
  bool read(ros::Time& packet_receive_time);
private:
  dynamixel::GroupSyncRead* sync_read_;

  DynamixelDriver* driver_;
  std::set<Dynamixel*> dynamixels_;

  std::map<std::string, ReadEntry> read_entries_;

  unsigned int indirect_address_index_;
  uint16_t indirect_data_address_;
  uint8_t total_data_length_;

};

}

#endif
