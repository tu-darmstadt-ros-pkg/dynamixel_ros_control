#ifndef DYNAMIXEL_ROS_CONTROL_SYNC_READ_MANAGER_H
#define DYNAMIXEL_ROS_CONTROL_SYNC_READ_MANAGER_H

#include <dynamixel_ros_control/dynamixel_driver.h>
#include <dynamixel_ros_control/dynamixel.h>
#include <dynamixel_sdk/group_sync_read.h>

namespace dynamixel_ros_control {

struct ReadEntry {
  ReadEntry();

  Dynamixel* dxl;
  std::string register_name;
  double* value;
};

class SyncReadManager {
public:
  SyncReadManager();
  // TODO make template?

  void addDynamixel(Dynamixel* dxl);
//  void addRegister(Dynamixel& dxl, std::string register_name, uint32_t* value);
  void addRegister(std::string register_name, std::vector<std::pair<Dynamixel*, double*>> dxl_value_pairs);

  /**
   * @brief init To be called by dynamixel driver.
   * Writes the indirect addresses and sets up the sync read
   * @return
   */
  bool init(DynamixelDriver& driver);
  bool read();
private:
  dynamixel::GroupSyncRead* sync_read_;

  std::set<Dynamixel*> dynamixels_;

  std::map<std::string, std::vector<ReadEntry>> read_entries_;
  std::map<std::string, unsigned int> register_indirect_index_map_;
  std::map<std::string, unsigned int> data_length_;

  unsigned int indirect_address_index_;
  uint16_t indirect_data_start_;
  uint8_t total_data_length_;

};

}

#endif
