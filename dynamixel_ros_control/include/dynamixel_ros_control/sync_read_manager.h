#ifndef DYNAMIXEL_ROS_CONTROL__SYNC_READ_MANAGER_H
#define DYNAMIXEL_ROS_CONTROL__SYNC_READ_MANAGER_H

#include <dynamixel_ros_control/dynamixel.h>
#include <dynamixel_sdk/group_sync_read.h>

namespace dynamixel_ros_control {

class SyncReadManager {
public:
  void addRegister(const Dynamixel& dxl, std::string register_name, uint32_t& value);

  /**
   * @brief init To be called by dynamixel driver.
   * Writes the indirect addresses and sets up the sync read
   * @return
   */
  bool init();
  bool read();
private:
  dynamixel::GroupSyncRead* sync_read_;
};

}

#endif
