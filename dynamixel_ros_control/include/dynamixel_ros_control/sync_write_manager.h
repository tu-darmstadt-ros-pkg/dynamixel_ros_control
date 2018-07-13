#ifndef DYNAMIXEL_ROS_CONTROL__SYNC_WRITE_MANAGER_H
#define DYNAMIXEL_ROS_CONTROL__SYNC_WRITE_MANAGER_H

#include <dynamixel_ros_control/dynamixel.h>
#include <dynamixel_sdk/group_sync_write.h>

namespace dynamixel_ros_control {

class SyncWriteManager {
public:
  void addRegister(const Dynamixel& dxl, std::string register_name, uint32_t& value);

  bool init(); // to be called by dynamixel driver
  bool write();
private:
  dynamixel::GroupSyncWrite* sync_write_;
};

}

#endif
