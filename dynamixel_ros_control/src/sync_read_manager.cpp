#include <dynamixel_ros_control/sync_read_manager.h>

namespace dynamixel_ros_control {

SyncReadManager::SyncReadManager() {

}

void SyncReadManager::addRegister(const Dynamixel& dxl, std::string register_name, uint32_t& value)
{

}

void SyncReadManager::addRegister(const Dynamixel& dxl, std::string register_name, double& value)
{

}

bool SyncReadManager::init(DynamixelDriver& driver)
{

}

bool SyncReadManager::read()
{
//  int  dxl_comm_result = COMM_TX_FAIL;
//  bool dxl_addparam_result = false;
//  bool dxl_getdata_result = false;

//  uint32_t position;


//  dynamixel_->item_ = dynamixel_->ctrl_table_["present_position"];
//  dynamixel_tool::ControlTableItem *addr_item = dynamixel_->item_;

//  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
//  {
//    dxl_addparam_result = groupSyncReadPosition_->addParam(multi_dynamixel_[num]->id_);
//    if (dxl_addparam_result != true)
//      return false;
//  }

//  dxl_comm_result = groupSyncReadPosition_->txRxPacket();
//  if (dxl_comm_result != COMM_SUCCESS)
//    packetHandler_->printTxRxResult(dxl_comm_result);

//  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
//  {
//    dxl_getdata_result = groupSyncReadPosition_->isAvailable(multi_dynamixel_[num]->id_, addr_item->address, addr_item->data_length);

//    if (dxl_getdata_result)
//    {
//      position  = groupSyncReadPosition_->getData(multi_dynamixel_[num]->id_, addr_item->address, addr_item->data_length);
//      pos[num] = position;
//    }
//  }

//  groupSyncReadPosition_->clearParam();

//  return dxl_getdata_result;

}

}
