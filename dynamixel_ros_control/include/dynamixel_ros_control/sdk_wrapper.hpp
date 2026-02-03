#ifndef DYNAMIXEL_ROS_CONTROL_SDK_WRAPPER_HPP
#define DYNAMIXEL_ROS_CONTROL_SDK_WRAPPER_HPP

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <rclcpp/rclcpp.hpp>

namespace dynamixel_ros_control {

// Forward declarations
class PortHandler;
class PacketHandler;
class GroupSyncRead;
class GroupSyncWrite;

/**
 * @brief Abstract Base Class for PortHandler
 */
class PortHandler
{
public:
  virtual ~PortHandler() = default;
  virtual bool openPort() = 0;
  virtual void closePort() = 0;
  virtual void clearPort() = 0;
  virtual void setPortName(const char* port_name) = 0;
  virtual std::string getPortName() = 0;
  virtual bool setBaudRate(const int baudrate) = 0;
  virtual int getBaudRate() = 0;

  // Method to get the raw SDK pointer (only for real implementation usage if needed)
  virtual dynamixel::PortHandler* getRaw()
  {
    return nullptr;
  }
};

/**
 * @brief Real implementation of PortHandler using dynamixel_sdk
 */
class RealPortHandler : public PortHandler
{
public:
  explicit RealPortHandler(const char* port_name)
  {
    handler_ = dynamixel::PortHandler::getPortHandler(port_name);
  }

  explicit RealPortHandler(dynamixel::PortHandler* handler)
      : handler_(handler)
  {}

  ~RealPortHandler() override
  {
    if (handler_)
      delete handler_;
  }

  bool openPort() override
  {
    return handler_->openPort();
  }
  void closePort() override
  {
    handler_->closePort();
  }
  void clearPort() override
  {
    handler_->clearPort();
  }
  void setPortName(const char* port_name) override
  {
    handler_->setPortName(port_name);
  }
  std::string getPortName() override
  {
    return handler_->getPortName();
  }
  bool setBaudRate(const int baudrate) override
  {
    return handler_->setBaudRate(baudrate);
  }
  int getBaudRate() override
  {
    return handler_->getBaudRate();
  }

  dynamixel::PortHandler* getRaw() override
  {
    return handler_;
  }

private:
  dynamixel::PortHandler* handler_;
};

/**
 * @brief Abstract Base Class for PacketHandler
 */
class PacketHandler
{
public:
  virtual ~PacketHandler() = default;
  virtual int ping(std::shared_ptr<PortHandler> port, uint8_t id, uint16_t* model_number, uint8_t* error = nullptr) = 0;
  virtual int reboot(std::shared_ptr<PortHandler> port, uint8_t id, uint8_t* error = nullptr) = 0;

  virtual int read1ByteTxRx(std::shared_ptr<PortHandler> port, uint8_t id, uint16_t address, uint8_t* data,
                            uint8_t* error = nullptr) = 0;
  virtual int read2ByteTxRx(std::shared_ptr<PortHandler> port, uint8_t id, uint16_t address, uint16_t* data,
                            uint8_t* error = nullptr) = 0;
  virtual int read4ByteTxRx(std::shared_ptr<PortHandler> port, uint8_t id, uint16_t address, uint32_t* data,
                            uint8_t* error = nullptr) = 0;

  virtual int write1ByteTxRx(std::shared_ptr<PortHandler> port, uint8_t id, uint16_t address, uint8_t data,
                             uint8_t* error = nullptr) = 0;
  virtual int write2ByteTxRx(std::shared_ptr<PortHandler> port, uint8_t id, uint16_t address, uint16_t data,
                             uint8_t* error = nullptr) = 0;
  virtual int write4ByteTxRx(std::shared_ptr<PortHandler> port, uint8_t id, uint16_t address, uint32_t data,
                             uint8_t* error = nullptr) = 0;

  virtual const char* getTxRxResult(int result) = 0;
  virtual const char* getRxPacketError(uint8_t error) = 0;

  virtual dynamixel::PacketHandler* getRaw()
  {
    return nullptr;
  }
};

/**
 * @brief Real implementation of PacketHandler using dynamixel_sdk
 */
class RealPacketHandler : public PacketHandler
{
public:
  explicit RealPacketHandler(float protocol_version)
  {
    handler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);
  }

  // SDK's getPacketHandler returns a static instance, so we must not delete it
  ~RealPacketHandler() override = default;

  int ping(std::shared_ptr<PortHandler> port, uint8_t id, uint16_t* model_number, uint8_t* error) override
  {
    return handler_->ping(port->getRaw(), id, model_number, error);
  }
  int reboot(std::shared_ptr<PortHandler> port, uint8_t id, uint8_t* error) override
  {
    return handler_->reboot(port->getRaw(), id, error);
  }

  int read1ByteTxRx(std::shared_ptr<PortHandler> port, uint8_t id, uint16_t address, uint8_t* data,
                    uint8_t* error) override
  {
    return handler_->read1ByteTxRx(port->getRaw(), id, address, data, error);
  }
  int read2ByteTxRx(std::shared_ptr<PortHandler> port, uint8_t id, uint16_t address, uint16_t* data,
                    uint8_t* error) override
  {
    return handler_->read2ByteTxRx(port->getRaw(), id, address, data, error);
  }
  int read4ByteTxRx(std::shared_ptr<PortHandler> port, uint8_t id, uint16_t address, uint32_t* data,
                    uint8_t* error) override
  {
    return handler_->read4ByteTxRx(port->getRaw(), id, address, data, error);
  }

  int write1ByteTxRx(std::shared_ptr<PortHandler> port, uint8_t id, uint16_t address, uint8_t data,
                     uint8_t* error) override
  {
    return handler_->write1ByteTxRx(port->getRaw(), id, address, data, error);
  }
  int write2ByteTxRx(std::shared_ptr<PortHandler> port, uint8_t id, uint16_t address, uint16_t data,
                     uint8_t* error) override
  {
    return handler_->write2ByteTxRx(port->getRaw(), id, address, data, error);
  }
  int write4ByteTxRx(std::shared_ptr<PortHandler> port, uint8_t id, uint16_t address, uint32_t data,
                     uint8_t* error) override
  {
    return handler_->write4ByteTxRx(port->getRaw(), id, address, data, error);
  }

  const char* getTxRxResult(int result) override
  {
    return handler_->getTxRxResult(result);
  }
  const char* getRxPacketError(uint8_t error) override
  {
    return handler_->getRxPacketError(error);
  }

  dynamixel::PacketHandler* getRaw() override
  {
    return handler_;
  }

private:
  dynamixel::PacketHandler* handler_;
};

/**
 * @brief Abstract Base Class for GroupSyncRead
 */
class GroupSyncRead
{
public:
  virtual ~GroupSyncRead() = default;
  virtual bool addParam(uint8_t id) = 0;
  virtual void removeParam(uint8_t id) = 0;
  virtual void clearParam() = 0;
  virtual int txRxPacket() = 0;
  virtual bool isAvailable(uint8_t id, uint16_t address, uint16_t data_length) = 0;
  virtual uint32_t getData(uint8_t id, uint16_t address, uint16_t data_length) = 0;
};

/**
 * @brief Real implementation of GroupSyncRead
 */
class RealGroupSyncRead : public GroupSyncRead
{
public:
  RealGroupSyncRead(std::shared_ptr<PortHandler> port, std::shared_ptr<PacketHandler> ph, uint16_t start_address,
                    uint16_t data_length)
  {
    group_sync_read_ = new dynamixel::GroupSyncRead(port->getRaw(), ph->getRaw(), start_address, data_length);
  }
  ~RealGroupSyncRead() override
  {
    delete group_sync_read_;
  }

  bool addParam(uint8_t id) override
  {
    return group_sync_read_->addParam(id);
  }
  void removeParam(uint8_t id) override
  {
    group_sync_read_->removeParam(id);
  }
  void clearParam() override
  {
    group_sync_read_->clearParam();
  }
  int txRxPacket() override
  {
    return group_sync_read_->txRxPacket();
  }
  bool isAvailable(uint8_t id, uint16_t address, uint16_t data_length) override
  {
    return group_sync_read_->isAvailable(id, address, data_length);
  }
  uint32_t getData(uint8_t id, uint16_t address, uint16_t data_length) override
  {
    return group_sync_read_->getData(id, address, data_length);
  }

private:
  dynamixel::GroupSyncRead* group_sync_read_;
};

/**
 * @brief Abstract Base Class for GroupSyncWrite
 */
class GroupSyncWrite
{
public:
  virtual ~GroupSyncWrite() = default;
  virtual bool addParam(uint8_t id, uint8_t* data) = 0;
  virtual void removeParam(uint8_t id) = 0;
  virtual bool changeParam(uint8_t id, uint8_t* data) = 0;
  virtual void clearParam() = 0;
  virtual int txPacket() = 0;
};

/**
 * @brief Real implementation of GroupSyncWrite
 */
class RealGroupSyncWrite : public GroupSyncWrite
{
public:
  RealGroupSyncWrite(std::shared_ptr<PortHandler> port, std::shared_ptr<PacketHandler> ph, uint16_t start_address,
                     uint16_t data_length)
  {
    group_sync_write_ = new dynamixel::GroupSyncWrite(port->getRaw(), ph->getRaw(), start_address, data_length);
  }
  ~RealGroupSyncWrite() override
  {
    delete group_sync_write_;
  }

  bool addParam(uint8_t id, uint8_t* data) override
  {
    return group_sync_write_->addParam(id, data);
  }
  void removeParam(uint8_t id) override
  {
    group_sync_write_->removeParam(id);
  }
  bool changeParam(uint8_t id, uint8_t* data) override
  {
    return group_sync_write_->changeParam(id, data);
  }
  void clearParam() override
  {
    group_sync_write_->clearParam();
  }
  int txPacket() override
  {
    return group_sync_write_->txPacket();
  }

private:
  dynamixel::GroupSyncWrite* group_sync_write_;
};

}  // namespace dynamixel_ros_control

#endif  // DYNAMIXEL_ROS_CONTROL_SDK_WRAPPER_HPP
