#ifndef DYNAMIXEL_ROS_CONTROL_SYNC_READ_MANAGER_H
#define DYNAMIXEL_ROS_CONTROL_SYNC_READ_MANAGER_H

#include <dynamixel_ros_control/dynamixel_driver.hpp>
#include <dynamixel_ros_control/dynamixel.hpp>
#include <dynamixel_ros_control/dynamixel.hpp>
#include <dynamixel_ros_control/sdk_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <set>

namespace dynamixel_ros_control {

struct DxlValue
{
  explicit DxlValue(double* _dvalue)
      : dvalue(_dvalue), bvalue(nullptr), ivalue(nullptr)
  {}
  explicit DxlValue(bool* _bvalue)
      : dvalue(nullptr), bvalue(_bvalue), ivalue(nullptr)
  {}
  explicit DxlValue(int32_t* _ivalue)
      : dvalue(nullptr), bvalue(nullptr), ivalue(_ivalue)
  {}
  double* dvalue;
  bool* bvalue;
  int32_t* ivalue;
};

using DxlValueMappingList = std::vector<std::pair<Dynamixel*, DxlValue>>;

struct ReadEntry
{
  std::string register_name;

  unsigned int indirect_index;
  uint16_t indirect_data_address;
  uint8_t data_length;

  DxlValueMappingList dxl_value_pairs;
  std::vector<double> offsets;
};

class SyncReadManager
{
public:
  SyncReadManager() = default;
  // TODO make template?

  /**
   * @brief Adds a dynamixel that is read synchronously
   * @param dxl Pointer to dynamixel
   */
  void addDynamixel(Dynamixel* dxl);
  [[nodiscard]] bool addRegister(std::string register_name, const DxlValueMappingList& dxl_value_pairs,
                                 std::vector<double> offsets = {});

  /**
   * @brief init To be called by dynamixel driver.
   * Writes the indirect addresses and sets up the sync read
   * @return
   */
  [[nodiscard]] bool init(DynamixelDriver& driver);

  /**
   * @brief Re-write indirect address mappings for the given motors without re-allocating
   * driver-side indirect-address slots. Use after a Dynamixel reboot, which wipes RAM
   * (including the indirect address pointer registers).
   * @param motors Subset of dynamixels whose indirect mappings should be rewritten.
   *               Motors not in this set are skipped.
   */
  [[nodiscard]] bool rewriteIndirectAddresses(const std::set<Dynamixel*>& motors);

  [[nodiscard]] bool read();
  [[nodiscard]] bool read(rclcpp::Time& packet_receive_time);

  [[nodiscard]] bool isOk() const;
  [[nodiscard]] unsigned int getErrorCount() const
  {
    return subsequent_error_count_;
  }
  void setErrorThreshold(unsigned int threshold);

private:
  std::shared_ptr<GroupSyncRead> sync_read_{nullptr};

  DynamixelDriver* driver_{nullptr};
  std::set<Dynamixel*> dynamixels_;

  std::map<std::string, ReadEntry> read_entries_;

  unsigned int indirect_address_index_{0};
  uint16_t indirect_data_address_{0};
  uint8_t total_data_length_{0};

  unsigned int subsequent_error_count_{0};
  unsigned int error_threshold_{25};
};

}  // namespace dynamixel_ros_control

#endif
