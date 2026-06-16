#ifndef DYNAMIXEL_ROS_CONTROL_SYNC_WRITE_MANAGER_H
#define DYNAMIXEL_ROS_CONTROL_SYNC_WRITE_MANAGER_H

#include <dynamixel_ros_control/dynamixel_driver.hpp>
#include <dynamixel_ros_control/dynamixel.hpp>
#include <dynamixel_ros_control/sdk_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <set>

namespace dynamixel_ros_control {

struct IndirectWriteDebugEntry
{
  std::string register_name;
  unsigned int indirect_index{0};
  uint16_t indirect_data_address{0};
  uint8_t data_length{0};
};

struct WriteEntry
{
  std::string register_name;
  unsigned int indirect_index{0};
  uint8_t data_length;
  uint16_t indirect_data_address;
  double* d_value{nullptr};
  bool* b_value{nullptr};
  double offset{0.0};
};

class SyncWriteManager
{
public:
  SyncWriteManager() = default;
  void addRegister(Dynamixel& dxl, const std::string& register_name, double& value, double offset = 0.0);
  void addRegister(Dynamixel& dxl, const std::string& register_name, bool& value);

  [[nodiscard]] bool init(DynamixelDriver& driver);

  /**
   * @brief Re-write indirect address mappings for the given motors without re-allocating
   * driver-side indirect-address slots. Use after a Dynamixel reboot, which wipes RAM
   * (including the indirect address pointer registers).
   * @param motors Subset of dynamixels whose indirect mappings should be rewritten.
   *               Motors not in this set are skipped.
   */
  [[nodiscard]] bool rewriteIndirectAddresses(const std::set<Dynamixel*>& motors);

  [[nodiscard]] bool release() const;
  [[nodiscard]] bool write();

  [[nodiscard]] std::vector<IndirectWriteDebugEntry>
  getIndirectDebugEntries(const Dynamixel& dxl, const std::vector<std::string>& register_names) const;

  [[nodiscard]] bool isOk() const;
  [[nodiscard]] unsigned int getErrorCount() const
  {
    return subsequent_error_count_;
  }
  void setErrorThreshold(unsigned int threshold);

private:
  std::optional<std::reference_wrapper<WriteEntry>> addEntry(Dynamixel& dxl, const std::string& register_name);
  std::unordered_map<Dynamixel*, std::vector<WriteEntry>> write_entries_;

  unsigned int indirect_address_index_{0};
  uint8_t total_data_length_{0};

  DynamixelDriver* driver_{nullptr};
  std::shared_ptr<GroupSyncWrite> sync_write_{nullptr};

  unsigned int subsequent_error_count_{0};
  unsigned int error_threshold_{25};
};

}  // namespace dynamixel_ros_control

#endif
