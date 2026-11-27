#ifndef DYNAMIXEL_ROS_CONTROL_SYNC_READ_MANAGER_H
#define DYNAMIXEL_ROS_CONTROL_SYNC_READ_MANAGER_H

#include <dynamixel_ros_control/dynamixel_driver.hpp>
#include <dynamixel_ros_control/dynamixel.hpp>
#include <dynamixel_ros_control/sdk_wrapper.hpp>
#include <dynamixel_ros_control/common.hpp>
#include <set>

namespace dynamixel_ros_control {

struct DxlValue
{
  /// @param _ratio replaces the register's own unit ratio unless 0
  explicit DxlValue(double* _dvalue, const double _ratio = 0.0)
      : dvalue(_dvalue), bvalue(nullptr), ivalue(nullptr), ratio(_ratio)
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
  double ratio{0.0};
};

using DxlValueMappingList = std::vector<std::pair<Dynamixel*, DxlValue>>;

struct ReadEntry
{
  std::string register_name;

  unsigned int indirect_index{0};
  uint16_t indirect_data_address{0};
  uint8_t data_length{0};

  DxlValueMappingList dxl_value_pairs;
  /// Register actually mapped per motor: register_name, or the padding register where it is absent.
  std::vector<std::string> motor_registers;

  /// Per-motor decode, resolved once in addRegister() and indexed like dxl_value_pairs. The cyclic
  /// read runs at the control rate over every motor and register, so it must not reach back into
  /// the control table. That is a std::map lookup keyed by register name, per motor, per cycle.
  /// A padding slot carries another motor's register in these bytes, and its value is dropped.
  std::vector<bool> is_padding;
  /// Factor turning a raw register value into its physical unit.
  std::vector<double> ratios;
  /// Whether the raw value is two's complement, taken from the control table item in addRegister()
  /// and checked to agree across every motor. One flag rather than one per motor: padding slots
  /// carry a different register and are discarded before decoding, so they never consult it.
  bool is_signed{false};
};

class SyncReadManager
{
public:
  SyncReadManager() = default;

  void addDynamixel(Dynamixel* dxl);

  /**
   * @brief Add a register to the group, read from every motor added so far.
   *
   * One sync read fetches one address range from every motor, so all must map the same bytes.
   * @param padding_register Same-length register that fills the slot on a motor without
   * @p register_name, its value discarded. Empty means every motor must have the register.
   */
  [[nodiscard]] bool addRegister(std::string register_name, const DxlValueMappingList& dxl_value_pairs,
                                 const std::string& padding_register = "");

  /// @brief Write the indirect addresses and build the group
  [[nodiscard]] bool init(DynamixelDriver& driver);

  /// @brief Rewrite the indirect address pointers of @p motors into the slots they already have,
  /// after a reboot wiped them
  [[nodiscard]] bool rewriteIndirectAddresses(const std::set<Dynamixel*>& motors);

  /// @brief Read every registered motor. Silent, so the control loop decides what to report.
  [[nodiscard]] bool read();
  /// @brief Why the last read() failed.
  [[nodiscard]] std::string lastError() const;

  /// @brief Read a subset of the motors in a temporary group, leaving the cyclic group and its error
  /// count alone. Motors not registered here are ignored.
  /// @return whether every requested motor answered
  [[nodiscard]] bool readSelected(const std::set<Dynamixel*>& motors);

  /// @brief Read @p motors in one transaction, and one at a time if that fails, which is the only
  /// way to learn who answered: a group read loses every motor after the first silent one.
  /// @return The motors whose data was read
  [[nodiscard]] std::set<Dynamixel*> readWhoeverAnswers(const std::set<Dynamixel*>& motors);

  /// @brief Whether a read can reach the bus at all
  [[nodiscard]] bool isInitialized() const
  {
    return sync_read_ != nullptr && !read_entries_.empty();
  }

  [[nodiscard]] bool isOk() const;
  [[nodiscard]] unsigned int getErrorCount() const
  {
    return subsequent_error_count_;
  }

private:
  [[nodiscard]] const rclcpp::Logger& dxlLogger() const
  {
    return driver_ != nullptr ? driver_->dxlLogger() : dynamixel_ros_control::dxlLogger();
  }
  /// @brief Decode a received group into the bound values, of @p only if not null.
  /// @param missing_ids Receives the IDs whose data the group did not provide
  void decodeGroup(GroupSyncRead& group, const std::set<Dynamixel*>* only, std::set<unsigned int>& missing_ids);

  std::shared_ptr<GroupSyncRead> sync_read_{nullptr};

  DynamixelDriver* driver_{nullptr};
  std::set<Dynamixel*> dynamixels_;

  std::map<std::string, ReadEntry> read_entries_;

  unsigned int indirect_address_index_{0};
  uint16_t indirect_data_address_{0};
  uint16_t total_data_length_{0};

  unsigned int subsequent_error_count_{0};
  int last_comm_result_{COMM_SUCCESS};
  std::set<unsigned int> missing_ids_;  ///< Of the last read()
};

}  // namespace dynamixel_ros_control

#endif
