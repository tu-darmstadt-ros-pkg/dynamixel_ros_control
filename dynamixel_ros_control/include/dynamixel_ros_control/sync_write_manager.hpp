#ifndef DYNAMIXEL_ROS_CONTROL_SYNC_WRITE_MANAGER_H
#define DYNAMIXEL_ROS_CONTROL_SYNC_WRITE_MANAGER_H

#include <dynamixel_ros_control/dynamixel_driver.hpp>
#include <dynamixel_ros_control/dynamixel.hpp>
#include <dynamixel_ros_control/sdk_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <dynamixel_ros_control/common.hpp>
#include <set>

namespace dynamixel_ros_control {

struct WriteEntry
{
  std::string register_name;
  uint8_t data_length{0};
  uint16_t indirect_data_address{0};
  double* d_value{nullptr};
  bool* b_value{nullptr};
  int32_t* i_value{nullptr};  ///< Raw register value, written without unit conversion.
  /// Factor turning a physical value into raw ticks, resolved once in addEntry(). The cyclic write
  /// encodes every motor and register each cycle and must not reach back into the control table,
  /// which is a std::map lookup keyed by register name.
  double ratio{1.0};
  /// Encoding of the register, resolved once in addEntry() for the same reason as the ratio.
  RegisterEncoding encoding{RegisterEncoding::unknown};
  /// Last value that was both finite and narrow enough for the register. Sent again in place of
  /// one that is neither, so a bad command holds rather than becomes a different setpoint.
  int32_t last_written_value{0};
  bool non_finite_reported{false};
  bool out_of_range_reported{false};
};

class SyncWriteManager
{
public:
  SyncWriteManager() = default;
  /// @brief Bind a value to a register on one motor. False means the register cannot be written and
  /// nothing was bound. A caller that carries on regardless configures a group that is missing the
  /// register, and a sync write reports success whatever it does or does not contain.
  /// @param ratio replaces the register's own unit ratio unless 0
  [[nodiscard]] bool addRegister(Dynamixel& dxl, const std::string& register_name, double& value, double ratio = 0.0);
  [[nodiscard]] bool addRegister(Dynamixel& dxl, const std::string& register_name, bool& value);
  /// @brief Bind a raw register value, sent verbatim with no unit conversion. For registers whose
  /// meaning is the number itself (the bus watchdog's tick count), where converting through a
  /// physical unit would only round it a second time.
  [[nodiscard]] bool addRegister(Dynamixel& dxl, const std::string& register_name, int32_t& value);

  [[nodiscard]] bool init(DynamixelDriver& driver);

  /// @brief Rewrite the indirect address pointers of @p motors into the slots they already have,
  /// after a reboot wiped them
  [[nodiscard]] bool rewriteIndirectAddresses(const std::set<Dynamixel*>& motors);

  /// @brief Write every registered motor. Silent on a failed transmit, so the control loop decides what to report.
  [[nodiscard]] bool write();
  /// @brief Why the last write() failed to transmit.
  [[nodiscard]] std::string lastError() const;

  /// @brief Write a subset of the motors in a temporary group, leaving the error count alone.
  /// Success means the packet went out, not that a motor applied it. Motors not registered here are
  /// ignored.
  [[nodiscard]] bool writeSelected(const std::set<Dynamixel*>& motors);

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
  std::optional<std::reference_wrapper<WriteEntry>> addEntry(Dynamixel& dxl, const std::string& register_name);

  /// @brief Convert one motor's bound values into its wire payload, updating last_written_value.
  void encodeMotor(Dynamixel* dxl, std::vector<WriteEntry>& entries, std::vector<unsigned char>& buffer);

  std::unordered_map<Dynamixel*, std::vector<WriteEntry>> write_entries_;

  /// Wire payload of the motor currently being encoded, reused across calls so the encoding
  /// allocates nothing. The SDK's GroupSyncWrite still allocates in every changeParam() and
  /// txPacket(). encodeMotor() assigns it to size before each use. Safe to share between
  /// write() and writeSelected() because every caller holds the interface's comm mutex.
  std::vector<unsigned char> encode_buffer_;

  unsigned int indirect_address_index_{0};
  uint16_t indirect_data_address_{0};  ///< Shared start address of the group's data range.
  uint16_t total_data_length_{0};

  DynamixelDriver* driver_{nullptr};
  std::shared_ptr<GroupSyncWrite> sync_write_{nullptr};

  unsigned int subsequent_error_count_{0};
  int last_comm_result_{COMM_SUCCESS};
  /// Latched by a rejected addRegister(), so init() refuses the group even for a caller that
  /// carried on, since a sync write reports success whatever it is missing
  bool binding_rejected_{false};
};

}  // namespace dynamixel_ros_control

#endif
