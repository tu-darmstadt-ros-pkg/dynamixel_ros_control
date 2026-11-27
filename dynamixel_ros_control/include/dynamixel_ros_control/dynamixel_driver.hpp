#ifndef DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_DRIVER_H
#define DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_DRIVER_H

#include <chrono>
#include <filesystem>
#include <optional>

#include <rclcpp/rclcpp.hpp>

#include <dynamixel_sdk/port_handler.h>
#include <dynamixel_sdk/packet_handler.h>
#include <dynamixel_sdk/group_sync_read.h>
#include <dynamixel_sdk/group_sync_write.h>

#include <dynamixel_ros_control/control_table.hpp>
#include <dynamixel_ros_control/log.hpp>
#include <dynamixel_ros_control/sdk_wrapper.hpp>

namespace dynamixel_ros_control {

/// The FTDI latency timer a bus needs, in ms. The adapter default of 16 ms delays every status packet.
constexpr int USB_LATENCY_TIMER_TARGET_MS = 1;

/// @brief Outcome of a single-register read or write, the SDK's COMM_* code and the motor's error
/// byte. describeError() turns it into text.
struct RegisterAccessResult
{
  /// Bit 7 of the status packet's error byte. The motor sets it on the answer to every instruction
  /// while a hardware error is latched, so it describes the motor, not this transaction.
  static constexpr uint8_t ALERT_BIT = 0x80;

  int comm_result{COMM_TX_FAIL};
  uint8_t packet_error{0};

  /// @brief Whether the instruction was carried out. The alert bit is excluded: a motor that is
  /// flagging a fault still reads and writes its registers, and a process that meets one already
  /// errored has to be able to reach it in order to reboot it.
  [[nodiscard]] bool success() const
  {
    return comm_result == COMM_SUCCESS && (packet_error & static_cast<uint8_t>(~ALERT_BIT)) == 0;
  }

  /// @brief Whether the motor is flagging a hardware error. hardware_error_status says which one.
  [[nodiscard]] bool alertPresent() const
  {
    return (packet_error & ALERT_BIT) != 0;
  }

  explicit operator bool() const
  {
    return success();
  }
};

/// @brief Counters of bus transactions, for the startup timing lines. A group transaction costs one
/// round trip whatever it carries, so it is counted apart from the addressed ones.
struct CommStats
{
  uint64_t reads{0};
  uint64_t writes{0};
  uint64_t read_failures{0};
  uint64_t write_failures{0};
  uint64_t pings{0};
  uint64_t ping_failures{0};
  uint64_t group_reads{0};   ///< Sync-read group transactions, whatever their member count.
  uint64_t group_writes{0};  ///< Sync-write group transactions, whatever their member count.
  double bus_time_ms{0.0};   ///< Wall time spent inside SDK TxRx calls.

  /// @brief Difference between this snapshot and an earlier one.
  [[nodiscard]] CommStats delta(const CommStats& since) const;

  [[nodiscard]] std::string toString() const;
};

/// @brief The port, the packet handler and the addressed register access of one bus
class DynamixelDriver
{
public:
  DynamixelDriver();

  /// @brief Initialize driver with port settings. Call connect() afterwards.
  /// @param use_dummy talk to the motors this process simulates instead of the serial port
  /// @param logger The logger of everything on this bus
  [[nodiscard]] bool init(const std::string& port_name, int baud_rate, bool use_dummy = false,
                          const rclcpp::Logger& logger = dynamixel_ros_control::dxlLogger());

  [[nodiscard]] const rclcpp::Logger& dxlLogger() const
  {
    return logger_;
  }

  [[nodiscard]] const std::string& portName() const
  {
    return port_name_;
  }

  /// @brief The latency timer of the USB serial adapter behind the port, in ms, from
  /// <sysfs_root>/bus/usb-serial/devices/<tty>/latency_timer. nullopt for a port without one, such
  /// as a pty or a ttyACM.
  [[nodiscard]] std::optional<int> usbLatencyTimerMs(const std::filesystem::path& sysfs_root) const;

  /// @brief Record one sync-read / sync-write group transaction. Called by the sync managers,
  /// which own the group objects the driver hands out.
  void countGroupRead()
  {
    comm_stats_.group_reads++;
    noteTraffic();
  }
  void countGroupWrite()
  {
    comm_stats_.group_writes++;
    noteTraffic();
  }

  /// @brief The longest silence between two bus transactions since the last call, in ms, and start
  /// measuring afresh. A silence longer than a bus watchdog window trips the motors' watchdogs.
  [[nodiscard]] double takeLongestSilenceMs();

  /// @brief Open the serial port and establish communication.
  [[nodiscard]] bool connect();

  /// @brief Load and cache the control table for a motor model.
  [[nodiscard]] ControlTable* loadControlTable(uint16_t model_number);

  /// @brief Ping a motor, retrieving its model number and the SDK COMM_* result
  /// code for failure diagnosis (timeout vs. corrupted response vs. port error).
  [[nodiscard]] bool ping(uint8_t id, uint16_t& model_number, int& comm_result_out) const;

  /// @brief Reboot a motor (clears hardware errors).
  [[nodiscard]] bool reboot(uint8_t id) const;

  /// @brief Write a value to a motor register. Does not log on failure.
  /// Callers are expected to log with their own context.
  [[nodiscard]] RegisterAccessResult writeRegister(uint8_t id, uint16_t address, uint8_t data_length,
                                                   int32_t value) const;

  /// @brief Read a value from a motor register. Does not log on failure.
  /// Callers are expected to log with their own context.
  [[nodiscard]] RegisterAccessResult readRegister(uint8_t id, uint16_t address, uint8_t data_length, int32_t& value_out,
                                                  bool is_signed) const;

  /// @brief Human-readable description of a RegisterAccessResult (empty if success).
  [[nodiscard]] std::string describeError(const RegisterAccessResult& result) const;

  /// @brief Create a GroupSyncWrite for efficient bulk writes.
  [[nodiscard]] std::shared_ptr<GroupSyncWrite> setSyncWrite(uint16_t address, uint16_t data_length) const;

  /// @brief Create a GroupSyncRead for efficient bulk reads.
  [[nodiscard]] std::shared_ptr<GroupSyncRead> setSyncRead(uint16_t address, uint16_t data_length) const;

  /// @brief Allocate indirect addresses for optimized bulk communication.
  [[nodiscard]] unsigned int requestIndirectAddresses(unsigned int data_length);

  /// @brief Convert SDK communication result code to human-readable string.
  [[nodiscard]] std::string communicationErrorToString(int comm_result) const;

  /// @brief Convert SDK packet error code to human-readable string.
  [[nodiscard]] std::string packetErrorToString(uint8_t error) const;

  /// @brief The accumulated transaction counters
  [[nodiscard]] const CommStats& getCommStats() const
  {
    return comm_stats_;
  }

private:
  bool loadSeriesMapping();
  ControlTable* readControlTable(std::string series);
  bool connectPort();
  bool setBaudRate(int baud_rate) const;

  const Sdk* sdk_{nullptr};
  std::shared_ptr<PacketHandler> packet_handler_;
  std::shared_ptr<PortHandler> port_handler_;

  unsigned int next_indirect_address_index_;

  std::string package_path_;
  std::map<uint16_t, std::string> model_number_to_series_;       ///< Maps model numbers to series names.
  std::map<std::string, ControlTable> series_to_control_table_;  ///< Cached control tables by series.

  // Configuration
  rclcpp::Logger logger_{dynamixel_ros_control::dxlLogger()};
  std::string port_name_;
  int baud_rate_;

  // Diagnostics (mutable: read/write/ping are const but should still be counted)
  mutable CommStats comm_stats_;

  /// Records the silence since the previous transaction. Every transaction calls it.
  void noteTraffic() const;
  mutable std::chrono::steady_clock::time_point last_traffic_{};
  mutable double longest_silence_ms_{0.0};
};

}  // namespace dynamixel_ros_control

#endif
