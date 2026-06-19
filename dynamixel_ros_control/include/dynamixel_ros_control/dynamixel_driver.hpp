#ifndef DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_DRIVER_H
#define DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_DRIVER_H

#include <rclcpp/rclcpp.hpp>

#include <dynamixel_sdk/port_handler.h>
#include <dynamixel_sdk/packet_handler.h>
#include <dynamixel_sdk/group_sync_read.h>
#include <dynamixel_sdk/group_sync_write.h>

#include <dynamixel_ros_control/control_table.hpp>
#include <dynamixel_ros_control/sdk_wrapper.hpp>

namespace dynamixel_ros_control {

/// @brief Convert a ROS interface name to a Dynamixel register name.
std::string interfaceNameToRegister(const std::string& interface_name);

/**
 * @brief Outcome of a single-register read/write on the bus.
 *
 * `comm_result` is the SDK's COMM_* code (transport-level: timeout, CRC, ...).
 * `packet_error` is the motor's protocol-level error byte (out-of-range,
 * overload, etc.). `success` is true only when both indicate no error.
 *
 * Callers that just want a yes/no can use `operator bool()`; callers that
 * want to log a specific failure can pass the whole struct to
 * `describeError()`.
 */
struct RegisterAccessResult
{
  bool success{false};
  int comm_result{0};
  uint8_t packet_error{0};

  explicit operator bool() const
  {
    return success;
  }
};

/**
 * @brief Low-level driver for Dynamixel communication.
 *
 * Handles serial port management, packet handling, and provides methods for
 * reading/writing registers on individual motors. Supports both real hardware
 * and mock motors for testing.
 */
class DynamixelDriver
{
public:
  DynamixelDriver();

  /// @brief Initialize driver with port settings. Call connect() afterwards.
  [[nodiscard]] bool init(const std::string& port_name, int baud_rate, bool use_dummy = false);

  /// @brief Open the serial port and establish communication.
  [[nodiscard]] bool connect();

  /// @brief Load and cache the control table for a motor model.
  [[nodiscard]] ControlTable* loadControlTable(uint16_t model_number);

  /// @brief Ping a motor to check if it's connected.
  [[nodiscard]] bool ping(uint8_t id) const;

  /// @brief Ping a motor and retrieve its model number.
  [[nodiscard]] bool ping(uint8_t id, uint16_t& model_number) const;

  /// @brief Reboot a motor (clears hardware errors).
  [[nodiscard]] bool reboot(uint8_t id) const;

  /// @brief Scan the bus for all connected motors.
  [[nodiscard]] std::vector<std::pair<uint8_t /*id*/, uint16_t /*model_number*/>> scan() const;

  /// @brief Register a mock motor for testing (use_dummy must be true).
  void addDummyMotor(uint8_t id, uint16_t model_number);

  /// @brief Write a value to a motor register. Does not log on failure;
  /// callers are expected to log with their own context.
  [[nodiscard]] RegisterAccessResult writeRegister(uint8_t id, uint16_t address, uint8_t data_length,
                                                   int32_t value) const;

  /// @brief Read a value from a motor register. Does not log on failure;
  /// callers are expected to log with their own context.
  [[nodiscard]] RegisterAccessResult readRegister(uint8_t id, uint16_t address, uint8_t data_length,
                                                  int32_t& value_out) const;

  /// @brief Human-readable description of a RegisterAccessResult (empty if success).
  [[nodiscard]] std::string describeError(const RegisterAccessResult& result) const;

  /// @brief Create a GroupSyncWrite for efficient bulk writes.
  [[nodiscard]] std::shared_ptr<GroupSyncWrite> setSyncWrite(uint16_t address, uint8_t data_length) const;

  /// @brief Create a GroupSyncRead for efficient bulk reads.
  [[nodiscard]] std::shared_ptr<GroupSyncRead> setSyncRead(uint16_t address, uint8_t data_length) const;

  /// @brief Allocate indirect addresses for optimized bulk communication.
  [[nodiscard]] bool requestIndirectAddresses(unsigned int data_length, unsigned int& address_start_index);

  /// @brief Release previously allocated indirect addresses.
  [[nodiscard]] bool releaseIndirectAddresses(unsigned int data_length, unsigned int address_start_index);

  /// @brief Convert SDK communication result code to human-readable string.
  [[nodiscard]] std::string communicationErrorToString(int comm_result) const;

  /// @brief Convert SDK packet error code to human-readable string.
  [[nodiscard]] std::string packetErrorToString(uint8_t error) const;

private:
  bool loadSeriesMapping();
  ControlTable* readControlTable(std::string series);
  bool setPacketHandler();
  bool setPortHandler(const std::string& port_name);
  bool connectPort();
  bool setBaudRate(int baud_rate) const;

  std::shared_ptr<PacketHandler> packet_handler_;
  std::shared_ptr<PortHandler> port_handler_;

  unsigned int next_indirect_address_index_;

  std::string package_path_;
  std::map<uint16_t, std::string> model_number_to_series_;       ///< Maps model numbers to series names.
  std::map<std::string, ControlTable> series_to_control_table_;  ///< Cached control tables by series.

  // Configuration
  std::string port_name_;
  int baud_rate_;
  bool use_dummy_;
};

}  // namespace dynamixel_ros_control

#endif
