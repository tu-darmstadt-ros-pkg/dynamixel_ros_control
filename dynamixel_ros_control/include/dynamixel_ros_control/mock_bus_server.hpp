#ifndef DYNAMIXEL_ROS_CONTROL_MOCK_BUS_SERVER_HPP
#define DYNAMIXEL_ROS_CONTROL_MOCK_BUS_SERVER_HPP

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "dynamixel_ros_control/mock_dynamixel.hpp"

namespace dynamixel_ros_control {

namespace protocol2 {

/// CRC-16 of Protocol 2.0 (polynomial 0x8005). The SDK keeps its own private.
uint16_t crc(const uint8_t* data, size_t length);

/// Byte stuffing of a packet body, from the instruction to the last parameter. FD follows every FF FF FD.
std::vector<uint8_t> stuff(const std::vector<uint8_t>& body);

/// The inverse of stuff().
std::vector<uint8_t> unstuff(const std::vector<uint8_t>& body);

/// A complete status packet, stuffed and with its CRC.
std::vector<uint8_t> statusPacket(uint8_t id, uint8_t error, const std::vector<uint8_t>& params);

}  // namespace protocol2

/// How a status packet goes wrong on the wire. The motor has acted on the instruction either way.
enum class WireFault : uint8_t
{
  CORRUPT,   ///< A CRC bit flips, the master reads COMM_RX_CORRUPT.
  TRUNCATE,  ///< Only its first half arrives, the master times out with COMM_RX_CORRUPT.
  DROP,      ///< Nothing arrives, the master times out with COMM_RX_TIMEOUT.
  DELAY      ///< It arrives after the master gave up, behind the next instruction that asks for a status.
};

/// An owned file descriptor, closed when it goes out of scope.
class UniqueFd
{
public:
  UniqueFd() = default;
  ~UniqueFd();
  UniqueFd(const UniqueFd&) = delete;
  UniqueFd& operator=(const UniqueFd&) = delete;

  [[nodiscard]] int get() const
  {
    return fd_;
  }
  /// Owns @p fd from now on, closing the one held before
  void reset(int fd);

private:
  int fd_{-1};
};

/// Fault count meaning "every status packet until cleared". See MockBusServer::injectWireFault().
constexpr int PERSISTENT_WIRE_FAULT = -1;

/// @brief Serves motors of MockDynamixelManager as Protocol 2.0 packets on a pseudo terminal, so the
/// real SDK and PortHandlerLinux talk to them as to a serial adapter. The firmware rules stay in
/// MockDynamixel, the server only frames them. It keeps no clock, whoever owns time steps the
/// manager.
class MockBusServer
{
public:
  /// Serves @p ids. With a @p link, a symlink there points at the pty, for a fixed port name.
  explicit MockBusServer(const std::vector<uint8_t>& ids, std::string link = {});
  ~MockBusServer();

  MockBusServer(const MockBusServer&) = delete;
  MockBusServer& operator=(const MockBusServer&) = delete;

  /// The path a master opens, the link if there is one and the pty otherwise.
  [[nodiscard]] const std::string& portName() const
  {
    return link_.empty() ? pty_name_ : link_;
  }

  /// @brief Returns once every instruction the master sent is served, so a goal written just before
  /// a plant step takes effect within it. Spins while bytes arrive, which needs a master that
  /// pauses between transactions, as a control loop does.
  void waitUntilIdle();

  /// The next @p count status packets suffer @p fault, or every one with PERSISTENT_WIRE_FAULT.
  void injectWireFault(WireFault fault, int count = 1);
  void clearWireFault();

  /// Bus power for every served motor. Off silences them and leaves them limp, on boots them with
  /// their RAM reset, as after a hard e-stop.
  void setPower(bool on);

private:
  void serve();
  void serveUntilStopped();
  void handle(uint8_t id, uint8_t instruction, const std::vector<uint8_t>& params);
  void syncInstruction(uint8_t instruction, uint16_t address, uint16_t length, const std::vector<uint8_t>& params);
  void reply(uint8_t id, uint8_t error, const std::vector<uint8_t>& params);
  void send(const std::vector<uint8_t>& bytes) const;
  std::optional<WireFault> takeWireFault();

  std::set<uint8_t> ids_;
  std::string link_;
  std::string pty_name_;
  UniqueFd master_fd_;
  // Held open so the master survives the plugin closing and reopening the port
  UniqueFd slave_fd_;
  // Held while serving what one read brought
  std::mutex serve_mutex_;
  std::mutex fault_mutex_;
  std::optional<WireFault> fault_;
  int fault_count_{0};
  // Status bytes held back by DELAY, touched only by the serving thread
  std::vector<uint8_t> delayed_;
  std::atomic<bool> stop_{false};
  // Set by a serving thread that hit an error and stopped, so nothing waits for it any more
  std::atomic<bool> failed_{false};
  std::thread thread_;
};

}  // namespace dynamixel_ros_control

#endif  // DYNAMIXEL_ROS_CONTROL_MOCK_BUS_SERVER_HPP
