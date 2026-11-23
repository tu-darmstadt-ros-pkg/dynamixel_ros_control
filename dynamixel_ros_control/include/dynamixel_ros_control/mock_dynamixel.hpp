#ifndef DYNAMIXEL_ROS_CONTROL_MOCK_DYNAMIXEL_HPP
#define DYNAMIXEL_ROS_CONTROL_MOCK_DYNAMIXEL_HPP

#include "dynamixel_ros_control/sdk_wrapper.hpp"

#include <algorithm>
#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace dynamixel_ros_control {

/// The model of a simulated motor whose joint names none (PH54-200-S500-R)
constexpr uint16_t DEFAULT_MOCK_MODEL_NUMBER = 2020;

enum class OperatingMode : uint8_t
{
  CURRENT_CONTROL = 0,
  VELOCITY_CONTROL = 1,
  POSITION_CONTROL = 3,
  EXTENDED_POSITION_CONTROL = 4,
  CURRENT_BASED_POSITION_CONTROL = 5,
  PWM_CONTROL = 16
};

/// The Hardware Error Status register, every bit, so a test cannot inject a pattern no name covers
enum HardwareErrorBit : uint8_t
{
  ERROR_NONE = 0x00,
  ERROR_INPUT_VOLTAGE = 0x01,
  ERROR_HALL_SENSOR = 0x02,
  ERROR_OVERHEATING = 0x04,
  ERROR_MOTOR_ENCODER = 0x08,
  ERROR_ELECTRICAL_SHOCK = 0x10,
  ERROR_OVERLOAD = 0x20
};

// The error field of a status packet: an error number for the instruction in bits 0 to 6, and an
// alert in bit 7 while a hardware error is set, so a refused write and a hot motor stay apart
constexpr uint8_t PACKET_ERROR_INSTRUCTION = 0x02;
constexpr uint8_t PACKET_ERROR_DATA_RANGE = 0x04;
constexpr uint8_t PACKET_ERROR_DATA_LENGTH = 0x05;
constexpr uint8_t PACKET_ERROR_DATA_LIMIT = 0x06;  ///< a value over its limit register
constexpr uint8_t PACKET_ERROR_ACCESS = 0x07;      ///< read only, or EEPROM while torque is on
constexpr uint8_t PACKET_ERROR_ALERT = 0x80;

/// How a motor fails to communicate. A motor can stop answering and still receive instructions.
enum class CommunicationFault : uint8_t
{
  NONE,
  DROP_STATUS_PACKETS,  ///< applies instructions, answers none
  DISCONNECTED          ///< neither answers nor receives
};

/// How a motor mishandles the writes to one register
enum class RegisterWriteFault : uint8_t
{
  DROP,    ///< acknowledged and not applied, the master sees success
  TIMEOUT  ///< neither answered nor applied
};

/// A read fault count meaning every read
constexpr int PERSISTENT_READ_FAULT = -1;

/// The status packet a motor answers an instruction with
struct MockStatus
{
  uint8_t error = 0;
  std::vector<uint8_t> params;
};

/// What a servo on an external plant commands, in the frame of its present_position register.
struct ServoCommand
{
  bool torque_on = false;
  double position = 0.0;       ///< reference, rad
  double velocity = 0.0;       ///< reference, rad/s
  double current_limit = 0.0;  ///< A
};

/// One simulated motor: the control table of its model, a kinematic model, the firmware rules the
/// plugin relies on and fault injection
class MockDynamixel
{
public:
  /// @throws std::runtime_error if devices/ has no usable control table for the model
  MockDynamixel(uint8_t id, uint16_t model_number);

  /// Steps the kinematic model by @p dt seconds
  void update(double dt);

  /// @brief Step the motor on an external plant such as a physics engine. The kinematic state
  /// becomes the plant's reference and the present registers report the measurement. While torque
  /// is off the reference follows the plant, so a torque on starts where the joint is.
  /// @param position measured, as present_position reports it, homing included, rad
  /// @param velocity measured, rad/s
  /// @param current measured, A
  ServoCommand followPlant(double dt, double position, double velocity, double current);

  /// @brief Write a register in its unit. A fresh mock holds 0 in its limits, which a plant would
  /// read as no current and no speed, so a simulation writes the factory values.
  /// @throws std::invalid_argument if the model has no such register or no conversion for its unit
  void setFactoryValue(const std::string& register_name, double value);

  // The registers, through the indirect pointers, without the firmware rules of a bus write
  uint8_t read1Byte(uint16_t address) const;
  uint16_t read2Byte(uint16_t address) const;
  uint32_t read4Byte(uint16_t address) const;
  int32_t read4ByteSigned(uint16_t address) const;
  void write1Byte(uint16_t address, uint8_t data);
  void write2Byte(uint16_t address, uint16_t data);
  void write4Byte(uint16_t address, uint32_t data);

  uint8_t getId() const
  {
    return id_;
  }
  uint16_t getModelNumber() const
  {
    return model_number_;
  }

  /// A bit selected in the Shutdown register also clears torque, which then only a reboot brings back
  void setHardwareError(uint8_t error_code);

  /// The error survives a reboot, as one whose cause persists
  void setHardwareErrorSticky(bool sticky)
  {
    hardware_error_sticky_ = sticky;
  }

  /// What a reboot does to the error. A sticky one is written again, because the reboot zeroed its register.
  void clearHardwareError()
  {
    setHardwareError(hardware_error_sticky_ ? hardware_error_.load() : 0);
  }

  uint8_t getHardwareError() const
  {
    return hardware_error_;
  }

  /// RAM to 0 and RAM pointers to their own slot, as a reboot leaves them
  void resetRAMRegisters();

  /// The output turned by hand: the position changes, nothing else does
  void moveByHand(double delta_rad)
  {
    const std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
    current_position_ += delta_rad;
    writePresentValues();
  }

  /// RAM back to defaults, and the position back as the absolute value within one turn
  void powerCycle();

  int getRebootCount() const
  {
    return reboot_count_;
  }
  void resetRebootCount()
  {
    reboot_count_ = 0;
  }

  void setCommunicationFault(CommunicationFault fault)
  {
    comm_fault_ = fault;
  }

  /// Writes to the register fail, also through an indirect slot
  void setRegisterWriteFault(const std::string& register_name, RegisterWriteFault fault);
  void clearRegisterWriteFaults();

  /// @brief Every read covering the register times out, also through an indirect slot. A motor
  /// answers a read whole or not at all, so a group over the register gets no answer from it.
  void setRegisterReadFault(const std::string& register_name);

  /// @brief The next @p count read packets covering the register time out, which tells a caller
  /// that retries from one that gives up. Counted per packet: readWhoeverAnswers() spends one on
  /// the group read and one on each probe.
  void setRegisterReadFaultCount(const std::string& register_name, int count);

  /// On by default as in the firmware, the indirect pointers of the P and PRO series included
  void setRejectEepromWritesWhileTorqued(bool reject);

  /// @brief Runs @p hook on the reading thread before an answered read of the register is served,
  /// which is a window the interface holds its bus lock across
  void setRegisterReadHook(const std::string& register_name, std::function<void()> hook);
  void clearRegisterReadHooks();

  /// One stored byte, by resolved address
  struct TracedWrite
  {
    uint16_t address;
    uint8_t value;
  };

  /// @brief Records the stored bytes in order, for rules about order that the end state does not
  /// show. A dropped write leaves no entry.
  void startWriteTrace();
  void stopWriteTrace();

  // Trace index of the last or first write of @p value to the register's first byte, or of any
  // write to the register, -1 for none
  int lastWriteIndex(const std::string& register_name, uint8_t value) const;
  int lastWriteIndex(const std::string& register_name) const;
  int firstWriteIndex(const std::string& register_name, uint8_t value) const;
  int firstWriteIndex(const std::string& register_name) const;

  /// 0 for a register the model does not have
  uint16_t getAddress(const std::string& name) const
  {
    auto it = address_map_.find(name);
    return (it != address_map_.end()) ? it->second : 0;
  }
  uint8_t getLength(const std::string& name) const
  {
    auto it = length_map_.find(name);
    return (it != length_map_.end()) ? it->second : 0;
  }

  /// The kinematic position, without the homing offset, rad
  double getCurrentPosition() const
  {
    const std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
    return current_position_;
  }
  /// The position present_position reports, homing included, rad
  double getJointPosition() const;
  double getCurrentVelocity() const
  {
    const std::lock_guard<std::recursive_mutex> lock(memory_mutex_);
    return current_velocity_;
  }

  uint8_t getLedRed() const
  {
    return led_red_addr_ > 0 ? read1Byte(led_red_addr_) : 0;
  }
  uint8_t getLedGreen() const
  {
    return led_green_addr_ > 0 ? read1Byte(led_green_addr_) : 0;
  }
  uint8_t getLedBlue() const
  {
    return led_blue_addr_ > 0 ? read1Byte(led_blue_addr_) : 0;
  }
  void setLedRed(uint8_t value)
  {
    if (led_red_addr_ > 0)
      write1Byte(led_red_addr_, value);
  }
  void setLedGreen(uint8_t value)
  {
    if (led_green_addr_ > 0)
      write1Byte(led_green_addr_, value);
  }
  void setLedBlue(uint8_t value)
  {
    if (led_blue_addr_ > 0)
      write1Byte(led_blue_addr_, value);
  }

  /// Bus writes to bus_watchdog, the traffic that repeated arming leaves invisible in the register
  uint64_t getBusWatchdogWriteCount() const
  {
    return bus_watchdog_write_count_.load(std::memory_order_relaxed);
  }
  void resetBusWatchdogWriteCount()
  {
    bus_watchdog_write_count_.store(0, std::memory_order_relaxed);
  }

  uint8_t getBusWatchdog() const
  {
    return bus_watchdog_addr_ > 0 ? read1Byte(bus_watchdog_addr_) : 0;
  }

  /// @brief Trips the bus watchdog as a window without packets does: the motor stops and the
  /// register reads -1 until written 0. Without the manager's timing model only this trips it,
  /// which keeps tests free of wall clock races.
  void tripBusWatchdog();

  /// Trips of the timing model, which a repaired trip leaves no other trace of
  [[nodiscard]] int timedBusWatchdogTrips() const;

  bool busWatchdogLatched() const;

  int32_t getHomingOffset() const
  {
    return homing_offset_addr_ > 0 ? read4ByteSigned(homing_offset_addr_) : 0;
  }
  void setHomingOffset(int32_t offset)
  {
    if (homing_offset_addr_ > 0)
      write4Byte(homing_offset_addr_, static_cast<uint32_t>(offset));
  }

private:
  // The bus side of the motor, for the classes that serve its packets
  friend class MockPacketHandler;
  friend class MockGroupSyncRead;
  friend class MockGroupSyncWrite;
  friend class MockBusServer;

  // Every instruction as the firmware handles it, nullopt where no status packet comes back. Sync
  // instructions take the same path and drop the status.
  std::optional<MockStatus> answerPing();
  std::optional<MockStatus> answerReboot();
  std::optional<MockStatus> answerRead(uint16_t address, uint16_t length);
  std::optional<MockStatus> answerWrite(uint16_t address, const uint8_t* data, uint16_t length);

  bool respondsToStatus() const
  {
    return comm_fault_ == CommunicationFault::NONE;
  }
  bool receivesWrites() const
  {
    return comm_fault_ != CommunicationFault::DISCONNECTED;
  }
  /// @p error with the alert bit a hardware error adds to every status
  uint8_t statusError(uint8_t error) const;
  /// Restarts the watchdog window of the timing model, a read only if the manager says reads count
  void hearInstruction(bool is_read);
  /// The error a write instruction is refused with as a whole, 0 if it is not. Caller holds memory_mutex_.
  uint8_t writeRefusal(uint16_t address, const uint8_t* data, uint16_t length) const;
  /// Whether @p target is outside the RAM area an indirect pointer may address (Data Range Error)
  bool pointerOutOfRange(uint16_t target) const;
  /// The RAM area as [begin, end). The firmware bounds pointers by the area, so gaps between
  /// listed registers are inside it.
  std::optional<std::pair<uint16_t, uint16_t>> ramArea() const;
  /// Writes one byte at a resolved address. Caller holds memory_mutex_.
  void store(uint16_t address, uint8_t data);

  enum class TraceSearch : uint8_t
  {
    FIRST,
    LAST
  };

  /// @p matches gets the register's address and width
  int findWriteIndex(const std::string& register_name, TraceSearch direction,
                     const std::function<bool(const TracedWrite&, uint16_t, uint8_t)>& matches) const;

  void loadModelSpecs(uint16_t model_number);
  /// Pointer n to its own data slot n, as a motor ships (e-Manual)
  void resetIndirectPointers();
  /// Whether a hardware error selected in the Shutdown register is set. Caller holds memory_mutex_.
  bool shutdownLatched() const;
  bool isGoalRegisterAddress(uint16_t address) const;
  /// drive_mode bit 2, in which the profile registers are durations, which the model ignores
  bool isTimeBasedProfile() const;
  double getPositionModeVelocityLimit() const;
  std::pair<double, double> getPositionLimits() const;
  double getAccelerationLimit() const;
  /// The unit of @p register_name per tick, 0 for a register without one
  double perTick(const std::string& register_name) const;
  void updatePositionMode(double dt, bool enforce_limits = true);
  void updateVelocityMode(double dt);
  void updateCurrentMode(double dt);
  void updateMovingFlag();
  void writePresentValues();
  void writePresentValues(double position, double velocity, double current);
  /// The register an indirect slot points to, any other address unchanged
  uint16_t resolveAddress(uint16_t address) const;
  /// Whether @p address is faulted, spending a use of a counted fault. Caller holds memory_mutex_.
  bool consumeReadFault(uint16_t address) const;

  uint8_t id_;
  uint16_t model_number_;
  std::vector<uint8_t> memory_;
  // The physics, the bus and the tests run on different threads. Recursive, because update() and
  // the bus re-enter the register accessors.
  mutable std::recursive_mutex memory_mutex_;
  std::map<std::string, uint16_t> address_map_;
  std::map<std::string, uint8_t> length_map_;
  std::map<std::string, std::string> unit_map_;
  /// Unit per tick, by unit name
  std::map<std::string, double> unit_conversions_;

  // Cached addresses
  uint16_t torque_enable_addr_ = 0;
  uint16_t operating_mode_addr_ = 0;
  uint16_t goal_position_addr_ = 0;
  uint16_t goal_velocity_addr_ = 0;
  uint16_t goal_current_addr_ = 0;
  uint16_t goal_pwm_addr_ = 0;
  uint16_t present_position_addr_ = 0;
  uint16_t present_velocity_addr_ = 0;
  uint16_t present_current_addr_ = 0;
  uint16_t current_limit_addr_ = 0;
  uint16_t velocity_limit_addr_ = 0;
  uint16_t acceleration_limit_addr_ = 0;
  uint16_t hardware_error_addr_ = 0;
  uint16_t moving_addr_ = 0;
  uint16_t profile_velocity_addr_ = 0;
  uint16_t profile_acceleration_addr_ = 0;
  uint16_t max_position_limit_addr_ = 0;
  uint16_t min_position_limit_addr_ = 0;
  uint16_t homing_offset_addr_ = 0;
  uint16_t led_red_addr_ = 0;
  uint16_t led_green_addr_ = 0;
  uint16_t led_blue_addr_ = 0;
  uint16_t bus_watchdog_addr_ = 0;
  double bus_silence_s_ = 0.0;
  int timed_bus_watchdog_trips_ = 0;
  std::atomic<uint64_t> bus_watchdog_write_count_{0};
  uint16_t drive_mode_addr_ = 0;
  uint16_t shutdown_addr_ = 0;

  // The units of the kinematic model, H42 values where a table has none
  double rad_per_tick_ = 0.00002068538;
  double rad_per_s_per_tick_ = 0.000344746;
  double ampere_per_tick_ = 0.001;

  double current_position_ = 0.0;
  double current_velocity_ = 0.0;
  double current_acceleration_ = 0.0;
  double current_current_ = 0.0;

  // What a motor moves at with neither limit nor profile set
  static constexpr double DEFAULT_VELOCITY_LIMIT = 100.0;     // rad/s
  static constexpr double DEFAULT_ACCELERATION_LIMIT = 50.0;  // rad/s^2

  // Atomic, as a test injects them while the control thread reads the motor
  std::atomic<uint8_t> hardware_error_{0};
  std::atomic<bool> hardware_error_sticky_{false};
  std::atomic<CommunicationFault> comm_fault_{CommunicationFault::NONE};
  // By resolved address, one entry per byte
  std::map<uint16_t, RegisterWriteFault> register_write_faults_;
  mutable std::map<uint16_t, int> register_read_faults_;  ///< reads left to fail
  std::map<uint16_t, std::function<void()>> register_read_hooks_;
  bool write_trace_enabled_ = false;
  std::vector<TracedWrite> write_trace_;
  std::atomic<int> reboot_count_{0};
  // (address, length)
  std::vector<std::pair<uint16_t, uint8_t>> ram_registers_;
  std::vector<std::pair<uint16_t, uint8_t>> eeprom_registers_;
  std::atomic<bool> reject_eeprom_writes_while_torqued_{true};

  uint16_t indirect_address_start_ = 0;
  uint16_t indirect_data_start_ = 0;
  uint16_t indirect_count_ = 0;
  bool indirect_in_ram_ = false;  ///< X series, a reboot resets the pointers
  bool profile_velocity_bounded_ = false;
};

/// The motors this process simulates. The plugin reaches them with use_dummy.
class MockDynamixelManager
{
public:
  static MockDynamixelManager& instance()
  {
    static MockDynamixelManager instance;
    return instance;
  }

  /// Leaves a motor that has the id alone. @throws std::runtime_error for a model it cannot simulate
  void addMotor(uint8_t id, uint16_t model_number);
  void removeMotor(uint8_t id);
  std::shared_ptr<MockDynamixel> getMotor(uint8_t id);
  std::vector<uint8_t> getConnectedIds();

  void update(double dt = 0.01);

  /// No motors and no faults, between tests
  void reset();

  /// Disconnects every motor, or reconnects them
  void setGlobalCommunicationError(bool enabled);
  /// The next @p count instruction packets reach the motors, then every motor disconnects as on a
  /// bus power cut in the middle of a transaction sequence
  void cutBusAfterPackets(int count);

  /// @brief Sync writes fail to reach the bus, as on a serial write error, the one way a sync
  /// write fails since no motor answers it
  void setSyncWriteTransmissionFailure(bool enabled);
  void failNextSyncWriteTransmissions(int count);
  /// Whether the sync write about to go out fails, spending one of a counted failure
  [[nodiscard]] bool takeSyncWriteTransmissionFailure();

  /// @brief Each watchdog trips by itself after a window without instructions. Off by default, as
  /// the controller manager tests run on wall time. @p fed_by_reads whether reads restart the window.
  void setBusWatchdogTiming(bool enabled, bool fed_by_reads);
  [[nodiscard]] bool busWatchdogTimed() const;
  [[nodiscard]] bool busWatchdogFedByReads() const;

  /// Instruction packets the in-process handlers sent since the process started, a sync instruction counting once
  [[nodiscard]] uint64_t packetCount() const
  {
    return packets_.load(std::memory_order_relaxed);
  }

private:
  friend class MockPacketHandler;
  friend class MockGroupSyncRead;
  friend class MockGroupSyncWrite;

  MockDynamixelManager();
  void countPacket();
  std::map<uint8_t, std::shared_ptr<MockDynamixel>> motors_;
  /// Sync writes left to fail, negative for all of them
  std::atomic<int> sync_write_tx_failures_{0};
  std::atomic<bool> bus_watchdog_timed_{false};
  std::atomic<bool> bus_watchdog_fed_by_reads_{true};
  std::atomic<uint64_t> packets_{0};
  /// Packets left before the cut, negative for none pending. Under mutex_.
  int packets_until_cut_{-1};
  std::mutex mutex_;
};

class MockPortHandler : public PortHandler
{
public:
  MockPortHandler(const char* port_name)
      : port_name_(port_name)
  {}

  bool openPort() override
  {
    return true;
  }
  std::string getPortName() override
  {
    return port_name_;
  }
  bool setBaudRate(const int baudrate) override
  {
    baud_rate_ = baudrate;
    return true;
  }
  int getBaudRate() override
  {
    return baud_rate_;
  }

private:
  std::string port_name_;
  int baud_rate_ = 57600;
};

/// Frames the answers of the simulated motors as the SDK's packet handler returns them
class MockPacketHandler : public PacketHandler
{
public:
  int ping(PortHandler& port, uint8_t id, uint16_t* model_number, uint8_t* error) override;
  int reboot(PortHandler& port, uint8_t id, uint8_t* error) override;

  int read1ByteTxRx(PortHandler& port, uint8_t id, uint16_t address, uint8_t* data, uint8_t* error) override;
  int read2ByteTxRx(PortHandler& port, uint8_t id, uint16_t address, uint16_t* data, uint8_t* error) override;
  int read4ByteTxRx(PortHandler& port, uint8_t id, uint16_t address, uint32_t* data, uint8_t* error) override;

  int write1ByteTxRx(PortHandler& port, uint8_t id, uint16_t address, uint8_t data, uint8_t* error) override;
  int write2ByteTxRx(PortHandler& port, uint8_t id, uint16_t address, uint16_t data, uint8_t* error) override;
  int write4ByteTxRx(PortHandler& port, uint8_t id, uint16_t address, uint32_t data, uint8_t* error) override;

  const char* getTxRxResult(int result) override;
  const char* getRxPacketError(uint8_t error) override;

private:
  template <typename T>
  static int readTxRx(uint8_t id, uint16_t address, T* data, uint8_t* error);
  template <typename T>
  static int writeTxRx(uint8_t id, uint16_t address, T data, uint8_t* error);
};

class MockGroupSyncRead : public GroupSyncRead
{
public:
  MockGroupSyncRead(std::shared_ptr<PortHandler> /*port*/, std::shared_ptr<PacketHandler> /*ph*/,
                    uint16_t start_address, uint16_t data_length)
      : start_address_(start_address), data_length_(data_length)
  {}

  bool addParam(uint8_t id) override
  {
    // As the SDK group, which is what surfaces two joints on one motor id
    if (std::find(ids_.begin(), ids_.end(), id) != ids_.end()) {
      return false;
    }
    ids_.push_back(id);
    return true;
  }
  int txRxPacket() override;
  bool isAvailable(uint8_t id, uint16_t address, uint16_t data_length) override;
  uint32_t getData(uint8_t id, uint16_t address, uint16_t data_length) override;

private:
  uint16_t start_address_;
  uint16_t data_length_;
  std::vector<uint8_t> ids_;
  std::map<uint8_t, std::vector<uint8_t>> data_map_;
};

class MockGroupSyncWrite : public GroupSyncWrite
{
public:
  MockGroupSyncWrite(std::shared_ptr<PortHandler> /*port*/, std::shared_ptr<PacketHandler> /*ph*/,
                     uint16_t start_address, uint16_t data_length)
      : start_address_(start_address), data_length_(data_length)
  {}

  bool addParam(uint8_t id, uint8_t* data) override;
  bool changeParam(uint8_t id, uint8_t* data) override;
  int txPacket() override;

private:
  uint16_t start_address_;
  uint16_t data_length_;
  std::map<uint8_t, std::vector<uint8_t>> params_;
};

}  // namespace dynamixel_ros_control

#endif  // DYNAMIXEL_ROS_CONTROL_MOCK_DYNAMIXEL_HPP
