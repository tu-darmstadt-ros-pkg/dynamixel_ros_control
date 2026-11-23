#include "dynamixel_ros_control/log.hpp"

#include <atomic>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <dynamixel_ros_control/dynamixel_driver.hpp>

#include <dynamixel_ros_control/common.hpp>
#include <ament_index_cpp/get_package_share_path.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <yaml-cpp/yaml.h>

namespace dynamixel_ros_control {

namespace {

class SerialSdk : public Sdk
{
public:
  std::shared_ptr<PortHandler> portHandler(const std::string& port_name) const override
  {
    return std::make_shared<RealPortHandler>(port_name.c_str());
  }
  std::shared_ptr<PacketHandler> packetHandler() const override
  {
    return std::make_shared<RealPacketHandler>(2.0F);
  }
  std::shared_ptr<GroupSyncRead> syncRead(const std::shared_ptr<PortHandler>& port,
                                          const std::shared_ptr<PacketHandler>& packet_handler, const uint16_t address,
                                          const uint16_t length) const override
  {
    return std::make_shared<RealGroupSyncRead>(port, packet_handler, address, length);
  }
  std::shared_ptr<GroupSyncWrite> syncWrite(const std::shared_ptr<PortHandler>& port,
                                            const std::shared_ptr<PacketHandler>& packet_handler,
                                            const uint16_t address, const uint16_t length) const override
  {
    return std::make_shared<RealGroupSyncWrite>(port, packet_handler, address, length);
  }
};

std::atomic<const Sdk*>& dummySdk()
{
  static std::atomic<const Sdk*> sdk{nullptr};
  return sdk;
}

}  // namespace

void setDummySdk(const Sdk* sdk)
{
  dummySdk() = sdk;
}

CommStats CommStats::delta(const CommStats& since) const
{
  CommStats d;
  d.reads = reads - since.reads;
  d.writes = writes - since.writes;
  d.read_failures = read_failures - since.read_failures;
  d.write_failures = write_failures - since.write_failures;
  d.pings = pings - since.pings;
  d.ping_failures = ping_failures - since.ping_failures;
  d.group_reads = group_reads - since.group_reads;
  d.group_writes = group_writes - since.group_writes;
  d.bus_time_ms = bus_time_ms - since.bus_time_ms;
  return d;
}

std::string CommStats::toString() const
{
  std::stringstream ss;
  ss << reads << " reads (" << read_failures << " failed), " << writes << " writes (" << write_failures << " failed), "
     << pings << " pings (" << ping_failures << " failed), " << group_reads << " group reads, " << group_writes
     << " group writes, bus busy " << static_cast<int>(bus_time_ms) << " ms";
  return ss.str();
}

DynamixelDriver::DynamixelDriver()
    : next_indirect_address_index_(0)
{}

bool DynamixelDriver::init(const std::string& port_name, const int baud_rate, bool use_dummy,
                           const rclcpp::Logger& logger)
{
  static const SerialSdk serial;
  logger_ = logger;
  sdk_ = use_dummy ? dummySdk().load() : &serial;
  if (sdk_ == nullptr) {
    DXL_LOG_FATAL("use_dummy needs motors simulated in this process, as the tests have. "
                  "To simulate a bus, run dynamixel_bus_emulator and point port_name at it.");
    return false;
  }
  port_name_ = port_name;
  // PortHandlerLinux strcpy's the name into a char[100]
  if (!use_dummy && port_name_.size() >= 100) {
    DXL_LOG_FATAL("Port name '" << port_name_ << "' is longer than the 99 characters the SDK can hold.");
    return false;
  }
  port_handler_ = sdk_->portHandler(port_name_);

  baud_rate_ = baud_rate;

  // Get package path
  package_path_ = ament_index_cpp::get_package_share_path("dynamixel_ros_control").string();
  if (package_path_.empty()) {
    DXL_LOG_FATAL("Could not find own package path.");
    return false;
  }

  // Get model number to series mapping
  if (!loadSeriesMapping()) {
    DXL_LOG_FATAL("Failed to load model number to series mapping.");
    return false;
  }

  packet_handler_ = sdk_->packetHandler();
  return true;
}

std::optional<int> DynamixelDriver::usbLatencyTimerMs(const std::filesystem::path& sysfs_root) const
{
  // port_name is usually a udev symlink, and sysfs knows the adapter only by its tty name
  std::error_code error;
  const std::filesystem::path port = std::filesystem::canonical(port_name_, error);
  if (error) {
    return std::nullopt;
  }
  const std::filesystem::path path = sysfs_root / "bus/usb-serial/devices" / port.filename() / "latency_timer";
  std::ifstream file(path);
  if (!file) {
    return std::nullopt;
  }
  int latency_ms = 0;
  if (!(file >> latency_ms)) {
    DXL_LOG_WARN("Could not read a latency timer from " << path << ".");
    return std::nullopt;
  }
  return latency_ms;
}

bool DynamixelDriver::connect()
{
  if (!connectPort()) {
    return false;
  }
  return setBaudRate(baud_rate_);
}

bool DynamixelDriver::loadSeriesMapping()
{
  const std::string path = package_path_ + "/devices/model_list.yaml";
  YAML::Node config;
  try {
    config = YAML::LoadFile(path);
  }
  catch (const YAML::BadFile&) {
    DXL_LOG_ERROR("Failed to read series mapping at '" << path << "'. Does the file exist?");
    return false;
  }
  catch (const YAML::Exception& e) {
    // The per-entry guard below only covers node conversion. A syntax error stops the parse before
    // any entry exists, and that throws out through on_init as a plugin that would not load.
    DXL_LOG_ERROR("Failed to parse series mapping at '" << path << "': " << e.what());
    return false;
  }
  if (!config.IsMap()) {
    DXL_LOG_ERROR("model_list.yaml is not a map (wrong format).");
    return false;
  }
  for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
    // Per entry, not around the loop: a single malformed line names itself and is skipped, rather
    // than the conversion throwing YAML::TypedBadConversion out through on_init.
    try {
      model_number_to_series_.emplace(it->first.as<uint16_t>(), it->second.as<std::string>());
    }
    catch (const YAML::Exception& e) {
      DXL_LOG_ERROR("Skipping malformed entry in model_list.yaml: " << e.what());
    }
  }
  if (model_number_to_series_.empty()) {
    DXL_LOG_ERROR("model_list.yaml yielded no usable model-to-series entries.");
    return false;
  }
  return true;
}

ControlTable* DynamixelDriver::readControlTable(std::string series)
{
  // Emplace empty table first, then load into it (avoids copy)
  auto [entry, inserted] = series_to_control_table_.try_emplace(series);
  if (!inserted) {
    // Already existed. Return the cached pointer.
    return &entry->second;
  }

  const std::string path = package_path_ + "/devices/models/" + series + ".yaml";
  // model_list.yaml names more series than this package ships tables for, so a missing file means
  // "this series is not supported yet", not a broken installation. Say which, because the two need
  // very different things from the reader.
  if (!std::filesystem::exists(path)) {
    DXL_LOG_ERROR("The '" << series << "' series is not supported yet: no control table ships at '" << path
                          << "'. Adding one is usually straightforward - copy the model's control table from "
                             "https://emanual.robotis.com into a new devices/models/"
                          << series
                          << ".yaml, following an existing file for the column layout "
                             "(address | name | length | access | memory | unit) and the unit_conversions and "
                             "indirect_addresses blocks.");
    series_to_control_table_.erase(entry);
    return nullptr;
  }
  if (!entry->second.loadFromYaml(path)) {
    DXL_LOG_ERROR("Failed to read the control table for the '" << series << "' series from '" << path << "'.");
    series_to_control_table_.erase(entry);
    return nullptr;
  }
  return &entry->second;
}

ControlTable* DynamixelDriver::loadControlTable(const uint16_t model_number)
{
  // Look up series name for model number
  if (auto it = model_number_to_series_.find(model_number); it != model_number_to_series_.end()) {
    // readControlTable() returns the cached table when one is already loaded.
    return readControlTable(it->second);
  }

  DXL_LOG_FATAL("Could not find series of model number " << model_number);
  return nullptr;
}

bool DynamixelDriver::ping(const uint8_t id, uint16_t& model_number, int& comm_result_out) const
{
  uint8_t error = 0;
  const Stopwatch timer;
  comm_result_out = packet_handler_->ping(*port_handler_, id, &model_number, &error);
  comm_stats_.bus_time_ms += timer.elapsedMs();
  comm_stats_.pings++;
  noteTraffic();
  if (comm_result_out != COMM_SUCCESS) {
    comm_stats_.ping_failures++;
  }
  return comm_result_out == COMM_SUCCESS;
}

bool DynamixelDriver::reboot(const uint8_t id) const
{
  uint8_t error = 0;
  DXL_LOG_DEBUG("[REBOOT] id " << static_cast<unsigned int>(id));
  int comm_result = packet_handler_->reboot(*port_handler_, id, &error);
  if (comm_result != COMM_SUCCESS) {
    DXL_LOG_ERROR("[ID " << static_cast<int>(id)
                         << "] Communication error while rebooting: " << communicationErrorToString(comm_result));
  }
  if (error != 0) {
    // Not a failure: the reboot was acknowledged, and the status byte carries the motor's alert
    // bit. Recovery only ever reboots motors that report a hardware error, so calling this
    // "Failed to reboot" would put a false failure in the log of every recovery that worked. What
    // it does mean is that the motor is still flagging a fault, and the read-back afterwards
    // settles that.
    DXL_LOG_WARN("[ID " << static_cast<int>(id) << "] Reboot acknowledged with a status flag set: "
                        << packetErrorToString(error) << " (error code " << static_cast<int>(error) << ").");
  }
  return comm_result == COMM_SUCCESS;
}

RegisterAccessResult DynamixelDriver::writeRegister(const uint8_t id, const uint16_t address, const uint8_t data_length,
                                                    int32_t value) const
{
  DXL_LOG_DEBUG("[Register Write] id " << static_cast<unsigned int>(id) << ", address: " << address << ", length: "
                                       << static_cast<unsigned int>(data_length) << ", value: " << value);
  uint8_t error = 0;
  int comm_result = COMM_TX_FAIL;

  const Stopwatch timer;
  if (data_length == 1) {
    auto value_8bit = static_cast<uint8_t>(value & 0xFF);
    comm_result = packet_handler_->write1ByteTxRx(*port_handler_, id, address, value_8bit, &error);
  } else if (data_length == 2) {
    auto value_16bit = static_cast<uint16_t>(value & 0xFFFF);
    comm_result = packet_handler_->write2ByteTxRx(*port_handler_, id, address, value_16bit, &error);
  } else if (data_length == 4) {
    uint32_t unsigned_value;
    std::memcpy(&unsigned_value, &value, sizeof(uint32_t));
    comm_result = packet_handler_->write4ByteTxRx(*port_handler_, id, address, unsigned_value, &error);
  }
  comm_stats_.bus_time_ms += timer.elapsedMs();
  comm_stats_.writes++;
  noteTraffic();

  const RegisterAccessResult result{comm_result, error};
  if (!result) {
    comm_stats_.write_failures++;
  }
  return result;
}

RegisterAccessResult DynamixelDriver::readRegister(const uint8_t id, const uint16_t address, const uint8_t data_length,
                                                   int32_t& value_out, const bool is_signed) const
{
  uint8_t error = 0;
  int comm_result = COMM_RX_FAIL;

  value_out = 0;

  const Stopwatch timer;
  // Zero-initialised because the SDK leaves the destination untouched when the read fails, and
  // value_out is assigned from it either way.
  // A 1- or 2-byte register is only sign-extended when it is classified two's complement, which
  // registerEncodings() decides. Extending an unsigned one turns a raw 255 into -1, which is the
  // value the caller then sees.
  if (data_length == 1) {
    uint8_t data = 0;
    comm_result = packet_handler_->read1ByteTxRx(*port_handler_, id, address, &data, &error);
    value_out = is_signed ? static_cast<int8_t>(data) : static_cast<int32_t>(data);
  } else if (data_length == 2) {
    uint16_t data = 0;
    comm_result = packet_handler_->read2ByteTxRx(*port_handler_, id, address, &data, &error);
    value_out = is_signed ? static_cast<int16_t>(data) : static_cast<int32_t>(data);
  } else if (data_length == 4) {
    uint32_t data = 0;
    comm_result = packet_handler_->read4ByteTxRx(*port_handler_, id, address, &data, &error);
    std::memcpy(&value_out, &data, sizeof(int32_t));
  } else {
    DXL_LOG_ERROR("Unsupported data length: " << static_cast<int>(data_length));
    return {COMM_RX_FAIL, 0};
  }
  comm_stats_.bus_time_ms += timer.elapsedMs();
  comm_stats_.reads++;
  noteTraffic();

  DXL_LOG_DEBUG("[Register Read] id " << static_cast<unsigned int>(id) << ", address: " << address << ", length: "
                                      << static_cast<unsigned int>(data_length) << ", value: " << value_out);

  const RegisterAccessResult result{comm_result, error};
  if (!result) {
    comm_stats_.read_failures++;
  }
  return result;
}

std::shared_ptr<GroupSyncWrite> DynamixelDriver::setSyncWrite(uint16_t address, uint16_t data_length) const
{
  return sdk_->syncWrite(port_handler_, packet_handler_, address, data_length);
}

std::shared_ptr<GroupSyncRead> DynamixelDriver::setSyncRead(uint16_t address, uint16_t data_length) const
{
  return sdk_->syncRead(port_handler_, packet_handler_, address, data_length);
}

unsigned int DynamixelDriver::requestIndirectAddresses(const unsigned int data_length)
{
  const unsigned int address_start_index = next_indirect_address_index_;
  next_indirect_address_index_ += data_length;
  DXL_LOG_DEBUG("[INDIRECT ADDRESS] Reserved indirect address index " << address_start_index << " with length "
                                                                      << data_length);
  return address_start_index;
}

std::string DynamixelDriver::communicationErrorToString(const int comm_result) const
{
  const char* error_cstr = packet_handler_->getTxRxResult(comm_result);
  return {error_cstr};
}

std::string DynamixelDriver::packetErrorToString(const uint8_t error) const
{
  const char* error_cstr = packet_handler_->getRxPacketError(error);
  return {error_cstr};
}

std::string DynamixelDriver::describeError(const RegisterAccessResult& result) const
{
  if (result.comm_result != COMM_SUCCESS) {
    return "comm error: " + communicationErrorToString(result.comm_result);
  }
  if (result.packet_error != 0) {
    return "packet error: " + packetErrorToString(result.packet_error);
  }
  return {};
}

bool DynamixelDriver::connectPort()
{
  if (!port_handler_->openPort()) {
    DXL_LOG_ERROR("Failed to open port '" << port_handler_->getPortName() << "'.");
    return false;
  }

  DXL_LOG_INFO("Succeeded to open port '" << port_handler_->getPortName() << "'.");
  next_indirect_address_index_ = 0;
  return true;
}

bool DynamixelDriver::setBaudRate(const int baud_rate) const
{
  if (!port_handler_->setBaudRate(baud_rate)) {
    DXL_LOG_ERROR("Failed to change the baudrate to " << baud_rate);
    return false;
  }

  DXL_LOG_INFO("Succeeded to change the baudrate to " << port_handler_->getBaudRate());
  return true;
}

void DynamixelDriver::noteTraffic() const
{
  const auto now = std::chrono::steady_clock::now();
  if (last_traffic_ != std::chrono::steady_clock::time_point{}) {
    longest_silence_ms_ =
        std::max(longest_silence_ms_, std::chrono::duration<double, std::milli>(now - last_traffic_).count());
  }
  last_traffic_ = now;
}

double DynamixelDriver::takeLongestSilenceMs()
{
  const double longest = longest_silence_ms_;
  longest_silence_ms_ = 0.0;
  return longest;
}

}  // namespace dynamixel_ros_control
