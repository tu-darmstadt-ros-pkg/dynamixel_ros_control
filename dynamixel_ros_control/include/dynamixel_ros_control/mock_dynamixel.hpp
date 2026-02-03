#ifndef DYNAMIXEL_ROS_CONTROL_MOCK_DYNAMIXEL_HPP
#define DYNAMIXEL_ROS_CONTROL_MOCK_DYNAMIXEL_HPP

#include "dynamixel_ros_control/sdk_wrapper.hpp"
#include "dynamixel_ros_control/log.hpp"
#include <map>
#include <vector>
#include <string>
#include <iostream>
#include <chrono>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace dynamixel_ros_control {

// Operating modes (matching Dynamixel protocol 2.0)
enum class OperatingMode : uint8_t
{
  CURRENT_CONTROL = 0,
  VELOCITY_CONTROL = 1,
  POSITION_CONTROL = 3,
  EXTENDED_POSITION_CONTROL = 4,
  CURRENT_BASED_POSITION_CONTROL = 5,
  PWM_CONTROL = 16
};

// Hardware error bits
enum HardwareErrorBit : uint8_t
{
  ERROR_NONE = 0,
  ERROR_INPUT_VOLTAGE = 0x01,
  ERROR_OVERHEATING = 0x04,
  ERROR_MOTOR_ENCODER = 0x08,
  ERROR_ELECTRICAL_SHOCK = 0x10,
  ERROR_OVERLOAD = 0x20
};

struct MockRegister
{
  uint16_t address;
  uint8_t length;
  std::string access;  // "R" or "RW"
  std::string name;
};

/**
 * @brief Mock Dynamixel motor with realistic physics simulation
 */
class MockDynamixel
{
public:
  MockDynamixel(uint8_t id, uint16_t model_number)
      : id_(id), model_number_(model_number), comm_error_enabled_(false)
  {
    // Resize memory to 2048 bytes (covers all Dynamixel models)
    memory_.resize(2048, 0);

    // Write model number at address 0
    write2Byte(0, model_number);
    // Write ID at address 7
    write1Byte(7, id);
    // Write Firmware version
    write1Byte(6, 1);

    // Load control table from YAML based on model number
    loadModelSpecs(model_number);

    // Initialize physics state
    initPhysicsState();
  }

  /**
   * @brief Update motor physics simulation
   * @param dt Time step in seconds
   */
  void update(double dt)
  {
    if (dt <= 0)
      return;

    // Check if torque is enabled
    bool torque_enabled = false;
    if (torque_enable_addr_ > 0) {
      torque_enabled = read1Byte(torque_enable_addr_) == 1;
    }

    if (!torque_enabled) {
      // When torque is disabled, assume motor holds position (or drifts - we keep stable)
      return;
    }

    // Get operating mode
    OperatingMode mode = OperatingMode::POSITION_CONTROL;
    if (operating_mode_addr_ > 0) {
      mode = static_cast<OperatingMode>(read1Byte(operating_mode_addr_));
    }

    switch (mode) {
      case OperatingMode::POSITION_CONTROL:
        // Position mode enforces position limits
        updatePositionMode(dt, true);
        break;
      case OperatingMode::EXTENDED_POSITION_CONTROL:
      case OperatingMode::CURRENT_BASED_POSITION_CONTROL:
        // Extended position mode does NOT enforce limits (multi-turn)
        updatePositionMode(dt, false);
        break;
      case OperatingMode::VELOCITY_CONTROL:
        updateVelocityMode(dt);
        break;
      case OperatingMode::CURRENT_CONTROL:
        updateCurrentMode(dt);
        break;
      default:
        updatePositionMode(dt, true);
        break;
    }

    // Update moving flag
    updateMovingFlag();

    // Write present values to registers
    writePresentValues();
  }

  void loadModelSpecs(uint16_t model_number)
  {
    std::string package_path = ament_index_cpp::get_package_share_directory("dynamixel_ros_control");
    std::string list_path = package_path + "/devices/model_list.yaml";

    try {
      YAML::Node list_node = YAML::LoadFile(list_path);
      std::string series = list_node[model_number].as<std::string>();

      std::string model_path = package_path + "/devices/models/" + series + ".yaml";
      YAML::Node model_node = YAML::LoadFile(model_path);

      // Load unit conversions
      if (model_node["unit_conversions"]) {
        auto conversions = model_node["unit_conversions"];
        if (conversions["rad"])
          rad_per_tick_ = conversions["rad"].as<double>();
        if (conversions["rad_per_s"])
          rad_per_s_per_tick_ = conversions["rad_per_s"].as<double>();
      }

      auto control_table = model_node["control_table"];
      for (const auto& item : control_table) {
        std::string line = item.as<std::string>();

        // Parse: "addr | name | len | access | memory | unit"
        std::vector<std::string> parts;
        std::stringstream ss(line);
        std::string segment;
        while (std::getline(ss, segment, '|')) {
          // Trim whitespace
          size_t start = segment.find_first_not_of(' ');
          size_t end = segment.find_last_not_of(' ');
          if (start != std::string::npos) {
            parts.push_back(segment.substr(start, end - start + 1));
          } else {
            parts.push_back("");
          }
        }

        if (parts.size() >= 3) {
          int addr = std::stoi(parts[0]);
          std::string name = parts[1];
          int len = std::stoi(parts[2]);

          address_map_[name] = addr;
          length_map_[name] = len;

          // Cache important addresses
          if (name == "torque_enable")
            torque_enable_addr_ = addr;
          else if (name == "operating_mode")
            operating_mode_addr_ = addr;
          else if (name == "goal_position")
            goal_position_addr_ = addr;
          else if (name == "goal_velocity")
            goal_velocity_addr_ = addr;
          else if (name == "goal_torque" || name == "goal_current")
            goal_current_addr_ = addr;
          else if (name == "present_position")
            present_position_addr_ = addr;
          else if (name == "present_velocity")
            present_velocity_addr_ = addr;
          else if (name == "present_current")
            present_current_addr_ = addr;
          else if (name == "velocity_limit")
            velocity_limit_addr_ = addr;
          else if (name == "acceleration_limit")
            acceleration_limit_addr_ = addr;
          else if (name == "hardware_error_status")
            hardware_error_addr_ = addr;
          else if (name == "moving")
            moving_addr_ = addr;
          else if (name == "profile_velocity")
            profile_velocity_addr_ = addr;
          else if (name == "profile_acceleration")
            profile_acceleration_addr_ = addr;
          else if (name == "max_position_limit")
            max_position_limit_addr_ = addr;
          else if (name == "min_position_limit")
            min_position_limit_addr_ = addr;
          else if (name == "homing_offset")
            homing_offset_addr_ = addr;
          else if (name == "led_red")
            led_red_addr_ = addr;
          else if (name == "led_green")
            led_green_addr_ = addr;
          else if (name == "led_blue")
            led_blue_addr_ = addr;
        }
      }

      // Load indirect addresses
      if (model_node["indirect_addresses"]) {
        auto indirect = model_node["indirect_addresses"];
        if (indirect.size() > 0) {
          std::string line = indirect[0].as<std::string>();
          std::stringstream ss(line);
          std::string segment;
          std::vector<std::string> parts;
          while (std::getline(ss, segment, '|')) {
            size_t start = segment.find_first_not_of(' ');
            size_t end = segment.find_last_not_of(' ');
            if (start != std::string::npos)
              parts.push_back(segment.substr(start, end - start + 1));
            else
              parts.push_back("");
          }
          if (parts.size() >= 3) {
            indirect_address_start_ = std::stoi(parts[0]);
            indirect_data_start_ = std::stoi(parts[1]);
            indirect_count_ = std::stoi(parts[2]);
          }
        }
      }
    }
    catch (const std::exception& e) {
      std::cerr << "Failed to load mock specs for model " << model_number << ": " << e.what() << std::endl;
    }
  }

  // Memory access methods
  uint8_t read1Byte(uint16_t address) const
  {
    uint16_t real_addr = resolveAddress(address);
    if (static_cast<size_t>(real_addr) >= memory_.size())
      return 0;
    return memory_[real_addr];
  }

  uint16_t read2Byte(uint16_t address) const
  {
    return static_cast<uint16_t>(read1Byte(address)) | (static_cast<uint16_t>(read1Byte(address + 1)) << 8);
  }

  uint32_t read4Byte(uint16_t address) const
  {
    return static_cast<uint32_t>(read1Byte(address)) | (static_cast<uint32_t>(read1Byte(address + 1)) << 8) |
           (static_cast<uint32_t>(read1Byte(address + 2)) << 16) |
           (static_cast<uint32_t>(read1Byte(address + 3)) << 24);
  }

  int32_t read4ByteSigned(uint16_t address) const
  {
    return static_cast<int32_t>(read4Byte(address));
  }

  void write1Byte(uint16_t address, uint8_t data)
  {
    uint16_t real_addr = resolveAddress(address);
    if (static_cast<size_t>(real_addr) < memory_.size())
      memory_[real_addr] = data;
  }

  void write2Byte(uint16_t address, uint16_t data)
  {
    write1Byte(address, data & 0xFF);
    write1Byte(address + 1, (data >> 8) & 0xFF);
  }

  void write4Byte(uint16_t address, uint32_t data)
  {
    write1Byte(address, data & 0xFF);
    write1Byte(address + 1, (data >> 8) & 0xFF);
    write1Byte(address + 2, (data >> 16) & 0xFF);
    write1Byte(address + 3, (data >> 24) & 0xFF);
  }

  uint8_t getId() const
  {
    return id_;
  }
  uint16_t getModelNumber() const
  {
    return model_number_;
  }

  // Error injection
  void setHardwareError(uint8_t error_code)
  {
    hardware_error_ = error_code;
    if (hardware_error_addr_ > 0) {
      write1Byte(hardware_error_addr_, error_code);
    }
  }

  void clearHardwareError()
  {
    setHardwareError(0);
  }

  uint8_t getHardwareError() const
  {
    return hardware_error_;
  }

  // Reboot tracking
  int getRebootCount() const
  {
    return reboot_count_;
  }

  void incrementRebootCount()
  {
    reboot_count_++;
  }

  void resetRebootCount()
  {
    reboot_count_ = 0;
  }

  void setCommunicationError(bool enabled)
  {
    comm_error_enabled_ = enabled;
  }

  bool hasCommunicationError() const
  {
    return comm_error_enabled_;
  }

  // Address lookup
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

  // Physics state access (for testing)
  // Returns the actuator position (internal position without homing offset)
  double getCurrentPosition() const
  {
    return current_position_;
  }
  // Returns the joint position (actuator position + homing offset, as reported by Present_Position register)
  double getJointPosition() const
  {
    int32_t homing_offset_ticks = (homing_offset_addr_ > 0) ? read4ByteSigned(homing_offset_addr_) : 0;
    double homing_offset_rad = static_cast<double>(homing_offset_ticks) * rad_per_tick_;
    return current_position_ + homing_offset_rad;
  }
  double getCurrentVelocity() const
  {
    return current_velocity_;
  }
  double getCurrentAcceleration() const
  {
    return current_acceleration_;
  }

  // LED state accessors (for testing)
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

  // Homing offset accessor
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
  void initPhysicsState()
  {
    current_position_ = 0.0;
    current_velocity_ = 0.0;
    current_acceleration_ = 0.0;
    current_current_ = 0.0;

    // Default limits (in ticks, will be converted)
    default_velocity_limit_ = 100.0;     // rad/s
    default_acceleration_limit_ = 50.0;  // rad/s^2
  }

  double getVelocityLimit() const
  {
    // For velocity mode: use velocity_limit register
    if (velocity_limit_addr_ > 0) {
      int32_t limit_ticks = read4ByteSigned(velocity_limit_addr_);
      if (limit_ticks > 0) {
        return static_cast<double>(limit_ticks) * rad_per_s_per_tick_;
      }
    }
    return default_velocity_limit_;
  }

  // For position mode: profile_velocity is the dynamic velocity limit
  double getPositionModeVelocityLimit() const
  {
    // First check profile_velocity (dynamic limit during position moves)
    if (profile_velocity_addr_ > 0) {
      int32_t profile_vel = read4ByteSigned(profile_velocity_addr_);
      if (profile_vel > 0) {
        return static_cast<double>(profile_vel) * rad_per_s_per_tick_;
      }
    }
    // Fall back to velocity_limit register
    if (velocity_limit_addr_ > 0) {
      int32_t limit_ticks = read4ByteSigned(velocity_limit_addr_);
      if (limit_ticks > 0) {
        return static_cast<double>(limit_ticks) * rad_per_s_per_tick_;
      }
    }
    return default_velocity_limit_;
  }

  // Get position limits for Position mode enforcement
  std::pair<double, double> getPositionLimits() const
  {
    double min_pos = -1e9;
    double max_pos = 1e9;

    if (min_position_limit_addr_ > 0) {
      int32_t min_ticks = read4ByteSigned(min_position_limit_addr_);
      min_pos = static_cast<double>(min_ticks) * rad_per_tick_;
    }
    if (max_position_limit_addr_ > 0) {
      int32_t max_ticks = read4ByteSigned(max_position_limit_addr_);
      max_pos = static_cast<double>(max_ticks) * rad_per_tick_;
    }
    return {min_pos, max_pos};
  }

  double getAccelerationLimit() const
  {
    if (acceleration_limit_addr_ > 0) {
      int32_t limit_ticks = read4ByteSigned(acceleration_limit_addr_);
      if (limit_ticks > 0) {
        return static_cast<double>(limit_ticks);  // Units vary by model
      }
    }
    if (profile_acceleration_addr_ > 0) {
      int32_t profile_acc = read4ByteSigned(profile_acceleration_addr_);
      if (profile_acc > 0) {
        return static_cast<double>(profile_acc);
      }
    }
    return default_acceleration_limit_;
  }

  void updatePositionMode(double dt, bool enforce_limits = true)
  {
    if (goal_position_addr_ == 0 || present_position_addr_ == 0)
      return;

    // Get goal position in radians
    // Goal Position register includes homing offset, so we need to subtract it
    // to get the actual target position (same as present_position behavior)
    int32_t goal_ticks = read4ByteSigned(goal_position_addr_);
    int32_t homing_offset_ticks = (homing_offset_addr_ > 0) ? read4ByteSigned(homing_offset_addr_) : 0;
    double goal_pos = static_cast<double>(goal_ticks - homing_offset_ticks) * rad_per_tick_;

    // Enforce position limits only in Position mode (not Extended Position mode)
    // Only apply limits if they have been explicitly set (i.e., not both 0)
    if (enforce_limits && max_position_limit_addr_ > 0 && min_position_limit_addr_ > 0) {
      auto [min_pos, max_pos] = getPositionLimits();
      // Only clamp if limits are actually configured (not both 0 or same value)
      if (min_pos != max_pos || min_pos != 0.0) {
        goal_pos = std::clamp(goal_pos, min_pos, max_pos);
      }
    }

    // Use profile_velocity as the velocity limit for position mode (dynamic limit)
    double vel_limit = getPositionModeVelocityLimit();
    double acc_limit = getAccelerationLimit();

    // Calculate position error
    double pos_error = goal_pos - current_position_;

    // Trapezoidal velocity profile
    // Calculate stopping distance
    double stopping_distance = (current_velocity_ * current_velocity_) / (2.0 * acc_limit);

    // Desired velocity based on position
    double desired_velocity = 0.0;
    if (std::abs(pos_error) > 0.0001) {
      // Direction towards goal
      double direction = (pos_error > 0) ? 1.0 : -1.0;

      if (std::abs(pos_error) <= stopping_distance) {
        // Deceleration phase
        desired_velocity = direction * std::sqrt(2.0 * acc_limit * std::abs(pos_error));
      } else {
        // Acceleration or cruise phase
        desired_velocity = direction * vel_limit;
      }
    }

    // Apply acceleration limits to velocity change
    double vel_error = desired_velocity - current_velocity_;
    double max_vel_change = acc_limit * dt;

    if (std::abs(vel_error) > max_vel_change) {
      current_acceleration_ = (vel_error > 0 ? 1.0 : -1.0) * acc_limit;
      current_velocity_ += current_acceleration_ * dt;
    } else {
      current_velocity_ = desired_velocity;
      current_acceleration_ = vel_error / dt;
    }

    // Clamp velocity to limits
    current_velocity_ = std::clamp(current_velocity_, -vel_limit, vel_limit);

    // Update position
    current_position_ += current_velocity_ * dt;

    // Enforce position limits on actual position (for Position mode only)
    if (enforce_limits && max_position_limit_addr_ > 0 && min_position_limit_addr_ > 0) {
      auto [min_pos, max_pos] = getPositionLimits();
      // Only clamp if limits are actually configured (not both 0 or same value)
      if (min_pos != max_pos || min_pos != 0.0) {
        if (current_position_ < min_pos) {
          current_position_ = min_pos;
          current_velocity_ = 0.0;
        } else if (current_position_ > max_pos) {
          current_position_ = max_pos;
          current_velocity_ = 0.0;
        }
      }
    }

    // Snap to goal if very close
    if (std::abs(current_position_ - goal_pos) < 0.0001 && std::abs(current_velocity_) < 0.001) {
      current_position_ = goal_pos;
      current_velocity_ = 0.0;
      current_acceleration_ = 0.0;
    }

    // Simulate current draw (proportional to acceleration/torque)
    current_current_ = std::abs(current_acceleration_) * 0.1;  // Simplified model
  }

  void updateVelocityMode(double dt)
  {
    if (goal_velocity_addr_ == 0)
      return;

    // Get goal velocity
    int32_t goal_vel_ticks = read4ByteSigned(goal_velocity_addr_);
    double goal_velocity = static_cast<double>(goal_vel_ticks) * rad_per_s_per_tick_;

    double acc_limit = getAccelerationLimit();

    // Apply acceleration limits
    double vel_error = goal_velocity - current_velocity_;
    double max_vel_change = acc_limit * dt;

    if (std::abs(vel_error) > max_vel_change) {
      current_acceleration_ = (vel_error > 0 ? 1.0 : -1.0) * acc_limit;
      current_velocity_ += current_acceleration_ * dt;
    } else {
      current_velocity_ = goal_velocity;
      current_acceleration_ = 0.0;
    }

    // Update position
    current_position_ += current_velocity_ * dt;

    // Simulate current
    current_current_ = std::abs(current_acceleration_) * 0.1;
  }

  void updateCurrentMode(double dt)
  {
    if (goal_current_addr_ == 0)
      return;

    // Get goal current (simplified: treat current as proportional to acceleration)
    int16_t goal_current_ticks = static_cast<int16_t>(read2Byte(goal_current_addr_));
    current_current_ = static_cast<double>(goal_current_ticks) * 0.001;  // Scale factor

    // Simple acceleration based on current
    current_acceleration_ = current_current_ * 10.0;  // Simplified torque -> acceleration

    // Update velocity and position
    current_velocity_ += current_acceleration_ * dt;
    current_position_ += current_velocity_ * dt;
  }

  void updateMovingFlag()
  {
    if (moving_addr_ > 0) {
      bool moving = std::abs(current_velocity_) > 0.001;
      write1Byte(moving_addr_, moving ? 1 : 0);
    }
  }

  void writePresentValues()
  {
    // Calculate homing offset
    int32_t homing_offset_ticks = 0;
    if (homing_offset_addr_ > 0) {
      homing_offset_ticks = read4ByteSigned(homing_offset_addr_);
    }

    // Write present position (actual position + homing offset)
    if (present_position_addr_ > 0) {
      int32_t pos_ticks = static_cast<int32_t>(current_position_ / rad_per_tick_);
      // Present Position = Actual Position + Homing Offset (per Dynamixel documentation)
      int32_t present_ticks = pos_ticks + homing_offset_ticks;
      write4Byte(present_position_addr_, static_cast<uint32_t>(present_ticks));
    }

    // Write present velocity
    if (present_velocity_addr_ > 0) {
      int32_t vel_ticks = static_cast<int32_t>(current_velocity_ / rad_per_s_per_tick_);
      write4Byte(present_velocity_addr_, static_cast<uint32_t>(vel_ticks));
    }

    // Write present current
    if (present_current_addr_ > 0) {
      int16_t current_ticks = static_cast<int16_t>(current_current_ * 1000.0);
      write2Byte(present_current_addr_, static_cast<uint16_t>(current_ticks));
    }
  }

  uint8_t id_;
  uint16_t model_number_;
  std::vector<uint8_t> memory_;
  std::map<std::string, uint16_t> address_map_;
  std::map<std::string, uint8_t> length_map_;

  // Cached addresses
  uint16_t torque_enable_addr_ = 0;
  uint16_t operating_mode_addr_ = 0;
  uint16_t goal_position_addr_ = 0;
  uint16_t goal_velocity_addr_ = 0;
  uint16_t goal_current_addr_ = 0;
  uint16_t present_position_addr_ = 0;
  uint16_t present_velocity_addr_ = 0;
  uint16_t present_current_addr_ = 0;
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

  // Unit conversions
  double rad_per_tick_ = 0.00002068538;  // Default for H42 series
  double rad_per_s_per_tick_ = 0.000344746;

  // Physics state
  double current_position_ = 0.0;
  double current_velocity_ = 0.0;
  double current_acceleration_ = 0.0;
  double current_current_ = 0.0;

  // Limits
  double default_velocity_limit_ = 100.0;
  double default_acceleration_limit_ = 50.0;

  // Error state
  uint8_t hardware_error_ = 0;
  bool comm_error_enabled_ = false;
  int reboot_count_ = 0;  // Tracks how many times this motor has been rebooted

  // Indirect addressing configuration
  uint16_t indirect_address_start_ = 0;
  uint16_t indirect_data_start_ = 0;
  uint16_t indirect_count_ = 0;

  // Helper to resolve indirect addresses
  uint16_t resolveAddress(uint16_t address) const
  {
    if (indirect_count_ > 0 && address >= indirect_data_start_ && address < indirect_data_start_ + indirect_count_) {

      uint16_t index = address - indirect_data_start_;
      uint16_t pointer_addr = indirect_address_start_ + (index * 2);

      // Pointer is 2 bytes (Low, High)
      if (static_cast<size_t>(pointer_addr + 1) < memory_.size()) {
        uint16_t target = memory_[pointer_addr] | (memory_[pointer_addr + 1] << 8);
        return target;
      }
    }
    return address;
  }
};

/**
 * @brief Manager for mock Dynamixel motors (singleton)
 */
class MockDynamixelManager
{
public:
  static MockDynamixelManager& instance()
  {
    static MockDynamixelManager instance;
    return instance;
  }

  void addMotor(uint8_t id, uint16_t model_number)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (motors_.find(id) == motors_.end()) {
      motors_.emplace(id, std::make_shared<MockDynamixel>(id, model_number));
    }
  }

  void removeMotor(uint8_t id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    motors_.erase(id);
  }

  std::shared_ptr<MockDynamixel> getMotor(uint8_t id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (motors_.find(id) != motors_.end()) {
      return motors_[id];
    }
    return nullptr;
  }

  std::vector<uint8_t> getConnectedIds()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<uint8_t> ids;
    for (auto const& [id, motor] : motors_) {
      ids.push_back(id);
    }
    return ids;
  }

  void update(double dt = 0.01)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& [id, motor] : motors_) {
      motor->update(dt);
    }
  }

  /**
   * @brief Reset manager state - removes all motors
   * Call this between tests to ensure clean state
   */
  void reset()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    motors_.clear();
  }

  /**
   * @brief Enable communication errors on all motors
   */
  void setGlobalCommunicationError(bool enabled)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& [id, motor] : motors_) {
      motor->setCommunicationError(enabled);
    }
  }

private:
  MockDynamixelManager() = default;
  std::map<uint8_t, std::shared_ptr<MockDynamixel>> motors_;
  std::mutex mutex_;
};

/**
 * @brief Mock PortHandler
 */
class MockPortHandler : public PortHandler
{
public:
  MockPortHandler(const char* port_name)
      : port_name_(port_name), is_open_(false)
  {}

  bool openPort() override
  {
    is_open_ = true;
    return true;
  }
  void closePort() override
  {
    is_open_ = false;
  }
  void clearPort() override {}
  void setPortName(const char* port_name) override
  {
    port_name_ = port_name;
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
  bool isOpen() const
  {
    return is_open_;
  }

private:
  std::string port_name_;
  int baud_rate_ = 57600;
  bool is_open_ = false;
};

/**
 * @brief Mock PacketHandler with error injection support
 */
class MockPacketHandler : public PacketHandler
{
public:
  MockPacketHandler(float protocol_version)
      : protocol_version_(protocol_version)
  {}

  int ping(std::shared_ptr<PortHandler> port, uint8_t id, uint16_t* model_number, uint8_t* error) override
  {
    (void) port;  // Unused in mock
    auto motor = MockDynamixelManager::instance().getMotor(id);
    if (!motor)
      return COMM_RX_TIMEOUT;
    if (motor->hasCommunicationError())
      return COMM_RX_TIMEOUT;

    if (model_number)
      *model_number = motor->read2Byte(0);
    if (error)
      *error = motor->getHardwareError();
    return COMM_SUCCESS;
  }

  int reboot(std::shared_ptr<PortHandler> port, uint8_t id, uint8_t* error) override
  {
    (void) port;  // Unused in mock
    auto motor = MockDynamixelManager::instance().getMotor(id);
    if (!motor)
      return COMM_RX_TIMEOUT;
    if (motor->hasCommunicationError())
      return COMM_RX_TIMEOUT;

    motor->incrementRebootCount();
    motor->clearHardwareError();
    if (error)
      *error = 0;
    return COMM_SUCCESS;
  }

  int read1ByteTxRx(std::shared_ptr<PortHandler> port, uint8_t id, uint16_t address, uint8_t* data,
                    uint8_t* error) override
  {
    (void) port;  // Unused in mock
    auto motor = MockDynamixelManager::instance().getMotor(id);
    if (!motor)
      return COMM_RX_TIMEOUT;
    if (motor->hasCommunicationError())
      return COMM_RX_TIMEOUT;

    if (data)
      *data = motor->read1Byte(address);
    if (error)
      *error = motor->getHardwareError();
    return COMM_SUCCESS;
  }

  int read2ByteTxRx(std::shared_ptr<PortHandler> port, uint8_t id, uint16_t address, uint16_t* data,
                    uint8_t* error) override
  {
    (void) port;  // Unused in mock
    auto motor = MockDynamixelManager::instance().getMotor(id);
    if (!motor)
      return COMM_RX_TIMEOUT;
    if (motor->hasCommunicationError())
      return COMM_RX_TIMEOUT;

    if (data)
      *data = motor->read2Byte(address);
    if (error)
      *error = motor->getHardwareError();
    return COMM_SUCCESS;
  }

  int read4ByteTxRx(std::shared_ptr<PortHandler> port, uint8_t id, uint16_t address, uint32_t* data,
                    uint8_t* error) override
  {
    (void) port;  // Unused in mock
    auto motor = MockDynamixelManager::instance().getMotor(id);
    if (!motor)
      return COMM_RX_TIMEOUT;
    if (motor->hasCommunicationError())
      return COMM_RX_TIMEOUT;

    if (data)
      *data = motor->read4Byte(address);
    if (error)
      *error = motor->getHardwareError();
    return COMM_SUCCESS;
  }

  int write1ByteTxRx(std::shared_ptr<PortHandler> port, uint8_t id, uint16_t address, uint8_t data,
                     uint8_t* error) override
  {
    (void) port;  // Unused in mock
    auto motor = MockDynamixelManager::instance().getMotor(id);
    if (!motor)
      return COMM_RX_TIMEOUT;
    if (motor->hasCommunicationError())
      return COMM_RX_TIMEOUT;

    motor->write1Byte(address, data);
    if (error)
      *error = motor->getHardwareError();
    return COMM_SUCCESS;
  }

  int write2ByteTxRx(std::shared_ptr<PortHandler> port, uint8_t id, uint16_t address, uint16_t data,
                     uint8_t* error) override
  {
    (void) port;  // Unused in mock
    auto motor = MockDynamixelManager::instance().getMotor(id);
    if (!motor)
      return COMM_RX_TIMEOUT;
    if (motor->hasCommunicationError())
      return COMM_RX_TIMEOUT;

    motor->write2Byte(address, data);
    if (error)
      *error = motor->getHardwareError();
    return COMM_SUCCESS;
  }

  int write4ByteTxRx(std::shared_ptr<PortHandler> port, uint8_t id, uint16_t address, uint32_t data,
                     uint8_t* error) override
  {
    (void) port;  // Unused in mock
    auto motor = MockDynamixelManager::instance().getMotor(id);
    if (!motor)
      return COMM_RX_TIMEOUT;
    if (motor->hasCommunicationError())
      return COMM_RX_TIMEOUT;

    motor->write4Byte(address, data);
    if (error)
      *error = motor->getHardwareError();
    return COMM_SUCCESS;
  }

  const char* getTxRxResult(int result) override
  {
    switch (result) {
      case COMM_SUCCESS:
        return "COMM_SUCCESS";
      case COMM_TX_FAIL:
        return "COMM_TX_FAIL";
      case COMM_RX_FAIL:
        return "COMM_RX_FAIL";
      case COMM_TX_ERROR:
        return "COMM_TX_ERROR";
      case COMM_RX_TIMEOUT:
        return "COMM_RX_TIMEOUT";
      default:
        return "COMM_ERROR";
    }
  }

  const char* getRxPacketError(uint8_t error) override
  {
    if (error == 0)
      return "";
    if (error & ERROR_OVERHEATING)
      return "Overheating";
    if (error & ERROR_OVERLOAD)
      return "Overload";
    return "Unknown Error";
  }

private:
  float protocol_version_;
};

/**
 * @brief Mock GroupSyncRead with multi-register support
 */
class MockGroupSyncRead : public GroupSyncRead
{
public:
  MockGroupSyncRead(std::shared_ptr<PortHandler> port, std::shared_ptr<PacketHandler> ph, uint16_t start_address,
                    uint16_t data_length)
      : start_address_(start_address), data_length_(data_length)
  {
    (void) port;  // Unused in mock
    (void) ph;    // Unused in mock
  }

  bool addParam(uint8_t id) override
  {
    ids_.push_back(id);
    return true;
  }

  void removeParam(uint8_t id) override
  {
    ids_.erase(std::remove(ids_.begin(), ids_.end(), id), ids_.end());
  }

  void clearParam() override
  {
    ids_.clear();
  }

  int txRxPacket() override
  {
    data_map_.clear();
    available_map_.clear();

    for (uint8_t id : ids_) {
      auto motor = MockDynamixelManager::instance().getMotor(id);
      if (motor && !motor->hasCommunicationError()) {
        // Read the entire range
        std::vector<uint8_t> data(data_length_);
        for (uint16_t i = 0; i < data_length_; ++i) {
          data[i] = motor->read1Byte(start_address_ + i);
        }
        data_map_[id] = data;
        available_map_[id] = true;
      } else {
        available_map_[id] = false;
      }
    }
    return COMM_SUCCESS;
  }

  bool isAvailable(uint8_t id, uint16_t address, uint16_t data_length) override
  {
    if (!available_map_[id])
      return false;
    if (address < start_address_)
      return false;
    if (address + data_length > start_address_ + data_length_)
      return false;
    return true;
  }

  uint32_t getData(uint8_t id, uint16_t address, uint16_t data_length) override
  {
    if (!isAvailable(id, address, data_length))
      return 0;

    const auto& data = data_map_[id];
    uint16_t offset = address - start_address_;

    uint32_t val = 0;
    for (uint16_t i = 0; i < data_length && i < 4; ++i) {
      val |= (data[offset + i] << (8 * i));
    }
    return val;
  }

private:
  uint16_t start_address_;
  uint16_t data_length_;
  std::vector<uint8_t> ids_;
  std::map<uint8_t, std::vector<uint8_t>> data_map_;
  std::map<uint8_t, bool> available_map_;
};

/**
 * @brief Mock GroupSyncWrite
 */
class MockGroupSyncWrite : public GroupSyncWrite
{
public:
  MockGroupSyncWrite(std::shared_ptr<PortHandler> port, std::shared_ptr<PacketHandler> ph, uint16_t start_address,
                     uint16_t data_length)
      : start_address_(start_address), data_length_(data_length)
  {
    (void) port;  // Unused in mock
    (void) ph;    // Unused in mock
  }

  bool addParam(uint8_t id, uint8_t* data) override
  {
    std::vector<uint8_t> d;
    d.assign(data, data + data_length_);
    params_[id] = d;
    return true;
  }

  void removeParam(uint8_t id) override
  {
    params_.erase(id);
  }

  bool changeParam(uint8_t id, uint8_t* data) override
  {
    if (params_.find(id) == params_.end())
      return false;
    std::vector<uint8_t> d;
    d.assign(data, data + data_length_);
    params_[id] = d;
    return true;
  }

  void clearParam() override
  {
    params_.clear();
  }

  int txPacket() override
  {
    for (auto const& [id, data] : params_) {
      auto motor = MockDynamixelManager::instance().getMotor(id);
      if (motor && !motor->hasCommunicationError()) {
        // Write each byte
        for (size_t i = 0; i < data.size(); ++i) {
          motor->write1Byte(start_address_ + i, data[i]);
        }
      }
    }
    return COMM_SUCCESS;
  }

private:
  uint16_t start_address_;
  uint16_t data_length_;
  std::map<uint8_t, std::vector<uint8_t>> params_;
};

}  // namespace dynamixel_ros_control

#endif  // DYNAMIXEL_ROS_CONTROL_MOCK_DYNAMIXEL_HPP
