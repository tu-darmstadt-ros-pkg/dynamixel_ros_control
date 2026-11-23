#include "dynamixel_ros_control/mock_bus_server.hpp"

#include <poll.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <system_error>

namespace dynamixel_ros_control {

namespace protocol2 {

namespace {
constexpr uint8_t HEADER[] = {0xFF, 0xFF, 0xFD};

bool endsWithHeader(const std::vector<uint8_t>& bytes)
{
  return bytes.size() >= 3 && std::equal(std::begin(HEADER), std::end(HEADER), bytes.end() - 3);
}
}  // namespace

uint16_t crc(const uint8_t* data, const size_t length)
{
  uint16_t crc = 0;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i] << 8);
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) != 0 ? static_cast<uint16_t>((crc << 1) ^ 0x8005) : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

std::vector<uint8_t> stuff(const std::vector<uint8_t>& body)
{
  std::vector<uint8_t> out;
  for (const uint8_t byte : body) {
    out.push_back(byte);
    if (endsWithHeader(out)) {
      out.push_back(0xFD);
    }
  }
  return out;
}

std::vector<uint8_t> unstuff(const std::vector<uint8_t>& body)
{
  std::vector<uint8_t> out;
  for (size_t i = 0; i < body.size(); ++i) {
    out.push_back(body[i]);
    if (endsWithHeader(out) && i + 1 < body.size() && body[i + 1] == 0xFD) {
      ++i;
    }
  }
  return out;
}

std::vector<uint8_t> statusPacket(const uint8_t id, const uint8_t error, const std::vector<uint8_t>& params)
{
  std::vector<uint8_t> body{INST_STATUS, error};
  body.insert(body.end(), params.begin(), params.end());
  body = stuff(body);
  const size_t length = body.size() + 2;
  std::vector<uint8_t> packet(std::begin(HEADER), std::end(HEADER));
  packet.insert(packet.end(), {0x00, id, static_cast<uint8_t>(length & 0xFF), static_cast<uint8_t>(length >> 8)});
  packet.insert(packet.end(), body.begin(), body.end());
  const uint16_t checksum = crc(packet.data(), packet.size());
  packet.push_back(static_cast<uint8_t>(checksum & 0xFF));
  packet.push_back(static_cast<uint8_t>(checksum >> 8));
  return packet;
}

}  // namespace protocol2

namespace {

constexpr size_t HEADER_SIZE = 7;  // FF FF FD 00, id, length

uint16_t word(const std::vector<uint8_t>& bytes, const size_t at)
{
  return static_cast<uint16_t>(bytes[at] | (bytes[at + 1] << 8));
}

[[noreturn]] void throwErrno(const std::string& what)
{
  throw std::system_error(errno, std::generic_category(), what);
}

}  // namespace

UniqueFd::~UniqueFd()
{
  reset(-1);
}

void UniqueFd::reset(const int fd)
{
  if (fd_ >= 0) {
    close(fd_);
  }
  fd_ = fd;
}

MockBusServer::MockBusServer(const std::vector<uint8_t>& ids, std::string link)
    : ids_(ids.begin(), ids.end()), link_(std::move(link))
{
  if (!link_.empty() && std::filesystem::exists(std::filesystem::symlink_status(link_)) &&
      !std::filesystem::is_symlink(link_)) {
    throw std::runtime_error("Refusing to replace '" + link_ + "', it is no symlink.");
  }
  char name[128];
  int master_fd = -1;
  int slave_fd = -1;
  if (openpty(&master_fd, &slave_fd, name, nullptr, nullptr) != 0) {
    throwErrno("openpty");
  }
  master_fd_.reset(master_fd);
  slave_fd_.reset(slave_fd);
  pty_name_ = name;
  // A fresh pty echoes and translates line endings until someone sets it raw
  termios attributes{};
  if (tcgetattr(slave_fd_.get(), &attributes) != 0) {
    throwErrno("tcgetattr");
  }
  cfmakeraw(&attributes);
  if (tcsetattr(slave_fd_.get(), TCSANOW, &attributes) != 0) {
    throwErrno("tcsetattr");
  }
  if (!link_.empty()) {
    std::filesystem::remove(link_);
    std::filesystem::create_symlink(pty_name_, link_);
  }
  thread_ = std::thread(&MockBusServer::serve, this);
}

MockBusServer::~MockBusServer()
{
  stop_ = true;
  thread_.join();
  if (!link_.empty()) {
    std::filesystem::remove(link_);
  }
}

void MockBusServer::serve()
{
  // An exception escaping the thread would terminate the process
  try {
    serveUntilStopped();
  }
  catch (const std::exception& error) {
    std::cerr << "MockBusServer on " << portName() << " stopped serving: " << error.what() << std::endl;
    failed_ = true;
  }
}

void MockBusServer::serveUntilStopped()
{
  std::vector<uint8_t> buffer;
  uint8_t chunk[512];
  while (!stop_) {
    pollfd poll_fd{master_fd_.get(), POLLIN, 0};
    if (poll(&poll_fd, 1, 10) <= 0) {
      continue;
    }
    const std::lock_guard<std::mutex> lock(serve_mutex_);
    const ssize_t count = read(master_fd_.get(), chunk, sizeof(chunk));
    if (count < 0 && errno != EINTR && errno != EAGAIN) {
      throwErrno("Reading the pty master");
    }
    if (count > 0) {
      buffer.insert(buffer.end(), chunk, chunk + count);
    }
    while (true) {
      static const uint8_t PREFIX[] = {0xFF, 0xFF, 0xFD, 0x00};  // the header and its reserved byte
      const auto start = std::search(buffer.begin(), buffer.end(), std::begin(PREFIX), std::end(PREFIX));
      // A partial header may still be completed by the next read
      buffer.erase(buffer.begin(), start == buffer.end() ? buffer.end() - std::min<size_t>(buffer.size(), 3) : start);
      if (buffer.size() < HEADER_SIZE) {
        break;
      }
      const size_t total = HEADER_SIZE + word(buffer, 5);
      if (buffer.size() < total) {
        break;
      }
      const std::vector<uint8_t> packet(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(total));
      buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(total));
      // A motor ignores a damaged instruction
      if (total < HEADER_SIZE + 3 || protocol2::crc(packet.data(), total - 2) != word(packet, total - 2)) {
        continue;
      }
      const auto body = protocol2::unstuff({packet.begin() + HEADER_SIZE, packet.end() - 2});
      // Too late for the instruction it answered, just in time to be taken for this one's answer
      if (body[0] != INST_SYNC_WRITE) {
        send(delayed_);
        delayed_.clear();
      }
      handle(packet[4], body[0], {body.begin() + 1, body.end()});
    }
  }
}

void MockBusServer::handle(const uint8_t id, const uint8_t instruction, const std::vector<uint8_t>& params)
{
  const uint16_t address = params.size() >= 2 ? word(params, 0) : 0;
  if (id == BROADCAST_ID && (instruction == INST_SYNC_READ || instruction == INST_SYNC_WRITE) && params.size() >= 4) {
    syncInstruction(instruction, address, word(params, 2), params);
    return;
  }
  // Another bus segment's motor, or nobody. Broadcast ping, write and reboot are left out because
  // the plugin addresses those by id
  const auto motor = ids_.count(id) != 0 ? MockDynamixelManager::instance().getMotor(id) : nullptr;
  if (!motor) {
    return;
  }
  const size_t length = params.size() > 2 ? params.size() - 2 : 0;
  const uint16_t read_length = params.size() == 4 ? word(params, 2) : 0;
  std::optional<MockStatus> status;
  if (instruction == INST_PING) {
    status = motor->answerPing();
  } else if (instruction == INST_REBOOT) {
    status = motor->answerReboot();
  } else if (instruction == INST_READ && (read_length == 1 || read_length == 2 || read_length == 4)) {
    status = motor->answerRead(address, read_length);
  } else if (instruction == INST_WRITE && (length == 1 || length == 2 || length == 4)) {
    status = motor->answerWrite(address, &params[2], static_cast<uint16_t>(length));
  } else if (motor->respondsToStatus()) {
    // The firmware rules cover only what the plugin sends, so anything else is refused loudly
    status = MockStatus{instruction == INST_WRITE || instruction == INST_READ ? PACKET_ERROR_DATA_LENGTH :
                                                                                PACKET_ERROR_INSTRUCTION,
                        {}};
  }
  if (status) {
    reply(id, status->error, status->params);
  }
}

// A sync read gets one status packet per motor in the order asked, a sync write none
void MockBusServer::syncInstruction(const uint8_t instruction, const uint16_t address, const uint16_t length,
                                    const std::vector<uint8_t>& params)
{
  const bool read = instruction == INST_SYNC_READ;
  const size_t stride = read ? 1 : 1 + length;
  for (size_t at = 4; at + stride <= params.size(); at += stride) {
    const uint8_t id = params[at];
    const auto motor = ids_.count(id) != 0 ? MockDynamixelManager::instance().getMotor(id) : nullptr;
    if (!motor) {
      continue;
    }
    if (!read) {
      (void) motor->answerWrite(address, &params[at + 1], length);
    } else if (const auto status = motor->answerRead(address, length)) {
      reply(id, status->error, status->params);
    }
  }
}

void MockBusServer::waitUntilIdle()
{
  while (true) {
    {
      const std::lock_guard<std::mutex> lock(serve_mutex_);
      // poll() on a tty first flushes bytes the kernel still queues between the two ends
      pollfd poll_fd{master_fd_.get(), POLLIN, 0};
      if (failed_ || poll(&poll_fd, 1, 0) == 0) {
        return;
      }
    }
    std::this_thread::yield();
  }
}

void MockBusServer::injectWireFault(const WireFault fault, const int count)
{
  if (count == 0) {
    return;
  }
  const std::lock_guard<std::mutex> lock(fault_mutex_);
  fault_ = fault;
  fault_count_ = count;
}

void MockBusServer::clearWireFault()
{
  const std::lock_guard<std::mutex> lock(fault_mutex_);
  fault_.reset();
}

std::optional<WireFault> MockBusServer::takeWireFault()
{
  const std::lock_guard<std::mutex> lock(fault_mutex_);
  const auto fault = fault_;
  if (fault_count_ > 0 && --fault_count_ == 0) {
    fault_.reset();
  }
  return fault;
}

void MockBusServer::setPower(const bool on)
{
  for (const uint8_t id : ids_) {
    const auto motor = MockDynamixelManager::instance().getMotor(id);
    if (!motor) {
      continue;
    }
    // Unpowered it is limp with its RAM gone, and it boots the same way
    motor->powerCycle();
    if (on) {
      motor->clearHardwareError();
    }
    motor->setCommunicationFault(on ? CommunicationFault::NONE : CommunicationFault::DISCONNECTED);
  }
}

void MockBusServer::reply(const uint8_t id, const uint8_t error, const std::vector<uint8_t>& params)
{
  auto packet = protocol2::statusPacket(id, error, params);
  const auto fault = takeWireFault();
  if (fault == WireFault::CORRUPT) {
    packet.back() ^= 0x01;
  } else if (fault == WireFault::TRUNCATE) {
    packet.resize(packet.size() / 2);
  } else if (fault == WireFault::DELAY) {
    delayed_.insert(delayed_.end(), packet.begin(), packet.end());
  }
  if (fault != WireFault::DROP && fault != WireFault::DELAY) {
    send(packet);
  }
}

void MockBusServer::send(const std::vector<uint8_t>& bytes) const
{
  size_t written = 0;
  while (written < bytes.size()) {
    const ssize_t count = write(master_fd_.get(), bytes.data() + written, bytes.size() - written);
    if (count < 0 && errno != EINTR) {
      throwErrno("Writing the pty master");
    }
    written += count > 0 ? static_cast<size_t>(count) : 0;
  }
}

}  // namespace dynamixel_ros_control
