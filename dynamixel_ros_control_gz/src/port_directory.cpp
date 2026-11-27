#include "port_directory.hpp"

#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

namespace dynamixel_ros_control_gz {

namespace {

[[noreturn]] void refuse(const std::filesystem::path& path, const std::string& reason)
{
  throw std::runtime_error("Refusing the port directory '" + path.string() + "', " + reason + ".");
}

}  // namespace

void securePortDirectory(const std::filesystem::path& directory)
{
  std::filesystem::path path;
  struct stat status{};
  for (const auto& part : std::filesystem::absolute(directory)) {
    path /= part;
    if (mkdir(path.c_str(), 0700) != 0 && errno != EEXIST) {
      refuse(path, std::strerror(errno));
    }
    if (lstat(path.c_str(), &status) != 0) {
      refuse(path, std::strerror(errno));
    }
    if (!S_ISDIR(status.st_mode) || (status.st_uid != getuid() && status.st_uid != 0)) {
      refuse(path, "it is no directory of this user or root");
    }
  }
  if (status.st_uid != getuid() || (status.st_mode & (S_IWGRP | S_IWOTH)) != 0) {
    refuse(path, "others may write to it");
  }
}

}  // namespace dynamixel_ros_control_gz
