#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>

#include "port_directory.hpp"

namespace fs = std::filesystem;
using dynamixel_ros_control_gz::securePortDirectory;

class PortDirectory : public ::testing::Test
{
protected:
  void SetUp() override
  {
    std::string pattern = (fs::temp_directory_path() / "port_directory_XXXXXX").string();
    ASSERT_NE(mkdtemp(pattern.data()), nullptr);
    root_ = pattern;
  }
  void TearDown() override
  {
    fs::remove_all(root_);
  }

  fs::path root_;
};

TEST_F(PortDirectory, CreatesMissingDirectoriesForThisUserOnly)
{
  const fs::path directory = root_ / "dxl_sim" / "athena";
  securePortDirectory(directory);
  for (const fs::path& created : {root_ / "dxl_sim", directory}) {
    ASSERT_TRUE(fs::is_directory(created));
    EXPECT_EQ(fs::status(created).permissions() & fs::perms::all, fs::perms::owner_all) << created;
  }
}

TEST_F(PortDirectory, AcceptsAnExistingDirectoryOfThisUser)
{
  fs::create_directory(root_ / "dxl_sim");
  fs::permissions(root_ / "dxl_sim", fs::perms::owner_all | fs::perms::group_read | fs::perms::others_read);
  EXPECT_NO_THROW(securePortDirectory(root_ / "dxl_sim"));
}

TEST_F(PortDirectory, RefusesADirectoryOthersMayWriteTo)
{
  fs::create_directory(root_ / "dxl_sim");
  fs::permissions(root_ / "dxl_sim", fs::perms::all);
  EXPECT_THROW(securePortDirectory(root_ / "dxl_sim"), std::runtime_error);
}

TEST_F(PortDirectory, RefusesAFileInItsPlace)
{
  fs::create_directory(root_ / "dxl_sim");
  std::fclose(std::fopen((root_ / "dxl_sim" / "athena").c_str(), "w"));
  EXPECT_THROW(securePortDirectory(root_ / "dxl_sim" / "athena"), std::runtime_error);
}

TEST_F(PortDirectory, RefusesALinkToADirectoryOfThisUser)
{
  fs::create_directory(root_ / "elsewhere", root_);
  fs::permissions(root_ / "elsewhere", fs::perms::owner_all);
  fs::create_directory_symlink(root_ / "elsewhere", root_ / "dxl_sim");
  EXPECT_THROW(securePortDirectory(root_ / "dxl_sim"), std::runtime_error);
}

TEST_F(PortDirectory, RefusesADirectoryOfRoot)
{
  if (getuid() == 0) {
    GTEST_SKIP() << "Running as root, every directory is this user's";
  }
  EXPECT_THROW(securePortDirectory("/usr"), std::runtime_error);
}
