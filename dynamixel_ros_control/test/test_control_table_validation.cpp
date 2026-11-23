// A control table is a transcription of a manual, and a mistyped address in one is invisible:
// registers are keyed by name, so a wrong address collides with another register instead of
// producing a duplicate key, and the table loads clean. The consequence only appears on hardware,
// as a write that lands on a neighbouring register.
//
// Two halves here. The first checks the validation itself against tables written to fail. The
// second loads every table this package ships, so a future transcription error fails the build
// rather than the robot.

#include <gtest/gtest.h>

#include <dynamixel_ros_control/control_table.hpp>

#include <boost/algorithm/string.hpp>
#include <dynamixel_ros_control/common.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace dynamixel_ros_control::test {

class ControlTableValidationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char dir_template[] = "/tmp/dxl_control_table_test_XXXXXX";
    dir_ = mkdtemp(dir_template);
  }

  void TearDown() override
  {
    if (!dir_.empty()) {
      std::filesystem::remove_all(dir_);
    }
  }

  /// Write a model file whose control_table rows are `rows`, and return its path.
  std::string writeModel(const std::string& rows, const std::string& units = "  rad: 0.001\n")
  {
    const std::string path = dir_ + "/model.yaml";
    std::ofstream file(path);
    file << "unit_conversions:\n"
         << units << "indirect_addresses:\n"
         << "  - 168 | 224 | 28 | RAM\n"
         << "control_table:\n"
         << rows;
    file.close();
    return path;
  }

  std::string dir_;
};

// The shape every negative case below is a deviation from. Without this, a test that fails for an
// unrelated reason still looks like it proved its point.
TEST_F(ControlTableValidationTest, WellFormedTableLoads)
{
  ControlTable table;
  EXPECT_TRUE(table.loadFromYaml(writeModel("  - 64  | torque_enable  | 1 | RW | RAM | bool\n"
                                            "  - 65  | led            | 1 | RW | RAM |\n"
                                            "  - 116 | goal_position  | 4 | RW | RAM | rad\n")));
}

TEST_F(ControlTableValidationTest, CurrentCapModesAreLoaded)
{
  ControlTable table;
  ASSERT_TRUE(table.loadFromYaml(writeModel("  - 64  | torque_enable  | 1 | RW | RAM | bool\n",
                                            "  rad: 0.001\ncurrent_cap_modes: [velocity, current_based_position]\n")));
  EXPECT_EQ(table.currentCapModes(), (std::set<std::string>{"velocity", "current_based_position"}));
}

// A mode that does not exist would refuse max_current in the mode it meant without saying why
TEST_F(ControlTableValidationTest, AnUnknownCurrentCapModeIsRefused)
{
  ControlTable table;
  EXPECT_FALSE(table.loadFromYaml(
      writeModel("  - 64  | torque_enable  | 1 | RW | RAM | bool\n", "  rad: 0.001\ncurrent_cap_modes: [torque]\n")));
}

// The exact shape of the shipped typo: a 2-byte register placed inside a 4-byte one.
TEST_F(ControlTableValidationTest, OverlappingRegistersAreRefused)
{
  ControlTable table;
  EXPECT_FALSE(table.loadFromYaml(writeModel("  - 24 | moving_threshold  | 4 | RW | EEPROM |\n"
                                             "  - 24 | min_voltage_limit | 2 | RW | EEPROM |\n")));
}

// Partial overlap, where neither register starts where the other does.
TEST_F(ControlTableValidationTest, PartiallyOverlappingRegistersAreRefused)
{
  ControlTable table;
  EXPECT_FALSE(table.loadFromYaml(writeModel("  - 100 | first  | 4 | RW | RAM |\n"
                                             "  - 102 | second | 4 | RW | RAM |\n")));
}

// A width the driver cannot transfer. read1/2/4ByteTxRx are the only direct accessors there are, so
// a 3-byte register is writable through the cyclic path and unreadable through the direct one.
TEST_F(ControlTableValidationTest, UnsupportedRegisterWidthIsRefused)
{
  ControlTable table;
  EXPECT_FALSE(table.loadFromYaml(writeModel("  - 100 | three_bytes | 3 | RW | RAM |\n")));
}

// Zero is the width a mistyped column most easily produces, and the one that does real damage: a
// sync write group of nothing but zero-length entries sizes its parameter buffer to zero and then
// indexes it.
TEST_F(ControlTableValidationTest, ZeroRegisterWidthIsRefused)
{
  ControlTable table;
  EXPECT_FALSE(table.loadFromYaml(writeModel("  - 100 | zero_bytes | 0 | RW | RAM |\n")));
}

// The three legal widths, together, so the rejection above cannot be over-broad.
TEST_F(ControlTableValidationTest, SupportedRegisterWidthsAreAccepted)
{
  ControlTable table;
  EXPECT_TRUE(table.loadFromYaml(writeModel("  - 100 | led             | 1 | RW | RAM |\n"
                                            "  - 101 | position_p_gain | 2 | RW | RAM |\n"
                                            "  - 103 | goal_position   | 4 | RW | RAM | rad\n")));
}

// The access column is kept, not just checked. Nothing downstream can refuse a write to a
// telemetry register unless the item remembers which kind it is.
TEST_F(ControlTableValidationTest, AccessTypeIsRetainedOnTheItem)
{
  ControlTable table;
  ASSERT_TRUE(table.loadFromYaml(writeModel("  - 116 | goal_position    | 4 | RW | RAM | rad\n"
                                            "  - 132 | present_position | 4 | R  | RAM | rad\n")));
  const ControlTableItem* goal = table.findItem("goal_position");
  const ControlTableItem* present = table.findItem("present_position");
  ASSERT_NE(goal, nullptr);
  ASSERT_NE(present, nullptr);
  EXPECT_TRUE(goal->writable());
  EXPECT_FALSE(present->writable()) << "a register marked R must not report itself as writable";
}

// Registers that merely touch are the normal case and must stay legal.
TEST_F(ControlTableValidationTest, AdjacentRegistersAreAccepted)
{
  ControlTable table;
  EXPECT_TRUE(table.loadFromYaml(writeModel("  - 100 | first           | 4 | RW | RAM |\n"
                                            "  - 104 | position_p_gain | 2 | RW | RAM |\n")));
}

// A repeated name is dropped by the map, leaving whichever row came first in charge.
TEST_F(ControlTableValidationTest, DuplicateRegisterNamesAreRefused)
{
  ControlTable table;
  EXPECT_FALSE(table.loadFromYaml(writeModel("  - 100 | goal_position | 4 | RW | RAM | rad\n"
                                             "  - 200 | goal_position | 4 | RW | RAM | rad\n")));
}

// An undeclared unit used to fall back to a ratio of 1.0, which sends the value through in ticks.
// For a position limit that is a plausible number in the wrong scale, not an obvious failure.
TEST_F(ControlTableValidationTest, UndeclaredUnitIsRefused)
{
  ControlTable table;
  EXPECT_FALSE(table.loadFromYaml(writeModel("  - 100 | goal_current | 2 | RW | RAM | A\n")));
}

// A malformed indirect address row used to be skipped while the file still loaded, leaving the
// group builder to allocate slots out of a block that was never read.
TEST_F(ControlTableValidationTest, MalformedIndirectAddressRowIsRefused)
{
  const std::string path = dir_ + "/model.yaml";
  std::ofstream file(path);
  file << "unit_conversions:\n  rad: 0.001\n"
       << "indirect_addresses:\n"
       << "  - 168 | 224 | RAM\n"  // three fields, not four
       << "control_table:\n"
       << "  - 64 | torque_enable | 1 | RW | RAM | bool\n";
  file.close();

  ControlTable table;
  EXPECT_FALSE(table.loadFromYaml(path));
}

// yaml-cpp throws on anything it cannot decode, and every loader below decodes nodes. An escaping
// exception reaches the controller manager, which reports it as a plugin that would not load
// rather than as the file that is wrong.
TEST_F(ControlTableValidationTest, MalformedYamlIsRefusedWithoutThrowing)
{
  const std::string path = dir_ + "/model.yaml";
  std::ofstream file(path);
  file << "unit_conversions:\n  rad: 0.001\n"
       << "control_table: [ this bracket never closes\n";
  file.close();

  ControlTable table;
  EXPECT_NO_THROW({ EXPECT_FALSE(table.loadFromYaml(path)); });
}

// A scalar that is not a number reaches as<double>() and throws from there.
TEST_F(ControlTableValidationTest, NonNumericRatioIsRefusedWithoutThrowing)
{
  ControlTable table;
  EXPECT_NO_THROW({
    EXPECT_FALSE(
        table.loadFromYaml(writeModel("  - 100 | goal_position | 4 | RW | RAM | rad\n", "  rad: not_a_number\n")));
  });
}

// A ratio divides on the way to the motor, so zero converts every value of that unit to a raw 0.
TEST_F(ControlTableValidationTest, ZeroRatioIsRefused)
{
  ControlTable table;
  EXPECT_FALSE(table.loadFromYaml(writeModel("  - 100 | goal_position | 4 | RW | RAM | rad\n", "  rad: 0.0\n")));
}

// A negative ratio loads and works, and silently reverses the register's direction.
TEST_F(ControlTableValidationTest, NegativeRatioIsRefused)
{
  ControlTable table;
  EXPECT_FALSE(table.loadFromYaml(writeModel("  - 100 | goal_position | 4 | RW | RAM | rad\n", "  rad: -0.001\n")));
}

TEST_F(ControlTableValidationTest, NonFiniteRatioIsRefused)
{
  ControlTable table;
  EXPECT_FALSE(table.loadFromYaml(writeModel("  - 100 | goal_position | 4 | RW | RAM | rad\n", "  rad: .inf\n")));
  ControlTable nan_table;
  EXPECT_FALSE(nan_table.loadFromYaml(writeModel("  - 100 | goal_position | 4 | RW | RAM | rad\n", "  rad: .nan\n")));
}

// ============================================================================
// Every shipped table
// ============================================================================

// The regression guard. A transcription error in any devices/models/*.yaml fails here.
TEST(ShippedControlTables, AllLoadAndValidate)
{
  const std::filesystem::path models = std::filesystem::path(TEST_DEVICES_DIR) / "models";
  ASSERT_TRUE(std::filesystem::exists(models)) << "No models directory at " << models;

  int checked = 0;
  for (const auto& entry : std::filesystem::directory_iterator(models)) {
    if (entry.path().extension() != ".yaml") {
      continue;
    }
    ControlTable table;
    EXPECT_TRUE(table.loadFromYaml(entry.path().string()))
        << "Shipped control table failed to validate: " << entry.path().filename().string();
    ++checked;
  }
  EXPECT_GT(checked, 0) << "No control tables were checked, so this test proves nothing";
}

// A measured value without its unit is reported in raw counts, ten times the volts for the supply
TEST(ShippedControlTables, MeasuredValuesCarryTheirUnit)
{
  const std::filesystem::path models = std::filesystem::path(TEST_DEVICES_DIR) / "models";
  int checked = 0;
  for (const auto& entry : std::filesystem::directory_iterator(models)) {
    ControlTable table;
    ASSERT_TRUE(table.loadFromYaml(entry.path().string())) << entry.path().filename().string();
    for (const auto& [name, unit] :
         {std::pair<std::string, std::string>{"present_input_voltage", "V"}, {"realtime_tick", "ms"}}) {
      if (const ControlTableItem* item = table.findItem(name)) {
        EXPECT_EQ(item->unit(), unit) << name << " in " << entry.path().filename().string();
        ++checked;
      }
    }
  }
  EXPECT_GT(checked, 0);
}

// Every model number in model_list.yaml either resolves to a table that loads, or names a series
// this package does not ship. The driver reports the second case clearly, so it is not a failure
// here, but a model pointed at a table that exists and does not load is.
TEST(ShippedControlTables, EveryMappedModelResolves)
{
  const std::filesystem::path devices(TEST_DEVICES_DIR);
  const YAML::Node list = YAML::LoadFile((devices / "model_list.yaml").string());
  ASSERT_TRUE(list.IsMap());

  int resolved = 0;
  for (const auto& entry : list) {
    const auto series = entry.second.as<std::string>();
    const std::filesystem::path path = devices / "models" / (series + ".yaml");
    if (!std::filesystem::exists(path)) {
      continue;  // recognised model, table not shipped
    }
    ControlTable table;
    EXPECT_TRUE(table.loadFromYaml(path.string()))
        << "Model " << entry.first.as<int>() << " maps to '" << series << "', which fails to validate";
    ++resolved;
  }
  EXPECT_GT(resolved, 0) << "No mapped model resolved to a shipped table";
}

// The bus watchdog arms through this address, and a wrong one writes some other register at the
// control rate. 546 is the P series' Bus Watchdog, 98 the X and MX 2.0 one, per the e-Manual.
TEST(ShippedControlTables, BusWatchdogAddress)
{
  for (const auto& [series, address] :
       {std::pair<std::string, uint16_t>{"PH", 546}, {"PM", 546}, {"XM", 98}, {"MX2", 98}}) {
    ControlTable table;
    ASSERT_TRUE(table.loadFromYaml((std::filesystem::path(TEST_DEVICES_DIR) / "models" / (series + ".yaml")).string()));
    const ControlTableItem* item = table.findItem("bus_watchdog");
    ASSERT_NE(item, nullptr) << series;
    EXPECT_EQ(item->address(), address) << series;
  }
}

// PRO-series acceleration is specified at the core motor while position and velocity are specified
// at the output shaft, so its ratio is the only one in a table that is not read straight off the
// e-Manual. Pinning the derivation here because a hand-computed constant with no test is how the
// placeholder it replaced survived so long.
//
// RH.yaml is deliberately absent. It shares the PRO register map but its own manual gives
// 214.577 rev/min^2 and 0.114 RPM at the output, so the register layout does not decide this.
TEST(ShippedControlTables, ProAccelerationIsTheCoreMotorUnitOverTheGearRatio)
{
  constexpr double CORE_ACCELERATION_REV_PER_MIN2 = 58000.0;  // e-Manual Unit column, PRO series
  for (const char* series : {"PROExt", "H42-20-S300-R", "L54-30-S400-R", "L54-30-S500-R", "L54-50-S290-R",
                             "L54-50-S500-R", "M42-10-S260-R", "M54-40-S250-R", "M54-60-S250-R"}) {
    const std::filesystem::path path =
        std::filesystem::path(TEST_DEVICES_DIR) / "models" / (std::string(series) + ".yaml");
    ControlTable table;
    ASSERT_TRUE(table.loadFromYaml(path.string())) << series;

    // Read through the registers that carry the units, so this checks the ratio as it is applied.
    const ControlTableItem* velocity = table.findItem("goal_velocity");
    const ControlTableItem* acceleration = table.findItem("goal_acceleration");
    ASSERT_NE(velocity, nullptr) << series;
    ASSERT_NE(acceleration, nullptr) << series;

    const double expected = velocity->dxlValueToUnitRatio() * CORE_ACCELERATION_REV_PER_MIN2 / 60.0;
    EXPECT_NEAR(acceleration->dxlValueToUnitRatio(), expected, 1e-9)
        << series << ": rad_per_s2 must be rad_per_s times 58000/60. The gear ratio cancels because "
        << "rad_per_s already carries it.";
  }
}

// ============================================================================
// Wire byte order
// ============================================================================
//
// These cannot fail on a little-endian host, which is every machine this runs on today. They pin
// the intended order so an accidentally reversed shift is caught, and they document that the sync
// groups build their bytes rather than copying the host's own representation of an int32.

TEST(RegisterByteOrder, ValuesArePackedLittleEndian)
{
  unsigned char buffer[4] = {0xAA, 0xAA, 0xAA, 0xAA};

  packRegisterValue(buffer, 0x1234, 2);
  EXPECT_EQ(buffer[0], 0x34);
  EXPECT_EQ(buffer[1], 0x12);
  EXPECT_EQ(buffer[2], 0xAA) << "a two byte register must not touch the third byte";

  packRegisterValue(buffer, 0x12345678, 4);
  EXPECT_EQ(buffer[0], 0x78);
  EXPECT_EQ(buffer[1], 0x56);
  EXPECT_EQ(buffer[2], 0x34);
  EXPECT_EQ(buffer[3], 0x12);

  packRegisterValue(buffer, -2, 1);
  EXPECT_EQ(buffer[0], 0xFE) << "a negative value is packed as its two's complement bytes";
}

TEST(RegisterByteOrder, ValuesAreUnpackedInTheirOwnSignedness)
{
  EXPECT_EQ(unpackRegisterValue(0x00FE, 1, false), 254);
  EXPECT_EQ(unpackRegisterValue(0x00FE, 1, true), -2);
  EXPECT_EQ(unpackRegisterValue(0xFFFF, 2, false), 65535);
  EXPECT_EQ(unpackRegisterValue(0xFFFF, 2, true), -1);
  EXPECT_EQ(unpackRegisterValue(0x7FFF, 2, true), 32767);
  // Four bytes are the whole value either way.
  EXPECT_EQ(unpackRegisterValue(0xFFFFFFFF, 4, true), -1);
  // High bytes outside the register's width are not its value.
  EXPECT_EQ(unpackRegisterValue(0xABCD00FF, 1, false), 255);
}

// ============================================================================
// The signedness classification is complete over the shipped tables
// ============================================================================
//
// Reading the raw rows rather than a loaded table on purpose. An unclassified narrow register
// fails the load, so a test built on ControlTable would report "this file did not load" for the
// first offending table and stop. These name every offender across all nineteen at once.

namespace {

struct TableRow
{
  std::string file;
  std::string name;
  unsigned int data_length;
};

/// Split the control_table rows of every shipped model file, without validating them.
std::vector<TableRow> shippedControlTableRows()
{
  std::vector<TableRow> rows;
  const std::filesystem::path models = std::filesystem::path(TEST_DEVICES_DIR) / "models";
  for (const auto& file : std::filesystem::directory_iterator(models)) {
    if (file.path().extension() != ".yaml") {
      continue;
    }
    const YAML::Node config = YAML::LoadFile(file.path().string());
    const YAML::Node table = config["control_table"];
    if (!table.IsSequence()) {
      continue;
    }
    for (const auto& line_node : table) {
      std::string line = line_node.as<std::string>();
      removeWhitespace(line);
      std::vector<std::string> parts;
      boost::split(parts, line, boost::is_any_of("|"));
      if (parts.size() != 6) {
        continue;  // malformed rows are the load validator's business, not this test's
      }
      rows.push_back({file.path().filename().string(), parts[1],
                      static_cast<unsigned int>(std::strtoul(parts[2].c_str(), nullptr, 10))});
    }
  }
  return rows;
}

}  // namespace

// A new model file, or a new register added to an existing one, must not pick up an encoding by
// omission. This is what makes an unclassified register a build failure rather than a robot that
// refuses to configure once someone tries to run it.
TEST(ShippedControlTables, EveryNarrowRegisterIsClassified)
{
  const std::vector<TableRow> rows = shippedControlTableRows();
  ASSERT_FALSE(rows.empty()) << "No control table rows were read, so this test proves nothing";

  std::set<std::string> unclassified;
  for (const auto& row : rows) {
    if (row.data_length != 1 && row.data_length != 2) {
      continue;  // a 4-byte value is copied bit for bit and never consults the classification
    }
    if (registerEncoding(row.name) == RegisterEncoding::unknown) {
      unclassified.insert(row.name + " (" + row.file + ")");
    }
  }
  EXPECT_TRUE(unclassified.empty())
      << "These 1/2-byte registers have no entry in registerEncodings(): " << iterableToString(unclassified)
      << ". Classify each from the model's e-Manual control table.";
}

// The other direction. A register renamed in the tables leaves its old classification behind,
// where it looks like a decision someone made about a register that no longer exists.
TEST(ShippedControlTables, NoClassificationIsOrphaned)
{
  const std::vector<TableRow> rows = shippedControlTableRows();
  ASSERT_FALSE(rows.empty());

  std::set<std::string> declared;
  for (const auto& row : rows) {
    declared.insert(row.name);
  }

  std::set<std::string> orphaned;
  for (const auto& [name, encoding] : registerEncodings()) {
    if (declared.find(name) == declared.end()) {
      orphaned.insert(name);
    }
  }
  EXPECT_TRUE(orphaned.empty()) << "registerEncodings() classifies registers no shipped table declares: "
                                << iterableToString(orphaned) << ". Renamed, or removed?";
}

// The rejection itself, and its scope. A narrow register nobody classified is refused; the same
// name at four bytes still loads, because that path never consults the classification.
TEST_F(ControlTableValidationTest, UnclassifiedNarrowRegisterIsRefused)
{
  ControlTable narrow;
  EXPECT_FALSE(narrow.loadFromYaml(writeModel("  - 100 | a_register_nobody_classified | 2 | RW | RAM |\n")));

  ControlTable wide;
  EXPECT_TRUE(wide.loadFromYaml(writeModel("  - 100 | a_register_nobody_classified | 4 | RW | RAM |\n")))
      << "a four byte register is copied bit for bit and needs no classification";
}

// The six that carry a sign, and a sample of the rest, asserted against the lists directly. Without
// this the classification is only ever exercised through whichever registers a test happens to read.
TEST(ShippedControlTables, KnownEncodings)
{
  for (const char* name :
       {"bus_watchdog", "goal_current", "goal_pwm", "present_current", "present_load", "present_pwm"}) {
    EXPECT_EQ(registerEncoding(name), RegisterEncoding::twos_complement) << name;
  }
  for (const char* name : {"id", "return_delay_time", "led_blue", "model_number", "present_temperature",
                           "hardware_error_status", "realtime_tick", "current_limit", "pwm_limit"}) {
    EXPECT_EQ(registerEncoding(name), RegisterEncoding::unsigned_value) << name;
  }
  EXPECT_EQ(registerEncoding("no_such_register"), RegisterEncoding::unknown);
}

}  // namespace dynamixel_ros_control::test

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
