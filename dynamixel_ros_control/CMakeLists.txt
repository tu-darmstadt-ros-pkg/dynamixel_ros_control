cmake_minimum_required(VERSION 3.8)
project(dynamixel_ros_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(hector_transmission_interface REQUIRED)
find_package(hector_transmission_interface_msgs REQUIRED)
find_package(controller_orchestrator REQUIRED)

set(HEADERS
    include/${PROJECT_NAME}/common.hpp
    include/${PROJECT_NAME}/joint.hpp
    include/${PROJECT_NAME}/dynamixel.hpp
    include/${PROJECT_NAME}/control_table.hpp
    include/${PROJECT_NAME}/control_table_item.hpp
    include/${PROJECT_NAME}/dynamixel_driver.hpp
    include/${PROJECT_NAME}/dynamixel_hardware_interface.hpp
    include/${PROJECT_NAME}/sync_read_manager.hpp
    include/${PROJECT_NAME}/sync_write_manager.hpp
    include/${PROJECT_NAME}/log.hpp)

set(SOURCES
    src/common.cpp
    src/joint.cpp
    src/dynamixel.cpp
    src/control_table.cpp
    src/control_table_item.cpp
    src/dynamixel_driver.cpp
    src/dynamixel_hardware_interface.cpp
    src/sync_read_manager.cpp
    src/sync_write_manager.cpp)
include_directories(include)

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp)
ament_target_dependencies(
  ${PROJECT_NAME}
  PUBLIC
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  hardware_interface
  pluginlib
  dynamixel_sdk
  yaml-cpp
  std_msgs
  std_srvs
  ament_index_cpp
  transmission_interface
  hector_transmission_interface
  hector_transmission_interface_msgs
  controller_orchestrator)

pluginlib_export_plugin_description_file(hardware_interface
                                         dynamixel_ros_control.xml)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mock_dynamixel test/test_mock_dynamixel.cpp)
  target_link_libraries(test_mock_dynamixel ${PROJECT_NAME})

  find_package(controller_manager REQUIRED)
  find_package(hector_testing_utils REQUIRED)
  find_package(realtime_tools REQUIRED)

  add_definitions(-DTEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/config")

  # Hardware interface test common dependencies
  set(HW_TEST_DEPS controller_manager hector_testing_utils realtime_tools)

  # Test: Normal usage (controller switching, movement, gripper, etc.)
  ament_add_gtest(test_hw_normal_usage test/test_hw_normal_usage.cpp)
  set_tests_properties(test_hw_normal_usage PROPERTIES TIMEOUT 300)
  target_link_libraries(test_hw_normal_usage ${PROJECT_NAME})
  ament_target_dependencies(test_hw_normal_usage ${HW_TEST_DEPS})

  # Test: LED colors
  ament_add_gtest(test_hw_led test/test_hw_led.cpp)
  set_tests_properties(test_hw_led PROPERTIES TIMEOUT 120)
  target_link_libraries(test_hw_led ${PROJECT_NAME})
  ament_target_dependencies(test_hw_led ${HW_TEST_DEPS})

  # Test: E-Stop functionality
  ament_add_gtest(test_hw_estop test/test_hw_estop.cpp)
  set_tests_properties(test_hw_estop PROPERTIES TIMEOUT 300)
  target_link_libraries(test_hw_estop ${PROJECT_NAME})
  ament_target_dependencies(test_hw_estop ${HW_TEST_DEPS})

  # Test: Torque enable/disable
  ament_add_gtest(test_hw_torque test/test_hw_torque.cpp)
  set_tests_properties(test_hw_torque PROPERTIES TIMEOUT 180)
  target_link_libraries(test_hw_torque ${PROJECT_NAME})
  ament_target_dependencies(test_hw_torque ${HW_TEST_DEPS})

  # Test: Transmission ratios and offset adjustment
  ament_add_gtest(test_hw_transmission test/test_hw_transmission.cpp)
  set_tests_properties(test_hw_transmission PROPERTIES TIMEOUT 300)
  target_link_libraries(test_hw_transmission ${PROJECT_NAME})
  ament_target_dependencies(test_hw_transmission ${HW_TEST_DEPS})

  # Test: Communication error handling and recovery
  ament_add_gtest(test_hw_communication_error
                  test/test_hw_communication_error.cpp)
  set_tests_properties(test_hw_communication_error PROPERTIES TIMEOUT 180)
  target_link_libraries(test_hw_communication_error ${PROJECT_NAME})
  ament_target_dependencies(test_hw_communication_error ${HW_TEST_DEPS})

  # Test: Combined state transitions (E-Stop + Torque + Calibration)
  ament_add_gtest(test_hw_combined_state test/test_hw_combined_state.cpp)
  set_tests_properties(test_hw_combined_state PROPERTIES TIMEOUT 240)
  target_link_libraries(test_hw_combined_state ${PROJECT_NAME})
  ament_target_dependencies(test_hw_combined_state ${HW_TEST_DEPS})

  # Test: Critical safety (torque enable with goal verification)
  ament_add_gtest(test_hw_safety test/test_hw_safety.cpp)
  set_tests_properties(test_hw_safety PROPERTIES TIMEOUT 120)
  target_link_libraries(test_hw_safety ${PROJECT_NAME})
  ament_target_dependencies(test_hw_safety ${HW_TEST_DEPS})

  # Test: Lifecycle (service availability)
  ament_add_gtest(test_hw_lifecycle test/test_hw_lifecycle.cpp)
  set_tests_properties(test_hw_lifecycle PROPERTIES TIMEOUT 60)
  target_link_libraries(test_hw_lifecycle ${PROJECT_NAME})
  ament_target_dependencies(test_hw_lifecycle ${HW_TEST_DEPS})

  # Test: Hardware error handling (red LED, e-stop activation, reboot)
  ament_add_gtest(test_hw_hardware_error test/test_hw_hardware_error.cpp)
  set_tests_properties(test_hw_hardware_error PROPERTIES TIMEOUT 180)
  target_link_libraries(test_hw_hardware_error ${PROJECT_NAME})
  ament_target_dependencies(test_hw_hardware_error ${HW_TEST_DEPS})

  # Test: Mimic joints (gripper finger joints following servo)
  ament_add_gtest(test_hw_mimic test/test_hw_mimic.cpp)
  set_tests_properties(test_hw_mimic PROPERTIES TIMEOUT 180)
  target_link_libraries(test_hw_mimic ${PROJECT_NAME})
  ament_target_dependencies(test_hw_mimic ${HW_TEST_DEPS})
endif()

# Install library and include folder
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# Install directories
install(DIRECTORY launch config devices DESTINATION share/${PROJECT_NAME})

ament_package()
