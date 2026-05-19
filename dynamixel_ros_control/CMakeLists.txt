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
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(hector_transmission_interface REQUIRED)
find_package(hector_transmission_interface_msgs REQUIRED)
find_package(controller_orchestrator REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(dynamixel_ros_control_msgs REQUIRED)

set(HEADERS
    include/${PROJECT_NAME}/common.hpp
    include/${PROJECT_NAME}/diagnostic_state.hpp
    include/${PROJECT_NAME}/dynamixel_diagnostics.hpp
    include/${PROJECT_NAME}/joint.hpp
    include/${PROJECT_NAME}/dynamixel.hpp
    include/${PROJECT_NAME}/control_table.hpp
    include/${PROJECT_NAME}/control_table_item.hpp
    include/${PROJECT_NAME}/dynamixel_driver.hpp
    include/${PROJECT_NAME}/dynamixel_hardware_interface.hpp
    include/${PROJECT_NAME}/joint_state_publisher_set.hpp
    include/${PROJECT_NAME}/sync_read_manager.hpp
    include/${PROJECT_NAME}/sync_write_manager.hpp
    include/${PROJECT_NAME}/log.hpp)

set(SOURCES
    src/common.cpp
    src/diagnostic_state.cpp
    src/dynamixel_diagnostics.cpp
    src/joint.cpp
    src/dynamixel.cpp
    src/control_table.cpp
    src/control_table_item.cpp
    src/dynamixel_driver.cpp
    src/dynamixel_hardware_interface.cpp
    src/joint_state_publisher_set.cpp
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
  diagnostic_msgs
  sensor_msgs
  std_msgs
  std_srvs
  ament_index_cpp
  transmission_interface
  hector_transmission_interface
  hector_transmission_interface_msgs
  controller_orchestrator
  realtime_tools
  dynamixel_ros_control_msgs)

pluginlib_export_plugin_description_file(hardware_interface
                                         dynamixel_ros_control.xml)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hector_testing_utils REQUIRED)
  find_package(realtime_tools REQUIRED)

  add_definitions(-DTEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/config")
  set(HW_TEST_DEPS controller_manager hector_testing_utils realtime_tools
                   dynamixel_ros_control_msgs)

  # Scale test timeouts under sanitizer builds (set by colcon asan-gcc / tsan
  # mixins). Sanitizers slow execution roughly 5x (asan) / 10x (tsan).
  set(_dxl_sanitizer_timeout_factor 1)
  if(CMAKE_CXX_FLAGS MATCHES "fsanitize=address")
    set(_dxl_sanitizer_timeout_factor 5)
  elseif(CMAKE_CXX_FLAGS MATCHES "fsanitize=thread")
    set(_dxl_sanitizer_timeout_factor 10)
  endif()

  # Register a gtest with the project library linked, optional ament
  # dependencies (variadic), and a TIMEOUT scaled by the sanitizer factor.
  function(dxl_add_gtest test_name source_file base_timeout)
    ament_add_gtest(${test_name} ${source_file})
    target_link_libraries(${test_name} ${PROJECT_NAME})
    if(NOT "${ARGN}" STREQUAL "")
      ament_target_dependencies(${test_name} ${ARGN})
    endif()
    math(EXPR scaled_timeout
         "${base_timeout} * ${_dxl_sanitizer_timeout_factor}")
    set_tests_properties(${test_name} PROPERTIES TIMEOUT ${scaled_timeout})
  endfunction()

  # Mock-only test: doesn't use rclcpp.
  dxl_add_gtest(test_mock_dynamixel test/test_mock_dynamixel.cpp 60)

  # Hardware interface integration tests.
  dxl_add_gtest(test_hw_normal_usage test/test_hw_normal_usage.cpp 300
                ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_led test/test_hw_led.cpp 120 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_estop test/test_hw_estop.cpp 300 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_torque test/test_hw_torque.cpp 180 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_transmission test/test_hw_transmission.cpp 300
                ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_communication_error
                test/test_hw_communication_error.cpp 180 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_combined_state test/test_hw_combined_state.cpp 240
                ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_safety test/test_hw_safety.cpp 120 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_lifecycle test/test_hw_lifecycle.cpp 60 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_hardware_error test/test_hw_hardware_error.cpp 180
                ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_partial_torque test/test_hw_partial_torque.cpp 180
                ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_profile test/test_hw_profile.cpp 180 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_mimic test/test_hw_mimic.cpp 180 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_manifest test/test_hw_manifest.cpp 300 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_health test/test_hw_health.cpp 300 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_bus_watchdog test/test_hw_bus_watchdog.cpp 60
                ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_realtime_publishers
                test/test_hw_realtime_publishers.cpp 120 ${HW_TEST_DEPS})
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
