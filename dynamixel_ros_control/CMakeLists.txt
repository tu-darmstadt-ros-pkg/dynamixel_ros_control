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
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_ros_control_msgs REQUIRED)

set(HEADERS
    include/${PROJECT_NAME}/common.hpp
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
  std_msgs
  std_srvs
  ament_index_cpp
  transmission_interface
  hector_transmission_interface
  hector_transmission_interface_msgs
  controller_orchestrator
  realtime_tools
  sensor_msgs
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

  # Detect sanitizer builds from CMAKE_CXX_FLAGS (set by colcon mixin asan-gcc /
  # tsan). Used to inflate test timeouts (sanitizers slow execution ~10x) and to
  # wire up the ASAN/LSAN/TSAN options + suppression file so leak detection
  # works under _Exit().
  set(_dxl_sanitizer_timeout_factor 1)
  set(_dxl_sanitizer_env "")
  set(_dxl_lsan_suppressions
      "${CMAKE_CURRENT_SOURCE_DIR}/test/config/lsan_suppressions.txt")
  set(_dxl_tsan_suppressions
      "${CMAKE_CURRENT_SOURCE_DIR}/test/config/tsan_suppressions.txt")
  if(CMAKE_CXX_FLAGS MATCHES "fsanitize=address")
    set(_dxl_sanitizer_timeout_factor 5)
    set(_dxl_asan_opts_list
        "new_delete_type_mismatch=0" "verify_asan_link_order=0"
        "detect_leaks=1" "halt_on_error=0" "abort_on_error=0")
    string(JOIN ":" _dxl_asan_opts ${_dxl_asan_opts_list})
    list(
      APPEND _dxl_sanitizer_env "ASAN_OPTIONS=${_dxl_asan_opts}"
      "LSAN_OPTIONS=suppressions=${_dxl_lsan_suppressions}:print_suppressions=0"
    )
    message(STATUS "dynamixel_ros_control: AddressSanitizer detected, "
                   "scaling test timeouts x${_dxl_sanitizer_timeout_factor}")
  elseif(CMAKE_CXX_FLAGS MATCHES "fsanitize=thread")
    set(_dxl_sanitizer_timeout_factor 10)
    set(_dxl_tsan_opts_list
        "halt_on_error=0" "abort_on_error=0" "second_deadlock_stack=1"
        "history_size=7" "suppressions=${_dxl_tsan_suppressions}")
    string(JOIN ":" _dxl_tsan_opts ${_dxl_tsan_opts_list})
    list(APPEND _dxl_sanitizer_env "TSAN_OPTIONS=${_dxl_tsan_opts}")
    message(STATUS "dynamixel_ros_control: ThreadSanitizer detected, "
                   "scaling test timeouts x${_dxl_sanitizer_timeout_factor}")
  endif()

  # For tsan we need to launch the test via `setarch -R` (ADDR_NO_RANDOMIZE) to
  # avoid the kernel-6.5+ ASLR-vs-TSan-shadow-region collision. ament's RUNNER
  # hook lets us swap in a shim that does exactly that.
  set(_dxl_test_runner_arg "")
  if(CMAKE_CXX_FLAGS MATCHES "fsanitize=thread")
    set(_dxl_test_runner_arg
        RUNNER "${CMAKE_CURRENT_SOURCE_DIR}/test/setarch_runner.py")
  endif()

  # Helper: register a gtest with consistent timeout/dependencies/sanitizer env.
  function(dxl_add_gtest test_name source_file base_timeout)
    ament_add_gtest(${test_name} ${source_file} ${_dxl_test_runner_arg})
    target_link_libraries(${test_name} ${PROJECT_NAME})
    if(NOT "${ARGN}" STREQUAL "")
      ament_target_dependencies(${test_name} ${ARGN})
    endif()
    math(EXPR scaled_timeout
         "${base_timeout} * ${_dxl_sanitizer_timeout_factor}")
    set_tests_properties(${test_name} PROPERTIES TIMEOUT ${scaled_timeout})
    if(_dxl_sanitizer_env)
      set_tests_properties(${test_name} PROPERTIES ENVIRONMENT
                                                   "${_dxl_sanitizer_env}")
    endif()
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
