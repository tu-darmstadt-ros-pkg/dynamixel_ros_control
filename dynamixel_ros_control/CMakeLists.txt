cmake_minimum_required(VERSION 3.16)
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
find_package(realtime_tools REQUIRED)

set(HEADERS
    include/${PROJECT_NAME}/common.hpp
    include/${PROJECT_NAME}/dynamixel_diagnostics.hpp
    include/${PROJECT_NAME}/joint.hpp
    include/${PROJECT_NAME}/dynamixel.hpp
    include/${PROJECT_NAME}/control_table.hpp
    include/${PROJECT_NAME}/control_table_item.hpp
    include/${PROJECT_NAME}/dynamixel_driver.hpp
    include/${PROJECT_NAME}/dynamixel_hardware_interface.hpp
    include/${PROJECT_NAME}/joint_state_publisher_set.hpp
    include/${PROJECT_NAME}/led_manager.hpp
    include/${PROJECT_NAME}/sync_groups.hpp
    include/${PROJECT_NAME}/sync_read_manager.hpp
    include/${PROJECT_NAME}/sync_write_manager.hpp
    include/${PROJECT_NAME}/log.hpp)

set(SOURCES
    src/bus_watchdog.cpp
    src/common.cpp
    src/dynamixel_diagnostics.cpp
    src/joint.cpp
    src/dynamixel.cpp
    src/control_table.cpp
    src/control_table_item.cpp
    src/dynamixel_driver.cpp
    src/dynamixel_hardware_interface.cpp
    src/dynamixel_hardware_interface_securing.cpp
    src/dynamixel_hardware_interface_recovery.cpp
    src/dynamixel_hardware_interface_torque.cpp
    src/joint_state_publisher_set.cpp
    src/led_manager.cpp
    src/sync_groups.cpp
    src/sync_read_manager.cpp
    src/sync_write_manager.cpp)
include_directories(include)

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
# For packages that link the libraries by target, such as a later simulation host
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC rclcpp::rclcpp
         rclcpp_lifecycle::rclcpp_lifecycle
         lifecycle_msgs::lifecycle_msgs
         hardware_interface::hardware_interface
         pluginlib::pluginlib
         dynamixel_sdk::dynamixel_sdk
         yaml-cpp::yaml-cpp
         diagnostic_msgs::diagnostic_msgs
         sensor_msgs::sensor_msgs
         std_msgs::std_msgs
         std_srvs::std_srvs
         ament_index_cpp::ament_index_cpp
         transmission_interface::transmission_interface
         hector_transmission_interface::hector_transmission_interface
         hector_transmission_interface_msgs::hector_transmission_interface_msgs
         realtime_tools::realtime_tools)

# The simulated motors and their bus, for the tests, the emulator and the gz host. The plugin
# never links it and reaches the motors through use_dummy only where a process simulates them.
add_library(
  ${PROJECT_NAME}_bus_sim SHARED
  src/mock_dynamixel.cpp include/${PROJECT_NAME}/mock_dynamixel.hpp
  src/mock_bus_server.cpp include/${PROJECT_NAME}/mock_bus_server.hpp
  src/servo_law.cpp include/${PROJECT_NAME}/servo_law.hpp)
target_link_libraries(${PROJECT_NAME}_bus_sim PUBLIC ${PROJECT_NAME})
add_executable(dynamixel_bus_emulator src/dynamixel_bus_emulator.cpp)
target_link_libraries(dynamixel_bus_emulator ${PROJECT_NAME}_bus_sim)

pluginlib_export_plugin_description_file(hardware_interface
                                         dynamixel_ros_control.xml)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_contract_tests REQUIRED)
  find_package(hector_controller_manager REQUIRED)
  find_package(hector_testing_utils REQUIRED)

  add_definitions(-DTEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/config")
  add_definitions(-DTEST_DEVICES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/devices")
  set(HW_TEST_DEPS controller_manager hector_testing_utils realtime_tools)

  # Scale test timeouts under sanitizer builds (set by colcon asan-gcc / tsan
  # mixins). Sanitizers slow execution roughly 5x (asan) / 10x (tsan).
  set(_dxl_sanitizer_timeout_factor 1)
  if(CMAKE_CXX_FLAGS MATCHES "fsanitize=address")
    set(_dxl_sanitizer_timeout_factor 5)
  elseif(CMAKE_CXX_FLAGS MATCHES "fsanitize=thread")
    set(_dxl_sanitizer_timeout_factor 10)
  endif()

  # Register a gtest with the simulated bus linked, optional package
  # dependencies (variadic, linked as <pkg>::<pkg>), and a TIMEOUT scaled by the
  # sanitizer factor.
  #
  # Every test that spins up a node, a controller manager or a topic runs under
  # ament_add_ros_isolated_gtest: it wraps the executable in
  # run_test_isolated.py, which leases a free ROS_DOMAIN_ID for the duration of
  # the test. DDS discovery is partitioned by domain, so parallel runs (ctest
  # -j) cannot see each other's /controller_manager, controller names or e-stop
  # topics. Setting ROS_DOMAIN_ID or DISABLE_ROS_ISOLATION in the environment
  # opts out, which is what you want when attaching a debugger and not
  # otherwise.
  #
  # TestHome gives each test a HOME and ROS_LOG_DIR of its own under DXL_TEST_HOME, in the build
  # tree, so nothing a test leaves behind ends up in /tmp or in the user's home. The ROS_LOG_DIR
  # set here catches the logs of a test that runs without it.
  function(dxl_add_gtest test_name source_file base_timeout)
    set(test_home ${CMAKE_CURRENT_BINARY_DIR}/test_home/${test_name})
    ament_add_ros_isolated_gtest(${test_name} ${source_file} ENV DXL_TEST_HOME=${test_home}
                                 ROS_LOG_DIR=${test_home}/log)
    target_link_libraries(${test_name} ${PROJECT_NAME}_bus_sim)
    target_compile_definitions(${test_name} PRIVATE DXL_TEST_TIMEOUT_FACTOR=${_dxl_sanitizer_timeout_factor})
    foreach(dep IN LISTS ARGN)
      target_link_libraries(${test_name} ${dep}::${dep})
    endforeach()
    math(EXPR scaled_timeout
         "${base_timeout} * ${_dxl_sanitizer_timeout_factor}")
    set_tests_properties(${test_name} PROPERTIES TIMEOUT ${scaled_timeout})
  endfunction()

  # Same as dxl_add_gtest, minus the domain isolation: these tests never touch
  # rclcpp, so they need no ROS domain of their own.
  function(dxl_add_plain_gtest test_name source_file base_timeout)
    ament_add_gtest(${test_name} ${source_file})
    target_link_libraries(${test_name} ${PROJECT_NAME}_bus_sim)
    target_compile_definitions(${test_name} PRIVATE DXL_TEST_TIMEOUT_FACTOR=${_dxl_sanitizer_timeout_factor})
    math(EXPR scaled_timeout
         "${base_timeout} * ${_dxl_sanitizer_timeout_factor}")
    set_tests_properties(${test_name} PROPERTIES TIMEOUT ${scaled_timeout})
  endfunction()

  dxl_add_plain_gtest(test_mock_dynamixel test/test_mock_dynamixel.cpp 60)
  dxl_add_plain_gtest(test_dynamixel_driver test/test_dynamixel_driver.cpp 60)
  dxl_add_plain_gtest(test_sync_managers test/test_sync_managers.cpp 60)
  # The plugin library alone, as ros2_control_node loads it
  ament_add_gtest(test_dummy_without_simulation
                  test/test_dummy_without_simulation.cpp)
  target_link_libraries(test_dummy_without_simulation ${PROJECT_NAME})
  dxl_add_plain_gtest(test_bus_watchdog_ticks test/test_bus_watchdog_ticks.cpp
                      60)
  dxl_add_plain_gtest(test_translation_loading
                      test/test_translation_loading.cpp 60)
  dxl_add_plain_gtest(test_control_table_validation
                      test/test_control_table_validation.cpp 60)
  dxl_add_plain_gtest(test_protocol2_framing test/test_protocol2_framing.cpp 60)
  dxl_add_plain_gtest(test_mock_bus_server test/test_mock_bus_server.cpp 60)
  dxl_add_plain_gtest(test_sdk_late_status test/test_sdk_late_status.cpp 60)
  dxl_add_plain_gtest(test_servo_law test/test_servo_law.cpp 60)
  dxl_add_plain_gtest(test_bus_emulator test/test_bus_emulator.cpp 60)
  target_compile_definitions(
    test_bus_emulator PRIVATE BUS_EMULATOR="$<TARGET_FILE:dynamixel_bus_emulator>")

  dxl_add_gtest(test_diagnostics_health test/test_diagnostics_health.cpp 60)

  # Hardware interface integration tests.
  dxl_add_gtest(test_hw_normal_usage test/test_hw_normal_usage.cpp 300
                ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_led test/test_hw_led.cpp 120 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_command_handles test/test_hw_command_handles.cpp 120
                ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_nan_hold test/test_hw_nan_hold.cpp 120 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_on_error test/test_hw_on_error.cpp 120 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_lifecycle_claims test/test_hw_lifecycle_claims.cpp 120
                ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_keep_goal test/test_hw_keep_goal.cpp 120 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_unknown_parameters test/test_hw_unknown_parameters.cpp 120
                ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_bus_watchdog_timing test/test_hw_bus_watchdog_timing.cpp 120
                ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_switch_refusal test/test_hw_switch_refusal.cpp 120 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_caps test/test_hw_caps.cpp 120 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_torque_off_mode test/test_hw_torque_off_mode.cpp 120 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_contract test/test_hw_contract.cpp 120 ${HW_TEST_DEPS}
                hardware_contract_tests)
  dxl_add_gtest(test_hw_watchdog_unsupported test/test_hw_watchdog_unsupported.cpp 60 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_port_name test/test_hw_port_name.cpp 60 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_wire_faults test/test_hw_wire_faults.cpp 120 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_hcm_async test/test_hw_hcm_async.cpp 300 ${HW_TEST_DEPS}
                hector_controller_manager)
  dxl_add_gtest(test_hw_executor_teardown test/test_hw_executor_teardown.cpp 300 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_control_mode test/test_hw_control_mode.cpp 180
                ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_strict_init test/test_hw_strict_init.cpp 180
                ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_reboot_indirect test/test_hw_reboot_indirect.cpp 180
                ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_reboot_unconnected test/test_hw_reboot_unconnected.cpp
                180 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_per_joint_torque test/test_hw_per_joint_torque.cpp 240
                ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_securing_order test/test_hw_securing_order.cpp 300
                ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_torque test/test_hw_torque.cpp 180 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_transmission test/test_hw_transmission.cpp 300
                ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_communication_error
                test/test_hw_communication_error.cpp 300 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_combined_state test/test_hw_combined_state.cpp 240
                ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_safety test/test_hw_safety.cpp 120 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_lifecycle test/test_hw_lifecycle.cpp 60 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_deactivate_current test/test_hw_deactivate_current.cpp
                60 ${HW_TEST_DEPS})
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
  dxl_add_gtest(test_hw_warm_start test/test_hw_warm_start.cpp 180
                ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_securing test/test_hw_securing.cpp 120 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_power_blip test/test_hw_power_blip.cpp 60 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_calibration_while_active test/test_hw_calibration_while_active.cpp 60 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_bus_ownership test/test_hw_bus_ownership.cpp 60 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_logging test/test_hw_logging.cpp 60 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_latency_timer test/test_hw_latency_timer.cpp 60 ${HW_TEST_DEPS})
  dxl_add_gtest(test_hw_allocations test/test_hw_allocations.cpp 60 ${HW_TEST_DEPS})

  # Wall clock bound, so it runs only with AMENT_RUN_PERFORMANCE_TESTS
  find_package(ament_cmake_google_benchmark REQUIRED)
  # It calls the offset services by name, so it must not reach a running robot
  set(benchmark_home ${CMAKE_CURRENT_BINARY_DIR}/test_home/benchmark_hw_offset_service_call)
  ament_add_google_benchmark(benchmark_hw_offset_service_call
                             test/benchmark_hw_offset_service_call.cpp
                             ENV ROS_AUTOMATIC_DISCOVERY_RANGE=OFF DXL_TEST_HOME=${benchmark_home}
                             ROS_LOG_DIR=${benchmark_home}/log)
  if(TARGET benchmark_hw_offset_service_call)
    ament_find_gtest()
    target_link_libraries(benchmark_hw_offset_service_call ${PROJECT_NAME}_bus_sim
                          ${GTEST_LIBRARIES})
    target_compile_definitions(benchmark_hw_offset_service_call
                               PRIVATE DXL_TEST_TIMEOUT_FACTOR=${_dxl_sanitizer_timeout_factor})
    foreach(dep IN LISTS HW_TEST_DEPS)
      target_link_libraries(benchmark_hw_offset_service_call ${dep}::${dep})
    endforeach()
  endif()
endif()

# Install library and include folder
install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_bus_sim
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS dynamixel_bus_emulator DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# Install directories
install(DIRECTORY launch config devices DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp
  diagnostic_msgs
  dynamixel_sdk
  hardware_interface
  hector_transmission_interface
  hector_transmission_interface_msgs
  lifecycle_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  std_msgs
  std_srvs
  transmission_interface
  yaml-cpp)

ament_package()
