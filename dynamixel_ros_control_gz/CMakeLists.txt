cmake_minimum_required(VERSION 3.16)
project(dynamixel_ros_control_gz)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(dynamixel_ros_control REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(sdformat_vendor REQUIRED)
find_package(sdformat REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(port_directory STATIC src/port_directory.cpp)
set_target_properties(port_directory PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The SDF names it dynamixel_ros_control_gz-system, so gz does not take lib/${PROJECT_NAME} for it
add_library(${PROJECT_NAME}-system SHARED src/dynamixel_bus_system.cpp)
target_link_libraries(
  ${PROJECT_NAME}-system
  port_directory
  ament_index_cpp::ament_index_cpp
  dynamixel_ros_control::dynamixel_ros_control_bus_sim
  gz-msgs::gz-msgs
  gz-plugin::register
  gz-sim::gz-sim
  gz-transport::gz-transport
  sdformat::sdformat
  yaml-cpp::yaml-cpp)

install(TARGETS ${PROJECT_NAME}-system LIBRARY DESTINATION lib)
install(PROGRAMS scripts/wait_for_ports DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_port_directory test/test_port_directory.cpp)
  target_link_libraries(test_port_directory port_directory)
  target_include_directories(test_port_directory PRIVATE src)

  ament_add_gtest(test_bus_watchdog test/test_bus_watchdog.cpp ENV
                  GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:${PROJECT_NAME}-system>)
  target_link_libraries(test_bus_watchdog dynamixel_ros_control::dynamixel_ros_control_bus_sim gz-sim::gz-sim)
  target_compile_definitions(test_bus_watchdog PRIVATE TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/config")

  ament_add_test(
    test_wait_for_ports
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_wait_for_ports.py -v
    ENV WAIT_FOR_PORTS=${CMAKE_CURRENT_SOURCE_DIR}/scripts/wait_for_ports)

  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  # In a ROS domain of its own, gz in a partition of its own
  add_launch_test(test/test_gz_bus.py TIMEOUT 180 RUNNER
                  "${ament_cmake_ros_DIR}/run_test_isolated.py" APPEND_ENV
                  GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:${PROJECT_NAME}-system>)
endif()

ament_package()
