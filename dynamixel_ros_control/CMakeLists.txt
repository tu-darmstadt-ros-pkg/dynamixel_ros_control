cmake_minimum_required(VERSION 3.8)
project(dynamixel_ros_control)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(yaml-cpp REQUIRED)
#find_package(dynamixel_ros_control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(transmission_interface REQUIRED)



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
  include/${PROJECT_NAME}/log.hpp
)

set(SOURCES
  src/common.cpp
  src/joint.cpp
  src/dynamixel.cpp
  src/control_table.cpp
  src/control_table_item.cpp
  src/dynamixel_driver.cpp
  src/dynamixel_hardware_interface.cpp
  src/sync_read_manager.cpp
  src/sync_write_manager.cpp
)
include_directories(include)

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp)
ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp rclcpp_lifecycle lifecycle_msgs hardware_interface pluginlib
    dynamixel_sdk yaml-cpp std_msgs std_srvs ament_index_cpp transmission_interface)

pluginlib_export_plugin_description_file(hardware_interface dynamixel_ros_control.xml)

# Install library and include folder
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-targets LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# Install directories
install(DIRECTORY launch config devices
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

