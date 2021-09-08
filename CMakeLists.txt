cmake_minimum_required(VERSION 3.5)
project(am_extruder_tool)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# compile serial communication helper node
add_executable(am_extruder_com
  src/AMExtruderComNode.cpp
  src/RS-232/rs232.c
)

target_include_directories(am_extruder_com
  PRIVATE include
  PRIVATE include/RS-232
)

ament_target_dependencies(am_extruder_com
  rclcpp
  std_msgs
)

# compile project
add_library(${PROJECT_NAME} SHARED
  src/AMExtruderHardware.cpp
  src/RS-232/rs232.c
)

target_include_directories(${PROJECT_NAME}
  PRIVATE include
  PRIVATE include/RS-232
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  std_msgs
)

pluginlib_export_plugin_description_file(hardware_interface am_extruder_tool_hardware.xml)

# install
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(TARGETS am_extruder_com DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include DESTINATION include)
install(DIRECTORY config description launch DESTINATION share/${PROJECT_NAME})

# tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Exports
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  std_msgs
)

ament_package()
