cmake_minimum_required(VERSION 3.8)
project(werdna_teleop)

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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(werdna_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(werdna_teleop_joy_node src/werdna_teleop_joy.cpp)

ament_target_dependencies(werdna_teleop_joy_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  werdna_msgs
)

install(TARGETS
  werdna_teleop_joy_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  src launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()