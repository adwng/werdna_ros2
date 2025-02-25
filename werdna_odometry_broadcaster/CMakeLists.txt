cmake_minimum_required(VERSION 3.8)
project(werdna_odometry_broadcaster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(werdna_odometry_broadcaster SHARED src/werdna_odometry_broadcaster.cpp)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(werdna_odometry_broadcaster_parameters
  src/werdna_odometry_broadcaster_parameters.yaml
)

target_link_libraries(werdna_odometry_broadcaster werdna_odometry_broadcaster_parameters)

target_include_directories(werdna_odometry_broadcaster PRIVATE include)

ament_target_dependencies(
  werdna_odometry_broadcaster
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_msgs
)

pluginlib_export_plugin_description_file(controller_interface werdna_odometry_broadcaster.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
