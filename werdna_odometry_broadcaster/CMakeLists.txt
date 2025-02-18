cmake_minimum_required(VERSION 3.8)
project(werdna_odometry_broadcaster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  tf2
  tf2_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(werdna_odometry_broadcaster_parameters
  src/werdna_odometry_broadcaster_parameters.yaml
)

add_library(
  werdna_odometry_broadcaster SHARED
  src/werdna_odometry_broadcaster.cpp
  src/odometry.cpp
)

target_compile_features(werdna_odometry_broadcaster PUBLIC cxx_std_17)
target_include_directories(werdna_odometry_broadcaster PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/werdna_odometry_broadcaster>
)
target_link_libraries(werdna_odometry_broadcaster PUBLIC werdna_odometry_broadcaster_parameters)
ament_target_dependencies(werdna_odometry_broadcaster PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(werdna_odometry_broadcaster PRIVATE "WERDNA_ODOMETRY_BROADCASTER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface werdna_odometry_broadcaster.xml)

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

install(
  DIRECTORY include/
  DESTINATION include/werdna_odometry_broadcaster
)
install(TARGETS werdna_odometry_broadcaster werdna_odometry_broadcaster_parameters
  EXPORT export_werdna_odometry_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_werdna_odometry_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
