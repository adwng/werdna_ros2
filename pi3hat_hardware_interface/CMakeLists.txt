cmake_minimum_required(VERSION 3.8)
project(pi3hat_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  pi3hat_hardware_interface
  SHARED
  hardware/pi3hat_system.cpp
)

# add include directory
target_include_directories(pi3hat_hardware_interface
PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hardware/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hardware/include/moteus>"
)


target_compile_features(
    pi3hat_hardware_interface PUBLIC cxx_std_17)

ament_target_dependencies(
  pi3hat_hardware_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "PI3HAT_HARDWARE_INTERFACE_BUILDING_DLL")

## Pi3hat library by mjbots
add_library(
  pi3hat
  SHARED
  hardware/include/motues/pi3hat.cc
)

# add include directory
target_include_directories(pi3hat
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
    
target_compile_features(
      pi3hat PUBLIC cxx_std_17)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface pi3hat_hardware_interface.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/pi3hat_hardware_interface
)

install(TARGETS pi3hat_hardware_interface
  EXPORT export_pi3hat_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_pi3hat_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
