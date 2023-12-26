cmake_minimum_required(VERSION 3.16)
project(omni3wdbot_hardware_interface LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(omni3wdbot_hardware_interface SHARED
  src/omni3wdbot_hardware_interface.cpp
  src/omnibot_comm.cpp
)
target_compile_features(omni3wdbot_hardware_interface PUBLIC cxx_std_17)
target_include_directories(omni3wdbot_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${Boost_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(omni3wdbot_hardware_interface PUBLIC ${Boost_LIBRARIES})

ament_target_dependencies(omni3wdbot_hardware_interface PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})


# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(omni3wdbot_hardware_interface PRIVATE "omni3wdbot_hardware_interface_BUILDING_DLL")
pluginlib_export_plugin_description_file(hardware_interface omni3wdbot_hardware_interface_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_omni_3wd_hardware
    test/test_load_omni_3wd_hardware.cpp
  )
  ament_target_dependencies(test_load_omni_3wd_hardware
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )
endif()

# for Test
install(
  DIRECTORY test/
  DESTINATION share/omni3wdbot_hardware_interface/test
)

install(TARGETS omni3wdbot_hardware_interface 
  EXPORT export_omni3wdbot_hardware_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_omni3wdbot_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()