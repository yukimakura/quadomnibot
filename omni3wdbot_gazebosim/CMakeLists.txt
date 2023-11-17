cmake_minimum_required(VERSION 3.8)
project(omni3wdbot_gazebosim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(cmdvel_translator src/cmdvel_translator.cpp)
ament_target_dependencies(cmdvel_translator rclcpp geometry_msgs)

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  cmdvel_translator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()