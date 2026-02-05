cmake_minimum_required(VERSION 3.8)
project(mach_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(ackermann_to_twist src/ackermann_to_twist.cpp)
ament_target_dependencies(ackermann_to_twist
  rclcpp
  ackermann_msgs
  geometry_msgs
)

install(TARGETS ackermann_to_twist
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY 
  launch
  config
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()