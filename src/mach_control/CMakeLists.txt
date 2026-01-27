cmake_minimum_required(VERSION 3.8)
project(mach_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Executables
add_executable(ackermann_teleop_key src/ackermann_teleop_key.cpp)
add_executable(ackermann_teleop_joy src/ackermann_teleop_joy.cpp)
add_executable(lane_follower_pid src/lane_follower_pid.cpp)
add_executable(lane_points_test src/lane_points_test.cpp)

# Include dirs
# target_include_directories(ackermann_teleop_key PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# target_include_directories(ackermann_teleop_joy PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# )

# Link dependencies
ament_target_dependencies(ackermann_teleop_key
  rclcpp
  ackermann_msgs
)
ament_target_dependencies(ackermann_teleop_joy
  rclcpp
  ackermann_msgs
  sensor_msgs
)
ament_target_dependencies(lane_follower_pid
  rclcpp
  ackermann_msgs
  std_msgs
)
ament_target_dependencies(lane_points_test
  rclcpp
  std_msgs
)

# Install
install(TARGETS
  ackermann_teleop_key
  ackermann_teleop_joy
  lane_follower_pid
  lane_points_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
