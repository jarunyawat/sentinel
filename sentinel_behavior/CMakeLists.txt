cmake_minimum_required(VERSION 3.8)
project(sentinel_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)   # <-- add this
find_package(sentinel_interfaces REQUIRED)   # <-- add this

include_directories(include)

add_executable(create_circle src/create_circle.cpp)
ament_target_dependencies(create_circle
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  sentinel_interfaces
)

add_executable(follow_circle src/follow_circle.cpp)
ament_target_dependencies(follow_circle
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  sentinel_interfaces
)

# Install launch files
install(
  DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

# Install nodes
install(TARGETS
  create_circle
  follow_circle
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
