cmake_minimum_required(VERSION 3.8)
project(sentinel_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_executable(tcp_cmd_vel_server src/tcp_cmd_vel_server.cpp)

ament_target_dependencies(tcp_cmd_vel_server
  rclcpp
  geometry_msgs
)

target_link_libraries(tcp_cmd_vel_server
  Boost::system
)

# Install launch files
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install nodes
install(
  TARGETS
  tcp_cmd_vel_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
