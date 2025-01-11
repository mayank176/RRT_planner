cmake_minimum_required(VERSION 3.8)
project(rrt_vis)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# Note that we're looking for 'rrt' here, not 'rrt_core'
find_package(rrt REQUIRED)

# Create the visualization node executable
add_executable(rrt_vis_node src/main.cpp)

target_include_directories(rrt_vis_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link against dependencies, including the rrt library
ament_target_dependencies(rrt_vis_node
  rclcpp
  visualization_msgs
  geometry_msgs
  std_msgs
  rrt
)

# Install the executable
install(TARGETS rrt_vis_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()