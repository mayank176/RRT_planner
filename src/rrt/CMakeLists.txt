cmake_minimum_required(VERSION 3.8)
project(rrt)

# Set C++ standard to C++17 which is required for ROS2
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find required packages that we'll need
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Create a library from your RRT implementation
# This makes your RRT code available as a library that other packages can use
add_library(${PROJECT_NAME}
  src/main.cpp  # Your RRT implementation file
)

# Make the headers accessible for this library
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link ROS2 dependencies to our library
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
)

# Install the library binary
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}    # Note this line - we define the export name
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Export the targets
# This makes our library findable by other packages using find_package
install(
  EXPORT export_${PROJECT_NAME}    # This must match the export name used above
  DESTINATION share/${PROJECT_NAME}/cmake
  NAMESPACE ${PROJECT_NAME}::
)

# Generate package configuration files
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
)

# This must be the last line
ament_package()