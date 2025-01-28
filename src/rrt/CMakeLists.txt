cmake_minimum_required(VERSION 3.8)
project(rrt)

# Set C++ standard to C++17 which is required for ROS2
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(${PROJECT_NAME}
  src/main.cpp  
)


target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)



ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
)


install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME} 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(
  DIRECTORY include/
  DESTINATION include
)


install(
  EXPORT export_${PROJECT_NAME}    
  DESTINATION share/${PROJECT_NAME}/cmake
  NAMESPACE ${PROJECT_NAME}::
)


ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
)

ament_package()