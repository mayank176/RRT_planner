cmake_minimum_required(VERSION 3.8)
project(mpc)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CasADi REQUIRED)
find_package(MKL REQUIRED)
find_package(rrt REQUIRED)


add_library(${PROJECT_NAME}_lib
  src/controls.cpp
  src/quadrotor.cpp
  src/sim.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${MKL_INCLUDE_DIR}
  ${CASADI_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_lib
  Eigen3::Eigen
  casadi
  ${MKL_LIBRARIES}
  ${CASADI_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  rrt
)

add_executable(${PROJECT_NAME}_node 
  src/main.cpp
)


target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
)


install(TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()