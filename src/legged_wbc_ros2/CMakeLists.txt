cmake_minimum_required(VERSION 3.8)
project(legged_wbc_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# WBC library
add_library(legged_wbc_lib SHARED
  src/HoQp.cpp
  src/WbcBase.cpp
  src/Task.cpp
)
ament_target_dependencies(legged_wbc_lib
  rclcpp
  Eigen3
)

# WBC node
add_executable(legged_wbc_node
  src/wbc_node.cpp
)
target_link_libraries(legged_wbc_node
  legged_wbc_lib
)
ament_target_dependencies(legged_wbc_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

# Install
install(TARGETS
  legged_wbc_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  legged_wbc_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()
