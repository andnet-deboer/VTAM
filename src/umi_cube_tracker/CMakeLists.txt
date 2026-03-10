cmake_minimum_required(VERSION 3.8)
project(umi_cube_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)

# Find OpenCV and ViSP
find_package(OpenCV REQUIRED)
find_package(VISP REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Include directories
include_directories(
  include
  ${VISP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Create the executable
add_executable(tracker_node src/tracker_node.cpp)

# Link dependencies
ament_target_dependencies(tracker_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_srvs
  tf2
  tf2_ros
  cv_bridge
)

target_link_libraries(tracker_node
  ${VISP_LIBRARIES}
  ${OpenCV_LIBS}
)

# Install rules
install(TARGETS tracker_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()