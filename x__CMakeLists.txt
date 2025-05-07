cmake_minimum_required(VERSION 3.8)
project(point_cloud_accumulator_pkg)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

# Define the executable
add_executable(point_cloud_accumulator_node src/point_cloud_accumulator_node.cpp)
target_include_directories(point_cloud_accumulator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(
  point_cloud_accumulator_node
  ${PCL_LIBRARIES}
)

# Add dependencies
ament_target_dependencies(
  point_cloud_accumulator_node
  rclcpp
  sensor_msgs
  pcl_conversions
  tf2
  tf2_ros
  tf2_sensor_msgs
)

# Install the executable
install(TARGETS point_cloud_accumulator_node
  DESTINATION lib/${PROJECT_NAME}
)

# Ament package export
ament_package()
