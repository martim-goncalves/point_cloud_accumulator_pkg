cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0074 NEW)
project(point_cloud_accumulator_pkg)


# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Source files
set(SOURCES
  src/point_cloud_accumulator_node.cpp
  src/accumulator.cpp
  src/voxel_scaler.cpp
  src/curves/logistic_sigmoid.cpp
  src/filters/color_consistency_filter.cpp
  src/filters/spatial_filter.cpp
  src/filters/statistical_outlier_filter.cpp
  src/filters/temporal_filter.cpp
  src/filters/tf_outlier_filter.cpp
  src/filters/nan_filter.cpp
  src/io/logger.cpp
  src/io/stop_watch.cpp
)

# Executable
add_executable(point_cloud_accumulator_node ${SOURCES})

# Link libraries
ament_target_dependencies(point_cloud_accumulator_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_eigen 
  pcl_conversions
)

target_link_libraries(point_cloud_accumulator_node ${PCL_LIBRARIES})

# Install
install(TARGETS point_cloud_accumulator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


# Export package
ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs tf2 tf2_ros tf2_sensor_msgs pcl_conversions PCL)
ament_package()
