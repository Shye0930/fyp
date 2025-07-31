cmake_minimum_required(VERSION 3.10)
project(pointcloud_to_grid)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Uncomment for warnings
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Suppress warnings (remove this in production)
add_compile_options(-w)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED) # Corrected from pcl_ros2
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Add the map_saver library
add_library(map_saver SHARED src/map_saver.cpp)
ament_target_dependencies(map_saver
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_ros
)
target_link_libraries(map_saver
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

# Add the pointcloud_to_grid_node executable
add_executable(pointcloud_to_grid_node src/pointcloud_to_grid_node.cpp)
ament_target_dependencies(pointcloud_to_grid_node
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_ros
)
target_link_libraries(pointcloud_to_grid_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  map_saver
)

# Install targets
install(TARGETS
  pointcloud_to_grid_node
  map_saver
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

ament_package()