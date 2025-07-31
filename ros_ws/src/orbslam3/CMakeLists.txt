cmake_minimum_required(VERSION 3.10)
project(orbslam3)

# [INFO] Set CMP0148 policy to suppress warning 
cmake_policy(SET CMP0148 OLD)

# You should set the PYTHONPATH to your own python site-packages path
set(ENV{PYTHONPATH} "/opt/ros/foxy/lib/python3.8/site-packages:/usr/local/lib/python3.8/dist-packages")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
# [TODO]: Uncomment this   
# add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# [TODO]: Remove this and allow warning 
add_compile_options(-w)



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(rclpy REQUIRED)
find_package(envision_interfaces REQUIRED)
find_package(image_transport REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${envision_interfaces_INCLUDE_DIRS}  # Add this line
)

link_directories(
  include
)

# Define common dependencies for all executables
set(COMMON_DEPENDENCIES
  rclcpp
  sensor_msgs
  cv_bridge
  ORB_SLAM3
  Pangolin
  OpenCV
  envision_interfaces
  image_transport
  visualization_msgs
  nav_msgs
  tf2_ros
)
add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
  src/utility.cc
)
ament_target_dependencies(mono ${COMMON_DEPENDENCIES})

add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
  src/utility.cc
)
ament_target_dependencies(rgbd ${COMMON_DEPENDENCIES} message_filters) 

add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
  src/utility.cc
)

ament_target_dependencies(stereo ${COMMON_DEPENDENCIES} message_filters) 

add_executable(kitti-stereo
  src/kitti-stereo/kitti-stereo.cpp
  src/kitti-stereo/kitti-stereo-slam-node.cpp
  src/utility.cc
)

ament_target_dependencies(kitti-stereo ${COMMON_DEPENDENCIES} message_filters) 

add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
  src/utility.cc
)
ament_target_dependencies(stereo-inertial ${COMMON_DEPENDENCIES})


install(TARGETS mono rgbd stereo stereo-inertial kitti-stereo
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
#install(DIRECTORY launch config vocabulary
#  DESTINATION share/${PROJECT_NAME}/)

install(PROGRAMS
  src/camera_publisher.py
  src/url_camera_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

