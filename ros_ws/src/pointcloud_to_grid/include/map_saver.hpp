#ifndef MAP_SAVER_HPP
#define MAP_SAVER_HPP

#include <fstream>
#include <string>

// ROS 2 includes
#include <rclcpp/rclcpp.hpp> // For ROS 2 logging and basic utilities
#include <nav_msgs/msg/occupancy_grid.hpp> // For OccupancyGrid message type
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For tf2::getYaw and geometry_msgs conversions
#include <tf2/LinearMath/Quaternion.h> // For tf2::Quaternion
#include <tf2/LinearMath/Matrix3x3.h> // For tf2::Matrix3x3

// Function declaration
// Added rclcpp::Logger as a parameter for ROS 2 logging
void saveOccupancyGridAsPGM(const nav_msgs::msg::OccupancyGrid& grid,
                            const std::string& map_path,
                            const std::string& map_name,
                            rclcpp::Logger logger);

#endif // MAP_SAVER_HPP
