#include "map_saver.hpp"

// The function definition now matches the declaration in the .hpp file
void saveOccupancyGridAsPGM(const nav_msgs::msg::OccupancyGrid& grid,
                            const std::string& map_path,
                            const std::string& map_name,
                            rclcpp::Logger logger) { // Added rclcpp::Logger parameter
    int width = grid.info.width;
    int height = grid.info.height;

    // Open the PGM file
    std::ofstream pgm_file(map_path + map_name + ".pgm");
    if (!pgm_file.is_open()) {
        // Use RCLCPP_ERROR for ROS 2 logging
        RCLCPP_ERROR(logger, "Failed to open file for writing: %s.pgm", (map_path + map_name).c_str());
        return;
    }

    // Write the PGM header
    pgm_file << "P2\n";  // P2 indicates ASCII grayscale image
    pgm_file << width << " " << height << "\n";
    pgm_file << "255\n";  // Maximum pixel value

    // Write the map data
    for (int y = height - 1; y >= 0; --y) { // Start from the top row
        for (int x = 0; x < width; ++x) {

            // Compute the rotated indices
            int rotated_x = width - 1 - x;   // Flip x-axis
            int rotated_y = height - 1 - y;  // Flip y-axis
            int rotated_index = rotated_x + rotated_y * width;

            // Get the grid value and convert to pixel
            signed char value = grid.data[rotated_index];
            int pixel;

            if (value == -1) {
                pixel = 205;  // Unknown -> light gray
            } else if (value == 0) {
                pixel = 254;  // Free space -> white
            } else {
                pixel = 0;    // Occupied space -> black
            }

            pgm_file << pixel << " ";
        }
        pgm_file << "\n";
    }

    pgm_file.close();
    // Use RCLCPP_INFO for ROS 2 logging
    RCLCPP_INFO(logger, "Saved PGM file: %s.pgm", (map_path + map_name).c_str());

    // Write the YAML file
    std::ofstream yaml_file(map_path + map_name + ".yaml");
    if (!yaml_file.is_open()) {
        // Use RCLCPP_ERROR for ROS 2 logging
        RCLCPP_ERROR(logger, "Failed to open file for writing: %s.yaml", (map_path + map_name).c_str());
        return;
    }

    yaml_file << "image: " << map_name << ".pgm\n";
    yaml_file << "resolution: " << grid.info.resolution << "\n";
    // Note: The origin negation logic here is based on your original code's comment.
    // Ensure this matches your specific grid generation and coordinate system.
    yaml_file << "origin: [" << -grid.info.origin.position.x << ", "
              << -grid.info.origin.position.y << ", "
              << 0 << "]\n"; // tf2::getYaw(grid.info.origin.orientation) could be used if the orientation is relevant and updated in the grid.
    yaml_file << "negate: 0\n";
    yaml_file << "occupied_thresh: 0.65\n";
    yaml_file << "free_thresh: 0.196\n";

    yaml_file.close();
    // Use RCLCPP_INFO for ROS 2 logging
    RCLCPP_INFO(logger, "Saved YAML file: %s.yaml", (map_path + map_name).c_str());
}
