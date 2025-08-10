#!/usr/bin/env python3

import rclpy
import matplotlib
matplotlib.use('TkAgg')  # Force TkAgg backend
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
import math
import os
import yaml
import cv2
import yaml
import numpy as np
import matplotlib.pyplot as plt

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        
        # Parameters
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)  # Degrees
        self.declare_parameter('goal_radius', 0.5)  # Meters
        self.declare_parameter('pose_topic', '/orb_slam3/camera_pose')
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('map_path', '~/Desktop/fyp/ros_ws/maps/map')
        self.declare_parameter('visualization_mode', 'none')  # Options: 'matplotlib', 'rviz', 'none'
        self.declare_parameter('grid_resolution', 0.05)  # meters per cell
        self.declare_parameter('grid_width', 1000)  # cells
        self.declare_parameter('grid_height', 1000)  # cells

        # Get parameters
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_yaw = math.radians(self.get_parameter('goal_yaw').value)
        self.goal_radius = self.get_parameter('goal_radius').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.goal_frame = self.get_parameter('goal_frame').value
        self.map_path = os.path.expanduser(self.get_parameter('map_path').value)
        self.visualization_mode = self.get_parameter('visualization_mode').value
        self.grid_resolution = self.get_parameter('grid_resolution').value
        self.grid_width = self.get_parameter('grid_width').value
        self.grid_height = self.get_parameter('grid_height').value

        # Calculate extent based on grid parameters
        self.extent_x_min = -self.grid_width * self.grid_resolution / 2.0
        self.extent_x_max = self.grid_width * self.grid_resolution / 2.0
        self.extent_y_min = -self.grid_height * self.grid_resolution / 2.0
        self.extent_y_max = self.grid_height * self.grid_resolution / 2.0
                
        
        self.goal_reached = False
        self.map_exists = False
        self.occupancy_grid = None
        
        # Check for map files and visualize
        self.check_and_load_map()
        
        self.goal_reached = False
        self.get_logger().info('GoalPublisher node initialized')

    def check_and_load_map(self):
        """Check if .pgm and .yaml files exist and load them."""
        pgm_path = self.map_path + '.pgm'
        yaml_path = self.map_path + '.yaml'
        
        # Check if both files exist
        self.map_exists = os.path.exists(pgm_path) and os.path.exists(yaml_path)
        
        if self.map_exists:
            self.get_logger().info(f'Map files found: {pgm_path}, {yaml_path}')
            
            # Load and visualize map based on mode
            if self.visualization_mode == 'matplotlib':
                self.visualize_map_matplotlib(pgm_path, yaml_path)
        else:
            self.get_logger().warn(f'Map files not found: {pgm_path}, {yaml_path}')

    # def visualize_map_matplotlib(self, pgm_path, yaml_path):
    #     """Visualize the map using Matplotlib, matching pointcloud_to_occupancy_map grid."""
    #     try:
    #         # Read .pgm file
    #         map_image = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    #         if map_image is None:
    #             self.get_logger().error(f'Failed to load .pgm file: {pgm_path}')
    #             return
            
    #         # Flip image to match ROS convention (origin at bottom-left)
    #         map_image = np.flipud(map_image)
            
    #         # Pad the image to match the full grid size if smaller
    #         if map_image.shape[1] < self.grid_width or map_image.shape[0] < self.grid_height:
    #             padded_image = np.full((self.grid_height, self.grid_width), 205, dtype=np.uint8)
    #             h_offset = (self.grid_height - map_image.shape[0]) // 2
    #             w_offset = (self.grid_width - map_image.shape[1]) // 2
    #             padded_image[h_offset:h_offset + map_image.shape[0], w_offset:w_offset + map_image.shape[1]] = map_image
    #             map_image = padded_image
            
    #         # Set figure size proportional to grid size (in inches, scaled by resolution)
    #         figsize_x = self.grid_width * self.grid_resolution / 10.0  # Scale factor of 10 for visibility
    #         figsize_y = self.grid_height * self.grid_resolution / 10.0
    #         plt.figure(figsize=(figsize_x, figsize_y))
            
    #         # Plot map
    #         # plt.figure(figsize=(10, 10))
    #         plt.imshow(map_image, cmap='gray', origin='lower', extent=[-5.0, 5.0, -5.0, 5.0])
    #         plt.title('Occupancy Grid Map')
    #         plt.xlabel('X (meters)')
    #         plt.ylabel('Y (meters)')
            
    #         # Plot goal position in world coordinates
    #         plt.plot(self.goal_x, self.goal_y, 'ro', markersize=10, label='Goal')
    #         plt.legend()
            
    #         plt.show() 
    #         self.get_logger().info('Map visualized in Matplotlib')
        
    #     except Exception as e:
    #         self.get_logger().error(f'Failed to visualize map in Matplotlib: {str(e)}')
    def visualize_map_matplotlib(self, pgm_path, yaml_path):
        """Visualize the map using Matplotlib, matching pointcloud_to_occupancy_map grid."""
        try:
            # Read .pgm file
            map_image = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
            if map_image is None:
                self.get_logger().error(f'Failed to load .pgm file: {pgm_path}')
                return
            
            # Flip image to match ROS convention (origin at bottom-left)
            map_image = np.flipud(map_image)
            
            # Pad the image to match the full grid size if smaller
            if map_image.shape[1] < self.grid_width or map_image.shape[0] < self.grid_height:
                padded_image = np.full((self.grid_height, self.grid_width), 205, dtype=np.uint8)
                h_offset = (self.grid_height - map_image.shape[0]) // 2
                w_offset = (self.grid_width - map_image.shape[1]) // 2
                padded_image[h_offset:h_offset + map_image.shape[0], w_offset:w_offset + map_image.shape[1]] = map_image
                map_image = padded_image
            
            # Set figure size proportional to grid size (in inches, scaled by resolution)
            figsize_x = self.grid_width * self.grid_resolution / 10.0  # Scale factor of 10 for visibility
            figsize_y = self.grid_height * self.grid_resolution / 10.0
            plt.figure(figsize=(figsize_x, figsize_y))
            
            # Plot map
            plt.imshow(map_image, cmap='gray', origin='lower', extent=[-5.0, 5.0, -5.0, 5.0])
            plt.title('Occupancy Grid Map')
            plt.xlabel('X (meters)')
            plt.ylabel('Y (meters)')

            markersize_from_radius = self.goal_radius * 30.0 
            
            # Plot goal position in world coordinates with the calculated markersize
            plt.plot(self.goal_x, self.goal_y, 'ro', markersize=markersize_from_radius, label='Goal')
            plt.legend()
            
            plt.show() 
            self.get_logger().info('Map visualized in Matplotlib')
    
        except Exception as e:
            self.get_logger().error(f'Failed to visualize map in Matplotlib: {str(e)}')

    def load_map_for_rviz(self, pgm_path, yaml_path):
        """Load map and publish as OccupancyGrid for RViz. (NOT DEVELOPED)"""
        pass

    def pose_callback(self, msg):
        """Check if the human has reached the goal area."""
        pass

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    try:
        if node.visualization_mode == 'matplotlib':
            plt.ion()  # Interactive mode for Matplotlib
            rclpy.spin(node)
            plt.close('all')
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()