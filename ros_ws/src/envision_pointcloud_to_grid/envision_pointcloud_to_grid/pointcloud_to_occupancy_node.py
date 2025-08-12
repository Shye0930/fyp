#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from std_srvs.srv import Trigger
import math
import yaml
import os
import cv2
import numpy as np
from scipy.spatial import KDTree

class PointCloudToMap(Node):
    def __init__(self):
        super().__init__('pointcloud_to_occupancy_node')
        
        # Parameters 
        self.declare_parameter('pointcloud_topic', '/orb_slam3/map_points')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'world')
        self.declare_parameter('grid_resolution', 0.05)  # meters per cell
        self.declare_parameter('grid_width', 1000)  # cells
        self.declare_parameter('grid_height', 1000)  # cells
        self.declare_parameter('height_min', 0.0)  # meters
        self.declare_parameter('height_max', 2.0)  # meters
        self.declare_parameter('min_points_per_cell', 5)  # Minimum points to mark cell as occupied
        self.declare_parameter('map_save_path', './maps')
        
        # Get parameters
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.grid_resolution = self.get_parameter('grid_resolution').value
        self.grid_width = int(self.get_parameter('grid_width').value)
        self.grid_height = int(self.get_parameter('grid_height').value)
        self.height_min = self.get_parameter('height_min').value
        self.height_max = self.get_parameter('height_max').value
        self.min_points_per_cell = int(self.get_parameter('min_points_per_cell').value)
        self.map_save_path = os.path.expanduser(self.get_parameter('map_save_path').value)

        # Debug flag:
        self.is_process_points_flag_non_spam = True
        
        # Subscribers and Publishers
        self.subscription = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self.pointcloud_callback,
            10)
        self.grid_map_publisher = self.create_publisher(OccupancyGrid, self.map_topic, 10)
        self.modified_pc2_publisher = self.create_publisher(PointCloud2, "grid_pointcloud", 10)


        # Service for saving map
        self.save_map_service = self.create_service(
            Trigger,
            '/pointcloud/save_map',
            self.save_map_callback)
        
        # TF2 for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Initialize map
        self.occupancy_grid = OccupancyGrid()
        self.occupancy_grid.header.frame_id = self.map_frame
        self.occupancy_grid.info.resolution = self.grid_resolution
        self.occupancy_grid.info.width = self.grid_width
        self.occupancy_grid.info.height = self.grid_height
        self.occupancy_grid.info.origin.position.x = -(self.grid_width * self.grid_resolution) / 2.0
        self.occupancy_grid.info.origin.position.y = -(self.grid_height * self.grid_resolution) / 2.0
        self.occupancy_grid.info.origin.position.z = 0.0
        self.occupancy_grid.info.origin.orientation.w = 1.0
        self.occupancy_grid.info.origin.orientation.x = 0.0
        self.occupancy_grid.info.origin.orientation.y = 0.0
        self.occupancy_grid.info.origin.orientation.z = 0.0
        self.occupancy_grid.data = [-1] * (self.grid_width * self.grid_height)  # Initialize as unknown

        self.topleft_x = (self.grid_width * self.grid_resolution) / 2.0
        self.bottomright_x = -(self.grid_width * self.grid_resolution) / 2.0
        self.topleft_y = (self.grid_height * self.grid_resolution) / 2.0
        self.bottomright_y = -(self.grid_height * self.grid_resolution) / 2.0
        self.cell_num_x = int(self.grid_width / self.grid_resolution)
        self.cell_num_y = int(self.grid_height / self.grid_resolution)
        
        # Point count grid for accumulation
        self.point_counts = np.zeros((self.grid_height, self.grid_width), dtype=np.int32)
        
        # Timer for periodic map publishing
        self.timer = self.create_timer(1.0, self.publish_map)  # Publish at 1 Hz
        
        self.get_logger().info('PointCloudToMap node initialized')

    def pointcloud_callback(self, msg):
        try:
            self.transform_pointcloud(msg)
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')

    def transform_pointcloud(self, msg):
        """Transform point cloud to map frame using tf2."""
        try:
            # Wait for transform
            transform = self.tf_buffer.lookup_transform(
                'world',
                'map',
                msg.header.stamp,
                rclpy.duration.Duration(seconds=1.0)) # Timeout 1 second
            
            # Read points
            points = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                point_stamped = PointStamped()
                point_stamped.header = msg.header
                point_stamped.point.x = point[0]
                point_stamped.point.y = point[1]
                point_stamped.point.z = point[2]
                
                # Transform point
                transformed_point = do_transform_point(point_stamped, transform)
                
                points.append([
                    transformed_point.point.x,
                    transformed_point.point.y,
                    transformed_point.point.z
                ])

            # Filter points by height
            min_z = self.height_min
            max_z = self.height_max
            
            points_np = np.array(points, dtype=np.float32)
            mask = (points_np[:, 2] >= min_z) & (points_np[:, 2] <= max_z)
            points_np = points_np[mask]

            # Delete the outliers
            if points_np.shape[0] >= 6:  # mean_k=5 + 1
                tree = KDTree(points_np)
                dists, _ = tree.query(points_np, k=6)  # k=mean_k + 1 (including self)
                mean_dists = np.mean(dists[:, 1:], axis=1)  # exclude distance to self
                mean = np.mean(mean_dists)
                std = np.std(mean_dists)
                mask = mean_dists <= mean + 1.0 * std  # std_mul=1.0
                points_np = points_np[mask]

            header = msg.header
            header.frame_id = self.map_frame  # Set to the target frame after transformation
            header.stamp = self.get_clock().now().to_msg()  # Use current time;

            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]

            transformed_pc_msg = pc2.create_cloud(header, fields, points_np)
            self.modified_pc2_publisher.publish(transformed_pc_msg)
            
            self.process_points(points_np, msg.header.stamp)
        
        except tf2_ros.LookupException as e:
            self.get_logger().warn(f'Transform not available: {str(e)}')
            self.is_process_points_flag_non_spam = True

        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f'Transform extrapolation error: {str(e)}')
            self.is_process_points_flag_non_spam = True



    def process_points(self, points, stamp):
        """Process points and update occupancy grid."""
        
        if self.is_process_points_flag_non_spam:
            self.get_logger().info("Processing process_points")
            self.is_process_points_flag_non_spam = False
            
        # Reset point counts for new update
        self.point_counts.fill(0)
        
        # Process each point
        for point in points:
            x, y, z = point
            
            # Filter by height
            if self.height_min <= z <= self.height_max:
                if x > 0.01 or x < -0.01 :

                    # Convert to grid coordinates
                    grid_x = int((x - self.occupancy_grid.info.origin.position.x) / self.grid_resolution)
                    grid_y = int((y - self.occupancy_grid.info.origin.position.y) / self.grid_resolution)
                    
                    # Check if within grid bounds
                    if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                        self.point_counts[grid_y, grid_x] += 1
        
        # Update occupancy grid
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                idx = y * self.grid_width + x
                if self.point_counts[y, x] >= self.min_points_per_cell:
                    self.occupancy_grid.data[idx] = 100  # Occupied
                elif self.point_counts[y, x] == 0:
                    self.occupancy_grid.data[idx] = -1  # Unknown
                else:
                    self.occupancy_grid.data[idx] = 0  # Free
        
        # Update header
        self.occupancy_grid.header.stamp = stamp
 
    def save_map_callback(self, request, response):
        """Save the occupancy grid to .pgm and .yaml files."""
        try:
            # Create directory if it doesn't exist
            os.makedirs(os.path.dirname(self.map_save_path), exist_ok=True)
            
            # Convert occupancy grid to image (0=free, 205=unknown, 255=occupied)
            map_array = np.array(self.occupancy_grid.data, dtype=np.int16).reshape(
                (self.grid_height, self.grid_width))
            image = np.zeros((self.grid_height, self.grid_width), dtype=np.uint8)
            image[map_array == -1] = 205  # Unknown (gray)
            image[map_array == 0] = 0     # Free (black)
            image[map_array == 100] = 255 # Occupied (white)
            
            # Flip vertically to match ROS map convention (origin at bottom-left)
            image = np.flipud(np.fliplr(image))  # Flip vertically and horizontally
            
            # Save .pgm file
            pgm_path = self.map_save_path + '.pgm'
            cv2.imwrite(pgm_path, image)
            
            # Save .yaml file
            yaml_path = self.map_save_path + '.yaml'
            map_metadata = {
                'image': os.path.basename(pgm_path),
                'resolution': self.grid_resolution,
                'origin': [
                    self.occupancy_grid.info.origin.position.x, # Negated to match the world frame
                    self.occupancy_grid.info.origin.position.y,
                    0.0
                ],
                'negate': 0,
                'occupied_thresh': 0.65,
                'free_thresh': 0.196
            }
            with open(yaml_path, 'w') as f:
                yaml.dump(map_metadata, f)
            
            response.success = True
            response.message = f'Map saved to {pgm_path} and {yaml_path}'
            self.get_logger().info(response.message)
        
        except Exception as e:
            response.success = False
            response.message = f'Failed to save map: {str(e)}'
            self.get_logger().error(response.message)
        
        return response

    def publish_map(self):
        """Publish the occupancy grid."""
        self.grid_map_publisher.publish(self.occupancy_grid)
        # self.get_logger().info('Published occupancy grid')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()