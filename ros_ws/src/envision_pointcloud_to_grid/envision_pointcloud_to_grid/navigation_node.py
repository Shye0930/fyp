#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Bool
import os
import cv2
import yaml
import numpy as np
import json
from geometry_msgs.msg import Pose, Point, Quaternion
import math
import tf2_ros

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        # Parameters
        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 3.0)
        self.declare_parameter('goal_yaw', 90.0)  # Degrees
        self.declare_parameter('pose_topic', '/orb_slam3/camera_pose')
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('map_path', '~/Desktop/fyp/maps/occu_map')
        self.declare_parameter('grid_resolution', 0.1)  # meters per cell
        self.declare_parameter('grid_width', 100)       # cells
        self.declare_parameter('grid_height', 100)      # cells
        self.declare_parameter('goal_radius', 0.5)      # Meters
        self.declare_parameter('path_save_path', '~/Desktop/fyp/maps/path.json')

        # Get parameters
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_yaw = math.radians(self.get_parameter('goal_yaw').value)
        self.pose_topic = self.get_parameter('pose_topic').value
        self.goal_frame = self.get_parameter('goal_frame').value
        self.map_path = os.path.expanduser(self.get_parameter('map_path').value)
        self.grid_resolution = self.get_parameter('grid_resolution').value
        self.grid_width = self.get_parameter('grid_width').value
        self.grid_height = self.get_parameter('grid_height').value
        self.goal_radius = self.get_parameter('goal_radius').value
        self.path_save_path = os.path.expanduser(self.get_parameter('path_save_path').value)

        # State variables
        self.current_pose = None
        self.occupancy_grid = None
        self.path = None
        self.path_calculated = False
        self.current_direction = 0  # 0: east, 1: north, 2: west, 3: south (in radians: 0, pi/2, pi, 3pi/2)

        # Publishers and Subscribers
        self.path_publisher = self.create_publisher(Path, '/planned_path', 10)
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)  # Publisher for map
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)  # Publisher for goal pose
        self.goal_reached_publisher = self.create_publisher(Bool, '/goal_reached', 10)
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10)

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer for publishing path, map, and goal pose
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('Navigation node initialized')
        self.publish_goal_pose()  # Publish goal pose on startup

    def publish_goal_pose(self):
        """Publish the goal pose as a PoseStamped message."""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.goal_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.goal_x
        goal_pose.pose.position.y = self.goal_y
        goal_pose.pose.position.z = 0.0
        quaternion = self.quaternion_from_euler(0, 0, self.goal_yaw)
        goal_pose.pose.orientation = Quaternion(x=0.0, y=-0.707, z=0.0, w=1.0)
        self.goal_pose_publisher.publish(goal_pose)
        # self.get_logger().info(f'Published goal pose at ({self.goal_x}, {self.goal_y}, {self.goal_yaw} rad)')

    def quaternion_from_euler(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0.0, 0.0, 0.0, 0.0]
        q[0] = sr * cp * cy - cr * sp * sy
        q[1] = cr * sp * cy + sr * cp * sy
        q[2] = cr * cp * sy - sr * sp * cy
        q[3] = cr * cp * cy + sr * sp * sy
        return q

    def load_map(self):
        """Load the occupancy grid from .pgm and .yaml files."""
        pgm_path = self.map_path + '.pgm'
        yaml_path = self.map_path + '.yaml'

        if not (os.path.exists(pgm_path) and os.path.exists(yaml_path)):
            self.get_logger().error(f'Map files not found: {pgm_path}, {yaml_path}')
            return False

        # Read .pgm file
        map_image = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if map_image is None:
            self.get_logger().error(f'Failed to load .pgm file: {pgm_path}')
            return False

        # Flip image to match ROS convention (origin at bottom-left)
        map_image = np.flipud(map_image)

        # Pad or crop to match grid size
        if map_image.shape[1] != self.grid_width or map_image.shape[0] != self.grid_height:
            padded_image = np.full((self.grid_height, self.grid_width), 205, dtype=np.uint8)
            h_offset = (self.grid_height - map_image.shape[0]) // 2
            w_offset = (self.grid_width - map_image.shape[1]) // 2
            padded_image[h_offset:h_offset + map_image.shape[0], w_offset:w_offset + map_image.shape[1]] = map_image
            map_image = padded_image

        # Read .yaml file for origin
        with open(yaml_path, 'r') as f:
            map_metadata = yaml.safe_load(f)
        origin_x = map_metadata['origin'][0]
        origin_y = map_metadata['origin'][1]
        self.get_logger().info(f'Loaded map origin: ({origin_x}, {origin_y})')

        # Normalize map values to ROS OccupancyGrid convention
        # 0 = free, 205 = unknown, 255 = occupied
        normalized_data = np.zeros_like(map_image, dtype=np.int8)
        normalized_data[map_image == 0] = 0    # Free space
        normalized_data[map_image == 205] = -1  # Unknown
        normalized_data[map_image == 255] = 100 # Occupied
        # Clamp any other values to [-1, 100] to fit the range
        normalized_data = np.clip(normalized_data, -1, 100)

        # Create OccupancyGrid
        self.occupancy_grid = OccupancyGrid()
        self.occupancy_grid.header.frame_id = self.goal_frame
        self.occupancy_grid.info.resolution = self.grid_resolution
        self.occupancy_grid.info.width = self.grid_width
        self.occupancy_grid.info.height = self.grid_height
        self.occupancy_grid.info.origin.position.x = -self.grid_width * self.grid_resolution / 2.0
        self.occupancy_grid.info.origin.position.y = -self.grid_height * self.grid_resolution / 2.0
        self.occupancy_grid.info.origin.position.z = 0.0
        self.occupancy_grid.info.origin.orientation.w = 1.0
        self.occupancy_grid.info.origin.orientation.x = 0.0
        self.occupancy_grid.info.origin.orientation.y = 0.0
        self.occupancy_grid.info.origin.orientation.z = 0.0
        self.occupancy_grid.data = normalized_data.flatten().tolist()

        self.get_logger().info('Map loaded successfully')
        return True

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid indices, accounting for origin."""
        grid_x = int((x - self.occupancy_grid.info.origin.position.x) / self.grid_resolution)
        grid_y = int((y - self.occupancy_grid.info.origin.position.y) / self.grid_resolution)
        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y):
        """Convert grid indices to world coordinates."""
        world_x = self.occupancy_grid.info.origin.position.x + grid_x * self.grid_resolution
        world_y = self.occupancy_grid.info.origin.position.y + grid_y * self.grid_resolution
        return world_x, world_y

    def is_occupied(self, grid_x, grid_y):
        """Check if a grid cell is occupied."""
        if (0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height):
            index = grid_y * self.grid_width + grid_x
            value = self.occupancy_grid.data[index]
            return value > 0 or value == -1  # Treat occupied (>0) or unknown (-1) as occupied
        return True  # Out of bounds is considered occupied

    def bresenham_line(self, x0, y0, x1, y1):
        """Generate points along a straight line using Bresenham's algorithm."""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return points

    def calculate_90_degree_path(self):
        """Calculate a path with 90-degree turns using a grid-based approach."""
        if self.current_pose is None or self.occupancy_grid is None:
            return

        start_x = self.current_pose.pose.position.x
        start_y = self.current_pose.pose.position.y
        end_x = self.goal_x
        end_y = self.goal_y

        # Convert to grid coordinates
        start_grid_x, start_grid_y = self.world_to_grid(start_x, start_y)
        end_grid_x, end_grid_y = self.world_to_grid(end_x, end_y)

        # Initialize path and instructions
        path_points = [(start_grid_x, start_grid_y)]
        instructions = []
        current_x, current_y = start_grid_x, start_grid_y
        target_x, target_y = end_grid_x, end_grid_y
        direction = self.current_direction  # Start with initial direction

        while current_x != target_x or current_y != target_y:
            # Determine the next move (prefer horizontal or vertical)
            dx = target_x - current_x
            dy = target_y - current_y

            if abs(dx) > abs(dy):
                # Move horizontally
                next_x = current_x + (1 if dx > 0 else -1)
                next_y = current_y
                next_direction = 0 if dx > 0 else 2  # 0: east, 2: west
            else:
                # Move vertically
                next_x = current_x
                next_y = current_y + (1 if dy > 0 else -1)
                next_direction = 1 if dy > 0 else 3  # 1: north, 3: south

            # Check path using Bresenham's line
            line_points = self.bresenham_line(current_x, current_y, next_x, next_y)
            clear_path = True
            for grid_x, grid_y in line_points[1:]:  # Skip start point
                if self.is_occupied(grid_x, grid_y):
                    clear_path = False
                    break

            if clear_path:
                current_x, current_y = next_x, next_y
                path_points.append((current_x, current_y))
                if next_direction != direction:
                    turn = (next_direction - direction) % 4
                    if turn == 1 or turn == 3:
                        instructions.append("turn right")
                    elif turn == 2:
                        instructions.append("turn around")
                    elif turn == -1 or turn == 3:
                        instructions.append("turn left")
                    direction = next_direction
            else:
                # Obstacle detected, try the other direction
                if abs(dx) > abs(dy):
                    next_y = current_y + (1 if dy > 0 else -1)
                    next_x = current_x
                    next_direction = 1 if dy > 0 else 3
                else:
                    next_x = current_x + (1 if dx > 0 else -1)
                    next_y = current_y
                    next_direction = 0 if dx > 0 else 2

                line_points = self.bresenham_line(current_x, current_y, next_x, next_y)
                clear_path = True
                for grid_x, grid_y in line_points[1:]:
                    if self.is_occupied(grid_x, grid_y):
                        clear_path = False
                        break

                if clear_path:
                    current_x, current_y = next_x, next_y
                    path_points.append((current_x, current_y))
                    if next_direction != direction:
                        turn = (next_direction - direction) % 4
                        if turn == 1 or turn == 3:
                            instructions.append("turn right")
                        elif turn == 2:
                            instructions.append("turn around")
                        elif turn == -1 or turn == 3:
                            instructions.append("turn left")
                        direction = next_direction
                else:
                    self.get_logger().warn(f'No clear path from ({current_x}, {current_y}) to goal')
                    break

        # Convert path to world coordinates and calculate distances
        world_path = []
        for i in range(len(path_points)):
            x, y = self.grid_to_world(path_points[i][0], path_points[i][1])
            world_path.append((x, y))
            if i > 0:
                prev_x, prev_y = world_path[i-1]
                distance = math.sqrt((x - prev_x)**2 + (y - prev_y)**2)
                if instructions:
                    self.get_logger().info(f"{instructions[i-1]}, walk approximately {distance:.1f} meters")

        # Create Path message
        path_msg = Path()
        path_msg.header.frame_id = self.goal_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in world_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position = Point(x=x, y=y, z=0.0)
            pose.pose.orientation = Quaternion(w=1.0)  # No rotation for simplicity
            path_msg.poses.append(pose)

        self.path = path_msg
        self.path_calculated = True
        self.current_direction = direction  # Update current direction

        # Save path to file
        path_data = [{'x': p.pose.position.x, 'y': p.pose.position.y} for p in path_msg.poses]
        with open(self.path_save_path, 'w') as f:
            json.dump(path_data, f)
        self.get_logger().info(f'Path saved to {self.path_save_path}')

    def load_saved_path(self):
        """Load a previously saved path if it exists."""
        if os.path.exists(self.path_save_path):
            with open(self.path_save_path, 'r') as f:
                path_data = json.load(f)
                path_msg = Path()
                path_msg.header.frame_id = self.goal_frame
                path_msg.header.stamp = self.get_clock().now().to_msg()
                for point in path_data:
                    pose = PoseStamped()
                    pose.header = path_msg.header
                    pose.pose.position = Point(x=point['x'], y=point['y'], z=0.0)
                    pose.pose.orientation = Quaternion(w=1.0)
                    path_msg.poses.append(pose)
                self.path = path_msg
                self.path_calculated = True
                self.get_logger().info(f'Loaded saved path from {self.path_save_path}')
            return True
        return False

    def pose_callback(self, msg):
        """Handle camera pose updates and trigger path calculation."""
        self.current_pose = msg
        if not self.path_calculated:
            if self.occupancy_grid is None:
                if self.load_map():
                    self.calculate_90_degree_path()
            else:
                self.calculate_90_degree_path()
        else:
            # Check if goal is reached
            dx = msg.pose.position.x - self.goal_x
            dy = msg.pose.position.y - self.goal_y
            distance = math.sqrt(dx**2 + dy**2)
            if distance <= self.goal_radius:
                goal_reached_msg = Bool()
                goal_reached_msg.data = True
                self.goal_reached_publisher.publish(goal_reached_msg)
                self.get_logger().info('Goal reached!')

    def timer_callback(self):
        """Publish the planned path, map, and goal pose if calculated."""
        if self.occupancy_grid is not None:
            self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
            self.map_publisher.publish(self.occupancy_grid)  # Publish map
        if self.path_calculated and self.path is not None:
            self.path.header.stamp = self.get_clock().now().to_msg()
            self.path_publisher.publish(self.path)
        self.publish_goal_pose()  # Publish goal pose periodically

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()