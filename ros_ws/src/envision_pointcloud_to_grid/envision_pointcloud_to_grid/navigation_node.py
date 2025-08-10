#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from geometry_msgs.msg import PoseArray
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
import heapq

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        # Parameters
        self.declare_parameter('start_x', 1.0)
        self.declare_parameter('start_y', 1.0)
        self.declare_parameter('is_camera_pose_available', False)
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
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.is_camera_pose_available = self.get_parameter('is_camera_pose_available').value
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
        

        # Debug flag:
        self.is_goal_publish_flag_non_spam = True
        self.is_start_publish_flag_non_spam = True

        self.checkpoints = None

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
        self.declare_parameter('start_pose_topic', '/start_pose')

        self.start_pose_publisher = self.create_publisher(PoseStamped, '/start_pose', 10)
        self.checkpoint_publisher = self.create_publisher(PoseArray, '/checkpoints', 10)

        self.publish_goal_pose() 

        self.segment_instructions = []
        self.current_segment = 0

        if self.is_camera_pose_available:
            self.pose_subscription = self.create_subscription(
                PoseStamped,
                self.pose_topic,
                self.pose_callback,
                10)
        else:
            # Load the map first
            self.load_map()
            self.current_pose = Pose()
            self.current_pose.position = Point(x=self.start_x, y=self.start_y, z=1.0)
            self.current_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            self.publish_start_pose()
            self.get_logger().info(f'Using fixed start position: ({self.start_x}, {self.start_y}) since no camera pose is available')
            
            self.get_logger().info("Calculating 90 degree path")
            self.calculate_90_degree_path()

            if self.segment_instructions:
                self.get_logger().info('Navigation instructions:')
                for instr in self.segment_instructions:
                    self.get_logger().info(f'Instruct: {instr}')
                        
        # Timer for publishing path, map, and goal pose
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('Navigation node initialized')
            
    

    # Function to publish start pose using start_x and start_y
    def publish_start_pose(self):
        """Publish the start pose as a PoseStamped message using start_x and start_y parameters."""
        start_pose_msg = PoseStamped()
        start_pose_msg.header.frame_id = self.goal_frame
        start_pose_msg.header.stamp = self.get_clock().now().to_msg()
        start_pose_msg.pose.position = Point(x=self.start_x, y=self.start_y, z=0.0)
        start_pose_msg.pose.orientation = Quaternion(x=0.0, y=-0.707, z=0.0, w=1.0)  # Default orientation (no rotation)

        self.start_pose_publisher.publish(start_pose_msg)
        if self.is_start_publish_flag_non_spam:
            self.is_start_publish_flag_non_spam = False
            self.get_logger().info(f"Published start pose: x={self.start_x}, y={self.start_y}")



    def publish_goal_pose(self):
        """Publish the goal pose as a PoseStamped message."""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.goal_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.goal_x
        goal_pose.pose.position.y = self.goal_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation = Quaternion(x=0.0, y=-0.707, z=0.0, w=1.0)
        self.goal_pose_publisher.publish(goal_pose)

        if self.is_goal_publish_flag_non_spam:
            self.is_goal_publish_flag_non_spam = False
            self.get_logger().info(f'Published goal pose at ({self.goal_x}, {self.goal_y}, {self.goal_yaw} rad)')


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
        normalized_data[map_image == 205] = -1  # Unknown (free floor)
        normalized_data[map_image == 255] = 100 # Occupied (walls)
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
        """Check if a grid cell is occupied, considering user size (0.5m x 0.5m) and 0.1m gap."""
        # User size: 0.5m x 0.5m, safety gap: 0.1m
        user_size_m = 0.3  # User width and height in meters
        safety_gap_m = 0.05 # Safety gap in meters
        user_cells = int(user_size_m / self.grid_resolution)  # Number of cells for 0.5m
        safety_cells = int(safety_gap_m / self.grid_resolution)  # Number of cells for 0.1m
        half_user_cells = user_cells // 2  # Half the user's footprint for centering

        # Check bounds for the user's footprint
        if not (half_user_cells <= grid_x < self.grid_width - half_user_cells and
                half_user_cells <= grid_y < self.grid_height - half_user_cells):
            return True  # Out of bounds is considered occupied

        # Check the user's footprint (0.5m x 0.5m) plus safety gap (0.1m)
        total_cells = half_user_cells + safety_cells
        for dx in range(-total_cells, total_cells + 1):
            for dy in range(-total_cells, total_cells + 1):
                check_x = grid_x + dx
                check_y = grid_y + dy
                if not (0 <= check_x < self.grid_width and 0 <= check_y < self.grid_height):
                    return True  # Out of bounds is occupied
                index = check_y * self.grid_width + check_x
                value = self.occupancy_grid.data[index]
                if value > 0:  # Occupied cell (black pixels)
                    return True
        return False

    def calculate_90_degree_path(self):
        """Calculate a path with 90-degree turns using A* on the grid, minimizing turns first, then steps."""
        if self.current_pose is None or self.occupancy_grid is None:
            self.get_logger().warn('No current pose or occupancy grid available')
            return

        # Handle inconsistency in current_pose type
        if isinstance(self.current_pose, PoseStamped):
            pose = self.current_pose.pose
        else:
            pose = self.current_pose

        start_x = pose.position.x
        start_y = pose.position.y
        end_x = self.goal_x
        end_y = self.goal_y

        self.get_logger().info("Converting world to grid")

        # Convert to grid coordinates
        start_grid = self.world_to_grid(start_x, start_y)
        goal_grid = self.world_to_grid(end_x, end_y)

        if self.is_occupied(start_grid[0], start_grid[1]) or self.is_occupied(goal_grid[0], goal_grid[1]):
            self.get_logger().warn('Start or goal is occupied or too close to obstacles')
            return

        self.get_logger().info("Starting A* pathfinding with minimal turns")

        # Directions: 0: east (1,0), 1: north (0,1), 2: west (-1,0), 3: south (0,-1)
        directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]

        # Turn cost: set higher than max possible steps (e.g., 201 for 100x100 grid, max steps ~200)
        turn_cost = 201
        step_cost = 1

        # A* setup: state is (grid_x, grid_y, dir)
        start_state = (start_grid[0], start_grid[1], self.current_direction)
        frontier = []
        heapq.heappush(frontier, (0, start_state))
        came_from = {start_state: None}
        cost_so_far = {start_state: 0}

        while frontier:
            self.get_logger().info("In frontier....")
            _, current = heapq.heappop(frontier)
            current_x, current_y, current_dir = current

            if (current_x, current_y) == goal_grid:
                break

            for new_dir in range(4):
                dx, dy = directions[new_dir]
                neighbor_x = current_x + dx
                neighbor_y = current_y + dy
                neighbor_state = (neighbor_x, neighbor_y, new_dir)

                if not (0 <= neighbor_x < self.grid_width and 0 <= neighbor_y < self.grid_height):
                    continue
                if self.is_occupied(neighbor_x, neighbor_y):
                    continue

                extra_turn_cost = turn_cost if new_dir != current_dir else 0
                new_cost = cost_so_far[current] + step_cost + extra_turn_cost

                if neighbor_state not in cost_so_far or new_cost < cost_so_far[neighbor_state]:
                    cost_so_far[neighbor_state] = new_cost
                    # Heuristic: Manhattan distance (ignores direction and turns, admissible)
                    manhattan = abs(neighbor_x - goal_grid[0]) + abs(neighbor_y - goal_grid[1])
                    priority = new_cost + manhattan
                    heapq.heappush(frontier, (priority, neighbor_state))
                    came_from[neighbor_state] = current

        if goal_grid not in [(s[0], s[1]) for s in came_from]:
            self.get_logger().warn('No path found to goal')
            return

        # Reconstruct grid path: find the goal state (any dir)
        goal_state = next(s for s in came_from if (s[0], s[1]) == goal_grid)
        grid_path = []
        current = goal_state
        while current is not None:
            grid_path.append((current[0], current[1]))
            current = came_from[current]
        grid_path.reverse()  # From start to goal

        # Convert to world coordinates
        world_path = [self.grid_to_world(gx, gy) for gx, gy in grid_path]

        self.get_logger().info("Creating path message")

        # Create Path message
        path_msg = Path()
        path_msg.header.frame_id = self.goal_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for wx, wy in world_path:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position = Point(x=wx, y=wy, z=0.0)
            pose_stamped.pose.orientation = Quaternion(w=1.0)
            path_msg.poses.append(pose_stamped)

        self.path = path_msg
        self.path_calculated = True

        # Save path to file
        path_data = [{'x': p.pose.position.x, 'y': p.pose.position.y} for p in path_msg.poses]
        with open(self.path_save_path, 'w') as f:
            json.dump(path_data, f)
        self.get_logger().info(f'Path saved to {self.path_save_path}')

        # Find checkpoints: start, turn points, goal
        checkpoints_grid = [grid_path[0]]
        for i in range(1, len(grid_path) - 1):
            prev = grid_path[i - 1]
            curr = grid_path[i]
            next_p = grid_path[i + 1]
            dx1, dy1 = curr[0] - prev[0], curr[1] - prev[1]
            dx2, dy2 = next_p[0] - curr[0], next_p[1] - curr[1]
            if (dx1, dy1) != (dx2, dy2):
                checkpoints_grid.append(curr)
        checkpoints_grid.append(grid_path[-1])

        # Convert to unique world checkpoints
        unique_checkpoints = [self.grid_to_world(gx, gy) for gx, gy in checkpoints_grid]

        # Publish checkpoints
        cp_msg = PoseArray()
        cp_msg.header = path_msg.header
        for cx, cy in unique_checkpoints:
            pose = Pose()
            pose.position = Point(x=cx, y=cy, z=0.0)
            pose.orientation = Quaternion(w=1.0)
            cp_msg.poses.append(pose)
        self.checkpoints = cp_msg

        self.get_logger().info("Generating instructions")
        self.segment_instructions = []

        # Generate instructions based on checkpoints
        direction_map = {(1, 0): 0, (-1, 0): 2, (0, 1): 1, (0, -1): 3}  # east, west, north, south
        current_dir = self.current_direction

        for i in range(len(checkpoints_grid) - 1):
            start_cp = checkpoints_grid[i]
            end_cp = checkpoints_grid[i + 1]
            dx = 1 if end_cp[0] > start_cp[0] else -1 if end_cp[0] < start_cp[0] else 0
            dy = 1 if end_cp[1] > start_cp[1] else -1 if end_cp[1] < start_cp[1] else 0
            seg_dir = direction_map.get((dx, dy))
            if seg_dir is None:
                continue  # Skip if no movement
            dist = math.sqrt((self.grid_resolution * (end_cp[0] - start_cp[0]))**2 + (self.grid_resolution * (end_cp[1] - start_cp[1]))**2)

            if dist > 0:
                turn = (seg_dir - current_dir) % 4
                instruction = ''
                if turn == 1:
                    self.get_logger().info('Turn right')
                    instruction += 'Turn right\n'
                elif turn == 3:
                    self.get_logger().info('Turn left')
                    instruction += 'Turn left\n'
                elif turn == 2:
                    self.get_logger().info('Turn around')
                    instruction += 'Turn around\n'
                self.get_logger().info(f'Walk approximately {dist:.1f} meters')
                instruction += f'Walk approximately {dist:.1f} meters'
                self.segment_instructions.append(instruction)
                current_dir = seg_dir

        self.current_direction = current_dir


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
                    self.get_logger().info('Calculating path w/ loaded map ...')
                    self.calculate_90_degree_path()
                else:
                    self.get_logger().error('Failed to load map. Map is required for path calculation.')
                    return
            else:
                self.get_logger().info('Calculating path...')
                self.calculate_90_degree_path()
            
            # After calculating path, output the first instruction if available
            if self.segment_instructions:
                self.get_logger().info('Starting navigation with first instruction:')
                self.get_logger().info(self.segment_instructions[0])
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
            
            # Check if reached the next checkpoint
            if self.current_segment < len(self.segment_instructions):
                next_cp_index = self.current_segment + 1
                if next_cp_index < len(self.checkpoints.poses):
                    next_cp = self.checkpoints.poses[next_cp_index]
                    dx = msg.pose.position.x - next_cp.position.x
                    dy = msg.pose.position.y - next_cp.position.y
                    dist = math.sqrt(dx**2 + dy**2)
                    if dist <= self.goal_radius:
                        self.current_segment += 1
                        if self.current_segment < len(self.segment_instructions):
                            # TODO: This is where i add txt to speech
                            self.get_logger().info('Reached checkpoint! Next instruction:')
                            self.get_logger().info(self.segment_instructions[self.current_segment])

    def timer_callback(self):
        """Publish the planned path, map, and goal pose if calculated."""
        if self.occupancy_grid is not None:
            self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
            self.map_publisher.publish(self.occupancy_grid)  # Publish map
        if self.path_calculated and self.path is not None:
            self.path.header.stamp = self.get_clock().now().to_msg()
            self.path_publisher.publish(self.path)
        self.publish_goal_pose()  # Publish goal pose periodically

        if self.checkpoints is not None:
            self.checkpoints.header.stamp = self.get_clock().now().to_msg()
            self.checkpoint_publisher.publish(self.checkpoints)

        if not self.is_camera_pose_available:
            self.publish_start_pose()

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()