#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import math

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
        
        # Get parameters
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_yaw = math.radians(self.get_parameter('goal_yaw').value)
        self.goal_radius = self.get_parameter('goal_radius').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.goal_frame = self.get_parameter('goal_frame').value
        
        # Publishers and Subscribers
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.goal_reached_publisher = self.create_publisher(Bool, '/goal_reached', 10)
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10)
        
        # Timer for publishing goal
        self.timer = self.create_timer(1.0, self.publish_goal)
        
        self.goal_reached = False
        self.get_logger().info('GoalPublisher node initialized')

    def publish_goal(self):
        """Publish the predetermined goal pose."""
        goal = PoseStamped()
        goal.header.frame_id = self.goal_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self.goal_x
        goal.pose.position.y = self.goal_y
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = math.sin(self.goal_yaw / 2.0)
        goal.pose.orientation.w = math.cos(self.goal_yaw / 2.0)
        self.goal_publisher.publish(goal)

    def pose_callback(self, msg):
        """Check if the human has reached the goal area."""
        if self.goal_reached:
            return
        
        dx = msg.pose.position.x - self.goal_x
        dy = msg.pose.position.y - self.goal_y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance <= self.goal_radius:
            self.goal_reached = True
            goal_reached_msg = Bool()
            goal_reached_msg.data = True
            self.goal_reached_publisher.publish(goal_reached_msg)
            self.get_logger().info('Goal reached!')

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()