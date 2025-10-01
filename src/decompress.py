#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class StereoImageDecompressor(Node):
    def __init__(self):
        super().__init__('stereo_image_decompressor')
        self.bridge = CvBridge()

        # QoS profile (reliable, keep last 10)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Parameters
        self.declare_parameter('input_topic', '/stereo/compressed')
        self.declare_parameter('left_output_topic', '/stereo/left_decompressed')
        self.declare_parameter('right_output_topic', '/stereo/right_decompressed')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)

        self.input_topic = self.get_parameter('input_topic').value
        self.left_output_topic = self.get_parameter('left_output_topic').value
        self.right_output_topic = self.get_parameter('right_output_topic').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value

        # Publishers for decompressed left and right images
        self.left_publisher = self.create_publisher(Image, self.left_output_topic, qos)
        self.right_publisher = self.create_publisher(Image, self.right_output_topic, qos)

        # Subscriber for compressed images
        self.subscription = self.create_subscription(
            CompressedImage, self.input_topic, self.compressed_image_callback, qos
        )

        self.get_logger().info(f"Subscribing to {self.input_topic}, publishing to {self.left_output_topic} and {self.right_output_topic}")

    def compressed_image_callback(self, msg: CompressedImage):
        try:
            # Convert compressed data to numpy array
            np_arr = np.frombuffer(msg.data, np.uint8)
            # Decompress JPEG
            cv_stitched = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_stitched is None:
                self.get_logger().warning("JPEG decompression failed")
                return

            # Optional: Show the stitched image with OpenCV
            cv2.imshow("Decompressed Stitched Image", cv_stitched)
            cv2.waitKey(1)

            # Split into left and right (assuming horizontal stitch)
            cv_left = cv_stitched[:, :self.image_width, :]
            cv_right = cv_stitched[:, self.image_width:, :]

            # Convert to ROS Image messages
            left_msg = self.bridge.cv2_to_imgmsg(cv_left, encoding='bgr8')
            left_msg.header = msg.header

            right_msg = self.bridge.cv2_to_imgmsg(cv_right, encoding='bgr8')
            right_msg.header = msg.header

            # Publish
            self.left_publisher.publish(left_msg)
            self.right_publisher.publish(right_msg)
            self.get_logger().info("Published decompressed left and right images")

        except Exception as e:
            self.get_logger().error(f"Error decompressing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = StereoImageDecompressor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down stereo image decompressor")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()