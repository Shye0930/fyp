#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import message_filters

class StereoImageCompressor(Node):
    def __init__(self):
        super().__init__('stereo_image_compressor')
        self.bridge = CvBridge()

        # QoS profile (reliable, keep last 10)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Parameters
        self.declare_parameter('left_topic', '/camera/camera/infra1/image_rect_raw')
        self.declare_parameter('right_topic', '/camera/camera/infra2/image_rect_raw')
        self.declare_parameter('compressed_topic', '/camera/stereo/compressed')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('jpeg_quality', 80)

        self.left_topic = self.get_parameter('left_topic').value
        self.right_topic = self.get_parameter('right_topic').value
        self.compressed_topic = self.get_parameter('compressed_topic').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        # Publisher for compressed stitched images
        self.publisher = self.create_publisher(CompressedImage, self.compressed_topic, qos)

        # Subscribers for left and right images with synchronization
        left_sub = message_filters.Subscriber(self, Image, self.left_topic, qos_profile=qos)
        right_sub = message_filters.Subscriber(self, Image, self.right_topic, qos_profile=qos)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [left_sub, right_sub], queue_size=10, slop=0.05
        )
        self.synchronizer.registerCallback(self.synced_callback)

        self.get_logger().info(f"Subscribing to {self.left_topic} and {self.right_topic}, publishing to {self.compressed_topic}")

    def synced_callback(self, left_msg: Image, right_msg: Image):
        try:
            # Convert ROS Images to OpenCV
            cv_left = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='bgr8')
            cv_right = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='bgr8')

            # Optional: Resize if necessary (assuming input images match params)
            cv_left = cv2.resize(cv_left, (self.image_width, self.image_height))
            cv_right = cv2.resize(cv_right, (self.image_width, self.image_height))

            # Stitch horizontally (left + right)
            cv_stitched = cv2.hconcat([cv_left, cv_right])

            # Compress to JPEG
            ret, jpeg_buffer = cv2.imencode('.jpg', cv_stitched, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if not ret:
                self.get_logger().warning("JPEG compression failed")
                return

            jpeg_bytes = jpeg_buffer.tobytes()

            # Publish compressed message
            compressed_msg = CompressedImage()
            compressed_msg.header = left_msg.header  # Use left header for timestamp/sync
            compressed_msg.format = 'jpeg'
            compressed_msg.data = jpeg_bytes
            self.publisher.publish(compressed_msg)
            self.get_logger().info("Published compressed stitched image")

        except Exception as e:
            self.get_logger().error(f"Error processing images: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = StereoImageCompressor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down stereo image compressor")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()