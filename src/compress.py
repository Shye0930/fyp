#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data

class ImageCompressor(Node):
    def __init__(self):
        super().__init__('image_compressor')
        self.bridge = CvBridge()

        # QoS profile for sensor data (best effort, keep last 10)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Parameters
        self.declare_parameter('input_topic', '/camera/camera/infra1/image_rect_raw')
        self.declare_parameter('output_topic', '/camera/compressed')
        self.declare_parameter('jpeg_quality', 80)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        # Publisher for compressed images
        self.publisher = self.create_publisher(CompressedImage, output_topic, qos_profile_sensor_data)
        # Subscriber for raw images
        self.subscription = self.create_subscription(
            Image, input_topic, self.image_callback, qos_profile_sensor_data
        )

        self.get_logger().info(f"Subscribing to {input_topic}, publishing to {output_topic}")

    def compress_and_publish(self, msg: Image, publisher, label: str, prefix_char: bytes):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Compress to JPEG
            ret, jpeg_buffer = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if not ret:
                self.get_logger().warning(f"{label}: JPEG compression failed")
                return

            jpeg_bytes = jpeg_buffer.tobytes()

            # Publish to /compressed topic
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = 'jpeg'
            compressed_msg.data = jpeg_bytes
            publisher.publish(compressed_msg)
            self.get_logger().info(f"{label}: Published compressed image")

        except Exception as e:
            self.get_logger().error(f"{label}: Error processing image: {str(e)}")

    def image_callback(self, msg: Image):
        self.compress_and_publish(msg, self.publisher, "Color Image", b'')

def main(args=None):
    rclpy.init(args=args)
    node = ImageCompressor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down image compressor")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()