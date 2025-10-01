#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.clock import Clock

class ImageDecompressor(Node):
    def __init__(self):
        super().__init__('image_decompressor')
        self.bridge = CvBridge()

        # QoS profile for sensor data (best effort, keep last 10)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Parameters
        self.declare_parameter('input_topic', '/camera/compressed')
        self.declare_parameter('output_topic', '/camera/image_decompressed')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # Publisher for decompressed (raw) images
        self.publisher = self.create_publisher(Image, output_topic, qos)
        # Subscriber for compressed images
        self.subscription = self.create_subscription(
            CompressedImage, input_topic, self.compressed_image_callback, qos
        )

        self.get_logger().info(f"Subscribing to {input_topic}, publishing to {output_topic}")

    def decompress_and_publish(self, msg: CompressedImage, publisher, label: str):
        try:
            # Convert compressed data to numpy array
            np_arr = np.frombuffer(msg.data, np.uint8)
            # Decompress JPEG
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None:
                self.get_logger().warning(f"{label}: JPEG decompression failed")
                return

            # Show with OpenCV
            cv2.imshow("Decompressed Image", cv_image)
            cv2.waitKey(1)   # required to refresh the imshow window

            # Convert OpenCV image to ROS Image message
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='mono8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header = msg.header
            publisher.publish(image_msg)
            self.get_logger().info(f"{label}: Published decompressed image")

        except Exception as e:
            self.get_logger().error(f"{label}: Error decompressing image: {str(e)}")

    def compressed_image_callback(self, msg: CompressedImage):
        self.decompress_and_publish(msg, self.publisher, "Color Image")

def main(args=None):
    rclpy.init(args=args)
    node = ImageDecompressor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down image decompressor")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()