#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import struct
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class RealsenseImageCompressor(Node):
    """
    ROS 2 Node that subscribes to RealSense RGB/Depth topics
    and republishes them as CompressedImage messages.

    - RGB: JPEG compression
    - Depth: PNG compression with standard compressed_depth_image_transport header (for 16UC1)
    """

    def __init__(self):
        super().__init__('realsense_image_compressor')
        self.bridge = CvBridge()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Parameters
        self.declare_parameter('rgb_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('rgb_compressed_topic', '/camera/color/image_raw/compressed')
        self.declare_parameter('depth_compressed_topic', '/camera/depth/image_rect_raw/compressedDepth')
        self.declare_parameter('jpeg_quality', 100)
        self.declare_parameter('png_level', 1)  # PNG compression level (0-9, lower is faster)

        self.rgb_topic = self.get_parameter('rgb_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.rgb_compressed_topic = self.get_parameter('rgb_compressed_topic').value
        self.depth_compressed_topic = self.get_parameter('depth_compressed_topic').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.png_level = self.get_parameter('png_level').value

        # Publishers
        self.rgb_pub = self.create_publisher(CompressedImage, self.rgb_compressed_topic, qos)
        self.depth_pub = self.create_publisher(CompressedImage, self.depth_compressed_topic, qos)

        # Subscribers
        self.create_subscription(Image, self.rgb_topic, self.rgb_callback, qos)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, qos)

        self.get_logger().info(
            f"Publishing compressed RGB -> {self.rgb_compressed_topic}, "
            f"compressedDepth -> {self.depth_compressed_topic}"
        )

    def rgb_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ret, jpeg_buffer = cv2.imencode('.jpg', cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if not ret:
                self.get_logger().warn("JPEG compression failed (RGB)")
                return

            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = "jpeg"
            compressed_msg.data = jpeg_buffer.tobytes()
            self.rgb_pub.publish(compressed_msg)
        except Exception as e:
            self.get_logger().error(f"Error compressing RGB: {e}")

    def depth_callback(self, msg: Image):
        """
        Compress 16UC1 depth as PNG with standard compressed_depth_image_transport format.
        Prepends 12-byte binary header to PNG data.
        """
        try:
            if msg.encoding not in ('16UC1', 'mono16'):
                self.get_logger().warn(f"Depth encoding {msg.encoding} not supported, expected 16UC1/mono16")
                return

            # Convert to numpy
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if cv_depth.dtype != np.uint16:
                cv_depth = cv_depth.astype(np.uint16)

            # Encode as PNG (lossless)
            params = [int(cv2.IMWRITE_PNG_COMPRESSION), self.png_level]
            ret, png_buffer = cv2.imencode('.png', cv_depth, params)
            if not ret:
                self.get_logger().warn("PNG compression failed (Depth)")
                return

            # Standard header (12 bytes): uint32 format (0 for PNG_RAW), float depthParam[2] (unused, set to 0.0)
            COMPRESSION_FORMAT_PNG_RAW = 0
            depth_param1 = 0.0
            depth_param2 = 0.0
            header = struct.pack('<Iff', COMPRESSION_FORMAT_PNG_RAW, depth_param1, depth_param2)

            # Concatenate header + PNG data
            combined = header + png_buffer.tobytes()

            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = "16UC1; compressedDepth"
            compressed_msg.data = combined

            self.depth_pub.publish(compressed_msg)
        except Exception as e:
            self.get_logger().error(f"Error compressing depth: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RealsenseImageCompressor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down compressor")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()