#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np
import struct
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class RealsenseImageDecompressor(Node):
    """
    ROS 2 Node that subscribes to compressed RGB and Depth topics and republishes as Image.
    """

    def __init__(self):
        super().__init__('realsense_image_decompressor')
        self.bridge = CvBridge()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Parameters for RGB
        self.declare_parameter('compressed_rgb_topic', '/camera/color/image_raw/compressed')
        self.declare_parameter('decompressed_rgb_topic', '/decompressed_rgb')

        # Parameters for Depth
        self.declare_parameter('compressed_depth_topic', '/camera/depth/image_rect_raw/compressedDepth')
        self.declare_parameter('decompressed_depth_topic', '/decompressed_depth')

        self.compressed_rgb_topic = self.get_parameter('compressed_rgb_topic').value
        self.decompressed_rgb_topic = self.get_parameter('decompressed_rgb_topic').value
        self.compressed_depth_topic = self.get_parameter('compressed_depth_topic').value
        self.decompressed_depth_topic = self.get_parameter('decompressed_depth_topic').value

        # Publishers
        self.rgb_pub = self.create_publisher(Image, self.decompressed_rgb_topic, qos)
        self.depth_pub = self.create_publisher(Image, self.decompressed_depth_topic, qos)

        # Subscribers
        self.create_subscription(CompressedImage, self.compressed_rgb_topic, self.rgb_callback, qos)
        self.create_subscription(CompressedImage, self.compressed_depth_topic, self.depth_callback, qos)

        self.get_logger().info(
            f"Subscribing to compressed RGB -> {self.compressed_rgb_topic}, "
            f"compressedDepth -> {self.compressed_depth_topic}, "
            f"publishing decompressed RGB -> {self.decompressed_rgb_topic}, "
            f"decompressed Depth -> {self.decompressed_depth_topic}"
        )

    def rgb_callback(self, msg: CompressedImage):
        try:
            if "jpeg" not in msg.format.lower():
                self.get_logger().warn("Message format not JPEG")
                return

            # Decode JPEG to OpenCV image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_img is None:
                self.get_logger().warn("JPEG decode failed")
                return

            # Convert back to Image msg (BGR8 encoding)
            decompressed_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
            decompressed_msg.header = msg.header
            self.rgb_pub.publish(decompressed_msg)
        except Exception as e:
            self.get_logger().error(f"Error decompressing RGB: {e}")

    def depth_callback(self, msg: CompressedImage):
        try:
            if "compressedDepth" not in msg.format:
                self.get_logger().warn("Message format not compressedDepth")
                return

            data = msg.data

            # Extract 12-byte header
            if len(data) < 12:
                self.get_logger().warn("Compressed data too short for header")
                return
            header = data[:12]
            png_data = np.frombuffer(data[12:], dtype=np.uint8)

            # Unpack header: uint32 format_code, float param1, float param2
            format_code, param1, param2 = struct.unpack('<Iff', header)

            # For PNG_RAW (0), direct decode
            if format_code == 0:  # PNG_RAW for 16UC1
                cv_depth = cv2.imdecode(png_data, cv2.IMREAD_UNCHANGED)
                if cv_depth is None:
                    self.get_logger().warn("PNG decode failed")
                    return
                if cv_depth.dtype != np.uint16:
                    self.get_logger().warn("Decoded image not uint16")
                    return

                # Convert back to Image msg
                decompressed_msg = self.bridge.cv2_to_imgmsg(cv_depth, encoding='16UC1')
                decompressed_msg.header = msg.header
                self.depth_pub.publish(decompressed_msg)
            else:
                self.get_logger().warn(f"Unsupported compression format code: {format_code}")
        except Exception as e:
            self.get_logger().error(f"Error decompressing depth: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RealsenseImageDecompressor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down decompressor")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()