#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import struct
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class DepthDecompressor(Node):
    """
    ROS 2 Node that subscribes to compressedDepth topic and republishes as Image.
    """

    def __init__(self):
        super().__init__('depth_decompressor')
        self.bridge = CvBridge()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Parameters
        self.declare_parameter('compressed_depth_topic', '/camera/depth/image_rect_raw/compressedDepth')
        self.declare_parameter('decompressed_depth_topic', '/decompressed_depth')

        self.compressed_depth_topic = self.get_parameter('compressed_depth_topic').value
        self.decompressed_depth_topic = self.get_parameter('decompressed_depth_topic').value

        # Publisher
        self.depth_pub = self.create_publisher(Image, self.decompressed_depth_topic, qos)

        # Subscriber
        self.create_subscription(CompressedImage, self.compressed_depth_topic, self.depth_callback, qos)

        self.get_logger().info(
            f"Subscribing to compressedDepth -> {self.compressed_depth_topic}, "
            f"publishing decompressed -> {self.decompressed_depth_topic}"
        )

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
    node = DepthDecompressor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down decompressor")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()