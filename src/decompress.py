#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import torch

# --- Global GPU/CPU setup ---
GPU_SUPPORT = False
CUPY_AVAILABLE = False
try:
    import cupy as cp
    if torch.cuda.is_available():
        CUPY_AVAILABLE = cp.cuda.runtime.getDeviceCount() > 0
        if CUPY_AVAILABLE:
            GPU_SUPPORT = True
            print("CUDA GPU is available. Using CuPy for acceleration.")
        else:
            print("CUDA GPU is not available. Using CPU.")
    else:
        print("CUDA GPU is not available. Using CPU.")
except ImportError:
    print("CuPy not installed. Using CPU.")

# Make sure GPU_SUPPORT is correctly set
if not GPU_SUPPORT and torch.backends.mps.is_available():
    GPU_SUPPORT = True
    print("Apple MPS GPU is available. Using PyTorch with MPS.")


class StereoImageDecompressor(Node):
    def __init__(self):
        super().__init__('stereo_image_decompressor')
        self.bridge = CvBridge()

        if torch.cuda.is_available():
            # Check for CUDA and CuPy availability
            self.cuda_available = True
            self.cupy_available = True
            self.get_logger().info(f"CUDA available: {self.cuda_available}, CuPy available: {self.cupy_available}")
        else:
            self.cupy_available = False
            self.cuda_available = False
            self.get_logger().info("CUDA not available, falling back to CPU")

        # QoS profile (reliable, keep last 10)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Parameters
        self.declare_parameter('input_topic', '/camera/stereo/compressed')
        self.declare_parameter('left_output_topic', '/camera/stereo/left_decompressed')
        self.declare_parameter('right_output_topic', '/camera/stereo/right_decompressed')
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
            if self.cuda_available:
                try:
                    # Use OpenCV CUDA for JPEG decoding
                    gpu_mat = cv2.cuda_GpuMat()
                    gpu_mat.upload(np_arr)
                    cv_stitched_gpu = cv2.cuda.imdecode(gpu_mat, cv2.IMREAD_COLOR)
                    if cv_stitched_gpu is None:
                        self.get_logger().warning("CUDA JPEG decompression failed, falling back to CPU")
                        cv_stitched = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    else:
                        if self.cupy_available:
                            # Transfer to CuPy for GPU-based splitting
                            cv_stitched = cp.asarray(cv_stitched_gpu.download())
                        else:
                            cv_stitched = cv_stitched_gpu.download()
                except Exception as cuda_e:
                    self.get_logger().warning(f"CUDA error: {str(cuda_e)}, falling back to CPU")
                    cv_stitched = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                cv_stitched = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_stitched is None:
                self.get_logger().warning("JPEG decompression failed")
                return

            # Verify stitched image dimensions
            if self.cupy_available:
                expected_shape = (self.image_height, 2 * self.image_width, 3)
                if cv_stitched.shape != expected_shape:
                    self.get_logger().warning(f"Unexpected stitched image size: {cv_stitched.shape}, expected {expected_shape}")
                    return
                # Split on GPU using CuPy
                cv_left = cv_stitched[:, :self.image_width, :].get()  # Transfer to CPU
                cv_right = cv_stitched[:, self.image_width:, :].get()  # Transfer to CPU
            else:
                if cv_stitched.shape[1] != 2 * self.image_width or cv_stitched.shape[0] != self.image_height:
                    self.get_logger().warning(f"Unexpected stitched image size: {cv_stitched.shape}, expected ({self.image_height}, {2 * self.image_width}, 3)")
                    return
                # Split on CPU
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

