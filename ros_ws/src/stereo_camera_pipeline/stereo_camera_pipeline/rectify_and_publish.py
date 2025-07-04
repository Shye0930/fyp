import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import message_filters

class StereoRectifierNode(Node):
    def __init__(self):
        super().__init__('stereo_rectifier_node')

        # Declare parameters for calibration file and topics
        self.declare_parameter('calibration_file', 'stereo_calibration.yaml')
        self.declare_parameter('left_raw_topic', '/stereo/left/image_raw')
        self.declare_parameter('right_raw_topic', '/stereo/right/image_raw')
        self.declare_parameter('left_rectified_topic', '/stereo/left/rectified_images')
        self.declare_parameter('right_rectified_topic', '/stereo/right/rectified_images')

        # Get parameter values
        self.calibration_file = self.get_parameter('calibration_file').get_parameter_value().string_value
        self.left_raw_topic = self.get_parameter('left_raw_topic').get_parameter_value().string_value
        self.right_raw_topic = self.get_parameter('right_raw_topic').get_parameter_value().string_value
        self.left_rectified_topic = self.get_parameter('left_rectified_topic').get_parameter_value().string_value
        self.right_rectified_topic = self.get_parameter('right_rectified_topic').get_parameter_value().string_value

        self.get_logger().info(f"[INFO] Loading calibration from: {self.calibration_file}")

        self.cv_bridge = CvBridge()
        self.M1l = None
        self.M2l = None
        self.M1r = None
        self.M2r = None

        self.load_and_init_rectification()

        # QoS profile for image topics
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10 # Matches the queue size in your C++ node
        )

        # Subscribers
        self.left_sub = message_filters.Subscriber(
            self, Image, self.left_raw_topic, qos_profile=qos_profile)
        self.right_sub = message_filters.Subscriber(
            self, Image, self.right_raw_topic, qos_profile=qos_profile)

        # Approximate Time Synchronizer
        # The queue_size should be sufficient to handle potential message arrival delays
        # You might need to tune this if you experience "Dropped message" warnings
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub], 
            queue_size=10, 
            slop=0.1 # Max time difference between messages to be synchronized (seconds)
        )
        self.ts.registerCallback(self.stereo_callback)

        # Publishers for rectified images
        self.pub_rectified_left = self.create_publisher(Image, self.left_rectified_topic, qos_profile)
        self.pub_rectified_right = self.create_publisher(Image, self.right_rectified_topic, qos_profile)

        self.get_logger().info("Stereo Rectifier Node started and ready to process images.")

    def load_and_init_rectification(self):
        try:
            with open(self.calibration_file, 'r') as f:
                settings = yaml.safe_load(f)

            K_l = np.array(settings['LEFT']['K']['data']).reshape(settings['LEFT']['K']['rows'], settings['LEFT']['K']['cols'])
            D_l = np.array(settings['LEFT']['D']['data']).reshape(settings['LEFT']['D']['rows'], settings['LEFT']['D']['cols'])
            R_l = np.array(settings['LEFT']['R']['data']).reshape(settings['LEFT']['R']['rows'], settings['LEFT']['R']['cols'])
            P_l = np.array(settings['LEFT']['P']['data']).reshape(settings['LEFT']['P']['rows'], settings['LEFT']['P']['cols'])
            
            K_r = np.array(settings['RIGHT']['K']['data']).reshape(settings['RIGHT']['K']['rows'], settings['RIGHT']['K']['cols'])
            D_r = np.array(settings['RIGHT']['D']['data']).reshape(settings['RIGHT']['D']['rows'], settings['RIGHT']['D']['cols'])
            R_r = np.array(settings['RIGHT']['R']['data']).reshape(settings['RIGHT']['R']['rows'], settings['RIGHT']['R']['cols'])
            P_r = np.array(settings['RIGHT']['P']['data']).reshape(settings['RIGHT']['P']['rows'], settings['RIGHT']['P']['cols'])

            cols_l = settings['LEFT']['width']
            rows_l = settings['LEFT']['height']
            cols_r = settings['RIGHT']['width']
            rows_r = settings['RIGHT']['height']

            # Extract 3x3 intrinsic part from P matrix for newCameraMatrix
            P_l_3x3 = P_l[:3, :3]
            P_r_3x3 = P_r[:3, :3]
            
            # Initialize undistort and rectify maps
            self.M1l, self.M2l = cv2.initUndistortRectifyMap(
                K_l, D_l, R_l, P_l_3x3, (cols_l, rows_l), cv2.CV_32F)
            self.M1r, self.M2r = cv2.initUndistortRectifyMap(
                K_r, D_r, R_r, P_r_3x3, (cols_r, rows_r), cv2.CV_32F)

            self.get_logger().info("[INFO] Calibration parameters loaded and rectification maps initialized.")

        except FileNotFoundError:
            self.get_logger().error(f"[ERROR] Calibration file not found: {self.calibration_file}")
            rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f"[ERROR] loading calibration or initializing maps: {e}")
            rclpy.shutdown()

    def stereo_callback(self, msgLeft, msgRight):
        if self.M1l is None:
            self.get_logger().warn("[WARN] Rectification maps not initialized. Skipping image processing.")
            return

        try:
            # Convert ROS Image messages to OpenCV images
            cv_image_left = self.cv_bridge.imgmsg_to_cv2(msgLeft, desired_encoding='passthrough')
            cv_image_right = self.cv_bridge.imgmsg_to_cv2(msgRight, desired_encoding='passthrough')

            # Perform rectification
            im_left_rectified = cv2.remap(cv_image_left, self.M1l, self.M2l, cv2.INTER_LINEAR)
            im_right_rectified = cv2.remap(cv_image_right, self.M1r, self.M2r, cv2.INTER_LINEAR)

            # Convert rectified OpenCV images back to ROS Image messages
            ros_img_left_rectified = self.cv_bridge.cv2_to_imgmsg(im_left_rectified) 
            ros_img_right_rectified = self.cv_bridge.cv2_to_imgmsg(im_right_rectified)

            # Copy headers from original messages
            ros_img_left_rectified.header = msgLeft.header
            ros_img_right_rectified.header = msgRight.header

            # Publish the rectified images
            self.pub_rectified_left.publish(ros_img_left_rectified)
            self.pub_rectified_right.publish(ros_img_right_rectified)

        except Exception as e:
            self.get_logger().error(f"[ERROR] Error in stereo callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = StereoRectifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()