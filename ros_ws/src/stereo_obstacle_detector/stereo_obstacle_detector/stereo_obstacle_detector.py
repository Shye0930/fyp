import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter # Import Parameter for explicit parameter declaration
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

# Assuming envision_interfaces.msg.ObstacleStatus is defined
from envision_interfaces.msg import ObstacleStatus

class StereoObstacleDetector(Node):
    def __init__(self):
        super().__init__('stereo_obstacle_detector')

        # Declare parameters for SGBM and obstacle detection
        self.declare_parameter('min_disparity', 0)
        self.declare_parameter('num_disparities', 64) # Must be divisible by 16
        self.declare_parameter('block_size', 5) # Odd number, 3-11 typically
        
        # Get initial block_size value to declare P1 and P2 correctly
        block_size_initial = self.get_parameter('block_size').value
        self.declare_parameter('P1', 8 * 3 * block_size_initial**2)
        self.declare_parameter('P2', 32 * 3 * block_size_initial**2)
        
        self.declare_parameter('disp12_max_diff', 1)
        self.declare_parameter('uniqueness_ratio', 10)
        self.declare_parameter('speckle_window_size', 100)
        self.declare_parameter('speckle_range', 32)

        self.declare_parameter('obstacle_distance_threshold_m', 1.5) # Meters

        # --- Declare focal_length and baseline as parameters ---
        self.declare_parameter('focal_length', 485.9461) # Default focal length (fx)
        self.declare_parameter('baseline', 0.078453409)  # Default baseline in meters

        # Retrieve parameter values
        self.focal_length = self.get_parameter('focal_length').value
        self.baseline = self.get_parameter('baseline').value

        # Subscribers for rectified stereo images
        self.left_image_sub = self.create_subscription(
            Image,
            '/stereo/left/rectified_images',
            self.left_image_callback,
            10
        )
        self.right_image_sub = self.create_subscription(
            Image,
            '/stereo/right/rectified_images',
            self.right_image_callback,
            10
        )
        
        # Publishers
        self.disparity_pub = self.create_publisher(Image, 'disparity_map', 10)
        self.depth_pub = self.create_publisher(Image, 'depth_map', 10) # Publishing as sensor_msgs/Image (float32)
        self.obstacle_status_pub = self.create_publisher(ObstacleStatus, 'obstacle_status', 10)

        self.bridge = CvBridge()

        self.left_image = None
        self.right_image = None

        self.get_logger().info('Stereo Obstacle Detector Node initialized.')
        self.get_logger().info(f"Using Focal Length (parameter): {self.focal_length}")
        self.get_logger().info(f"Using Baseline (parameter): {self.baseline}")


    def left_image_callback(self, msg):
        self.left_image = msg
        self.process_stereo_images()

    def right_image_callback(self, msg):
        self.right_image = msg
        self.process_stereo_images()

    def process_stereo_images(self):
        # Only check for images, no need for camera info
        if self.left_image is None or self.right_image is None:
            return

        # Convert ROS Image messages to OpenCV images (grayscale for SGBM)
        try:
            img_left = self.bridge.imgmsg_to_cv2(self.left_image, desired_encoding='bgr8')
            img_right = self.bridge.imgmsg_to_cv2(self.right_image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting images: {e}")
            return

        gray_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)

        # Get SGBM parameters from ROS parameters
        min_disparity = self.get_parameter('min_disparity').value
        num_disparities = self.get_parameter('num_disparities').value
        block_size = self.get_parameter('block_size').value
        
        # Re-calculate P1 and P2 based on current block_size parameter value
        p1 = 8 * 3 * block_size**2
        p2 = 32 * 3 * block_size**2

        disp12_max_diff = self.get_parameter('disp12_max_diff').value
        uniqueness_ratio = self.get_parameter('uniqueness_ratio').value
        speckle_window_size = self.get_parameter('speckle_window_size').value
        speckle_range = self.get_parameter('speckle_range').value

        # Create StereoSGBM object
        stereo = cv2.StereoSGBM_create(
            minDisparity=min_disparity,
            numDisparities=num_disparities,
            blockSize=block_size,
            P1=p1,
            P2=p2,
            disp12MaxDiff=disp12_max_diff,
            uniquenessRatio=uniqueness_ratio,
            speckleWindowSize=speckle_window_size,
            speckleRange=speckle_range,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )

        # Compute disparity map
        disparity = stereo.compute(gray_left, gray_right).astype(np.float32) / 16.0 # Divide by 16 as per OpenCV docs

        

        # Normalize disparity for visualization (optional)
        normalized_disparity = cv2.normalize(disparity, None, alpha=255,
                                             beta=0, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        # Publish disparity map
        disparity_msg = self.bridge.cv2_to_imgmsg(normalized_disparity, encoding='mono8')
        disparity_msg.header = self.left_image.header # Use the header from the input image
        self.disparity_pub.publish(disparity_msg)

        # Calculate depth map
        # Z = f * B / d
        # Use the hardcoded focal_length and baseline
        if self.focal_length > 0 and self.baseline > 0:
            # Initialize depth_map with a maximum float value for invalid areas
            depth_map = np.full_like(disparity, np.finfo(np.float32).max, dtype=np.float32)
            valid_disparities_mask = disparity > 0 # Only calculate depth for positive disparities
            depth_map[valid_disparities_mask] = (self.focal_length * self.baseline) / disparity[valid_disparities_mask]
            
            # Publish depth map (as float32 image)
            depth_msg = self.bridge.cv2_to_imgmsg(depth_map, encoding='passthrough') # Use passthrough for float32
            depth_msg.header = self.left_image.header
            self.depth_pub.publish(depth_msg)

            # Obstacle avoidance logic
            obstacle_distance_threshold_m = self.get_parameter('obstacle_distance_threshold_m').value
            
            # Find minimum depth in a region of interest (e.g., center of the image)
            h, w = depth_map.shape
            # Define an ROI for obstacle detection (e.g., lower half, center width)
            roi_h_start = int(h * 0.25) 
            roi_h_end = int(h * 0.75) #h 
            roi_w_start = int(w * 0.3)
            roi_w_end = int(w * 0.7)
            roi = depth_map[roi_h_start:roi_h_end, roi_w_start:roi_w_end]

            # Filter out infinite values (where disparity was 0 or invalid) and check if ROI is not empty
            # finite_depths_in_roi = roi[~np.isinf(roi)]
            finite_depths_in_roi = roi[~np.isinf(roi)]
            
            min_depth_in_roi = np.finfo(np.float32).max # Initialize with max float value
            if finite_depths_in_roi.size > 0:
                min_depth_in_roi = np.min(finite_depths_in_roi)

            obstacle_detected = False
            if min_depth_in_roi > 0.75 and min_depth_in_roi < obstacle_distance_threshold_m:
                obstacle_detected = True
                self.get_logger().info(f"OBSTACLE DETECTED! Minimum distance: {min_depth_in_roi:.2f} m")

            # Publish obstacle status
            status_msg = ObstacleStatus()
            status_msg.obstacle_detected = obstacle_detected
            status_msg.min_distance = float(min_depth_in_roi)
            self.obstacle_status_pub.publish(status_msg)

        else:
            self.get_logger().warn("Focal length or baseline is zero or not set, cannot compute depth.")

        # Clear images for next processing cycle
        self.left_image = None
        self.right_image = None


def main(args=None):
    rclpy.init(args=args)
    node = StereoObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()