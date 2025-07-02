import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo # Import the service type
from cv_bridge import CvBridge
import cv2
import threading
import queue
import requests

class CameraStreamThread(threading.Thread):
    def __init__(self, url, name, node):
        super().__init__()
        self.url = url
        self.name = name
        self.node = node
        self.cap = None
        self.latest_frame = None
        self.ret = False
        self.running = True
        self.frame_lock = threading.Lock()
        self.set_flash(128) # Ensure flash is set on initialization

    def set_flash(self, intensity):
        """HTTP control method for LED intensity."""
        intensity = max(0, min(255, intensity))
        try:
            url = f"{self.url}:80/control?var=led_intensity&val={intensity}"
            response = requests.get(url, timeout=2)
            if response.status_code == 200:
                self.intensity = intensity
                self.node.get_logger().info(f"Set {self.name} camera LED intensity to {intensity}.")
            return response.status_code == 200
        except Exception as e:
            self.node.get_logger().error(f"HTTP LED intensity control for {self.name} failed: {e}")
            return False

    def run(self):
        self.node.get_logger().info(f"Connecting to {self.name} camera at {self.url}:81/stream...")
        self.stream_url = self.url + ":81/stream"
        self.cap = cv2.VideoCapture(self.stream_url)

        if not self.cap.isOpened():
            self.node.get_logger().error(f"Failed to open {self.name} stream.")
            self.running = False
            return

        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.frame_lock:
                    self.latest_frame = frame
                    self.ret = True
            else:
                self.node.get_logger().warn(f"Failed to read frame from {self.name} camera.")
                self.ret = False
                # Reconnection logic could be added here if the stream consistently breaks
        self.cap.release()
        self.node.get_logger().info(f"{self.name} camera thread stopped.")

    def get_frame(self):
        with self.frame_lock:
            return self.ret, self.latest_frame.copy() if self.latest_frame is not None else None

    def stop(self):
        self.set_flash(0) # Turn off flash when stopping
        self.running = False


class StereoCameraPublisher(Node):

    def __init__(self):
        super().__init__('stereo_camera_publisher')
        self.left_image_pub = self.create_publisher(Image, '/stereo/left/image_raw', 10)
        self.right_image_pub = self.create_publisher(Image, '/stereo/right/image_raw', 10)
        self.left_info_pub = self.create_publisher(CameraInfo, '/stereo/left/camera_info', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, '/stereo/right/camera_info', 10)
        self.bridge = CvBridge()

        self.left_url = 'http://192.168.68.57'
        self.right_url = 'http://192.168.68.58'

        # Store camera info internally
        self.left_camera_info = CameraInfo()
        self.right_camera_info = CameraInfo()

        # Create dedicated threads for each camera
        self.left_cam_thread = CameraStreamThread(self.left_url, "left", self)
        self.right_cam_thread = CameraStreamThread(self.right_url, "right", self)

        self.left_cam_thread.start()
        self.right_cam_thread.start()

        # --- ADD DUMMY SERVICES HERE ---
        # Left camera set_camera_info service
        self.left_set_camera_info_service = self.create_service(
            SetCameraInfo,
            'left_camera/set_camera_info', # Service name
            self.handle_left_set_camera_info # Callback function
        )
        self.get_logger().info('Left camera left_camera/set_camera_info service created.')

        # Right camera set_camera_info service
        self.right_set_camera_info_service = self.create_service(
            SetCameraInfo,
            'right_camera/set_camera_info', # Service name
            self.handle_right_set_camera_info # Callback function
        )
        self.get_logger().info('Right camera right_camera/set_camera_info service created.')
        # --- END DUMMY SERVICES ---

        # Timer to periodically get frames from threads and publish
        self.timer = self.create_timer(0.01, self.publish_stereo_frames)

    def handle_left_set_camera_info(self, request, response):
        """
        Dummy service handler for /stereo/left/set_camera_info.
        It pretends to accept the camera info and logs it.
        """
        self.get_logger().info(f"Received SetCameraInfo request for left camera.")
        # In a real scenario, you would validate and store this camera info.
        # For a dummy service, we just acknowledge it.
        self.left_camera_info = request.camera_info # Store the received info
        response.success = True
        response.status_message = "Left camera info received successfully (dummy)."
        return response

    def handle_right_set_camera_info(self, request, response):
        """
        Dummy service handler for /stereo/right/set_camera_info.
        It pretends to accept the camera info and logs it.
        """
        self.get_logger().info(f"Received SetCameraInfo request for right camera.")
        # In a real scenario, you would validate and store this camera info.
        # For a dummy service, we just acknowledge it.
        self.right_camera_info = request.camera_info # Store the received info
        response.success = True
        response.status_message = "Right camera info received successfully (dummy)."
        return response

    def publish_stereo_frames(self):
        ret_l, frame_l = self.left_cam_thread.get_frame()
        ret_r, frame_r = self.right_cam_thread.get_frame()

        if ret_l and ret_r:
            now = self.get_clock().now().to_msg()

            # Publish images
            msg_l = self.bridge.cv2_to_imgmsg(frame_l, encoding='bgr8')
            msg_r = self.bridge.cv2_to_imgmsg(frame_r, encoding='bgr8')

            msg_l.header.stamp = now
            msg_r.header.stamp = now
            msg_l.header.frame_id = 'left_camera_link' # Use a proper frame_id for TF compatibility
            msg_r.header.frame_id = 'right_camera_link'

            self.left_image_pub.publish(msg_l)
            self.right_image_pub.publish(msg_r)

            # Publish CameraInfo (either a dummy one or the one received via service)
            # If you received info via service, use that. Otherwise, generate a dummy.
            
            # Left CameraInfo
            if self.left_camera_info.width == 0 or self.left_camera_info.height == 0:
                # If no info received yet, create a dummy one based on current frame
                h, w = frame_l.shape[:2]
                self.left_camera_info.header.stamp = now
                self.left_camera_info.header.frame_id = 'left_camera_link'
                self.left_camera_info.width = w
                self.left_camera_info.height = h
                # Fill in other fields (K, D, R, P) with zeros or default values if needed
                # For a true dummy, just width/height is often enough for some tools.
                # K (intrinsics matrix) should be 3x3, D (distortion coeffs) 1x5, R (rectification) 3x3, P (projection) 3x4
                self.left_camera_info.k = [0.0]*9 # All zeros for dummy
                self.left_camera_info.d = [0.0]*5 # All zeros for dummy
                self.left_camera_info.r = [0.0]*9
                self.left_camera_info.p = [0.0]*12
            else:
                # Use the info received from the service
                self.left_camera_info.header.stamp = now
                self.left_camera_info.header.frame_id = 'left_camera_link' # Ensure frame_id is consistent
            self.left_info_pub.publish(self.left_camera_info)


            # Right CameraInfo
            if self.right_camera_info.width == 0 or self.right_camera_info.height == 0:
                h, w = frame_r.shape[:2]
                self.right_camera_info.header.stamp = now
                self.right_camera_info.header.frame_id = 'right_camera_link'
                self.right_camera_info.width = w
                self.right_camera_info.height = h
                self.right_camera_info.k = [0.0]*9
                self.right_camera_info.d = [0.0]*5
                self.right_camera_info.r = [0.0]*9
                self.right_camera_info.p = [0.0]*12
            else:
                self.right_camera_info.header.stamp = now
                self.right_camera_info.header.frame_id = 'right_camera_link'
            self.right_info_pub.publish(self.right_camera_info)

        else:
            # self.get_logger().warn("Failed to read one or both frames from camera threads. Waiting for data...")
            pass # Keep it less verbose if it's just a temporary blip


    def destroy_node(self):
        self.left_cam_thread.stop()
        self.right_cam_thread.stop()
        self.left_cam_thread.join()
        self.right_cam_thread.join()
        self.get_logger().info("Shutting down camera threads.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt detected, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()