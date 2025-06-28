// Follow instructions here to install ROS2-Foxy Edition:
// https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade

sudo apt install ros-foxy-desktop python3-argcomplete

sudo apt install ros-dev-tools

source /opt/ros/foxy/setup.bash
source ~/colcon_ws/install/setup.bash


// Now move onto installing the ROS2 SLAM Wrapper from here:
// https://github.com/doeswork/EASY-ORB-SLAM3/tree/master/src/orbslam3_ros2

sudo apt install ros-$ROS_DISTRO-vision-opencv && sudo apt install ros-$ROS_DISTRO-message-filters

mkdir -p colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/doeswork/EASY-ORB-SLAM3.git

cd ~/colcon_ws/src
cp -r EASY-ORB-SLAM3/src/orbslam3_ros2 ./orbslam3
rm -rf EASY-ORB-SLAM3

nano ~/colcon_ws/src/orbslam3/CMakeLists.txt
//Now change the content in LINE 5 to this:
set(ENV{PYTHONPATH} "/opt/ros/foxy/lib/python3.8/site-packages:/usr/local/lib/python3.8/dist-packages")

export CMAKE_PREFIX_PATH="/usr/local:$CMAKE_PREFIX_PATH"
export CMAKE_PREFIX_PATH=~/vcpkg/installed/arm64-linux/share/pangolin
export CMAKE_PREFIX_PATH=~/vcpkg/installed/arm64-linux/share
//Use the below two if on WSL!!!
export CMAKE_PREFIX_PATH=~/vcpkg/installed/x64-linux/share/pangolin
export CMAKE_PREFIX_PATH=~/vcpkg/installed/x64-linux/share

## For some reason adding where to find pangolin files in bashrc does not work...?

source ~/.bashrc



colcon build --symlink-install --packages-select orbslam3

=== Just need do the below once ===
nano ~/.bashrc
# Add these 2 lines at the end of the file
source /opt/ros/foxy/setup.bash
source ~/colcon_ws/install/setup.bash
source ~/.bashrc
=== Just need do the above once ===

cd ~/colcon_ws/src/orbslam3/vocabulary
tar -xzf ORBvoc.txt.tar.gz

nano ~/colcon_ws/src/orbslam3/src/url_camera_publisher.py

========== SCRIPT HERE (overwrite whatever is inside to the below) ===========
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading

class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera', 10)
        self.bridge = CvBridge()
        self.video_url = 'http://192.168.121.123:81/stream'
        self.running = True

        # Start thread
        self.thread = threading.Thread(target=self.stream_loop)
        self.thread.start()

    def stream_loop(self):
        self.get_logger().info(f"Connecting to {self.video_url} using OpenCV VideoCapture...")
        cap = cv2.VideoCapture(self.video_url)

        if not cap.isOpened():
            self.get_logger().error("Failed to open stream with OpenCV.")
            return

        while self.running:
            ret, frame = cap.read()
            if ret:
                msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                self.publisher_.publish(msg)
                self.get_logger().info("Published a frame.")
            else:
                self.get_logger().warn("Failed to read frame.")
                rclpy.sleep(0.1)

        cap.release()

    def destroy_node(self):
        self.running = False
        self.thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
================ SCRIPT HERE ===================

chmod +x ~/colcon_ws/src/orbslam3/src/url_camera_publisher.py

sudo apt install python3-pip
pip install --upgrade numpy
pip install --upgrade opencv-python

cd ~/colcon_ws
ros2 run orbslam3 url_camera_publisher.py


// IN ANOTHER TERMINAL DO THE BELOW
ros2 run orbslam3 mono ~/colcon_ws/src/orbslam3/vocabulary/ORBvoc.txt ~/colcon_ws/src/orbslam3/config/monocular/TUM1.yaml

===================================|========================================
===================================|========================================
===================================|========================================
===================================|========================================
===================================|========================================
===================================|========================================
===================================|========================================









==================== for running STEREO ===========================
nano ~/colcon_ws/src/orbslam3/src/stereo_url_camera_publisher.py
chmod +x ~/colcon_ws/src/orbslam3/src/stereo_url_camera_publisher.py
================ SCRIPT HERE ===================
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading

class StereoCameraPublisher(Node):

    def __init__(self):
        super().__init__('stereo_camera_publisher')
        self.left_pub = self.create_publisher(Image, '/camera/left', 10)
        self.right_pub = self.create_publisher(Image, '/camera/right', 10)
        self.bridge = CvBridge()

        # Replace these with your actual camera stream URLs
        self.left_url = 'http://192.168.121.123:81/stream'
        self.right_url = 'http://192.168.121.153:81/stream'

        self.running = True
        self.thread = threading.Thread(target=self.stream_loop)
        self.thread.start()

    def stream_loop(self):
        self.get_logger().info(f"Connecting to cameras...")
        cap_left = cv2.VideoCapture(self.left_url)
        cap_right = cv2.VideoCapture(self.right_url)

        if not cap_left.isOpened() or not cap_right.isOpened():
            self.get_logger().error("Failed to open one or both streams.")
            return

        while self.running:
            ret_l, frame_l = cap_left.read()
            ret_r, frame_r = cap_right.read()

            if ret_l and ret_r:
                now = self.get_clock().now().to_msg()

                # Convert to Image messages
                msg_l = self.bridge.cv2_to_imgmsg(frame_l, encoding='bgr8')
                msg_r = self.bridge.cv2_to_imgmsg(frame_r, encoding='bgr8')

                msg_l.header.stamp = now
                msg_r.header.stamp = now

                self.left_pub.publish(msg_l)
                self.right_pub.publish(msg_r)

                self.get_logger().info("Published left and right frames.")
            else:
                self.get_logger().warn("Failed to read one or both frames.")

        cap_left.release()
        cap_right.release()

    def destroy_node(self):
        self.running = False
        self.thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
================ SCRIPT HERE ===================
ros2 run orbslam3 stereo ~/colcon_ws/src/orbslam3/vocabulary/ORBvoc.txt ~/colcon_ws/src/orbslam3/config/stereo/TUM-VI.yaml false


In another terminal:
python3 ~/colcon_ws/src/orbslam3/src/stereo_url_camera_publisher.py
