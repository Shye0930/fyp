# Camera Calibration 
This markdown will be a guide on calibrating stereo camera with ros2 foxy. We will be following this [link](https://docs.ros.org/en/rolling/p/camera_calibration/doc/tutorial_stereo.html)


## Installing camera_calibration package 
```shell
sudo apt install ros-foxy-camera-calibration
pip3 install simver
```

To verify the installation, run the following command which shows where the package is installed.
```shell
ros2 pkg prefix camera_calibration
```

## Moving of edited files
In your directory `/opt/ros/foxy/lib/python3.8/site-packages` do 
```shell
sudo rm -rf /opt/ros/foxy/lib/python3.8/site-packages/camera_calibration # Remove the folder

# Afterwards move the files within the assets folder to the dest folder
sudo cp -r /home/<your name>/Desktop/projects/fyp/Installation Guide/assets/camera_calibration /opt/ros/foxy/lib/python3.8/site-packages/
```

## Arduino Code for esp32 camera

```cpp
#include "esp_camera.h"
#include <WiFi.h>

//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well

// ===================
// Select camera model
// ===================
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE  // Has PSRAM
//#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_CAMS3_UNIT  // Has PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
//#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
// ** Espressif Internal Boards **
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD
//#define CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3 // Has PSRAM
//#define CAMERA_MODEL_DFRobot_Romeo_ESP32S3 // Has PSRAM
#include "camera_pins.h"

// ===========================
// Enter your WiFi credentials
// ===========================
const char *ssid = "";
const char *password = "";

framesize_t current_cam_framesize;
int current_cam_quality;
gainceiling_t current_cam_gain;

void startCameraServer();
void setupLedFlash(int pin);

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // camera_config_t config;
  // config.ledc_channel = LEDC_CHANNEL_0;
  // config.ledc_timer = LEDC_TIMER_0;
  // config.pin_d0 = Y2_GPIO_NUM;
  // config.pin_d1 = Y3_GPIO_NUM;
  // config.pin_d2 = Y4_GPIO_NUM;
  // config.pin_d3 = Y5_GPIO_NUM;
  // config.pin_d4 = Y6_GPIO_NUM;
  // config.pin_d5 = Y7_GPIO_NUM;
  // config.pin_d6 = Y8_GPIO_NUM;
  // config.pin_d7 = Y9_GPIO_NUM;
  // config.pin_xclk = XCLK_GPIO_NUM;
  // config.pin_pclk = PCLK_GPIO_NUM;
  // config.pin_vsync = VSYNC_GPIO_NUM;
  // config.pin_href = HREF_GPIO_NUM;
  // config.pin_sccb_sda = SIOD_GPIO_NUM;
  // config.pin_sccb_scl = SIOC_GPIO_NUM;
  // config.pin_pwdn = PWDN_GPIO_NUM;
  // config.pin_reset = RESET_GPIO_NUM;
  // config.xclk_freq_hz = 10000000 ; // TOOD: I change here 20000000 8000000;
  // config.frame_size = FRAMESIZE_VGA;
  // config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  // //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  // config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  // config.fb_location = CAMERA_FB_IN_PSRAM;
  // config.jpeg_quality = 12;
  // config.fb_count = 1;


 camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 23000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.fb_count = 2;


  current_cam_framesize = FRAMESIZE_HVGA;
  current_cam_quality = 15; //10-63 lower number means higher quality
  current_cam_gain = (gainceiling_t)0;

  config.frame_size = current_cam_framesize;
  config.jpeg_quality = current_cam_quality; 

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } 

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  
  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated

  if(s->id.PID == OV2640_PID){
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, -2);   // up the brightness just a bit
    s->set_contrast(s, 0);
    s->set_saturation(s,2);
    s->set_hmirror(s,0);
    s->set_gainceiling(s, current_cam_gain);
  }


#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  if(psramFound()){
    Serial.println("PSRAM available");
  }

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {
  // // Do nothing. Everything is done in another task by the web server
  delay(10000);
}

```


## Publish the left and right frames
Create a python script to facilitate publishing the mjepg frames with open cv using 2 ESP32 camera

```python
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
        self.left_url = 'http://192.168.68.56:8081/stream'
        self.right_url = 'http://192.168.68.55:8081/stream'

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
```

### View the publish frames
```sh
sudo apt install ros-foxy-rqt-image-view
```

## Calibration
Run this python script

```python
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, CameraInfo
# from cv_bridge import CvBridge
# import cv2
# import threading

# class StereoCameraPublisher(Node):

#     def __init__(self):
#         super().__init__('stereo_camera_publisher')
#         self.left_image_pub = self.create_publisher(Image, '/stereo/left/image_raw', 10)
#         self.right_image_pub = self.create_publisher(Image, '/stereo/right/image_raw', 10)
#         self.left_info_pub = self.create_publisher(CameraInfo, '/stereo/left/camera_info', 10)
#         self.right_info_pub = self.create_publisher(CameraInfo, '/stereo/right/camera_info', 10)
#         self.bridge = CvBridge()

#         # Replace these with your actual camera stream URLs
#         self.left_url = 'http://192.168.68.55:8081/stream'
#         self.right_url = 'http://192.168.68.58:8081/stream'

#         self.running = True
#         self.thread = threading.Thread(target=self.stream_loop)
#         self.thread.start()

#     def stream_loop(self):
#         self.get_logger().info("Connecting to cameras...")
#         cap_left = cv2.VideoCapture(self.left_url)
#         cap_right = cv2.VideoCapture(self.right_url)

#         if not cap_left.isOpened() or not cap_right.isOpened():
#             self.get_logger().error("Failed to open one or both streams.")
#             return

#         while self.running:
#             ret_l, frame_l = cap_left.read()
#             ret_r, frame_r = cap_right.read()

#             if ret_l and ret_r:
#                 now = self.get_clock().now().to_msg()

#                 msg_l = self.bridge.cv2_to_imgmsg(frame_l, encoding='bgr8')
#                 msg_r = self.bridge.cv2_to_imgmsg(frame_r, encoding='bgr8')

#                 msg_l.header.stamp = now
#                 msg_r.header.stamp = now
#                 msg_l.header.frame_id = 'left_camera'
#                 msg_r.header.frame_id = 'right_camera'

#                 self.left_image_pub.publish(msg_l)
#                 self.right_image_pub.publish(msg_r)

#                 # Publish dummy CameraInfo (for calibration to work)
#                 h, w = frame_l.shape[:2]
#                 cam_info = CameraInfo()
#                 cam_info.header.stamp = now
#                 cam_info.header.frame_id = 'left_camera'
#                 cam_info.width = w
#                 cam_info.height = h
#                 self.left_info_pub.publish(cam_info)

#                 cam_info.header.frame_id = 'right_camera'
#                 self.right_info_pub.publish(cam_info)

#                 self.get_logger().info("Published stereo frames + camera info.")
#             else:
#                 self.get_logger().warn("Failed to read one or both frames.")

#         cap_left.release()
#         cap_right.release()

#     def destroy_node(self):
#         self.running = False
#         self.thread.join()
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = StereoCameraPublisher()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

```


Newer code since the above will cause a significant lag

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import threading
import queue # Import queue for thread-safe data passing
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
        self.frame_lock = threading.Lock() # To protect access to latest_frame
        self.set_flash(128)


    def set_flash(self, intensity):
      """HTTP control method"""
      intensity = max(0, min(255, intensity))
      try:
          url = f"{self.url}:80/control?var=led_intensity&val={intensity}"
          response = requests.get(url, timeout=2)
          if response.status_code == 200:
              self.intensity = intensity
          return response.status_code == 200
      except Exception as e:
          print(f"HTTP framesize control failed: {e}")
          return False
        

    def run(self):
        self.node.get_logger().info(f"Connecting to {self.name} camera at {self.url}:81/stream...")
        self.stream_url = self.url + ":81/stream"
        self.cap = cv2.VideoCapture(self.stream_url)

        if not self.cap.isOpened():
            self.node.get_logger().error(f"Failed to open {self.name} stream.")
            self.running = False
            return

        # Optional: Try to reduce buffer size if supported by the camera/stream
        # Some IP cameras or certain backends might ignore this.
        # self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.frame_lock:
                    self.latest_frame = frame
                    self.ret = True
            else:
                self.node.get_logger().warn(f"Failed to read frame from {self.name} camera.")
                self.ret = False
                # Consider adding a small sleep or reconnection logic here if stream breaks
                # time.sleep(0.1) 
        self.cap.release()
        self.node.get_logger().info(f"{self.name} camera thread stopped.")

    def get_frame(self):
        with self.frame_lock:
            # Return a copy to avoid external modification while this thread updates it
            return self.ret, self.latest_frame.copy() if self.latest_frame is not None else None

    def stop(self):
        self.set_flash(0)
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
        self.right_url = 'http://192.168.68.56' #USB-C module

        # Create dedicated threads for each camera
        self.left_cam_thread = CameraStreamThread(self.left_url, "left", self)
        self.right_cam_thread = CameraStreamThread(self.right_url, "right", self)

        self.left_cam_thread.start()
        self.right_cam_thread.start()

        # Timer to periodically get frames from threads and publish
        self.timer = self.create_timer(0.01, self.publish_stereo_frames) # ~30 FPS

    def publish_stereo_frames(self):
        ret_l, frame_l = self.left_cam_thread.get_frame()
        ret_r, frame_r = self.right_cam_thread.get_frame()

        if ret_l and ret_r:
            now = self.get_clock().now().to_msg()

            msg_l = self.bridge.cv2_to_imgmsg(frame_l, encoding='bgr8')
            msg_r = self.bridge.cv2_to_imgmsg(frame_r, encoding='bgr8')

            msg_l.header.stamp = now
            msg_r.header.stamp = now
            msg_l.header.frame_id = 'left_camera'
            msg_r.header.frame_id = 'right_camera'

            self.left_image_pub.publish(msg_l)
            self.right_image_pub.publish(msg_r)

            # Publish dummy CameraInfo (for calibration to work)
            h, w = frame_l.shape[:2]
            cam_info = CameraInfo()
            cam_info.header.stamp = now
            cam_info.header.frame_id = 'left_camera'
            cam_info.width = w
            cam_info.height = h

            self.left_info_pub.publish(cam_info)

            cam_info.header.frame_id = 'right_camera'
            self.right_info_pub.publish(cam_info)

            # self.get_logger().info("Published stereo frames + camera info.") # This can be very verbose
        else:
            self.get_logger().warn("Failed to read one or both frames from camera threads. Waiting for data...")


    def destroy_node(self):
        self.left_cam_thread.stop()
        self.right_cam_thread.stop()
        self.left_cam_thread.join()
        self.right_cam_thread.join()
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

```

### Input the camera IP
In the 2 statement below, change them to your corresponding ESP32 camera IP

```python
self.left_url = 'http://192.168.68.57'
self.right_url = 'http://192.168.68.56' #USB-C module
```


I am using a 6x9 checkboard of size:30mm

To determine the size of the checkboard, do note that 
- width: Number of intersection points of squares in the long side of the calibration board. Hence for a 6x9 checkboard, the width is (9-1) = 8
- height: Number of intersection points of squares in the short side of the calibration board. Hence for a 6x9 checkboard, the height is (6-1) = 5

Hence the command is 
```sh
ros2 run camera_calibration cameracalibrator --approximate 0.1 --size 8x5 --square 0.029 right:=/stereo/right/image_raw left:=/stereo/left/image_raw right_camera:=/stereo/right left_camera:=/stereo/left

```
Afterwards, ensure that in the display you set to the **correct** (pinhole/fisheye) lens.

![image](./assets/camera_calibrator_display.png)

### Moving the checkerboard
This is taken directly from this [link](https://docs.ros.org/en/rolling/p/camera_calibration/doc/tutorial_stereo.html)


In order to get a good calibration you will need to move the checkerboard around in the camera frame such that:

* The checkerboard is detected at the left and right edges of the field of view (X calibration).

* The checkerboard is detected at the top and bottom edges of the field of view (Y calibration).

* The checkerboard is detected at various angles to the camera (“Skew”).

* The checkerboard fills the entire field of view (Size calibration).

* checkerboard tilted to the left, right, top and bottom (X,Y, and Size calibration).


### Saving the calibration
After the program has automatically taken enough samples, you should be able to press the calibrate button and save the config. Do not press the `config` button as it is not applicable to the ESP32-Camera

Additionally, the console should have printed some logs and you will need to **__save__** the text below `Stereo pinhole calibration` as it contains the T and R matrix to compute the Stereo.T_c1_c2 for our orb slam3 config later on.



## Reference 
1. [Overview of camera claibration](https://docs.ros.org/en/rolling/p/camera_calibration/doc/index.html)
2. [Tutorial: Stereo Calibration](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration)
3. [Nodes: Camera Calibrator](https://docs.ros.org/en/rolling/p/camera_calibration/doc/components.html#camera-calibrator)
4. [Fix ros foxy fisheye calibration](https://github.com/ros-perception/image_pipeline/issues/637)