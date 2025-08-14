## <h2 style="color:#1F1F1F; background-color:#B0B0B0; text-align:center; text-style:bold; font-family:'Chalkboard' ;">Stereo Camera Pipeline</h2>

When utilizing `stereo_processor_node.py` directly or launching it via `ros2 launch stereo_camera_pipeline stereo_pipeline.launch.py`, it is necessary to employ a dedicated configuration file. This is due to differences in the expected YAML format, particularly concerning directives like `%YAML`, which the Python processing in this package may not handle in the same manner as ORB-SLAM3's configuration loader.

For a compatible configuration structure, please consult `config/stereo/stereo_calibration.yaml`. This file serves as a direct reference for inputting parameters suitable for the **stereo_camera_pipeline** package.

Conversely, `config/stereo/Esp32s.yaml` provides an example of the specific configuration format used for ORB-SLAM3 when working with rectified stereo inputs.

If using the **stereo_camera_pipeline** package, ensure the YAML file in the `config` directory is updated, as it serves as the fallback configuration if the provided path is invalid.

### **Publishing Rectified Stereo Frames**

To publish rectified stereo frames, you can choose one of the following methods:

#### **Using the Python Script (`stereo_processor_node.py`)**

If you're running the node directly from the Python script located at `src/stereo_processor_node.py`, update the calibration file path on **line 137** to point to your desired YAML calibration file.

#### **Using the ROS 2 Launch System**
Launch the stereo processing pipeline with:

```sh
ros2 launch stereo_camera_pipeline stereo_pipeline.launch.py
```

## <h2 style="color:#1F1F1F; background-color:#B0B0B0; text-align:center; text-style:bold; font-family:'Chalkboard' ;">Running ORB SLAM 3 via ros2</h2>

### **Overview**
This ROS 2 Foxy package provides a wrapper for ORB-SLAM3, enabling seamless integration with ROS 2 for various sensor modalities. I would like to express my gratitude to the following projects for their inspiration and foundational work:

- [Tran97/orb_slam3_ros_docker](https://github.com/Tran97/orb_slam3_ros_docker): For inspiring the implementation of RealSense to map-to-world transforms.

- [thien94/orb_slam3_ros](https://github.com/thien94/orb_slam3_ros): For serving as a foundation for to publish ORB-SLAM3 features into a ROS 2.

- [doeswork/EASY-ORB-SLAM3](https://github.com/doeswork/EASY-ORB-SLAM3/tree/master/src/orbslam3_ros2): For providing a key foundation for the ROS 2 ORB-SLAM3 wrapper.

Their contributions were instrumental in shaping this project, allowing me to build upon their work to create a robust ROS 2 integration for ORB-SLAM3.

### **Prerequisites**
- Ensure the ROS 2 topics `/stereo/left/rectified_images` and `/stereo/right/rectified_images` are published using the **stereo_camera_pipeline** package.

### **Execution Commands**
Run the following commands in the `FYP` folder:

#### **Testing with Kitti-stereo**
Modify the different path in the launch file.

For sequence folder, it should look something like:
```
   datasets/
   └── kitti
      └── sequences
         ├── 00
         │   ├── image_0
         │   └── image_1
         └── 03
               ├── image_0
               └── image_1
```
To launch:
```sh
ros2 launch orbslam3 kitti_demo.launch.py 
```
#### **For Intel RealSense D435 (RGB-D Mode)**

```sh
# For Intel RealSense D435, run for RGB-D
ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true enable_infra1:=false enable_infra2:=false align_depth.enable:=true enable_sync:=true rgb_camera.color_profile:=640x480x30 depth_module.depth_profile:=640x480x30

# Enable IR emitter
ros2 param set /camera/camera depth_module.emitter_enabled 1

ros2 launch orbslam3 d435_rgbd.launch.py
```

#### **For Intel RealSense D435 (Stereo Mode)**

```sh
ros2 launch realsense2_camera rs_launch.py enable_color:=false enable_depth:=false enable_infra1:=true enable_infra2:=true depth_module.infra_profile:=640x480x30 

# Disable IR emitter
ros2 param set /camera/camera depth_module.emitter_enabled 0


ros2 launch orbslam3 d435_stereo.launch.py
```

#### **To Publish Stereo Frames**
```sh
ros2 launch stereo_camera_pipeline stereo_pipeline.launch.py 
```

#### **To Run ORB-SLAM3**
```sh
ros2 run orbslam3 stereo /home/shye/Desktop/projects/fyp/ORB_SLAM_3_COMMUNITY/Vocabulary/ORBvoc.txt /home/shye/Desktop/projects/fyp/config/stereo/Esp32s.yaml

# OR via launch file
ros2 launch orbslam3 d435_stereo.launch.py
```

#### **To Launch RViz**
```sh
ros2 run rviz2 rviz2 -d config/stereo/orb_slam3_no_imu.rviz
```

#### **Saving the Map**

1. **Automatic Save**: If `System.SaveAtlasToFile` is set in the settings file, the map will be saved automatically when the ROS node is terminated.

2. **Manual Save via Service**:
   - Verify the service interface:
     ```sh
     ros2 interface show envision_interfaces/srv/SaveMap
     ```
   - Check if the service exists:
     ```sh
     ros2 service list
     ```
   - Save the map:
     ```sh
     ros2 service call /orb_slam3/save_map envision_interfaces/srv/SaveMap "{name: 'kitti_map'}"
     ```

## <h2 style="color:#1F1F1F; background-color:#B0B0B0; text-align:center; text-style:bold; font-family:'Chalkboard' ;">Running Stereo Obstacle detector</h2>

### **Prerequisites**
- Ensure the ROS 2 topics `/stereo/left/rectified_images` and `/stereo/right/rectified_images` are published using the `stereo_camera_pipeline` package.

### **Configuration**

To set up the detector for your specific cameras, you'll just have to edit the two camera channels in the launch file.

Navigate to `ros_ws/src/stereo_obstacle_detector/launch/stereo_detector.launch.py`.

In this file, locate the Node configuration and replace the default topics with your camera's topics:

```python
# The section to edit in stereo_detector.launch.py
Node(
    package='stereo_obstacle_detector',
    executable='stereo_detector_node',
    name='stereo_detector',
    parameters=[
        # ... other parameters
        {'left_topic': '/camera/camera/infra1/image_rect_raw'},
        {'right_topic': '/camera/camera/infra2/image_rect_raw'},
    ]
)
```

### Flashing ESP32 arduino code
This allows the esp32 to subscribe to obstacle detected topic to vibrate the motor accordingly. 

Flash the arduino code into the ESP32

```ino
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/bool.h>

#define LED_PIN 23
#define LED_PIN_TEST 18
#define MOTOR_PWM_PIN 18 // PWM pin for motor control
bool led_test_state = false;

bool motor_state = false;

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
  } while (0)

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_subscription_t obstacle_sub;
rcl_publisher_t motor_pub; // New publisher for motor state
std_msgs__msg__Bool obstacle_msg;
std_msgs__msg__Bool motor_state_msg; // Message for the new publisher

void obstacle_callback(const void *msgin) {
  const std_msgs__msg__Bool *obstacle_msg = (std_msgs__msg__Bool *)msgin;
  // Set the motor state based on the obstacle detection
  motor_state = true;

  digitalWrite(MOTOR_PWM_PIN, HIGH);
  delay(1000); // Vibrate for 1 second
  digitalWrite(MOTOR_PWM_PIN, LOW);
 
  
  // Print the obstacle detection status
  Serial.print("Obstacle detected: ");
  Serial.println(obstacle_msg->data ? "true" : "false");
}

bool create_entities()
{
  const char *node_name = "esp32_obstacle_node";
  const char *ns = "";
  const int domain_id = 0;

  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, domain_id);
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  rclc_node_init_default(&node, node_name, ns, &support);

  // Initialize the subscription for obstacle detection
  rclc_subscription_init(
    &obstacle_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/obstacle_detected",
    &rmw_qos_profile_default
  );

  // Initialize the publisher for motor state
  rclc_publisher_init_default(
    &motor_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/motor_vibrating"
  );

  unsigned int num_handles = 1;
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  rclc_executor_add_subscription(&executor, &obstacle_sub, &obstacle_msg, &obstacle_callback, ON_NEW_DATA);

  return true;
}

void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&obstacle_sub, &node);
  rcl_publisher_fini(&motor_pub, &node); // Clean up the new publisher
  rclc_executor_fini(&executor);
  rcl_init_options_fini(&init_options);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}
 
void setup() {
  Serial.begin(115200);
  Serial.println("Setting up");

  set_microros_transports();
  //set_microros_wifi_transports("WIFI-SSID", "WIFI-PW", "HOST_IP", 8888)

  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN_TEST, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  obstacle_msg.data = false;
  motor_state_msg.data = false; // Initialize the new message
  state = WAITING_AGENT;
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

        // Publish the motor state
        motor_state_msg.data = motor_state;
        rcl_publish(&motor_pub, &motor_state_msg, NULL);
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  // Update LED and motor based on current state
  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }
}
```

### **Notes**
- Update the focal length and baseline parameters in the launch file located at `ros_ws/src/stereo_obstacle_detector/launch`.
- Adjust the SGBM parameters if necessary.

Run with:
```sh
ros2 launch stereo_obstacle_detector stereo_detector.launch.py
```
In another terminal:
```sh
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 
```

## <h2 style="color:#1F1F1F; background-color:#B0B0B0; text-align:center; text-style:bold; font-family:'Chalkboard' ;">Image Masker</h2>

### **Prerequisites**
- Ensure the ROS 2 topics `/stereo/left/rectified_images` and `/stereo/right/rectified_images` are published (e.g., by the `stereo_camera_pipeline` package).

### **Note**
Place your YOLO segmentation model (e.g., `yolov8n-seg.pt`) in the `ros_ws/src/image_masker/model/` directory.
- **[Warning]** The YOLO model must support segmentation for proper functionality.
- Update parameters such as `yolo_model_path`, `person_class_id`, or `publish_masked_images` in the launch file located at `ros_ws/src/image_masker/launch/` if needed.
- Masked images will be published to `/stereo/left/rect/masked` and `/stereo/right/rect/masked`.

Run with:
```sh
ros2 launch image_masker image_masker.launch.py
```


## <h2 style="color:#1F1F1F; background-color:#B0B0B0; text-align:center; text-style:bold; font-family:'Chalkboard' ;">Navigational Speaker</h2>

### **Prerequisites**
Install the required dependencies:
```sh
pip3 install gtts
sudo apt install xdg-utils
sudo apt install mpv
```

### **Execution**
Run the script:
```sh
python3 src/text_to_speech.py
```

## <h2 style="color:#1F1F1F; background-color:#B0B0B0; text-align:center; text-style:bold; font-family:'Chalkboard' ;">Pointcloud to grid</h2>

### **Overview**
The `pointcloud_to_grid` package, developed by JKK Research, converts `sensor_msgs/PointCloud2` LIDAR data into `nav_msgs/OccupancyGrid` 2D map data based on intensity and/or height. For more details, visit the repository: [jkk-research/pointcloud_to_grid](https://github.com/jkk-research/pointcloud_to_grid).

### **Prerequisites**
**[Warning]** Ensure the following dependencies are installed and built in the correct order to avoid compilation issues.

#### **Required Dependencies**
1. Clone the `grid_map` repository from [ANYbotics/grid_map](https://github.com/ANYbotics/grid_map) using the `foxy-devel` branch:
   ```sh
   git clone https://github.com/ANYbotics/grid_map -b foxy-devel
   ```
2. Install ROS 2 and PCL (Point Cloud Library) dependencies as required by the package. Ensure you have sourced your ROS 2 workspace:
   ```sh
   source ~/ros2_ws/install/setup.bash
   ```
3. Install dependencies:
   ```sh
   sudo apt install ros-foxy-image-transport-plugins
   sudo apt-get install ros-foxy-rosbag2-storage-mcap
   ```

#### **Build Order**
To successfully build the `pointcloud_to_grid` package, follow this specific order:
1. Build the `grid_map_camke_helpers` CMake helpers first.
2. Build the `grid_map_msgs` package next.
3. Finally, build the `pointcloud_to_grid` package.

Example build commands in your ROS 2 workspace:
```sh
cd ~/ros2_ws/src
git clone https://github.com/ANYbotics/grid_map -b foxy-devel
cd ~/ros2_ws
```
Then extract the `grid_map_camke_helpers` and `grid_map_msgs` to your ros workspace folder
```sh
colcon build --symlink-install --packages-select grid_map_camke_helpers 
colcon build --symlink-install --packages-select grid_map_msgs 
colcon build --symlink-install --packages-select pointcloud_to_grid 
```

## <h2 style="color:#1F1F1F; background-color:#B0B0B0; text-align:center; text-style:bold; font-family:'Chalkboard' ;">Envision Pointcloud to grid</h2>

### **Custom Pointcloud to Grid Node**

To set up the environment, install the required ROS package:

```sh
sudo apt install ros-foxy-sensor-msgs-py
```

This project includes three main executable components:

1. ### **Goal Viewer**  
   Visualizes the map using Matplotlib, enabling users to interactively select and set navigation goals.

   To launch, run 
   ```sh

   ros2 launch envision_pointcloud_to_grid goal_viewer.launch.py
   ```

2. ### **Pointcloud to Occupancy Node**  

   #### **Overview**
   Converts pointcloud data from ORB-SLAM3 (received via the pointcloud channel) into an occupancy grid map. 

   This ROS 2 package converts sensor_msgs/PointCloud2 LIDAR data to nav_msgs/OccupancyGrid 2D map data based on height, inspired by the work of [Lucasmogsan/pointcloud_to_grid](https://github.com/Lucasmogsan/pointcloud_to_grid) which served as a foundation for this ROS 2 adaptation.

   To launch, run 
   ```sh
   ros2 launch envision_pointcloud_to_grid pointcloud_to_occupancy_node.launch.py
   ```
   The generated map is saved as a PGM file using the following service call:

   ```sh
   ros2 service call /pointcloud/save_map std_srvs/srv/Trigger
   ```

3. ### **Navigation**  
   Loads the PGM map generated from the pointcloud data, creates a path to the user-defined goal, and uses ORB-SLAM3 camera pose data to track the user's location within the map, guiding them to the target destination.

    To launch, run 
   ```sh
   ros2 launch envision_pointcloud_to_grid navigation.launch.py
   ```



