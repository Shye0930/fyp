## <h2 style="color:#1F1F1F; background-color:#B0B0B0; text-align:center; text-style:bold; font-family:'Chalkboard' ;">Stereo Camera Pipeline</h2>

When utilizing `stereo_processor_node.py` directly or launching it via `ros2 launch stereo_camera_pipeline stereo_pipeline.launch.py`, it is necessary to employ a dedicated configuration file. This is due to differences in the expected YAML format, particularly concerning directives like `%YAML`, which the Python processing in this package may not handle in the same manner as ORB-SLAM3's configuration loader.

For a compatible configuration structure, please consult `config/stereo/stereo_calibration.yaml`. This file serves as a direct reference for inputting parameters suitable for the stereo_camera_pipeline package.

Conversely, `config/stereo/Esp32s.yaml` provides an example of the specific configuration format used for ORB-SLAM3 when working with rectified stereo inputs.

If using the `stereo_camera_pipeline` package, ensure the YAML file in the `config` directory is updated, as it serves as the fallback configuration if the provided path is invalid.

### Publishing Rectified Stereo Frames

To publish rectified stereo frames, you can choose one of the following methods:

#### **1. Using the Python Script (`stereo_processor_node.py`)**

If you're running the node directly from the Python script located at `src/stereo_processor_node.py`, update the calibration file path on **line 137** to point to your desired YAML calibration file.

#### **2. Using the ROS 2 Launch System**

Launch the stereo processing pipeline with:

```sh
ros2 launch stereo_camera_pipeline stereo_pipeline.launch.py
```

## <h2 style="color:#1F1F1F; background-color:#B0B0B0; text-align:center; text-style:bold; font-family:'Chalkboard' ;">Running ORB SLAM 3 via ros2</h2>

#### **Prequsites**
- Ensure the ROS 2 topics `/stereo/left/rectified_images` and `/stereo/right/rectified_images` are published using the `stereo_camera_pipeline` package.

### Execution Commands
Run the following commands in the `FYP` folder:

#### For Intel RealSense D435 (RGB-D Mode)
```sh
# For Intel realsense d435, run for rgbd
ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true enable_infra1:=false enable_infra2:=false align_depth.enable:=true enable_sync:=true rgb_camera.color_profile:=640x480x30 depth_module.depth_profile:=640x480x30

# Enable IR emitter
ros2 param set /camera/camera depth_module.emitter_enabled 1

ros2 launch orbslam3 d435_rgbd.launch.py
```

#### For Intel RealSense D435 (Stereo Mode)
```sh
ros2 launch realsense2_camera rs_launch.py enable_color:=false enable_depth:=false enable_infra1:=true enable_infra2:=true depth_module.infra_profile:=640x480x30 

# Disable IR emitter
ros2 param set /camera/camera depth_module.emitter_enabled 0


ros2 launch orbslam3 d435_stereo.launch.py
```

#### To Publish Stereo Frames
```sh
ros2 launch stereo_camera_pipeline stereo_pipeline.launch.py 
```

#### To Run ORB-SLAM3
```sh
ros2 run orbslam3 stereo /home/shye/Desktop/projects/fyp/ORB_SLAM_3_COMMUNITY/Vocabulary/ORBvoc.txt /home/shye/Desktop/projects/fyp/config/stereo/Esp32s.yaml

#OR via launch file
ros2 launch orbslam3 d435_stereo.launch.py
```

#### To Launch RViz
```sh
ros2 run rviz2 rviz2 -d config/stereo/orb_slam3_no_imu.rviz
```

### Saving the Map

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

### **Prequsites**
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

### Notes
- Update the focal length and baseline parameters in the launch file located at `ros_ws/src/stereo_obstacle_detector/launch`.
- Adjust the SGBM parameters if necessary.

Run with:
```sh
ros2 launch stereo_obstacle_detector stereo_detector.launch.py
```

## <h2 style="color:#1F1F1F; background-color:#B0B0B0; text-align:center; text-style:bold; font-family:'Chalkboard' ;">Image Masker</h2>

### Prerequisites
- Ensure the ROS 2 topics `/stereo/left/rectified_images` and `/stereo/right/rectified_images` are published (e.g., by the `stereo_camera_pipeline` package).

#### **Note**
Place your YOLO segmentation model (e.g., `yolov8n-seg.pt`) in the `ros_ws/src/image_masker/model/` directory.
- **[Warning]** The YOLO model must support segmentation for proper functionality.
- Update parameters such as `yolo_model_path`, `person_class_id`, or `publish_masked_images` in the launch file located at `ros_ws/src/image_masker/launch/` if needed.
- Masked images will be published to `/stereo/left/rect/masked` and `/stereo/right/rect/masked`.

Run with:
```sh
ros2 launch image_masker image_masker.launch.py
```


## <h2 style="color:#1F1F1F; background-color:#B0B0B0; text-align:center; text-style:bold; font-family:'Chalkboard' ;">Navigational Speaker</h2>
### Prerequisites
Install the required dependencies:
```sh
pip3 install gtts
sudo apt install xdg-utils
sudo apt install mpv
```

### Execution
Run the script:
```sh
python3 src/text_to_speech.py
```

## <h2 style="color:#1F1F1F; background-color:#B0B0B0; text-align:center; text-style:bold; font-family:'Chalkboard' ;">Pointcloud to grid</h2>

### Overview
The `pointcloud_to_grid` package, developed by JKK Research, converts `sensor_msgs/PointCloud2` LIDAR data into `nav_msgs/OccupancyGrid` 2D map data based on intensity and/or height. For more details, visit the repository: [jkk-research/pointcloud_to_grid](https://github.com/jkk-research/pointcloud_to_grid).

### Prerequisites
**[Warning]** Ensure the following dependencies are installed and built in the correct order to avoid compilation issues.

#### Required Dependencies
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

#### Build Order
To successfully build the `pointcloud_to_grid` package, follow this specific order:
1. Build the `grid_map_camke_helpers` CMake helpers first.
2. Build the `grid_map_msgs` package next.
3. Finally, build the `pointcloud_to_grid` package.

Example build commands in your ROS 2 workspace:
```sh
cd ~/ros2_ws/src
git clone https://github.com/ANYbotics/grid_map -b foxy-devl
cd ~/ros2_ws
```
Then extract the `grid_map_camke_helpers` and `grid_map_msgs` to your ros workspace folder
```sh
colcon build --symlink-install --packages-select grid_map_camke_helpers 
colcon build --symlink-install --packages-select grid_map_msgs 
colcon build --symlink-install --packages-select pointcloud_to_grid 
```

## <h2 style="color:#1F1F1F; background-color:#B0B0B0; text-align:center; text-style:bold; font-family:'Chalkboard' ;">Envision Pointcloud to grid</h2>

### Custom Pointcloud to Grid Node

To set up the environment, install the required ROS package:

```bash
sudo apt install ros-foxy-sensor-msgs-py
```

This project includes three main executable components:

1. **Goal Viewer**  
   Visualizes the map using Matplotlib, enabling users to interactively select and set navigation goals.

   To launch, run 
   ```sh

   ros2 launch envision_pointcloud_to_grid goal_viewer.launch.py
   ```

2. **Pointcloud to Occupancy Node**  
   Converts pointcloud data from ORB-SLAM3 (received via the pointcloud channel) into an occupancy grid map. 

   To launch, run 
   ```sh
   ros2 launch envision_pointcloud_to_grid pointcloud_to_occupancy_node.launch.py
   ```
   The generated map is saved as a PGM file using the following service call:

   ```bash
   ros2 service call /pointcloud/save_map std_srvs/srv/Trigger
   ```

3. **Navigation**  
   Loads the PGM map generated from the pointcloud data, creates a path to the user-defined goal, and uses ORB-SLAM3 camera pose data to track the user's location within the map, guiding them to the target destination.

    To launch, run 
   ```sh
   ros2 launch envision_pointcloud_to_grid navigation.launch.py
   ```


## <h2 style="color:#1F1F1F; background-color:#B0B0B0; text-align:center; text-style:bold; font-family:'Chalkboard' ;">ROS2 Bag Record Misc Stuff</h2>

### Record stereo (w/ realsense)
```sh
ros2 bag record -o my_house_stereo /orb_slam3/all_points /orb_slam3/kf_markers /orb_slam3/kf_markers_array /orb_slam3/tracked_points /orb_slam3/tracking_image /orb_slam3_ros/trajectory /camera/camera/infra1/image_rect_raw /camera/camera/infra2/image_rect_raw 
```
### Record rgbd (w/ realsense)
```sh
ros2 bag record -o my_house_rgbd /orb_slam3/all_points /orb_slam3/kf_markers /orb_slam3/kf_markers_array /orb_slam3/tracked_points /orb_slam3/tracking_image /orb_slam3_ros/trajectory /camera/camera/color/image_raw /camera/camera/aligned_depth_to_color/image_raw
```

