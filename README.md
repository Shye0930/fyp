# <h1 style="color:#1F1F1F; background-color:#EDEDED; text-align:center; text-style:bold; font-family:'Chalkboard' ;">Envision </h1>


| Proj # 	| CCDS24-0982 	|
|---	|---	|
| Acad Yr/ Sem 	| 2024/2 	|
| Proj Title 	| Smart Navigation Device for the Visually Impaired 	|
| Proj Summary 	| This project aims to build wearable devices that can guide people with visual impairment in indoor navigation such as home, retail establishments. The core technology for achieving this is visual localization and mapping. When the user enters the environment, a map will be downloaded to the device. The wearable devices will run visual localization that tracks the movement of the user over time. In addition, the wearable device will a provide audio/tactile feedback to the user to ensure safe and comfortable navigation. 	|

# Project Setup and Execution Guide

## Pre-requisites installation guide
1. The comprehensive guide for installing all necessary pre-requisites is available [here](./assets/Installation_guide.md)

2. KITTI Dataset (Optional)
    
- Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php

- Execute the following command. Change KITTIX.yaml by KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change PATH_TO_DATASET_FOLDER to the uncompressed dataset folder. Change SEQUENCE_NUMBER to 00, 01, 02,.., 11.


## Building the Project
**[Warning]** Ensure at least 13 GB of RAM is available for the build process; otherwise, activate swap space to avoid issues.

Navigate to the `FYP` folder and execute the following commands to build the project:

```sh
chmod +x scripts/build_everything.sh scripts/rebuild_essential.sh
./scripts/build_everything.sh
```

After the initial build with `build_everything.sh`, if changes are made to `ORB_SLAM3_COMMUNITY` or `ros_ws/orbslam3`, update the build by running:

```sh
./scripts/rebuild_essential.sh
```

## Calibration of IP camera
Calibration of cameras in stereo vision is crucial to accurately determine the relative position and orientation between the two cameras. It ensures that corresponding points in the image pairs can be correctly matched, which is essential for precise depth estimation. Without calibration, 3D reconstruction and distance measurements would be unreliable and distorted.

Refer to the calibration tutorial located at:
```md
├── camera
│   ├── arduino
│   └── **camera_calibration**
```

Tutorial to calibrate [here](./camera/camera_calibration/README.md) 


## Mapping
[ ] Come out with the steps to perform mapping
[ ] See if I need to think about merging the old pgm maps and stuff to create a bigger occu map

## Navigation 
[ ] Come out with the steps to perform navigation 


## List of ros2 packages used in the project
Below is a comprehensive list of the ROS 2 packages integrated into this project. Each package is linked to its respective section for detailed setup and execution instructions.

- [Stereo Camera Pipeline](#stereo-camera-pipeline): Processes and publishes rectified stereo images for downstream applications.
- [ORB-SLAM3 ROS](#running-orb-slam-3-via-ros2): Implements the ORB-SLAM3 with a ros wrapper to publish point cloud, camera pose and etc.
- [Stereo Obstacle Detector](#running-stereo-obstacle-detector): Detects obstacles using stereo vision data for obstacle avoidance purposes.
- [Image Masker](#image-masker): Applies segmentation-based masking to stereo images using YOLO models.
- [Navigational Speaker](#navigational-speaker): Converts text to speech for navigational audio feedback.
- ~~[Pointcloud to Grid](#pointcloud-to-grid): Converts point cloud data into 2D occupancy grids for mapping and navigation.~~ (Package not in used cause it requires lidar to calculate intensity)
- [Envision Pointcloud to Grid](#envision-pointcloud-to-grid): Converts point cloud data into 2D occupancy grids for mapping, goal mapping and navigation. 

Explore each section for detailed configuration and usage instructions specific to each package.


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


#### **Prequsites**
- Ensure the ROS 2 topics `/stereo/left/rectified_images` and `/stereo/right/rectified_images` are published using the `stereo_camera_pipeline` package.

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
The `pointcloud_to_grid` package, developed by JKK Research, converts `sensor_msgs/PointCloud2` LIDAR data into `nav_msgs/OccupancyGrid` 2D map data based on intensity and/or height. For more details, visit the repository: [jkk-research/pointcloud_to_grid](https://github.com/jkk-research/pointcloud_to_grid).[](https://github.com/jkk-research/pointcloud_to_grid)

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

### My own pointcloud to grid node:

sudo apt install ros-foxy-sensor-msgs-py

### To save map
ros2 service call /pointcloud/save_map std_srvs/srv/Trigger



## <h2 style="color:#1F1F1F; background-color:#B0B0B0; text-align:center; text-style:bold; font-family:'Chalkboard' ;">ROS2 Bag Record Misc Stuff</h2>

### Record stereo (w/ realsense)
```sh
ros2 bag record -o my_house_stereo /orb_slam3/all_points /orb_slam3/kf_markers /orb_slam3/kf_markers_array /orb_slam3/tracked_points /orb_slam3/tracking_image /orb_slam3_ros/trajectory /camera/camera/infra1/image_rect_raw /camera/camera/infra2/image_rect_raw 
```
### Record rgbd (w/ realsense)
```sh
ros2 bag record -o my_house_stereo /orb_slam3/all_points /orb_slam3/kf_markers /orb_slam3/kf_markers_array /orb_slam3/tracked_points /orb_slam3/tracking_image /orb_slam3_ros/trajectory /camera/camera/color/image_raw /camera/camera/aligned_depth_to_color/image_raw
```



## <h2 style="color:#1F1F1F; background-color:#B0B0B0; text-align:center; text-style:bold; font-family:'Chalkboard' ;"> Debug info used throughout the repo </h2>

[INFO]: For informational messages, especially when the code is doing something expected but noteworthy (e.g., successful initialization, a specific mode being activated).

[DEBUG]: For messages related to debugging specific issues or temporary print statements you might remove later.

[WARN]: For potential issues that don't stop execution but might lead to problems (e.g., a default value being used because a parameter was missing).

[ERROR]: For critical errors that prevent the code from functioning correctly.

[TODO]: For features or improvements that need to be implemented.

[FIXME]: For known bugs that need fixing.

[HACK]: For a workaround that might not be the cleanest solution but gets the job done for now.

[NOTE]: For important design decisions or caveats.



