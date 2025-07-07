# Stereo Obstacle Detector ROS 2 Package

## Overview
The `stereo_obstacle_detector` package provides a ROS 2 node that utilizes a rectified stereo camera pair to compute disparity and depth maps. It then processes these depth maps to detect obstacles within a defined Region of Interest (ROI) and publishes the minimum detected distance. This package is ideal for low-cost obstacle avoidance and depth sensing applications using off-the-shelf cameras.

## Features
* **Stereo Depth Estimation:** Computes disparity and depth maps using OpenCV's StereoSGBM algorithm.
* **Configurable Parameters:** All StereoSGBM parameters are exposed as ROS 2 parameters for flexible tuning.
* **Adjustable ROI:** Defines a configurable Region of Interest for obstacle detection in the center of the image.
* **Depth Filtering:** Filters out invalid, extremely close, or excessively far depth values within the ROI to prevent erroneous obstacle detections.
* **Visualizations:** Publishes normalized 8-bit disparity and depth maps for easy visualization, with configurable display ranges to "omit extremes" (pure black/white).
* **Obstacle Status:** Publishes a custom `ObstacleStatus` message indicating if an obstacle is detected and its minimum distance.

## Dependencies
* ROS 2 (foxy)
* `cv_bridge`
* `opencv-python` (installed via pip or system package)
* `numpy`
* `message_filters` (for image synchronization)
* `envision_interfaces` (a custom ROS 2 package containing the `ObstacleStatus` message, you will need to define or replace this if it's not available in your workspace)

## Installation

1.  **Clone the repository:**
    ```bash
    cd <your_ros2_workspace>/src
    git clone [https://github.com/your_username/stereo_obstacle_detector.git](https://github.com/your_username/stereo_obstacle_detector.git) # Replace with actual repo URL
    ```
2.  **Install dependencies:**
    Ensure you have `opencv-python` installed:
    ```bash
    pip install opencv-python numpy
    # For ROS 2 dependencies (if not already installed)
    sudo apt update
    sudo apt install ros-<ros2-distro>-cv-bridge ros-<ros2-distro>-image-transport ros-<ros2-distro>-message-filters
    ```
    If `envision_interfaces` is a custom message package, ensure it's in your workspace and built.
3.  **Build the workspace:**
    ```bash
    cd <your_ros2_workspace>
    colcon build
    source install/setup.bash
    ```

## Usage

### 1. Camera Setup
This node expects rectified stereo image pairs. You need a separate node (e.g., `camera_stereo_node`, `usb_cam` combined with `image_proc`) that publishes:
* `/stereo/left/rectified_images` (`sensor_msgs/msg/Image`)
* `/stereo/right/rectified_images` (`sensor_msgs/msg/Image`)

Make sure your stereo cameras are calibrated and the images are rectified correctly. Accurate rectification is crucial for good depth estimation.

### 2. Launch the Node

You can launch the `stereo_obstacle_detector` node using a launch file. An example launch file (`stereo_obstacle_detector.launch.py`) would look like this:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stereo_obstacle_detector', # Replace if your package has a different name
            executable='stereo_obstacle_detector_node', 
            name='stereo_obstacle_detector',
            output='screen',
            parameters=[
                # StereoSGBM Parameters
                {'min_disparity': 0},
                {'num_disparities': 96},        # Must be a multiple of 16
                {'block_size': 9},              # Must be an odd number
                # P1 and P2 are calculated dynamically in the code based on block_size,
                # but can be overridden here if needed.
                # {'P1': 1944}, 
                # {'P2': 7776}, 
                {'disp12_max_diff': 1},         # Strictness of left-right consistency check
                {'uniqueness_ratio': 18},       # Uniqueness percentage for matching
                {'speckle_window_size': 200},   # Size of disparity window for speckle filtering
                {'speckle_range': 4},           # Max disparity variation within speckle window

                # Obstacle Detection Threshold
                {'obstacle_distance_threshold_m': 1.5}, # Distance in meters to trigger obstacle detection

                # Camera Calibration Parameters (essential for accurate depth)
                {'focal_length': 485.9461},     # Focal length in pixels (fx from calibration matrix)
                {'baseline': 0.078453409},      # Baseline in meters (distance between camera centers)

                # Visualization & ROI Filtering Parameters
                {'display_disparity_min': 10.0},  # Min disparity value to map to black in display (farther objects)
                {'display_disparity_max': 80.0},  # Max disparity value to map to white in display (closer objects)
                {'display_depth_min_m': 0.8},     # Min depth value (meters) to consider for obstacle detection and map to white in display (filter out very close noise)
                {'display_depth_max_m': 8.0},     # Max depth value (meters) to consider for obstacle detection and map to black in display (filter out very far/background)
            ]
        )
    ])

```

## Topics
### Subscribed Topics

* /stereo/left/rectified_images (sensor_msgs/msg/Image)

* /stereo/right/rectified_images (sensor_msgs/msg/Image)

### Published Topics

* `/disparity_map` (sensor_msgs/msg/Image, mono8 encoding): An 8-bit normalized disparity map for visualization. Higher pixel values (brighter) indicate closer objects.

* `/depth_map` (sensor_msgs/msg/Image, passthrough/32FC1 encoding): The raw depth map with values in meters (float32). np.finfo(np.float32).max indicates invalid depth.

* `/depth_map_display` (sensor_msgs/msg/Image, mono8 encoding): An 8-bit normalized depth map for controlled visualization. Values are mapped such that display_depth_min_m is white and display_depth_max_m is black.

* `/obstacle_status` (envision_interfaces/msg/ObstacleStatus): Custom message containing:

    * bool obstacle_detected: True if min_distance is less than obstacle_distance_threshold_m.

    * float32 min_distance: The minimum detected depth (in meters) within the ROI, after filtering.