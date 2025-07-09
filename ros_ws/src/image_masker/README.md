# Image Masker ROS2 Package

## Overview

The `image_masker` package is a ROS2 Python node designed for real-time processing of stereo camera feeds. It subscribes to rectified left and right image topics, utilizes a YOLO (You Only Look Once) segmentation model to detect and mask out persons in the images, and then publishes the processed images. This package is particularly useful for applications like SLAM (Simultaneous Localization and Mapping) where dynamic objects (such as people) can introduce noise and inaccuracies into the mapping process.

The masked regions (where persons are detected) are filled with a white color. The node also calculates and logs the Frames Per Second (FPS) for both incoming stereo streams. To enhance performance, image processing for left and right images is handled concurrently using Python's `threading` module.


## Install Ultralytics
```sh
pip install ultralytics
```

## Features

* **Stereo Image Subscription:** Subscribes to `/stereo/left/rectified_images` and `/stereo/right/rectified_images` topics.
* **Time Synchronization:** Uses `message_filters.ApproximateTimeSynchronizer` to ensure left and right images are processed as synchronized pairs.
* **Real-time Person Segmentation:** Integrates a YOLO segmentation model (recommended: `yolov8n-seg.pt`) to identify persons in the image.
* **Person Masking:** Replaces detected person regions with a solid white color in the output images.
* **FPS Calculation:** Continuously monitors and logs the FPS for both incoming image streams.
* **Multi-threading:** Processes left and right images concurrently using separate threads to improve throughput.
* **Flexible Image Handling:** Automatically adapts to both grayscale and color (BGR) input image formats.
* **ROS2 Parameterization:** Allows configuration of the YOLO model path, person class ID, and publishing flag via ROS2 parameters.

## Prerequisites

* **ROS2 Environment:** A working ROS2 installation (e.g., Humble, Iron, Jazzy).
* **Python 3:** ROS2 Python packages require Python 3.
* **Python Libraries:**
    * `rclpy` (ROS2 Python client library)
    * `sensor_msgs` (ROS2 Image message type)
    * `cv_bridge` (ROS2-OpenCV interface)
    * `opencv-python` (OpenCV library)
    * `numpy` (Numerical operations)
    * `ultralytics` (YOLO implementation)
    * `message_filters` (for topic synchronization)
    * `torch` (PyTorch, required by Ultralytics YOLO)

You can install the Python dependencies using pip:

```sh
sudo apt update
sudo apt install ros-foxy-cv-bridge ros-foxy-message-filters
```


## Building the package
```sh
colcon build --symlink-install --packages-select image_masker
source ~/.bashrc
```

## Usage
To launch the image_masker node, use the provided launch file:
```sh
ros2 launch image_masker image_masker.launch.py
```

### Subscribed Topics

* `/stereo/left/rectified_images (sensor_msgs/msg/Image)`

* `/stereo/right/rectified_images (sensor_msgs/msg/Image)`

### Published Topics

    /stereo/left/rect/masked (sensor_msgs/msg/Image) - The left image with persons masked in white.

    /stereo/right/rect/masked (sensor_msgs/msg/Image) - The right image with persons masked in white.

### Node Parameters

You can modify these parameters in the image_masker.launch.py file:

`yolo_model_path (string)`: Path to your YOLO segmentation model file (e.g., yolov8n-seg.pt). Default: `$(find-pkg-share image_masker)/model/yolov8n-seg.pt`

`person_class_id (integer)`: The class ID corresponding to 'person' in your YOLO model's training dataset (e.g., 0 for COCO dataset). Default: `0`

`publish_masked_images (boolean)`: Set to true to enable publishing of the masked images, false otherwise. Default: true

### Example Workflow

1. Start your stereo camera driver that publishes to /stereo/left/rectified_images and /stereo/right/rectified_images.

2. Launch the image_masker node:
Bash

```sh
ros2 launch image_masker image_masker.launch.py
```