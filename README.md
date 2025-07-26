# Envision


| Proj # 	| CCDS24-0982 	|
|---	|---	|
| Acad Yr/ Sem 	| 2024/2 	|
| Proj Title 	| Smart Navigation Device for the Visually Impaired 	|
| Proj Summary 	| This project aims to build wearable devices that can guide people with visual impairment in indoor navigation such as home, retail establishments. The core technology for achieving this is visual localization and mapping. When the user enters the environment, a map will be downloaded to the device. The wearable devices will run visual localization that tracks the movement of the user over time. In addition, the wearable device will a provide audio/tactile feedback to the user to ensure safe and comfortable navigation. 	|


## Pre-requisites installation guide
1. The comprehensive guide for installing all necessary pre-requisites is available [here](https://github.com/NAIRBS/ORBSLAM3-Ubuntu-20.04)


## Setting up swap
This is only relevant if your system has less than 16 GB of ram. The guide below is found [here](https://askubuntu.com/questions/178712/how-to-increase-swap-space)

```sh
Resize Swap to 8GB

# Turn swap off
# This moves stuff in swap to the main memory and might take several minutes
sudo swapoff -a

# Create an empty swapfile
# Note that "1M" is basically just the unit and count is an integer.
# Together, they define the size. In this case 8GiB.
sudo dd if=/dev/zero of=/swapfile bs=1M count=8192

# Set the correct permissions
sudo chmod 0600 /swapfile

sudo mkswap /swapfile  # Set up a Linux swap area
sudo swapon /swapfile  # Turn the swap on

grep Swap /proc/meminfo # To check if it worked
```
### To make the swap permanent (persist on restart)
Add this line to the end of your /etc/fstab:
```vim
/swapfile none swap sw 0 0
```


## Building of the whole project 
**[WARN]** It is crucial to note that you should have at least 13 GB of ram available to build else you will have to activate swap

Within the FYP folder run
```sh
chmod +x scripts/build_everything.sh scripts/rebuild_essential.sh
scripts/build_everything.sh
```
After running build_everything once, any subsequent edits to ORB_SLAM3_COMMUNITY or ros_ws/orbslam3 require only running scripts/rebuild_essential.sh to update the build.
```sh
scripts/rebuild_essentials.sh
```

## Calibration of IP camera
Calibration of cameras in stereo vision is crucial to accurately determine the relative position and orientation between the two cameras. It ensures that corresponding points in the image pairs can be correctly matched, which is essential for precise depth estimation. Without calibration, 3D reconstruction and distance measurements would be unreliable and distorted.

Find the tutorial to calibrate [here](./camera/camera_calibration/README.md) which is located 
```md
├── camera
│   ├── arduino
│   └── **camera_calibration**
```

## Editing of different config file
When utilizing `stereo_processor_node.py` directly or launching it via `ros2 launch stereo_camera_pipeline stereo_pipeline.launch.py`, it is necessary to employ a dedicated configuration file. This is due to differences in the expected YAML format, particularly concerning directives like `%YAML`, which the Python processing in this package may not handle in the same manner as ORB-SLAM3's configuration loader.

For a compatible configuration structure, please consult `config/stereo/stereo_calibration.yaml`. This file serves as a direct reference for inputting parameters suitable for the stereo_camera_pipeline package.

Conversely, `config/stereo/Esp32s.yaml` provides an example of the specific configuration format used for ORB-SLAM3 when working with rectified stereo inputs.

Addtionally, if you use the ros2 stereo_camera_pipeline package, do modify the YAML under the config directory as that is the config the node will fallback to if the path provided is invalid

### Publishing Rectified Stereo Frames

To publish rectified stereo frames, you can choose one of the following methods:

#### **1. Using the Python Script (`stereo_processor_node.py`)**

If you're running the node directly from the Python script located at `src/stereo_processor_node.py`, update the calibration file path on **line 137** to point to your desired YAML calibration file.

#### **2. Using the ROS 2 Launch System**

If you're using the `stereo_camera_pipeline` ROS 2 package, you can launch the stereo processing pipeline using either of the following commands:

```sh
ros2 launch stereo_camera_pipeline stereo_pipeline.launch.py
```

## Running ORB SLAM 3 via ros2

#### **Prequsites**
- Need to have the ros2 topic `/stereo/left/rectified_images` and `/stereo/right/rectified_images` published using `stereo_camera_pipeline` package

Within the FYP folder run
```sh
# To publish the frames of the 2 cameras
ros2 launch stereo_camera_pipeline stereo_pipeline.launch.py 

# To boot up orb slam3
ros2 run orbslam3 stereo /home/shye/Desktop/projects/fyp/ORB_SLAM_3_COMMUNITY/Vocabulary/ORBvoc.txt /home/shye/Desktop/projects/fyp/config/stereo/Esp32s.yaml

# To launch Rviz
ros2 run rviz2 rviz2 -d config/stereo/orb_slam3_no_imu.rviz
```

## Running Stereo Obstacle detector

#### **Prequsites**
- Need to have the ros2 topic `/stereo/left/rectified_images` and `/stereo/right/rectified_images` published using `stereo_camera_pipeline` package

#### **Note**
- Edit focal length, baseline of camera parameter within the launch file in the `ros_ws/src/stereo_obstacle_detector/launch` folder 
- Edit the SBGM parameter if needed

Run with:
```sh
ros2 launch stereo_obstacle_detector stereo_detector.launch.py
```

## Image Masker

#### **Prequsites**
- Need to have the ROS2 topics `/stereo/left/rectified_images` and `/stereo/right/rectified_images` published (e.g., by a stereo camera driver like `stereo_camera_pipeline`).

#### **Note**
- Ensure your chosen YOLO segmentation model (e.g., `yolov8n-seg.pt`) is placed in the `ros_ws/src/image_masker/model/` directory. 
- **[WARN]** You need a YOLO model that supports segmentation for it to work!
- Edit the `yolo_model_path`, `person_class_id`, or `publish_masked_images` parameters within the launch file located in the `ros_ws/src/image_masker/launch/` folder if needed.
- The masked images will be published to `/stereo/left/rect/masked` and `/stereo/right/rect/masked`.

Run with:
```sh
ros2 launch image_masker image_masker.launch.py
```


## Navigational speaker
```sh
pip3 install gtts
sudo apt install xdg-utils
sudo apt install mpv


# Script is run 
python3 src/text_to_speech.py
```

## Debug info used through the repo

[INFO]: For informational messages, especially when the code is doing something expected but noteworthy (e.g., successful initialization, a specific mode being activated).

[DEBUG]: For messages related to debugging specific issues or temporary print statements you might remove later.

[WARN]: For potential issues that don't stop execution but might lead to problems (e.g., a default value being used because a parameter was missing).

[ERROR]: For critical errors that prevent the code from functioning correctly.

[TODO]: For features or improvements that need to be implemented.

[FIXME]: For known bugs that need fixing.

[HACK]: For a workaround that might not be the cleanest solution but gets the job done for now.

[NOTE]: For important design decisions or caveats.