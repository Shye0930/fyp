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
- [ ] Come out with the steps to perform mapping

- [ ] See if I need to think about merging the old pgm maps and stuff to create a bigger occu map

## Navigation 
- [ ] Come out with the steps to perform navigation 


## List of ros2 packages used in the project
Below is a comprehensive list of the ROS 2 packages integrated into this project. Each package is linked to its respective section for detailed setup and execution instructions.

- [Stereo Camera Pipeline](./assets/project_components.md#stereo-camera-pipeline): Processes and publishes rectified stereo images for downstream applications.
- [ORB-SLAM3 ROS](#running-orb-slam-3-via-ros2): Implements the ORB-SLAM3 with a ros wrapper to publish point cloud, camera pose and etc.
- [Stereo Obstacle Detector](#running-stereo-obstacle-detector): Detects obstacles using stereo vision data for obstacle avoidance purposes.
- [Image Masker](#image-masker): Applies segmentation-based masking to stereo images using YOLO models.
- [Navigational Speaker](#navigational-speaker): Converts text to speech for navigational audio feedback.
- ~~[Pointcloud to Grid](#pointcloud-to-grid): Converts point cloud data into 2D occupancy grids for mapping and navigation.~~ (Package not in used cause it requires lidar to calculate intensity)
- [Envision Pointcloud to Grid](#envision-pointcloud-to-grid): Converts point cloud data into 2D occupancy grids for mapping, goal mapping and navigation. 

Explore each section for detailed configuration and usage instructions specific to each package.



## <h2 style="color:#1F1F1F; background-color:#B0B0B0; text-align:center; text-style:bold; font-family:'Chalkboard' ;"> Debug info used throughout the repo </h2>

[INFO]: For informational messages, especially when the code is doing something expected but noteworthy (e.g., successful initialization, a specific mode being activated).

[DEBUG]: For messages related to debugging specific issues or temporary print statements you might remove later.

[WARN]: For potential issues that don't stop execution but might lead to problems (e.g., a default value being used because a parameter was missing).

[ERROR]: For critical errors that prevent the code from functioning correctly.

[TODO]: For features or improvements that need to be implemented.

[FIXME]: For known bugs that need fixing.

[HACK]: For a workaround that might not be the cleanest solution but gets the job done for now.

[NOTE]: For important design decisions or caveats.



