# <h1 style="color:#1F1F1F; background-color:#EDEDED; text-align:center; text-style:bold; font-family:'Chalkboard' ;">Envision </h1>


| Proj # 	| CCDS24-0982 	|
|---	|---	|
| Acad Yr/ Sem 	| 2024/2 	|
| Proj Title 	| Smart Navigation Device for the Visually Impaired 	|
| Proj Summary 	| This project aims to build wearable devices that can guide people with visual impairment in indoor navigation such as home, retail establishments. The core technology for achieving this is visual localization and mapping. When the user enters the environment, a map will be downloaded to the device. The wearable devices will run visual localization that tracks the movement of the user over time. In addition, the wearable device will a provide audio/tactile feedback to the user to ensure safe and comfortable navigation. 	|


## Pre-requisites installation guide
1. The comprehensive guide for installing all necessary pre-requisites is available [here](#installation-guide)

2. KITTI Dataset (Optional)
    
- Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php

- Execute the following command. Change KITTIX.yaml by KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change PATH_TO_DATASET_FOLDER to the uncompressed dataset folder. Change SEQUENCE_NUMBER to 00, 01, 02,.., 11.


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

## <h2 style="color:#1F1F1F; background-color:#B0B0B0; text-align:center; text-style:bold; font-family:'Chalkboard' ;"> Stereo Camera Pipeline </h2>
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

## <h2 style="color:#1F1F1F; background-color:#B0B0B0; text-align:center; text-style:bold; font-family:'Chalkboard' ;"> Running ORB SLAM 3 via ros2 </h2>

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

#### ** Saving the map**


* Option 1: If System.SaveAtlasToFile is set in the settings file, the map file will be automatically saved when you kill the ros node.

* Option 2: You can also call the following ros service at the end of the session
```sh 
# Before saving, check if the interface exist
ros2 interface show envision_interfaces/srv/SaveMap

# Check if service exist 
ros2 service list

ros2 service call /orb_slam3/save_map envision_interfaces/srv/SaveMap "{name: 'kitti_map'}"
```

## <h2 style="color:#1F1F1F; background-color:#B0B0B0; text-align:center; text-style:bold; font-family:'Chalkboard' ;"> Running Stereo Obstacle detector </h2>


#### **Prequsites**
- Need to have the ros2 topic `/stereo/left/rectified_images` and `/stereo/right/rectified_images` published using `stereo_camera_pipeline` package

#### **Note**
- Edit focal length, baseline of camera parameter within the launch file in the `ros_ws/src/stereo_obstacle_detector/launch` folder 
- Edit the SBGM parameter if needed

Run with:
```sh
ros2 launch stereo_obstacle_detector stereo_detector.launch.py
```

## <h2 style="color:#1F1F1F; background-color:#B0B0B0; text-align:center; text-style:bold; font-family:'Chalkboard' ;"> Image Masker </h2>

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


## <h2 style="color:#1F1F1F; background-color:#B0B0B0; text-align:center; text-style:bold; font-family:'Chalkboard' ;"> Navigational Speaker </h2>
```sh
pip3 install gtts
sudo apt install xdg-utils
sudo apt install mpv

# Script is run 
python3 src/text_to_speech.py
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


## <h2 style="color:#1F1F1F; background-color:#EDEDED; text-align:center; text-style:bold; font-family:'Chalkboard' ;">Installation Guide </h1>

This guide provides step-by-step instructions for installing `librealsense2` and ORB-SLAM3 with its ROS wrapper on an ARM device running Ubuntu 24.04. Use the default terminal for all commands.

Some of the commands have been extracted in courtesy of [NAIRBS - 
ORBSLAM3-Ubuntu-20.04
](https://github.com/NAIRBS/ORBSLAM3-Ubuntu-20.04/tree/93808909d9e5da211da3389934851d905aabf3f4)


    for librealsense, realsense-ros, ORBSLAM3, ORBSLAM3_ROS on ARM Device (Ubuntu 24.04)

---

## Installing librealsense2

### Update System Packages
```bash
sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
sudo apt-get install git wget build-essential
sudo apt update
sudo apt install build-essential libtool autoconf unzip
```

### Install CMake from Source
```bash
version=3.31
build=8

mkdir ~/temp
cd ~/temp
wget https://cmake.org/files/v$version/cmake-$version.$build.tar.gz
tar -xzvf cmake-$version.$build.tar.gz
cd cmake-$version.$build/
./bootstrap
make -j$(nproc)
sudo make install
cmake --version
```

### Install Dependencies
```bash
sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
```

### Clone and Build librealsense
```bash
cd ~/temp
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
sudo apt install v4l-utils
sudo ./scripts/setup_udev_rules.sh
```

### Apply Kernel Patch
For Ubuntu 24.04 (noble) with LTS kernel 6.5:
```bash
./scripts/patch-realsense-ubuntu-lts-hwe.sh
```
For Ubuntu 20.04 with LTS kernel (< 5.13):
```bash
./scripts/patch-realsense-ubuntu-lts.sh
```

### Build and Install librealsense
```bash
cmake ../ -DCMAKE_BUILD_TYPE=Release
make
sudo make -j$(($(nproc)-1)) install
```

---

## Installing ORB-SLAM3 and ROS Wrapper

### Prerequisites
1. **ROS Foxy**: Follow the [ROS Foxy installation guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).

#### Increase Swap Size
```bash
sudo swapoff -a
sudo dd if=/dev/zero of=/swapfile bs=1M count=10240
sudo chmod 0600 /swapfile
sudo mkswap /swapfile
grep Swap /proc/meminfo
```

Add to `/etc/fstab`:
```
/swapfile none swap sw 0 0
```

### Installation Steps

#### Install Basic Dependencies
```bash
sudo apt update
sudo apt install -y zip unzip
```

#### Install vcpkg
```bash
cd ~/temp
wget -qO vcpkg.tar.gz https://github.com/microsoft/vcpkg/archive/master.tar.gz
sudo mkdir /opt/vcpkg
sudo tar xf vcpkg.tar.gz --strip-components=1 -C /opt/vcpkg
sudo /opt/vcpkg/bootstrap-vcpkg.sh
sudo ln -s /opt/vcpkg/vcpkg /usr/local/bin/vcpkg
```

#### Install Additional Dependencies
```bash
sudo apt-get install -y nasm ffmpeg libmpfr-dev libgmp3-dev libmpc-dev
sudo apt-get install -y libxmu-dev libxi-dev libgl-dev
sudo apt install libgl1-mesa-dev libegl1-mesa-dev libglu1-mesa-dev libx11-dev
```

#### Install Catch2
```bash
cd ~/temp
git clone https://github.com/catchorg/Catch2.git
cd Catch2/
cmake -Bbuild -H. -DBUILD_TESTING=OFF
sudo cmake --build build/ --target install
```

#### Install Pangolin
```bash
cd ~/temp
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin/
# Edit ./scripts/install_prerequisites.sh to remove all instances of "catch2"
gedit ./scripts/install_prerequisites.sh
./scripts/install_prerequisites.sh --dry-run recommended
./scripts/install_prerequisites.sh -m apt all
```

#### Install vcpkg and Pangolin
```bash
cd ~
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.sh
./vcpkg integrate install
sudo apt install libgl1-mesa-dev libegl1-mesa-dev libglu1-mesa-dev libx11-dev
./vcpkg install pangolin
```

Verify Pangolin installation:
```bash
find ~/vcpkg/installed -name "PangolinConfig.cmake"
```

Add to `~/.bashrc`:
```bash
export CMAKE_PREFIX_PATH=~/vcpkg/installed/arm64-linux/share/pangolin
export CMAKE_PREFIX_PATH="$HOME/vcpkg/installed/arm64-linux/share/:$CMAKE_PREFIX_PATH"
```

#### Install OpenCV 4.2.0
```bash
sudo apt remove libdc1394-dev
sudo apt-get install python3-opencv
sudo apt upgrade libopencv-dev python3-opencv
python3 -c "import cv2; print(cv2.__version__)"
```

Install additional dependencies:
```bash
sudo apt install -y libgtk2.0-dev pkg-config bison libxi-dev libxtst-dev
sudo apt install -y libx11-dev libxft-dev libxext-dev libxrandr-dev
```

Install dependencies via vcpkg:
```bash
cd ~/vcpkg
sudo apt install python3-pip
pip3 install jinja2
python3 -m pip install meson ninja mako
./vcpkg install libsystemd:arm64-linux
./vcpkg install opencv4
```

Add to `~/.bashrc`:
```bash
export OpenCV_DIR=~/vcpkg/installed/arm64-linux/share/opencv4
export PATH=$PATH:~/vcpkg/installed/arm64-linux/bin
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:~/vcpkg/installed/arm64-linux/lib/pkgconfig
```

Verify OpenCV version:
```bash
pkg-config --modversion opencv4
```

#### Install ORB-SLAM3
```bash
sudo apt install libcurl4-openssl-dev libssh-dev libssl-dev
sudo apt-get install libboost-all-dev
cd ~/Desktop
git clone --recursive https://github.com/Shye0930/fyp
cd fyp
sudo apt install ros-foxy-cv-bridge
```

Afterwards follow the sh file in ./scripts/build_everything.sh



#### Build ORB-SLAM3 and ROS Wrapper
Add to `~/.bashrc`:
```bash
echo "source ~/Desktop/fyp/ros_ws/install/setup.bash" >> ~/.bashrc
```

Install ROS dependencies:
```bash
sudo apt install ros-foxy-pcl-ros
```

#### Install realsense-ros
```bash
cd ~
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src/
git clone --branch 4.56.3 https://github.com/IntelRealSense/realsense-ros.git
cd ~/ros2_ws
sudo apt-get install python3-rosdep -y
rm -rf ~/.ros/rosdep/sources.cache
sudo rosdep init
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
sudo apt install ros-foxy-diagnostic-updater
colcon build
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## References
1. [Compile CMake from Source](https://askubuntu.com/questions/355565/how-do-i-install-the-latest-version-of-cmake-from-the-command-line)
2. [librealsense2 Installation for Linux](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)
3. [librealsense Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)
4. [realsense-ros](https://github.com/IntelRealSense/realsense-ros)