## <h2 style="color:#1F1F1F; background-color:#EDEDED; text-align:center; text-style:bold; font-family:'Chalkboard' ;">Installation Guide</h1>

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
sudo apt install build-essential libtool autoconf unzip openssl libssl-dev
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
mkdir build && cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release
make
sudo make -j$(($(nproc)-1)) install
```

---

### Installing ORB-SLAM3 and ROS Wrapper

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
sudo rosdep init  --include-eol-distros #for Foxy and earlier
rosdep update --include-eol-distros #for Foxy and earlier
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
sudo apt install ros-foxy-diagnostic-updater
colcon build
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Installing Micro Ros

```bash
cd ~/Desktop/fyp
mkdir uros_ws && cd uros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
sudo rosdep update --include-eol-distros && rosdep install --from-paths src --ignore-src -y
colcon build
echo "source ~/Desktop/fyp/uros_ws/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### Building micro-ROS-Agent

Using this package is possible to install a ready to use micro-ROS-Agent:
```sh
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source ~/.bashrc
```

### Add the micro ros Arduino library to Arduino (Foxy)

Follow these steps to add the micro-ROS library for the Foxy distribution to your Arduino IDE:

1. **Download the Library**:
   - Visit the micro-ROS Arduino repository: [https://github.com/micro-ROS/micro_ros_arduino/tree/foxy](https://github.com/micro-ROS/micro_ros_arduino/tree/foxy).
   -  "Download ZIP" to download the `micro_ros_arduino-foxy.zip` file.

2. **Add the Library to Arduino IDE**:
   - Open the Arduino IDE on your computer.
   - Navigate to **Sketch > Include Library > Add .ZIP Library**.
   - Locate and select the downloaded `micro_ros_arduino-foxy.zip` file.
   - Click **Open** to import the library.

3. **Verify Installation**:
   - Go to **File > Examples** in the Arduino IDE.
   - Look for the `micro_ros_arduino` section in the examples menu to confirm the library is installed correctly.
   - You can test the library by opening one of the example sketches (e.g., `micro-ros_publisher`).
---

## References
1. [Compile CMake from Source](https://askubuntu.com/questions/355565/how-do-i-install-the-latest-version-of-cmake-from-the-command-line)
2. [librealsense2 Installation for Linux](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)
3. [librealsense Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)
4. [realsense-ros](https://github.com/IntelRealSense/realsense-ros)