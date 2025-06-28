//Dependencies required
0. Brew
1. Pangolin
2. OpenCV
3. Eigen3
4. DBoW2 and g2o (Included in Thirdparty folder) << Already in ORB SLAM
5. Python
6. ROS (Optional)

1. Pangolin (ATTEMPT 2)
// Try with this installation method >> https://lindevs.com/install-vcpkg-on-ubuntu
//1a. Install vcpkg?
cd ~
sudo apt update
sudo apt install -y zip unzip
sudo apt install -y build-essential pkg-config
wget -qO vcpkg.tar.gz https://github.com/microsoft/vcpkg/archive/master.tar.gz
sudo mkdir /opt/vcpkg
sudo tar xf vcpkg.tar.gz --strip-components=1 -C /opt/vcpkg
sudo /opt/vcpkg/bootstrap-vcpkg.sh
sudo ln -s /opt/vcpkg/vcpkg /usr/local/bin/vcpkg
vcpkg version
rm -rf vcpkg.tar.gz

//Install pangolin through vcpkg: (dependencies are require for this dependency install)
sudo apt-get install nasm
sudo apt install ffmpeg

//Install this for whatever reason
sudo apt install libmpfr-dev libgmp3-dev libmpc-dev
sudo apt-get install libxmu-dev libxi-dev libgl-dev

<!-- git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd ~/Pangolin/
./scripts/install_prerequisites.sh --dry-run recommended
./scripts/install_prerequisites.sh -m apt all -->

cd ~

sudo apt install cmake

git clone https://github.com/catchorg/Catch2.git
cd Catch2
cmake -Bbuild -H. -DBUILD_TESTING=OFF
sudo cmake --build build/ --target install

cd ~

git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.sh
./vcpkg integrate install

sudo apt install libgl1-mesa-dev libegl1-mesa-dev libglu1-mesa-dev libx11-dev


./vcpkg install pangolin

find ~/vcpkg/installed -name "PangolinConfig.cmake"
//returns something similar to:
/home/nairb/vcpkg/installed/x64-linux/share/pangolin/PangolinConfig.cmake
//adjust below so it builds properly, however it should be the about the same, YOU NEED BOTH OF THIS
export CMAKE_PREFIX_PATH=~/vcpkg/installed/arm64-linux/share/pangolin
export CMAKE_PREFIX_PATH=~/vcpkg/installed/arm64-linux/share

2. OpenCV
cd ~
sudo apt-get install python3-opencv
sudo apt upgrade libopencv-dev python3-opencv
sudo apt install idle
idle //This will open a python gui window
// In this window
import cv2 as cv
print(cv.__version__)
// Check no issues
----------------------------------------------- NEW STUFF ------------------
sudo apt update
sudo apt install libgtk2.0-dev pkg-config
cd ~/vcpkg
git pull
./vcpkg update
pip3 install jinja2
python3 -m pip install meson ninja jinja2 mako
sudo apt-get install bison
sudo apt-get install libxi-dev libxtst-dev
sudo apt install libx11-dev libxft-dev libxext-dev
sudo apt install libxrandr-dev
./vcpkg install libsystemd:arm64-linux
./vcpkg install opencv4

## If have error while using vcpkg for libffi
nano ~/vcpkg/ports/libffi/portfile.cmake
# comment out lines in line 22 and line 23
#vcpkg_cmake_get_vars(... ADDITIONAL_LANGUAGES ASM) << remove stuff in brackets
#include("${cmake_vars_file}")

## The command below should show opencv 4.11.0 not 4.2.0
python3 -c "import cv2; print(cv2.__version__)"

export OpenCV_DIR=~/vcpkg/installed/x64-linux/share/opencv4
export PATH=$PATH:~/vcpkg/installed/x64-linux/bin
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:~/vcpkg/installed/x64-linux/lib/pkgconfig
pkg-config --modversion opencv4

sudo apt install python3-pip
pip install --upgrade numpy
pip install --upgrade opencv-python


# Use bigger swap file just in case
sudo swapoff /swapfile
sudo rm /swapfile

sudo fallocate -l 32G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

export CMAKE_PREFIX_PATH=~/vcpkg/installed/arm64-linux/share/pangolin
export CMAKE_PREFIX_PATH=~/vcpkg/installed/arm64-linux/share

ETC. Install random other libaries at this point...
sudo apt-get install libboost-all-dev

cd ~/ORB_SLAM3
./build.sh

## After it rebuilds go onto rebuild the ROS2 Wrapper and workspace.


----------------------------------------------- NEW STUFF ------------------
3. Eigen
cd ~
sudo apt update
sudo apt install libeigen3-dev
sudo apt update && sudo apt upgrade -y

<!-- // Install orb slam 3 from https://github.com/UZ-SLAMLab/ORB_SLAM3
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3

sudo apt install libcurl4-openssl-dev
sudo apt install libssh-dev libssl-dev

cd ORB_SLAM3
chmod +x build.sh

// You need to change these files before building
cd ~/ORB_SLAM3/Examples/Monocular
nano mono_tum_vi.cc

// Change the last false, 0, file_name into true
// Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout << endl << "-------" << endl;
    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,false, 0, file_name);
    float imageScale = SLAM.GetImageScale();


cd ..


// update Cmakelists.txt from -std=c++11 to -std=c++14
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

// install this for more RAM??
sudo apt install zram-config
// add a swap file
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab


// Finally start building, you might see some red stuff, believe in the script and keep waiting until it is done.
./build.sh

TESTING:

mkdir -p ~/Datasets/TUM-VI
cd ~/Datasets/TUM-VI
// wget https://vision.in.tum.de/tumvi/exported/euroc/1024_16/dataset-corridor4_1024_16.tar << don't install this first
wget https://vision.in.tum.de/tumvi/exported/euroc/512_16/dataset-corridor4_512_16.tar
tar -xf dataset-corridor4_1024_16.tar


cd ~/ORB_SLAM3/
nano ~/ORB_SLAM3/tum_vi_examples.sh
// Look at the tele file and copy pastes

chmod +x ~/ORB_SLAM3/tum_vi_examples.sh

// Run the example
./tum_vi_examples.sh -->
