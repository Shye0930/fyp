# TODO: Only to rebuild slam and ros wrapper
# This file is meant to be run only after you have build everything


echo " "
echo "Configuring and building ORB_SLAM3 ..."
cd ../ORB_SLAM_3_COMMUNITY/
rm -rf build 
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)


echo " "
echo "Configuring and building ROS2 Wrapper for ORB_SLAM3 ..."
cd ../ros_ws
colcon build --symlink-install --packages-select orbslam3 --cmake-clean-cache