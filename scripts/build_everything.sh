echo "Configuring and building Thirdparty/DBoW2 ..."
cd ../ORB_SLAM_3_COMMUNITY/Thirdparty/DBoW2
rm -rf build 
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc) # j4 to limit the number of cores so my computer dont go crazy


echo " "
echo "Configuring and building Thirdparty/g2o ..."
cd ../../g2o
rm -rf build 
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

echo " "
echo "Configuring and building Thirdparty/Sophus ..."
cd ../../Sophus
rm -rf build 
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install

echo " "
echo "Uncompress vocabulary ..."
cd ../../../Vocabulary
rm -f ORBvoc.bin
rm -f ORBvoc.txt
tar -xf ORBvoc.txt.tar.gz

echo " "
echo "Configuring and building ORB_SLAM3 ..."
cd ..
rm -rf build 
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

echo " "
echo "Converting vocabulary to binary"
cd ..
./tools/bin_vocabulary


echo " "
echo "Building ROS2 Envision custom interfaces"
cd ../ros_ws
colcon build --symlink-install --packages-select envision_interfaces

echo " "
echo "Building ROS2 wrapper for orb slam3"
rm -rf build log install
colcon build --symlink-install --packages-select orbslam3

echo " "
echo "Building ROS2 stereo camera pipeline"
colcon build --symlink-install --packages-select stereo_camera_pipeline

echo " "
echo "Building ROS2 Stereo_obstacle_detector"
colcon build --symlink-install --packages-select stereo_obstacle_detector

echo " "
echo "Building ROS2 Image_masker"
colcon build --symlink-install --packages-select image_masker

echo " "
echo "Building ROS2 Grid Map Cmake helpers"
colcon build --symlink-install --packages-select grid_map_cmake_helpers

echo " "
echo "Building ROS2 Grid Map msgs"
colcon build --symlink-install --packages-select grid_map_msgs

echo " "
echo "Building ROS2 Pointcloud to Grid"
colcon build --symlink-install --packages-select pointcloud_to_grid


