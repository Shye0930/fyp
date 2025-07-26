# TODO: Only to rebuild slam and ros wrapper
# This file is meant to be run only after you have build everything


#echo " "
#echo "Configuring and building ORB_SLAM3 ..."
#cd ../ORB_SLAM_3_COMMUNITY/
#rm -rf build 
#mkdir build
#cd build
#cmake .. -DCMAKE_BUILD_TYPE=Release
#make -j$(nproc)

echo " "
echo "Building ROS2 Envision custom interfaces"
cd ../ros_ws/
rm -rf build log install
colcon build --symlink-install --packages-select envision_interfaces

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
echo "Configuring and building ROS2 Wrapper for ORB_SLAM3 ..."
colcon build --symlink-install --packages-select orbslam3 --cmake-clean-cache


