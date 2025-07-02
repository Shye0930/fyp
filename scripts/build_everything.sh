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

echo "Converting vocabulary to binary"
rm -f bin_vocabulary
cd ..
./tools/bin_vocabulary

#TODO: Include building of ros2 wrapper too.