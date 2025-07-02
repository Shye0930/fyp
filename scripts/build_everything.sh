echo "Configuring and building Thirdparty/DBoW2 ..."
cd ../ORB_SLAM_3_COMMUNITY/Thirdparty/DBoW2
rm -rf build 
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j


echo " "
echo "Configuring and building Thirdparty/g2o ..."
cd ../../g2o
rm -rf build 
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

echo " "
echo "Configuring and building Thirdparty/Sophus ..."
cd ../../Sophus
rm -rf build 
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

echo " "
echo "Uncompress vocabulary ..."
cd ../../../Vocabulary
rm -f ORBvoc.bin
rm -f ORBvoc.txt
tar -xf ORBvoc.txt.tar.gz
cd ..

echo " "
echo "Configuring and building ORB_SLAM3 ..."
rm -rf build 
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j



echo "Converting vocabulary to binary"
rm -f bin_vocabulary
cd ..
./tools/bin_vocabulary

