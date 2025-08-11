#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include "rclcpp/rclcpp.hpp"
#include "kitti-stereo-slam-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    // Usage: ros2 run orbslam_ros stereo path_to_vocabulary path_to_settings path_to_kitti_sequence
    if(argc < 4)
    {
        std::cerr << "\nUsage: ros2 run orbslam_ros kitti-stereo path_to_vocabulary path_to_settings path_to_kitti_sequence" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    bool visualization = true;
    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, visualization);

    // Pass the KITTI sequence path to the StereoSlamNode constructor
    auto node = std::make_shared<StereoSlamNode>(&pSLAM, argv[2], "false", argv[3]); // "false" for doRectify, argv[3] is KITTI path

    node->initialize(); // This will setup publishers/services and log that it's using KITTI

    RCLCPP_INFO(node->get_logger(), "Stereo SLAM node initialized and the node name is %s", node->get_name());

    std::cout << "============================ " << std::endl;

    // Call the new method to process the KITTI sequence
    node->ProcessKITTIStereo();


    // Shutdown
    rclcpp::shutdown();

    return 0;
}