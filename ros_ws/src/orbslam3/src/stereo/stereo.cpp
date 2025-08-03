#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include "rclcpp/rclcpp.hpp"
#include "stereo-slam-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    // if(argc < 4)
    // {
    //     std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings do_rectify" << std::endl;
    //     return 1;
    // }

    // [HACK] Cut away the need for do_rectify as the retification will be done by other node 
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    sensor_type = ORB_SLAM3::System::STEREO;


    bool visualization = true;
    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, visualization);

    auto node = std::make_shared<StereoSlamNode>(&pSLAM, argv[2], "false"); // change argv[3] to false - which is to recitify 

    node->initialize();

    RCLCPP_INFO(node->get_logger(), "Stereo SLAM node initialized and the node name is %s", node->get_name());


    std::cout << "============================ " << std::endl;

    // Spin the node
    rclcpp::spin(node);

    // Shutdown
    rclcpp::shutdown();

    return 0;
}
