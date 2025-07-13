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


    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    bool visualization = true;
    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, visualization);

    auto node = std::make_shared<StereoSlamNode>(&pSLAM, argv[2], "false"); // change argv[3] to false - which is to recitify 

    std::string node_name = "Stereo_node";

    world_frame_id = "map";
    cam_frame_id = "camera";

    image_transport::ImageTransport it(node);

    // Setup publishers and services
    setup_publishers(node, it, node_name);
    setup_services(node, node_name);

    RCLCPP_INFO(node->get_logger(), "Stereo SLAM node initialized and the node name is %s", node_name.c_str());

    std::cout << "============================ " << std::endl;

    // Spin the node
    rclcpp::spin(node);

    // Shutdown
    rclcpp::shutdown();

    return 0;
}
