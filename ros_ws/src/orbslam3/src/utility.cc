

#include "utility.hpp"


ORB_SLAM3::System* pSLAM;
ORB_SLAM3::System::eSensor sensor_type = ORB_SLAM3::System::NOT_SET;

std::string world_frame_id, cam_frame_id, imu_frame_id;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tracked_mappoints_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tracked_keypoints_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_mappoints_pub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr kf_markers_pub;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
image_transport::Publisher tracking_img_pub;


bool save_map_srv(const std::shared_ptr<envision_interfaces::srv::SaveMap::Request> req, 
    std::shared_ptr<envision_interfaces::srv::SaveMap::Response> res){

    res->success = pSLAM->SaveMap(req->name);

    if (res->success)
        RCLCPP_INFO(rclcpp::get_logger("save_map_srv"), "Map was saved as %s.osa", req->name.c_str());
    else
        RCLCPP_ERROR(rclcpp::get_logger("save_map_srv"), "Map could not be saved.");
}

bool save_traj_srv(const std::shared_ptr<envision_interfaces::srv::SaveMap::Request> req, 
    std::shared_ptr<envision_interfaces::srv::SaveMap::Response> res){

    const string cam_traj_file = req.name + "_cam_traj.txt";
    const string kf_traj_file = req.name + "_kf_traj.txt";       
    
    try{
        pSLAM->SaveTrajectoryEuRoC(cam_traj_file);
        pSLAM->SaveKeyFrameTrajectoryEuRoC(kf_traj_file);
        res->success = true;
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        res->success = false;
    } catch (...) {
        std::cerr << "Unknows exeption" << std::endl;
        res->success = false;
    }

    if(!res->success)
        RCLCPP_ERROR(rclcpp::get_logger("save_traj_srv"), "Estimated trajectory could not be saved.");
}


void setup_services(std::shared_ptr<rclcpp::Node> node, const std::string &node_name)
{
    // [INFO] Service are created and automatically advertises over the network
    rclcpp::Service<orb_slam3_ros::srv::SaveMap>::SharedPtr save_map_service = node->create_service<orb_slam3_ros::srv::SaveMap>(
        node_name + "/save_map", &save_map_srv);

    rclcpp::Service<orb_slam3_ros::srv::SaveTraj>::SharedPtr save_traj_service = node->create_service<orb_slam3_ros::srv::SaveTraj>(
        node_name + "/save_traj", &save_traj_srv);
}

void setup_publishers(std::shared_ptr<rclcpp::Node> node, image_transport::ImageTransport &image_transport,
    const std::string &node_name){

    pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        node_name + "/camera_pose", 1);

    tracked_mappoints_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        node_name + "/tracked_points", 1);

    tracked_keypoints_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        node_name + "/tracked_key_points", 1);

    all_mappoints_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        node_name + "/all_points", 1);

    tracking_img_pub = image_transport.advertise(
        node_name + "/tracking_image", 1);

    kf_markers_pub = node->create_publisher<visualization_msgs::msg::Marker>(
        node_name + "/kf_markers", 1000);

    if (sensor_type == ORB_SLAM3::System::IMU_MONOCULAR ||
        sensor_type == ORB_SLAM3::System::IMU_STEREO ||
        sensor_type == ORB_SLAM3::System::IMU_RGBD)
    {
        odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(
            node_name + "/body_odom", 1);
    }
}