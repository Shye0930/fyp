#include "rgbd-slam-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("orb_slam3"),
    m_SLAM(pSLAM)
{

}

RgbdSlamNode::~RgbdSlamNode()
{
   if(m_SLAM){
        // Stop all threads
        m_SLAM->Shutdown();

        // Save camera trajectory
        m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
        m_SLAM->SaveTrajectoryTUM("Trajectory.txt");
    }
}

void RgbdSlamNode::initialize(){


    this->declare_parameter<std::string>("rgb_topic", "/camera/rgb");
    this->declare_parameter<std::string>("depth_topic", "/camera/depth");

    // 2. Get the values of the parameters.
    std::string rgb_topic;
    std::string depth_topic;
    this->get_parameter("rgb_topic", rgb_topic);
    this->get_parameter("depth_topic", depth_topic);

    m_image_transport = std::make_unique<image_transport::ImageTransport>(this->shared_from_this());
    
    std::string node_name = this->get_name();
    
    // Setup publishers and services
    setup_publishers(this->shared_from_this(), *m_image_transport, node_name);
    setup_services(this->shared_from_this(), node_name, m_SLAM);

    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), rgb_topic);
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), depth_topic);

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);

}

void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
    // Copy the ros rgb image message to cv::Mat.

    builtin_interfaces::msg::Time msg_time = msgRGB->header.stamp;


    world_frame_id = "map";
    cam_frame_id = "camera";
    imu_frame_id = "imu";

    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp));

    if (m_SLAM->GetTrackingState() == ORB_SLAM3::Tracking::OK) {
        try {
            publish_topics(m_SLAM,msg_time, world_frame_id, cam_frame_id, imu_frame_id);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception during publish_topics: %s", e.what());
        }
    }
}
