#include "stereo-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    stringstream ss(strDoRectify);
    ss >> boolalpha >> doRectify;

    std::cout << "[INFO] doRectify is " << (doRectify ? "true" : "false") << std::endl;
    RCLCPP_INFO(this->get_logger(), "[INFO] doRectify is %s", doRectify ? "true" : "false");
    
    if (doRectify){

        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if(!fsSettings.isOpened()){
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() || rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0){
            std::stringstream error_msg;
            error_msg << "[ERROR] Calibration parameters to rectify stereo are missing!";
            
            cerr << "[ERROR] Calibration parameters to rectify stereo are missing!" << endl;

            if (K_l.empty()) cerr << "  K_l is empty." << endl;
            if (K_r.empty()) cerr << "  K_r is empty." << endl;
            if (P_l.empty()) cerr << "  P_l is empty." << endl;
            if (P_r.empty()) cerr << "  P_r is empty." << endl;
            if (R_l.empty()) cerr << "  R_l is empty." << endl;
            if (R_r.empty()) cerr << "  R_r is empty." << endl;
            if (D_l.empty()) cerr << "  D_l is empty." << endl;
            if (D_r.empty()) cerr << "  D_r is empty." << endl;

            if (rows_l == 0) cerr << "  rows_l is zero." << endl;
            if (rows_r == 0) cerr << "  rows_r is zero." << endl;
            if (cols_l == 0) cerr << "  cols_l is zero." << endl;
            if (cols_r == 0) cerr << "  cols_r is zero." << endl;

            
            RCLCPP_ERROR(this->get_logger(), "%s", error_msg.str().c_str());
            throw std::runtime_error(error_msg.str());
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
    }

    
}

void StereoSlamNode::initialize()
{
    // Now it's safe to call shared_from_this() because the node is fully constructed
    // and should be managed by a shared_ptr in the calling main function.
    m_image_transport = std::make_unique<image_transport::ImageTransport>(this->shared_from_this());

    std::string node_name = this->get_name();
    world_frame_id = "map";
    cam_frame_id = "camera";

    sensor_type = ORB_SLAM3::System::STEREO;

    // Setup publishers and services
    setup_publishers(this->shared_from_this(), *m_image_transport, node_name);
    setup_services(this->shared_from_this(), node_name, m_SLAM);

    // [HACK] Change "/camera/left/image_raw" to "/stereo/left/rectified_images"
    left_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this->shared_from_this(), "/stereo/left/rectified_images");
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this->shared_from_this(), "/stereo/right/rectified_images");

    // pub_rectified_left = this->create_publisher<sensor_msgs::msg::Image>("/stereo/left/rectified_images", 10);
    // pub_rectified_right = this->create_publisher<sensor_msgs::msg::Image>("/stereo/right/rectified_images", 10);

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);
}

StereoSlamNode::~StereoSlamNode()
{
    if(m_SLAM){
        // Stop all threads
        m_SLAM->Shutdown();

        // Save camera trajectory
        m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
        m_SLAM->SaveTrajectoryTUM("Trajectory.txt");
    }
    
}

void StereoSlamNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight)
{
    // Copy the ros rgb image message to cv::Mat.
    builtin_interfaces::msg::Time msg_time = msgLeft->header.stamp;

    try {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
        cv_ptrRight = cv_bridge::toCvShare(msgRight);

    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    if (doRectify){
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        m_SLAM->TrackStereo(imLeft, imRight, Utility::StampToSec(msgLeft->header.stamp));
        publish_topics(m_SLAM, msg_time);

    } else {
        m_SLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, Utility::StampToSec(msgLeft->header.stamp));

        if (m_SLAM->GetTrackingState() == ORB_SLAM3::Tracking::OK) {
            try {
                publish_topics(m_SLAM, msg_time);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception during publish_topics: %s", e.what());
            }
        }
    }

}
