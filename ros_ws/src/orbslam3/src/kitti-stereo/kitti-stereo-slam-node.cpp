#include "kitti-stereo-slam-node.hpp"

#include <opencv2/core/core.hpp>
#include <fstream>
#include <chrono>

using std::placeholders::_1;
using std::placeholders::_2;

// Modified constructor to accept path to sequence
StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify, const std::string& strPathToSequence)
:   Node("orb_slam3"),
    m_SLAM(pSLAM),
    m_strPathToSequence(strPathToSequence),
    m_currentImageIdx(0)
{

    // Load KITTI images and timestamps
    if (!m_strPathToSequence.empty()) {
        LoadImages(m_strPathToSequence, m_vstrImageLeft, m_vstrImageRight, m_vTimestamps);
        m_nImages = m_vstrImageLeft.size();
        RCLCPP_INFO(this->get_logger(), "Loaded %d images from KITTI sequence: %s", m_nImages, m_strPathToSequence.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "No KITTI sequence path provided. Node will attempt to subscribe to topics.");
    }
}

void StereoSlamNode::initialize()
{
    // Now it's safe to call shared_from_this() because the node is fully constructed
    // and should be managed by a shared_ptr in the calling main function.
    m_image_transport = std::make_unique<image_transport::ImageTransport>(this->shared_from_this());

    std::string node_name = this->get_name();

    sensor_type = ORB_SLAM3::System::STEREO;

    // Setup publishers and services
    setup_publishers(this->shared_from_this(), *m_image_transport, node_name);
    setup_services(this->shared_from_this(), node_name, m_SLAM);

    // If a KITTI sequence path is provided, we don't need subscribers
    if (m_strPathToSequence.empty()) {
        // [HACK] Change "/camera/left/image_raw" to "/stereo/left/rectified_images"
        left_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this->shared_from_this(), "/stereo/left/rectified_images");
        right_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this->shared_from_this(), "/stereo/right/rectified_images");

        syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *left_sub, *right_sub);
        syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);
        RCLCPP_INFO(this->get_logger(), "Subscribing to image topics for stereo input.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Processing KITTI sequence directly. Not subscribing to image topics.");
    }
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
    // This method is now only called if we are subscribing to topics (i.e., m_strPathToSequence is empty)
    builtin_interfaces::msg::Time msg_time = msgLeft->header.stamp;

    world_frame_id = "map";
    cam_frame_id = "camera";
    imu_frame_id = "imu";

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
        publish_topics(m_SLAM,msg_time, world_frame_id, cam_frame_id, imu_frame_id );

    } else {
        m_SLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, Utility::StampToSec(msgLeft->header.stamp));

        if (m_SLAM->GetTrackingState() == ORB_SLAM3::Tracking::OK) {
            try {
                publish_topics(m_SLAM,msg_time, world_frame_id, cam_frame_id, imu_frame_id);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception during publish_topics: %s", e.what());
            }
        }
    }
}

void StereoSlamNode::ProcessKITTIStereo()
{
    if (m_strPathToSequence.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No KITTI sequence path provided to process. Call `spin()` instead if using subscriptions.");
        return;
    }

    world_frame_id = "world";
    cam_frame_id = "camera";
    imu_frame_id = "imu";

    float imageScale = m_SLAM->GetImageScale();

    // Main loop
    cv::Mat imLeft, imRight;
    for(m_currentImageIdx = 0; m_currentImageIdx < m_nImages; ++m_currentImageIdx)
    {
        // Read left and right images from file
        imLeft = cv::imread(m_vstrImageLeft[m_currentImageIdx], cv::IMREAD_UNCHANGED);
        imRight = cv::imread(m_vstrImageRight[m_currentImageIdx], cv::IMREAD_UNCHANGED);
        double tframe = m_vTimestamps[m_currentImageIdx];

        if(imLeft.empty() || imRight.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load image at index %d: %s or %s",
                         m_currentImageIdx, m_vstrImageLeft[m_currentImageIdx].c_str(), m_vstrImageRight[m_currentImageIdx].c_str());
            continue; // Skip to the next image
        }

        if(imageScale != 1.f)
        {
            int width = imLeft.cols * imageScale;
            int height = imLeft.rows * imageScale;
            cv::resize(imLeft, imLeft, cv::Size(width, height));
            cv::resize(imRight, imRight, cv::Size(width, height));
        }

        // Create a dummy header stamp for publishing
        builtin_interfaces::msg::Time msg_time;
        msg_time.sec = static_cast<int32_t>(tframe);
        msg_time.nanosec = static_cast<uint32_t>((tframe - msg_time.sec) * 1e9);

        m_SLAM->TrackStereo(imLeft, imRight, tframe);

        if (m_SLAM->GetTrackingState() == ORB_SLAM3::Tracking::OK) {
            try {
                publish_topics(m_SLAM, msg_time, world_frame_id, cam_frame_id, imu_frame_id);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception during publish_topics: %s", e.what());
            }
        }
        // This is a blocking loop. If you want to allow other ROS 2 callbacks,
        // you might need to use a timer or a different execution model.
        // For now, it will process the sequence as fast as possible.
        // To allow some ROS processing in between, you can add:
        rclcpp::spin_some(this->shared_from_this());

        // Basic delay to simulate real-time, if needed (KITTI uses timestamps for this)
        if (m_currentImageIdx < m_nImages - 1) {
            double T = m_vTimestamps[m_currentImageIdx+1] - tframe;
            // Sleep for (T - processing_time) if processing_time < T
            // For now, we are just processing as fast as possible, ORB-SLAM3 usually handles the timing.
        }
    }
    RCLCPP_INFO(this->get_logger(), "Finished processing KITTI sequence.");
}

void StereoSlamNode::LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    if (!fTimes.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open times.txt at %s", strPathTimeFile.c_str());
        throw std::runtime_error("Could not open times.txt");
    }

    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}