#ifndef __STEREO_SLAM_NODE_HPP__
#define __STEREO_SLAM_NODE_HPP__

#include "utility.hpp"
#include <iomanip> // For std::setw and std::setfill

class StereoSlamNode : public rclcpp::Node
{
public:
    StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify, const std::string& strPathToSequence);

    ~StereoSlamNode();

    void initialize();

    // New method to process KITTI sequence
    void ProcessKITTIStereo();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;

    void GrabStereo(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD);

    ORB_SLAM3::System* m_SLAM;

    bool doRectify;
    cv::Mat M1l,M2l,M1r,M2r;

    cv_bridge::CvImageConstPtr cv_ptrLeft;
    cv_bridge::CvImageConstPtr cv_ptrRight;

    std::string world_frame_id;
    std::string cam_frame_id;
    std::string imu_frame_id;

    // These will no longer be used if we load from files, but keep them if you ever switch back to topic subscriptions
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > left_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > right_sub;
    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;

    std::unique_ptr<image_transport::ImageTransport> m_image_transport;

    // --- New members for KITTI sequence handling ---
    std::string m_strPathToSequence;
    std::vector<std::string> m_vstrImageLeft;
    std::vector<std::string> m_vstrImageRight;
    std::vector<double> m_vTimestamps;
    int m_nImages;
    int m_currentImageIdx;

    void LoadImages(const std::string &strPathToSequence, std::vector<std::string> &vstrImageLeft,
                    std::vector<std::string> &vstrImageRight, std::vector<double> &vTimestamps);
};

#endif