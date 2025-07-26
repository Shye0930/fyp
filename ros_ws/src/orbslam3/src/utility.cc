

#include "utility.hpp"


// ORB_SLAM3::System* pSLAM;
ORB_SLAM3::System::eSensor sensor_type = ORB_SLAM3::System::NOT_SET;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tracked_mappoints_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tracked_keypoints_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_mappoints_pub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr kf_markers_pub;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
image_transport::Publisher tracking_img_pub;


void save_map_srv(ORB_SLAM3::System* pSLAM_instance,
                  const std::shared_ptr<envision_interfaces::srv::SaveMap::Request> req,
                  std::shared_ptr<envision_interfaces::srv::SaveMap::Response> res){
    res->success = pSLAM_instance->SaveMap(req->name);

    if (res->success)
        RCLCPP_INFO(rclcpp::get_logger("save_map_srv"), "Map was saved as %s.osa", req->name.c_str());
    else
        RCLCPP_ERROR(rclcpp::get_logger("save_map_srv"), "Map could not be saved.");
}

void save_traj_srv(ORB_SLAM3::System* pSLAM_instance,
                   const std::shared_ptr<envision_interfaces::srv::SaveMap::Request> req,
                   std::shared_ptr<envision_interfaces::srv::SaveMap::Response> res){
    const std::string cam_traj_file = req->name + "_cam_traj.txt";
    const std::string kf_traj_file = req->name + "_kf_traj.txt";

    try{
        pSLAM_instance->SaveTrajectoryEuRoC(cam_traj_file);
        pSLAM_instance->SaveKeyFrameTrajectoryEuRoC(kf_traj_file);
        res->success = true;
    } catch (const std::exception &e) {
        std::cerr << "save_traj_srv: " << e.what() << std::endl;
        res->success = false;
    } catch (...) {
        std::cerr << "Unknows exeption" << std::endl;
        res->success = false;
    }

    if(!res->success)
        RCLCPP_ERROR(rclcpp::get_logger("save_traj_srv"), "Estimated trajectory could not be saved.");
}

void setup_services(std::shared_ptr<rclcpp::Node> node, const std::string &node_name, ORB_SLAM3::System* pSLAM_instance)
{
    RCLCPP_INFO(node->get_logger(), "[INFO] Setting up services for the node %s", node_name.c_str());

    // Use a lambda function to capture pSLAM_instance
    static rclcpp::Service<envision_interfaces::srv::SaveMap>::SharedPtr save_map_service = node->create_service<envision_interfaces::srv::SaveMap>(
        node_name + "/save_map",
        [pSLAM_instance](const std::shared_ptr<envision_interfaces::srv::SaveMap::Request> request,
                         std::shared_ptr<envision_interfaces::srv::SaveMap::Response> response) {
            save_map_srv(pSLAM_instance, request, response);
        });

    static rclcpp::Service<envision_interfaces::srv::SaveMap>::SharedPtr save_traj_service = node->create_service<envision_interfaces::srv::SaveMap>(
        node_name + "/save_traj",
        [pSLAM_instance](const std::shared_ptr<envision_interfaces::srv::SaveMap::Request> request,
                         std::shared_ptr<envision_interfaces::srv::SaveMap::Response> response) {
            save_traj_srv(pSLAM_instance, request, response);
        });
}


void setup_publishers(std::shared_ptr<rclcpp::Node> node, image_transport::ImageTransport &image_transport,
    const std::string &node_name){

    RCLCPP_INFO(node->get_logger(), "[INFO] Setting up publisher for the node %s", node_name.c_str());

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

void publish_topics(ORB_SLAM3::System* pSLAM_instance,const rclcpp::Time &msg_time, const std::string world_frame_id, const std::string cam_frame_id, const std::string imu_frame_id,const Eigen::Vector3f &Wbb)
{

    RCLCPP_INFO(rclcpp::get_logger("orbslam3"), "World frame ID: %s, Camera frame ID: %s, IMU frame ID: %s", 
                world_frame_id.c_str(), cam_frame_id.c_str(), imu_frame_id.c_str());

    // *** Crucial Check Here ***
    if (pSLAM_instance == nullptr) {
        // Log an error or warning, as this indicates a severe problem
        RCLCPP_ERROR(rclcpp::get_logger("orbslam3"), "Error: ORB_SLAM3::System pointer (pSLAM) is null. Cannot retrieve camera pose.");
        return; // Exit the function to prevent dereferencing a null pointer
    } else {
        static bool x = false;
        RCLCPP_INFO_ONCE(rclcpp::get_logger("orbslam3"), "not null");
    }
    Sophus::SE3f Twc = pSLAM_instance->GetCamTwc();

    if (Twc.translation().array().isNaN()[0] || Twc.rotationMatrix().array().isNaN()(0,0)) // avoid publishing NaN
        return;

    

    // Common topics
    publish_camera_pose(pSLAM_instance,world_frame_id,Twc, msg_time);
    publish_tf_transform(pSLAM_instance,Twc, world_frame_id, cam_frame_id, msg_time);

    publish_tracking_img(pSLAM_instance,world_frame_id,pSLAM_instance->GetCurrentFrame(), msg_time);

    publish_keypoints(pSLAM_instance,world_frame_id,pSLAM_instance->GetTrackedMapPoints(), pSLAM_instance->GetTrackedKeyPoints(), msg_time);

    publish_tracked_points(pSLAM_instance,world_frame_id,pSLAM_instance->GetTrackedMapPoints(), msg_time);

    publish_all_points(pSLAM_instance,world_frame_id,pSLAM_instance->GetAllMapPoints(), msg_time);
    publish_kf_markers(pSLAM_instance,world_frame_id,pSLAM_instance->GetAllKeyframePoses(), msg_time);

    // IMU-specific topics
    if (sensor_type == ORB_SLAM3::System::IMU_MONOCULAR ||
         sensor_type == ORB_SLAM3::System::IMU_STEREO ||
         sensor_type == ORB_SLAM3::System::IMU_RGBD)
     {
         Sophus::SE3f Twb = pSLAM_instance->GetImuTwb();
         Eigen::Vector3f Vwb = pSLAM_instance->GetImuVwb();

         Sophus::Matrix3f Rwb = Twb.rotationMatrix();
         Eigen::Vector3f Wwb = Rwb * Wbb;

         publish_tf_transform(pSLAM_instance,Twb, world_frame_id, imu_frame_id, msg_time);
         publish_body_odom(pSLAM_instance, world_frame_id, imu_frame_id,Twb, Vwb, Wwb, msg_time);
     }
}



void publish_body_odom(ORB_SLAM3::System* pSLAM_instance,
                       const std::string world_frame_id, 
                       const std::string imu_frame_id,
                       const Sophus::SE3f &Twb_SE3f,
                       const Eigen::Vector3f &Vwb_E3f,
                       const Eigen::Vector3f &ang_vel_body,
                       const rclcpp::Time &msg_time)
{
    nav_msgs::msg::Odometry odom_msg;

    odom_msg.header.stamp = msg_time;
    odom_msg.header.frame_id = world_frame_id;
    odom_msg.child_frame_id = imu_frame_id;

    // Position
    odom_msg.pose.pose.position.x = Twb_SE3f.translation().x();
    odom_msg.pose.pose.position.y = Twb_SE3f.translation().y();
    odom_msg.pose.pose.position.z = Twb_SE3f.translation().z();

    // Orientation (Note: .coeffs() is [x, y, z, w] in Eigen)
    Eigen::Quaternionf q = Twb_SE3f.unit_quaternion();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // Linear velocity
    odom_msg.twist.twist.linear.x = Vwb_E3f.x();
    odom_msg.twist.twist.linear.y = Vwb_E3f.y();
    odom_msg.twist.twist.linear.z = Vwb_E3f.z();

    // Angular velocity
    odom_msg.twist.twist.angular.x = ang_vel_body.x();
    odom_msg.twist.twist.angular.y = ang_vel_body.y();
    odom_msg.twist.twist.angular.z = ang_vel_body.z();

    if(odom_pub)
        odom_pub->publish(odom_msg);  // ROS 2 publishers use `->publish(...)`
}

void publish_camera_pose(ORB_SLAM3::System* pSLAM_instance,const std::string world_frame_id,const Sophus::SE3f &Tcw_SE3f, const rclcpp::Time &msg_time)
{
    geometry_msgs::msg::PoseStamped pose_msg;

    pose_msg.header.stamp = msg_time;
    pose_msg.header.frame_id = world_frame_id;

    // Position
    pose_msg.pose.position.x = Tcw_SE3f.translation().x();
    pose_msg.pose.position.y = Tcw_SE3f.translation().y();
    pose_msg.pose.position.z = Tcw_SE3f.translation().z();

    // Orientation
    Eigen::Quaternionf q = Tcw_SE3f.unit_quaternion();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    if(pose_pub)
        pose_pub->publish(pose_msg);  // ROS 2: use `->publish`
}

void publish_tf_transform(ORB_SLAM3::System* pSLAM_instance,Sophus::SE3f T_SE3f, std::string frame_id, std::string child_frame_id, rclcpp::Time msg_time)
{
    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = msg_time;
    tf_msg.header.frame_id = frame_id;
    tf_msg.child_frame_id = child_frame_id;

    tf_msg.transform = SE3f_to_transform_msg(T_SE3f);

    static std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    if (!tf_broadcaster) {
        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(rclcpp::Node::make_shared("tf_static_broadcaster"));
    }

    tf_broadcaster->sendTransform(tf_msg);
}

void publish_tracking_img(ORB_SLAM3::System* pSLAM_instance,const std::string world_frame_id,cv::Mat image, rclcpp::Time msg_time)
{
    std_msgs::msg::Header header;

    header.stamp = msg_time;
    header.frame_id = world_frame_id;

    auto rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

    
    tracking_img_pub.publish(*rendered_image_msg);
}


void publish_keypoints(ORB_SLAM3::System* pSLAM_instance,const std::string world_frame_id,std::vector<ORB_SLAM3::MapPoint*> tracked_map_points, std::vector<cv::KeyPoint> tracked_keypoints, rclcpp::Time msg_time)
{   
    std::vector<cv::KeyPoint> finalKeypoints;

    if (tracked_keypoints.empty())
        return;

    for (size_t i = 0; i < tracked_map_points.size() && i < tracked_keypoints.size(); i++) {
        if (tracked_map_points[i]) {  // if the MapPoint pointer is not nullptr
            finalKeypoints.push_back(tracked_keypoints[i]);
        }
    }

    // Create a blank image. Adjust dimensions as per your requirement.
    //int width = 640;  // Assuming a standard 640x480 size. Change as needed.
    //int height = 480;
    //cv::Mat blankImg = cv::Mat::zeros(height, width, CV_8UC3);  // Black image

    // Draw keypoints on the blank image.
    //cv::drawKeypoints(blankImg, finalKeypoints, blankImg, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DEFAULT);

    // Display the image (optional)
    //cv::imshow("Keypoints", blankImg);
    //cv::waitKey(1); 

    sensor_msgs::msg::PointCloud2 cloud = keypoints_to_pointcloud(finalKeypoints,world_frame_id, msg_time);

    if(tracked_keypoints_pub)
        tracked_keypoints_pub->publish(cloud);
}

void publish_tracked_points(ORB_SLAM3::System* pSLAM_instance,const std::string world_frame_id,std::vector<ORB_SLAM3::MapPoint*> tracked_points, rclcpp::Time msg_time)
{
    sensor_msgs::msg::PointCloud2 cloud = mappoint_to_pointcloud(tracked_points,world_frame_id, msg_time);
    
    if(tracked_mappoints_pub)
        tracked_mappoints_pub->publish(cloud);
}

void publish_all_points(ORB_SLAM3::System* pSLAM_instance,const std::string world_frame_id,std::vector<ORB_SLAM3::MapPoint*> map_points, rclcpp::Time msg_time)
{
    sensor_msgs::msg::PointCloud2 cloud = mappoint_to_pointcloud(map_points,world_frame_id, msg_time);
    
    if(all_mappoints_pub)
        all_mappoints_pub->publish(cloud);
}

// More details: http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html
void publish_kf_markers(ORB_SLAM3::System* pSLAM_instance, const std::string world_frame_id, std::vector<Sophus::SE3f> vKFposes, rclcpp::Time msg_time)
{
    int numKFs = vKFposes.size();
    if (numKFs == 0)
        return;
    
    visualization_msgs::msg::Marker kf_markers;
    kf_markers.header.frame_id = world_frame_id;
    kf_markers.header.stamp = msg_time;
    kf_markers.ns = "kf_markers";
    kf_markers.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    kf_markers.action = visualization_msgs::msg::Marker::ADD;
    kf_markers.pose.orientation.w = 1.0;
    kf_markers.lifetime = rclcpp::Duration(0, 0); // Zero duration for persistent markers

    kf_markers.id = 0;
    kf_markers.scale.x = 0.05;
    kf_markers.scale.y = 0.05;
    kf_markers.scale.z = 0.05;
    kf_markers.color.g = 1.0;
    kf_markers.color.a = 1.0;

    for (int i = 0; i <= numKFs; i++) 
    {
        geometry_msgs::msg::Point kf_marker;
        kf_marker.x = vKFposes[i].translation().x();
        kf_marker.y = vKFposes[i].translation().y();
        kf_marker.z = vKFposes[i].translation().z();
        kf_markers.points.push_back(kf_marker);
    }
    
    if(kf_markers_pub)
        kf_markers_pub->publish(kf_markers);
}

sensor_msgs::msg::PointCloud2 keypoints_to_pointcloud(const std::vector<cv::KeyPoint>& keypoints,const std::string world_frame_id, rclcpp::Time msg_time)
{
    const int num_channels = 3; // x, y, z

    sensor_msgs::msg::PointCloud2 cloud;

    cloud.header.stamp = msg_time;
    cloud.header.frame_id = world_frame_id;
    cloud.height = 1;
    cloud.width = keypoints.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = { "x", "y", "z" };

    for (int i = 0; i < num_channels; i++) {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    for (unsigned int i = 0; i < cloud.width; i++) {
        float data_array[num_channels] = {
            keypoints[i].pt.x,
            keypoints[i].pt.y,
            0.0f // Z value is 0 for 2D keypoints
        };

        memcpy(cloud_data_ptr + (i * cloud.point_step), data_array, num_channels * sizeof(float));
    }
    return cloud;
}

sensor_msgs::msg::PointCloud2 mappoint_to_pointcloud(const std::vector<ORB_SLAM3::MapPoint*>& map_points,const std::string world_frame_id, rclcpp::Time msg_time)
{
    const int num_channels = 3; // x, y, z

    if (map_points.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("orbslam3"), "Map point vector is empty!");
        return sensor_msgs::msg::PointCloud2();
    }

    sensor_msgs::msg::PointCloud2 cloud;

    cloud.header.stamp = msg_time;
    cloud.header.frame_id = world_frame_id;
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = { "x", "y", "z" };

    for (int i = 0; i < num_channels; i++) {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    for (unsigned int i = 0; i < cloud.width; i++) {
        if (map_points[i]) {
            Eigen::Vector3d P3Dw = map_points[i]->GetWorldPos().cast<double>();

            float data_array[num_channels] = {
                static_cast<float>(P3Dw.x()),
                static_cast<float>(P3Dw.y()),
                static_cast<float>(P3Dw.z())
            };

            memcpy(cloud_data_ptr + (i * cloud.point_step), data_array, num_channels * sizeof(float));
        }
    }
    return cloud;
}


cv::Mat SE3f_to_cvMat(Sophus::SE3f T_SE3f)
{
    cv::Mat T_cvmat;

    Eigen::Matrix4f T_Eig3f = T_SE3f.matrix();
    cv::eigen2cv(T_Eig3f, T_cvmat);
    
    return T_cvmat;
}

geometry_msgs::msg::Transform SE3f_to_transform_msg(const Sophus::SE3f &T_SE3f)
{
    Eigen::Vector3f t_vec = T_SE3f.translation();
    Eigen::Quaternionf q = T_SE3f.unit_quaternion();

    geometry_msgs::msg::Transform transform_msg;

    transform_msg.translation.x = t_vec.x();
    transform_msg.translation.y = t_vec.y();
    transform_msg.translation.z = t_vec.z();

    transform_msg.rotation.x = q.x();
    transform_msg.rotation.y = q.y();
    transform_msg.rotation.z = q.z();
    transform_msg.rotation.w = q.w();

    return transform_msg;
}
