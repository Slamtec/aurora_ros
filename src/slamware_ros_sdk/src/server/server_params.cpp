
#include "server_params.h"

#include <cmath>

namespace slamware_ros_sdk {

    //////////////////////////////////////////////////////////////////////////

    const float C_FLT_PI = ((float)M_PI);
    const float C_FLT_2PI = (C_FLT_PI * 2);

    //////////////////////////////////////////////////////////////////////////

    ServerParams::ServerParams()
    : Node("server_params")
    {
        resetToDefault();
    }

    void ServerParams::resetToDefault()
    {
        this->declare_parameter<std::string>("ip_address", "192.168.11.1");
        this->declare_parameter<int>("reconn_wait_ms", 1000 * 3);

        this->declare_parameter<bool>("angle_compensate", true);
        this->declare_parameter<bool>("ladar_data_clockwise", true);

        this->declare_parameter<std::string>("robot_frame", "base_link");
        this->declare_parameter<std::string>("laser_frame", "laser");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("imu_frame", "imu_link");
        this->declare_parameter<std::string>("camera_left", "camera_left");
        this->declare_parameter<std::string>("camera_right", "camera_right");

        this->declare_parameter<float>("odometry_pub_period", 0.1f);
        this->declare_parameter<float>("robot_pose_pub_period", 0.05f);
        this->declare_parameter<float>("scan_pub_period", 0.1f);
        this->declare_parameter<float>("map_update_period", 0.3f);
        this->declare_parameter<float>("map_pub_period", 0.3f);

        this->declare_parameter<float>("map_sync_once_get_max_wh", 100.f);
        this->declare_parameter<float>("map_update_near_robot_half_wh", 8.0f);

        this->declare_parameter<float>("imu_raw_data_period", 0.05f);
        this->declare_parameter<float>("system_status_pub_period", 0.1f);
        this->declare_parameter<float>("stereo_image_pub_period", 0.1f);
        this->declare_parameter<float>("point_cloud_pub_period", 0.2f);

        this->declare_parameter<std::string>("scan_topic", "/slamware_ros_sdk_server_node/scan");
        this->declare_parameter<std::string>("robot_pose_topic", "/slamware_ros_sdk_server_node/robot_pose");
        this->declare_parameter<std::string>("odom_topic", "/slamware_ros_sdk_server_node/odom");
        this->declare_parameter<std::string>("map_topic", "/slamware_ros_sdk_server_node/map");
        this->declare_parameter<std::string>("map_info_topic", "/slamware_ros_sdk_server_node/map_metadata");
        this->declare_parameter<std::string>("system_status_topic_name", "/slamware_ros_sdk_server_node/system_status");
        this->declare_parameter<std::string>("relocalization_status_topic_name", "/slamware_ros_sdk_server_node/relocalization_status");
        this->declare_parameter<std::string>("left_image_raw_topic_name", "/slamware_ros_sdk_server_node/left_image_raw");
        this->declare_parameter<std::string>("right_image_raw_topic_name", "/slamware_ros_sdk_server_node/right_image_raw");
        this->declare_parameter<std::string>("point_cloud_topic_name", "/slamware_ros_sdk_server_node/point_cloud");
        this->declare_parameter<std::string>("stereo_keypoints_topic_name", "/slamware_ros_sdk_server_node/stereo_keypoints");

        this->declare_parameter<std::string>("imu_raw_data_topic", "/slamware_ros_sdk_server_node/imu_raw_data");
    }

    void ServerParams::setBy(const std::shared_ptr<rclcpp::Node> nhRos)
    {
        std::string strVal;
        bool bVal;
        int iVal;
        float fVal;
        if (nhRos->has_parameter("ip_address")) {
            nhRos->declare_parameter<std::string>("ip_address", strVal);
        }
        if (nhRos->has_parameter("reconn_wait_ms")) {
            nhRos->declare_parameter<int>("reconn_wait_ms", iVal);
        }
        if (nhRos->has_parameter("angle_compensate")) {
            nhRos->declare_parameter<bool>("angle_compensate", bVal);
        }
        if (nhRos->has_parameter("ladar_data_clockwise")) {
            nhRos->declare_parameter<bool>("ladar_data_clockwise", bVal);
        }

        if (nhRos->has_parameter("robot_frame")) {
            nhRos->declare_parameter<std::string>("robot_frame", strVal);
        }
        if (nhRos->has_parameter("laser_frame")) {
            nhRos->declare_parameter<std::string>("laser_frame", strVal);
        }
        if (nhRos->has_parameter("map_frame")) {
            nhRos->declare_parameter<std::string>("map_frame", strVal);
        }
        if (nhRos->has_parameter("odom_frame"))
        {
            nhRos->declare_parameter<std::string>("odom_frame", strVal);
        }
        if (nhRos->has_parameter("imu_frame")) {
            nhRos->declare_parameter<std::string>("imu_frame", strVal);
        }
        if (nhRos->has_parameter("camera_left")) {
            nhRos->declare_parameter<std::string>("camera_left", strVal);
        }
        if (nhRos->has_parameter("camera_right")) {
            nhRos->declare_parameter<std::string>("camera_right", strVal);
        }
        if (nhRos->has_parameter("odometry_pub_period"))
        {
            nhRos->declare_parameter<float>("odometry_pub_period", fVal);
        }
        if (nhRos->has_parameter("robot_pose_pub_period")) {
            nhRos->declare_parameter<float>("robot_pose_pub_period", fVal);
        }
        if (nhRos->has_parameter("scan_pub_period")) {
            nhRos->declare_parameter<float>("scan_pub_period", fVal);
        }
        if (nhRos->has_parameter("map_update_period")) {
            nhRos->declare_parameter<float>("map_update_period", fVal);
        }
        if (nhRos->has_parameter("map_pub_period")) {
            nhRos->declare_parameter<float>("map_pub_period", fVal);
        }
        if (nhRos->has_parameter("map_sync_once_get_max_wh")) {
            nhRos->declare_parameter<float>("map_sync_once_get_max_wh", fVal);
        }
        if (nhRos->has_parameter("map_update_near_robot_half_wh")) {
            nhRos->declare_parameter<float>("map_update_near_robot_half_wh", fVal);
        }
        if (nhRos->has_parameter("system_status_pub_period")) {
            nhRos->declare_parameter<float>("system_status_pub_period", fVal);
        }
        if (nhRos->has_parameter("stereo_image_pub_period")) {
            nhRos->declare_parameter<float>("stereo_image_pub_period", fVal);
        }
        if (nhRos->has_parameter("point_cloud_pub_period")) {
            nhRos->declare_parameter<float>("point_cloud_pub_period", fVal);
        }

        if (nhRos->has_parameter("scan_topic")) {
            nhRos->declare_parameter<std::string>("scan_topic", strVal);
        }
        if (nhRos->has_parameter("odom_topic"))
        {
            nhRos->declare_parameter<std::string>("odom_topic", strVal);
        }
        if (nhRos->has_parameter("robot_pose_topic")) {
            nhRos->declare_parameter<std::string>("robot_pose_topic", strVal);
        }
        if (nhRos->has_parameter("map_topic")) {
            nhRos->declare_parameter<std::string>("map_topic", strVal);
        }
        if (nhRos->has_parameter("map_info_topic")) {
            nhRos->declare_parameter<std::string>("map_info_topic", strVal);
        }
        if (nhRos->has_parameter("system_status_topic_name")) {
            nhRos->declare_parameter<std::string>("system_status_topic_name", strVal);
        }
        if (nhRos->has_parameter("relocalization_status_topic_name")) {
            nhRos->declare_parameter<std::string>("relocalization_status_topic_name", strVal);
        }
        if (nhRos->has_parameter("left_image_raw_topic_name")) {
            nhRos->declare_parameter<std::string>("left_image_raw_topic_name", strVal);
        }
        if (nhRos->has_parameter("right_image_raw_topic_name")) {
            nhRos->declare_parameter<std::string>("right_image_raw_topic_name", strVal);
        }
        if (nhRos->has_parameter("point_cloud_topic_name")) {
            nhRos->declare_parameter<std::string>("point_cloud_topic_name", strVal);
        }
        if (nhRos->has_parameter("stereo_keypoints_topic_name")) {
            nhRos->declare_parameter<std::string>("stereo_keypoints_topic_name", strVal);
        }
        if (nhRos->has_parameter("right_image_keypoints_topic_name")) {
            nhRos->declare_parameter<std::string>("right_image_keypoints_topic_name", strVal);
        }
        
        if (nhRos->has_parameter("imu_raw_data_topic")) {
            nhRos->declare_parameter<std::string>("imu_raw_data_topic", strVal);
        }
        if (nhRos->has_parameter("imu_raw_data_period")) {
            nhRos->declare_parameter<float>("imu_raw_data_period", fVal);
        }
    }

    //////////////////////////////////////////////////////////////////////////
    
}
