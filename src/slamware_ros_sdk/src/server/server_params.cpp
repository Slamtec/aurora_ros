/**
 * @file server_params.cpp
 * @brief Implementation of the ServerParams class for managing server parameters in the SLAMWARE ROS SDK.
 */

#include "server_params.h"

#include <cmath>

namespace slamware_ros_sdk
{

    //////////////////////////////////////////////////////////////////////////

    const float C_FLT_PI = ((float)M_PI);
    const float C_FLT_2PI = (C_FLT_PI * 2);

    //////////////////////////////////////////////////////////////////////////

    ServerParams::ServerParams()
    {
        resetToDefault();
    }

    void ServerParams::resetToDefault()
    {
        ip_address = "192.168.11.1";
        robot_port = 1445;
        reconn_wait_ms = (1000U * 3U);

        angle_compensate = true;
        ladar_data_clockwise = true;

        robot_frame = "base_link";
        laser_frame = "laser";
        map_frame = "map";
        imu_frame = "imu_link";
        camera_left = "camera_left";
        camera_right = "camera_right";
        odom_frame = "odom";

        robot_pose_pub_period = 0.05f;
        scan_pub_period = 0.1f;
        map_update_period = 0.2f;
        map_pub_period = 0.2f;

        map_sync_once_get_max_wh = 100.f;
        map_update_near_robot_half_wh = 8.0f;

        imu_raw_data_period = 0.05f;
        system_status_pub_period = 0.05f;
        // stereo_image_pub_period = 0.067f;
        stereo_image_pub_period = 0.05f;
        point_cloud_pub_period = 0.1f;
        robot_basic_state_pub_period = 0.1f;
        odometry_pub_period = 0.1f;
        enhanced_imaging_pub_period = 0.05f;

        scan_topic = "/slamware_ros_sdk_server_node/scan";
        robot_pose_topic = "/slamware_ros_sdk_server_node/robot_pose";
        odom_topic = "/slamware_ros_sdk_server_node/odom";
        map_topic = "/slamware_ros_sdk_server_node/map";
        map_info_topic = "/slamware_ros_sdk_server_node/map_metadata";
        system_status_topic_name = "/slamware_ros_sdk_server_node/system_status";
        relocalization_status_topic_name = "/slamware_ros_sdk_server_node/relocalization_status";
        left_image_raw_topic_name = "/slamware_ros_sdk_server_node/left_image_raw";
        right_image_raw_topic_name = "/slamware_ros_sdk_server_node/right_image_raw";
        point_cloud_topic_name = "/slamware_ros_sdk_server_node/point_cloud";
        stereo_keypoints_topic_name = "/slamware_ros_sdk_server_node/stereo_keypoints";
        imu_raw_data_topic = "/slamware_ros_sdk_server_node/imu_raw_data";

        // Enhanced imaging topics
        depth_image_raw_topic_name = "/slamware_ros_sdk_server_node/depth_image_raw";
        depth_image_colorized_topic_name = "/slamware_ros_sdk_server_node/depth_image_colorized";
        semantic_segmentation_topic_name = "/slamware_ros_sdk_server_node/semantic_segmentation";

        event_period = 1.0f;
    }

    void ServerParams::setBy(const ros::NodeHandle &nhRos)
    {

        nhRos.getParam("event_period", event_period);

        // new ----------------
        nhRos.getParam("ip_address", ip_address);
        nhRos.getParam("reconn_wait_ms", reconn_wait_ms);

        nhRos.getParam("angle_compensate", angle_compensate);
        nhRos.getParam("ladar_data_clockwise", ladar_data_clockwise);

        nhRos.getParam("robot_frame", robot_frame);
        nhRos.getParam("laser_frame", laser_frame);
        nhRos.getParam("map_frame", map_frame);
        nhRos.getParam("imu_frame", imu_frame);
        nhRos.getParam("camera_left", camera_left);
        nhRos.getParam("camera_right", camera_right);
        nhRos.getParam("odom_frame", odom_frame);

        nhRos.getParam("robot_pose_pub_period", robot_pose_pub_period);
        nhRos.getParam("scan_pub_period", scan_pub_period);
        nhRos.getParam("map_update_period", map_update_period);
        nhRos.getParam("map_pub_period", map_pub_period);

        nhRos.getParam("map_sync_once_get_max_wh", map_sync_once_get_max_wh);
        nhRos.getParam("map_update_near_robot_half_wh", map_update_near_robot_half_wh);

        nhRos.getParam("imu_raw_data_period", imu_raw_data_period);
        nhRos.getParam("system_status_pub_period", system_status_pub_period);
        nhRos.getParam("stereo_image_pub_period", stereo_image_pub_period);
        nhRos.getParam("point_cloud_pub_period", point_cloud_pub_period);
        nhRos.getParam("robot_basic_state_pub_period", robot_basic_state_pub_period);
        nhRos.getParam("odometry_pub_period", odometry_pub_period);

        nhRos.getParam("scan_topic", scan_topic);
        nhRos.getParam("robot_pose_topic", robot_pose_topic);
        nhRos.getParam("odom_topic", odom_topic);
        nhRos.getParam("map_topic", map_topic);
        nhRos.getParam("map_info_topic", map_info_topic);
        nhRos.getParam("system_status_topic_name", system_status_topic_name);
        nhRos.getParam("relocalization_status_topic_name", relocalization_status_topic_name);
        nhRos.getParam("left_image_raw_topic_name", left_image_raw_topic_name);
        nhRos.getParam("right_image_raw_topic_name", right_image_raw_topic_name);
        nhRos.getParam("point_cloud_topic_name", point_cloud_topic_name);
        nhRos.getParam("stereo_keypoints_topic_name", stereo_keypoints_topic_name);
        nhRos.getParam("imu_raw_data_topic", imu_raw_data_topic);
        // Enhanced imaging topics
        nhRos.getParam("depth_image_raw_topic_name", depth_image_raw_topic_name);
        nhRos.getParam("depth_image_colorized_topic_name", depth_image_colorized_topic_name);
        nhRos.getParam("semantic_segmentation_topic_name", semantic_segmentation_topic_name);
    }

}
