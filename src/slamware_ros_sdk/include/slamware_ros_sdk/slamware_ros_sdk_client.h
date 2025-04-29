/**
 * @file slamware_ros_sdk_client.h
 * @brief Header file for the Slamware ROS SDK Client class.
 */

#pragma once

#include <ros/ros.h>
#include <tf/message_filter.h>
#include <slamware_ros_sdk/Vec2DInt32.h>
#include <slamware_ros_sdk/Line2DFlt32Array.h>
#include <slamware_ros_sdk/RectInt32.h>
#include <slamware_ros_sdk/RobotDeviceInfo.h>
#include <slamware_ros_sdk/BasicSensorInfoArray.h>
#include <slamware_ros_sdk/BasicSensorValueDataArray.h>
#include <slamware_ros_sdk/RobotBasicState.h>
#include <slamware_ros_sdk/SyncMapRequest.h>
#include <slamware_ros_sdk/MoveByDirectionRequest.h>
#include <slamware_ros_sdk/MoveByThetaRequest.h>
#include <slamware_ros_sdk/MoveToRequest.h>
#include <slamware_ros_sdk/MoveToLocationsRequest.h>
#include <slamware_ros_sdk/RotateToRequest.h>
#include <slamware_ros_sdk/RotateRequest.h>
#include <slamware_ros_sdk/RecoverLocalizationRequest.h>
#include <slamware_ros_sdk/ClearMapRequest.h>
#include <slamware_ros_sdk/SetMapUpdateRequest.h>
#include <slamware_ros_sdk/SetMapLocalizationRequest.h>
#include <slamware_ros_sdk/GoHomeRequest.h>
#include <slamware_ros_sdk/CancelActionRequest.h>
#include <slamware_ros_sdk/AddLineRequest.h>
#include <slamware_ros_sdk/AddLinesRequest.h>
#include <slamware_ros_sdk/RemoveLineRequest.h>
#include <slamware_ros_sdk/ClearLinesRequest.h>
#include <slamware_ros_sdk/MoveLineRequest.h>
#include <slamware_ros_sdk/MoveLinesRequest.h>
#include <slamware_ros_sdk/SyncGetStcm.h>
#include <slamware_ros_sdk/SyncSetStcm.h>

namespace slamware_ros_sdk
{

    class SlamwareRosSdkClient
    {
    public:
        typedef std::string fs_path_t;

    public:
        explicit SlamwareRosSdkClient(ros::NodeHandle &nhRos, const char *serverNodeName = nullptr, const char *msgNamePrefix = nullptr);
        ~SlamwareRosSdkClient();

        void syncMap(const SyncMapRequest &msg) { return pubSyncMap_.publish(msg); }
        void clearMap(const ClearMapRequest &msg) { pubClearMap_.publish(msg); }
        void setMapUpdate(const SetMapUpdateRequest &msg) { pubSetMapUpdate_.publish(msg); }
        void setMapLocalization(const SetMapLocalizationRequest &msg) { pubSetMapLocalization_.publish(msg); }

        // get stcm and write to filePath.
        bool syncGetStcm(std::string &errMsg, const std::string &filePath);
        // load stcm from filePath, and upload to slamware.
        bool syncSetStcm(const std::string &mapfile, std::string &errMsg);

    private:
        std::string genTopicFullName_(const std::string &strName) const { return msgNamePrefix_ + strName; }

    private:
        ros::NodeHandle *nh_;
        std::string sdkServerNodeName_;
        std::string msgNamePrefix_;

        ros::Publisher pubSyncMap_;
        ros::Publisher pubClearMap_;
        ros::Publisher pubSetMapUpdate_;
        ros::Publisher pubSetMapLocalization_;
        ros::ServiceClient scSyncGetStcm_;
        ros::ServiceClient scSyncSetStcm_;
    };

}
