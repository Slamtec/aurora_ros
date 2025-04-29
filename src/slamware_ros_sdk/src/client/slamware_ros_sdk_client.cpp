/**
 * @file slamware_ros_sdk_client.cpp
 * @brief Implementation of the SlamwareRosSdkClient class for interacting with the Slamware ROS SDK.
 */

#include <slamware_ros_sdk/slamware_ros_sdk_client.h>

namespace slamware_ros_sdk
{

    SlamwareRosSdkClient::SlamwareRosSdkClient(ros::NodeHandle &nhRos, const char *serverNodeName, const char *msgNamePrefix)
        : nh_(&nhRos)
    {
        if (nullptr != serverNodeName)
            sdkServerNodeName_ = serverNodeName;
        else
            sdkServerNodeName_ = "slamware_ros_sdk_server_node";

        if (nullptr != msgNamePrefix)
        {
            msgNamePrefix_ = msgNamePrefix;
        }
        else if (!sdkServerNodeName_.empty())
        {
            if ('/' != sdkServerNodeName_.front())
                msgNamePrefix_ = "/";
            msgNamePrefix_ += sdkServerNodeName_;
            if ('/' != msgNamePrefix_.back())
                msgNamePrefix_ += "/";
        }

        // initialize publishers
        {
            pubSyncMap_ = nh_->advertise<SyncMapRequest>(genTopicFullName_("sync_map"), 1);

            pubClearMap_ = nh_->advertise<ClearMapRequest>(genTopicFullName_("clear_map"), 1);
            pubSetMapUpdate_ = nh_->advertise<SetMapUpdateRequest>(genTopicFullName_("set_map_update"), 1);
            pubSetMapLocalization_ = nh_->advertise<SetMapLocalizationRequest>(genTopicFullName_("set_map_localization"), 1);
        }

        // initialize service clients
        {
            scSyncGetStcm_ = nh_->serviceClient<SyncGetStcm>(genTopicFullName_("sync_get_stcm"));
            scSyncSetStcm_ = nh_->serviceClient<SyncSetStcm>(genTopicFullName_("sync_set_stcm"));
        }
    }

    SlamwareRosSdkClient::~SlamwareRosSdkClient()
    {
        //
    }

    bool SlamwareRosSdkClient::syncGetStcm(std::string &errMsg, const std::string &filePath)
    {

        errMsg.clear();

        slamware_ros_sdk::SyncGetStcm srv;
        srv.request.mapfile = filePath;

        if (!scSyncGetStcm_.waitForExistence(ros::Duration(1.0)))
        {
            if (!ros::ok())
            {
                ROS_ERROR("Interrupted while waiting for the service. Exiting.");
                errMsg = "Interrupted while waiting for the service. Exiting.";
                return false;
            }
            ROS_INFO("Service not available, waiting again...");
        }

        if (scSyncGetStcm_.call(srv))
        {
            if (srv.response.success)
            {
                ROS_INFO("Map downloaded successfully");
                return true;
            }
            else
            {
                ROS_ERROR("Failed to get map info: %s", srv.response.message.c_str());
                errMsg = "Failed to get map info: " + srv.response.message;
                return false;
            }
        }
        else
        {
            ROS_ERROR("Failed to call service sync_get_stcm");
            errMsg = "Failed to call service sync_get_stcm";
            return false;
        }
    }

    bool SlamwareRosSdkClient::syncSetStcm(const std::string &mapfile,
                                           std::string &errMsg)
    {
        errMsg.clear();
        slamware_ros_sdk::SyncSetStcm srv;
        srv.request.mapfile = mapfile;

        ROS_INFO("Uploading map %s", mapfile.c_str());

        if (!scSyncSetStcm_.waitForExistence(ros::Duration(1.0)))
        {
            if (!ros::ok())
            {
                ROS_ERROR("Interrupted while waiting for the service. Exiting.");
                return false;
            }
            ROS_INFO("Service not available, waiting again...");
        }

        if (scSyncSetStcm_.call(srv))
        {
            if (srv.response.success)
            {
                ROS_INFO("Map upload completed successfully");
                return true;
            }
            else
            {
                ROS_ERROR("Failed to upload map: %s", srv.response.message.c_str());
                errMsg = "Failed to upload map " + srv.response.message;
                return false;
            }
        }
        else
        {
            ROS_ERROR("Failed to call service sync_set_stcm");
            errMsg = "Failed to call service sync_set_stcm";
            return false;
        }
    }

}
