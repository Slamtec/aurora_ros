
#include <slamware_ros_sdk/slamware_ros_sdk_client.h>

namespace slamware_ros_sdk {
    
    SlamwareRosSdkClient::SlamwareRosSdkClient(const char* serverNodeName
        , const char* msgNamePrefix
        )
        : Node("slamware_ros_sdk_client")
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
            pubSyncMap_ = this->create_publisher<slamware_ros_sdk::msg::SyncMapRequest>(genTopicFullName_("sync_map"), 1);
            pubClearMap_ = this->create_publisher<slamware_ros_sdk::msg::ClearMapRequest>(genTopicFullName_("clear_map"), 1);
            pubSetMapUpdate_ = this->create_publisher<slamware_ros_sdk::msg::SetMapUpdateRequest>(genTopicFullName_("set_map_update"), 1);
            pubSetMapLocalization_ = this->create_publisher<slamware_ros_sdk::msg::SetMapLocalizationRequest>(genTopicFullName_("set_map_localization"), 1);
        }

        // initialize service clients
        {
            scSyncGetStcm_ = this->create_client<slamware_ros_sdk::srv::SyncGetStcm>(genTopicFullName_("sync_get_stcm"));
            scSyncSetStcm_ = this->create_client<slamware_ros_sdk::srv::SyncSetStcm>(genTopicFullName_("sync_set_stcm"));
        }
    }

    SlamwareRosSdkClient::~SlamwareRosSdkClient()
    {
        //
    }

    bool SlamwareRosSdkClient::syncGetStcm(std::string& errMsg , const std::string& filePath)
    {
        errMsg.clear();
        {
            auto request = std::make_shared<slamware_ros_sdk::srv::SyncGetStcm::Request>();
            request->mapfile = filePath;

            while (!scSyncGetStcm_->wait_for_service(std::chrono::seconds(1)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    errMsg = "Interrupted while waiting for the service. Exiting.";
                    return false;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
            }

            auto result = scSyncGetStcm_->async_send_request(request);

            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = result.get();
                if (response->success)
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Map downloaded successfully");
                    return true;
                }
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to get map info: %s", response->message.c_str());
                    errMsg = "Failed to get map info: " + response->message;
                    return false;
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service sync_get_stcm");
                errMsg = "Failed to call service sync_get_stcm";
                return false;
            }
        }
        return true;
    }

    bool SlamwareRosSdkClient::syncSetStcm(const std::string &mapfile, 
        std::string& errMsg)
    {
        errMsg.clear();
        auto request = std::make_shared<slamware_ros_sdk::srv::SyncSetStcm::Request>();
        request->mapfile = mapfile;
        RCLCPP_INFO(this->get_logger(), "Uploading map %s", mapfile.c_str());
        while (!scSyncSetStcm_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto result = scSyncSetStcm_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result.get();
            if (response->success)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Map upload completed successfully");
                return true;
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to upload map: %s", response->message.c_str());
                errMsg = "Failed to upload map " + response->message;
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service sync_set_stcm");
            errMsg = "Failed to call service sync_set_stcm";
            return false;
        }
    }
}
