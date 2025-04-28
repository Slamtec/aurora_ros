
#pragma once

#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/message_filter.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <slamware_ros_sdk/msg/sync_map_request.hpp>
#include <slamware_ros_sdk/msg/clear_map_request.hpp>
#include <slamware_ros_sdk/msg/set_map_update_request.hpp>
#include <slamware_ros_sdk/msg/set_map_localization_request.hpp>

#include <slamware_ros_sdk/srv/sync_get_stcm.hpp>
#include <slamware_ros_sdk/srv/sync_set_stcm.hpp>

#include <boost/filesystem/path.hpp>
#include <memory>

namespace slamware_ros_sdk {

    class SlamwareRosSdkClient: public rclcpp::Node
    {
    public:
        typedef boost::filesystem::path             fs_path_t;

    public:
        explicit SlamwareRosSdkClient(const char* serverNodeName = nullptr
            , const char* msgNamePrefix = nullptr
            );
        ~SlamwareRosSdkClient();

    public:
        //////////////////////////////////////////////////////////////////////////

        void syncMap(const slamware_ros_sdk::msg::SyncMapRequest& msg) { return pubSyncMap_->publish(msg); }

        void clearMap(const slamware_ros_sdk::msg::ClearMapRequest& msg) { pubClearMap_->publish(msg); }
        void setMapUpdate(const slamware_ros_sdk::msg::SetMapUpdateRequest& msg) { pubSetMapUpdate_->publish(msg); }
        void setMapLocalization(const slamware_ros_sdk::msg::SetMapLocalizationRequest& msg) { pubSetMapLocalization_->publish(msg); }

        // get stcm and write to filePath.
        bool syncGetStcm(std::string& errMsg , const std::string& filePath);
        // load stcm from filePath, and upload to slamware.
        bool syncSetStcm(const std::string &mapfile,
             std::string& errMsg
            );

        //////////////////////////////////////////////////////////////////////////

    private:
        std::string genTopicFullName_(const std::string& strName) const { return msgNamePrefix_ + strName; }

    private:
        std::string sdkServerNodeName_;
        std::string msgNamePrefix_;

        rclcpp::Publisher<slamware_ros_sdk::msg::SyncMapRequest>::SharedPtr pubSyncMap_;
        rclcpp::Publisher<slamware_ros_sdk::msg::ClearMapRequest>::SharedPtr pubClearMap_;
        rclcpp::Publisher<slamware_ros_sdk::msg::SetMapUpdateRequest>::SharedPtr pubSetMapUpdate_;
        rclcpp::Publisher<slamware_ros_sdk::msg::SetMapLocalizationRequest>::SharedPtr pubSetMapLocalization_;
        rclcpp::Client<slamware_ros_sdk::srv::SyncGetStcm>::SharedPtr scSyncGetStcm_;
        rclcpp::Client<slamware_ros_sdk::srv::SyncSetStcm>::SharedPtr scSyncSetStcm_;
    };

}
