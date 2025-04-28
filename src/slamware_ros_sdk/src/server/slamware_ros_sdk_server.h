#pragma once

#include "server_workers.h"

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

#include <slamware_ros_sdk/srv/sync_get_stcm.hpp>
#include <slamware_ros_sdk/srv/sync_set_stcm.hpp>
#include <slamware_ros_sdk/srv/relocalization_request.hpp>
#include <slamware_ros_sdk/msg/relocalization_cancel_request.hpp>
#include <slamware_ros_sdk/msg/sync_map_request.hpp>
#include <slamware_ros_sdk/msg/clear_map_request.hpp>
#include <slamware_ros_sdk/msg/set_map_update_request.hpp>
#include <slamware_ros_sdk/msg/set_map_localization_request.hpp>

#include <message_filters/subscriber.h>

#include <atomic>
#include <aurora_pubsdk_inc.h>
#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>


namespace slamware_ros_sdk
{
    class SlamwareRosSdkServer : public rclcpp::Node
    {
    private:
        friend class ServerWorkerBase;

    public:
        SlamwareRosSdkServer();
        ~SlamwareRosSdkServer();

        bool startRun(std::string& errMsg);
        void requestStop();
        void waitUntilStopped(); // not thread-safe

    public:
        void requestSyncMap();

        rp::standalone::aurora::RemoteSDK* safeGetAuroraSdk() 
        {
            std::lock_guard<std::mutex> lkGuard(auroraSdkLock_);
            return auroraSdk_;
        }

    private:
        enum ServerState
        {
            ServerStateNotInit
            , ServerStateRunning
            , ServerStateStopped
        };

    private:
        static std::chrono::milliseconds sfConvFloatSecToChronoMs_(float fSec);

        bool isRunning_() const { return ServerStateRunning == state_.load(); }
        bool shouldContinueRunning_() const;

        const ServerParams& serverParams_() const { return params_; }
        
        const std::unique_ptr<tf2_ros::TransformBroadcaster>& tfBroadcaster_() const { return tfBrdcstr_; }
        std::unique_ptr<tf2_ros::TransformBroadcaster>& tfBroadcaster_() { return tfBrdcstr_; }

        ServerWorkData_ConstPtr safeGetWorkData_() const;
        ServerWorkData_Ptr safeGetMutableWorkData_();

        bool init_(std::string& errMsg);
        void cleanup_();
        void workThreadFun_();
        void roughSleepWait_(std::uint32_t maxSleepMs, std::uint32_t onceSleepMs);
        void loopTryConnectToSlamwarePlatform_();
        bool reinitWorkLoop_();
        void loopWork_();
        void loopTryConnectToAuroraSdk_();
        void connectAuroraSdk_();
        void disconnectAuroraSdk_();
        bool discoverAndSelectAuroraDevice(rp::standalone::aurora::RemoteSDK *sdk, rp::standalone::aurora::SDKServerConnectionDesc &selectedDeviceDesc);

        //////////////////////////////////////////////////////////////////////////
        // subscribed messages
        void msgCbSyncMap_(const slamware_ros_sdk::msg::SyncMapRequest::SharedPtr msg);
        void msgCbClearMap_(const slamware_ros_sdk::msg::ClearMapRequest::SharedPtr msg);
        void msgCbSetMapUpdate_(const slamware_ros_sdk::msg::SetMapUpdateRequest::SharedPtr msg);
        void msgCbSetMapLocalization_(const slamware_ros_sdk::msg::SetMapLocalizationRequest::SharedPtr msg);
        void msgCbRelocalizationCancel_(const slamware_ros_sdk::msg::RelocalizationCancelRequest::SharedPtr msg);

        //////////////////////////////////////////////////////////////////////////
        // advertised services
        //////////////////////////////////////////////////////////////////////////
        bool srvCbSyncGetStcm_(slamware_ros_sdk::srv::SyncGetStcm::Request::SharedPtr req, slamware_ros_sdk::srv::SyncGetStcm::Response::SharedPtr resp);
        bool srvCbSyncSetStcm_(slamware_ros_sdk::srv::SyncSetStcm::Request::SharedPtr req, slamware_ros_sdk::srv::SyncSetStcm::Response::SharedPtr resp);
        bool srvCbRelocalizationRequest_(slamware_ros_sdk::srv::RelocalizationRequest::Request::SharedPtr req,
                                         slamware_ros_sdk::srv::RelocalizationRequest::Response::SharedPtr resp);
        rclcpp::Service<slamware_ros_sdk::srv::RelocalizationRequest>::SharedPtr relocalization_request_srv_;
        void checkRelocalizationStatus();

    private:     
        std::atomic<ServerState> state_;
        std::atomic<bool> isStopRequested_;
        ServerParams params_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tfBrdcstr_;

        mutable std::mutex workDatLock_;
        ServerWorkData_Ptr workDat_;
        std::vector<ServerWorkerBase_Ptr> serverWorkers_;
        // subscriptions  
        rclcpp::Subscription<slamware_ros_sdk::msg::SyncMapRequest>::SharedPtr subSyncMap_;
        rclcpp::Subscription<slamware_ros_sdk::msg::ClearMapRequest>::SharedPtr subClearMap_;
        rclcpp::Subscription<slamware_ros_sdk::msg::SetMapUpdateRequest>::SharedPtr subSetMapUpdate_;
        rclcpp::Subscription<slamware_ros_sdk::msg::SetMapLocalizationRequest>::SharedPtr subSetMapLocalization_;
        rclcpp::Subscription<slamware_ros_sdk::msg::RelocalizationCancelRequest>::SharedPtr subRelocalizationCancel_;
        
        // services
        rclcpp::Service<slamware_ros_sdk::srv::SyncGetStcm>::SharedPtr srvSyncGetStcm_;
        rclcpp::Service<slamware_ros_sdk::srv::SyncSetStcm>::SharedPtr srvSyncSetStcm_;

        std::thread workThread_;
        rp::standalone::aurora::RemoteSDK *auroraSdk_;
        std::mutex auroraSdkLock_;
        std::atomic_bool auroraSdkConnected_;
        std::atomic_bool relocalization_active_;
        std::future<void> relocalization_future_;
        std::atomic<bool> cancel_requested_;
    };

}
