/**
 * @file slamware_ros_sdk_server.h
 * @brief Header file for the Slamware ROS SDK Server class.
 *
 * @author kint.zhao
 * @date 2017-07-21
 *
 * @modified by yun.li@slamtec.com, 2019
 */

#pragma once

#include "server_workers.h"

#include <slamware_ros_sdk/SyncGetStcm.h>
#include <slamware_ros_sdk/SyncSetStcm.h>
#include <slamware_ros_sdk/RelocalizationRequest.h>
#include <slamware_ros_sdk/RelocalizationCancelRequest.h>
#include <slamware_ros_sdk/SyncMapRequest.h>
#include <slamware_ros_sdk/ClearMapRequest.h>
#include <slamware_ros_sdk/SetMapUpdateRequest.h>
#include <slamware_ros_sdk/SetMapLocalizationRequest.h>
#include <message_filters/subscriber.h>

#include <atomic>
#include <thread>
#include <mutex>
#include <future>
#include <functional>
#include <aurora_pubsdk_inc.h>

namespace slamware_ros_sdk
{

    class SlamwareRosSdkServer
    {
    private:
        friend class ServerWorkerBase;

    public:
        SlamwareRosSdkServer();
        ~SlamwareRosSdkServer();

        bool startRun(std::string &errMsg);
        void requestStop();
        void waitUntilStopped(); // not thread-safe

    public:
        void requestSyncMap();
        rp::standalone::aurora::RemoteSDK *safeGetAuroraSdk()
        {
            std::lock_guard<std::mutex> lkGuard(auroraSdkLock_);
            return auroraSdk_;
        }

    private:
        enum ServerState
        {
            ServerStateNotInit,
            ServerStateRunning,
            ServerStateStopped
        };

        // typedef long     slamware_platform_t;

        template <class MsgT>
        struct msg_cb_help_t
        {
            typedef MsgT msg_t;
            typedef typename msg_t::ConstPtr const_msg_shared_ptr;
            typedef void (SlamwareRosSdkServer::*msg_cb_perform_fun_t)(const const_msg_shared_ptr &);
            typedef std::function<void(const const_msg_shared_ptr &)> ros_cb_fun_t; // callback function for ROS.
        };

        template <class SrvMsgT>
        struct srv_cb_help_t
        {
            typedef SrvMsgT srv_msg_t;
            typedef typename srv_msg_t::Request request_t;
            typedef typename srv_msg_t::Response response_t;
            typedef bool (SlamwareRosSdkServer::*srv_cb_perform_fun_t)(request_t &, response_t &);
            typedef std::function<bool(request_t &, response_t &)> ros_cb_fun_t; // callback function for ROS.
        };

    private:
        static std::chrono::milliseconds sfConvFloatSecToBoostMs_(float fSec);

        bool isRunning_() const { return ServerStateRunning == state_.load(); }
        bool shouldContinueRunning_() const;

        const ros::NodeHandle &rosNodeHandle_() const { return nh_; }
        ros::NodeHandle &rosNodeHandle_() { return nh_; }

        const ServerParams &serverParams_() const { return params_; }

        const tf::TransformBroadcaster &tfBroadcaster_() const { return tfBrdcstr_; }
        tf::TransformBroadcaster &tfBroadcaster_() { return tfBrdcstr_; }

        ServerWorkData_ConstPtr safeGetWorkData_() const;
        ServerWorkData_Ptr safeGetMutableWorkData_();

        bool safeIsSlamwarePlatformConnected_() const;

        bool init_(std::string &errMsg);
        void cleanup_();

        void workThreadFun_();

        void roughSleepWait_(std::uint32_t maxSleepMs, std::uint32_t onceSleepMs);
        void loopTryConnectToSlamwarePlatform_();

        bool reinitWorkLoop_();
        void loopWork_();

        //////////////////////////////////////////////////////////////////////////
        // subscribed messages
        //////////////////////////////////////////////////////////////////////////

        template <class MsgT>
        void msgCbWrapperFun_T_(typename msg_cb_help_t<MsgT>::msg_cb_perform_fun_t mfpCbPerform, const std::string &msgTopic, const typename msg_cb_help_t<MsgT>::const_msg_shared_ptr &msg);
        template <class MsgT>
        ros::Subscriber subscribe_T_(const std::string &msgTopic, std::uint32_t queueSize, typename msg_cb_help_t<MsgT>::msg_cb_perform_fun_t mfpCbPerform);

        void msgCbSyncMap_(const SyncMapRequest::ConstPtr &msg);
        void msgCbClearMap_(const ClearMapRequest::ConstPtr &msg);
        void msgCbSetMapUpdate_(const SetMapUpdateRequest::ConstPtr &msg);
        void msgCbSetMapLocalization_(const SetMapLocalizationRequest::ConstPtr &msg);

        // Relocalization cancel
        void msgCbRelocalizationCancel_(const RelocalizationCancelRequest::ConstPtr &msg);

        //////////////////////////////////////////////////////////////////////////
        // advertised services
        //////////////////////////////////////////////////////////////////////////

        template <class SrvMsgT>
        bool srvCbWrapperFun_T_(typename srv_cb_help_t<SrvMsgT>::srv_cb_perform_fun_t mfpCbPerform, const std::string &srvMsgTopic, typename srv_cb_help_t<SrvMsgT>::request_t &req, typename srv_cb_help_t<SrvMsgT>::response_t &resp);
        template <class SrvMsgT>
        ros::ServiceServer advertiseService_T_(const std::string &srvMsgTopic, typename srv_cb_help_t<SrvMsgT>::srv_cb_perform_fun_t mfpCbPerform);

        bool srvCbSyncGetStcm_(SyncGetStcm::Request &req, SyncGetStcm::Response &resp);
        bool srvCbSyncSetStcm_(SyncSetStcm::Request &req, SyncSetStcm::Response &resp);

        bool srvCbRelocalizationRequest_(slamware_ros_sdk::RelocalizationRequest::Request &req, slamware_ros_sdk::RelocalizationRequest::Response &resp);

        void loopTryConnectToAuroraSdk_();
        void connectAuroraSdk_();
        void disconnectAuroraSdk_();
        bool discoverAndSelectAuroraDevice(rp::standalone::aurora::RemoteSDK *sdk, rp::standalone::aurora::SDKServerConnectionDesc &selectedDeviceDesc);
        //////////////////////////////////////////////////////////////////////////
        void checkRelocalizationStatus();

    private:
        std::atomic<ServerState> state_;
        std::atomic<bool> isStopRequested_;

        ros::NodeHandle nh_;
        ServerParams params_;

        tf::TransformBroadcaster tfBrdcstr_;

        mutable std::mutex workDatLock_;
        ServerWorkData_Ptr workDat_;

        std::vector<ServerWorkerBase_Ptr> serverWorkers_;

        // subscriptions
        ros::Subscriber subSyncMap_;
        ros::Subscriber subClearMap_;
        ros::Subscriber subSetMapUpdate_;
        ros::Subscriber subSetMapLocalization_;
        ros::Subscriber subRelocalizationCancel_;

        // rpos::actions::VelocityControlMoveAction velocityControllAction_;

        // services
        ros::ServiceServer srvSyncGetStcm_;
        ros::ServiceServer srvSyncSetStcm_;
        ros::ServiceServer relocalization_request_srv_;

        std::thread workThread_;

        mutable std::mutex slamwarePltfmLock_;
        // slamware_platform_t slamwarePltfm_;
        rp::standalone::aurora::RemoteSDK *auroraSdk_;

        std::mutex auroraSdkLock_;
        std::atomic_bool auroraSdkConnected_;
        std::atomic_bool relocalization_active_;
        std::future<void> relocalization_future_;
        std::atomic<bool> cancel_requested_;
    };

}
