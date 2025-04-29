/**
 * @file slamware_ros_sdk_server.cpp
 * @brief Implementation of the Slamware ROS SDK Server.
 */
#include "slamware_ros_sdk_server.h"

#include <cassert>
#include <stdexcept>
#include <cmath>
#include <chrono>

namespace slamware_ros_sdk
{

    //////////////////////////////////////////////////////////////////////////

    SlamwareRosSdkServer::SlamwareRosSdkServer()
        : state_(ServerStateNotInit),
          isStopRequested_(false),
          nh_("~"),
          relocalization_active_(false),
          cancel_requested_(false)
    {
        //
    }

    SlamwareRosSdkServer::~SlamwareRosSdkServer()
    {
        cleanup_();
    }

    bool SlamwareRosSdkServer::startRun(std::string &errMsg)
    {
        errMsg.clear();

        const auto oldState = state_.load();
        if (ServerStateNotInit != oldState && ServerStateStopped != oldState)
        {
            errMsg = "it is running or already initialized.";
            return false;
        }

        isStopRequested_.store(false);
        bool bRet = init_(errMsg);
        if (bRet)
        {
            state_.store(ServerStateRunning);
            workThread_ = std::thread(&SlamwareRosSdkServer::workThreadFun_, this);
        }

        if (!bRet)
        {
            cleanup_();
        }
        return bRet;
    }

    void SlamwareRosSdkServer::requestStop()
    {
        isStopRequested_.store(true);
    }

    void SlamwareRosSdkServer::waitUntilStopped()
    {
        if (workThread_.joinable())
            workThread_.join();
        assert(!isRunning_());
    }

    void SlamwareRosSdkServer::requestSyncMap()
    {
        auto wkDat = safeGetMutableWorkData_();
        if (wkDat)
            wkDat->syncMapRequested.store(true);
        auto aurora = safeGetAuroraSdk();
        if (aurora)
            aurora->controller.resyncMapData();
    }

    std::chrono::milliseconds SlamwareRosSdkServer::sfConvFloatSecToBoostMs_(
        float fSec)
    {
        if (fSec < 0.0f)
            throw std::runtime_error("invalid float value of seconds.");

        const std::uint32_t uMs = static_cast<std::uint32_t>(std::floor(fSec * 1000));
        return std::chrono::milliseconds(uMs);
    }

    bool SlamwareRosSdkServer::shouldContinueRunning_() const
    {
        return (!isStopRequested_.load());
    }

    ServerWorkData_ConstPtr SlamwareRosSdkServer::safeGetWorkData_() const
    {
        std::lock_guard<std::mutex> lkGuard(workDatLock_);
        return workDat_;
    }

    ServerWorkData_Ptr SlamwareRosSdkServer::safeGetMutableWorkData_()
    {
        std::lock_guard<std::mutex> lkGuard(workDatLock_);
        return workDat_;
    }

    bool SlamwareRosSdkServer::init_(std::string & /*errMsg*/)
    {
        params_.resetToDefault();
        params_.setBy(nh_);
        auroraSdkConnected_.store(false);
        {
            std::lock_guard<std::mutex> lkGuard(workDatLock_);
            workDat_ = std::make_shared<ServerWorkData>();
        }

        if (!auroraSdkConnected_.load())
        {
            loopTryConnectToAuroraSdk_();
        }
        // init all workers
        {
            serverWorkers_.clear();

            const auto defaultUpdateIntervalForNoneUpdateWorkers = std::chrono::milliseconds(1000u * 60u);

            {
                auto svrWk = std::make_shared<ServerRobotDeviceInfoWorker>(this, "RobotDeviceInfo", defaultUpdateIntervalForNoneUpdateWorkers);
                serverWorkers_.push_back(svrWk);
            }
            {
                ROS_INFO("Odometry:%.4f", params_.odometry_pub_period);
                auto svrWk = std::make_shared<ServerOdometryWorker>(this, "Odometry", sfConvFloatSecToBoostMs_(params_.odometry_pub_period));
                serverWorkers_.push_back(svrWk);
            }
            if (0 < params_.robot_pose_pub_period)
            {
                ROS_INFO("RobotPose:%.4f", params_.robot_pose_pub_period);
                auto svrWk = std::make_shared<ServerRobotPoseWorker>(this, "RobotPose", sfConvFloatSecToBoostMs_(params_.robot_pose_pub_period));
                serverWorkers_.push_back(svrWk);
            }

            if (0 < params_.map_update_period)
            {
                ROS_INFO("ExploreMapUpdate:%.4f", params_.map_update_period);
                auto svrWk = std::make_shared<ServerExploreMapUpdateWorker>(this, "ExploreMapUpdate", sfConvFloatSecToBoostMs_(params_.map_update_period));
                serverWorkers_.push_back(svrWk);
            }

            if (0 < params_.map_pub_period)
            {
                ROS_INFO("ExploreMapPublish:%.4f", params_.map_pub_period);
                auto svrWk = std::make_shared<ServerExploreMapPublishWorker>(this, "ExploreMapPublish", sfConvFloatSecToBoostMs_(params_.map_pub_period));
                serverWorkers_.push_back(svrWk);
            }

            if (0 < params_.scan_pub_period)
            {
                ROS_INFO("LaserScan:%.4f", params_.scan_pub_period);
                auto svrWk = std::make_shared<ServerLaserScanWorker>(this, "LaserScan", sfConvFloatSecToBoostMs_(params_.scan_pub_period));
                serverWorkers_.push_back(svrWk);
            }

            if (0 < params_.imu_raw_data_period)
            {
                ROS_INFO("ServerImuRawDataWorker:%.4f", params_.imu_raw_data_period);
                auto svrWk = std::make_shared<ServerImuRawDataWorker>(this, "ServerImuRawDataWorker", sfConvFloatSecToBoostMs_(params_.imu_raw_data_period));
                serverWorkers_.push_back(svrWk);
            }
            /*new */
            {
                ROS_INFO("RosConnectWorker:%.4f", params_.robot_basic_state_pub_period);
                auto svrWk = std::make_shared<RosConnectWorker>(this, "RosConnectWorker", sfConvFloatSecToBoostMs_(params_.robot_basic_state_pub_period));
                serverWorkers_.push_back(svrWk);
            }

            {
                ROS_INFO("SystemStatus:%.4f", params_.system_status_pub_period);
                // add ServerSystemStatusWorker
                auto svrWk = std::make_shared<ServerSystemStatusWorker>(this, "SystemStatus", sfConvFloatSecToBoostMs_(params_.system_status_pub_period));
                serverWorkers_.push_back(svrWk);
            }

            {
                ROS_INFO("StereoImage:%.4f", params_.stereo_image_pub_period);
                // add ServerStereoImageWorker
                auto svrWk = std::make_shared<ServerStereoImageWorker>(this, "StereoImage", sfConvFloatSecToBoostMs_(params_.stereo_image_pub_period));
                serverWorkers_.push_back(svrWk);
            }

            {
                ROS_INFO("PointCloud:%.4f", params_.point_cloud_pub_period);
                // add ServerPointCloudWorker
                auto svrWk = std::make_shared<ServerPointCloudWorker>(this, "PointCloud", sfConvFloatSecToBoostMs_(params_.point_cloud_pub_period));
                serverWorkers_.push_back(svrWk);
            }

            /**/
        }

        // init all subscriptions
        {
            subSyncMap_ = nh_.subscribe<slamware_ros_sdk::SyncMapRequest>(
                "sync_map", 1,
                &SlamwareRosSdkServer::msgCbSyncMap_, this);

            subClearMap_ = nh_.subscribe<slamware_ros_sdk::ClearMapRequest>(
                "clear_map", 1,
                &SlamwareRosSdkServer::msgCbClearMap_, this);

            subSetMapUpdate_ = nh_.subscribe<slamware_ros_sdk::SetMapUpdateRequest>(
                "set_map_update", 1,
                &SlamwareRosSdkServer::msgCbSetMapUpdate_, this);

            subSetMapLocalization_ = nh_.subscribe<slamware_ros_sdk::SetMapLocalizationRequest>(
                "set_map_localization", 1,
                &SlamwareRosSdkServer::msgCbSetMapLocalization_, this);

            subRelocalizationCancel_ = nh_.subscribe<slamware_ros_sdk::RelocalizationCancelRequest>(
                "relocalization/cancel", 1,
                &SlamwareRosSdkServer::msgCbRelocalizationCancel_, this);
        }

        // init all services
        {
            srvSyncGetStcm_ = nh_.advertiseService(
                "sync_get_stcm",
                &SlamwareRosSdkServer::srvCbSyncGetStcm_, this);

            srvSyncSetStcm_ = nh_.advertiseService(
                "sync_set_stcm",
                &SlamwareRosSdkServer::srvCbSyncSetStcm_, this);

            relocalization_request_srv_ = nh_.advertiseService(
                "relocalization",
                &SlamwareRosSdkServer::srvCbRelocalizationRequest_, this);
        }

        return true;
    }

    void SlamwareRosSdkServer::cleanup_()
    {
        if (isRunning_())
            requestStop();
        waitUntilStopped();

        // de-init all services
        {
            srvSyncGetStcm_ = ros::ServiceServer();
            srvSyncSetStcm_ = ros::ServiceServer();
            relocalization_request_srv_ = ros::ServiceServer();
        }

        // de-init all subscriptions
        {
            subSyncMap_ = ros::Subscriber();
            subClearMap_ = ros::Subscriber();

            subSetMapUpdate_ = ros::Subscriber();
            subSetMapLocalization_ = ros::Subscriber();
            subRelocalizationCancel_ = ros::Subscriber();
        }

        // de-init all publishers
        {
            serverWorkers_.clear();
        }

        {
            std::lock_guard<std::mutex> lkGuard(workDatLock_);
            workDat_.reset();
        }

        state_.store(ServerStateNotInit);
    }

    void SlamwareRosSdkServer::workThreadFun_()
    {
        assert(ServerStateRunning == state_.load());
        ROS_INFO("SlamwareRosSdkServer, work thread begin.");

        while (shouldContinueRunning_() && ros::ok())
        {
            if (!auroraSdkConnected_.load())
            {
                loopTryConnectToAuroraSdk_();
                if (!auroraSdkConnected_.load())
                    continue;
            }

            try
            {
                loopWork_();
            }
            catch (const std::exception &excp)
            {
                ROS_FATAL("loopWork_(), exception: %s.", excp.what());
            }
            catch (...)
            {
                ROS_FATAL("loopWork_(), unknown exception.");
            }

            disconnectAuroraSdk_();

            if (shouldContinueRunning_())
            {
                const std::uint32_t maxSleepMs = (1000u * 3u);
                ROS_INFO("wait %u ms to reconnect and restart work loop.", maxSleepMs);
                roughSleepWait_(maxSleepMs, 100U);
            }
        }

        ROS_INFO("SlamwareRosSdkServer, work thread end.");
        state_.store(ServerStateStopped);
    }

    void SlamwareRosSdkServer::roughSleepWait_(
        std::uint32_t maxSleepMs, std::uint32_t onceSleepMs)
    {
        const auto durOnceSleep = std::chrono::milliseconds(onceSleepMs);
        auto tpNow = std::chrono::steady_clock::now();
        const auto maxSleepTimepoint = tpNow + std::chrono::milliseconds(maxSleepMs);
        while (shouldContinueRunning_() && tpNow < maxSleepTimepoint)
        {
            std::this_thread::sleep_for(durOnceSleep);
            tpNow = std::chrono::steady_clock::now();
        }
    }

    bool SlamwareRosSdkServer::reinitWorkLoop_()
    {
        const std::uint32_t cntWorkers = static_cast<std::uint32_t>(serverWorkers_.size());

        ROS_INFO("reset all %u workers on work loop begin.", cntWorkers);
        for (auto it = serverWorkers_.begin(), itEnd = serverWorkers_.end(); itEnd != it; ++it)
        {
            const auto &svrWk = (*it);
            const auto wkName = svrWk->getWorkerName();
            try
            {
                svrWk->resetOnWorkLoopBegin();
            }
            catch (const std::exception &excp)
            {
                ROS_ERROR("worker: %s, resetOnWorkLoopBegin(), exception: %s.", wkName.c_str(), excp.what());
                return false;
            }
        }

        const std::uint32_t maxSleepMs = (1000u * 2u);
        const std::uint32_t maxLoopTryCnt = 3;
        for (std::uint32_t t = 1; (t <= maxLoopTryCnt && shouldContinueRunning_()); ++t)
        {
            std::uint32_t cntOk = 0;
            for (auto it = serverWorkers_.begin(), itEnd = serverWorkers_.end(); itEnd != it; ++it)
            {
                const auto &svrWk = (*it);
                const auto wkName = svrWk->getWorkerName();
                try
                {
                    if (svrWk->isWorkLoopInitOk())
                    {
                        ++cntOk;
                    }
                    else
                    {
                        if (svrWk->reinitWorkLoop())
                            ++cntOk;
                        else
                            ROS_WARN("failed to init work loop, worker: %s.", wkName.c_str());
                    }
                }
                catch (const std::exception &excp)
                {
                    ROS_ERROR("worker: %s, reinitWorkLoop(), exception: %s.", wkName.c_str(), excp.what());
                }
                catch (...)
                {
                    ROS_ERROR("worker: %s, reinitWorkLoop(), unknown exception.", wkName.c_str());
                }
            }
            // check if all workers are ok.
            if (cntWorkers == cntOk)
            {
                return true;
            }
            else if (t < maxLoopTryCnt)
            {
                ROS_WARN("(%u / %u) cntWorkers: %u, cntOk: %u, wait %u ms to retry.", t, maxLoopTryCnt, cntWorkers, cntOk, maxSleepMs);
                roughSleepWait_(maxSleepMs, 100U);
            }
            else
            {
                ROS_WARN("(%u / %u) cntWorkers: %u, cntOk: %u.", t, maxLoopTryCnt, cntWorkers, cntOk);
            }
        }
        return false;
    }

    void SlamwareRosSdkServer::loopWork_()
    {

        if (reinitWorkLoop_())
        {
            ROS_INFO("successed to reinit all workers on work loop begin.");
        }
        else
        {
            ROS_ERROR("failed or cancelled to reinit work loop.");
            return;
        }

        while (shouldContinueRunning_())
        {
            std::chrono::steady_clock::time_point minNextTriggerTimepoint = std::chrono::steady_clock::now() + std::chrono::milliseconds(100U);

            for (auto it = serverWorkers_.begin(), itEnd = serverWorkers_.end(); itEnd != it; ++it)
            {
                const auto &svrWk = (*it);
                const auto wkName = svrWk->getWorkerName();
                bool shouldReconnect = false;
                try
                {
                    auto tpStart = std::chrono::steady_clock::now();
                    svrWk->checkToPerform();
                    auto tpEnd = std::chrono::steady_clock::now();
                    auto ts = std::chrono::duration_cast<std::chrono::milliseconds>(tpEnd - tpStart).count();
                    if (ts > 100)
                        ROS_INFO("worker(%s) time cost: %ld", wkName.c_str(), ts);
                }

                catch (const std::exception &excp)
                {
                    ROS_ERROR("worker name: %s, exception: %s.", wkName.c_str(), excp.what());
                }
                catch (...)
                {
                    ROS_ERROR("worker name: %s, unknown exception.", wkName.c_str());
                }

                if (shouldReconnect)
                {
                    ROS_ERROR("it should reconnect to slamware.");
                    return;
                }

                const auto tmpNextTp = svrWk->getNextTimepointToTrigger();
                if (tmpNextTp < minNextTriggerTimepoint)
                    minNextTriggerTimepoint = tmpNextTp;
            }

            auto tpNow = std::chrono::steady_clock::now();
            if (tpNow <= minNextTriggerTimepoint)
            {
                const auto durSleep = std::chrono::duration_cast<std::chrono::milliseconds>(minNextTriggerTimepoint - tpNow);
                std::this_thread::sleep_for(durSleep);
            }
        }
    }

    //////////////////////////////////////////////////////////////////////////

    template <class MsgT>
    ros::Subscriber SlamwareRosSdkServer::subscribe_T_(const std::string &msgTopic, std::uint32_t queueSize, typename msg_cb_help_t<MsgT>::msg_cb_perform_fun_t mfpCbPerform)
    {
        typedef msg_cb_help_t<MsgT> TheMsgCbHelpT;

        typename TheMsgCbHelpT::ros_cb_fun_t rosCbFun(
            std::bind(&SlamwareRosSdkServer::msgCbWrapperFun_T_<MsgT>, this, mfpCbPerform, msgTopic, _1));
        return nh_.subscribe(msgTopic, queueSize, rosCbFun);
    }

    void SlamwareRosSdkServer::msgCbSyncMap_(
        const SyncMapRequest::ConstPtr & /*msg*/)
    {
        requestSyncMap();
    }

    void SlamwareRosSdkServer::msgCbClearMap_(
        const ClearMapRequest::ConstPtr &msg)
    {
        auto aurora = safeGetAuroraSdk();
        aurora->controller.requireMapReset();
    }

    void SlamwareRosSdkServer::msgCbSetMapUpdate_(
        const SetMapUpdateRequest::ConstPtr &msg)
    {
        auto aurora = safeGetAuroraSdk();
        aurora->controller.requireMappingMode();
    }

    void SlamwareRosSdkServer::msgCbSetMapLocalization_(
        const SetMapLocalizationRequest::ConstPtr &msg)
    {
        auto aurora = safeGetAuroraSdk();
        aurora->controller.requirePureLocalizationMode();
    }

    bool SlamwareRosSdkServer::srvCbSyncGetStcm_(
        SyncGetStcm::Request &req, SyncGetStcm::Response &resp)
    {
        const char *mapfile = req.mapfile.c_str();
        auto aurora = safeGetAuroraSdk();
        std::promise<int> resultPromise;
        auto resultFuture = resultPromise.get_future();

        auto resultCallback = [](void *userData, int isOK)
        {
            auto promise = reinterpret_cast<std::promise<bool> *>(userData);
            promise->set_value(isOK != 0);
        };

        if (!aurora->mapManager.startDownloadSession(mapfile, resultCallback, &resultPromise))
        {
            ROS_ERROR("Failed to start map storage session");
            resp.success = false;
            resp.message = "Failed to start map storage session";
            return false;
        }

        while (resultFuture.wait_for(std::chrono::milliseconds(100)) == std::future_status::timeout)
        {
            slamtec_aurora_sdk_mapstorage_session_status_t status;
            if (!aurora->mapManager.querySessionStatus(status))
            {
                ROS_ERROR("Failed to query storage status");
                resp.success = false;
                resp.message = "Failed to query storage status";
                return false;
            }

            ROS_INFO("Map download progress: %f", status.progress);
            ros::Duration(0.1).sleep();
        }

        int result = resultFuture.get();
        if (result == 0)
        {
            ROS_ERROR("Failed to save map");
            resp.success = false;
            resp.message = "Failed to save map";
            return false;
        }

        ROS_INFO("Map saved successfully");
        resp.success = true;
        resp.message = "Map saved successfully";
        return true;
    }

    bool SlamwareRosSdkServer::srvCbSyncSetStcm_(
        SyncSetStcm::Request &req, SyncSetStcm::Response &resp)
    {
        auto aurora = safeGetAuroraSdk();
        {
            std::promise<bool> resultPromise;
            auto resultFuture = resultPromise.get_future();

            auto resultCallback = [](void *userData, int isOK)
            {
                auto promise = reinterpret_cast<std::promise<bool> *>(userData);
                promise->set_value(isOK != 0);
            };

            ROS_INFO("Starting map upload: %s", req.mapfile.c_str());

            if (!aurora->mapManager.startUploadSession(req.mapfile.c_str(), resultCallback, &resultPromise))
            {
                ROS_ERROR("Failed to start map upload session");
                resp.success = false;
                resp.message = "Failed to start map upload session";
                return false;
            }

            while (resultFuture.wait_for(std::chrono::milliseconds(100)) == std::future_status::timeout)
            {
                slamtec_aurora_sdk_mapstorage_session_status_t status;
                if (!aurora->mapManager.querySessionStatus(status))
                {
                    ROS_ERROR("Failed to query upload status");
                    resp.success = false;
                    resp.message = "Failed to query upload status";
                    return false;
                }

                ROS_INFO("Map upload progress: %f", status.progress);
                ros::Duration(0.1).sleep();
            }

            int result = resultFuture.get();
            if (result == 0)
            {
                ROS_ERROR("Failed to upload map");
                resp.success = false;
                resp.message = "Failed to upload map";
                return false;
            }

            ROS_INFO("Map uploaded successfully");
            resp.success = true;
            resp.message = "Map uploaded successfully";
        }

        requestSyncMap();
        aurora->controller.requireMappingMode();

        return true;
    }

    bool SlamwareRosSdkServer::discoverAndSelectAuroraDevice(rp::standalone::aurora::RemoteSDK *sdk, rp::standalone::aurora::SDKServerConnectionDesc &selectedDeviceDesc)
    {
        std::vector<rp::standalone::aurora::SDKServerConnectionDesc> serverList;
        size_t count = sdk->getDiscoveredServers(serverList, 32);
        if (count == 0)
        {
            ROS_ERROR("No aurora devices found");
            return false;
        }

        ROS_INFO("Found %zu aurora devices", count);
        for (size_t i = 0; i < count; i++)
        {
            ROS_INFO("Device %zu", i);
            for (size_t j = 0; j < serverList[i].size(); ++j)
            {
                auto &connectionOption = serverList[i][j];
                ROS_INFO("  option %zu: %s", j, connectionOption.toLocatorString().c_str());
            }
        }

        // Select the first device
        selectedDeviceDesc = serverList[0];
        ROS_INFO("Selected first device: %s", selectedDeviceDesc[0].toLocatorString().c_str());
        return true;
    }

    void SlamwareRosSdkServer::loopTryConnectToAuroraSdk_()
    {
        std::uint32_t tryCnt = 1;
        while (shouldContinueRunning_())
        {
            {
                ROS_INFO("Trying to connect to Aurora, tryCnt: %u.", tryCnt);
                connectAuroraSdk_();
                if (auroraSdkConnected_.load())
                {
                    ROS_INFO("Connected to Aurora, OK, tryCnt: %u.", tryCnt);
                    return;
                }

                int reconnWaitMs;
                if (!nh_.getParam("reconn_wait_ms", reconnWaitMs))
                {
                    reconnWaitMs = 3000; // default value
                }
                ROS_ERROR("Failed to connect to Aurora, tryCnt: %u, waiting %d ms to retry.", tryCnt, reconnWaitMs);
            }

            int reconnWaitMs;
            nh_.getParam("reconn_wait_ms", reconnWaitMs);
            const std::uint32_t maxSleepMs = (0 <= reconnWaitMs ? (std::uint32_t)reconnWaitMs : 0U);
            roughSleepWait_(maxSleepMs, 100U);
            ++tryCnt;
        }
    }

    void SlamwareRosSdkServer::connectAuroraSdk_()
    {
        auroraSdk_ = rp::standalone::aurora::RemoteSDK::CreateSession();
        if (auroraSdk_ == nullptr)
        {
            ROS_ERROR("Failed to create Aurora SDK session.");
            return;
        }

        std::string ip;
        int port;
        if (!nh_.getParam("ip_address", ip))
        {
            ip = "127.0.0.1"; // default IP
        }
        if (!nh_.getParam("robot_port", port))
        {
            port = 1445; // default port
        }

        rp::standalone::aurora::SDKServerConnectionDesc selectedDeviceDesc;
        const char *connectionString = !ip.empty() ? ip.c_str() : nullptr;

        if (connectionString == nullptr)
        {
            ROS_INFO("Device connection string not provided, trying to discover Aurora devices...");
            std::this_thread::sleep_for(std::chrono::seconds(5));

            if (!discoverAndSelectAuroraDevice(auroraSdk_, selectedDeviceDesc))
            {
                ROS_ERROR("Failed to discover Aurora devices");
                return;
            }
        }
        else
        {
            selectedDeviceDesc = rp::standalone::aurora::SDKServerConnectionDesc(connectionString);
            ROS_INFO("Selected device: %s", selectedDeviceDesc[0].toLocatorString().c_str());
        }

        // Connect to the selected device
        ROS_INFO("Connecting to the selected device...");
        if (!auroraSdk_->connect(selectedDeviceDesc))
        {
            ROS_ERROR("Failed to connect to the selected device");
            return;
        }
        ROS_INFO("Connected to the selected device");
        auroraSdk_->controller.setMapDataSyncing(true);
        auroraSdkConnected_.store(true);

        LIDAR2DGridMapGenerationOptions genOption;
        if (!auroraSdk_->lidar2DMapBuilder.startPreviewMapBackgroundUpdate(genOption))
        {
            ROS_ERROR("Failed to start preview map update");
        }
        auroraSdkConnected_.store(true);
    }

    void SlamwareRosSdkServer::disconnectAuroraSdk_()
    {
        if (auroraSdkConnected_.load())
        {
            {
                std::lock_guard<std::mutex> lkGuard(auroraSdkLock_);
                auroraSdk_->lidar2DMapBuilder.stopPreviewMapBackgroundUpdate();
                auroraSdk_->disconnect();
                auroraSdk_->release();
            }
            auroraSdkConnected_.store(false);
        }
    }

    void SlamwareRosSdkServer::msgCbRelocalizationCancel_(
        const RelocalizationCancelRequest::ConstPtr &msg)
    {
        // Handle relocalization cancel using Aurora SDK
        auto aurora = safeGetAuroraSdk();
        if (!aurora->controller.cancelRelocalization())
        {
            ROS_ERROR("Failed to cancel relocalization");
            return;
        }
        ROS_INFO("Relocalization cancel requested");

        cancel_requested_.store(true);
        relocalization_active_.store(false);
    }

    bool SlamwareRosSdkServer::srvCbRelocalizationRequest_(
        slamware_ros_sdk::RelocalizationRequest::Request &req,
        slamware_ros_sdk::RelocalizationRequest::Response &resp)
    {
        if (relocalization_active_.load())
        {
            ROS_INFO("Relocalization already in progress");
            resp.success = false;
            return false;
        }
        // Handle relocalization request using Aurora SDK
        auto aurora = safeGetAuroraSdk();
        aurora->controller.requireRelocalization();
        ROS_INFO("Relocalization requested");
        relocalization_active_.store(true);
        cancel_requested_.store(false);

        relocalization_future_ = std::async(std::launch::async, [this]()
                                            {
            while (true)
            {
                checkRelocalizationStatus();
                if (cancel_requested_.load() || !relocalization_active_.load())
                {
                    return;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } });

        resp.success = true;
        return true;
    }

    void SlamwareRosSdkServer::checkRelocalizationStatus()
    {
        slamtec_aurora_sdk_relocalization_status_t result;
        auto aurora = safeGetAuroraSdk();
        aurora->dataProvider.peekRelocalizationStatus(result);
        if (result.status == 2)
        {
            ROS_INFO("Relocalization succeeded");
            relocalization_active_.store(false);
        }
        else if (result.status == 3)
        {
            ROS_INFO("Relocalization failed");
            relocalization_active_.store(false);
        }
        else if (result.status == 4)
        {
            ROS_INFO("Relocalization canceled by system");
            relocalization_active_.store(false);
        }
    }
}
