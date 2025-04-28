#include  "slamware_ros_sdk_server.h"
#include <stdexcept>
#include <cmath>
#include <memory>

namespace slamware_ros_sdk {

    using namespace std::placeholders;
    //////////////////////////////////////////////////////////////////////////

    SlamwareRosSdkServer::SlamwareRosSdkServer()
        : Node("slamware_ros_sdk_server")
        , state_(ServerStateNotInit)
        , isStopRequested_(false)
        , relocalization_active_(false)
        , cancel_requested_(false)
    {
        tfBrdcstr_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    SlamwareRosSdkServer::~SlamwareRosSdkServer()
    {
        cleanup_();
    }

    bool SlamwareRosSdkServer::startRun(std::string& errMsg)
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

    std::chrono::milliseconds SlamwareRosSdkServer::sfConvFloatSecToChronoMs_(float fSec)
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

    bool SlamwareRosSdkServer::discoverAndSelectAuroraDevice(rp::standalone::aurora::RemoteSDK *sdk, rp::standalone::aurora::SDKServerConnectionDesc &selectedDeviceDesc)
    {
        std::vector<SDKServerConnectionDesc> serverList;
        size_t count = sdk->getDiscoveredServers(serverList, 32);
        if (count == 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "No aurora devices found");
            return false;
        }

        RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "Found %ld aurora devices", count);
        for (size_t i = 0; i < count; i++)
        {
            RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "Device %ld", i);
            for (size_t j = 0; j < serverList[i].size(); ++j)
            {
                auto &connectionOption = serverList[i][j];
                RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "  option %ld: %s", j, connectionOption.toLocatorString().c_str());
            }
        }

        // select the first device
        selectedDeviceDesc = serverList[0];
        RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "Selected first device: %s", selectedDeviceDesc[0].toLocatorString().c_str());
        return true;
    }


    void SlamwareRosSdkServer::loopTryConnectToAuroraSdk_()
    {
        std::uint32_t tryCnt = 1;
        while (shouldContinueRunning_())
        {
            {
                RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "try to connect to aurora, tryCnt: %u.", tryCnt);
                connectAuroraSdk_();
                if (auroraSdkConnected_.load())
                {
                    RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "connect to aurora, OK, tryCnt: %u.", tryCnt);
                    return;
                }
                int iVal = params_.getParameter<int>("reconn_wait_ms");
                RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "connect to aurora, FAILED, tryCnt: %u, wait %d ms to retry."
                    , tryCnt, iVal);
            }
            int iVal = params_.getParameter<int>("reconn_wait_ms");
            const std::uint32_t maxSleepMs = (0 <= iVal ? (std::uint32_t)iVal : 0U);
            roughSleepWait_(maxSleepMs, 100U);
            ++tryCnt;
        }
    }


    void SlamwareRosSdkServer::connectAuroraSdk_()
    {
        auroraSdk_ = rp::standalone::aurora::RemoteSDK::CreateSession();
        if(auroraSdk_ == nullptr)
        {
            RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "Failed to create aurora sdk session.");
            return;
        }

        std::string ip = params_.getParameter<std::string>("ip_address");
        rp::standalone::aurora::SDKServerConnectionDesc selectedDeviceDesc;
        const char *connectionString = nullptr;
        if (!ip.empty())
        {
            connectionString = ip.c_str();
        }

        if (connectionString == nullptr)
        {
            RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "Device connection string not provided, try to discover aurora devices..." );
            std::this_thread::sleep_for(std::chrono::seconds(5));

            if (!discoverAndSelectAuroraDevice(auroraSdk_, selectedDeviceDesc))
            {
                RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "Failed to discover aurora devices");
                return;
            }
        }
        else
        {
            selectedDeviceDesc = SDKServerConnectionDesc(connectionString);
            RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "Selected device: %s", selectedDeviceDesc[0].toLocatorString().c_str());
        }

        // connect to the selected device
        RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "Connecting to the selected device...");
        if (!auroraSdk_->connect(selectedDeviceDesc))
        {
            RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "Failed to connect to the selected device");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "Connected to the selected device");
        auroraSdk_->controller.setMapDataSyncing(true);
        LIDAR2DGridMapGenerationOptions genOption;
        if(!auroraSdk_->lidar2DMapBuilder.startPreviewMapBackgroundUpdate(genOption)) 
        {
            RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "Failed to start preview map update");
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

    bool SlamwareRosSdkServer::init_(std::string& /*errMsg*/)
    {
        // params_.resetToDefault();  // TODO:
        std::string ip = params_.getParameter<std::string>("ip_address");
        RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "ip:%s", ip.c_str());
        
        auroraSdkConnected_.store(false);
        params_.setBy(shared_from_this());
        {
            std::lock_guard<std::mutex> lkGuard(workDatLock_);
            workDat_ = std::make_shared<ServerWorkData>();
        }

        if(!auroraSdkConnected_.load())
        {
            loopTryConnectToAuroraSdk_();
        }

        // init all workers
        {
            serverWorkers_.clear();

            const auto defaultUpdateIntervalForNoneUpdateWorkers = std::chrono::milliseconds(1000u * 60u);

            {
                auto svrWk = std::make_shared<ServerOdometryWorker>(this, "Odometry", sfConvFloatSecToChronoMs_(params_.getParameter<float>("odometry_pub_period")));
                serverWorkers_.push_back(svrWk);
            }

            if (0 < params_.getParameter<float>("robot_pose_pub_period"))
            {
                auto svrWk = std::make_shared<ServerRobotPoseWorker>(this, "RobotPose", sfConvFloatSecToChronoMs_(params_.getParameter<float>("robot_pose_pub_period")));
                serverWorkers_.push_back(svrWk);
            }

            if (0 < params_.getParameter<float>("map_update_period"))
            {
                auto svrWk = std::make_shared<ServerExploreMapUpdateWorker>(this, "ExploreMapUpdate", sfConvFloatSecToChronoMs_(params_.getParameter<float>("map_update_period")));
                serverWorkers_.push_back(svrWk);
            }

            if (0 < params_.getParameter<float>("map_pub_period"))
            {
                auto svrWk = std::make_shared<ServerExploreMapPublishWorker>(this, "ExploreMapPublish", sfConvFloatSecToChronoMs_(params_.getParameter<float>("map_pub_period")));
                serverWorkers_.push_back(svrWk);
            }

            if (0 < params_.getParameter<float>("scan_pub_period"))
            {
                auto svrWk = std::make_shared<ServerLaserScanWorker>(this, "LaserScan", sfConvFloatSecToChronoMs_(params_.getParameter<float>("scan_pub_period")));
                serverWorkers_.push_back(svrWk);
            }
            {
                auto svrWk = std::make_shared<ServerImuRawDataWorker>(this, "ServerImuRawDataWorker", sfConvFloatSecToChronoMs_(params_.getParameter<float>("imu_raw_data_period")));
                serverWorkers_.push_back(svrWk);
            }

            {
                auto svrWk = std::make_shared<RosConnectWorker>(this, "RosConnectWorker", sfConvFloatSecToChronoMs_(params_.getParameter<float>("robot_basic_state_pub_period")));
                serverWorkers_.push_back(svrWk);
            }

            {
                // add ServerSystemStatusWorker
                auto svrWk = std::make_shared<ServerSystemStatusWorker>(this, "SystemStatus", sfConvFloatSecToChronoMs_(params_.getParameter<float>("system_status_pub_period")));
                serverWorkers_.push_back(svrWk);
            }

            {
                //add ServerStereoImageWorker
                auto svrWk = std::make_shared<ServerStereoImageWorker>(this, "StereoImage", sfConvFloatSecToChronoMs_(params_.getParameter<float>("stereo_image_pub_period")));
                serverWorkers_.push_back(svrWk);
            }

            {
                //add ServerPointCloudWorker
                auto svrWk = std::make_shared<ServerPointCloudWorker>(this, "PointCloud", sfConvFloatSecToChronoMs_(params_.getParameter<float>("point_cloud_pub_period")));   
                serverWorkers_.push_back(svrWk);
            }
        }

        // init all subscriptions
        {
            subSyncMap_ = this->create_subscription<slamware_ros_sdk::msg::SyncMapRequest>(
                "/slamware_ros_sdk_server_node/sync_map", 1,
                std::bind(&SlamwareRosSdkServer::msgCbSyncMap_, this, std::placeholders::_1));

            subClearMap_ = this->create_subscription<slamware_ros_sdk::msg::ClearMapRequest>(
                "/slamware_ros_sdk_server_node/clear_map", 1,
                std::bind(&SlamwareRosSdkServer::msgCbClearMap_, this, std::placeholders::_1));

            subSetMapUpdate_ = this->create_subscription<slamware_ros_sdk::msg::SetMapUpdateRequest>(
                "/slamware_ros_sdk_server_node/set_map_update", 1,
                std::bind(&SlamwareRosSdkServer::msgCbSetMapUpdate_, this, std::placeholders::_1));

            subSetMapLocalization_ = this->create_subscription<slamware_ros_sdk::msg::SetMapLocalizationRequest>(
                "/slamware_ros_sdk_server_node/set_map_localization", 1,
                std::bind(&SlamwareRosSdkServer::msgCbSetMapLocalization_, this, std::placeholders::_1));

            subRelocalizationCancel_ = this->create_subscription<slamware_ros_sdk::msg::RelocalizationCancelRequest>(
                "/slamware_ros_sdk_server_node/relocalization/cancel", 1,
                std::bind(&SlamwareRosSdkServer::msgCbRelocalizationCancel_, this, std::placeholders::_1));
        }

        // init all services
        {
            srvSyncGetStcm_ = this->create_service<slamware_ros_sdk::srv::SyncGetStcm>(
                "/slamware_ros_sdk_server_node/sync_get_stcm",
                std::bind(&SlamwareRosSdkServer::srvCbSyncGetStcm_, this, std::placeholders::_1, std::placeholders::_2));

            srvSyncSetStcm_ = this->create_service<slamware_ros_sdk::srv::SyncSetStcm>(
                "/slamware_ros_sdk_server_node/sync_set_stcm",
                std::bind(&SlamwareRosSdkServer::srvCbSyncSetStcm_, this, std::placeholders::_1, std::placeholders::_2));

            relocalization_request_srv_ = this->create_service<slamware_ros_sdk::srv::RelocalizationRequest>(
                "/slamware_ros_sdk_server_node/relocalization",
                std::bind(&SlamwareRosSdkServer::srvCbRelocalizationRequest_, this, std::placeholders::_1, std::placeholders::_2));
        }
        return true;
    }

    void SlamwareRosSdkServer::cleanup_()
    {
        if (isRunning_())
            requestStop();
        waitUntilStopped();

        disconnectAuroraSdk_();

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
        RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "SlamwareRosSdkServer, work thread begin.");

        while (shouldContinueRunning_()
            && rclcpp::ok() // ros::ok()
            )
        {
            if(!auroraSdkConnected_.load())
            {
                loopTryConnectToAuroraSdk_();
                if (!auroraSdkConnected_.load())
                    continue;
            }

            try
            {
                loopWork_();
            }
            catch (const std::exception& excp)
            {
                RCLCPP_FATAL(rclcpp::get_logger("slamware ros sdk server"), "loopWork_(), exception: %s.", excp.what());
            }
            catch (...)
            {
                RCLCPP_FATAL(rclcpp::get_logger("slamware ros sdk server"), "loopWork_(), unknown exception.");
            }
            
            disconnectAuroraSdk_();

            if (shouldContinueRunning_())
            {
                const std::uint32_t maxSleepMs = (1000u * 3u);
                RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "wait %u ms to reconnect and restart work loop.", maxSleepMs);
                roughSleepWait_(maxSleepMs, 100U);
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "SlamwareRosSdkServer, work thread end.");
        state_.store(ServerStateStopped);
    }

    void SlamwareRosSdkServer::roughSleepWait_(std::uint32_t maxSleepMs, std::uint32_t onceSleepMs)
    {
        const auto durOnceSleep = std::chrono::milliseconds(onceSleepMs);
        auto tpNow = std::chrono::steady_clock::now();
        const auto maxSleepTimepoint = tpNow + std::chrono::milliseconds(maxSleepMs);
        while (shouldContinueRunning_()
            && tpNow < maxSleepTimepoint
            )
        {
            std::this_thread::sleep_for(durOnceSleep);
            tpNow = std::chrono::steady_clock::now();
        }
    }
   
    bool SlamwareRosSdkServer::reinitWorkLoop_()
    {
        const std::uint32_t cntWorkers = static_cast<std::uint32_t>(serverWorkers_.size());

        RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "reset all %u workers on work loop begin.", cntWorkers);
        for (auto it = serverWorkers_.begin(), itEnd = serverWorkers_.end(); itEnd != it; ++it)
        {
            const auto& svrWk = (*it);
            const auto wkName = svrWk->getWorkerName();
            try
            {
                svrWk->resetOnWorkLoopBegin();
            }
            catch (const std::exception& excp)
            {
                RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "worker: %s, resetOnWorkLoopBegin(), exception: %s.", wkName.c_str(), excp.what());
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
                const auto& svrWk = (*it);
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
                            RCLCPP_WARN(rclcpp::get_logger("slamware ros sdk server"), "failed to init work loop, woker: %s.", wkName.c_str());
                    }
                }
                catch (const std::exception& excp)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "worker: %s, reinitWorkLoop(), exception: %s.", wkName.c_str(), excp.what());
                }
                catch (...)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "worker: %s, reinitWorkLoop(), unknown exception.", wkName.c_str());
                }

            }
            // check if all workers are ok.            
            if (cntWorkers == cntOk)
            {
                return true;
            }
            else if (t < maxLoopTryCnt)
            {
                RCLCPP_WARN(rclcpp::get_logger("slamware ros sdk server"), "(%u / %u) cntWorkers: %u, cntOk: %u, wait %u ms to retry.", t, maxLoopTryCnt, cntWorkers, cntOk, maxSleepMs);
                roughSleepWait_(maxSleepMs, 100U);
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("slamware ros sdk server"), "(%u / %u) cntWorkers: %u, cntOk: %u.", t, maxLoopTryCnt, cntWorkers, cntOk);
            }
        }
        return false;
    }

    void SlamwareRosSdkServer::loopWork_()
    {
        if (reinitWorkLoop_())
        {
            RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "successed to reinit all workers on work loop begin.");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "failed or cancelled to reinit work loop.");
            return;
        }

        while (shouldContinueRunning_())
        {
            std::chrono::steady_clock::time_point minNextTriggerTimepoint = std::chrono::steady_clock::now() + std::chrono::milliseconds(100U);

            for (auto it = serverWorkers_.begin(), itEnd = serverWorkers_.end(); itEnd != it; ++it)
            {
                const auto& svrWk = (*it);
                const auto wkName = svrWk->getWorkerName();
                bool shouldReconnect = false;
                try
                {
                    svrWk->checkToPerform();
                }
                catch (const std::exception& excp)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "worker name: %s, exception: %s.", wkName.c_str(), excp.what());
                }
                catch (...)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "worker name: %s, unknown exception.", wkName.c_str());
                }

                if (shouldReconnect)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "it should reconnect to slamware.");
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
    void SlamwareRosSdkServer::msgCbSyncMap_(const slamware_ros_sdk::msg::SyncMapRequest::SharedPtr /*msg*/)
    {
        RCLCPP_INFO(this->get_logger(), "Received sync map request");
        requestSyncMap();
    }

    void SlamwareRosSdkServer::msgCbClearMap_(const slamware_ros_sdk::msg::ClearMapRequest::SharedPtr /*msg*/)
    {
        RCLCPP_INFO(this->get_logger(), "Received clear map request");
        auto aurora = safeGetAuroraSdk();
        aurora->controller.requireMapReset();
    }

    void SlamwareRosSdkServer::msgCbSetMapUpdate_(const slamware_ros_sdk::msg::SetMapUpdateRequest::SharedPtr /*msg*/)
    {
        RCLCPP_INFO(this->get_logger(), "Received set map update request");
        auto aurora = safeGetAuroraSdk();
        aurora->controller.requireMappingMode();
    }

    void SlamwareRosSdkServer::msgCbSetMapLocalization_(const slamware_ros_sdk::msg::SetMapLocalizationRequest::SharedPtr /*msg*/)
    {
        RCLCPP_INFO(this->get_logger(), "Received set map localization request");
        auto aurora = safeGetAuroraSdk();
        aurora->controller.requirePureLocalizationMode();
    }

    void SlamwareRosSdkServer::msgCbRelocalizationCancel_(const slamware_ros_sdk::msg::RelocalizationCancelRequest::SharedPtr /*msg*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Relocalization cancel requested");

        auto aurora = safeGetAuroraSdk();
        if (!aurora->controller.cancelRelocalization()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to cancel relocalization");
            return;
        }
        cancel_requested_.store(true);
        relocalization_active_.store(false);
    }

    //////////////////////////////////////////////////////////////////////////
    bool SlamwareRosSdkServer::srvCbSyncGetStcm_(slamware_ros_sdk::srv::SyncGetStcm::Request::SharedPtr req, slamware_ros_sdk::srv::SyncGetStcm::Response::SharedPtr resp)
    {
        const char *mapfile = req->mapfile.c_str();     
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
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to start map storage session");
            resp->success = false;
            resp->message = "Failed to start map storage session";
            return false;
        }

        while (resultFuture.wait_for(std::chrono::milliseconds(100)) == std::future_status::timeout)
        {
            slamtec_aurora_sdk_mapstorage_session_status_t status;
            if (!aurora->mapManager.querySessionStatus(status))
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to query storage status");
                resp->success = false;
                resp->message = "Failed to query storage status";
                return false;
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Map download progress: %f", status.progress);
            std::this_thread::yield();
        }

        int result = resultFuture.get();
        if (result == 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to save map");
            resp->success = false;
            resp->message = "Failed to save map";
            return false;
        }

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Map saved successfully");
        resp->success = true;
        resp->message = "Map saved successfully";
        return true;
    }

    bool SlamwareRosSdkServer::srvCbSyncSetStcm_(slamware_ros_sdk::srv::SyncSetStcm::Request::SharedPtr req, slamware_ros_sdk::srv::SyncSetStcm::Response::SharedPtr resp)
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
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting map upload %s", req->mapfile.c_str());
            if (!aurora->mapManager.startUploadSession(req->mapfile.c_str(), resultCallback, &resultPromise))
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to start map upload session");
                resp->success = false;
                resp->message = "Failed to start map upload session";
                return false;
            }

            while (resultFuture.wait_for(std::chrono::milliseconds(100)) == std::future_status::timeout)
            {
                slamtec_aurora_sdk_mapstorage_session_status_t status;
                if (!aurora->mapManager.querySessionStatus(status))
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to query upload status");
                    resp->success = false;
                    resp->message = "Failed to query upload status";
                    return false;
                }

                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Map upload progress: %f", status.progress);
                std::this_thread::yield();
            }

            int result = resultFuture.get();
            if (result == 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to upload map");
                resp->success = false;
                resp->message = "Failed to upload map";
                return false;
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Map uploaded successfully");
            resp->success = true;
            resp->message = "Map uploaded successfully";
        }
        requestSyncMap();
        return true;
    }

    //////////////////////////////////////////////////////////////////////////
    bool SlamwareRosSdkServer::srvCbRelocalizationRequest_(slamware_ros_sdk::srv::RelocalizationRequest::Request::SharedPtr req,
                                                           slamware_ros_sdk::srv::RelocalizationRequest::Response::SharedPtr resp)
    {
        if(relocalization_active_.load())
        {
            RCLCPP_ERROR(this->get_logger(), "Relocalization already in progress");
            resp->success = false;
            return false;
        }
        // Handle relocalization request using Aurora SDK
        auto aurora = safeGetAuroraSdk();
        aurora->controller.requireRelocalization();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Relocalization requested");
        relocalization_active_.store(true);
        cancel_requested_.store(false);


        relocalization_future_ = std::async(std::launch::async, [this]() {
            while (true)
            {
                checkRelocalizationStatus();
                if (cancel_requested_.load() || !relocalization_active_.load())
                {
                    return;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });

        resp->success = true;
        return true;
    }

    void SlamwareRosSdkServer::checkRelocalizationStatus()
    {
        slamtec_aurora_sdk_relocalization_status_t result;
        auto aurora = safeGetAuroraSdk();
        aurora->dataProvider.peekRelocalizationStatus(result);
        if (result.status == 2)
        {
            RCLCPP_INFO(this->get_logger(), "Relocalization succeeded");
            relocalization_active_.store(false);
        }
        else if (result.status == 3)
        {
            RCLCPP_ERROR(this->get_logger(), "Relocalization failed");
            relocalization_active_.store(false);
        }
        else if (result.status == 4)
        {
            RCLCPP_INFO(this->get_logger(), "Relocalization canceled by system");
            relocalization_active_.store(false);
        }
    }
}
