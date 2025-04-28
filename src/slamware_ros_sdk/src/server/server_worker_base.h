
#pragma once

#include "server_params.h"
#include "server_work_data.h"

#include <tf2_ros/transform_broadcaster.h>

#include <chrono>

#include <memory>

namespace slamware_ros_sdk {

    class SlamwareRosSdkServer;

    class ServerWorkerBase
    {
    public:
        typedef std::chrono::steady_clock         clock_t;
        typedef clock_t::time_point                 time_point_t;

    public:
        ServerWorkerBase(SlamwareRosSdkServer* pRosSdkServer
            , const std::string& wkName
            , const std::chrono::milliseconds& triggerInterval
            );
        virtual ~ServerWorkerBase();

        const std::string& getWorkerName() const { return workerName_; }

        time_point_t getNextTimepointToTrigger() const { return nextTimepointToTrigger_; }
        
        bool isWorkLoopInitOk() const { return isWorkLoopInitOk_; }
        virtual void resetOnWorkLoopBegin();
        virtual bool reinitWorkLoop();

        virtual void checkToPerform();

    protected:
        virtual void doPerform() = 0;

    protected:
        SlamwareRosSdkServer* rosSdkServer() const { return rosSdkServer_; }

        std::shared_ptr<rclcpp::Node> rosNodeHandle() const;
        const ServerParams& serverParams() const;
        std::unique_ptr<tf2_ros::TransformBroadcaster>& tfBroadcaster() const;

        ServerWorkData_ConstPtr workData() const;
        ServerWorkData_Ptr mutableWorkData();

    private:
        SlamwareRosSdkServer* rosSdkServer_;
        std::string workerName_;

    protected:
        bool isWorkLoopInitOk_;
        std::chrono::milliseconds triggerInterval_;
        time_point_t nextTimepointToTrigger_;
    };

    typedef std::shared_ptr<ServerWorkerBase>         ServerWorkerBase_Ptr;
    
}
