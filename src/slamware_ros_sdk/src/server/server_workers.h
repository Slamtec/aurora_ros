#pragma once

#include "server_worker_base.h"

#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "std_msgs/String.h"
#include <aurora_pubsdk_inc.h>
#include <slamware_ros_sdk/SystemStatus.h>
#include <slamware_ros_sdk/RelocalizationStatus.h>
#include <aurora_pubsdk_inc.h>
#include <optional>
#include <chrono>
#include <opencv2/opencv.hpp>

using namespace rp::standalone::aurora;

namespace slamware_ros_sdk
{

    //////////////////////////////////////////////////////////////////////////

    class ServerRobotDeviceInfoWorker : public ServerWorkerBase
    {
    public:
        typedef ServerWorkerBase super_t;

    public:
        ServerRobotDeviceInfoWorker(SlamwareRosSdkServer *pRosSdkServer, const std::string &wkName, const std::chrono::milliseconds &triggerInterval);
        virtual ~ServerRobotDeviceInfoWorker();

        // virtual bool reinitWorkLoop();

    protected:
        virtual void doPerform();

    private:
        ros::Publisher pubRobotDeviceInfo_;
    };

    //////////////////////////////////////////////////////////////////////////

    class ServerOdometryWorker : public ServerWorkerBase
    {
    public:
        typedef ServerWorkerBase super_t;

    public:
        ServerOdometryWorker(SlamwareRosSdkServer *pRosSdkServer, const std::string &wkName, const std::chrono::milliseconds &triggerInterval);
        virtual ~ServerOdometryWorker();

        virtual bool reinitWorkLoop();

    protected:
        virtual void doPerform();
        double getYawFromQuaternion(const geometry_msgs::Quaternion &quat);

    private:
        ros::Publisher pubOdometry_;
        geometry_msgs::PoseStamped lastPoseStamped_;
        ros::Time lastPoseTime_;
        bool firstPoseReceived_;
    };

    //////////////////////////////////////////////////////////////////////////

    class ServerRobotPoseWorker : public ServerWorkerBase
    {
    public:
        ServerRobotPoseWorker(SlamwareRosSdkServer *pRosSdkServer, const std::string &wkName, const std::chrono::milliseconds &triggerInterval);
        virtual ~ServerRobotPoseWorker();

    protected:
        virtual void doPerform();

    private:
        ros::Publisher pubRobotPose_;
        uint64_t lastTimestamp_;
    };

    //////////////////////////////////////////////////////////////////////////

    class ServerExploreMapUpdateWorker : public ServerWorkerBase
    {
    public:
        typedef ServerWorkerBase super_t;

    public:
        ServerExploreMapUpdateWorker(SlamwareRosSdkServer *pRosSdkServer, const std::string &wkName, const std::chrono::milliseconds &triggerInterval);
        virtual ~ServerExploreMapUpdateWorker();

        virtual bool reinitWorkLoop();

    protected:
        virtual void doPerform();

    private:
        void requestReinitMap_();
        bool checkToReinitMap_(rp::standalone::aurora::RemoteSDK *sdk, const ServerWorkData_Ptr &wkDat);

        bool checkRecvResolution_(float recvResolution, const ServerWorkData_Ptr &wkDat);

        bool updateMapInCellIdxRect_(const rp::standalone::aurora::OccupancyGridMap2DRef &prevMap, const utils::RectangleF &reqRect, const ServerWorkData_Ptr &wkDat);

        bool syncWholeMap_(const ServerParams &srvParams, rp::standalone::aurora::RemoteSDK *sdk, const ServerWorkData_Ptr &wkDat);

        bool updateMapNearRobot_(const ServerParams &srvParams, rp::standalone::aurora::RemoteSDK *sdk, const ServerWorkData_Ptr &wkDat);

    private:
        bool shouldReinitMap_;
        bool hasSyncedWholeMap_;
        time_point_t lastMapUpdateTime_;
        ros::Time mapInitTime_;
    };

    //////////////////////////////////////////////////////////////////////////

    class ServerExploreMapPublishWorker : public ServerWorkerBase
    {
    public:
        ServerExploreMapPublishWorker(SlamwareRosSdkServer *pRosSdkServer, const std::string &wkName, const std::chrono::milliseconds &triggerInterval);
        virtual ~ServerExploreMapPublishWorker();

    protected:
        virtual void doPerform();

    private:
        ros::Publisher pubMapDat_;
        ros::Publisher pubMapInfo_;
    };

    //////////////////////////////////////////////////////////////////////////

    class ServerLaserScanWorker : public ServerWorkerBase
    {
    public:
        ServerLaserScanWorker(SlamwareRosSdkServer *pRosSdkServer, const std::string &wkName, const std::chrono::milliseconds &triggerInterval);
        virtual ~ServerLaserScanWorker();

    protected:
        virtual void doPerform();

    private:
        void fillRangeMinMaxInMsg_(const std::vector<slamtec_aurora_sdk_lidar_scan_point_t> &laserPoints, sensor_msgs::LaserScan &msgScan) const;

        float calcAngleInNegativePiToPi_(float angle) const;

        std::uint32_t calcCompensateDestIndexBySrcAngle_(float srcAngle, bool isAnglesReverse) const;
        bool isSrcAngleMoreCloseThanOldSrcAngle_(float srcAngle, float destAngle, float oldSrcAngle) const;
        void compensateAndfillRangesInMsg_(const std::vector<slamtec_aurora_sdk_lidar_scan_point_t> &laserPoints, bool isClockwise, bool isLaserDataReverse, sensor_msgs::LaserScan &msgScan) const;
        void compensateAndfillRangeInMsg_(const slamtec_aurora_sdk_lidar_scan_point_t &laserPoint, bool isClockwise, sensor_msgs::LaserScan &msgScan, std::vector<float> &tmpSrcAngles) const;
        void fillOriginalRangesInMsg_(const std::vector<slamtec_aurora_sdk_lidar_scan_point_t> &laserPoints, bool isLaserDataReverse, sensor_msgs::LaserScan &msgScan) const;
        void fillOriginalRangeInMsg_(const slamtec_aurora_sdk_lidar_scan_point_t &laserPoint, int index, sensor_msgs::LaserScan &msgScan) const;

    private:
        std::uint32_t compensatedAngleCnt_;
        float absAngleIncrement_;

        ros::Publisher pubLaserScan_;
        std::uint64_t latestLidarStartTimestamp_;
    };

    //////////////////////////////////////////////////////////////////////////

    class ServerImuRawDataWorker : public ServerWorkerBase
    {
    public:
        typedef ServerWorkerBase super_t;

    public:
        ServerImuRawDataWorker(SlamwareRosSdkServer *pRosSdkServer, const std::string &wkName, const std::chrono::milliseconds &triggerInterval);
        virtual ~ServerImuRawDataWorker();

        virtual bool reinitWorkLoop();

    protected:
        virtual void doPerform();

    private:
        uint64_t lastTimestamp_;
        ros::Publisher pubImuRawData_;
        _Float32 acc_scale_;
        _Float32 gyro_scale_;
    };

    //////////////////////////////////////////////////////////////////////////
    class RosConnectWorker : public ServerWorkerBase
    {
    public:
        typedef ServerWorkerBase super_t;

    public:
        RosConnectWorker(SlamwareRosSdkServer *pRosSdkServer, const std::string &wkName, const std::chrono::milliseconds &triggerInterval);
        virtual ~RosConnectWorker();

        virtual bool reinitWorkLoop();

    protected:
        virtual void doPerform();

    private:
        ros::Publisher pubRosConnect_;
    };

    class ServerSystemStatusWorker : public ServerWorkerBase
    {
    public:
        typedef ServerWorkerBase super_t;

    public:
        ServerSystemStatusWorker(SlamwareRosSdkServer *pRosSdkServer, const std::string &wkName, const std::chrono::milliseconds &triggerInterval);
        virtual ~ServerSystemStatusWorker();

        virtual bool reinitWorkLoop();

    protected:
        virtual void doPerform();

    private:
        ros::Publisher pubSystemStatus_;
        ros::Publisher pubRelocalizaitonStatus_;
        std::optional<slamtec_aurora_sdk_device_status_t> lastDeviceStatus_;
        std::optional<slamtec_aurora_sdk_relocalization_status_type_t> lastRelocalizationStatus_;
    };

    //////////////////////////////////////////////////////////////////////////

    class ServerStereoImageWorker : public ServerWorkerBase
    {
    public:
        typedef ServerWorkerBase super_t;

    public:
        ServerStereoImageWorker(SlamwareRosSdkServer *pRosSdkServer, const std::string &wkName, const std::chrono::milliseconds &triggerInterval);
        virtual ~ServerStereoImageWorker();

        virtual bool reinitWorkLoop();

    protected:
        virtual void doPerform();

    private:
        ros::Publisher pubLeftImage_;
        ros::Publisher pubRightImage_;

        ros::Publisher pubStereoKeyPoints_;
        uint64_t lastTimestamp_;
    };

    //////////////////////////////////////////////////////////////////////////

    class ServerEnhancedImagingWorker: public ServerWorkerBase
    {
    public:
        typedef ServerWorkerBase super_t;
        
    public:
        ServerEnhancedImagingWorker(SlamwareRosSdkServer* pRosSdkServer
            , const std::string& wkName
            , const std::chrono::milliseconds& triggerInterval
            );
        virtual ~ServerEnhancedImagingWorker();

        virtual bool reinitWorkLoop();

    protected:
        virtual void doPerform();

    private:
        // Helper functions
        void processDepthCamera(const std_msgs::Header& header);
        void processSemanticSegmentation(const std_msgs::Header& header);
        cv::Mat createCameraOverlay(const cv::Mat& cameraImage, const cv::Mat& colorizedSegMap);
        cv::Mat colorizeSegmentationMap(const cv::Mat& segMap);
        void generateClassColors(int numClasses);
        
        // Depth camera publishers
        ros::Publisher pubDepthImage_;
        ros::Publisher pubDepthColorized_;
        ros::Publisher pubDensePointCloud_;  // colored PointCloud2 from POINT3D frame
        
        // Semantic segmentation publishers
        ros::Publisher pubSemanticSegmentation_;

        std::vector<cv::Vec3b> classColors_;
        
        // Status flags
        bool depthCameraSupported_;
        bool semanticSegmentationSupported_;
        bool isInitialized_;

        //store the latest timestamp of received image
        uint64_t depth_lastTimestamp_;
        uint64_t segmentation_lastTimestamp_;
    };

    //////////////////////////////////////////////////////////////////////////

    class ServerPointCloudWorker : public ServerWorkerBase
    {
    public:
        typedef ServerWorkerBase super_t;

    public:
        ServerPointCloudWorker(SlamwareRosSdkServer *pRosSdkServer, const std::string &wkName, const std::chrono::milliseconds &triggerInterval);
        virtual ~ServerPointCloudWorker();

        virtual bool reinitWorkLoop();

    protected:
        virtual void doPerform();

    private:
        ros::Publisher pubPointCloud_;
    };

    //////////////////////////////////////////////////////////////////////////
    class ServerRawImageWorker : public ServerWorkerBase
    {
    public:
        typedef ServerWorkerBase super_t;

    public:
        ServerRawImageWorker(SlamwareRosSdkServer *pRosSdkServer, const std::string &wkName, const std::chrono::milliseconds &triggerInterval);
        virtual ~ServerRawImageWorker();

        virtual bool reinitWorkLoop();

    protected:
        virtual void doPerform();

    private:
        ros::Publisher pubLeftImage_;
        ros::Publisher pubRightImage_;
        uint64_t lastTimestamp_;
    };

}