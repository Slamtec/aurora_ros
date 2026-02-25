/**
 * @file server_workers.cpp
 * @brief Implementation of various server workers for the Slamware ROS SDK.
 */

#include "server_workers.h"
#include "slamware_ros_sdk_server.h"

#include <assert.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <random>
#include <cmath>

namespace slamware_ros_sdk
{

    //////////////////////////////////////////////////////////////////////////
    ServerRobotDeviceInfoWorker::ServerRobotDeviceInfoWorker(
        SlamwareRosSdkServer *pRosSdkServer,
        const std::string &wkName,
        const std::chrono::milliseconds &triggerInterval)
        : super_t(pRosSdkServer, wkName, triggerInterval)
    {
        auto &nhRos = rosNodeHandle();
    }

    ServerRobotDeviceInfoWorker::~ServerRobotDeviceInfoWorker()
    {
        //
    }

    void ServerRobotDeviceInfoWorker::doPerform()
    {
        // do nothing
    }

    //////////////////////////////////////////////////////////////////////////

    ServerRobotPoseWorker::ServerRobotPoseWorker(
        SlamwareRosSdkServer *pRosSdkServer,
        const std::string &wkName,
        const std::chrono::milliseconds &triggerInterval)
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval),lastTimestamp_(0)
    {
        const auto &srvParams = serverParams();
        auto &nhRos = rosNodeHandle();
        pubRobotPose_ = nhRos.advertise<geometry_msgs::PoseStamped>(srvParams.robot_pose_topic, 10);
    }

    ServerRobotPoseWorker::~ServerRobotPoseWorker()
    {
        //
    }

    void ServerRobotPoseWorker::doPerform()
    {
        const auto &srvParams = serverParams();
        auto &tfBrdcst = tfBroadcaster();
        auto wkDat = mutableWorkData();

        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        if (!auroraSDK)
        {
            ROS_INFO("doPerform(), auroraSDK is null.");
            return;
        }

        slamtec_aurora_sdk_pose_se3_t pose;
        // auroraSDK->dataProvider.getCurrentPoseSE3(pose);
        uint64_t pose_timestamp;
        if(auroraSDK->dataProvider.getCurrentPoseSE3WithTimestamp(pose,pose_timestamp))
        {
            if(pose_timestamp>lastTimestamp_)
            {
                lastTimestamp_ = pose_timestamp;
            }
            else
            {
                return;
            }
        }
        else
        {
            return;
        }
        // current pose from opencv axis to ros1 axis
        wkDat->robotPose.header.stamp = ros::Time::now();
        wkDat->robotPose.pose.position.x = pose.translation.x;
        wkDat->robotPose.pose.position.y = pose.translation.y;
        wkDat->robotPose.pose.position.z = pose.translation.z;

        wkDat->robotPose.pose.orientation.x = pose.quaternion.x;
        wkDat->robotPose.pose.orientation.y = pose.quaternion.y;
        wkDat->robotPose.pose.orientation.z = pose.quaternion.z;
        wkDat->robotPose.pose.orientation.w = pose.quaternion.w;

        // ROS_INFO("vslam_pose - Translation: x=%f, y=%f, z=%f, Rotation: x=%f, y=%f, z=%f, w=%f",
        //  vslam_pose.getOrigin().x(), vslam_pose.getOrigin().y(), vslam_pose.getOrigin().z(),
        //  vslam_pose.getRotation().x(), vslam_pose.getRotation().y(),
        //  vslam_pose.getRotation().z(), vslam_pose.getRotation().w());

        // publish robot_pose transform
        // geometry_msgs::TransformStamped t;
        // t.header.stamp = ros::Time::now();
        // t.header.frame_id = srvParams.map_frame;
        // t.child_frame_id = srvParams.robot_frame;

        // t.transform.translation.x = wkDat->robotPose.pose.position.x;
        // t.transform.translation.y = wkDat->robotPose.pose.position.y;
        // t.transform.translation.z = wkDat->robotPose.pose.position.z;
        // t.transform.rotation = wkDat->robotPose.pose.orientation;
        // tfBrdcst.sendTransform(t);

        geometry_msgs::PoseStamped ros_pose;
        ros_pose.header.frame_id = srvParams.map_frame;
        ros_pose.header.stamp = wkDat->robotPose.header.stamp;
        ros_pose.pose = wkDat->robotPose.pose;
        pubRobotPose_.publish(ros_pose);
    }

    //////////////////////////////////////////////////////////////////////////
    ServerOdometryWorker::ServerOdometryWorker(
        SlamwareRosSdkServer *pRosSdkServer,
        const std::string &wkName,
        const std::chrono::milliseconds &triggerInterval)
        : super_t(pRosSdkServer, wkName, triggerInterval),
          lastPoseStamped_(geometry_msgs::PoseStamped()),
          firstPoseReceived_(false)
    {

        const auto &srvParams = serverParams();
        auto nhRos = rosNodeHandle();
        pubOdometry_ = nhRos.advertise<nav_msgs::Odometry>(srvParams.odom_topic, 10);
    }

    ServerOdometryWorker::~ServerOdometryWorker()
    {
        //
    }

    bool ServerOdometryWorker::reinitWorkLoop()
    {

        if (!this->super_t::reinitWorkLoop())
            return false;
        firstPoseReceived_ = false;
        isWorkLoopInitOk_ = true;
        return isWorkLoopInitOk_;
    }
    double ServerOdometryWorker::getYawFromQuaternion(const geometry_msgs::Quaternion &quat)
    {
        tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        return tf::getYaw(q);
    }

    void ServerOdometryWorker::doPerform()
    {
        const auto &srvParams = serverParams();
        ros::Time currentTime = ros::Time::now();
        auto wkDat = mutableWorkData();
        const auto &currentPoseStamped = wkDat->robotPose;

        if (!firstPoseReceived_)
        {
            lastPoseStamped_ = currentPoseStamped;
            // lastPoseTime_ = currentTime;
            firstPoseReceived_ = true;
            return;
        }

        float deltaX = currentPoseStamped.pose.position.x - lastPoseStamped_.pose.position.x;
        float deltaY = currentPoseStamped.pose.position.y - lastPoseStamped_.pose.position.y;
        double deltaYaw = getYawFromQuaternion(currentPoseStamped.pose.orientation) -
                          getYawFromQuaternion(lastPoseStamped_.pose.orientation);
        double dt = (currentPoseStamped.header.stamp - lastPoseStamped_.header.stamp).toSec();
        double vx = deltaX / dt;
        double vy = deltaY / dt;
        double vth = deltaYaw / dt;

        nav_msgs::Odometry odom;
        odom.header.stamp = currentPoseStamped.header.stamp;
        odom.header.frame_id = srvParams.odom_frame;
        odom.child_frame_id = srvParams.robot_frame;

        odom.pose.pose = currentPoseStamped.pose;
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;
        pubOdometry_.publish(odom);

        auto &tfBrdcst = tfBroadcaster();
        geometry_msgs::TransformStamped odomTrans;
        odomTrans.header.stamp = currentPoseStamped.header.stamp;
        odomTrans.header.frame_id = srvParams.odom_frame;
        odomTrans.child_frame_id = srvParams.robot_frame;
        odomTrans.transform.translation.x = currentPoseStamped.pose.position.x;
        odomTrans.transform.translation.y = currentPoseStamped.pose.position.y;
        odomTrans.transform.translation.z = currentPoseStamped.pose.position.z;
        odomTrans.transform.rotation = currentPoseStamped.pose.orientation;
        tfBrdcst.sendTransform(odomTrans);

        lastPoseStamped_ = currentPoseStamped;
    }
    //////////////////////////////////////////////////////////////////////////

    ServerExploreMapUpdateWorker::ServerExploreMapUpdateWorker(
        SlamwareRosSdkServer *pRosSdkServer,
        const std::string &wkName,
        const std::chrono::milliseconds &triggerInterval)
        : super_t(pRosSdkServer, wkName, triggerInterval),
          shouldReinitMap_(true),
          hasSyncedWholeMap_(false)
    {
    }

    ServerExploreMapUpdateWorker::~ServerExploreMapUpdateWorker()
    {
        //
    }

    bool ServerExploreMapUpdateWorker::reinitWorkLoop()
    {
        if (!this->super_t::reinitWorkLoop())
            return false;
        isWorkLoopInitOk_ = false;
        hasSyncedWholeMap_ = false;

        requestReinitMap_();

        isWorkLoopInitOk_ = true;
        return isWorkLoopInitOk_;
    }

    void ServerExploreMapUpdateWorker::doPerform()
    {
        const auto &srvParams = serverParams();
        auto wkDat = mutableWorkData();

        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        if (!auroraSDK)
        {
            ROS_ERROR("Failed to get aurora sdk");
            return;
        }

        if (!checkToReinitMap_(auroraSDK, wkDat))
            return;

        if (!hasSyncedWholeMap_)
        {
            ros::Duration dt = ros::Time::now() - mapInitTime_;
            if (dt.toSec() > 10)
            {
                wkDat->syncMapRequested.store(true);
                hasSyncedWholeMap_ = true;
                ROS_INFO("sync the map after preview map generated.");
            }
        }

        if (wkDat->syncMapRequested.load())
        {
            ROS_INFO("try to sync whole explore map.");
            if (syncWholeMap_(srvParams, auroraSDK, wkDat))
            {
                wkDat->syncMapRequested.store(false);
                ROS_INFO("whole explore map synchronized.");
            }
            else
            {
                ROS_WARN("failed to sync whole explore map.");
            }
            lastMapUpdateTime_ = clock_t::now();
            return;
        }
        updateMapNearRobot_(srvParams, auroraSDK, wkDat);
    }

    void ServerExploreMapUpdateWorker::requestReinitMap_()
    {
        shouldReinitMap_ = true;
    }

    bool ServerExploreMapUpdateWorker::checkToReinitMap_(
        rp::standalone::aurora::RemoteSDK *sdk,
        const ServerWorkData_Ptr &wkDat)
    {
        if (!shouldReinitMap_)
            return true;

        ROS_INFO("try to reinit explore map.");
        wkDat->syncMapRequested.store(true);

        sdk->lidar2DMapBuilder.requireRedrawPreviewMap();
        LIDAR2DGridMapGenerationOptions genOption;
        sdk->lidar2DMapBuilder.getPreviewMapGenerationOptions(genOption);
        wkDat->exploreMapHolder.reinit(genOption.resolution);

        ROS_INFO(
            "explore map initialized, resolution: %f, moreCellCntToExtend: %d.",
            genOption.resolution,
            wkDat->exploreMapHolder.getMoreCellCountToExtend());
        shouldReinitMap_ = false;
        mapInitTime_ = ros::Time::now();
        return true;
    }

    bool ServerExploreMapUpdateWorker::checkRecvResolution_(
        float recvResolution, const ServerWorkData_Ptr &wkDat)
    {
        const auto fResolution = wkDat->exploreMapHolder.resolution();
        if (std::abs(fResolution - recvResolution) <
            std::numeric_limits<float>::epsilon())
            return true;

        ROS_ERROR("local resolution: %f, received resolution: %f, request reinit.", fResolution, recvResolution);
        requestReinitMap_();
        return false;
    }

    bool ServerExploreMapUpdateWorker::updateMapInCellIdxRect_(
        const rp::standalone::aurora::OccupancyGridMap2DRef &prevMap,
        const utils::RectangleF &reqRect,
        const ServerWorkData_Ptr &wkDat)
    {
        const auto &fResolution = prevMap.getResolution();

        static const size_t MAX_FETCH_BLOCK_CXCY = 16 * 1024; // 16*16 to get 256 MB
        size_t fetchSize = (size_t)(reqRect.width() * reqRect.height() / fResolution / fResolution);
        if (fetchSize > (MAX_FETCH_BLOCK_CXCY * MAX_FETCH_BLOCK_CXCY))
        {
            // using smaller block to fetch
            float fetchLogicCXCY = MAX_FETCH_BLOCK_CXCY * fResolution;
            for (float y = 0; y < reqRect.height(); y += fetchLogicCXCY)
            {
                for (float x = 0; x < reqRect.width(); x += fetchLogicCXCY)
                {
                    slamtec_aurora_sdk_rect_t fetchRect;
                    fetchRect.x = x + reqRect.x();
                    fetchRect.y = y + reqRect.y();
                    fetchRect.width = fetchLogicCXCY;
                    fetchRect.height = fetchLogicCXCY;
                    std::vector<uint8_t> mapData;
                    slamtec_aurora_sdk_2d_gridmap_fetch_info_t info;
                    prevMap.readCellData(fetchRect, info, mapData, false);
                    wkDat->exploreMapHolder.setMapData(info.real_x, info.real_y, fResolution, info.cell_width, info.cell_height, mapData.data());
                }
            }
        }
        else
        {
            slamtec_aurora_sdk_rect_t fetchRect;
            fetchRect.x = reqRect.x();
            fetchRect.y = reqRect.y();
            fetchRect.width = reqRect.width();
            fetchRect.height = reqRect.height();
            std::vector<uint8_t> mapData;
            slamtec_aurora_sdk_2d_gridmap_fetch_info_t info;
            prevMap.readCellData(fetchRect, info, mapData, false);
            wkDat->exploreMapHolder.setMapData(info.real_x, info.real_y, fResolution, info.cell_width, info.cell_height, mapData.data());
        }

        if (!checkRecvResolution_(fResolution, wkDat))
            return false;

        return true;
    }

    bool ServerExploreMapUpdateWorker::syncWholeMap_(
        const ServerParams &srvParams,
        rp::standalone::aurora::RemoteSDK *sdk,
        const ServerWorkData_Ptr &wkDat)
    {
        auto &prevMap = sdk->lidar2DMapBuilder.getPreviewMap();

        slamtec_aurora_sdk_2dmap_dimension_t mapDimension;
        prevMap.getMapDimension(mapDimension);
        utils::RectangleF knownArea(mapDimension.min_x, mapDimension.min_y,
                                    mapDimension.max_x - mapDimension.min_x,
                                    mapDimension.max_y - mapDimension.min_y);

        const auto knownCellIdxRect = wkDat->exploreMapHolder.calcRoundedCellIdxRect(knownArea);
        ROS_INFO("known area: ((%f, %f), (%f, %f)), cell rect : ((% d, % d), (% d, % d)) ",
                 knownArea.x(), knownArea.y(), knownArea.width(), knownArea.height(),
                 knownCellIdxRect.x(), knownCellIdxRect.y(), knownCellIdxRect.width(), knownCellIdxRect.height());
        if (ServerMapHolder::sfIsCellIdxRectEmpty(knownCellIdxRect))
        {
            ROS_INFO("sync map, knownCellIdxRect is empty.");
            return false;
        }

        wkDat->exploreMapHolder.clear();
        wkDat->exploreMapHolder.reserveByCellIdxRect(knownCellIdxRect);

        return updateMapInCellIdxRect_(prevMap, knownArea, wkDat);
    }

    bool ServerExploreMapUpdateWorker::updateMapNearRobot_(
        const ServerParams &srvParams,
        rp::standalone::aurora::RemoteSDK *sdk,
        const ServerWorkData_Ptr &wkDat)
    {
        float fVal = srvParams.map_update_near_robot_half_wh;
        const float fHalfWH = std::max<float>(2.0f, fVal);
        const float fWH = fHalfWH + fHalfWH;
        const auto nearRobotArea = utils::RectangleF(
            static_cast<float>(wkDat->robotPose.pose.position.x - fHalfWH),
            static_cast<float>(wkDat->robotPose.pose.position.y - fHalfWH),
            fWH, fWH);
        const auto nearRobotIdxRect = wkDat->exploreMapHolder.calcMinBoundingCellIdxRect(nearRobotArea);
        auto &prevMap = sdk->lidar2DMapBuilder.getPreviewMap();
        slamtec_aurora_sdk_2dmap_dimension_t mapDimension;
        prevMap.getMapDimension(mapDimension);
        utils::RectangleF knownArea(mapDimension.min_x, mapDimension.min_y,
                                    mapDimension.max_x - mapDimension.min_x,
                                    mapDimension.max_y - mapDimension.min_y);
        const auto knownCellIdxRect = wkDat->exploreMapHolder.calcRoundedCellIdxRect(knownArea);
#if 0
        ROS_INFO("known area: ((%f, %f), (%f, %f)), cell rect: ((%d, %d), (%d, %d))."
            , knownArea.x(), knownArea.y(), knownArea.width(), knownArea.height()
            , knownCellIdxRect.x(), knownCellIdxRect.y(), knownCellIdxRect.width(), knownCellIdxRect.height()
            );
#endif
        if (ServerMapHolder::sfIsCellIdxRectEmpty(knownCellIdxRect))
        {
            ROS_ERROR("update map, knownCellIdxRect is empty, request sync map.");
            rosSdkServer()->requestSyncMap();
            return false;
        }

        const auto reqIdxRect = ServerMapHolder::sfIntersectionOfCellIdxRect(nearRobotIdxRect, knownCellIdxRect);
        if (ServerMapHolder::sfIsCellIdxRectEmpty(reqIdxRect))
        {
            ROS_WARN("knownCellIdxRect: ((%d, %d), (%d, %d)), nearRobotIdxRect : ((% d, % d), (% d, % d)), intersection is empty.",
                     knownCellIdxRect.x(), knownCellIdxRect.y(),
                     knownCellIdxRect.width(), knownCellIdxRect.height(),
                     nearRobotIdxRect.x(), nearRobotIdxRect.y(),
                     nearRobotIdxRect.width(), nearRobotIdxRect.height());
            return false;
        }
        const auto &reqArea = wkDat->exploreMapHolder.calcAreaByCellIdxRect(reqIdxRect);

        return updateMapInCellIdxRect_(prevMap, reqArea, wkDat);
    }

    //////////////////////////////////////////////////////////////////////////

    ServerExploreMapPublishWorker::ServerExploreMapPublishWorker(
        SlamwareRosSdkServer *pRosSdkServer,
        const std::string &wkName,
        const std::chrono::milliseconds &triggerInterval)
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
        const auto &srvParams = serverParams();
        auto &nhRos = rosNodeHandle();
        pubMapDat_ = nhRos.advertise<nav_msgs::OccupancyGrid>(srvParams.map_topic, 1, true);
        pubMapInfo_ = nhRos.advertise<nav_msgs::MapMetaData>(srvParams.map_info_topic, 1, true);
    }

    ServerExploreMapPublishWorker::~ServerExploreMapPublishWorker()
    {
        //
    }

    void ServerExploreMapPublishWorker::doPerform()
    {
        const auto &srvParams = serverParams();
        auto wkDat = workData();

        if (wkDat->exploreMapHolder.isMapDataEmpty())
        {
            // ROS_WARN("current explore map data is empty.");
            return;
        }

        nav_msgs::GetMap::Response msgMap;
        wkDat->exploreMapHolder.fillRosMapMsg(msgMap);

        // Set the header information on the map
        msgMap.map.header.stamp = ros::Time::now();
        msgMap.map.header.frame_id = srvParams.map_frame;

        pubMapDat_.publish(msgMap.map);
        pubMapInfo_.publish(msgMap.map.info);
    }

    //////////////////////////////////////////////////////////////////////////

    ServerLaserScanWorker::ServerLaserScanWorker(
        SlamwareRosSdkServer *pRosSdkServer,
        const std::string &wkName,
        const std::chrono::milliseconds &triggerInterval)
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval),
          compensatedAngleCnt_(360u),
          absAngleIncrement_(C_FLT_2PI / compensatedAngleCnt_),
          latestLidarStartTimestamp_(0)
    {
        const auto &srvParams = serverParams();
        auto &nhRos = rosNodeHandle();
        pubLaserScan_ = nhRos.advertise<sensor_msgs::LaserScan>(srvParams.scan_topic, 10);
    }

    ServerLaserScanWorker::~ServerLaserScanWorker()
    {
        //
    }

    void ServerLaserScanWorker::doPerform()
    {
        auto aurora = rosSdkServer()->safeGetAuroraSdk();
        if (!aurora)
        {

            ROS_ERROR("Failed to get aurora sdk");
            return;
        }
        const auto &srvParams = serverParams();
        auto &tfBrdcst = tfBroadcaster();
        auto wkDat = workData();

        ros::Time startScanTime(0), endScanTime(0);
        rp::standalone::aurora::SingleLayerLIDARScan scan;
        slamtec_aurora_sdk_pose_se3_t poseSE3;
        startScanTime = ros::Time::now();
        if (!aurora->dataProvider.peekRecentLIDARScanSingleLayer(scan, poseSE3, false))
        {
            return;
        }
        endScanTime = ros::Time::now();

        double dblScanDur = (endScanTime - startScanTime).toSec();

        const auto &points = scan.scanData;

        if (points.size() < 2)
        {
            ROS_ERROR("laser points count: %u, too small, skip publish.", (unsigned int)points.size());
            return;
        }

        sensor_msgs::LaserScan msgScan;
        msgScan.header.stamp = startScanTime;
        msgScan.header.frame_id = srvParams.laser_frame;
        fillRangeMinMaxInMsg_(points, msgScan);

        bool isLaserDataReverse = false;
        if ((points.back().angle < points.front().angle && !srvParams.ladar_data_clockwise) ||
            (points.back().angle > points.front().angle && srvParams.ladar_data_clockwise))
        {
            isLaserDataReverse = true;
        }

        if (srvParams.angle_compensate)
        {
            compensateAndfillRangesInMsg_(points, srvParams.ladar_data_clockwise, isLaserDataReverse, msgScan);
        }
        else
        {
            fillOriginalRangesInMsg_(points, isLaserDataReverse, msgScan);
        }
        assert(2 <= msgScan.ranges.size());
        msgScan.scan_time = dblScanDur;
        msgScan.time_increment = dblScanDur / (double)(msgScan.ranges.size() - 1);

        pubLaserScan_.publish(msgScan);

        geometry_msgs::TransformStamped t;
        t.header.stamp = msgScan.header.stamp;
        t.header.frame_id = srvParams.map_frame;
        t.child_frame_id = srvParams.laser_frame;

        t.transform.translation.x = poseSE3.translation.x;
        t.transform.translation.y =  poseSE3.translation.y;
        t.transform.translation.z =  poseSE3.translation.z;
        t.transform.rotation.x = poseSE3.quaternion.x;
        t.transform.rotation.y = poseSE3.quaternion.y;
        t.transform.rotation.z = poseSE3.quaternion.z;
        t.transform.rotation.w = poseSE3.quaternion.w;
        tfBrdcst.sendTransform(t);
    }

    void ServerLaserScanWorker::fillRangeMinMaxInMsg_(
        const std::vector<slamtec_aurora_sdk_lidar_scan_point_t> &laserPoints,
        sensor_msgs::LaserScan &msgScan) const
    {
        msgScan.range_min = std::numeric_limits<float>::infinity();
        msgScan.range_max = 0.0f;
        for (auto cit = laserPoints.cbegin(), citEnd = laserPoints.cend(); citEnd != cit; ++cit)
        {

            const float tmpDist = cit->dist;

            if (tmpDist < msgScan.range_min)
            {
                msgScan.range_min = std::max<float>(0.0f, tmpDist);
            }

            if (msgScan.range_max < tmpDist)
            {
                msgScan.range_max = tmpDist;
            }
        }
    }

    float ServerLaserScanWorker::calcAngleInNegativePiToPi_(float angle) const
    {
        float fRes = std::fmod(angle + C_FLT_PI, C_FLT_2PI);
        if (fRes < 0.0f)
            fRes += C_FLT_2PI;
        fRes -= C_FLT_PI;

        if (fRes < -C_FLT_PI)
            fRes = -C_FLT_PI;
        else if (C_FLT_PI <= fRes)
            fRes = -C_FLT_PI;
        return fRes;
    }

    std::uint32_t ServerLaserScanWorker::calcCompensateDestIndexBySrcAngle_(
        float srcAngle,
        bool isAnglesReverse) const
    {
        assert(-C_FLT_PI <= srcAngle && srcAngle < C_FLT_PI);

        float fDiff = (isAnglesReverse ? (C_FLT_PI - srcAngle) : (srcAngle + C_FLT_PI));
        fDiff = std::max<float>(0.0f, fDiff);
        fDiff = std::min<float>(fDiff, C_FLT_2PI);

        std::uint32_t destIdx = static_cast<std::uint32_t>(std::round(fDiff / absAngleIncrement_));
        if (compensatedAngleCnt_ <= destIdx)
            destIdx = 0;
        return destIdx;
    }

    bool ServerLaserScanWorker::isSrcAngleMoreCloseThanOldSrcAngle_(
        float srcAngle,
        float destAngle,
        float oldSrcAngle) const
    {
        assert(-C_FLT_PI <= srcAngle && srcAngle < C_FLT_PI);
        assert(-C_FLT_PI <= destAngle && destAngle < C_FLT_PI);
        assert(-C_FLT_PI <= oldSrcAngle && oldSrcAngle < C_FLT_PI);

        float newDiff = std::abs(destAngle - srcAngle);
        if (C_FLT_2PI <= newDiff)
            newDiff = 0.0f;
        else if (C_FLT_PI < newDiff)
            newDiff = C_FLT_2PI - newDiff;

        float oldDiff = std::abs(destAngle - oldSrcAngle);
        if (C_FLT_2PI <= oldDiff)
            oldDiff = 0.0f;
        else if (C_FLT_PI < oldDiff)
            oldDiff = C_FLT_2PI - oldDiff;

        return (newDiff < oldDiff);
    }

    void ServerLaserScanWorker::compensateAndfillRangesInMsg_(
        const std::vector<slamtec_aurora_sdk_lidar_scan_point_t> &laserPoints,
        bool isClockwise, bool isLaserDataReverse,
        sensor_msgs::LaserScan &msgScan) const
    {
        assert(2 <= laserPoints.size());

        msgScan.ranges.clear();
        msgScan.intensities.clear();
        msgScan.ranges.resize(compensatedAngleCnt_, std::numeric_limits<float>::infinity());
        msgScan.intensities.resize(compensatedAngleCnt_, 0.0);

        if (!isClockwise)
        {
            msgScan.angle_min = -C_FLT_PI;
            msgScan.angle_max = C_FLT_PI - absAngleIncrement_;
            msgScan.angle_increment = absAngleIncrement_;
        }
        else
        {
            msgScan.angle_min = C_FLT_PI;
            msgScan.angle_max = (-C_FLT_PI + absAngleIncrement_);
            msgScan.angle_increment = -absAngleIncrement_;
        }

        std::vector<float> tmpSrcAngles(compensatedAngleCnt_);
        if (!isLaserDataReverse)
        {
            for (int i = 0; i < laserPoints.size(); ++i)
            {
                compensateAndfillRangeInMsg_(laserPoints[i], isClockwise, msgScan, tmpSrcAngles);
            }
        }
        else
        {
            for (int i = laserPoints.size() - 1; i >= 0; --i)
            {
                compensateAndfillRangeInMsg_(laserPoints[i], isClockwise, msgScan, tmpSrcAngles);
            }
        }
    }

    void ServerLaserScanWorker::compensateAndfillRangeInMsg_(
        const slamtec_aurora_sdk_lidar_scan_point_t &laserPoint,
        bool isClockwise, sensor_msgs::LaserScan &msgScan,
        std::vector<float> &tmpSrcAngles) const
    {

        const float srcAngle = calcAngleInNegativePiToPi_(laserPoint.angle);
        const std::uint32_t destIdx = calcCompensateDestIndexBySrcAngle_(srcAngle, isClockwise);
        assert(destIdx < compensatedAngleCnt_);
        const float destAngle = calcAngleInNegativePiToPi_(msgScan.angle_min + msgScan.angle_increment * destIdx);

        const bool shouldWrite = (std::isinf(msgScan.ranges[destIdx]) || isSrcAngleMoreCloseThanOldSrcAngle_(srcAngle, destAngle, tmpSrcAngles[destIdx]));
        if (shouldWrite)
        {
            msgScan.ranges[destIdx] = laserPoint.dist;
            msgScan.intensities[destIdx] = laserPoint.quality;
            tmpSrcAngles[destIdx] = srcAngle;
        }
    }

    void ServerLaserScanWorker::fillOriginalRangesInMsg_(
        const std::vector<slamtec_aurora_sdk_lidar_scan_point_t> &laserPoints,
        bool isLaserDataReverse,
        sensor_msgs::LaserScan &msgScan) const
    {
        msgScan.intensities.resize(laserPoints.size());
        msgScan.ranges.resize(laserPoints.size());

        int index = 0;
        if (!isLaserDataReverse)
        {
            for (int i = 0; i < laserPoints.size(); ++i)
            {
                fillOriginalRangeInMsg_(laserPoints[i], index++, msgScan);
            }
            msgScan.angle_min = laserPoints.front().angle;
            msgScan.angle_max = laserPoints.back().angle;
            msgScan.angle_increment = (msgScan.angle_max - msgScan.angle_min) / (double)(msgScan.ranges.size() - 1);
        }
        else
        {
            for (int i = laserPoints.size() - 1; i >= 0; --i)
            {
                fillOriginalRangeInMsg_(laserPoints[i], index++, msgScan);
            }
            msgScan.angle_min = laserPoints.back().angle;
            msgScan.angle_max = laserPoints.front().angle;
            msgScan.angle_increment = (msgScan.angle_max - msgScan.angle_min) / (double)(msgScan.ranges.size() - 1);
        }
    }

    void ServerLaserScanWorker::fillOriginalRangeInMsg_(
        const slamtec_aurora_sdk_lidar_scan_point_t &laserPoint,
        int index,
        sensor_msgs::LaserScan &msgScan) const
    {

        msgScan.ranges[index] = laserPoint.dist;
        msgScan.intensities[index] = laserPoint.quality;
    }

    //////////////////////////////////////////////////////////////////////////

    ServerImuRawDataWorker::ServerImuRawDataWorker(
        SlamwareRosSdkServer *pRosSdkServer,
        const std::string &wkName,
        const std::chrono::milliseconds &triggerInterval)
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval), lastTimestamp_(0)
    {
       
        auto &nhRos = rosNodeHandle();
        const auto &srvParams = serverParams();
        pubImuRawData_ = nhRos.advertise<sensor_msgs::Imu>(srvParams.imu_raw_data_topic, 10);
        //convert g to m/s2
        acc_scale_ = 9.806649344;
        // convert dps to rad/s
        gyro_scale_ = M_PI/180;
    }

    ServerImuRawDataWorker::~ServerImuRawDataWorker()
    {
        //
    }

    bool ServerImuRawDataWorker::reinitWorkLoop()
    {

        return true;
    }

    void ServerImuRawDataWorker::doPerform()
    {
        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        if (!auroraSDK)
        {
            ROS_ERROR("Failed to get aurora sdk");
            return;
        }
        const auto &srvParams = serverParams();
        std::vector<slamtec_aurora_sdk_imu_data_t> imuData;
        if (auroraSDK->dataProvider.peekIMUData(imuData))
        {
            for (auto &imu : imuData)
            {
                if (imu.timestamp_ns <= lastTimestamp_)
                {
                    // ignore the old data that has been fetched before
                    continue;
                }
                lastTimestamp_ = imu.timestamp_ns;
                sensor_msgs::Imu imuRawDataRos;
                imuRawDataRos.linear_acceleration.x = imu.acc[0]*acc_scale_;
                imuRawDataRos.linear_acceleration.y = imu.acc[1]*acc_scale_;
                imuRawDataRos.linear_acceleration.z = imu.acc[2]*acc_scale_;
                imuRawDataRos.angular_velocity.x = imu.gyro[0]*gyro_scale_;
                imuRawDataRos.angular_velocity.y = imu.gyro[1]*gyro_scale_;
                imuRawDataRos.angular_velocity.z = imu.gyro[2]*gyro_scale_;
                imuRawDataRos.header.stamp = ros::Time::now();
                imuRawDataRos.header.frame_id = srvParams.imu_frame;
                pubImuRawData_.publish(imuRawDataRos);
            }
        }
    }

    //////////////////////////////////////////////////////////////////////////

    RosConnectWorker::RosConnectWorker(
        SlamwareRosSdkServer *pRosSdkServer,
        const std::string &wkName,
        const std::chrono::milliseconds &triggerInterval)
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
        auto &nhRos = rosNodeHandle();
        pubRosConnect_ = nhRos.advertise<std_msgs::String>("state", 1);
    }

    RosConnectWorker::~RosConnectWorker()
    {
        //
    }

    bool RosConnectWorker::reinitWorkLoop()
    {
        if (!this->super_t::reinitWorkLoop())
            return false;

        isWorkLoopInitOk_ = true;
        return isWorkLoopInitOk_;
    }

    void RosConnectWorker::doPerform()
    {
        auto connectStatus = std_msgs::String();
        connectStatus.data = "connected";

        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        if (!auroraSDK)
        {
            connectStatus.data = "disconnected";
        }
        pubRosConnect_.publish(connectStatus);
    }

    ServerSystemStatusWorker::ServerSystemStatusWorker(
        SlamwareRosSdkServer *pRosSdkServer,
        const std::string &wkName,
        const std::chrono::milliseconds &triggerInterval)
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval),
          lastRelocalizationStatus_(std::nullopt)
    {
        const auto &srvParams = serverParams();
        auto &nhRos = rosNodeHandle();
        pubSystemStatus_ = nhRos.advertise<slamware_ros_sdk::SystemStatus>(srvParams.system_status_topic_name, 1);
        pubRelocalizaitonStatus_ = nhRos.advertise<slamware_ros_sdk::RelocalizationStatus>(srvParams.relocalization_status_topic_name, 1);
    }

    ServerSystemStatusWorker::~ServerSystemStatusWorker()
    {
        //
    }

    bool ServerSystemStatusWorker::reinitWorkLoop()
    {
        // Reinitialize the work loop if necessary
        return true;
    }

    void ServerSystemStatusWorker::doPerform()
    {
        auto aurora = rosSdkServer()->safeGetAuroraSdk();
        if (!aurora)
        {
            ROS_ERROR("Failed to get aurora sdk");
            return;
        }

        slamtec_aurora_sdk_device_status_desc_t result;
        slamware_ros_sdk::SystemStatus resp;
        if (aurora->dataProvider.peekVSLAMSystemStatus(result))
        {
            // get current time in ns
            resp.timestamp_ns = ros::Time::now().toNSec();
            switch (result.status)
            {
            case SLAMTEC_AURORA_SDK_DEVICE_INITED:
            {
                if (lastDeviceStatus_.has_value() && lastDeviceStatus_.value() == SLAMTEC_AURORA_SDK_DEVICE_INITED)
                {
                    resp.status = "DeviceRunning";
                }
                else
                    resp.status = "DeviceInited";
                break;
            }
            case SLAMTEC_AURORA_SDK_DEVICE_INIT_FAILED:
                resp.status = "DeviceInitFailed";
                break;
            case SLAMTEC_AURORA_SDK_DEVICE_LOOP_CLOSURE:
                resp.status = "DeviceLoopClosure";
                break;
            case SLAMTEC_AURORA_SDK_DEVICE_OPTIMIZATION_COMPLETED:
            {
                if (lastDeviceStatus_.has_value() && lastDeviceStatus_.value() == SLAMTEC_AURORA_SDK_DEVICE_OPTIMIZATION_COMPLETED)
                {
                    resp.status = "DeviceRunning";
                }
                else
                    resp.status = "DeviceOptimizationCompleted";
                break;
            }
            case SLAMTEC_AURORA_SDK_DEVICE_TRACKING_LOST:
                resp.status = "DeviceTrackingLost";
                break;
            case SLAMTEC_AURORA_SDK_DEVICE_TRACKING_RECOVERED:
            {
                if (lastDeviceStatus_.has_value() && lastDeviceStatus_.value() == SLAMTEC_AURORA_SDK_DEVICE_TRACKING_RECOVERED)
                {
                    resp.status = "DeviceRunning";
                }
                else
                    resp.status = "DeviceTrackingRecovered";
                break;
            }
            case SLAMTEC_AURORA_SDK_DEVICE_MAP_CLEARED:
                resp.status = "DeviceMapCleared";
                break;
            case SLAMTEC_AURORA_SDK_DEVICE_MAP_LOADING_STARTED:
                resp.status = "DeviceMapLoadingStarted";
                break;
            case SLAMTEC_AURORA_SDK_DEVICE_MAP_SAVING_STARTED:
                resp.status = "DeviceMapSavingStarted";
                break;
            case SLAMTEC_AURORA_SDK_DEVICE_MAP_LOADING_COMPLETED:
            {
                if (lastDeviceStatus_.has_value() && lastDeviceStatus_.value() == SLAMTEC_AURORA_SDK_DEVICE_MAP_LOADING_COMPLETED)
                {
                    resp.status = "DeviceRunning";
                }
                else
                    resp.status = "DeviceMapLoadingCompleted";
                break;
            }
            case SLAMTEC_AURORA_SDK_DEVICE_MAP_SAVING_COMPLETED:
            {
                if (lastDeviceStatus_.has_value() && lastDeviceStatus_.value() == SLAMTEC_AURORA_SDK_DEVICE_MAP_SAVING_COMPLETED)
                {
                    resp.status = "DeviceRunning";
                }
                else
                    resp.status = "DeviceMapSavingCompleted";
                break;
            }
            case 13:
            {
                if (lastDeviceStatus_.has_value() && lastDeviceStatus_.value() == 13)
                {
                    resp.status = "DeviceRunning";
                }
                else
                    resp.status = "DeviceRelocSuccess";
                break;
            }
            case 14:
            {
                if (lastDeviceStatus_.has_value() && lastDeviceStatus_.value() == 14)
                {
                    resp.status = "DeviceRunning";
                }
                else
                    resp.status = "DeviceRelocFailed";
                break;
            }
            case 15:
            {
                if (lastDeviceStatus_.has_value() && lastDeviceStatus_.value() == 15)
                {
                    resp.status = "DeviceRunning";
                }
                else
                    resp.status = "DeviceRelocCancelled";
                break;
            }
            case 16:
                resp.status = "DeviceRelocRunning";
                break;
            default:
                return;
            }

            lastDeviceStatus_ = result.status;
            pubSystemStatus_.publish(resp);
        }

        slamtec_aurora_sdk_relocalization_status_t reloc_status;
        slamware_ros_sdk::RelocalizationStatus reloc_resp;
        if (aurora->dataProvider.peekRelocalizationStatus(reloc_status))
        {
            reloc_resp.timestamp_ns = ros::Time::now().toNSec();
            ;
            {
                switch (reloc_status.status)
                {
                case SLAMTEC_AURORA_SDK_RELOCALIZATION_NONE:
                    reloc_resp.status = "RelocalizationNone";
                    break;
                case SLAMTEC_AURORA_SDK_RELOCALIZATION_STARTED:
                    reloc_resp.status = "RelocalizationRunning";
                    break;
                case SLAMTEC_AURORA_SDK_RELOCALIZATION_SUCCEED:
                {
                    if (lastRelocalizationStatus_.has_value() && lastRelocalizationStatus_.value() == SLAMTEC_AURORA_SDK_RELOCALIZATION_SUCCEED)
                    {
                        reloc_resp.status = "RelocalizationNone";
                    }
                    else
                        reloc_resp.status = "RelocalizationSucceed";
                    break;
                }
                case SLAMTEC_AURORA_SDK_RELOCALIZATION_FAILED:
                {
                    if (lastRelocalizationStatus_.has_value() && lastRelocalizationStatus_.value() == SLAMTEC_AURORA_SDK_RELOCALIZATION_FAILED)
                    {
                        reloc_resp.status = "RelocalizationNone";
                    }
                    else
                        reloc_resp.status = "RelocalizationFailed";
                    break;
                }
                case SLAMTEC_AURORA_SDK_RELOCALIZATION_ABORTED:
                {
                    if (lastRelocalizationStatus_.has_value() && lastRelocalizationStatus_.value() == SLAMTEC_AURORA_SDK_RELOCALIZATION_ABORTED)
                    {
                        reloc_resp.status = "RelocalizationNone";
                    }
                    else
                        reloc_resp.status = "RelocalizationCanceled";
                    break;
                }
                default:
                    break;
                }
            }

            lastRelocalizationStatus_ = reloc_status.status;
            pubRelocalizaitonStatus_.publish(reloc_resp);
        }
    }

    ServerStereoImageWorker::ServerStereoImageWorker(
        SlamwareRosSdkServer *pRosSdkServer,
        const std::string &wkName,
        const std::chrono::milliseconds &triggerInterval)
        : super_t(pRosSdkServer, wkName, triggerInterval),lastTimestamp_(0)
    {
        const auto &srvParams = serverParams();
        auto nhRos = rosNodeHandle();
        pubLeftImage_ = nhRos.advertise<sensor_msgs::Image>(srvParams.left_image_raw_topic_name, 1);
        pubRightImage_ = nhRos.advertise<sensor_msgs::Image>(srvParams.right_image_raw_topic_name, 1);
        pubStereoKeyPoints_ = nhRos.advertise<sensor_msgs::Image>(srvParams.stereo_keypoints_topic_name, 1);
    }

    ServerStereoImageWorker::~ServerStereoImageWorker()
    {
    }

    bool ServerStereoImageWorker::reinitWorkLoop()
    {
        // Reinitialize the work loop if necessary
        return true;
    }
    void ServerStereoImageWorker::doPerform()
    {
        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        RemoteTrackingFrameInfo trackingFrame;
        if (auroraSDK->dataProvider.peekTrackingData(trackingFrame))
        {
            if(trackingFrame.trackingInfo.timestamp_ns>lastTimestamp_)
            {
                lastTimestamp_ = trackingFrame.trackingInfo.timestamp_ns;
            }
            else
            {
                return;
            }
        }
       else
       {
            return;
       }
        
        cv::Mat left, right;
        trackingFrame.leftImage.toMat(left);
        trackingFrame.rightImage.toMat(right);

        // Check if images are empty before processing
        if (left.empty() || right.empty()) {
            return;
        }

        // Convert BGR or GRAY to RGB
        if(left.channels()==3)
            cv::cvtColor(left, left, cv::COLOR_BGR2RGB);
        else if(left.channels()==1)
            cv::cvtColor(left, left, cv::COLOR_GRAY2RGB);
        else
            return;
        const auto &srvParams = serverParams();
        // Get left and right images from the Aurora SDK
     
        if(!srvParams.raw_image_on)
        {
            std_msgs::Header header_left;
            cv_bridge::CvImage img_bridge_left;
            header_left.frame_id = srvParams.camera_left;
            header_left.stamp = ros::Time::now();
            img_bridge_left = cv_bridge::CvImage(header_left, sensor_msgs::image_encodings::RGB8, left);
            sensor_msgs::Image leftImage;
            img_bridge_left.toImageMsg(leftImage);
            pubLeftImage_.publish(leftImage);
        }
        if(right.channels()==3)
            cv::cvtColor(right, right, cv::COLOR_BGR2RGB);
        else if(right.channels()==1)
            cv::cvtColor(right, right, cv::COLOR_GRAY2RGB);
        else
            return;
        
        if(!srvParams.raw_image_on)
        {
            std_msgs::Header header_right;
            cv_bridge::CvImage img_bridge_right;
            header_right.frame_id = srvParams.camera_right;
            header_right.stamp = ros::Time::now();
            img_bridge_right = cv_bridge::CvImage(header_right, sensor_msgs::image_encodings::RGB8, right);
            sensor_msgs::Image rightImage;
            img_bridge_right.toImageMsg(rightImage);
            pubRightImage_.publish(rightImage);
        }
        // Get left and right keypoints from the Aurora SDK
        for (size_t i = 0; i < trackingFrame.getKeypointsLeftCount(); i++)
        {
            auto &kp = trackingFrame.getKeypointsLeftBuffer()[i];
            if (kp.flags)
                cv::circle(left, cv::Point(kp.x, kp.y), 2, cv::Scalar(0, 255, 0), 2);
        }
        for (size_t i = 0; i < trackingFrame.getKeypointsRightCount(); i++)
        {
            auto &kp = trackingFrame.getKeypointsRightBuffer()[i];
            if (kp.flags)
                cv::circle(right, cv::Point(kp.x, kp.y), 2, cv::Scalar(0, 255, 0), 2);
        }

        cv::Mat merged;
        cv::hconcat(left, right, merged);
        std_msgs::Header header_stereo_keypoints;
        cv_bridge::CvImage img_bridge_stereo;
        header_stereo_keypoints.frame_id = srvParams.camera_left;
        header_stereo_keypoints.stamp = ros::Time::now();
        img_bridge_stereo = cv_bridge::CvImage(header_stereo_keypoints, sensor_msgs::image_encodings::RGB8, merged);
        sensor_msgs::Image mergeImage;
        img_bridge_stereo.toImageMsg(mergeImage);
        pubStereoKeyPoints_.publish(mergeImage);
    }

    ServerPointCloudWorker::ServerPointCloudWorker(
        SlamwareRosSdkServer *pRosSdkServer,
        const std::string &wkName,
        const std::chrono::milliseconds &triggerInterval)
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
        const auto &srvParams = serverParams();
        auto nhRos = rosNodeHandle();
        pubPointCloud_ = nhRos.advertise<sensor_msgs::PointCloud2>(srvParams.point_cloud_topic_name, 1);
    }

    ServerPointCloudWorker::~ServerPointCloudWorker()
    {
        //
    }

    bool ServerPointCloudWorker::reinitWorkLoop()
    {
        // Reinitialize the work loop if necessary
        return true;
    }
    void ServerPointCloudWorker::doPerform()
    {
        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        if (!auroraSDK)
        {
            ROS_ERROR("Failed to get aurora sdk");
            return;
        }

        int activeMapId = -1;
        slamtec_aurora_sdk_map_desc_t activeMapDesc;
        std::vector<slamtec_aurora_sdk_map_point_desc_t> mapPoints;
        slamtec_aurora_sdk_global_map_desc_t globalMapDesc;
        if (!auroraSDK->dataProvider.getGlobalMappingInfo(globalMapDesc))
        {
            ROS_ERROR("Failed to get global mapping info");
            return;
        }

        activeMapId = globalMapDesc.activeMapID;
        // to access the map data, we need to create a map data visitor
        RemoteMapDataVisitor mapDataVisitor;
        mapDataVisitor.subscribeMapPointData([&](const slamtec_aurora_sdk_map_point_desc_t &mapPointDesc)
                                             {
                                            // handle the map point data
                                            mapPoints.push_back(mapPointDesc); });

        // start the accessing session, this will block the background syncing thread during the accessing process.
        if (!auroraSDK->dataProvider.accessMapData(mapDataVisitor, {(uint32_t)activeMapId}))
        {
            ROS_ERROR("Failed to start the accessing session");
            return;
        }

        // tf_vslam_to_ros.setValue(0, 1, 0,
        //                        -1, 0, 0,
        //                        0, 0, 1);
        const tf::Matrix3x3 tf_vslam_to_ros(1, 0, 0,
                                            0, 1, 0,
                                            0, 0, 1);
        // publish map point to sensor_msgs::msg::PointCloud2
        const auto &srvParams = serverParams();
        const int num_channels = 3; // x y z
        sensor_msgs::PointCloud2 cloud;
        cloud.header.stamp = ros::Time::now();
        cloud.header.frame_id = srvParams.map_frame;
        cloud.height = 1;
        cloud.width = mapPoints.size();
        cloud.is_dense = true;
        cloud.is_bigendian = false;
        cloud.point_step = num_channels * sizeof(float);
        cloud.row_step = cloud.point_step * cloud.width;
        cloud.fields.resize(num_channels);

        std::string channel_id[] = {"x", "y", "z"};
        for (int i = 0; i < num_channels; i++)
        {
            cloud.fields[i].name = channel_id[i];
            cloud.fields[i].offset = i * sizeof(float);
            cloud.fields[i].count = 1;
            cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
        }

        cloud.data.resize(cloud.row_step * cloud.height);

        // mapPoints to cloud
        unsigned char *cloud_data_ptr = &(cloud.data[0]);

        float data_array[num_channels];
        for (unsigned int i = 0; i < cloud.width; i++)
        {
            // map points from orbslam2 aixs to ros axis
            tf::Vector3 mapPoint(mapPoints[i].position.x, mapPoints[i].position.y, mapPoints[i].position.z);
            tf::Vector3 mapPoint_ros = tf_vslam_to_ros * mapPoint;
            data_array[0] = mapPoint_ros.x();
            data_array[1] = mapPoint_ros.y();
            data_array[2] = mapPoint_ros.z();
            memcpy(cloud_data_ptr + (i * cloud.point_step),
                   data_array, num_channels * sizeof(float));
        }

        pubPointCloud_.publish(cloud);
    }
    //////////////////////////////////////////////////////////////////////////

    ServerEnhancedImagingWorker::ServerEnhancedImagingWorker(SlamwareRosSdkServer *pRosSdkServer,
                                                             const std::string &wkName,
                                                             const std::chrono::milliseconds &triggerInterval)
        : super_t(pRosSdkServer, wkName, triggerInterval)
        , depthCameraSupported_(false)
        , semanticSegmentationSupported_(false)
        , isInitialized_(false)
        , depth_lastTimestamp_(0),segmentation_lastTimestamp_(0)
    {
        const auto &srvParams = serverParams();
        auto nhRos = rosNodeHandle();

        // Initialize depth camera publishers
        pubDepthImage_ = nhRos.advertise<sensor_msgs::Image>(srvParams.depth_image_raw_topic_name, 5);
        pubDepthColorized_ = nhRos.advertise<sensor_msgs::Image>(srvParams.depth_image_colorized_topic_name, 5);
        pubDensePointCloud_ = nhRos.advertise<sensor_msgs::PointCloud2>("depth_camera/points", 5);

        // Initialize semantic segmentation publishers
        pubSemanticSegmentation_ = nhRos.advertise<sensor_msgs::Image>(srvParams.semantic_segmentation_topic_name, 5);
    }

    ServerEnhancedImagingWorker::~ServerEnhancedImagingWorker()
    {
    }

    bool ServerEnhancedImagingWorker::reinitWorkLoop()
    {
        if (!this->super_t::reinitWorkLoop())
            return false;

        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        if (!auroraSDK)
        {
            isWorkLoopInitOk_ = false;
            return false;
        }

        // Check depth camera support
        depthCameraSupported_ = auroraSDK->enhancedImaging.isDepthCameraSupported();
        if (depthCameraSupported_)
        {
            auroraSDK->controller.setEnhancedImagingSubscription(SLAMTEC_AURORA_SDK_ENHANCED_IMAGE_TYPE_DEPTH, true);
            // Enable depth camera post-filtering (refines depth data; flags currently ignored by SDK)
            auroraSDK->enhancedImaging.setDepthCameraPostFiltering(true, 0);
            ROS_INFO("Depth camera supported, subscription enabled, post-filtering on");
        } else {
            ROS_WARN("Depth camera not supported");
        }

        // Check semantic segmentation support
        semanticSegmentationSupported_ = auroraSDK->enhancedImaging.isSemanticSegmentationReady();
        if (semanticSegmentationSupported_)
        {
            auroraSDK->controller.setEnhancedImagingSubscription(SLAMTEC_AURORA_SDK_ENHANCED_IMAGE_TYPE_SEMANTIC, true);
            ROS_INFO("Semantic segmentation supported and subscription enabled");

            // Get semantic segmentation label information
            std::string labelSetName;
            if (auroraSDK->enhancedImaging.getSemanticSegmentationLabelSetName(labelSetName))
            {
                ROS_INFO("Semantic segmentation label set: %s", labelSetName.c_str());
            }

            slamtec_aurora_sdk_semantic_segmentation_label_info_t labelInfo;
            if (auroraSDK->enhancedImaging.getSemanticSegmentationLabels(labelInfo))
            {
                for (int i = 0; i < labelInfo.label_count; ++i)
                {
                    ROS_INFO("Label ID: %d, Name: %s", i, labelInfo.label_names[i].name);
                }
                // Generate class colors
                generateClassColors(labelInfo.label_count);
                ROS_INFO("Class colors generated");
            }
        } else {
            ROS_WARN("Semantic segmentation not supported");
        }

        isInitialized_ = true;
        isWorkLoopInitOk_ = true;
        return isWorkLoopInitOk_;
    }

    void ServerEnhancedImagingWorker::doPerform()
    {
        if (!isInitialized_)
        {
            return;
        }

        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        if (!auroraSDK)
        {
            return;
        }

        auto currentTime = ros::Time::now();
        std_msgs::Header header;
        header.stamp = currentTime;
        header.frame_id = "camera_left";

        // Process depth camera data
        if (depthCameraSupported_)
        {
            processDepthCamera(header);
        }

        // Process semantic segmentation data
        if (semanticSegmentationSupported_)
        {
            processSemanticSegmentation(header);
        }
    }

    void ServerEnhancedImagingWorker::processDepthCamera(const std_msgs::Header &header)
    {
        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        if (!auroraSDK)
        {
            return;
        }

        RemoteEnhancedImagingFrame depthFrame;

        // Early return if cannot peek frame
        if (auroraSDK->enhancedImaging.peekDepthCameraFrame(depthFrame, SLAMTEC_AURORA_SDK_DEPTHCAM_FRAME_TYPE_DEPTH_MAP))
        {
            if(depthFrame.desc.timestamp_ns>depth_lastTimestamp_)
            {
                depth_lastTimestamp_ = depthFrame.desc.timestamp_ns;
            }
            else
            {
                return;
            }
        }
        else
        {
            return;
        }

        cv::Mat depthImg;
        depthFrame.image.toMat(depthImg);

        // Publish raw depth image
        cv_bridge::CvImage depthBridge(header, sensor_msgs::image_encodings::TYPE_32FC1, depthImg);
        pubDepthImage_.publish(depthBridge.toImageMsg());

        // Create and publish colorized depth image
        cv::Mat heatmap;
        depthImg.convertTo(heatmap, CV_8UC1, -255.0 / 10.0f, 255);
        cv::Mat depthColorized;
        cv::applyColorMap(heatmap, depthColorized, cv::COLORMAP_JET);

        cv_bridge::CvImage depthColorizedBridge(header, sensor_msgs::image_encodings::BGR8, depthColorized);
        pubDepthColorized_.publish(depthColorizedBridge.toImageMsg());

        // ----------------------------------------------------------------
        // Dense coloured PointCloud2 from device-unprojected POINT3D frame
        // ----------------------------------------------------------------
        if (pubDensePointCloud_.getNumSubscribers() == 0)
            return;

        // Peek the POINT3D frame (same timestamp bucket as depthFrame)
        RemoteEnhancedImagingFrame point3dFrame;
        slamtec_aurora_sdk_errorcode_t errCode;
        if (!auroraSDK->enhancedImaging.peekDepthCameraFrame(
                point3dFrame, SLAMTEC_AURORA_SDK_DEPTHCAM_FRAME_TYPE_POINT3D, &errCode))
        {
            ROS_WARN_THROTTLE(5.0, "processDepthCamera: failed to peek POINT3D frame (err %d)", (int)errCode);
            return;
        }

        cv::Mat xyzMat;  // CV_32FC3, metric XYZ in camera-0 frame, size == depth image
        point3dFrame.image.toMat(xyzMat);
        if (xyzMat.empty())
            return;

        // Peek the rectified left image for per-pixel colour
        RemoteEnhancedImagingFrame textureFrame;
        bool hasTexture = auroraSDK->enhancedImaging.peekDepthCameraRelatedRectifiedImage(
            textureFrame, depthFrame.desc.timestamp_ns, &errCode);

        cv::Mat textureBGR;
        if (hasTexture)
        {
            textureFrame.image.toMat(textureBGR);
            if (!textureBGR.empty() && textureBGR.channels() == 1)
                cv::cvtColor(textureBGR, textureBGR, cv::COLOR_GRAY2BGR);
            // Resize texture to match point cloud grid if dimensions differ
            if (!textureBGR.empty() &&
                (textureBGR.cols != xyzMat.cols || textureBGR.rows != xyzMat.rows))
            {
                cv::resize(textureBGR, textureBGR, xyzMat.size(), 0, 0, cv::INTER_LINEAR);
            }
        }

        // Build an organised PointCloud2 (same width/height as the depth grid so
        // downstream tools can treat it as a structured cloud if desired)
        const int rows = xyzMat.rows;
        const int cols = xyzMat.cols;

        sensor_msgs::PointCloud2 cloud;
        cloud.header = header;
        cloud.height = rows;
        cloud.width  = cols;
        cloud.is_dense = false;  // may contain NaN for invalid pixels

        // Field layout: x, y, z (float32), padding, rgb packed as float32
        sensor_msgs::PointField fx, fy, fz, frgb;
        fx.name = "x";    fx.offset = 0;  fx.datatype = sensor_msgs::PointField::FLOAT32; fx.count = 1;
        fy.name = "y";    fy.offset = 4;  fy.datatype = sensor_msgs::PointField::FLOAT32; fy.count = 1;
        fz.name = "z";    fz.offset = 8;  fz.datatype = sensor_msgs::PointField::FLOAT32; fz.count = 1;
        frgb.name = "rgb"; frgb.offset = 16; frgb.datatype = sensor_msgs::PointField::FLOAT32; frgb.count = 1;
        cloud.fields = {fx, fy, fz, frgb};
        cloud.point_step = 32;  // 4 floats × 4 bytes + 4 bytes padding × 2 (pad after z, pad after rgb)
        // Simpler: use 5 × float32 layout – x,y,z,_,rgb
        // point_step = 20 bytes total but must be aligned; use 20 directly.
        // Re-layout cleanly: x(0) y(4) z(8) pad(12) rgb(16) pad(20) → step=20 is fine.
        cloud.point_step = 20;
        frgb.offset = 16;
        cloud.fields = {fx, fy, fz, frgb};

        cloud.row_step = cloud.point_step * cloud.width;
        cloud.data.resize(cloud.row_step * cloud.height, 0);

        uint8_t* dataPtr = cloud.data.data();
        for (int r = 0; r < rows; ++r)
        {
            for (int c = 0; c < cols; ++c)
            {
                uint8_t* pointPtr = dataPtr + r * cloud.row_step + c * cloud.point_step;
                const cv::Vec3f& xyz = xyzMat.at<cv::Vec3f>(r, c);

                float px = xyz[0], py = xyz[1], pz = xyz[2];

                // Emit NaN for invalid points so is_dense=false consumers skip them
                if (std::isnan(px) || std::isnan(py) || std::isnan(pz) ||
                    std::isinf(px) || std::isinf(py) || std::isinf(pz) || pz <= 0.0f)
                {
                    const float nan = std::numeric_limits<float>::quiet_NaN();
                    memcpy(pointPtr + 0,  &nan, 4);
                    memcpy(pointPtr + 4,  &nan, 4);
                    memcpy(pointPtr + 8,  &nan, 4);
                    continue;
                }

                memcpy(pointPtr + 0,  &px, 4);
                memcpy(pointPtr + 4,  &py, 4);
                memcpy(pointPtr + 8,  &pz, 4);

                // Pack BGR → RGB uint32 in the float rgb field
                uint8_t b = 128, g = 128, rv = 128;
                if (!textureBGR.empty())
                {
                    const cv::Vec3b& bgr = textureBGR.at<cv::Vec3b>(r, c);
                    b = bgr[0]; g = bgr[1]; rv = bgr[2];
                }
                uint32_t rgb32 = (static_cast<uint32_t>(rv) << 16)
                               | (static_cast<uint32_t>(g)  <<  8)
                               |  static_cast<uint32_t>(b);
                float rgbF;
                memcpy(&rgbF, &rgb32, 4);
                memcpy(pointPtr + 16, &rgbF, 4);
            }
        }

        pubDensePointCloud_.publish(cloud);
    }

    void ServerEnhancedImagingWorker::processSemanticSegmentation(const std_msgs::Header &header)
    {
        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        if (!auroraSDK)
        {
            return;
        }

        RemoteEnhancedImagingFrame segFrame;
        // Early return if cannot peek frame
        if (auroraSDK->enhancedImaging.peekSemanticSegmentationFrame(segFrame))
        {
            if(segFrame.desc.timestamp_ns>segmentation_lastTimestamp_)
            {
                segmentation_lastTimestamp_ = segFrame.desc.timestamp_ns;
            }
            else
            {
                return;
            }
        }
        else
        {
            return;
        }

        cv::Mat localSegMap;
        segFrame.image.toMat(localSegMap);
        auto currentSegMap = localSegMap.clone();

        if (currentSegMap.empty())
        {
            return;
        }

        // Peek camera preview image for overlay (if needed)
        RemoteStereoImagePair cameraImagePair;
        cv::Mat currentCameraImage;

        if (auroraSDK->dataProvider.peekCameraPreviewImage(cameraImagePair, segFrame.desc.timestamp_ns, true))
        {
            cameraImagePair.leftImage.toMat(currentCameraImage);
            if (currentCameraImage.channels() == 1)
            {
                cv::cvtColor(currentCameraImage, currentCameraImage, cv::COLOR_GRAY2BGR);
            }
        }

        auto currentColorizedSegMap = colorizeSegmentationMap(currentSegMap);
        auto currentOverlayBase = createCameraOverlay(currentCameraImage, currentColorizedSegMap);

        // Publish colorized segmentation
        cv_bridge::CvImage segBridge(header, sensor_msgs::image_encodings::BGR8, currentOverlayBase);
        pubSemanticSegmentation_.publish(segBridge.toImageMsg());
    }

    cv::Mat ServerEnhancedImagingWorker::createCameraOverlay(const cv::Mat &cameraImage, const cv::Mat &colorizedSegMap)
    {
        if (cameraImage.empty() || colorizedSegMap.empty())
        {
            return colorizedSegMap.clone();
        }

        cv::Mat overlay = cameraImage.clone();

        // Optimized blending using OpenCV vectorized operations
        cv::Mat mask;
        cv::inRange(colorizedSegMap, cv::Scalar(1, 1, 1), cv::Scalar(255, 255, 255), mask);

        // Blend only non-black pixels
        cv::Mat blendedSeg;
        cv::addWeighted(cameraImage, 0.6, colorizedSegMap, 0.4, 0, blendedSeg);
        blendedSeg.copyTo(overlay, mask);

        return overlay;
    }

    cv::Mat ServerEnhancedImagingWorker::colorizeSegmentationMap(const cv::Mat &segMap)
    {
        cv::Mat colorized(segMap.size(), CV_8UC3);
        for (int y = 0; y < segMap.rows; y++)
        {
            for (int x = 0; x < segMap.cols; x++)
            {
                uint8_t classId = segMap.at<uint8_t>(y, x);
                if (classId < classColors_.size())
                {
                    colorized.at<cv::Vec3b>(y, x) = classColors_[classId];
                }
                else
                {
                    colorized.at<cv::Vec3b>(y, x) = cv::Vec3b(128, 128, 128); // Gray for unknown classes
                }
            }
        }

        return colorized;
    }

    void ServerEnhancedImagingWorker::generateClassColors(int numClasses)
    {
        classColors_.clear();
        classColors_.resize(numClasses);

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(50, 255);

        // Set background (index 0) to black for transparency effect
        classColors_[0] = cv::Vec3b(0, 0, 0);

        // Generate random colors for other classes
        for (int i = 1; i < numClasses; i++)
        {
            classColors_[i] = cv::Vec3b(dis(gen), dis(gen), dis(gen));
        }
    }

    ServerRawImageWorker::ServerRawImageWorker(
        SlamwareRosSdkServer *pRosSdkServer,
        const std::string &wkName,
        const std::chrono::milliseconds &triggerInterval)
        : super_t(pRosSdkServer, wkName, triggerInterval),lastTimestamp_(0)
    {
        const auto &srvParams = serverParams();
        auto nhRos = rosNodeHandle();
        pubLeftImage_ = nhRos.advertise<sensor_msgs::Image>(srvParams.left_image_raw_topic_name, 1);
        pubRightImage_ = nhRos.advertise<sensor_msgs::Image>(srvParams.right_image_raw_topic_name, 1);
    }

    ServerRawImageWorker::~ServerRawImageWorker()
    {
    }

    bool ServerRawImageWorker::reinitWorkLoop()
    {
        // Reinitialize the work loop if necessary
        return true;
    }
    void ServerRawImageWorker::doPerform()
    {
        
    }


}