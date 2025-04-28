
#pragma once

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "server_map_holder.h"

#include <atomic>
#include <memory>

namespace slamware_ros_sdk {

    struct ServerWorkData
    {
    public:
        geometry_msgs::msg::PoseStamped robotPose;

        std::atomic<bool> syncMapRequested;
        ServerMapHolder exploreMapHolder;

    public:
        ServerWorkData();

    public:
        static inline bool sfIsDigitalSensorValueImpact(float fVal) { return fVal < FLT_EPSILON; }
    };

    typedef std::shared_ptr<ServerWorkData>               ServerWorkData_Ptr;
    typedef std::shared_ptr<const ServerWorkData>         ServerWorkData_ConstPtr;
    
}
