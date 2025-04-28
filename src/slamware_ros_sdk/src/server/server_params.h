#pragma once
#include "rclcpp/rclcpp.hpp"

namespace slamware_ros_sdk {

    extern const float C_FLT_PI;
    extern const float C_FLT_2PI;

    class ServerParams : public rclcpp::Node
    {
    public:

        ServerParams();

        void resetToDefault();
        void setBy(const std::shared_ptr<rclcpp::Node> nhRos);

        template<class ParameterT>
        ParameterT getParameter(const std::string& name) const
        {
            ParameterT retV;

            if (this->has_parameter(name)) {
                this->get_parameter<ParameterT>(name, retV);
            }
            return retV;
        }
    }; 
}
