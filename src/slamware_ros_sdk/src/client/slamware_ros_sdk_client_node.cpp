#include "rclcpp/rclcpp.hpp"

#include <slamware_ros_sdk/slamware_ros_sdk_client.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<slamware_ros_sdk::SlamwareRosSdkClient>();

    std::string errMsg;
    node->syncGetStcm(errMsg, "example_map.stcm");
    node->syncSetStcm("example_map.stcm", errMsg);

    rclcpp::spin(node);
    rclcpp::shutdown();
  return 0;
}
