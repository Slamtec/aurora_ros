/**
 * @file slamware_ros_sdk_server_node.cpp
 * @brief Entry point for the Slamware ROS SDK server node.
 *
 * Original author: kint.zhao (huasheng_zyh@163.com), 2017-07-21
 * Modified by: yun.li@slamtec.com, 2019
 */

#include "slamware_ros_sdk_server.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
  std::string errMsg;
  ros::init(argc, argv, "slamware_ros_sdk_server_node");

  slamware_ros_sdk::SlamwareRosSdkServer rosSdkServer;
  if (!rosSdkServer.startRun(errMsg))
  {
    ROS_ERROR("failed to start slamware ros sdk server: %s.", errMsg.c_str());
    return -1;
  }

  ros::spin();

  rosSdkServer.requestStop();
  rosSdkServer.waitUntilStopped();

  return 0;
}
