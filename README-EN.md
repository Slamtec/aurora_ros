# slamware_ros_sdk 
Currently supports ROS Noetic

# Download the corresponding version of the ROS SDK
Please download the relevant software package from the Slamtec official website: https://www.slamtec.com/cn/support

# Development Environment Requirements
Based on the Ubuntu 20.04 operating system with ROS installed.

# Hardware Requirements
To use the ROS SDK, you need an Aurora mapping device. Ensure it is powered on and configured with the appropriate IP address. The `slamware_ros_sdk_server_node` will attempt to connect to the device after being launched.

# Compilation
1.Download the source code:

```bash
git clone -b feature/aurora-ros https://git.eng.slamtec.com/slamware/ros/ros_sdk.git
```

2.Compile

```bash
cd ros_sdk
source /opt/ros/noetic/setup.bash
catkin_make
```

3.Launch the node
If the Aurora device is in AP mode, connect to the Aurora hotspot.

Run the following command to launch the node:

```bash
source devel/setup.bash
roslaunch slamware_ros_sdk slamware_ros_sdk_server_and_view.launch ip_address:=192.168.11.1
```

4.For detailed information about the aurora_ros_sdk_server_node, refer to the related Wiki documentation:
https://developer.slamtec.com/docs/slamware/aurora-ros2-sdk/slamware_ros_sdk_server_node/
