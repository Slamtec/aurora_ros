# slamware_ros_sdk 
目前支持 ROS Noetic

# 下载相关版本的ros sdk
请于思岚科技官方网站下载相关的软件包  https://www.slamtec.com/cn/support

# 开发环境需求
基于 Ubuntu 20.04 操作系统，并装有 ROS

# 硬件需求
为使用 ROS SDK，您需要一台 Auraro 空间建图设备，开启并配置合适的 IP 地址。slamware_ros_sdk_server_node 节点启动后将尝试连接该设备。

# 编译
1.下载源码

```bash
git clone https://github.com/Slamtec/aurora_ros
```

2.编译

```bash
cd aurora_ros
source /opt/ros/noetic/setup.bash
catkin_make
```

3.启动节点
若Aurora设备处于AP模式，连接Aurora 热点。

执行如下命令行启动节点

```bash
source devel/setup.bash
roslaunch slamware_ros_sdk slamware_ros_sdk_server_and_view.launch ip_address:=192.168.11.1
```

4.具体相关的aurora_ros_sdk_server_node节点可以查看相关的Wiki说明
https://developer.slamtec.com/docs/slamware/aurora-ros2-sdk/slamware_ros_sdk_server_node/
