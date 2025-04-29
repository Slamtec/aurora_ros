# Welcome to Aurora ROS2 SDK

Aurora是SLAMTEC公司全新打造的融合激光、视觉、惯导和深度学习技术，一体化定位与建图感知传感器。该传感器无需外部依赖，开机即可实现室内外三维高精建图机提供六自由度定位能力。

## Get Started

### 目录结构

Aurora ROS2 SDK包含了您开发过程中可能会用到的资源、代码，其目录结构组织如下：

| 目录                     | 说明              |
| ---------------------- | --------------- |
| src                    | 源码              |
| --slamware_ros_sdk     | ROS SDK源码包      |
| --aurora_remote_public | Aurora相关头文件与库文件 |

### 开发环境需求

基于Ubuntu 20.04 / 22.04操作系统，并装有ROS2软件包。

### 硬件需求

为使用ROS2 SDK，您需要一台基于Auraro空间建图设备，开启并配置合适的IP地址。slamware_ros_sdk_server_node节点启动后将尝试连接该设备。

### Hello World

#### 1. 创建工作空间

将存放源码的src放入一个空的工作目录，详情见：<a href="http://wiki.ros.org/catkin">https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html，使用colcon build初始化工作空间

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

#### 2. 编译

```bash
cd ..
colcon build
```

#### 3. 配置工作空间系统环境

```bash
source install/setup.bash
```

**为了每次启动终端时自动加载环境：** 将以下命令添加到 `~/.bashrc` 中

```
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**由于aurora_remote_public lib为动态库，因此需要将所用平台路径加入到LD_LIBRARY_PATH中，如将slamware_ros2_sdk_linux-x86_64-gcc9放置在~/ros_ws/src文件夹下，需要将以下命令添加到 `~/.bashrc` 中**

```
export LD_LIBRARY_PATH=~/ros_ws/src/slamware_ros2_sdk_linux-x86_64-gcc9/src/aurora_remote_public/lib/linux_x86_64:$LD_LIBRARY_PATH
```

#### 4. 启动节点

若Aurora设备处于AP模式，连接Aurora WIFI，启动节点

```bash
ros2 launch slamware_ros_sdk slamware_ros_sdk_server_and_view.xml ip_address:=192.168.11.1
```

#### 5. 查看详细文档
关于aurora_ros_sdk_server_node的详细信息，请参考相关Wiki文档：https://developer.slamtec.com/docs/slamware/aurora-ros2-sdk/slamware_ros_sdk_server_node/