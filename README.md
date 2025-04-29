# Welcome to Aurora ROS2 SDK

([中文版点此](README.zh-CN.md))

Aurora is a newly developed integrated positioning and mapping sensor by SLAMTEC, which combines LiDAR, vision, inertial navigation, and deep learning technologies. This sensor requires no external dependencies and can provide six degrees of freedom (DOF) positioning capabilities with high-precision 3D mapping for both indoor and outdoor environments immediately after powering on..

## Get Started
### Directory Structure

The Aurora ROS2 SDK contains the resources and code you may need during your development process. The directory structure is organized as follows:

| Directory              | Description                               |
| ---------------------- | ----------------------------------------- |
| src                    | Source code                               |
| --slamware_ros_sdk     | Source code of Slamware ROS SDK           |
| --aurora_remote_public | Aurora-related header files and libraries |

### Development Environment

- The SDK is based on Ubuntu 20.04 / 22.04 operating systems and requires the installation of ROS2 packages.

### Hardware Requirements

To use the ROS2 SDK, you will need a device based on Aurora spatial mapping. The device should be powered on and configured with an appropriate IP address. The `slamware_ros_sdk_server_node` will attempt to connect to this device once started.

### Hello World

#### 1. Create workspace

Place the `src` containing the source code into an empty workspace directory. For details, refer to: <a href="http://wiki.ros.org/catkin">https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html</a>, and use `colcon build` to initialize the workspace.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

#### 2. Compile

```bash
cd ..
colcon build
```

#### 3. Setup workspace environment

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Since `aurora_remote_public` library is a dynamic library, you need to add the platform path to `LD_LIBRARY_PATH`. For example, if you place `slamware_ros2_sdk_linux-x86_64-gcc9` in the `~/ros_ws/src` folder, you need to add the following command to `~/.bashrc`:**

```
export LD_LIBRARY_PATH=~/ros_ws/src/slamware_ros2_sdk_linux-x86_64-gcc9/src/aurora_remote_public/lib/linux_x86_64:$LD_LIBRARY_PATH
```

#### 4. Launch the Node

If the Aurora device is in AP mode, connect to the Aurora Wi-Fi and launch the node.

```bash
ros2 launch slamware_ros_sdk slamware_ros_sdk_server_and_view.xml ip_address:=192.168.11.1
```

#### 5. View Detailed Documentation
For detailed information about the aurora_ros_sdk_server_node, refer to the related Wiki documentation: https://developer.slamtec.com/docs/slamware/aurora-ros2-sdk/slamware_ros_sdk_server_node/