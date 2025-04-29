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

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

#### 2. Download Source Code
```bash
git clone -b ros2 https://github.com/Slamtec/aurora_ros.git
```

#### 3. Compile

```bash
cd ..
colcon build
```

#### 4. Setup workspace environment

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Since `aurora_remote_public` library is a dynamic library, you need to add the platform path to `LD_LIBRARY_PATH`. For example, if you place `aurora_ros` in the `~/ros2_ws/src` folder, you need to add the following command to `~/.bashrc`:**

```
export LD_LIBRARY_PATH=~/ros2_ws/src/aurora_ros/src/aurora_remote_public/lib/linux_x86_64:$LD_LIBRARY_PATH
```

#### 5. Launch the Node

If the Aurora device is in AP mode, connect to the Aurora Wi-Fi and launch the node.

```bash
ros2 launch slamware_ros_sdk slamware_ros_sdk_server_and_view.xml ip_address:=192.168.11.1
```

#### 6. View Detailed Documentation
For detailed information about the aurora_ros_sdk_server_node, refer to the related Wiki documentation: https://developer.slamtec.com/docs/slamware/aurora-ros2-sdk/slamware_ros_sdk_server_node/