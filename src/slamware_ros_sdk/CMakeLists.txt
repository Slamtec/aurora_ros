cmake_minimum_required(VERSION 3.5)
project(slamware_ros_sdk)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Determine platform and architecture
if(WIN32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(AURORA_LIB_PATH "${PROJECT_SOURCE_DIR}/../aurora_remote_public/lib/win64")
    else()
        message(FATAL_ERROR "Unsupported architecture for Windows. Only x64 is supported.")
    endif()
elseif(UNIX AND NOT APPLE)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        set(AURORA_LIB_PATH "${PROJECT_SOURCE_DIR}/../aurora_remote_public/lib/linux_aarch64")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
        set(AURORA_LIB_PATH "${PROJECT_SOURCE_DIR}/../aurora_remote_public/lib/linux_x86_64")
    else()
        message(FATAL_ERROR "Unsupported architecture for Linux. Only aarch64 and x86_64 are supported.")
    endif()
else()
    message(FATAL_ERROR "Unsupported platform. Only Windows and Linux are supported.")
endif()

# Set the library name
set(AURORA_LIB_NAME slamtec_aurora_remote_sdk)

# Add the determined library path to the linker search path
include_directories(${PROJECT_SOURCE_DIR}/../aurora_remote_public/include)
link_directories(${AURORA_LIB_PATH})

message(STATUS "Aurora SDK library path: ${AURORA_LIB_PATH}")

# find dependencies
find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Threads) 

rosidl_generate_interfaces(${PROJECT_NAME}
 msg/SyncMapRequest.msg
 msg/ClearMapRequest.msg
 msg/SetMapUpdateRequest.msg
 msg/SetMapLocalizationRequest.msg
 msg/RelocalizationStatus.msg
 msg/RelocalizationCancelRequest.msg
 msg/SystemStatus.msg
 srv/SyncGetStcm.srv
 srv/SyncSetStcm.srv
 srv/RelocalizationRequest.srv
 DEPENDENCIES nav_msgs sensor_msgs std_msgs
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(std_srvs)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(tf2_msgs)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(message_runtime)
ament_export_dependencies(rosidl_default_runtime)

include_directories(include)
include_directories(${AMENT_PREFIX_PATH})
include_directories(~/ros2_humble/ros2-linux/include)
include_directories($ENV{AMENT_PREFIX_PATH}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/build/slamware_ros_sdk/rosidl_generator_cpp)
include_directories(${OpenCV_INCLUDE_DIRS}
)

add_executable(slamware_ros_sdk_server_node
  src/server/server_params.cpp
  src/server/server_map_holder.cpp
  src/server/server_work_data.cpp
  src/server/server_worker_base.cpp
  src/server/server_workers.cpp
  src/server/slamware_ros_sdk_server_node.cpp
  src/server/slamware_ros_sdk_server.cpp
)

ament_target_dependencies(slamware_ros_sdk_server_node rclcpp std_msgs std_srvs tf2_ros tf2_geometry_msgs cv_bridge sensor_msgs OpenCV)

if(COMMAND rosidl_get_typesupport_target)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
  target_link_libraries(slamware_ros_sdk_server_node ${cpp_typesupport_target})
else()
  rosidl_target_interfaces(slamware_ros_sdk_server_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

target_include_directories(slamware_ros_sdk_server_node
  PRIVATE ${SLTC_SDK_INC_DIR}
)
target_compile_options(slamware_ros_sdk_server_node 
  PRIVATE -Wno-deprecated-declarations
)

target_link_libraries(slamware_ros_sdk_server_node 
  ${AURORA_LIB_PATH}/libslamtec_aurora_remote_sdk.so
  pthread
  dl
  rt
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ${PROJECT_NAME}_server_node 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
ament_export_include_directories(include/${PROJECT_NAME})

install(FILES
  launch/slamware_ros_sdk_server_node.xml
  launch/view_slamware_ros_sdk_server_node.xml
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY 
  rviz
	DESTINATION share/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
