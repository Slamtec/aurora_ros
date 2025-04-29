cmake_minimum_required(VERSION 3.0.2)
project(slamware_ros_sdk)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

#Define Color
string(ASCII 27 Esc) 
set(Red "${Esc}[31m")
set(Green "${Esc}[32m")
set(Yellow "${Esc}[33m")
set(Blue "${Esc}[34m")
set(Magenta "${Esc}[35m")
set(Cyan "${Esc}[36m")
set(White "${Esc}[37m")

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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=gnu++11")

# Find OpenCV
find_package(OpenCV REQUIRED)



# Add the determined library path to the linker search path
get_filename_component(AURORA_LIB_PATH_ABS "${AURORA_LIB_PATH}" ABSOLUTE)

include_directories(${PROJECT_SOURCE_DIR}/../aurora_remote_public/include)
link_directories(${AURORA_LIB_PATH_ABS})
message(STATUS "")
message(STATUS "${Green}Aurora SDK library path: ${AURORA_LIB_PATH_ABS}${Reset}")
message(STATUS "Please add ${Green}export LD_LIBRARY_PATH=${AURORA_LIB_PATH_ABS}:\$LD_LIBRARY_PATH ${Reset} to ~/.bashrc")
message(STATUS "")


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
  cv_bridge
)
# Find thread library
find_package(Threads REQUIRED)


add_message_files(
  FILES
  OptionalBool.msg
  OptionalInt8.msg
  OptionalInt16.msg
  OptionalInt32.msg
  OptionalInt64.msg
  OptionalUInt8.msg
  OptionalUInt16.msg
  OptionalUInt32.msg
  OptionalUInt64.msg
  OptionalFlt32.msg
  OptionalFlt64.msg
  Vec2DInt32.msg
  Vec2DFlt32.msg
  Line2DFlt32.msg
  Line2DFlt32Array.msg
  RectInt32.msg
  RectFlt32.msg
  RobotDeviceInfo.msg
  MapKind.msg
  ArtifactUsage.msg
  SensorType.msg
  ImpactType.msg
  BasicSensorInfo.msg
  BasicSensorInfoArray.msg
  BasicSensorValue.msg
  BasicSensorValueData.msg
  BasicSensorValueDataArray.msg
  ActionDirection.msg
  RobotBasicState.msg
  SyncMapRequest.msg
  MoveOptionFlag.msg
  MoveOptions.msg
  MoveByDirectionRequest.msg
  MoveByThetaRequest.msg
  MoveToRequest.msg
  MoveToLocationsRequest.msg
  RotateToRequest.msg
  RotateRequest.msg
  LocalizationMovement.msg
  OptionalLocalizationMovement.msg
  LocalizationOptions.msg
  RecoverLocalizationRequest.msg
  ClearMapRequest.msg
  SetMapUpdateRequest.msg
  SetMapLocalizationRequest.msg
  GoHomeRequest.msg
  CancelActionRequest.msg
  AddLineRequest.msg
  AddLinesRequest.msg
  RemoveLineRequest.msg
  ClearLinesRequest.msg
  MoveLineRequest.msg
  MoveLinesRequest.msg
  RelocalizationCancelRequest.msg
  SystemStatus.msg
  RelocalizationStatus.msg
)

add_service_files(
  FILES
  RelocalizationRequest.srv
  SyncGetStcm.srv
  SyncSetStcm.srv
)

generate_messages(
  DEPENDENCIES
  nav_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES slamware_ros_sdk
  CATKIN_DEPENDS nav_msgs roscpp rospy sensor_msgs std_msgs tf message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
message("${INCLUDE_DIRECTORIES}")

add_library(${PROJECT_NAME} STATIC
  src/client/slamware_ros_sdk_client.cpp
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(slamware_ros_sdk_server_node
  src/server/server_params.cpp
  src/server/server_map_holder.cpp
  src/server/server_work_data.cpp
  src/server/server_worker_base.cpp
  src/server/server_workers.cpp
  src/server/slamware_ros_sdk_server_node.cpp
  src/server/slamware_ros_sdk_server.cpp
)
add_dependencies(slamware_ros_sdk_server_node ${PROJECT_NAME}_generate_messages_cpp)
target_include_directories(slamware_ros_sdk_server_node
  PRIVATE ${SLTC_SDK_INC_DIR}
)
target_compile_options(slamware_ros_sdk_server_node
  PRIVATE -Wno-deprecated-declarations
)
target_link_libraries(slamware_ros_sdk_server_node
${AURORA_LIB_PATH}/libslamtec_aurora_remote_sdk.so
  ${catkin_LIBRARIES}
  pthread
  dl
  rt
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_server_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES
  launch/slamware_ros_sdk_server_node.launch
  launch/view_slamware_ros_sdk_server_node.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)