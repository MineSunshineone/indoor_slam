cmake_minimum_required(VERSION 3.8)
project(bxi_nav_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NavPose.msg"
  "msg/NavStatus.msg"
  "msg/MappingStatus.msg"
  "srv/SetInitialPose.srv"
  "srv/SaveMap.srv"
  "srv/ClearTerrain.srv"
  "action/NavGoto.action"
  "action/BuildMap.action"
  "action/FollowWaypoints.action"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
