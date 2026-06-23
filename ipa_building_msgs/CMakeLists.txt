cmake_minimum_required(VERSION 3.8)
project(ipa_building_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# rena-patches ROS2 port: only the coverage path is generated. The segmentation /
# room-sequence actions, the RoomInformation/RoomSequence msgs, and the srvs are
# not carried over (segmentation is supplied externally in the RENA stack).
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/RoomExploration.action"
  DEPENDENCIES geometry_msgs sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
