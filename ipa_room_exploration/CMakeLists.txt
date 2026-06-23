cmake_minimum_required(VERSION 3.8)
project(ipa_room_exploration)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread chrono)
find_package(Threads REQUIRED)
find_package(ipa_building_navigation REQUIRED)

# for the ROS2 action server
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ipa_building_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# rena-patches ROS2 port: build only the boustrophedon coverage library. The other
# explorers (flow_network / convex_SPP — need CoinOR; neural_network; grid_point),
# the dynamic_reconfigure config and the actionlib server are not built here.
add_library(room_exploration
  common/src/boustrophedon_explorator.cpp
  common/src/room_rotator.cpp
  common/src/meanshift2d.cpp
  ros/src/fov_to_robot_mapper.cpp
)
target_include_directories(room_exploration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/common/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ros/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(room_exploration
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  Threads::Threads
)
ament_target_dependencies(room_exploration ipa_building_navigation)

install(DIRECTORY common/include/ DESTINATION include)
install(DIRECTORY ros/include/ DESTINATION include)
install(TARGETS room_exploration
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenCV Eigen3 Boost ipa_building_navigation)

# ROS2 coverage action server (wraps the boustrophedon library)
add_executable(room_exploration_server ros/src/room_exploration_action_server_ros2.cpp)
target_link_libraries(room_exploration_server room_exploration ${OpenCV_LIBRARIES})
ament_target_dependencies(room_exploration_server
  rclcpp rclcpp_action ipa_building_msgs sensor_msgs geometry_msgs cv_bridge)
install(TARGETS room_exploration_server DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
