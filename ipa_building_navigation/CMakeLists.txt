cmake_minimum_required(VERSION 3.8)
project(ipa_building_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread chrono)
find_package(Threads REQUIRED)

# rena-patches ROS2 port: build only the TSP-solver library used by the coverage
# planner. Concorde (external binary + ros::package), the clique/set-cover/trolley
# library and the ROS action servers are intentionally not built.
add_library(tsp_solvers
  common/src/A_star_pathplanner.cpp
  common/src/node.cpp
  common/src/nearest_neighbor_TSP.cpp
  common/src/genetic_TSP.cpp
)
target_include_directories(tsp_solvers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/common/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(tsp_solvers
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  Threads::Threads
)

install(DIRECTORY common/include/
  DESTINATION include
)
install(TARGETS tsp_solvers
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenCV Eigen3 Boost)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
