#pragma once
// rena-patch (ROS2): the TSP solvers only used ros/ros.h for ROS_INFO-style
// logging. ROS2 has no ros/ros.h, so provide the few logging macros these
// headers/sources rely on, backed by plain stdio.
#include <cstdio>

#ifndef ROS_INFO
#define ROS_INFO(...)  do { std::printf(__VA_ARGS__); std::printf("\n"); } while (0)
#endif
#ifndef ROS_WARN
#define ROS_WARN(...)  do { std::printf(__VA_ARGS__); std::printf("\n"); } while (0)
#endif
#ifndef ROS_ERROR
#define ROS_ERROR(...) do { std::printf(__VA_ARGS__); std::printf("\n"); } while (0)
#endif
