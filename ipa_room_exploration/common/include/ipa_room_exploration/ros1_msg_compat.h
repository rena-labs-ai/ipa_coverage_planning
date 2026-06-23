#pragma once
// rena-patch (ROS2 port): the ipa algorithm core was written against the ROS1
// POD message types geometry_msgs::Pose2D / Point32 / Polygon. ROS2 only ships
// geometry_msgs::msg::* (different namespace, .hpp headers). To keep the 17k-line
// computational core completely untouched, we provide these lightweight POD types
// here. The library never publishes ROS messages — the ROS2 action server wrapper
// converts between these PODs and geometry_msgs::msg::* at its boundary.
#include <vector>

namespace geometry_msgs {
struct Pose2D { double x = 0.0; double y = 0.0; double theta = 0.0; };
struct Point32 { float x = 0.0f; float y = 0.0f; float z = 0.0f; };
struct Polygon { std::vector<Point32> points; };
}  // namespace geometry_msgs
