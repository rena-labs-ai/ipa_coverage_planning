/*
 * room_exploration_action_server_ros2.cpp
 *
 * rena-patch: a ROS2 (rclcpp_action) coverage-path server, replacing the ROS1
 * actionlib server. It wraps the boustrophedon coverage library and exposes the
 * ipa_building_msgs/RoomExploration action. Compared to the ROS1 server this port:
 *   - supports the boustrophedon explorer only (the explorer we ported),
 *   - supports footprint mode (planning_mode == 1); FOV mode (2) is not yet ported,
 *   - replaces dynamic_reconfigure with ROS2 parameters,
 *   - drops the cob_map_accessibility_analysis path-feasibility step.
 *
 * Note on types: the boustrophedon library is ROS-message-agnostic and works with
 * the POD geometry_msgs::Pose2D (see ros1_msg_compat.h). This node converts those
 * to the ROS2 geometry_msgs::msg::* types at the action boundary.
 */

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <ipa_building_msgs/action/room_exploration.hpp>
#include <ipa_room_exploration/boustrophedon_explorator.h>

using RoomExploration = ipa_building_msgs::action::RoomExploration;
using GoalHandle = rclcpp_action::ServerGoalHandle<RoomExploration>;
using namespace std::placeholders;

class RoomExplorationServer : public rclcpp::Node
{
public:
  RoomExplorationServer() : rclcpp::Node("room_exploration_server")
  {
    min_cell_area_ = this->declare_parameter<double>("min_cell_area", 10.0);
    path_eps_ = this->declare_parameter<double>("path_eps", 2.0);
    grid_obstacle_offset_ = this->declare_parameter<double>("grid_obstacle_offset", 0.0);
    max_deviation_from_track_ = this->declare_parameter<int>("max_deviation_from_track", -1);
    cell_visiting_order_ = this->declare_parameter<int>("cell_visiting_order", 1);
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");

    action_server_ = rclcpp_action::create_server<RoomExploration>(
      this, "room_exploration",
      std::bind(&RoomExplorationServer::handleGoal, this, _1, _2),
      std::bind(&RoomExplorationServer::handleCancel, this, _1),
      std::bind(&RoomExplorationServer::handleAccepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "room_exploration_server (boustrophedon, footprint) ready.");
  }

private:
  rclcpp_action::Server<RoomExploration>::SharedPtr action_server_;
  BoustrophedonExplorer boustrophedon_explorer_;

  double min_cell_area_;
  double path_eps_;
  double grid_obstacle_offset_;
  int max_deviation_from_track_;
  int cell_visiting_order_;
  std::string map_frame_;

  static constexpr int PLAN_FOR_FOOTPRINT = 1;

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID&,
                                         std::shared_ptr<const RoomExploration::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    // run on a separate thread so the executor stays responsive
    std::thread{std::bind(&RoomExplorationServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<RoomExploration::Result>();

    if (goal->planning_mode != PLAN_FOR_FOOTPRINT)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "planning_mode=%d not supported; this ROS2 port plans for footprint (mode 1) only.",
                   goal->planning_mode);
      goal_handle->abort(result);
      return;
    }

    // ---- convert the input map (mono8: 0=obstacle, 255=free) ----
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(goal->input_map, sensor_msgs::image_encodings::MONO8);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge conversion failed: %s", e.what());
      goal_handle->abort(result);
      return;
    }
    cv::Mat room_map = cv_ptr->image;

    const cv::Point2d map_origin(goal->map_origin.position.x, goal->map_origin.position.y);
    const float map_resolution = goal->map_resolution;  // [m/cell]
    const cv::Point starting_position(
      (goal->starting_position.x - map_origin.x) / map_resolution,
      (goal->starting_position.y - map_origin.y) / map_resolution);

    // square grid cell side length that fits into the coverage-radius circle
    const double grid_spacing_in_meter = goal->coverage_radius * std::sqrt(2.0);
    const double grid_spacing_in_pixel = grid_spacing_in_meter / map_resolution;
    const Eigen::Matrix<float, 2, 1> zero_vector(0.f, 0.f);

    RCLCPP_INFO(this->get_logger(),
                "planning coverage: map %dx%d, res %.3f m/cell, coverage_radius %.3f m (grid %.2f px)",
                room_map.cols, room_map.rows, map_resolution, goal->coverage_radius, grid_spacing_in_pixel);

    // ---- run the boustrophedon coverage planner (footprint) ----
    std::vector<geometry_msgs::Pose2D> exploration_path;  // POD type from the library
    try
    {
      boustrophedon_explorer_.getExplorationPath(
        room_map, exploration_path, map_resolution, starting_position, map_origin,
        grid_spacing_in_pixel, grid_obstacle_offset_, path_eps_, cell_visiting_order_,
        true /*plan_for_footprint*/, zero_vector, min_cell_area_, max_deviation_from_track_);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "coverage planning failed: %s", e.what());
      goal_handle->abort(result);
      return;
    }

    if (exploration_path.empty())
    {
      RCLCPP_WARN(this->get_logger(), "coverage planner returned an empty path.");
      goal_handle->abort(result);
      return;
    }

    // ---- build the result (POD -> ROS2 msgs) ----
    result->coverage_path.reserve(exploration_path.size());
    result->coverage_path_pose_stamped.reserve(exploration_path.size());
    const rclcpp::Time stamp = this->now();
    for (const auto& p : exploration_path)
    {
      geometry_msgs::msg::Pose2D pose2d;
      pose2d.x = p.x;
      pose2d.y = p.y;
      pose2d.theta = p.theta;
      result->coverage_path.push_back(pose2d);

      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = map_frame_;
      ps.header.stamp = stamp;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.position.z = 0.0;
      ps.pose.orientation.z = std::sin(p.theta / 2.0);
      ps.pose.orientation.w = std::cos(p.theta / 2.0);
      result->coverage_path_pose_stamped.push_back(ps);
    }

    RCLCPP_INFO(this->get_logger(), "coverage path computed: %zu waypoints.", exploration_path.size());
    goal_handle->succeed(result);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoomExplorationServer>());
  rclcpp::shutdown();
  return 0;
}
