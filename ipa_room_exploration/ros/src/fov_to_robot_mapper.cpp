/*
 * fov_to_robot_mapper.cpp — maps a field-of-view path to robot poses.
 * Copyright (c) 2016 Fraunhofer Institute for Manufacturing Engineering and
 * Automation (IPA). Authors: Florian Jordan, Richard Bormann.
 * Licensed under the GNU Lesser General Public License (LGPL) v3 or later.
 *
 * rena-patch (ROS2 port): the cob_map_accessibility_analysis branch (perimeter
 * accessibility analysis) is removed. mapPath now uses a directly computed pose
 * shift, then an A* fallback — both cob-free. Footprint-mode coverage does not
 * call mapPath() at all. computeFOVCenterAndRadius() is unchanged.
 */

#include <ipa_room_exploration/fov_to_robot_mapper.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

void mapPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& robot_path,
		const std::vector<geometry_msgs::Pose2D>& fov_path, const Eigen::Matrix<float, 2, 1>& robot_to_fov_vector,
		const double map_resolution, const cv::Point2d map_origin, const cv::Point& starting_point)
{
	// initialize helper classes
	AStarPlanner path_planner;
	const double map_resolution_inv = 1.0/map_resolution;

	// initialize the robot position in accessible space to enable the Astar planner to find a path from the beginning
	cv::Point robot_pos(starting_point.x, starting_point.y);

	// map the given robot to fov vector into pixel coordinates
	Eigen::Matrix<float, 2, 1> robot_to_fov_vector_pixel;
	robot_to_fov_vector_pixel << robot_to_fov_vector(0,0)*map_resolution_inv, robot_to_fov_vector(1,0)*map_resolution_inv;
	const double fov_radius_pixel = robot_to_fov_vector_pixel.norm();
	const double fov_to_front_offset_angle = atan2((double)robot_to_fov_vector(1,0), (double)robot_to_fov_vector(0,0));

	// go through the given poses and calculate accessible robot poses
	int found_with_astar = 0, found_with_shift = 0, not_found = 0;
	for(std::vector<geometry_msgs::Pose2D>::const_iterator pose=fov_path.begin(); pose!=fov_path.end(); ++pose)
	{
		bool found_pose = false;

		// 1. try with a directly computed pose shift
		{
			const float sin_theta = std::sin(pose->theta);
			const float cos_theta = std::cos(pose->theta);
			Eigen::Matrix<float, 2, 2> R;
			R << cos_theta, -sin_theta, sin_theta, cos_theta;

			// calculate the resulting rotated relative vector and the corresponding robot position
			Eigen::Matrix<float, 2, 1> v_rel_rot = R * robot_to_fov_vector_pixel;
			Eigen::Matrix<float, 2, 1> robot_position;
			robot_position << pose->x-v_rel_rot(0,0), pose->y-v_rel_rot(1,0);

			// check the accessibility of the found point
			geometry_msgs::Pose2D current_pose;
			if(robot_position(0,0) >= 0 && robot_position(1,0) >= 0 && robot_position(0,0) < room_map.cols &&
					robot_position(1,0) < room_map.rows && room_map.at<uchar>((int)robot_position(1,0), (int)robot_position(0,0)) == 255) // accessible
			{
				current_pose.x = (robot_position(0,0) * map_resolution) + map_origin.x;
				current_pose.y = (robot_position(1,0) * map_resolution) + map_origin.y;
				current_pose.theta = pose->theta;
				found_pose = true;
				robot_path.push_back(current_pose);
				robot_pos = cv::Point((int)robot_position(0,0), (int)robot_position(1,0));
				++found_with_shift;
			}
		}

		// 2. fallback: compute the A* path from robot position to fov_center and stop at the right distance
		if (found_pose==false)
		{
			cv::Point fov_position(pose->x, pose->y);
			std::vector<cv::Point> astar_path;
			path_planner.planPath(room_map, robot_pos, fov_position, 1.0, 0.0, map_resolution, 0, &astar_path);

			cv::Point accessible_position;
			for(std::vector<cv::Point>::iterator point=astar_path.begin(); point!=astar_path.end(); ++point)
			{
				if(cv::norm(*point-fov_position) <= fov_radius_pixel)
				{
					accessible_position = *point;
					found_pose = true;
					break;
				}
			}

			if(found_pose == true)
			{
				geometry_msgs::Pose2D current_pose;
				current_pose.x = (accessible_position.x * map_resolution) + map_origin.x;
				current_pose.y = (accessible_position.y * map_resolution) + map_origin.y;
				current_pose.theta = std::atan2(pose->y-accessible_position.y, pose->x-accessible_position.x) - fov_to_front_offset_angle;
				robot_path.push_back(current_pose);
				robot_pos = accessible_position;
				++found_with_astar;
			}
		}

		if (found_pose==false)
			++not_found;
	}
	std::cout << "mapPath: found with shift: " << found_with_shift << ",   with A*: " << found_with_astar
			<< ",   not found: " << not_found << std::endl;
}


// computes the field of view center and the radius of the maximum incircle of a given field of view quadrilateral
// the metric coordinates are relative to the robot base coordinate system (i.e. the robot center)
// coordinate system definition: x points in forward direction of robot and camera, y points to the left side  of the robot and z points upwards. x and y span the ground plane.
// fitting_circle_center_point_in_meter this is also considered the center of the field of view, because around this point the maximum radius incircle can be found that is still inside the fov
// fov_resolution resolution of the fov center and incircle computations, in [pixels/m]
void computeFOVCenterAndRadius(const std::vector<Eigen::Matrix<float, 2, 1> >& fov_corners_meter,
		float& fitting_circle_radius_in_meter, Eigen::Matrix<float, 2, 1>& fitting_circle_center_point_in_meter, const double fov_resolution)
{
	// The general solution for the largest incircle is to find the critical points of a Voronoi graph and select the one
	// with largest distance to the sides as circle center and its closest distance to the quadrilateral sides as radius
	// see: http://math.stackexchange.com/questions/1948356/largest-incircle-inside-a-quadrilateral-radius-calculation
	// -------------> easy solution: distance transform on fov, take max. value that is closest to center

	// read out field of view and convert to pixel coordinates, determine min, max and center coordinates
	std::vector<cv::Point> fov_corners_pixel;
	cv::Point center_point_pixel(0,0);
	cv::Point min_point(100000, 100000);
	cv::Point max_point(-100000, -100000);
	for(int i = 0; i < 4; ++i)
	{
		fov_corners_pixel.push_back(cv::Point(fov_corners_meter[i](0,0)*fov_resolution, fov_corners_meter[i](1,0)*fov_resolution));
		center_point_pixel += fov_corners_pixel.back();
		min_point.x = std::min(min_point.x, fov_corners_pixel.back().x);
		min_point.y = std::min(min_point.y, fov_corners_pixel.back().y);
		max_point.x = std::max(max_point.x, fov_corners_pixel.back().x);
		max_point.y = std::max(max_point.y, fov_corners_pixel.back().y);
	}
	center_point_pixel.x = center_point_pixel.x/4 - min_point.x + 1;
	center_point_pixel.y = center_point_pixel.y/4 - min_point.y + 1;

	// draw an image of the field of view and compute a distance transform
	cv::Mat fov_image = cv::Mat::zeros(4+max_point.y-min_point.y, 4+max_point.x-min_point.x, CV_8UC1);
	std::vector<std::vector<cv::Point> > polygon_array(1,fov_corners_pixel);
	for (size_t i=0; i<polygon_array[0].size(); ++i)
	{
		polygon_array[0][i].x -= min_point.x-1;
		polygon_array[0][i].y -= min_point.y-1;
	}
	cv::fillPoly(fov_image, polygon_array, cv::Scalar(255));
	cv::Mat fov_distance_transform;
#if CV_MAJOR_VERSION<=3
	cv::distanceTransform(fov_image, fov_distance_transform, CV_DIST_L2, CV_DIST_MASK_PRECISE);
#else
	cv::distanceTransform(fov_image, fov_distance_transform, cv::DIST_L2, cv::DIST_MASK_PRECISE);
#endif

	// determine the point(s) with maximum distance to the rim of the field of view, if multiple points apply, take the one closest to the center
	float max_dist_val = 0.;
	cv::Point max_dist_point(0,0);
	double center_dist = 1e10;
	for (int v=0; v<fov_distance_transform.rows; ++v)
	{
		for (int u=0; u<fov_distance_transform.cols; ++u)
		{
			if (fov_distance_transform.at<float>(v,u)>max_dist_val)
			{
				max_dist_val = fov_distance_transform.at<float>(v,u);
				max_dist_point = cv::Point(u,v);
				center_dist = (center_point_pixel.x-u)*(center_point_pixel.x-u) + (center_point_pixel.y-v)*(center_point_pixel.y-v);
			}
			else if (fov_distance_transform.at<float>(v,u) == max_dist_val)
			{
				double cdist = (center_point_pixel.x-u)*(center_point_pixel.x-u) + (center_point_pixel.y-v)*(center_point_pixel.y-v);
				if (cdist < center_dist)
				{
					max_dist_val = fov_distance_transform.at<float>(v,u);
					max_dist_point = cv::Point(u,v);
					center_dist = cdist;
				}
			}
		}
	}

	// compute fitting_circle_radius and center point (round last digit)
	fitting_circle_radius_in_meter = 10*(int)(((max_dist_val-1.f)+5)*0.1) / fov_resolution;
	std::cout << "fitting_circle_radius: " << fitting_circle_radius_in_meter << " m" << std::endl;
	cv::Point2d center_point = cv::Point2d(10*(int)(((max_dist_point.x+min_point.x-1)+5.)*0.1), 10*(int)(((max_dist_point.y+min_point.y-1)+5.)*0.1))*(1./fov_resolution);
	std::cout << "center point: " << center_point << " m" << std::endl;
	fitting_circle_center_point_in_meter << center_point.x, center_point.y;
}
