/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef LANE_PLANNER_VMAP_HPP
#define LANE_PLANNER_VMAP_HPP

#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <vector_map/vector_map.h>
#include <tablet_socket_msgs/route_cmd.h>
#include "autoware_msgs/DTLane.h"
#include "autoware_msgs/Lane.h"

namespace lane_planner {

namespace vmap {

constexpr int LNO_ALL = -1;
constexpr int LNO_CROSSING = 0;
constexpr int LNO_MOSTLEFT = 1;

constexpr int TRAFFIC_LIGHT_RED = 0;
constexpr int TRAFFIC_LIGHT_GREEN = 1;
constexpr int TRAFFIC_LIGHT_UNKNOWN = 2;

constexpr double RADIUS_MAX = 90000000000;

struct VectorMap {
	std::vector<vector_map::Point> points;
	std::vector<vector_map::Lane> lanes;
	std::vector<vector_map::Node> nodes;
	std::vector<vector_map::StopLine> stoplines;
	std::vector<vector_map::DTLane> dtlanes;
};

void write_waypoints(const std::vector<vector_map::Point>& points, double velocity, const std::string& path);

double compute_reduction(const vector_map::DTLane& d, double w);

bool is_straight_dtlane(const vector_map::DTLane& dtlane);
bool is_curve_dtlane(const vector_map::DTLane& dtlane);
bool is_crossroad_dtlane(const vector_map::DTLane& dtlane);
bool is_clothoid_dtlane(const vector_map::DTLane& dtlane);
bool is_connection_dtlane(const VectorMap& fine_vmap, int index);

geometry_msgs::Point create_geometry_msgs_point(const vector_map::Point& vp);
vector_map::Point create_vector_map_point(const geometry_msgs::Point& gp);
autoware_msgs::DTLane create_waypoint_follower_dtlane(const vector_map::DTLane& vd);
vector_map::DTLane create_vector_map_dtlane(const autoware_msgs::DTLane& wd);

VectorMap create_lane_vmap(const VectorMap& vmap, int lno);
VectorMap create_coarse_vmap_from_lane(const autoware_msgs::Lane& lane);
VectorMap create_coarse_vmap_from_route(const tablet_socket_msgs::route_cmd& route);
VectorMap create_fine_vmap(const VectorMap& lane_vmap, int lno, const VectorMap& coarse_vmap, double search_radius,
			   int waypoint_max);

std::vector<vector_map::Point> create_branching_points(const VectorMap& vmap);
std::vector<vector_map::Point> create_merging_points(const VectorMap& vmap);

void publish_add_marker(const ros::Publisher& pub, const visualization_msgs::Marker& marker,
			const std::vector<vector_map::Point>& points);
void publish_delete_marker(const ros::Publisher& pub, const visualization_msgs::Marker& marker);

} // namespace vmap

} // namespace lane_planner

#endif // LANE_PLANNER_VMAP_HPP
