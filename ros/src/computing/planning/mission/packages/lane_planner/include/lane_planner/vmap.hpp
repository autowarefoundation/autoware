/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef LANE_PLANNER_VMAP_HPP
#define LANE_PLANNER_VMAP_HPP

#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <map_file/PointClass.h>
#include <map_file/Lane.h>
#include <map_file/Node.h>
#include <map_file/StopLine.h>
#include <map_file/DTLane.h>
#include <tablet_socket/route_cmd.h>
#include <waypoint_follower/dtlane.h>
#include <waypoint_follower/lane.h>

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
	std::vector<map_file::PointClass> points;
	std::vector<map_file::Lane> lanes;
	std::vector<map_file::Node> nodes;
	std::vector<map_file::StopLine> stoplines;
	std::vector<map_file::DTLane> dtlanes;
};

void write_waypoints(const std::vector<map_file::PointClass>& points, double velocity, const std::string& path);

double compute_reduction(const map_file::DTLane& d, double w);

bool is_straight_dtlane(const map_file::DTLane& dtlane);
bool is_curve_dtlane(const map_file::DTLane& dtlane);
bool is_crossroad_dtlane(const map_file::DTLane& dtlane);
bool is_clothoid_dtlane(const map_file::DTLane& dtlane);
bool is_connection_dtlane(const VectorMap& fine_vmap, int index);

geometry_msgs::Point create_geometry_msgs_point(const map_file::PointClass& mp);
map_file::PointClass create_map_file_pointclass(const geometry_msgs::Point& gp);
waypoint_follower::dtlane create_waypoint_follower_dtlane(const map_file::DTLane& md);
map_file::DTLane create_map_file_dtlane(const waypoint_follower::dtlane& wd);

VectorMap create_lane_vmap(const VectorMap& vmap, int lno);
VectorMap create_coarse_vmap_from_lane(const waypoint_follower::lane& lane);
VectorMap create_coarse_vmap_from_route(const tablet_socket::route_cmd& route);
VectorMap create_fine_vmap(const VectorMap& lane_vmap, int lno, const VectorMap& coarse_vmap, double search_radius,
			   int waypoint_max);

std::vector<map_file::PointClass> create_branching_points(const VectorMap& vmap);
std::vector<map_file::PointClass> create_merging_points(const VectorMap& vmap);

void publish_add_marker(const ros::Publisher& pub, const visualization_msgs::Marker& marker,
			const std::vector<map_file::PointClass>& points);
void publish_delete_marker(const ros::Publisher& pub, const visualization_msgs::Marker& marker);

} // namespace vmap

} // namespace lane_planner

#endif // LANE_PLANNER_VMAP_HPP
