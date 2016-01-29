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

#include <fstream>
#include <tuple>

#include <ros/console.h>

#include <tablet_socket/Waypoint.h>

#include <geo_pos_conv.hh>
#include <lane_planner/vmap.hpp>

namespace lane_planner {

namespace vmap {

namespace {

void write_waypoint(const map_file::PointClass& point, double yaw, double velocity, const std::string& path,
		    bool first);

double compute_direction_angle(const map_file::PointClass& p1, const map_file::PointClass& p2);

bool is_branching_point(const VectorMap& vmap, const map_file::PointClass& point);
bool is_merging_point(const VectorMap& vmap, const map_file::PointClass& point);
bool is_branching_lane(const map_file::Lane& lane);
bool is_merging_lane(const map_file::Lane& lane);

map_file::PointClass find_start_point(const VectorMap& vmap, const map_file::Lane& lane);
map_file::PointClass find_end_point(const VectorMap& vmap, const map_file::Lane& lane);
map_file::PointClass find_departure_point(const VectorMap& lane_vmap, int lno,
					  const std::vector<map_file::PointClass>& coarse_points,
					  double search_radius);
map_file::PointClass find_arrival_point(const VectorMap& lane_vmap, int lno,
					const std::vector<map_file::PointClass>& coarse_points,
					double search_radius);
map_file::PointClass find_nearest_point(const VectorMap& vmap, const map_file::PointClass& point);
std::vector<map_file::PointClass> find_near_points(const VectorMap& vmap, const map_file::PointClass& point,
						   double search_radius);

map_file::Lane find_lane(const VectorMap& vmap, int lno, const map_file::PointClass& point);
map_file::Lane find_prev_lane(const VectorMap& vmap, int lno, const map_file::Lane& lane);
map_file::Lane find_next_lane(const VectorMap& vmap, int lno, const map_file::Lane& lane);
map_file::Lane find_next_branching_lane(const VectorMap& vmap, int lno, const map_file::Lane& lane,
					double coarse_angle, double search_radius);

void write_waypoint(const map_file::PointClass& point, double yaw, double velocity, const std::string& path,
		    bool first)
{
	// reverse X-Y axis
	if (first) {
		std::ofstream ofs(path.c_str());
		ofs << std::fixed << point.ly << ","
		    << std::fixed << point.bx << ","
		    << std::fixed << point.h << ","
		    << std::fixed << yaw << std::endl;
	} else {
		std::ofstream ofs(path.c_str(), std::ios_base::app);
		ofs << std::fixed << point.ly << ","
		    << std::fixed << point.bx << ","
		    << std::fixed << point.h << ","
		    << std::fixed << yaw << ","
		    << std::fixed << velocity << std::endl;
	}
}

double compute_direction_angle(const map_file::PointClass& p1, const map_file::PointClass& p2)
{
	return (atan2(p2.ly - p1.ly, p2.bx - p1.bx) * (180 / M_PI)); // -180 to 180 degrees
}

bool is_branching_point(const VectorMap& vmap, const map_file::PointClass& point)
{
	map_file::Lane lane = find_lane(vmap, LNO_ALL, point);
	if (lane.lnid < 0)
		return false;

	lane = find_prev_lane(vmap, LNO_ALL, lane);
	if (lane.lnid < 0)
		return false;

	return is_branching_lane(lane);
}

bool is_merging_point(const VectorMap& vmap, const map_file::PointClass& point)
{
	map_file::Lane lane = find_lane(vmap, LNO_ALL, point);
	if (lane.lnid < 0)
		return false;

	return is_merging_lane(lane);
}

bool is_branching_lane(const map_file::Lane& lane)
{
	return (lane.jct == 1 || lane.jct == 2 || lane.jct == 5);
}

bool is_merging_lane(const map_file::Lane& lane)
{
	return (lane.jct == 3 || lane.jct == 4 || lane.jct == 5);
}

map_file::PointClass find_start_point(const VectorMap& vmap, const map_file::Lane& lane)
{
	map_file::PointClass error;
	error.pid = -1;

	for (const map_file::Node& n : vmap.nodes) {
		if (n.nid != lane.bnid)
			continue;
		for (const map_file::PointClass& p : vmap.points) {
			if (p.pid != n.pid)
				continue;
			return p;
		}
	}

	return error;
}

map_file::PointClass find_end_point(const VectorMap& vmap, const map_file::Lane& lane)
{
	map_file::PointClass error;
	error.pid = -1;

	for (const map_file::Node& n : vmap.nodes) {
		if (n.nid != lane.fnid)
			continue;
		for (const map_file::PointClass& p : vmap.points) {
			if (p.pid != n.pid)
				continue;
			return p;
		}
	}

	return error;
}

map_file::PointClass find_departure_point(const VectorMap& lane_vmap, int lno,
					  const std::vector<map_file::PointClass>& coarse_points,
					  double search_radius)
{
	map_file::PointClass coarse_p1 = coarse_points[0];
	map_file::PointClass coarse_p2 = coarse_points[1];

	map_file::PointClass nearest_point = find_nearest_point(lane_vmap, coarse_p1);
	if (nearest_point.pid < 0)
		return nearest_point;

	std::vector<map_file::PointClass> near_points = find_near_points(lane_vmap, coarse_p1, search_radius);
	double coarse_angle = compute_direction_angle(coarse_p1, coarse_p2);
	double score = 180 + search_radius; // XXX better way?
	for (const map_file::PointClass& p1 : near_points) {
		map_file::Lane l = find_lane(lane_vmap, lno, p1);
		if (l.lnid < 0)
			continue;

		map_file::PointClass p2 = find_end_point(lane_vmap, l);
		if (p2.pid < 0)
			continue;

		double a = compute_direction_angle(p1, p2);
		a = fabs(a - coarse_angle);
		if (a > 180)
			a = fabs(a - 360);
		double d = hypot(p1.bx - coarse_p1.bx, p1.ly - coarse_p1.ly);
		double s = a + d;
		if (s <= score) {
			nearest_point = p1;
			score = s;
		}
	}

	return nearest_point;
}

map_file::PointClass find_arrival_point(const VectorMap& lane_vmap, int lno,
					const std::vector<map_file::PointClass>& coarse_points,
					double search_radius)
{
	map_file::PointClass coarse_p1 = coarse_points[coarse_points.size() - 1];
	map_file::PointClass coarse_p2 = coarse_points[coarse_points.size() - 2];

	map_file::PointClass nearest_point = find_nearest_point(lane_vmap, coarse_p1);
	if (nearest_point.pid < 0)
		return nearest_point;

	std::vector<map_file::PointClass> near_points = find_near_points(lane_vmap, coarse_p1, search_radius);
	double coarse_angle = compute_direction_angle(coarse_p1, coarse_p2);
	double score = 180 + search_radius; // XXX better way?
	for (const map_file::PointClass& p1 : near_points) {
		map_file::Lane l = find_lane(lane_vmap, lno, p1);
		if (l.lnid < 0)
			continue;

		l = find_prev_lane(lane_vmap, lno, l);
		if (l.lnid < 0)
			continue;

		map_file::PointClass p2 = find_start_point(lane_vmap, l);
		if (p2.pid < 0)
			continue;

		double a = compute_direction_angle(p1, p2);
		a = fabs(a - coarse_angle);
		if (a > 180)
			a = fabs(a - 360);
		double d = hypot(p1.bx - coarse_p1.bx, p1.ly - coarse_p1.ly);
		double s = a + d;
		if (s <= score) {
			nearest_point = p1;
			score = s;
		}
	}

	return nearest_point;
}

map_file::PointClass find_nearest_point(const VectorMap& vmap, const map_file::PointClass& point)
{
	map_file::PointClass nearest_point;
	nearest_point.pid = -1;

	double distance = DBL_MAX;
	for (const map_file::PointClass& p : vmap.points) {
		double d = hypot(p.bx - point.bx, p.ly - point.ly);
		if (d <= distance) {
			nearest_point = p;
			distance = d;
		}
	}

	return nearest_point;
}

std::vector<map_file::PointClass> find_near_points(const VectorMap& vmap, const map_file::PointClass& point,
						   double search_radius)
{
	std::vector<map_file::PointClass> near_points;
	for (const map_file::PointClass& p : vmap.points) {
		double d = hypot(p.bx - point.bx, p.ly - point.ly);
		if (d <= search_radius)
			near_points.push_back(p);
	}

	return near_points;
}

map_file::Lane find_lane(const VectorMap& vmap, int lno, const map_file::PointClass& point)
{
	map_file::Lane error;
	error.lnid = -1;

	for (const map_file::Node& n : vmap.nodes) {
		if (n.pid != point.pid)
			continue;
		for (const map_file::Lane& l : vmap.lanes) {
			if (lno != LNO_ALL && l.lno != lno)
				continue;
			if (l.bnid != n.nid)
				continue;
			return l;
		}
	}

	return error;
}

map_file::Lane find_prev_lane(const VectorMap& vmap, int lno, const map_file::Lane& lane)
{
	map_file::Lane error;
	error.lnid = -1;

	if (is_merging_lane(lane)) {
		for (const map_file::Lane& l : vmap.lanes) {
			if (lno != LNO_ALL && l.lno != lno)
				continue;
			if (l.lnid != lane.blid && l.lnid != lane.blid2 && l.lnid != lane.blid3 &&
			    l.lnid != lane.blid4)
				continue;
			return l;
		}
	} else {
		for (const map_file::Lane& l : vmap.lanes) {
			if (l.lnid != lane.blid)
				continue;
			return l;
		}
	}

	return error;
}

map_file::Lane find_next_lane(const VectorMap& vmap, int lno, const map_file::Lane& lane)
{
	map_file::Lane error;
	error.lnid = -1;

	if (is_branching_lane(lane)) {
		for (const map_file::Lane& l : vmap.lanes) {
			if (lno != LNO_ALL && l.lno != lno)
				continue;
			if (l.lnid != lane.flid && l.lnid != lane.flid2 && l.lnid != lane.flid3 &&
			    l.lnid != lane.flid4)
				continue;
			return l;
		}
	} else {
		for (const map_file::Lane& l : vmap.lanes) {
			if (l.lnid != lane.flid)
				continue;
			return l;
		}
	}

	return error;
}

map_file::Lane find_next_branching_lane(const VectorMap& vmap, int lno, const map_file::Lane& lane,
					double coarse_angle, double search_radius)
{
	map_file::Lane error;
	error.lnid = -1;

	map_file::PointClass p1 = find_end_point(vmap, lane);
	if (p1.pid < 0)
		return error;

	std::vector<std::tuple<map_file::PointClass, map_file::Lane>> candidates;
	for (const map_file::Lane& l1 : vmap.lanes) {
		if (lno != LNO_ALL && l1.lno != lno)
			continue;
		if (l1.lnid == lane.flid || l1.lnid == lane.flid2 || l1.lnid == lane.flid3 || l1.lnid == lane.flid4) {
			map_file::Lane l2 = l1;
			map_file::PointClass p = find_end_point(vmap, l2);
			if (p.pid < 0)
				continue;
			map_file::PointClass p2 = p;
			double d = hypot(p2.bx - p1.bx, p2.ly - p1.ly);
			while (d <= search_radius && l2.flid != 0 && !is_branching_lane(l2)) {
				l2 = find_next_lane(vmap, LNO_ALL, l2);
				if (l2.lnid < 0)
					break;
				p = find_end_point(vmap, l2);
				if (p.pid < 0)
					break;
				p2 = p;
				d = hypot(p2.bx - p1.bx, p2.ly - p1.ly);
			}
			candidates.push_back(std::make_tuple(p2, l1));
		}
	}

	if (candidates.empty())
		return error;

	map_file::Lane branching_lane;
	double angle = 180;
	for (const std::tuple<map_file::PointClass, map_file::Lane>& c : candidates) {
		map_file::PointClass p2 = std::get<0>(c);
		double a = compute_direction_angle(p1, p2);
		a = fabs(a - coarse_angle);
		if (a > 180)
			a = fabs(a - 360);
		if (a <= angle) {
			branching_lane = std::get<1>(c);
			angle = a;
		}
	}

	return branching_lane;
}

} // namespace

void write_waypoints(const std::vector<map_file::PointClass>& points, double velocity, const std::string& path)
{
	if (points.size() < 2)
		return;

	size_t last_index = points.size() - 1;
	for (size_t i = 0; i < points.size(); ++i) {
		double yaw;
		if (i == last_index) {
			geometry_msgs::Point p1 = create_geometry_msgs_point(points[i]);
			geometry_msgs::Point p2 = create_geometry_msgs_point(points[i - 1]);
			yaw = atan2(p2.y - p1.y, p2.x - p1.x);
			yaw -= M_PI;
		} else {
			geometry_msgs::Point p1 = create_geometry_msgs_point(points[i]);
			geometry_msgs::Point p2 = create_geometry_msgs_point(points[i + 1]);
			yaw = atan2(p2.y - p1.y, p2.x - p1.x);
		}

		write_waypoint(points[i], yaw, velocity, path, (i == 0));
	}
}

double compute_reduction(const map_file::DTLane& d, double w)
{
	return (1 - fabs(1 / d.r) * w); // 0 to 1 rates
}

bool is_straight_dtlane(const map_file::DTLane& dtlane)
{
	return (dtlane.apara == 0 && dtlane.r == RADIUS_MAX);
}

bool is_curve_dtlane(const map_file::DTLane& dtlane)
{
	return (dtlane.apara == 0 && dtlane.r != RADIUS_MAX);
}

// XXX better way?
bool is_crossroad_dtlane(const map_file::DTLane& dtlane)
{
	// take crossroad for 10 radius or less
	return (fabs(dtlane.r) <= 10);
}

bool is_clothoid_dtlane(const map_file::DTLane& dtlane)
{
	return (dtlane.apara != 0);
}

// XXX better way?
bool is_connection_dtlane(const VectorMap& fine_vmap, int index)
{
	const map_file::DTLane& dtlane = fine_vmap.dtlanes[index];
	int size = fine_vmap.dtlanes.size();

	int change = 0;
	int straight = 0;
	for (int i = index - 1; i >= 0; --i) {
		if (dtlane.r != fine_vmap.dtlanes[i].r) {
			++change;
			if (is_straight_dtlane(fine_vmap.dtlanes[i]))
				++straight;
			break;
		}
	}
	for (int i = index + 1; i < size; ++i) {
		if (dtlane.r != fine_vmap.dtlanes[i].r) {
			++change;
			if (is_straight_dtlane(fine_vmap.dtlanes[i]))
				++straight;
			break;
		}
	}
	if (change == 1 && straight == 1)
		return true;
	if (straight == 2)
		return true;

	return false;
}

geometry_msgs::Point create_geometry_msgs_point(const map_file::PointClass& mp)
{
	// reverse X-Y axis
	geometry_msgs::Point gp;
	gp.x = mp.ly;
	gp.y = mp.bx;
	gp.z = mp.h;

	return gp;
}

map_file::PointClass create_map_file_pointclass(const geometry_msgs::Point& gp)
{
	// reverse X-Y axis
	map_file::PointClass mp;
	mp.bx = gp.y;
	mp.ly = gp.x;
	mp.h = gp.z;

	return mp;
}

waypoint_follower::dtlane create_waypoint_follower_dtlane(const map_file::DTLane& md)
{
	waypoint_follower::dtlane wd;
	wd.dist = md.dist;
	wd.dir = md.dir;
	wd.apara = md.apara;
	wd.r = md.r;
	wd.slope = md.slope;
	wd.cant = md.cant;
	wd.lw = md.lw;
	wd.rw = md.rw;

	return wd;
}

map_file::DTLane create_map_file_dtlane(const waypoint_follower::dtlane& wd)
{
	map_file::DTLane md;
	md.dist = wd.dist;
	md.dir = wd.dir;
	md.apara = wd.apara;
	md.r = wd.r;
	md.slope = wd.slope;
	md.cant = wd.cant;
	md.lw = wd.lw;
	md.rw = wd.rw;

	return md;
}

VectorMap create_lane_vmap(const VectorMap& vmap, int lno)
{
	VectorMap lane_vmap;
	for (const map_file::Lane& l : vmap.lanes) {
		if (lno != LNO_ALL && l.lno != lno)
			continue;
		lane_vmap.lanes.push_back(l);

		for (const map_file::Node& n : vmap.nodes) {
			if (n.nid != l.bnid && n.nid != l.fnid)
				continue;
			lane_vmap.nodes.push_back(n);

			for (const map_file::PointClass& p : vmap.points) {
				if (p.pid != n.pid)
					continue;
				lane_vmap.points.push_back(p);
			}
		}

		for (const map_file::StopLine& s : vmap.stoplines) {
			if (s.linkid != l.lnid)
				continue;
			lane_vmap.stoplines.push_back(s);
		}

		for (const map_file::DTLane& d : vmap.dtlanes) {
			if (d.did != l.did)
				continue;
			lane_vmap.dtlanes.push_back(d);
		}
	}

	return lane_vmap;
}

VectorMap create_coarse_vmap_from_lane(const waypoint_follower::lane& lane)
{
	VectorMap coarse_vmap;
	for (const waypoint_follower::waypoint& w : lane.waypoints)
		coarse_vmap.points.push_back(create_map_file_pointclass(w.pose.pose.position));

	return coarse_vmap;
}

VectorMap create_coarse_vmap_from_route(const tablet_socket::route_cmd& route)
{
	geo_pos_conv geo;
	geo.set_plane(7);

	VectorMap coarse_vmap;
	for (const tablet_socket::Waypoint& w : route.point) {
		geo.llh_to_xyz(w.lat, w.lon, 0);

		map_file::PointClass p;
		p.bx = geo.x();
		p.ly = geo.y();
		coarse_vmap.points.push_back(p);
	}

	return coarse_vmap;
}

VectorMap create_fine_vmap(const VectorMap& lane_vmap, int lno, const VectorMap& coarse_vmap, double search_radius,
			   int waypoint_max)
{
	VectorMap fine_vmap;
	VectorMap null_vmap;

	map_file::PointClass departure_point;
	departure_point.pid = -1;
	if (lno == LNO_ALL)
		departure_point = find_nearest_point(lane_vmap, coarse_vmap.points.front());
	else {
		for (int i = lno; i >= LNO_CROSSING; --i) {
			departure_point = find_departure_point(lane_vmap, i, coarse_vmap.points, search_radius);
			if (departure_point.pid >= 0)
				break;
		}
	}
	if (departure_point.pid < 0)
		return null_vmap;

	map_file::PointClass arrival_point;
	arrival_point.pid = -1;
	if (lno == LNO_ALL)
		arrival_point = find_nearest_point(lane_vmap, coarse_vmap.points.back());
	else {
		for (int i = lno; i >= LNO_CROSSING; --i) {
			arrival_point = find_arrival_point(lane_vmap, i, coarse_vmap.points, search_radius);
			if (arrival_point.pid >= 0)
				break;
		}
	}
	if (arrival_point.pid < 0)
		return null_vmap;

	map_file::PointClass point = departure_point;
	map_file::Lane lane = find_lane(lane_vmap, LNO_ALL, point);
	if (lane.lnid < 0)
		return null_vmap;

	bool finish = false;
	for (int i = 0; i < waypoint_max; ++i) {
		fine_vmap.points.push_back(point);

		// last is equal to previous dtlane
		map_file::DTLane dtlane;
		dtlane.did = -1;
		for (const map_file::DTLane& d : lane_vmap.dtlanes) {
			if (d.did == lane.did) {
				dtlane = d;
				break;
			}
		}
		fine_vmap.dtlanes.push_back(dtlane);

		// last is equal to previous stopline
		map_file::StopLine stopline;
		stopline.id = -1;
		for (const map_file::StopLine& s : lane_vmap.stoplines) {
			if (s.linkid == lane.lnid) {
				stopline = s;
				break;
			}
		}
		fine_vmap.stoplines.push_back(stopline);

		if (finish)
			break;

		fine_vmap.lanes.push_back(lane);

		point = find_end_point(lane_vmap, lane);
		if (point.pid < 0)
			return null_vmap;
		if (point.bx == arrival_point.bx && point.ly == arrival_point.ly) {
			finish = true;
			continue;
		}

		if (is_branching_lane(lane)) {
			map_file::PointClass coarse_p1 = find_end_point(lane_vmap, lane);
			if (coarse_p1.pid < 0)
				return null_vmap;

			coarse_p1 = find_nearest_point(coarse_vmap, coarse_p1);
			if (coarse_p1.pid < 0)
				return null_vmap;

			map_file::PointClass coarse_p2;
			double distance = -1;
			for (const map_file::PointClass& p : coarse_vmap.points) {
				if (distance == -1) {
					if (p.bx == coarse_p1.bx && p.ly == coarse_p1.ly)
						distance = 0;
					continue;
				}
				coarse_p2 = p;
				distance = hypot(coarse_p2.bx - coarse_p1.bx, coarse_p2.ly - coarse_p1.ly);
				if (distance > search_radius)
					break;
			}
			if (distance <= 0)
				return null_vmap;

			double coarse_angle = compute_direction_angle(coarse_p1, coarse_p2);
			if (lno == LNO_ALL) {
				lane = find_next_branching_lane(lane_vmap, LNO_ALL, lane, coarse_angle, search_radius);
			} else {
				map_file::Lane l;
				l.lnid = -1;
				for (int j = lno; j >= LNO_CROSSING; --j) {
					l = find_next_branching_lane(lane_vmap, j, lane, coarse_angle, search_radius);
					if (l.lnid >= 0)
						break;
				}
				lane = l;
			}
		} else {
			lane = find_next_lane(lane_vmap, LNO_ALL, lane);
		}
		if (lane.lnid < 0)
			return null_vmap;
	}

	if (!finish) {
		ROS_ERROR_STREAM("lane is too long");
		return null_vmap;
	}

	return fine_vmap;
}

std::vector<map_file::PointClass> create_branching_points(const VectorMap& vmap)
{
	std::vector<map_file::PointClass> branching_points;
	for (const map_file::PointClass& p : vmap.points) {
		if (!is_branching_point(vmap, p))
			continue;
		branching_points.push_back(p);
	}

	return branching_points;
}

std::vector<map_file::PointClass> create_merging_points(const VectorMap& vmap)
{
	std::vector<map_file::PointClass> merging_points;
	for (const map_file::PointClass& p : vmap.points) {
		if (!is_merging_point(vmap, p))
			continue;
		merging_points.push_back(p);
	}

	return merging_points;
}

void publish_add_marker(const ros::Publisher& pub, const visualization_msgs::Marker& marker,
			const std::vector<map_file::PointClass>& points)
{
	visualization_msgs::Marker m;
	m.header.frame_id = marker.header.frame_id;
	m.ns = marker.ns;
	m.id = marker.id;
	m.type = marker.type;
	m.scale = marker.scale;
	m.color = marker.color;
	m.frame_locked = marker.frame_locked;
	for (const map_file::PointClass& p : points)
		m.points.push_back(create_geometry_msgs_point(p));

	m.header.stamp = ros::Time::now();
	m.action = visualization_msgs::Marker::ADD;

	pub.publish(m);
}

void publish_delete_marker(const ros::Publisher& pub, const visualization_msgs::Marker& marker)
{
	visualization_msgs::Marker m;
	m.header.frame_id = marker.header.frame_id;
	m.ns = marker.ns;
	m.id = marker.id;

	m.header.stamp = ros::Time::now();
	m.action = visualization_msgs::Marker::DELETE;

	pub.publish(m);
}

} // namespace vmap

} // namespace lane_planner
