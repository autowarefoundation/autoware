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

#include <cmath>
#include <tuple>

#include <vmap_utility.hpp>

map_file::Lane vmap_find_lane(const VectorMap& vmap, const map_file::PointClass& point)
{
	map_file::Lane error;
	error.lnid = -1;

	for (const map_file::Node& n : vmap.nodes) {
		if (n.pid != point.pid)
			continue;
		for (const map_file::Lane& l : vmap.lanes) {
			if (l.bnid != n.nid)
				continue;
			return l;
		}
	}

	return error;
}

map_file::Lane vmap_find_prev_lane(const VectorMap& vmap, const map_file::Lane& lane)
{
	map_file::Lane error;
	error.lnid = -1;

	for (const map_file::Lane& l : vmap.lanes) {
		if (l.lnid != lane.blid)
			continue;
		return l;
	}

	return error;
}

map_file::Lane vmap_find_next_lane(const VectorMap& vmap, const map_file::Lane& lane)
{
	map_file::Lane error;
	error.lnid = -1;

	for (const map_file::Lane& l : vmap.lanes) {
		if (l.lnid != lane.flid)
			continue;
		return l;
	}

	return error;
}

map_file::Lane vmap_find_junction_lane(const VectorMap& vmap, const map_file::Lane& lane, double angle)
{
	map_file::Lane error;
	error.lnid = -1;

	map_file::PointClass p1 = vmap_find_end_point(vmap, lane);
	if (p1.pid < 0)
		return error;

	std::vector<std::tuple<map_file::PointClass, map_file::Lane>> tuples;
	for (const map_file::Lane& l : vmap.lanes) {
		if (l.lnid == lane.flid || l.lnid == lane.flid2 || l.lnid == lane.flid3 || l.lnid == lane.flid4) {
			map_file::Lane p2_lane = l;
			map_file::PointClass p2 = vmap_find_end_point(vmap, p2_lane);
			if (p2.pid < 0)
				continue;
			double distance = hypot(p1.bx - p2.bx, p1.ly - p2.ly);
			while (p2_lane.flid != 0 && (p2_lane.jct < 1 || p2_lane.jct > 2) &&
			       distance <= VMAP_JUNCTION_DISTANCE_MAX) {
				p2_lane = vmap_find_next_lane(vmap, p2_lane);
				if (p2_lane.lnid < 0)
					break;
				map_file::PointClass p = vmap_find_end_point(vmap, p2_lane);
				if (p.pid < 0)
					break;
				p2 = p;
				distance = hypot(p1.bx - p2.bx, p1.ly - p2.ly);
			}
			tuples.push_back(std::make_tuple(p2, l));
		}
	}

	if (tuples.empty())
		return error;

	map_file::Lane junction_lane;
	double min = VMAP_ANGLE_MAX;
	for (const std::tuple<map_file::PointClass, map_file::Lane>& t : tuples) {
		map_file::PointClass p2 = std::get<0>(t);
		double a = vmap_compute_direction_angle(p1, p2);
		a = fabs(a - angle);
		if (a > 180)
			a = fabs(a - 360);
		if (a <= min) {
			min = a;
			junction_lane = std::get<1>(t);
		}
	}

	return junction_lane;
}

map_file::PointClass vmap_find_start_point(const VectorMap& vmap, const map_file::Lane& lane)
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

map_file::PointClass vmap_find_end_point(const VectorMap& vmap, const map_file::Lane& lane)
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

map_file::PointClass vmap_find_nearest_point(const VectorMap& vmap, const map_file::PointClass& point)
{
	map_file::PointClass error;
	error.pid = -1;

	if (vmap.points.empty())
		return error;

	map_file::PointClass nearest_point = vmap.points[0];
	double min = hypot(nearest_point.bx - point.bx, nearest_point.ly - point.ly);
	for (const map_file::PointClass& p : vmap.points) {
		double d = hypot(p.bx - point.bx, p.ly - point.ly);
		if (d <= min) {
			min = d;
			nearest_point = p;
		}
	}

	return nearest_point;
}

map_file::PointClass vmap_find_start_nearest_point(const VectorMap& vmap, const map_file::PointClass& p1,
						   const map_file::PointClass& p2, double radius)
{
	map_file::PointClass nearest_point = vmap_find_nearest_point(vmap, p1);
	if (nearest_point.pid < 0)
		return nearest_point;

	std::vector<map_file::PointClass> near_points = vmap_find_near_points(vmap, p1, radius);
	double angle = vmap_compute_direction_angle(p1, p2);
	double min = VMAP_ANGLE_MAX + radius;
	for (const map_file::PointClass& n1 : near_points) {
		map_file::Lane l = vmap_find_lane(vmap, n1);
		if (l.lnid < 0)
			continue;

		map_file::PointClass n2 = vmap_find_end_point(vmap, l);
		if (n2.pid < 0)
			continue;

		double a = vmap_compute_direction_angle(n1, n2);
		double d = hypot(n1.bx - p1.bx, n1.ly - p1.ly);
		double m = fabs(a - angle);
		if (m > 180)
			m = fabs(m - 360);
		m += d;
		if (m < min) {
			min = m;
			nearest_point = n1;
		}
	}

	return nearest_point;
}

map_file::PointClass vmap_find_end_nearest_point(const VectorMap& vmap, const map_file::PointClass& p1,
						 const map_file::PointClass& p2, double radius)
{
	map_file::PointClass nearest_point = vmap_find_nearest_point(vmap, p1);
	if (nearest_point.pid < 0)
		return nearest_point;

	std::vector<map_file::PointClass> near_points = vmap_find_near_points(vmap, p1, radius);
	double angle = vmap_compute_direction_angle(p1, p2);
	double min = VMAP_ANGLE_MAX + radius;
	for (const map_file::PointClass& n1 : near_points) {
		map_file::Lane l = vmap_find_lane(vmap, n1);
		if (l.lnid < 0)
			continue;

		l = vmap_find_prev_lane(vmap, l);
		if (l.lnid < 0)
			continue;

		map_file::PointClass n2 = vmap_find_start_point(vmap, l);
		if (n2.pid < 0)
			continue;

		double a = vmap_compute_direction_angle(n1, n2);
		double d = hypot(n1.bx - p1.bx, n1.ly - p1.ly);
		double m = fabs(a - angle);
		if (m > 180)
			m = fabs(m - 360);
		m += d;
		if (m < min) {
			min = m;
			nearest_point = n1;
		}
	}

	return nearest_point;
}

std::vector<map_file::PointClass> vmap_find_near_points(const VectorMap& vmap, const map_file::PointClass& point,
							double radius)
{
	std::vector<map_file::PointClass> near_points;

	for (const map_file::PointClass& p : vmap.points) {
		double d = hypot(p.bx - point.bx, p.ly - point.ly);
		if (d <= radius)
			near_points.push_back(p);
	}

	return near_points;
}

double vmap_compute_direction_angle(const map_file::PointClass& p1, const map_file::PointClass& p2)
{
	return (atan2(p2.ly - p1.ly, p2.bx - p1.bx) * (180 / M_PI)); // -180 to 180 degrees
}
