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

map_file::PointClass vmap_find_nearest_point(const VectorMap& vmap, double x, double y)
{
	map_file::PointClass nearest_point = vmap.points[0];

	double min = hypot(x - nearest_point.bx, y - nearest_point.ly);
	for (const map_file::PointClass& p : vmap.points) {
		double distance = hypot(x - p.bx, y - p.ly);
		if (distance < min) {
			min = distance;
			nearest_point = p;
		}
	}

	return nearest_point;
}
