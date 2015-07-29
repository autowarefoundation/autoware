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

int to_lane_index(const map_file::PointClass& point,
		  const std::vector<map_file::Node>& nodes,
		  const std::vector<map_file::Lane>& lanes)
{
	for (const map_file::Node& node : nodes) {
		if (node.pid != point.pid)
			continue;
		for (int i = 0; i < static_cast<int>(lanes.size()); i++) {
			if (lanes[i].bnid != node.nid)
				continue;
			return i;
		}
	}

	return -1;
}

int to_next_lane_index(const map_file::Lane& lane,
		       const std::vector<map_file::Lane>& lanes)
{
	for (int i = 0; i < static_cast<int>(lanes.size()); i++) {
		if (lanes[i].lnid != lane.flid)
			continue;
		return i;
	}

	return -1;
}

int to_beginning_point_index(const map_file::Lane& lane,
			     const std::vector<map_file::Node>& nodes,
			     const std::vector<map_file::PointClass>& points)
{
	for (const map_file::Node& node : nodes) {
		if (node.nid != lane.bnid)
			continue;
		for (int i = 0; i < static_cast<int>(points.size()); i++) {
			if (points[i].pid != node.pid)
				continue;
			return i;
		}
	}

	return -1;
}

int to_finishing_point_index(const map_file::Lane& lane,
			     const std::vector<map_file::Node>& nodes,
			     const std::vector<map_file::PointClass>& points)
{
	for (const map_file::Node& node : nodes) {
		if (node.nid != lane.fnid)
			continue;
		for (int i = 0; i < static_cast<int>(points.size()); i++) {
			if (points[i].pid != node.pid)
				continue;
			return i;
		}
	}

	return -1;
}

map_file::PointClass
search_nearest(const std::vector<map_file::PointClass>& points,
	       double x, double y)
{
	map_file::PointClass nearest_point = points[0];
	double min = hypot(x - points[0].bx, y - points[0].ly);
	for (const map_file::PointClass& point : points) {
		double distance = hypot(x - point.bx, y - point.ly);
		if (distance < min) {
			min = distance;
			nearest_point = point;
		}
	}

	return nearest_point;
}
