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

#ifndef _VMAP_UTILITY_HPP
#define _VMAP_UTILITY_HPP

#include <vector>

#include <map_file/PointClassArray.h>
#include <map_file/LaneArray.h>
#include <map_file/NodeArray.h>
#include <map_file/StopLineArray.h>
#include <map_file/DTLaneArray.h>

struct VectorMap {
	std::vector<map_file::PointClass> points;
	std::vector<map_file::Lane> lanes;
	std::vector<map_file::Node> nodes;
	std::vector<map_file::StopLine> stoplines;
	std::vector<map_file::DTLane> dtlanes;
};

map_file::Lane vmap_find_lane(const VectorMap& vmap, const map_file::PointClass& point);
map_file::Lane vmap_find_prev_lane(const VectorMap& vmap, const map_file::Lane& lane);
map_file::Lane vmap_find_next_lane(const VectorMap& vmap, const map_file::Lane& lane);
map_file::PointClass vmap_find_start_point(const VectorMap& vmap, const map_file::Lane& lane);
map_file::PointClass vmap_find_end_point(const VectorMap& vmap, const map_file::Lane& lane);
map_file::PointClass vmap_find_nearest_point(const VectorMap& vmap, double x, double y);

#endif /* !_VMAP_UTILITY_HPP */
