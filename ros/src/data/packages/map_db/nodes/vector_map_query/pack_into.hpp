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

#ifndef _PACK_INTO_HPP
#define _PACK_INTO_HPP

#include "vector_map_query.h"

#include <map_file/PointClassArray.h>
#include <map_file/VectorClassArray.h>
#include <map_file/LineClassArray.h>
#include <map_file/AreaClassArray.h>
#include <map_file/PoleClassArray.h>
#include <map_file/BoxClassArray.h>

#include <map_file/DTLaneArray.h>
#include <map_file/NodeArray.h>
#include <map_file/LaneArray.h>

#include <map_file/RoadEdgeArray.h>
#include <map_file/GutterArray.h>
#include <map_file/CurbArray.h>
#include <map_file/WhiteLineArray.h>
#include <map_file/StopLineArray.h>
#include <map_file/ZebraZoneArray.h>
#include <map_file/CrossWalkArray.h>
#include <map_file/RoadMarkArray.h>
#include <map_file/PoleArray.h>
#include <map_file/RoadSignArray.h>
#include <map_file/SignalArray.h>
#include <map_file/StreetLightArray.h>
#include <map_file/UtilityPoleArray.h>
#include <map_file/GuardRailArray.h>
#include <map_file/SideWalkArray.h>
#include <map_file/CrossRoadArray.h>

/*
 * Graphical Primitive Class
 */

map_file::PointClassArray pack_point_class_array(const std::vector<PointClass>&);
map_file::VectorClassArray pack_vector_class_array(const std::vector<VectorClass>&);
map_file::LineClassArray pack_line_class_array(const std::vector<LineClass>&);
map_file::AreaClassArray pack_area_class_array(const std::vector<AreaClass>&);
map_file::PoleClassArray pack_pole_class_array(const std::vector<PoleClass>&);
map_file::BoxClassArray pack_box_class_array(const std::vector<BoxClass>&);

/*
 * Road Data
 */

map_file::DTLaneArray pack_dtlane_array(const std::vector<DTLane>&);
map_file::NodeArray pack_node_array(const std::vector<Node>&);
map_file::LaneArray pack_lane_array(const std::vector<Lane>&);

/*
 * Object Data
 */

map_file::RoadEdgeArray pack_roadedge_array(const std::vector<RoadEdge>&);
map_file::GutterArray pack_gutter_array(const std::vector<Gutter>&);
map_file::CurbArray pack_curb_array(const std::vector<Curb>&);
map_file::WhiteLineArray pack_whiteline_array(const std::vector<WhiteLine>&);
map_file::StopLineArray pack_stopline_array(const std::vector<StopLine>&);
map_file::ZebraZoneArray pack_zebrazone_array(const std::vector<ZebraZone>&);
map_file::CrossWalkArray pack_crosswalk_array(const std::vector<CrossWalk>&);
map_file::RoadMarkArray pack_roadmark_array(const std::vector<RoadMark>&);
map_file::PoleArray pack_pole_array(const std::vector<Pole>&);
map_file::RoadSignArray pack_roadsign_array(const std::vector<RoadSign>&);
map_file::SignalArray pack_signal_array(const std::vector<Signal>&);
map_file::StreetLightArray pack_streetlight_array(const std::vector<StreetLight>&);
map_file::UtilityPoleArray pack_utilitypole_array(const std::vector<UtilityPole>&);
map_file::GuardRailArray pack_guardrail_array(const std::vector<GuardRail>&);
map_file::SideWalkArray pack_sidewalk_array(const std::vector<SideWalk>&);
map_file::CrossRoadArray pack_crossroad_array(const std::vector<CrossRoad>&);

#endif /* !_PACK_INTO_HPP */
