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

#include "pack_into.hpp"

/*
 * Graphical Primitive Class
 */

static map_file::PointClass pack_point_class(const PointClass& src)
{
	map_file::PointClass dst;

	dst.pid = src.pid;
	dst.b = src.b;
	dst.l = src.l;
	dst.h = src.h;
	dst.bx = src.bx;
	dst.ly = src.ly;
	dst.ref = src.ref;
	dst.mcode1 = src.mcode1;
	dst.mcode2 = src.mcode2;
	dst.mcode3 = src.mcode3;

	return dst;
}

static map_file::VectorClass pack_vector_class(const VectorClass& src)
{
	map_file::VectorClass dst;

	dst.vid = src.vid;
	dst.pid = src.pid;
	dst.hang = src.hang;
	dst.vang = src.vang;

	return dst;
}

static map_file::LineClass pack_line_class(const LineClass& src)
{
	map_file::LineClass dst;

	dst.lid = src.lid;
	dst.bpid = src.bpid;
	dst.fpid = src.fpid;
	dst.blid = src.blid;
	dst.flid = src.flid;

	return dst;
}

static map_file::AreaClass pack_area_class(const AreaClass& src)
{
	map_file::AreaClass dst;

	dst.aid = src.aid;
	dst.slid = src.slid;
	dst.elid = src.elid;

	return dst;
}

static map_file::PoleClass pack_pole_class(const PoleClass& src)
{
	map_file::PoleClass dst;

	dst.plid = src.plid;
	dst.vid = src.vid;
	dst.length = src.length;
	dst.dim = src.dim;

	return dst;
}

static map_file::BoxClass pack_box_class(const BoxClass& src)
{
	map_file::BoxClass dst;

	dst.bid = src.bid;
	dst.pid1 = src.pid1;
	dst.pid2 = src.pid2;
	dst.pid3 = src.pid3;
	dst.pid4 = src.pid4;
	dst.height = src.height;

	return dst;
}

map_file::PointClassArray pack_point_class_array(const std::vector<PointClass>& src)
{
	map_file::PointClassArray dst;

	for (size_t i = 0; i < src.size(); ++i) {
		if (src[i].pid <= 0)
			continue;
		dst.point_classes.push_back(pack_point_class(src[i]));
	}

	return dst;
}

map_file::VectorClassArray pack_vector_class_array(const std::vector<VectorClass>& src)
{
	map_file::VectorClassArray dst;

	for (size_t i = 0; i < src.size(); ++i) {
		if (src[i].vid <= 0)
			continue;
		dst.vector_classes.push_back(pack_vector_class(src[i]));
	}

	return dst;
}

map_file::LineClassArray pack_line_class_array(const std::vector<LineClass>& src)
{
	map_file::LineClassArray dst;

	for (size_t i = 0; i < src.size(); ++i) {
		if (src[i].lid <= 0)
			continue;
		dst.line_classes.push_back(pack_line_class(src[i]));
	}

	return dst;
}

map_file::AreaClassArray pack_area_class_array(const std::vector<AreaClass>& src)
{
	map_file::AreaClassArray dst;

	for (size_t i = 0; i < src.size(); ++i) {
		if (src[i].aid <= 0)
			continue;
		dst.area_classes.push_back(pack_area_class(src[i]));
	}

	return dst;
}

map_file::PoleClassArray pack_pole_class_array(const std::vector<PoleClass>& src)
{
	map_file::PoleClassArray dst;

	for (size_t i = 0; i < src.size(); ++i) {
		if (src[i].plid <= 0)
			continue;
		dst.pole_classes.push_back(pack_pole_class(src[i]));
	}

	return dst;
}

map_file::BoxClassArray pack_box_class_array(const std::vector<BoxClass>& src)
{
	map_file::BoxClassArray dst;

	for (size_t i = 0; i < src.size(); ++i) {
		if (src[i].bid <= 0)
			continue;
		dst.box_classes.push_back(pack_box_class(src[i]));
	}

	return dst;
}

/*
 * Road Data
 */

static map_file::DTLane pack_dtlane(const DTLane& src)
{
	map_file::DTLane dst;

	dst.did = src.did;
	dst.dist = src.dist;
	dst.pid = src.pid;
	dst.dir = src.dir;
	dst.apara = src.apara;
	dst.r = src.r;
	dst.slope = src.slope;
	dst.cant = src.cant;
	dst.lw = src.lw;
	dst.rw = src.rw;

	return dst;
}

static map_file::Node pack_node(const Node& src)
{
	map_file::Node dst;

	dst.nid = src.nid;
	dst.pid = src.pid;

	return dst;
}

static map_file::Lane pack_lane(const Lane& src)
{
	map_file::Lane dst;

	dst.lnid = src.lnid;
	dst.did = src.did;
	dst.blid = src.blid;
	dst.flid = src.flid;
	dst.bnid = src.bnid;
	dst.fnid = src.fnid;
	dst.jct = src.jct;
	dst.blid2 = src.blid2;
	dst.blid3 = src.blid3;
	dst.blid4 = src.blid4;
	dst.flid2 = src.flid2;
	dst.flid3 = src.flid3;
	dst.flid4 = src.flid4;
	dst.clossid = src.clossid;
	dst.span = src.span;
	dst.lcnt = src.lcnt;
	dst.lno = src.lno;

	return dst;
}

map_file::DTLaneArray pack_dtlane_array(const std::vector<DTLane>& src)
{
	map_file::DTLaneArray dst;

	for (size_t i = 0; i < src.size(); ++i) {
		if (src[i].did <= 0)
			continue;
		dst.dtlanes.push_back(pack_dtlane(src[i]));
	}

	return dst;
}

map_file::NodeArray pack_node_array(const std::vector<Node>& src)
{
	map_file::NodeArray dst;

	for (size_t i = 0; i < src.size(); ++i) {
		if (src[i].nid <= 0)
			continue;
		dst.nodes.push_back(pack_node(src[i]));
	}

	return dst;
}

map_file::LaneArray pack_lane_array(const std::vector<Lane>& src)
{
	map_file::LaneArray dst;

	for (size_t i = 0; i < src.size(); ++i) {
		if (src[i].lnid <= 0)
			continue;
		dst.lanes.push_back(pack_lane(src[i]));
	}

	return dst;
}

/*
 * Object Data
 */

static map_file::RoadEdge pack_roadedge(const RoadEdge& src)
{
	map_file::RoadEdge dst;

	dst.id = src.id;
	dst.lid = src.lid;
	dst.linkid = src.linkid;

	return dst;
}

static map_file::Gutter pack_gutter(const Gutter& src)
{
	map_file::Gutter dst;

	dst.id = src.id;
	dst.aid = src.aid;
	dst.type = src.type;
	dst.linkid = src.linkid;

	return dst;
}

static map_file::Curb pack_curb(const Curb& src)
{
	map_file::Curb dst;

	dst.id = src.id;
	dst.lid = src.lid;
	dst.height = src.height;
	dst.width = src.width;
	dst.dir = src.dir;
	dst.linkid = src.linkid;

	return dst;
}

static map_file::WhiteLine pack_whiteline(const WhiteLine& src)
{
	map_file::WhiteLine dst;

	dst.id = src.id;
	dst.lid = src.lid;
	dst.width = src.width;
	dst.color = src.color;
	dst.type = src.type;
	dst.linkid = src.linkid;

	return dst;
}

static map_file::StopLine pack_stopline(const StopLine& src)
{
	map_file::StopLine dst;

	dst.id = src.id;
	dst.lid = src.lid;
	dst.tlid = src.tlid;
	dst.signid = src.signid;
	dst.linkid = src.linkid;

	return dst;
}

static map_file::ZebraZone pack_zebrazone(const ZebraZone& src)
{
	map_file::ZebraZone dst;

	dst.id = src.id;
	dst.aid = src.aid;
	dst.linkid = src.linkid;

	return dst;
}

static map_file::CrossWalk pack_crosswalk(const CrossWalk& src)
{
	map_file::CrossWalk dst;

	dst.id = src.id;
	dst.aid = src.aid;
	dst.type = src.type;
	dst.bdid = src.bdid;
	dst.linkid = src.linkid;

	return dst;
}

static map_file::RoadMark pack_roadmark(const RoadMark& src)
{
	map_file::RoadMark dst;

	dst.id = src.id;
	dst.aid = src.aid;
	dst.type = src.type;
	dst.linkid = src.linkid;

	return dst;
}

static map_file::Pole pack_pole(const Pole& src)
{
	map_file::Pole dst;

	dst.id = src.id;
	dst.plid = src.plid;
	dst.linkid = src.linkid;

	return dst;
}

static map_file::RoadSign pack_roadsign(const RoadSign& src)
{
	map_file::RoadSign dst;

	dst.id = src.id;
	dst.vid = src.vid;
	dst.plid = src.plid;
	dst.type = src.type;
	dst.linkid = src.linkid;

	return dst;
}

static map_file::Signal pack_signal(const Signal& src)
{
	map_file::Signal dst;

	dst.id = src.id;
	dst.vid = src.vid;
	dst.plid = src.plid;
	dst.type = src.type;
	dst.linkid = src.linkid;

	return dst;
}

static map_file::StreetLight pack_streetlight(const StreetLight& src)
{
	map_file::StreetLight dst;

	dst.id = src.id;
	dst.lid = src.lid;
	dst.plid = src.plid;
	dst.linkid = src.linkid;

	return dst;
}

static map_file::UtilityPole pack_utilitypole(const UtilityPole& src)
{
	map_file::UtilityPole dst;

	dst.id = src.id;
	dst.plid = src.plid;
	dst.linkid = src.linkid;

	return dst;
}

static map_file::GuardRail pack_guardrail(const GuardRail& src)
{
	map_file::GuardRail dst;

	dst.id = src.id;
	dst.aid = src.aid;
	dst.type = src.type;
	dst.linkid = src.linkid;

	return dst;
}

static map_file::SideWalk pack_sidewalk(const SideWalk& src)
{
	map_file::SideWalk dst;

	dst.id = src.id;
	dst.aid = src.aid;
	dst.linkid = src.linkid;

	return dst;
}

static map_file::CrossRoad pack_crossroad(const CrossRoad& src)
{
	map_file::CrossRoad dst;

	dst.id = src.id;
	dst.aid = src.aid;
	dst.linkid = src.linkid;

	return dst;
}

map_file::RoadEdgeArray pack_roadedge_array(const std::vector<RoadEdge>& src)
{
	map_file::RoadEdgeArray dst;

	for (size_t i = 0; i < src.size(); ++i) {
		if (src[i].id <= 0)
			continue;
		dst.road_edges.push_back(pack_roadedge(src[i]));
	}

	return dst;
}

map_file::GutterArray pack_gutter_array(const std::vector<Gutter>& src)
{
	map_file::GutterArray dst;

	for (size_t i = 0; i < src.size(); ++i) {
		if (src[i].id <= 0)
			continue;
		dst.gutters.push_back(pack_gutter(src[i]));
	}

	return dst;
}

map_file::CurbArray pack_curb_array(const std::vector<Curb>& src)
{
	map_file::CurbArray dst;

	for (size_t i = 0; i < src.size(); ++i) {
		if (src[i].id <= 0)
			continue;
		dst.curbs.push_back(pack_curb(src[i]));
	}

	return dst;
}

map_file::WhiteLineArray pack_whiteline_array(const std::vector<WhiteLine>& src)
{
	map_file::WhiteLineArray dst;

	for (size_t i = 0; i < src.size(); ++i) {
		if (src[i].id <= 0)
			continue;
		dst.white_lines.push_back(pack_whiteline(src[i]));
	}

	return dst;
}

map_file::StopLineArray pack_stopline_array(const std::vector<StopLine>& src)
{
	map_file::StopLineArray dst;

	for (size_t i = 0; i < src.size(); ++i) {
		if (src[i].id <= 0)
			continue;
		dst.stop_lines.push_back(pack_stopline(src[i]));
	}

	return dst;
}

map_file::ZebraZoneArray pack_zebrazone_array(const std::vector<ZebraZone>& src)
{
	map_file::ZebraZoneArray dst;

	for (size_t i = 0; i < src.size(); ++i) {
		if (src[i].id <= 0)
			continue;
		dst.zebra_zones.push_back(pack_zebrazone(src[i]));
	}

	return dst;
}

map_file::CrossWalkArray pack_crosswalk_array(const std::vector<CrossWalk>& src)
{
	map_file::CrossWalkArray dst;

	for (size_t i = 0; i < src.size(); ++i) {
		if (src[i].id <= 0)
			continue;
		dst.cross_walks.push_back(pack_crosswalk(src[i]));
	}

	return dst;
}

map_file::RoadMarkArray pack_roadmark_array(const std::vector<RoadMark>& src)
{
	map_file::RoadMarkArray dst;

	for (size_t i = 0; i < src.size(); ++i) {
		if (src[i].id <= 0)
			continue;
		dst.road_marks.push_back(pack_roadmark(src[i]));
	}

	return dst;
}

map_file::PoleArray pack_pole_array(const std::vector<Pole>& src)
{
	map_file::PoleArray dst;

	for (size_t i = 0; i < src.size(); ++i) {
		if (src[i].id <= 0)
			continue;
		dst.poles.push_back(pack_pole(src[i]));
	}

	return dst;
}

map_file::RoadSignArray pack_roadsign_array(const std::vector<RoadSign>& src)
{
	map_file::RoadSignArray dst;

	for (size_t i = 0; i < src.size(); ++i) {
		if (src[i].id <= 0)
			continue;
		dst.road_signs.push_back(pack_roadsign(src[i]));
	}

	return dst;
}

map_file::SignalArray pack_signal_array(const std::vector<Signal>& src)
{
	map_file::SignalArray dst;

	for (size_t i = 0; i < src.size(); ++i) {
		if (src[i].id <= 0)
			continue;
		dst.signals.push_back(pack_signal(src[i]));
	}

	return dst;
}

map_file::StreetLightArray pack_streetlight_array(const std::vector<StreetLight>& src)
{
	map_file::StreetLightArray dst;

	for (size_t i = 0; i < src.size(); ++i) {
		if (src[i].id <= 0)
			continue;
		dst.street_lights.push_back(pack_streetlight(src[i]));
	}

	return dst;
}

map_file::UtilityPoleArray pack_utilitypole_array(const std::vector<UtilityPole>& src)
{
	map_file::UtilityPoleArray dst;

	for (size_t i = 0; i < src.size(); ++i) {
		if (src[i].id <= 0)
			continue;
		dst.utility_poles.push_back(pack_utilitypole(src[i]));
	}

	return dst;
}

map_file::GuardRailArray pack_guardrail_array(const std::vector<GuardRail>& src)
{
	map_file::GuardRailArray dst;

	for (size_t i = 0; i < src.size(); ++i) {
		if (src[i].id <= 0)
			continue;
		dst.guard_rails.push_back(pack_guardrail(src[i]));
	}

	return dst;
}

map_file::SideWalkArray pack_sidewalk_array(const std::vector<SideWalk>& src)
{
	map_file::SideWalkArray dst;

	for (size_t i = 0; i < src.size(); ++i) {
		if (src[i].id <= 0)
			continue;
		dst.side_walks.push_back(pack_sidewalk(src[i]));
	}

	return dst;
}

map_file::CrossRoadArray pack_crossroad_array(const std::vector<CrossRoad>& src)
{
	map_file::CrossRoadArray dst;

	for (size_t i = 0; i < src.size(); ++i) {
		if (src[i].id <= 0)
			continue;
		dst.cross_roads.push_back(pack_crossroad(src[i]));
	}

	return dst;
}
