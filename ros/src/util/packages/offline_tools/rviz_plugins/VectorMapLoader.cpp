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


/*
 * VectorMapLoader.cpp
 *
 *  Created on: Nov 14, 2018
 *      Author: sujiwo
 */


#include <vector>
#include <string>
#include "VectorMapLoader.h"
#include "access_private.hpp"


using namespace std;
using namespace boost::filesystem;


using vector_map::VectorMap;
using vector_map::Category;
using vector_map::Color;
using vector_map::Key;

using vector_map::Point;
using vector_map::Vector;
using vector_map::Line;
using vector_map::Area;
using vector_map::Pole;
using vector_map::Box;
using vector_map::DTLane;
using vector_map::Node;
using vector_map::Lane;
using vector_map::WayArea;
using vector_map::RoadEdge;
using vector_map::Gutter;
using vector_map::Curb;
using vector_map::WhiteLine;
using vector_map::StopLine;
using vector_map::ZebraZone;
using vector_map::CrossWalk;
using vector_map::RoadMark;
using vector_map::RoadPole;
using vector_map::RoadSign;
using vector_map::Signal;
using vector_map::StreetLight;
using vector_map::UtilityPole;
using vector_map::GuardRail;
using vector_map::SideWalk;
using vector_map::DriveOnPortion;
using vector_map::CrossRoad;
using vector_map::SideStrip;
using vector_map::CurveMirror;
using vector_map::Wall;
using vector_map::Fence;
using vector_map::RailCrossing;

using vector_map::PointArray;
using vector_map::VectorArray;
using vector_map::LineArray;
using vector_map::AreaArray;
using vector_map::PoleArray;
using vector_map::BoxArray;
using vector_map::DTLaneArray;
using vector_map::NodeArray;
using vector_map::LaneArray;
using vector_map::WayAreaArray;
using vector_map::RoadEdgeArray;
using vector_map::GutterArray;
using vector_map::CurbArray;
using vector_map::WhiteLineArray;
using vector_map::StopLineArray;
using vector_map::ZebraZoneArray;
using vector_map::CrossWalkArray;
using vector_map::RoadMarkArray;
using vector_map::RoadPoleArray;
using vector_map::RoadSignArray;
using vector_map::SignalArray;
using vector_map::StreetLightArray;
using vector_map::UtilityPoleArray;
using vector_map::GuardRailArray;
using vector_map::SideWalkArray;
using vector_map::DriveOnPortionArray;
using vector_map::CrossRoadArray;
using vector_map::SideStripArray;
using vector_map::CurveMirrorArray;
using vector_map::WallArray;
using vector_map::FenceArray;
using vector_map::RailCrossingArray;

using vector_map::Handle;


/*
 * Need to copy these functions from inside vector_map.cpp,
 * because because protected inside anonymous namespace
 */
void updatePoint(std::map<Key<Point>, Point>& map, const PointArray& msg)
{
  map = std::map<Key<Point>, Point>();
  for (const auto& item : msg.data)
  {
    if (item.pid == 0)
      continue;
    map.insert(std::make_pair(Key<Point>(item.pid), item));
  }
}

void updateVector(std::map<Key<Vector>, Vector>& map, const VectorArray& msg)
{
  map = std::map<Key<Vector>, Vector>();
  for (const auto& item : msg.data)
  {
    if (item.vid == 0)
      continue;
    map.insert(std::make_pair(Key<Vector>(item.vid), item));
  }
}

void updateLine(std::map<Key<Line>, Line>& map, const LineArray& msg)
{
  map = std::map<Key<Line>, Line>();
  for (const auto& item : msg.data)
  {
    if (item.lid == 0)
      continue;
    map.insert(std::make_pair(Key<Line>(item.lid), item));
  }
}

void updateArea(std::map<Key<Area>, Area>& map, const AreaArray& msg)
{
  map = std::map<Key<Area>, Area>();
  for (const auto& item : msg.data)
  {
    if (item.aid == 0)
      continue;
    map.insert(std::make_pair(Key<Area>(item.aid), item));
  }
}

void updatePole(std::map<Key<Pole>, Pole>& map, const PoleArray& msg)
{
  map = std::map<Key<Pole>, Pole>();
  for (const auto& item : msg.data)
  {
    if (item.plid == 0)
      continue;
    map.insert(std::make_pair(Key<Pole>(item.plid), item));
  }
}

void updateBox(std::map<Key<Box>, Box>& map, const BoxArray& msg)
{
  map = std::map<Key<Box>, Box>();
  for (const auto& item : msg.data)
  {
    if (item.bid == 0)
      continue;
    map.insert(std::make_pair(Key<Box>(item.bid), item));
  }
}

void updateDTLane(std::map<Key<DTLane>, DTLane>& map, const DTLaneArray& msg)
{
  map = std::map<Key<DTLane>, DTLane>();
  for (const auto& item : msg.data)
  {
    if (item.did == 0)
      continue;
    map.insert(std::make_pair(Key<DTLane>(item.did), item));
  }
}

void updateNode(std::map<Key<Node>, Node>& map, const NodeArray& msg)
{
  map = std::map<Key<Node>, Node>();
  for (const auto& item : msg.data)
  {
    if (item.nid == 0)
      continue;
    map.insert(std::make_pair(Key<Node>(item.nid), item));
  }
}

void updateLane(std::map<Key<Lane>, Lane>& map, const LaneArray& msg)
{
  map = std::map<Key<Lane>, Lane>();
  for (const auto& item : msg.data)
  {
    if (item.lnid == 0)
      continue;
    map.insert(std::make_pair(Key<Lane>(item.lnid), item));
  }
}

void updateWayArea(std::map<Key<WayArea>, WayArea>& map, const WayAreaArray& msg)
{
  map = std::map<Key<WayArea>, WayArea>();
  for (const auto& item : msg.data)
  {
    if (item.waid == 0)
      continue;
    map.insert(std::make_pair(Key<WayArea>(item.waid), item));
  }
}

void updateRoadEdge(std::map<Key<RoadEdge>, RoadEdge>& map, const RoadEdgeArray& msg)
{
  map = std::map<Key<RoadEdge>, RoadEdge>();
  for (const auto& item : msg.data)
  {
    if (item.id == 0)
      continue;
    map.insert(std::make_pair(Key<RoadEdge>(item.id), item));
  }
}

void updateGutter(std::map<Key<Gutter>, Gutter>& map, const GutterArray& msg)
{
  map = std::map<Key<Gutter>, Gutter>();
  for (const auto& item : msg.data)
  {
    if (item.id == 0)
      continue;
    map.insert(std::make_pair(Key<Gutter>(item.id), item));
  }
}

void updateCurb(std::map<Key<Curb>, Curb>& map, const CurbArray& msg)
{
  map = std::map<Key<Curb>, Curb>();
  for (const auto& item : msg.data)
  {
    if (item.id == 0)
      continue;
    map.insert(std::make_pair(Key<Curb>(item.id), item));
  }
}

void updateWhiteLine(std::map<Key<WhiteLine>, WhiteLine>& map, const WhiteLineArray& msg)
{
  map = std::map<Key<WhiteLine>, WhiteLine>();
  for (const auto& item : msg.data)
  {
    if (item.id == 0)
      continue;
    map.insert(std::make_pair(Key<WhiteLine>(item.id), item));
  }
}

void updateStopLine(std::map<Key<StopLine>, StopLine>& map, const StopLineArray& msg)
{
  map = std::map<Key<StopLine>, StopLine>();
  for (const auto& item : msg.data)
  {
    if (item.id == 0)
      continue;
    map.insert(std::make_pair(Key<StopLine>(item.id), item));
  }
}

void updateZebraZone(std::map<Key<ZebraZone>, ZebraZone>& map, const ZebraZoneArray& msg)
{
  map = std::map<Key<ZebraZone>, ZebraZone>();
  for (const auto& item : msg.data)
  {
    if (item.id == 0)
      continue;
    map.insert(std::make_pair(Key<ZebraZone>(item.id), item));
  }
}

void updateCrossWalk(std::map<Key<CrossWalk>, CrossWalk>& map, const CrossWalkArray& msg)
{
  map = std::map<Key<CrossWalk>, CrossWalk>();
  for (const auto& item : msg.data)
  {
    if (item.id == 0)
      continue;
    map.insert(std::make_pair(Key<CrossWalk>(item.id), item));
  }
}

void updateRoadMark(std::map<Key<RoadMark>, RoadMark>& map, const RoadMarkArray& msg)
{
  map = std::map<Key<RoadMark>, RoadMark>();
  for (const auto& item : msg.data)
  {
    if (item.id == 0)
      continue;
    map.insert(std::make_pair(Key<RoadMark>(item.id), item));
  }
}

void updateRoadPole(std::map<Key<RoadPole>, RoadPole>& map, const RoadPoleArray& msg)
{
  map = std::map<Key<RoadPole>, RoadPole>();
  for (const auto& item : msg.data)
  {
    if (item.id == 0)
      continue;
    map.insert(std::make_pair(Key<RoadPole>(item.id), item));
  }
}

void updateRoadSign(std::map<Key<RoadSign>, RoadSign>& map, const RoadSignArray& msg)
{
  map = std::map<Key<RoadSign>, RoadSign>();
  for (const auto& item : msg.data)
  {
    if (item.id == 0)
      continue;
    map.insert(std::make_pair(Key<RoadSign>(item.id), item));
  }
}

void updateSignal(std::map<Key<Signal>, Signal>& map, const SignalArray& msg)
{
  map = std::map<Key<Signal>, Signal>();
  for (const auto& item : msg.data)
  {
    if (item.id == 0)
      continue;
    map.insert(std::make_pair(Key<Signal>(item.id), item));
  }
}

void updateStreetLight(std::map<Key<StreetLight>, StreetLight>& map, const StreetLightArray& msg)
{
  map = std::map<Key<StreetLight>, StreetLight>();
  for (const auto& item : msg.data)
  {
    if (item.id == 0)
      continue;
    map.insert(std::make_pair(Key<StreetLight>(item.id), item));
  }
}

void updateUtilityPole(std::map<Key<UtilityPole>, UtilityPole>& map, const UtilityPoleArray& msg)
{
  map = std::map<Key<UtilityPole>, UtilityPole>();
  for (const auto& item : msg.data)
  {
    if (item.id == 0)
      continue;
    map.insert(std::make_pair(Key<UtilityPole>(item.id), item));
  }
}

void updateGuardRail(std::map<Key<GuardRail>, GuardRail>& map, const GuardRailArray& msg)
{
  map = std::map<Key<GuardRail>, GuardRail>();
  for (const auto& item : msg.data)
  {
    if (item.id == 0)
      continue;
    map.insert(std::make_pair(Key<GuardRail>(item.id), item));
  }
}

void updateSideWalk(std::map<Key<SideWalk>, SideWalk>& map, const SideWalkArray& msg)
{
  map = std::map<Key<SideWalk>, SideWalk>();
  for (const auto& item : msg.data)
  {
    if (item.id == 0)
      continue;
    map.insert(std::make_pair(Key<SideWalk>(item.id), item));
  }
}

void updateDriveOnPortion(std::map<Key<DriveOnPortion>, DriveOnPortion>& map, const DriveOnPortionArray& msg)
{
  map = std::map<Key<DriveOnPortion>, DriveOnPortion>();
  for (const auto& item : msg.data)
  {
    if (item.id == 0)
      continue;
    map.insert(std::make_pair(Key<DriveOnPortion>(item.id), item));
  }
}

void updateCrossRoad(std::map<Key<CrossRoad>, CrossRoad>& map, const CrossRoadArray& msg)
{
  map = std::map<Key<CrossRoad>, CrossRoad>();
  for (const auto& item : msg.data)
  {
    if (item.id == 0)
      continue;
    map.insert(std::make_pair(Key<CrossRoad>(item.id), item));
  }
}

void updateSideStrip(std::map<Key<SideStrip>, SideStrip>& map, const SideStripArray& msg)
{
  map = std::map<Key<SideStrip>, SideStrip>();
  for (const auto& item : msg.data)
  {
    if (item.id == 0)
      continue;
    map.insert(std::make_pair(Key<SideStrip>(item.id), item));
  }
}

void updateCurveMirror(std::map<Key<CurveMirror>, CurveMirror>& map, const CurveMirrorArray& msg)
{
  map = std::map<Key<CurveMirror>, CurveMirror>();
  for (const auto& item : msg.data)
  {
    if (item.id == 0)
      continue;
    map.insert(std::make_pair(Key<CurveMirror>(item.id), item));
  }
}

void updateWall(std::map<Key<Wall>, Wall>& map, const WallArray& msg)
{
  map = std::map<Key<Wall>, Wall>();
  for (const auto& item : msg.data)
  {
    if (item.id == 0)
      continue;
    map.insert(std::make_pair(Key<Wall>(item.id), item));
  }
}

void updateFence(std::map<Key<Fence>, Fence>& map, const FenceArray& msg)
{
  map = std::map<Key<Fence>, Fence>();
  for (const auto& item : msg.data)
  {
    if (item.id == 0)
      continue;
    map.insert(std::make_pair(Key<Fence>(item.id), item));
  }
}

void updateRailCrossing(std::map<Key<RailCrossing>, RailCrossing>& map, const RailCrossingArray& msg)
{
  map = std::map<Key<RailCrossing>, RailCrossing>();
  for (const auto& item : msg.data)
  {
    if (item.id == 0)
      continue;
    map.insert(std::make_pair(Key<RailCrossing>(item.id), item));
  }
}

namespace {
bool isValidMarker(const visualization_msgs::Marker& marker)
{
	return marker.action == visualization_msgs::Marker::ADD;
}
}


const vector<string> VectorMapFileNames
    {
      "idx.csv",
      "point.csv",
      "vector.csv",
      "line.csv",
      "area.csv",
      "pole.csv",
      "box.csv",
      "dtlane.csv",
      "node.csv",
      "lane.csv",
      "wayarea.csv",
      "roadedge.csv",
      "gutter.csv",
      "curb.csv",
      "whiteline.csv",
      "stopline.csv",
      "zebrazone.csv",
      "crosswalk.csv",
      "road_surface_mark.csv",
      "poledata.csv",
      "roadsign.csv",
      "signaldata.csv",
      "streetlight.csv",
      "utilitypole.csv",
      "guardrail.csv",
      "sidewalk.csv",
      "driveon_portion.csv",
      "intersection.csv",
      "sidestrip.csv",
      "curvemirror.csv",
      "wall.csv",
      "fence.csv",
      "railroad_crossing.csv"
    };



void insertMarkerArray(visualization_msgs::MarkerArray& a1, const visualization_msgs::MarkerArray& a2)
{
	a1.markers.insert(a1.markers.end(), a2.markers.begin(), a2.markers.end());
}


visualization_msgs::Marker createLinkedLineMarker(const std::string& ns, int id, Color color, const VectorMap& vmap,
                                                  const Line& line)
{
  Area area;
  area.aid = 1; // must set valid aid
  area.slid = line.lid;
  return createAreaMarker(ns, id, color, vmap, area);
}

visualization_msgs::MarkerArray createRoadEdgeMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& road_edge : vmap.findByFilter([](const RoadEdge& road_edge){return true;}))
  {
    if (road_edge.lid == 0)
    {
      ROS_ERROR_STREAM("[createRoadEdgeMarkerArray] invalid road_edge: " << road_edge);
      continue;
    }

    Line line = vmap.findByKey(Key<Line>(road_edge.lid));
    if (line.lid == 0)
    {
      ROS_ERROR_STREAM("[createRoadEdgeMarkerArray] invalid line: " << line);
      continue;
    }

    if (line.blid == 0) // if beginning line
    {
      visualization_msgs::Marker marker = createLinkedLineMarker("road_edge", id++, color, vmap, line);
      if (isValidMarker(marker))
        marker_array.markers.push_back(marker);
      else
        ROS_ERROR_STREAM("[createRoadEdgeMarkerArray] failed createLinkedLineMarker: " << line);
    }
  }
  return marker_array;
}

visualization_msgs::MarkerArray createGutterMarkerArray(const VectorMap& vmap, Color no_cover_color,
                                                        Color cover_color, Color grating_color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& gutter : vmap.findByFilter([](const Gutter& gutter){return true;}))
  {
    if (gutter.aid == 0)
    {
      ROS_ERROR_STREAM("[createGutterMarkerArray] invalid gutter: " << gutter);
      continue;
    }

    Area area = vmap.findByKey(Key<Area>(gutter.aid));
    if (area.aid == 0)
    {
      ROS_ERROR_STREAM("[createGutterMarkerArray] invalid area: " << area);
      continue;
    }

    visualization_msgs::Marker marker;
    switch (gutter.type)
    {
    case Gutter::NO_COVER:
      marker = createAreaMarker("gutter", id++, no_cover_color, vmap, area);
      break;
    case Gutter::COVER:
      marker = createAreaMarker("gutter", id++, cover_color, vmap, area);
      break;
    case Gutter::GRATING:
      marker = createAreaMarker("gutter", id++, grating_color, vmap, area);
      break;
    default:
      ROS_ERROR_STREAM("[createGutterMarkerArray] unknown gutter.type: " << gutter);
      continue;
    }
    if (isValidMarker(marker))
      marker_array.markers.push_back(marker);
    else
      ROS_ERROR_STREAM("[createGutterMarkerArray] failed createAreaMarker: " << area);
  }
  return marker_array;
}

visualization_msgs::MarkerArray createCurbMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& curb : vmap.findByFilter([](const Curb& curb){return true;}))
  {
    if (curb.lid == 0)
    {
      ROS_ERROR_STREAM("[createCurbMarkerArray] invalid curb: " << curb);
      continue;
    }

    Line line = vmap.findByKey(Key<Line>(curb.lid));
    if (line.lid == 0)
    {
      ROS_ERROR_STREAM("[createCurbMarkerArray] invalid line: " << line);
      continue;
    }

    if (line.blid == 0) // if beginning line
    {
      visualization_msgs::Marker marker = createLinkedLineMarker("curb", id++, color, vmap, line);
      // XXX: The visualization_msgs::Marker::LINE_STRIP is difficult to deal with curb.width and curb.height.
      if (isValidMarker(marker))
        marker_array.markers.push_back(marker);
      else
        ROS_ERROR_STREAM("[createCurbMarkerArray] failed createLinkedLineMarker: " << line);
    }
  }
  return marker_array;
}

visualization_msgs::MarkerArray createWhiteLineMarkerArray(const VectorMap& vmap, Color white_color,
                                                           Color yellow_color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& white_line : vmap.findByFilter([](const WhiteLine& white_line){return true;}))
  {
    if (white_line.lid == 0)
    {
      ROS_ERROR_STREAM("[createWhiteLineMarkerArray] invalid white_line: " << white_line);
      continue;
    }
    if (white_line.type == WhiteLine::DASHED_LINE_BLANK) // if invisible line
      continue;

    Line line = vmap.findByKey(Key<Line>(white_line.lid));
    if (line.lid == 0)
    {
      ROS_ERROR_STREAM("[createWhiteLineMarkerArray] invalid line: " << line);
      continue;
    }

    if (line.blid == 0) // if beginning line
    {
      visualization_msgs::Marker marker;
      switch (white_line.color)
      {
      case 'W':
        marker = createLinkedLineMarker("white_line", id++, white_color, vmap, line);
        break;
      case 'Y':
        marker = createLinkedLineMarker("white_line", id++, yellow_color, vmap, line);
        break;
      default:
        ROS_ERROR_STREAM("[createWhiteLineMarkerArray] unknown white_line.color: " << white_line);
        continue;
      }
      // XXX: The visualization_msgs::Marker::LINE_STRIP is difficult to deal with white_line.width.
      if (isValidMarker(marker))
        marker_array.markers.push_back(marker);
      else
        ROS_ERROR_STREAM("[createWhiteLineMarkerArray] failed createLinkedLineMarker: " << line);
    }
  }
  return marker_array;
}

visualization_msgs::MarkerArray createStopLineMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& stop_line : vmap.findByFilter([](const StopLine& stop_line){return true;}))
  {
    if (stop_line.lid == 0)
    {
      ROS_ERROR_STREAM("[createStopLineMarkerArray] invalid stop_line: " << stop_line);
      continue;
    }

    Line line = vmap.findByKey(Key<Line>(stop_line.lid));
    if (line.lid == 0)
    {
      ROS_ERROR_STREAM("[createStopLineMarkerArray] invalid line: " << line);
      continue;
    }

    if (line.blid == 0) // if beginning line
    {
      visualization_msgs::Marker marker = createLinkedLineMarker("stop_line", id++, color, vmap, line);
      if (isValidMarker(marker))
        marker_array.markers.push_back(marker);
      else
        ROS_ERROR_STREAM("[createStopLineMarkerArray] failed createLinkedLineMarker: " << line);
    }
  }
  return marker_array;
}

visualization_msgs::MarkerArray createZebraZoneMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& zebra_zone : vmap.findByFilter([](const ZebraZone& zebra_zone){return true;}))
  {
    if (zebra_zone.aid == 0)
    {
      ROS_ERROR_STREAM("[createZebraZoneMarkerArray] invalid zebra_zone: " << zebra_zone);
      continue;
    }

    Area area = vmap.findByKey(Key<Area>(zebra_zone.aid));
    if (area.aid == 0)
    {
      ROS_ERROR_STREAM("[createZebraZoneMarkerArray] invalid area: " << area);
      continue;
    }

    visualization_msgs::Marker marker = createAreaMarker("zebra_zone", id++, color, vmap, area);
    if (isValidMarker(marker))
      marker_array.markers.push_back(marker);
    else
      ROS_ERROR_STREAM("[createZebraZoneMarkerArray] failed createAreaMarker: " << area);
  }
  return marker_array;
}

visualization_msgs::MarkerArray createCrossWalkMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& cross_walk : vmap.findByFilter([](const CrossWalk& cross_walk){return true;}))
  {
    if (cross_walk.aid == 0)
    {
      ROS_ERROR_STREAM("[createCrossWalkMarkerArray] invalid cross_walk: " << cross_walk);
      continue;
    }

    Area area = vmap.findByKey(Key<Area>(cross_walk.aid));
    if (area.aid == 0)
    {
      ROS_ERROR_STREAM("[createCrossWalkMarkerArray] invalid area: " << area);
      continue;
    }

    visualization_msgs::Marker marker = createAreaMarker("cross_walk", id++, color, vmap, area);
    if (isValidMarker(marker))
      marker_array.markers.push_back(marker);
    else
      ROS_ERROR_STREAM("[createCrossWalkMarkerArray] failed createAreaMarker: " << area);
  }
  return marker_array;
}

visualization_msgs::MarkerArray createRoadMarkMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& road_mark : vmap.findByFilter([](const RoadMark& road_mark){return true;}))
  {
    if (road_mark.aid == 0)
    {
      ROS_ERROR_STREAM("[createRoadMarkMarkerArray] invalid road_mark: " << road_mark);
      continue;
    }

    Area area = vmap.findByKey(Key<Area>(road_mark.aid));
    if (area.aid == 0)
    {
      ROS_ERROR_STREAM("[createRoadMarkMarkerArray] invalid area: " << area);
      continue;
    }

    visualization_msgs::Marker marker = createAreaMarker("road_mark", id++, color, vmap, area);
    if (isValidMarker(marker))
      marker_array.markers.push_back(marker);
    else
      ROS_ERROR_STREAM("[createRoadMarkMarkerArray] failed createAreaMarker: " << area);
  }
  return marker_array;
}

visualization_msgs::MarkerArray createRoadPoleMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& road_pole : vmap.findByFilter([](const RoadPole& road_pole){return true;}))
  {
    if (road_pole.plid == 0)
    {
      ROS_ERROR_STREAM("[createRoadPoleMarkerArray] invalid road_pole: " << road_pole);
      continue;
    }

    Pole pole = vmap.findByKey(Key<Pole>(road_pole.plid));
    if (pole.plid == 0)
    {
      ROS_ERROR_STREAM("[createRoadPoleMarkerArray] invalid pole: " << pole);
      continue;
    }

    visualization_msgs::Marker marker = createPoleMarker("road_pole", id++, color, vmap, pole);
    if (isValidMarker(marker))
      marker_array.markers.push_back(marker);
    else
      ROS_ERROR_STREAM("[createRoadPoleMarkerArray] failed createPoleMarker: " << pole);
  }
  return marker_array;
}

visualization_msgs::MarkerArray createRoadSignMarkerArray(const VectorMap& vmap, Color sign_color, Color pole_color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& road_sign : vmap.findByFilter([](const RoadSign& road_sign){return true;}))
  {
    if (road_sign.vid == 0)
    {
      ROS_ERROR_STREAM("[createRoadSignMarkerArray] invalid road_sign: " << road_sign);
      continue;
    }

    Vector vector = vmap.findByKey(Key<Vector>(road_sign.vid));
    if (vector.vid == 0)
    {
      ROS_ERROR_STREAM("[createRoadSignMarkerArray] invalid vector: " << vector);
      continue;
    }

    Pole pole;
    if (road_sign.plid != 0)
    {
      pole = vmap.findByKey(Key<Pole>(road_sign.plid));
      if (pole.plid == 0)
      {
        ROS_ERROR_STREAM("[createRoadSignMarkerArray] invalid pole: " << pole);
        continue;
      }
    }

    visualization_msgs::Marker vector_marker = createVectorMarker("road_sign", id++, sign_color, vmap, vector);
    if (isValidMarker(vector_marker))
      marker_array.markers.push_back(vector_marker);
    else
      ROS_ERROR_STREAM("[createRoadSignMarkerArray] failed createVectorMarker: " << vector);

    if (road_sign.plid != 0)
    {
      visualization_msgs::Marker pole_marker = createPoleMarker("road_sign", id++, pole_color, vmap, pole);
      if (isValidMarker(pole_marker))
        marker_array.markers.push_back(pole_marker);
      else
        ROS_ERROR_STREAM("[createRoadSignMarkerArray] failed createPoleMarker: " << pole);
    }
  }
  return marker_array;
}

visualization_msgs::MarkerArray createSignalMarkerArray(const VectorMap& vmap, Color red_color, Color blue_color,
                                                        Color yellow_color, Color other_color, Color pole_color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& signal : vmap.findByFilter([](const Signal& signal){return true;}))
  {
    if (signal.vid == 0)
    {
      ROS_ERROR_STREAM("[createSignalMarkerArray] invalid signal: " << signal);
      continue;
    }

    Vector vector = vmap.findByKey(Key<Vector>(signal.vid));
    if (vector.vid == 0)
    {
      ROS_ERROR_STREAM("[createSignalMarkerArray] invalid vector: " << vector);
      continue;
    }

    Pole pole;
    if (signal.plid != 0)
    {
      pole = vmap.findByKey(Key<Pole>(signal.plid));
      if (pole.plid == 0)
      {
        ROS_ERROR_STREAM("[createSignalMarkerArray] invalid pole: " << pole);
        continue;
      }
    }

    visualization_msgs::Marker vector_marker;
    switch (signal.type)
    {
    case Signal::RED:
    case Signal::PEDESTRIAN_RED:
      vector_marker = createVectorMarker("signal", id++, red_color, vmap, vector);
      break;
    case Signal::BLUE:
    case Signal::PEDESTRIAN_BLUE:
      vector_marker = createVectorMarker("signal", id++, blue_color, vmap, vector);
      break;
    case Signal::YELLOW:
      vector_marker = createVectorMarker("signal", id++, yellow_color, vmap, vector);
      break;
    case Signal::RED_LEFT:
      vector_marker = createVectorMarker("signal", id++, Color::LIGHT_RED, vmap, vector);
          break;
    case Signal::BLUE_LEFT:
      vector_marker = createVectorMarker("signal", id++, Color::LIGHT_GREEN, vmap, vector);
          break;
    case Signal::YELLOW_LEFT:
      vector_marker = createVectorMarker("signal", id++, Color::LIGHT_YELLOW, vmap, vector);
          break;
    case Signal::OTHER:
      vector_marker = createVectorMarker("signal", id++, other_color, vmap, vector);
      break;
    default:
      ROS_WARN_STREAM("[createSignalMarkerArray] unknown signal.type: " << signal.type << " Creating Marker as OTHER.");
      vector_marker = createVectorMarker("signal", id++, Color::GRAY, vmap, vector);
      break;
    }
    if (isValidMarker(vector_marker))
      marker_array.markers.push_back(vector_marker);
    else
      ROS_ERROR_STREAM("[createSignalMarkerArray] failed createVectorMarker: " << vector);

    if (signal.plid != 0)
    {
      visualization_msgs::Marker pole_marker = createPoleMarker("signal", id++, pole_color, vmap, pole);
      if (isValidMarker(pole_marker))
        marker_array.markers.push_back(pole_marker);
      else
        ROS_ERROR_STREAM("[createSignalMarkerArray] failed createPoleMarker: " << pole);
    }
  }
  return marker_array;
}

visualization_msgs::MarkerArray createStreetLightMarkerArray(const VectorMap& vmap, Color light_color,
                                                             Color pole_color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& street_light : vmap.findByFilter([](const StreetLight& street_light){return true;}))
  {
    if (street_light.lid == 0)
    {
      ROS_ERROR_STREAM("[createStreetLightMarkerArray] invalid street_light: " << street_light);
      continue;
    }

    Line line = vmap.findByKey(Key<Line>(street_light.lid));
    if (line.lid == 0)
    {
      ROS_ERROR_STREAM("[createStreetLightMarkerArray] invalid line: " << line);
      continue;
    }

    Pole pole;
    if (street_light.plid != 0)
    {
      pole = vmap.findByKey(Key<Pole>(street_light.plid));
      if (pole.plid == 0)
      {
        ROS_ERROR_STREAM("[createStreetLightMarkerArray] invalid pole: " << pole);
        continue;
      }
    }

    if (line.blid == 0) // if beginning line
    {
      visualization_msgs::Marker line_marker = createLinkedLineMarker("street_light", id++, light_color, vmap, line);
      if (isValidMarker(line_marker))
        marker_array.markers.push_back(line_marker);
      else
        ROS_ERROR_STREAM("[createStreetLightMarkerArray] failed createLinkedLineMarker: " << line);
    }

    if (street_light.plid != 0)
    {
      visualization_msgs::Marker pole_marker = createPoleMarker("street_light", id++, pole_color, vmap, pole);
      if (isValidMarker(pole_marker))
        marker_array.markers.push_back(pole_marker);
      else
        ROS_ERROR_STREAM("[createStreetLightMarkerArray] failed createPoleMarker: " << pole);
    }
  }
  return marker_array;
}

visualization_msgs::MarkerArray createUtilityPoleMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& utility_pole : vmap.findByFilter([](const UtilityPole& utility_pole){return true;}))
  {
    if (utility_pole.plid == 0)
    {
      ROS_ERROR_STREAM("[createUtilityPoleMarkerArray] invalid utility_pole: " << utility_pole);
      continue;
    }

    Pole pole = vmap.findByKey(Key<Pole>(utility_pole.plid));
    if (pole.plid == 0)
    {
      ROS_ERROR_STREAM("[createUtilityPoleMarkerArray] invalid pole: " << pole);
      continue;
    }

    visualization_msgs::Marker marker = createPoleMarker("utility_pole", id++, color, vmap, pole);
    if (isValidMarker(marker))
      marker_array.markers.push_back(marker);
    else
      ROS_ERROR_STREAM("[createUtilityPoleMarkerArray] failed createPoleMarker: " << pole);
  }
  return marker_array;
}

visualization_msgs::MarkerArray createGuardRailMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& guard_rail : vmap.findByFilter([](const GuardRail& guard_rail){return true;}))
  {
    if (guard_rail.aid == 0)
    {
      ROS_ERROR_STREAM("[createGuardRailMarkerArray] invalid guard_rail: " << guard_rail);
      continue;
    }

    Area area = vmap.findByKey(Key<Area>(guard_rail.aid));
    if (area.aid == 0)
    {
      ROS_ERROR_STREAM("[createGuardRailMarkerArray] invalid area: " << area);
      continue;
    }

    visualization_msgs::Marker marker = createAreaMarker("guard_rail", id++, color, vmap, area);
    if (isValidMarker(marker))
      marker_array.markers.push_back(marker);
    else
      ROS_ERROR_STREAM("[createGuardRailMarkerArray] failed createAreaMarker: " << area);
  }
  return marker_array;
}

visualization_msgs::MarkerArray createSideWalkMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& side_walk : vmap.findByFilter([](const SideWalk& side_walk){return true;}))
  {
    if (side_walk.aid == 0)
    {
      ROS_ERROR_STREAM("[createSideWalkMarkerArray] invalid side_walk: " << side_walk);
      continue;
    }

    Area area = vmap.findByKey(Key<Area>(side_walk.aid));
    if (area.aid == 0)
    {
      ROS_ERROR_STREAM("[createSideWalkMarkerArray] invalid area: " << area);
      continue;
    }

    visualization_msgs::Marker marker = createAreaMarker("side_walk", id++, color, vmap, area);
    if (isValidMarker(marker))
      marker_array.markers.push_back(marker);
    else
      ROS_ERROR_STREAM("[createSideWalkMarkerArray] failed createAreaMarker: " << area);
  }
  return marker_array;
}

visualization_msgs::MarkerArray createDriveOnPortionMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& drive_on_portion : vmap.findByFilter([](const DriveOnPortion& drive_on_portion){return true;}))
  {
    if (drive_on_portion.aid == 0)
    {
      ROS_ERROR_STREAM("[createDriveOnPortionMarkerArray] invalid drive_on_portion: " << drive_on_portion);
      continue;
    }

    Area area = vmap.findByKey(Key<Area>(drive_on_portion.aid));
    if (area.aid == 0)
    {
      ROS_ERROR_STREAM("[createDriveOnPortionMarkerArray] invalid area: " << area);
      continue;
    }

    visualization_msgs::Marker marker = createAreaMarker("drive_on_portion", id++, color, vmap, area);
    if (isValidMarker(marker))
      marker_array.markers.push_back(marker);
    else
      ROS_ERROR_STREAM("[createDriveOnPortionMarkerArray] failed createAreaMarker: " << area);
  }
  return marker_array;
}

visualization_msgs::MarkerArray createCrossRoadMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& cross_road : vmap.findByFilter([](const CrossRoad& cross_road){return true;}))
  {
    if (cross_road.aid == 0)
    {
      ROS_ERROR_STREAM("[createCrossRoadMarkerArray] invalid cross_road: " << cross_road);
      continue;
    }

    Area area = vmap.findByKey(Key<Area>(cross_road.aid));
    if (area.aid == 0)
    {
      ROS_ERROR_STREAM("[createCrossRoadMarkerArray] invalid area: " << area);
      continue;
    }

    visualization_msgs::Marker marker = createAreaMarker("cross_road", id++, color, vmap, area);
    if (isValidMarker(marker))
      marker_array.markers.push_back(marker);
    else
      ROS_ERROR_STREAM("[createCrossRoadMarkerArray] failed createAreaMarker: " << area);
  }
  return marker_array;
}

visualization_msgs::MarkerArray createSideStripMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& side_strip : vmap.findByFilter([](const SideStrip& side_strip){return true;}))
  {
    if (side_strip.lid == 0)
    {
      ROS_ERROR_STREAM("[createSideStripMarkerArray] invalid side_strip: " << side_strip);
      continue;
    }

    Line line = vmap.findByKey(Key<Line>(side_strip.lid));
    if (line.lid == 0)
    {
      ROS_ERROR_STREAM("[createSideStripMarkerArray] invalid line: " << line);
      continue;
    }

    if (line.blid == 0) // if beginning line
    {
      visualization_msgs::Marker marker = createLinkedLineMarker("side_strip", id++, color, vmap, line);
      if (isValidMarker(marker))
        marker_array.markers.push_back(marker);
      else
        ROS_ERROR_STREAM("[createSideStripMarkerArray] failed createLinkedLineMarker: " << line);
    }
  }
  return marker_array;
}

visualization_msgs::MarkerArray createCurveMirrorMarkerArray(const VectorMap& vmap, Color mirror_color,
                                                             Color pole_color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& curve_mirror : vmap.findByFilter([](const CurveMirror& curve_mirror){return true;}))
  {
    if (curve_mirror.vid == 0 || curve_mirror.plid == 0)
    {
      ROS_ERROR_STREAM("[createCurveMirrorMarkerArray] invalid curve_mirror: " << curve_mirror);
      continue;
    }

    Vector vector = vmap.findByKey(Key<Vector>(curve_mirror.vid));
    if (vector.vid == 0)
    {
      ROS_ERROR_STREAM("[createCurveMirrorMarkerArray] invalid vector: " << vector);
      continue;
    }

    Pole pole = vmap.findByKey(Key<Pole>(curve_mirror.plid));
    if (pole.plid == 0)
    {
      ROS_ERROR_STREAM("[createCurveMirrorMarkerArray] invalid pole: " << pole);
      continue;
    }

    visualization_msgs::Marker vector_marker = createVectorMarker("curve_mirror", id++, mirror_color, vmap, vector);
    if (isValidMarker(vector_marker))
      marker_array.markers.push_back(vector_marker);
    else
      ROS_ERROR_STREAM("[createCurveMirrorMarkerArray] failed createVectorMarker: " << vector);

    visualization_msgs::Marker pole_marker = createPoleMarker("curve_mirror", id++, pole_color, vmap, pole);
    if (isValidMarker(pole_marker))
      marker_array.markers.push_back(pole_marker);
    else
      ROS_ERROR_STREAM("[createCurveMirrorMarkerArray] failed createPoleMarker: " << pole);
  }
  return marker_array;
}

visualization_msgs::MarkerArray createWallMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& wall : vmap.findByFilter([](const Wall& wall){return true;}))
  {
    if (wall.aid == 0)
    {
      ROS_ERROR_STREAM("[createWallMarkerArray] invalid wall: " << wall);
      continue;
    }

    Area area = vmap.findByKey(Key<Area>(wall.aid));
    if (area.aid == 0)
    {
      ROS_ERROR_STREAM("[createWallMarkerArray] invalid area: " << area);
      continue;
    }

    visualization_msgs::Marker marker = createAreaMarker("wall", id++, color, vmap, area);
    if (isValidMarker(marker))
      marker_array.markers.push_back(marker);
    else
      ROS_ERROR_STREAM("[createWallMarkerArray] failed createAreaMarker: " << area);
  }
  return marker_array;
}

visualization_msgs::MarkerArray createFenceMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& fence : vmap.findByFilter([](const Fence& fence){return true;}))
  {
    if (fence.aid == 0)
    {
      ROS_ERROR_STREAM("[createFenceMarkerArray] invalid fence: " << fence);
      continue;
    }

    Area area = vmap.findByKey(Key<Area>(fence.aid));
    if (area.aid == 0)
    {
      ROS_ERROR_STREAM("[createFenceMarkerArray] invalid area: " << area);
      continue;
    }

    visualization_msgs::Marker marker = createAreaMarker("fence", id++, color, vmap, area);
    if (isValidMarker(marker))
      marker_array.markers.push_back(marker);
    else
      ROS_ERROR_STREAM("[createFenceMarkerArray] failed createAreaMarker: " << area);
  }
  return marker_array;
}

visualization_msgs::MarkerArray createRailCrossingMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& rail_crossing : vmap.findByFilter([](const RailCrossing& rail_crossing){return true;}))
  {
    if (rail_crossing.aid == 0)
    {
      ROS_ERROR_STREAM("[createRailCrossingMarkerArray] invalid rail_crossing: " << rail_crossing);
      continue;
    }

    Area area = vmap.findByKey(Key<Area>(rail_crossing.aid));
    if (area.aid == 0)
    {
      ROS_ERROR_STREAM("[createRailCrossingMarkerArray] invalid area: " << area);
      continue;
    }

    visualization_msgs::Marker marker = createAreaMarker("rail_crossing", id++, color, vmap, area);
    if (isValidMarker(marker))
      marker_array.markers.push_back(marker);
    else
      ROS_ERROR_STREAM("[createRailCrossingMarkerArray] failed createAreaMarker: " << area);
  }
  return marker_array;
}


template <class T, class U>
U createObjectArray(const std::string& file_path)
{
	U obj_array;
	// NOTE: Autoware want to use map messages with or without /use_sim_time.
	// Therefore we don't set obj_array.header.stamp.
	// obj_array.header.stamp = ros::Time::now();
	obj_array.header.frame_id = "map";
	obj_array.data = vector_map::parse<T>(file_path);
	return obj_array;
}


VectorMapLoader::VectorMapLoader(const std::string &directory) :
	vmDir(directory)
{
	if (!is_directory(vmDir))
		throw runtime_error("Not a directory");

	loadAll ();
}

// Strange bits for accessing private members

typedef vector_map::Handle<Point, PointArray> _HandlePoint;
typedef vector_map::Handle<Vector, VectorArray> _HandleVector;
typedef vector_map::Handle<Line, LineArray> _HandleLine;
typedef vector_map::Handle<Area, AreaArray> _HandleArea;
typedef vector_map::Handle<Pole, PoleArray> _HandlePole;
typedef vector_map::Handle<Box, BoxArray> _HandleBox;
typedef vector_map::Handle<DTLane, DTLaneArray> _HandleDTLane;
typedef vector_map::Handle<Node, NodeArray> _HandleNode;
typedef vector_map::Handle<Lane, LaneArray> _HandleLane;
typedef vector_map::Handle<WayArea, WayAreaArray> _HandleWayArea;
typedef vector_map::Handle<RoadEdge, RoadEdgeArray> _HandleRoadEdge;
typedef vector_map::Handle<Gutter, GutterArray> _HandleGutter;
typedef vector_map::Handle<Curb, CurbArray> _HandleCurb;
typedef vector_map::Handle<WhiteLine, WhiteLineArray> _HandleWhiteLine;
typedef vector_map::Handle<StopLine, StopLineArray> _HandleStopLine;
typedef vector_map::Handle<ZebraZone, ZebraZoneArray> _HandleZebraZone;
typedef vector_map::Handle<CrossWalk, CrossWalkArray> _HandleCrossWalk;
typedef vector_map::Handle<RoadMark, RoadMarkArray> _HandleRoadMark;
typedef vector_map::Handle<RoadPole, RoadPoleArray> _HandleRoadPole;
typedef vector_map::Handle<RoadSign, RoadSignArray> _HandleRoadSign;
typedef vector_map::Handle<Signal, SignalArray> _HandleSignal;
typedef vector_map::Handle<StreetLight, StreetLightArray> _HandleStreetLight;
typedef vector_map::Handle<UtilityPole, UtilityPoleArray> _HandleUtilityPole;
typedef vector_map::Handle<GuardRail, GuardRailArray> _HandleGuardRail;
typedef vector_map::Handle<SideWalk, SideWalkArray> _HandleSideWalk;
typedef vector_map::Handle<DriveOnPortion, DriveOnPortionArray> _HandleDriveOnPortion;
typedef vector_map::Handle<CrossRoad, CrossRoadArray> _HandleCrossRoad;
typedef vector_map::Handle<SideStrip, SideStripArray> _HandleSideStrip;
typedef vector_map::Handle<CurveMirror, CurveMirrorArray> _HandleCurveMirror;
typedef vector_map::Handle<Wall, WallArray> _HandleWall;
typedef vector_map::Handle<Fence, FenceArray> _HandleFence;
typedef vector_map::Handle<RailCrossing, RailCrossingArray> _HandleRailCrossing;

ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandlePoint, point_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleVector, vector_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleLine, line_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleArea, area_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandlePole, pole_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleBox, box_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleDTLane, dtlane_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleNode, node_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleLane, lane_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleWayArea, way_area_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleRoadEdge, road_edge_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleGutter, gutter_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleCurb, curb_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleWhiteLine, white_line_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleStopLine, stop_line_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleZebraZone, zebra_zone_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleCrossWalk, cross_walk_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleRoadMark, road_mark_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleRoadPole, road_pole_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleRoadSign, road_sign_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleSignal, signal_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleStreetLight, street_light_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleUtilityPole, utility_pole_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleGuardRail, guard_rail_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleSideWalk, side_walk_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleDriveOnPortion, drive_on_portion_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleCrossRoad, cross_road_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleSideStrip, side_strip_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleCurveMirror, curve_mirror_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleWall, wall_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleFence, fence_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, _HandleRailCrossing, rail_crossing_);

typedef std::map<vector_map::Key<Point>, Point> _MapPoint;
typedef std::map<vector_map::Key<Vector>, Vector> _MapVector;
typedef std::map<vector_map::Key<Line>, Line> _MapLine;
typedef std::map<vector_map::Key<Area>, Area> _MapArea;
typedef std::map<vector_map::Key<Pole>, Pole> _MapPole;
typedef std::map<vector_map::Key<Box>, Box> _MapBox;
typedef std::map<vector_map::Key<DTLane>, DTLane> _MapDTLane;
typedef std::map<vector_map::Key<Node>, Node> _MapNode;
typedef std::map<vector_map::Key<Lane>, Lane> _MapLane;
typedef std::map<vector_map::Key<WayArea>, WayArea> _MapWayArea;
typedef std::map<vector_map::Key<RoadEdge>, RoadEdge> _MapRoadEdge;
typedef std::map<vector_map::Key<Gutter>, Gutter> _MapGutter;
typedef std::map<vector_map::Key<Curb>, Curb> _MapCurb;
typedef std::map<vector_map::Key<WhiteLine>, WhiteLine> _MapWhiteLine;
typedef std::map<vector_map::Key<StopLine>, StopLine> _MapStopLine;
typedef std::map<vector_map::Key<ZebraZone>, ZebraZone> _MapZebraZone;
typedef std::map<vector_map::Key<CrossWalk>, CrossWalk> _MapCrossWalk;
typedef std::map<vector_map::Key<RoadMark>, RoadMark> _MapRoadMark;
typedef std::map<vector_map::Key<RoadPole>, RoadPole> _MapRoadPole;
typedef std::map<vector_map::Key<RoadSign>, RoadSign> _MapRoadSign;
typedef std::map<vector_map::Key<Signal>, Signal> _MapSignal;
typedef std::map<vector_map::Key<StreetLight>, StreetLight> _MapStreetLight;
typedef std::map<vector_map::Key<UtilityPole>, UtilityPole> _MapUtilityPole;
typedef std::map<vector_map::Key<GuardRail>, GuardRail> _MapGuardRail;
typedef std::map<vector_map::Key<SideWalk>, SideWalk> _MapSideWalk;
typedef std::map<vector_map::Key<DriveOnPortion>, DriveOnPortion> _MapDriveOnPortion;
typedef std::map<vector_map::Key<CrossRoad>, CrossRoad> _MapCrossRoad;
typedef std::map<vector_map::Key<SideStrip>, SideStrip> _MapSideStrip;
typedef std::map<vector_map::Key<CurveMirror>, CurveMirror> _MapCurveMirror;
typedef std::map<vector_map::Key<Wall>, Wall> _MapWall;
typedef std::map<vector_map::Key<Fence>, Fence> _MapFence;
typedef std::map<vector_map::Key<RailCrossing>, RailCrossing> _MapRailCrossing;

ACCESS_PRIVATE_FIELD(_HandlePoint, _MapPoint, map_);
ACCESS_PRIVATE_FIELD(_HandleVector, _MapVector, map_);
ACCESS_PRIVATE_FIELD(_HandleLine, _MapLine, map_);
ACCESS_PRIVATE_FIELD(_HandleArea, _MapArea, map_);
ACCESS_PRIVATE_FIELD(_HandlePole, _MapPole, map_);
ACCESS_PRIVATE_FIELD(_HandleBox, _MapBox, map_);
ACCESS_PRIVATE_FIELD(_HandleDTLane, _MapDTLane, map_);
ACCESS_PRIVATE_FIELD(_HandleNode, _MapNode, map_);
ACCESS_PRIVATE_FIELD(_HandleLane, _MapLane, map_);
ACCESS_PRIVATE_FIELD(_HandleWayArea, _MapWayArea, map_);
ACCESS_PRIVATE_FIELD(_HandleRoadEdge, _MapRoadEdge, map_);
ACCESS_PRIVATE_FIELD(_HandleGutter, _MapGutter, map_);
ACCESS_PRIVATE_FIELD(_HandleCurb, _MapCurb, map_);
ACCESS_PRIVATE_FIELD(_HandleWhiteLine, _MapWhiteLine, map_);
ACCESS_PRIVATE_FIELD(_HandleStopLine, _MapStopLine, map_);
ACCESS_PRIVATE_FIELD(_HandleZebraZone, _MapZebraZone, map_);
ACCESS_PRIVATE_FIELD(_HandleCrossWalk, _MapCrossWalk, map_);
ACCESS_PRIVATE_FIELD(_HandleRoadMark, _MapRoadMark, map_);
ACCESS_PRIVATE_FIELD(_HandleRoadPole, _MapRoadPole, map_);
ACCESS_PRIVATE_FIELD(_HandleRoadSign, _MapRoadSign, map_);
ACCESS_PRIVATE_FIELD(_HandleSignal, _MapSignal, map_);
ACCESS_PRIVATE_FIELD(_HandleStreetLight, _MapStreetLight, map_);
ACCESS_PRIVATE_FIELD(_HandleUtilityPole, _MapUtilityPole, map_);
ACCESS_PRIVATE_FIELD(_HandleGuardRail, _MapGuardRail, map_);
ACCESS_PRIVATE_FIELD(_HandleSideWalk, _MapSideWalk, map_);
ACCESS_PRIVATE_FIELD(_HandleDriveOnPortion, _MapDriveOnPortion, map_);
ACCESS_PRIVATE_FIELD(_HandleCrossRoad, _MapCrossRoad, map_);
ACCESS_PRIVATE_FIELD(_HandleSideStrip, _MapSideStrip, map_);
ACCESS_PRIVATE_FIELD(_HandleCurveMirror, _MapCurveMirror, map_);
ACCESS_PRIVATE_FIELD(_HandleWall, _MapWall, map_);
ACCESS_PRIVATE_FIELD(_HandleFence, _MapFence, map_);
ACCESS_PRIVATE_FIELD(_HandleRailCrossing, _MapRailCrossing, map_);



void VectorMapLoader::loadAll()
{
	auto point_pth = vmDir / path("point.csv");
	auto vector_pth = vmDir / path("vector.csv");
	auto line_pth = vmDir / path("line.csv");
	auto area_pth = vmDir / path("area.csv");
	auto pole_pth = vmDir / path("pole.csv");
	auto box_pth = vmDir / path("box.csv");
	auto dtlane_pth = vmDir / path("dtlane.csv");
	auto node_pth = vmDir / path("node.csv");
	auto lane_pth = vmDir / path("lane.csv");
	auto way_area_pth = vmDir / path("wayarea.csv");
	auto road_edge_pth = vmDir / path("roadedge.csv");
	auto gutter_pth = vmDir / path("gutter.csv");
	auto curb_pth = vmDir / path("curb.csv");
	auto white_line_pth = vmDir / path("whiteline.csv");
	auto stop_line_pth = vmDir / path("stopline.csv");
	auto zebra_zone_pth = vmDir / path("zebrazone.csv");
	auto cross_walk_pth = vmDir / path("crosswalk.csv");
	auto road_mark_pth = vmDir / path("road_surface_mark.csv");
	auto road_pole_pth = vmDir / path("poledata.csv");
	auto road_sign_pth = vmDir / path("roadsign.csv");
	auto signal_pth = vmDir / path("signaldata.csv");
	auto street_light_pth = vmDir / path("streetlight.csv");
	auto utility_pole_pth = vmDir / path("utilitypole.csv");
	auto guard_rail_pth = vmDir / path("guardrail.csv");
	auto side_walk_pth = vmDir / path("sidewalk.csv");
	auto drive_on_portion_pth = vmDir / path("driveon_portion.csv");
	auto cross_road_pth = vmDir / path("intersection.csv");
	auto side_strip_pth = vmDir / path("sidestrip.csv");
	auto curve_mirror_pth = vmDir / path("curvemirror.csv");
	auto wall_pth = vmDir / path("wall.csv");
	auto fence_pth = vmDir / path("fence.csv");
	auto rail_crossing_pth = vmDir / path("railroad_crossing.csv");

	auto PointArrayObj = createObjectArray<Point, PointArray> (point_pth.string());
	auto VectorArrayObj = createObjectArray<Vector, VectorArray> (vector_pth.string());
	auto LineArrayObj = createObjectArray<Line, LineArray> (line_pth.string());
	auto AreaArrayObj = createObjectArray<Area, AreaArray> (area_pth.string());
	auto PoleArrayObj = createObjectArray<Pole, PoleArray> (pole_pth.string());
	auto BoxArrayObj = createObjectArray<Box, BoxArray> (box_pth.string());
	auto DTLaneArrayObj = createObjectArray<DTLane, DTLaneArray> (dtlane_pth.string());
	auto NodeArrayObj = createObjectArray<Node, NodeArray> (node_pth.string());
	auto LaneArrayObj = createObjectArray<Lane, LaneArray> (lane_pth.string());
	auto WayAreaArrayObj = createObjectArray<WayArea, WayAreaArray> (way_area_pth.string());
	auto RoadEdgeArrayObj = createObjectArray<RoadEdge, RoadEdgeArray> (road_edge_pth.string());
	auto GutterArrayObj = createObjectArray<Gutter, GutterArray> (gutter_pth.string());
	auto CurbArrayObj = createObjectArray<Curb, CurbArray> (curb_pth.string());
	auto WhiteLineArrayObj = createObjectArray<WhiteLine, WhiteLineArray> (white_line_pth.string());
	auto StopLineArrayObj = createObjectArray<StopLine, StopLineArray> (stop_line_pth.string());
	auto ZebraZoneArrayObj = createObjectArray<ZebraZone, ZebraZoneArray> (zebra_zone_pth.string());
	auto CrossWalkArrayObj = createObjectArray<CrossWalk, CrossWalkArray> (cross_walk_pth.string());
	auto RoadMarkArrayObj = createObjectArray<RoadMark, RoadMarkArray> (road_mark_pth.string());
	auto RoadPoleArrayObj = createObjectArray<RoadPole, RoadPoleArray> (road_pole_pth.string());
	auto RoadSignArrayObj = createObjectArray<RoadSign, RoadSignArray> (road_sign_pth.string());
	auto SignalArrayObj = createObjectArray<Signal, SignalArray> (signal_pth.string());
	auto StreetLightArrayObj = createObjectArray<StreetLight, StreetLightArray> (street_light_pth.string());
	auto UtilityPoleArrayObj = createObjectArray<UtilityPole, UtilityPoleArray> (utility_pole_pth.string());
	auto GuardRailArrayObj = createObjectArray<GuardRail, GuardRailArray> (guard_rail_pth.string());
	auto SideWalkArrayObj = createObjectArray<SideWalk, SideWalkArray> (side_walk_pth.string());
	auto DriveOnPortionArrayObj = createObjectArray<DriveOnPortion, DriveOnPortionArray> (drive_on_portion_pth.string());
	auto CrossRoadArrayObj = createObjectArray<CrossRoad, CrossRoadArray> (cross_road_pth.string());
	auto SideStripArrayObj = createObjectArray<SideStrip, SideStripArray> (side_strip_pth.string());
	auto CurveMirrorArrayObj = createObjectArray<CurveMirror, CurveMirrorArray> (curve_mirror_pth.string());
	auto WallArrayObj = createObjectArray<Wall, WallArray> (wall_pth.string());
	auto FenceArrayObj = createObjectArray<Fence, FenceArray> (fence_pth.string());
	auto RailCrossingArrayObj = createObjectArray<RailCrossing, RailCrossingArray> (rail_crossing_pth.string());

	updatePoint (access_private::map_(access_private::point_(*this)), PointArrayObj);
	updateVector (access_private::map_(access_private::vector_(*this)), VectorArrayObj);
	updateLine (access_private::map_(access_private::line_(*this)), LineArrayObj);
	updateArea (access_private::map_(access_private::area_(*this)), AreaArrayObj);
	updatePole (access_private::map_(access_private::pole_(*this)), PoleArrayObj);
	updateBox (access_private::map_(access_private::box_(*this)), BoxArrayObj);
	updateDTLane (access_private::map_(access_private::dtlane_(*this)), DTLaneArrayObj);
	updateNode (access_private::map_(access_private::node_(*this)), NodeArrayObj);
	updateLane (access_private::map_(access_private::lane_(*this)), LaneArrayObj);
	updateWayArea (access_private::map_(access_private::way_area_(*this)), WayAreaArrayObj);
	updateRoadEdge (access_private::map_(access_private::road_edge_(*this)), RoadEdgeArrayObj);
	updateGutter (access_private::map_(access_private::gutter_(*this)), GutterArrayObj);
	updateCurb (access_private::map_(access_private::curb_(*this)), CurbArrayObj);
	updateWhiteLine (access_private::map_(access_private::white_line_(*this)), WhiteLineArrayObj);
	updateStopLine (access_private::map_(access_private::stop_line_(*this)), StopLineArrayObj);
	updateZebraZone (access_private::map_(access_private::zebra_zone_(*this)), ZebraZoneArrayObj);
	updateCrossWalk (access_private::map_(access_private::cross_walk_(*this)), CrossWalkArrayObj);
	updateRoadMark (access_private::map_(access_private::road_mark_(*this)), RoadMarkArrayObj);
	updateRoadPole (access_private::map_(access_private::road_pole_(*this)), RoadPoleArrayObj);
	updateRoadSign (access_private::map_(access_private::road_sign_(*this)), RoadSignArrayObj);
	updateSignal (access_private::map_(access_private::signal_(*this)), SignalArrayObj);
	updateStreetLight (access_private::map_(access_private::street_light_(*this)), StreetLightArrayObj);
	updateUtilityPole (access_private::map_(access_private::utility_pole_(*this)), UtilityPoleArrayObj);
	updateGuardRail (access_private::map_(access_private::guard_rail_(*this)), GuardRailArrayObj);
	updateSideWalk (access_private::map_(access_private::side_walk_(*this)), SideWalkArrayObj);
	updateDriveOnPortion (access_private::map_(access_private::drive_on_portion_(*this)), DriveOnPortionArrayObj);
	updateCrossRoad (access_private::map_(access_private::cross_road_(*this)), CrossRoadArrayObj);
	updateSideStrip (access_private::map_(access_private::side_strip_(*this)), SideStripArrayObj);
	updateCurveMirror (access_private::map_(access_private::curve_mirror_(*this)), CurveMirrorArrayObj);
	updateWall (access_private::map_(access_private::wall_(*this)), WallArrayObj);
	updateFence (access_private::map_(access_private::fence_(*this)), FenceArrayObj);
	updateRailCrossing (access_private::map_(access_private::rail_crossing_(*this)), RailCrossingArrayObj);

	// At this point, the files have been loaded. Let's backup the points before any transform operation
	const auto &pointMap = access_private::map_(access_private::point_(*this));
	pointOrig.insert(pointMap.begin(), pointMap.end());
}


visualization_msgs::MarkerArray::ConstPtr
VectorMapLoader::getAsMessages()
{
	visualization_msgs::MarkerArray::Ptr msgPtr(new visualization_msgs::MarkerArray);
	insertMarkerArray(*msgPtr, createRoadEdgeMarkerArray(*this, Color::GRAY));
	insertMarkerArray(*msgPtr, createGutterMarkerArray(*this, Color::GRAY, Color::GRAY, Color::GRAY));
	insertMarkerArray(*msgPtr, createCurbMarkerArray(*this, Color::GRAY));
	insertMarkerArray(*msgPtr, createWhiteLineMarkerArray(*this, Color::WHITE, Color::YELLOW));
	insertMarkerArray(*msgPtr, createStopLineMarkerArray(*this, Color::WHITE));
	insertMarkerArray(*msgPtr, createZebraZoneMarkerArray(*this, Color::WHITE));
	insertMarkerArray(*msgPtr, createCrossWalkMarkerArray(*this, Color::WHITE));
	insertMarkerArray(*msgPtr, createRoadMarkMarkerArray(*this, Color::WHITE));
	insertMarkerArray(*msgPtr, createRoadPoleMarkerArray(*this, Color::GRAY));
	insertMarkerArray(*msgPtr, createRoadSignMarkerArray(*this, Color::GREEN, Color::GRAY));
	insertMarkerArray(*msgPtr, createSignalMarkerArray(*this, Color::RED, Color::BLUE, Color::YELLOW, Color::CYAN,
														  Color::GRAY));
	insertMarkerArray(*msgPtr, createStreetLightMarkerArray(*this, Color::YELLOW, Color::GRAY));
	insertMarkerArray(*msgPtr, createUtilityPoleMarkerArray(*this, Color::GRAY));
	insertMarkerArray(*msgPtr, createGuardRailMarkerArray(*this, Color::LIGHT_BLUE));
	insertMarkerArray(*msgPtr, createSideWalkMarkerArray(*this, Color::GRAY));
	insertMarkerArray(*msgPtr, createDriveOnPortionMarkerArray(*this, Color::LIGHT_CYAN));
	insertMarkerArray(*msgPtr, createCrossRoadMarkerArray(*this, Color::LIGHT_GREEN));
	insertMarkerArray(*msgPtr, createSideStripMarkerArray(*this, Color::GRAY));
	insertMarkerArray(*msgPtr, createCurveMirrorMarkerArray(*this, Color::MAGENTA, Color::GRAY));
	insertMarkerArray(*msgPtr, createWallMarkerArray(*this, Color::LIGHT_YELLOW));
	insertMarkerArray(*msgPtr, createFenceMarkerArray(*this, Color::LIGHT_RED));
	insertMarkerArray(*msgPtr, createRailCrossingMarkerArray(*this, Color::LIGHT_MAGENTA));

	return msgPtr;
}


void
VectorMapLoader::setPointOffset
(const ptScalar x_offset, const ptScalar y_offset, const ptScalar z_offset)
{
	auto &pointMap = access_private::map_(access_private::point_(*this));
	for (auto &pt: pointOrig) {
		auto key = pt.first;
		pointMap[key].bx = pt.second.bx + y_offset;
		pointMap[key].ly = pt.second.ly + x_offset;
		pointMap[key].h = pt.second.h + z_offset;
	}
}
