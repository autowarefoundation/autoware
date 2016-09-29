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

#include <tf/transform_datatypes.h>
#include <vector_map/vector_map.h>

namespace vector_map
{
namespace
{
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
} // namespace

bool VectorMap::hasSubscribed(category_t category) const
{
  if (category & POINT)
  {
    if (point_.empty())
      return false;
  }
  if (category & VECTOR)
  {
    if (vector_.empty())
      return false;
  }
  if (category & LINE)
  {
    if (line_.empty())
      return false;
  }
  if (category & AREA)
  {
    if (area_.empty())
      return false;
  }
  if (category & POLE)
  {
    if (pole_.empty())
      return false;
  }
  if (category & BOX)
  {
    if (box_.empty())
      return false;
  }
  if (category & DTLANE)
  {
    if (dtlane_.empty())
      return false;
  }
  if (category & NODE)
  {
    if (node_.empty())
      return false;
  }
  if (category & LANE)
  {
    if (lane_.empty())
      return false;
  }
  if (category & WAY_AREA)
  {
    if (way_area_.empty())
      return false;
  }
  if (category & ROAD_EDGE)
  {
    if (road_edge_.empty())
      return false;
  }
  if (category & GUTTER)
  {
    if (gutter_.empty())
      return false;
  }
  if (category & CURB)
  {
    if (curb_.empty())
      return false;
  }
  if (category & WHITE_LINE)
  {
    if (white_line_.empty())
      return false;
  }
  if (category & STOP_LINE)
  {
    if (stop_line_.empty())
      return false;
  }
  if (category & ZEBRA_ZONE)
  {
    if (zebra_zone_.empty())
      return false;
  }
  if (category & CROSS_WALK)
  {
    if (cross_walk_.empty())
      return false;
  }
  if (category & ROAD_MARK)
  {
    if (road_mark_.empty())
      return false;
  }
  if (category & ROAD_POLE)
  {
    if (road_pole_.empty())
      return false;
  }
  if (category & ROAD_SIGN)
  {
    if (road_sign_.empty())
      return false;
  }
  if (category & SIGNAL)
  {
    if (signal_.empty())
      return false;
  }
  if (category & STREET_LIGHT)
  {
    if (street_light_.empty())
      return false;
  }
  if (category & UTILITY_POLE)
  {
    if (utility_pole_.empty())
      return false;
  }
  if (category & GUARD_RAIL)
  {
    if (guard_rail_.empty())
      return false;
  }
  if (category & SIDE_WALK)
  {
    if (side_walk_.empty())
      return false;
  }
  if (category & DRIVE_ON_PORTION)
  {
    if (drive_on_portion_.empty())
      return false;
  }
  if (category & CROSS_ROAD)
  {
    if (cross_road_.empty())
      return false;
  }
  if (category & SIDE_STRIP)
  {
    if (side_strip_.empty())
      return false;
  }
  if (category & CURVE_MIRROR)
  {
    if (curve_mirror_.empty())
      return false;
  }
  if (category & WALL)
  {
    if (wall_.empty())
      return false;
  }
  if (category & FENCE)
  {
    if (fence_.empty())
      return false;
  }
  if (category & RAIL_CROSSING)
  {
    if (rail_crossing_.empty())
      return false;
  }
  return true;
}

void VectorMap::registerSubscriber(ros::NodeHandle& nh, category_t category)
{
  if (category & POINT)
  {
    point_.registerSubscriber(nh, "vector_map_info/point");
    point_.registerUpdater(updatePoint);
  }
  if (category & VECTOR)
  {
    vector_.registerSubscriber(nh, "vector_map_info/vector");
    vector_.registerUpdater(updateVector);
  }
  if (category & LINE)
  {
    line_.registerSubscriber(nh, "vector_map_info/line");
    line_.registerUpdater(updateLine);
  }
  if (category & AREA)
  {
    area_.registerSubscriber(nh, "vector_map_info/area");
    area_.registerUpdater(updateArea);
  }
  if (category & POLE)
  {
    pole_.registerSubscriber(nh, "vector_map_info/pole");
    pole_.registerUpdater(updatePole);
  }
  if (category & BOX)
  {
    box_.registerSubscriber(nh, "vector_map_info/box");
    box_.registerUpdater(updateBox);
  }
  if (category & DTLANE)
  {
    dtlane_.registerSubscriber(nh, "vector_map_info/dtlane");
    dtlane_.registerUpdater(updateDTLane);
  }
  if (category & NODE)
  {
    node_.registerSubscriber(nh, "vector_map_info/node");
    node_.registerUpdater(updateNode);
  }
  if (category & LANE)
  {
    lane_.registerSubscriber(nh, "vector_map_info/lane");
    lane_.registerUpdater(updateLane);
  }
  if (category & WAY_AREA)
  {
    way_area_.registerSubscriber(nh, "vector_map_info/way_area");
    way_area_.registerUpdater(updateWayArea);
  }
  if (category & ROAD_EDGE)
  {
    road_edge_.registerSubscriber(nh, "vector_map_info/road_edge");
    road_edge_.registerUpdater(updateRoadEdge);
  }
  if (category & GUTTER)
  {
    gutter_.registerSubscriber(nh, "vector_map_info/gutter");
    gutter_.registerUpdater(updateGutter);
  }
  if (category & CURB)
  {
    curb_.registerSubscriber(nh, "vector_map_info/curb");
    curb_.registerUpdater(updateCurb);
  }
  if (category & WHITE_LINE)
  {
    white_line_.registerSubscriber(nh, "vector_map_info/white_line");
    white_line_.registerUpdater(updateWhiteLine);
  }
  if (category & STOP_LINE)
  {
    stop_line_.registerSubscriber(nh, "vector_map_info/stop_line");
    stop_line_.registerUpdater(updateStopLine);
  }
  if (category & ZEBRA_ZONE)
  {
    zebra_zone_.registerSubscriber(nh, "vector_map_info/zebra_zone");
    zebra_zone_.registerUpdater(updateZebraZone);
  }
  if (category & CROSS_WALK)
  {
    cross_walk_.registerSubscriber(nh, "vector_map_info/cross_walk");
    cross_walk_.registerUpdater(updateCrossWalk);
  }
  if (category & ROAD_MARK)
  {
    road_mark_.registerSubscriber(nh, "vector_map_info/road_mark");
    road_mark_.registerUpdater(updateRoadMark);
  }
  if (category & ROAD_POLE)
  {
    road_pole_.registerSubscriber(nh, "vector_map_info/road_pole");
    road_pole_.registerUpdater(updateRoadPole);
  }
  if (category & ROAD_SIGN)
  {
    road_sign_.registerSubscriber(nh, "vector_map_info/road_sign");
    road_sign_.registerUpdater(updateRoadSign);
  }
  if (category & SIGNAL)
  {
    signal_.registerSubscriber(nh, "vector_map_info/signal");
    signal_.registerUpdater(updateSignal);
  }
  if (category & STREET_LIGHT)
  {
    street_light_.registerSubscriber(nh, "vector_map_info/street_light");
    street_light_.registerUpdater(updateStreetLight);
  }
  if (category & UTILITY_POLE)
  {
    utility_pole_.registerSubscriber(nh, "vector_map_info/utility_pole");
    utility_pole_.registerUpdater(updateUtilityPole);
  }
  if (category & GUARD_RAIL)
  {
    guard_rail_.registerSubscriber(nh, "vector_map_info/guard_rail");
    guard_rail_.registerUpdater(updateGuardRail);
  }
  if (category & SIDE_WALK)
  {
    side_walk_.registerSubscriber(nh, "vector_map_info/side_walk");
    side_walk_.registerUpdater(updateSideWalk);
  }
  if (category & DRIVE_ON_PORTION)
  {
    drive_on_portion_.registerSubscriber(nh, "vector_map_info/drive_on_portion");
    drive_on_portion_.registerUpdater(updateDriveOnPortion);
  }
  if (category & CROSS_ROAD)
  {
    cross_road_.registerSubscriber(nh, "vector_map_info/cross_road");
    cross_road_.registerUpdater(updateCrossRoad);
  }
  if (category & SIDE_STRIP)
  {
    side_strip_.registerSubscriber(nh, "vector_map_info/side_strip");
    side_strip_.registerUpdater(updateSideStrip);
  }
  if (category & CURVE_MIRROR)
  {
    curve_mirror_.registerSubscriber(nh, "vector_map_info/curve_mirror");
    curve_mirror_.registerUpdater(updateCurveMirror);
  }
  if (category & WALL)
  {
    wall_.registerSubscriber(nh, "vector_map_info/wall");
    wall_.registerUpdater(updateWall);
  }
  if (category & FENCE)
  {
    fence_.registerSubscriber(nh, "vector_map_info/fence");
    fence_.registerUpdater(updateFence);
  }
  if (category & RAIL_CROSSING)
  {
    rail_crossing_.registerSubscriber(nh, "vector_map_info/rail_crossing");
    rail_crossing_.registerUpdater(updateRailCrossing);
  }
}

VectorMap::VectorMap()
{
}

void VectorMap::subscribe(ros::NodeHandle& nh, category_t category)
{
  registerSubscriber(nh, category);
  ros::Rate rate(1);
  while (ros::ok() && !hasSubscribed(category))
  {
    ros::spinOnce();
    rate.sleep();
  }
}

void VectorMap::subscribe(ros::NodeHandle& nh, category_t category, const ros::Duration& timeout)
{
  registerSubscriber(nh, category);
  ros::Rate rate(1);
  ros::Time end = ros::Time::now() + timeout;
  while (ros::ok() && !hasSubscribed(category) && ros::Time::now() < end)
  {
    ros::spinOnce();
    rate.sleep();
  }
}

Point VectorMap::findByKey(const Key<Point>& key) const
{
  return point_.findByKey(key);
}

Vector VectorMap::findByKey(const Key<Vector>& key) const
{
  return vector_.findByKey(key);
}

Line VectorMap::findByKey(const Key<Line>& key) const
{
  return line_.findByKey(key);
}

Area VectorMap::findByKey(const Key<Area>& key) const
{
  return area_.findByKey(key);
}

Pole VectorMap::findByKey(const Key<Pole>& key) const
{
  return pole_.findByKey(key);
}

Box VectorMap::findByKey(const Key<Box>& key) const
{
  return box_.findByKey(key);
}

DTLane VectorMap::findByKey(const Key<DTLane>& key) const
{
  return dtlane_.findByKey(key);
}

Node VectorMap::findByKey(const Key<Node>& key) const
{
  return node_.findByKey(key);
}

Lane VectorMap::findByKey(const Key<Lane>& key) const
{
  return lane_.findByKey(key);
}

WayArea VectorMap::findByKey(const Key<WayArea>& key) const
{
  return way_area_.findByKey(key);
}

RoadEdge VectorMap::findByKey(const Key<RoadEdge>& key) const
{
  return road_edge_.findByKey(key);
}

Gutter VectorMap::findByKey(const Key<Gutter>& key) const
{
  return gutter_.findByKey(key);
}

Curb VectorMap::findByKey(const Key<Curb>& key) const
{
  return curb_.findByKey(key);
}

WhiteLine VectorMap::findByKey(const Key<WhiteLine>& key) const
{
  return white_line_.findByKey(key);
}

StopLine VectorMap::findByKey(const Key<StopLine>& key) const
{
  return stop_line_.findByKey(key);
}

ZebraZone VectorMap::findByKey(const Key<ZebraZone>& key) const
{
  return zebra_zone_.findByKey(key);
}

CrossWalk VectorMap::findByKey(const Key<CrossWalk>& key) const
{
  return cross_walk_.findByKey(key);
}

RoadMark VectorMap::findByKey(const Key<RoadMark>& key) const
{
  return road_mark_.findByKey(key);
}

RoadPole VectorMap::findByKey(const Key<RoadPole>& key) const
{
  return road_pole_.findByKey(key);
}

RoadSign VectorMap::findByKey(const Key<RoadSign>& key) const
{
  return road_sign_.findByKey(key);
}

Signal VectorMap::findByKey(const Key<Signal>& key) const
{
  return signal_.findByKey(key);
}

StreetLight VectorMap::findByKey(const Key<StreetLight>& key) const
{
  return street_light_.findByKey(key);
}

UtilityPole VectorMap::findByKey(const Key<UtilityPole>& key) const
{
  return utility_pole_.findByKey(key);
}

GuardRail VectorMap::findByKey(const Key<GuardRail>& key) const
{
  return guard_rail_.findByKey(key);
}

SideWalk VectorMap::findByKey(const Key<SideWalk>& id) const
{
  return side_walk_.findByKey(id);
}

DriveOnPortion VectorMap::findByKey(const Key<DriveOnPortion>& key) const
{
  return drive_on_portion_.findByKey(key);
}

CrossRoad VectorMap::findByKey(const Key<CrossRoad>& key) const
{
  return cross_road_.findByKey(key);
}

SideStrip VectorMap::findByKey(const Key<SideStrip>& id) const
{
  return side_strip_.findByKey(id);
}

CurveMirror VectorMap::findByKey(const Key<CurveMirror>& key) const
{
  return curve_mirror_.findByKey(key);
}

Wall VectorMap::findByKey(const Key<Wall>& key) const
{
  return wall_.findByKey(key);
}

Fence VectorMap::findByKey(const Key<Fence>& key) const
{
  return fence_.findByKey(key);
}

RailCrossing VectorMap::findByKey(const Key<RailCrossing>& key) const
{
  return rail_crossing_.findByKey(key);
}

std::vector<Point> VectorMap::findByFilter(const Filter<Point>& filter) const
{
  return point_.findByFilter(filter);
}

std::vector<Vector> VectorMap::findByFilter(const Filter<Vector>& filter) const
{
  return vector_.findByFilter(filter);
}

std::vector<Line> VectorMap::findByFilter(const Filter<Line>& filter) const
{
  return line_.findByFilter(filter);
}

std::vector<Area> VectorMap::findByFilter(const Filter<Area>& filter) const
{
  return area_.findByFilter(filter);
}

std::vector<Pole> VectorMap::findByFilter(const Filter<Pole>& filter) const
{
  return pole_.findByFilter(filter);
}

std::vector<Box> VectorMap::findByFilter(const Filter<Box>& filter) const
{
  return box_.findByFilter(filter);
}

std::vector<DTLane> VectorMap::findByFilter(const Filter<DTLane>& filter) const
{
  return dtlane_.findByFilter(filter);
}

std::vector<Node> VectorMap::findByFilter(const Filter<Node>& filter) const
{
  return node_.findByFilter(filter);
}

std::vector<Lane> VectorMap::findByFilter(const Filter<Lane>& filter) const
{
  return lane_.findByFilter(filter);
}

std::vector<WayArea> VectorMap::findByFilter(const Filter<WayArea>& filter) const
{
  return way_area_.findByFilter(filter);
}

std::vector<RoadEdge> VectorMap::findByFilter(const Filter<RoadEdge>& filter) const
{
  return road_edge_.findByFilter(filter);
}

std::vector<Gutter> VectorMap::findByFilter(const Filter<Gutter>& filter) const
{
  return gutter_.findByFilter(filter);
}

std::vector<Curb> VectorMap::findByFilter(const Filter<Curb>& filter) const
{
  return curb_.findByFilter(filter);
}

std::vector<WhiteLine> VectorMap::findByFilter(const Filter<WhiteLine>& filter) const
{
  return white_line_.findByFilter(filter);
}

std::vector<StopLine> VectorMap::findByFilter(const Filter<StopLine>& filter) const
{
  return stop_line_.findByFilter(filter);
}

std::vector<ZebraZone> VectorMap::findByFilter(const Filter<ZebraZone>& filter) const
{
  return zebra_zone_.findByFilter(filter);
}

std::vector<CrossWalk> VectorMap::findByFilter(const Filter<CrossWalk>& filter) const
{
  return cross_walk_.findByFilter(filter);
}

std::vector<RoadMark> VectorMap::findByFilter(const Filter<RoadMark>& filter) const
{
  return road_mark_.findByFilter(filter);
}

std::vector<RoadPole> VectorMap::findByFilter(const Filter<RoadPole>& filter) const
{
  return road_pole_.findByFilter(filter);
}

std::vector<RoadSign> VectorMap::findByFilter(const Filter<RoadSign>& filter) const
{
  return road_sign_.findByFilter(filter);
}

std::vector<Signal> VectorMap::findByFilter(const Filter<Signal>& filter) const
{
  return signal_.findByFilter(filter);
}

std::vector<StreetLight> VectorMap::findByFilter(const Filter<StreetLight>& filter) const
{
  return street_light_.findByFilter(filter);
}

std::vector<UtilityPole> VectorMap::findByFilter(const Filter<UtilityPole>& filter) const
{
  return utility_pole_.findByFilter(filter);
}

std::vector<GuardRail> VectorMap::findByFilter(const Filter<GuardRail>& filter) const
{
  return guard_rail_.findByFilter(filter);
}

std::vector<SideWalk> VectorMap::findByFilter(const Filter<SideWalk>& filter) const
{
  return side_walk_.findByFilter(filter);
}

std::vector<DriveOnPortion> VectorMap::findByFilter(const Filter<DriveOnPortion>& filter) const
{
  return drive_on_portion_.findByFilter(filter);
}

std::vector<CrossRoad> VectorMap::findByFilter(const Filter<CrossRoad>& filter) const
{
  return cross_road_.findByFilter(filter);
}

std::vector<SideStrip> VectorMap::findByFilter(const Filter<SideStrip>& filter) const
{
  return side_strip_.findByFilter(filter);
}

std::vector<CurveMirror> VectorMap::findByFilter(const Filter<CurveMirror>& filter) const
{
  return curve_mirror_.findByFilter(filter);
}

std::vector<Wall> VectorMap::findByFilter(const Filter<Wall>& filter) const
{
  return wall_.findByFilter(filter);
}

std::vector<Fence> VectorMap::findByFilter(const Filter<Fence>& filter) const
{
  return fence_.findByFilter(filter);
}

std::vector<RailCrossing> VectorMap::findByFilter(const Filter<RailCrossing>& filter) const
{
  return rail_crossing_.findByFilter(filter);
}

void VectorMap::registerCallback(const Callback<PointArray>& cb)
{
  point_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<VectorArray>& cb)
{
  vector_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<LineArray>& cb)
{
  line_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<AreaArray>& cb)
{
  area_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<PoleArray>& cb)
{
  pole_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<BoxArray>& cb)
{
  box_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<DTLaneArray>& cb)
{
  dtlane_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<NodeArray>& cb)
{
  node_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<LaneArray>& cb)
{
  lane_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<WayAreaArray>& cb)
{
  way_area_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<RoadEdgeArray>& cb)
{
  road_edge_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<GutterArray>& cb)
{
  gutter_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<CurbArray>& cb)
{
  curb_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<WhiteLineArray>& cb)
{
  white_line_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<StopLineArray>& cb)
{
  stop_line_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<ZebraZoneArray>& cb)
{
  zebra_zone_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<CrossWalkArray>& cb)
{
  cross_walk_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<RoadMarkArray>& cb)
{
  road_mark_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<RoadPoleArray>& cb)
{
  road_pole_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<RoadSignArray>& cb)
{
  road_sign_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<SignalArray>& cb)
{
  signal_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<StreetLightArray>& cb)
{
  street_light_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<UtilityPoleArray>& cb)
{
  utility_pole_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<GuardRailArray>& cb)
{
  guard_rail_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<SideWalkArray>& cb)
{
  side_walk_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<DriveOnPortionArray>& cb)
{
  drive_on_portion_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<CrossRoadArray>& cb)
{
  cross_road_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<SideStripArray>& cb)
{
  side_strip_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<CurveMirrorArray>& cb)
{
  curve_mirror_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<WallArray>& cb)
{
  wall_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<FenceArray>& cb)
{
  fence_.registerCallback(cb);
}

void VectorMap::registerCallback(const Callback<RailCrossingArray>& cb)
{
  rail_crossing_.registerCallback(cb);
}

const double COLOR_VALUE_MIN = 0.0;
const double COLOR_VALUE_MAX = 1.0;
const double COLOR_VALUE_MEDIAN = 0.5;
const double COLOR_VALUE_LIGHT_LOW = 0.56;
const double COLOR_VALUE_LIGHT_HIGH = 0.93;

std_msgs::ColorRGBA createColorRGBA(Color color)
{
  std_msgs::ColorRGBA color_rgba;
  color_rgba.r = COLOR_VALUE_MIN;
  color_rgba.g = COLOR_VALUE_MIN;
  color_rgba.b = COLOR_VALUE_MIN;
  color_rgba.a = COLOR_VALUE_MAX;

  switch (color)
  {
  case BLACK:
    break;
  case GRAY:
    color_rgba.r = COLOR_VALUE_MEDIAN;
    color_rgba.g = COLOR_VALUE_MEDIAN;
    color_rgba.b = COLOR_VALUE_MEDIAN;
    break;
  case LIGHT_RED:
    color_rgba.r = COLOR_VALUE_LIGHT_HIGH;
    color_rgba.g = COLOR_VALUE_LIGHT_LOW;
    color_rgba.b = COLOR_VALUE_LIGHT_LOW;
    break;
  case LIGHT_GREEN:
    color_rgba.r = COLOR_VALUE_LIGHT_LOW;
    color_rgba.g = COLOR_VALUE_LIGHT_HIGH;
    color_rgba.b = COLOR_VALUE_LIGHT_LOW;
    break;
  case LIGHT_BLUE:
    color_rgba.r = COLOR_VALUE_LIGHT_LOW;
    color_rgba.g = COLOR_VALUE_LIGHT_LOW;
    color_rgba.b = COLOR_VALUE_LIGHT_HIGH;
    break;
  case LIGHT_YELLOW:
    color_rgba.r = COLOR_VALUE_LIGHT_HIGH;
    color_rgba.g = COLOR_VALUE_LIGHT_HIGH;
    color_rgba.b = COLOR_VALUE_LIGHT_LOW;
    break;
  case LIGHT_CYAN:
    color_rgba.r = COLOR_VALUE_LIGHT_LOW;
    color_rgba.g = COLOR_VALUE_LIGHT_HIGH;
    color_rgba.b = COLOR_VALUE_LIGHT_HIGH;
    break;
  case LIGHT_MAGENTA:
    color_rgba.r = COLOR_VALUE_LIGHT_HIGH;
    color_rgba.g = COLOR_VALUE_LIGHT_LOW;
    color_rgba.b = COLOR_VALUE_LIGHT_HIGH;
    break;
  case RED:
    color_rgba.r = COLOR_VALUE_MAX;
    break;
  case GREEN:
    color_rgba.g = COLOR_VALUE_MAX;
    break;
  case BLUE:
    color_rgba.b = COLOR_VALUE_MAX;
    break;
  case YELLOW:
    color_rgba.r = COLOR_VALUE_MAX;
    color_rgba.g = COLOR_VALUE_MAX;
    break;
  case CYAN:
    color_rgba.g = COLOR_VALUE_MAX;
    color_rgba.b = COLOR_VALUE_MAX;
    break;
  case MAGENTA:
    color_rgba.r = COLOR_VALUE_MAX;
    color_rgba.b = COLOR_VALUE_MAX;
    break;
  case WHITE:
    color_rgba.r = COLOR_VALUE_MAX;
    color_rgba.g = COLOR_VALUE_MAX;
    color_rgba.b = COLOR_VALUE_MAX;
    break;
  default:
    color_rgba.a = COLOR_VALUE_MIN; // hide color from view
    break;
  }

  return color_rgba;
}

void enableMarker(visualization_msgs::Marker& marker)
{
  marker.action = visualization_msgs::Marker::ADD;
}

void disableMarker(visualization_msgs::Marker& marker)
{
  marker.action = visualization_msgs::Marker::DELETE;
}

bool isValidMarker(const visualization_msgs::Marker& marker)
{
  return marker.action == visualization_msgs::Marker::ADD;
}

extern const double MAKER_SCALE_POINT = 0.08;
extern const double MAKER_SCALE_VECTOR = 0.08;
extern const double MAKER_SCALE_VECTOR_LENGTH = 0.64;
extern const double MAKER_SCALE_LINE = 0.08;
extern const double MAKER_SCALE_AREA = 0.08;
extern const double MAKER_SCALE_BOX = 0.08;

visualization_msgs::Marker createMarker(const std::string& ns, int id, int type)
{
  visualization_msgs::Marker marker;
  // NOTE: Autoware want to use map messages with or without /use_sim_time.
  // Therefore we don't set marker.header.stamp.
  // marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "map";
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.lifetime = ros::Duration();
  marker.frame_locked = true;
  disableMarker(marker);
  return marker;
}

visualization_msgs::Marker createPointMarker(const std::string& ns, int id, Color color, const Point& point)
{
  visualization_msgs::Marker marker = createMarker(ns, id, visualization_msgs::Marker::SPHERE);
  if (point.pid == 0)
    return marker;

  marker.pose.position = convertPointToGeomPoint(point);
  marker.pose.orientation = convertVectorToGeomQuaternion(Vector());
  marker.scale.x = MAKER_SCALE_POINT;
  marker.scale.y = MAKER_SCALE_POINT;
  marker.scale.z = MAKER_SCALE_POINT;
  marker.color = createColorRGBA(color);
  if (marker.color.a == COLOR_VALUE_MIN)
    return marker;

  enableMarker(marker);
  return marker;
}

visualization_msgs::Marker createVectorMarker(const std::string& ns, int id, Color color, const VectorMap& vmap,
                                              const Vector& vector)
{
  visualization_msgs::Marker marker = createMarker(ns, id, visualization_msgs::Marker::ARROW);
  if (vector.vid == 0)
    return marker;

  Point point = vmap.findByKey(Key<Point>(vector.pid));
  if (point.pid == 0)
    return marker;

  marker.pose.position = convertPointToGeomPoint(point);
  marker.pose.orientation = convertVectorToGeomQuaternion(vector);
  marker.scale.x = MAKER_SCALE_VECTOR_LENGTH;
  marker.scale.y = MAKER_SCALE_VECTOR;
  marker.scale.z = MAKER_SCALE_VECTOR;
  marker.color = createColorRGBA(color);
  if (marker.color.a == COLOR_VALUE_MIN)
    return marker;

  enableMarker(marker);
  return marker;
}

visualization_msgs::Marker createLineMarker(const std::string& ns, int id, Color color, const VectorMap& vmap,
                                            const Line& line)
{
  visualization_msgs::Marker marker = createMarker(ns, id, visualization_msgs::Marker::LINE_STRIP);
  if (line.lid == 0)
    return marker;

  Point bp = vmap.findByKey(Key<Point>(line.bpid));
  if (bp.pid == 0)
    return marker;

  Point fp = vmap.findByKey(Key<Point>(line.fpid));
  if (fp.pid == 0)
    return marker;

  marker.points.push_back(convertPointToGeomPoint(bp));
  marker.points.push_back(convertPointToGeomPoint(fp));

  marker.scale.x = MAKER_SCALE_LINE;
  marker.color = createColorRGBA(color);
  if (marker.color.a == COLOR_VALUE_MIN)
    return marker;

  enableMarker(marker);
  return marker;
}

visualization_msgs::Marker createAreaMarker(const std::string& ns, int id, Color color, const VectorMap& vmap,
                                            const Area& area)
{
  visualization_msgs::Marker marker = createMarker(ns, id, visualization_msgs::Marker::LINE_STRIP);
  if (area.aid == 0)
    return marker;

  Line line = vmap.findByKey(Key<Line>(area.slid));
  if (line.lid == 0)
    return marker;
  if (line.blid != 0) // must set beginning line
    return marker;

  while (line.flid != 0)
  {
    Point bp = vmap.findByKey(Key<Point>(line.bpid));
    if (bp.pid == 0)
      return marker;

    Point fp = vmap.findByKey(Key<Point>(line.fpid));
    if (fp.pid == 0)
      return marker;

    marker.points.push_back(convertPointToGeomPoint(bp));
    marker.points.push_back(convertPointToGeomPoint(fp));

    line = vmap.findByKey(Key<Line>(line.flid));
    if (line.lid == 0)
      return marker;
  }

  Point bp = vmap.findByKey(Key<Point>(line.bpid));
  if (bp.pid == 0)
    return marker;

  Point fp = vmap.findByKey(Key<Point>(line.fpid));
  if (fp.pid == 0)
    return marker;

  marker.points.push_back(convertPointToGeomPoint(bp));
  marker.points.push_back(convertPointToGeomPoint(fp));

  marker.scale.x = MAKER_SCALE_AREA;
  marker.color = createColorRGBA(color);
  if (marker.color.a == COLOR_VALUE_MIN)
    return marker;

  enableMarker(marker);
  return marker;
}

visualization_msgs::Marker createPoleMarker(const std::string& ns, int id, Color color, const VectorMap& vmap,
                                            const Pole& pole)
{
  visualization_msgs::Marker marker = createMarker(ns, id, visualization_msgs::Marker::CYLINDER);
  if (pole.plid == 0)
    return marker;
  // XXX: The following conditions are workaround for pole.csv of Nagoya University's campus.
  if (pole.length == 0 || pole.dim == 0)
    return marker;

  Vector vector = vmap.findByKey(Key<Vector>(pole.vid));
  if (vector.vid == 0)
    return marker;
  // XXX: The visualization_msgs::Marker::CYLINDER is difficult to display other than vertical pole.
  if (vector.vang != 0)
    return marker;

  Point point = vmap.findByKey(Key<Point>(vector.pid));
  if (point.pid == 0)
    return marker;

  geometry_msgs::Point geom_point = convertPointToGeomPoint(point);
  geom_point.z += pole.length / 2;
  marker.pose.position = geom_point;
  vector.vang -= 90;
  marker.pose.orientation = convertVectorToGeomQuaternion(vector);
  marker.scale.x = pole.dim;
  marker.scale.y = pole.dim;
  marker.scale.z = pole.length;
  marker.color = createColorRGBA(color);
  if (marker.color.a == COLOR_VALUE_MIN)
    return marker;

  enableMarker(marker);
  return marker;
}

visualization_msgs::Marker createBoxMarker(const std::string& ns, int id, Color color, const VectorMap& vmap,
                                           const Box& box)
{
  visualization_msgs::Marker marker = createMarker(ns, id, visualization_msgs::Marker::LINE_STRIP);
  if (box.bid == 0)
    return marker;

  Point p1 = vmap.findByKey(Key<Point>(box.pid1));
  if (p1.pid == 0)
    return marker;

  Point p2 = vmap.findByKey(Key<Point>(box.pid2));
  if (p2.pid == 0)
    return marker;

  Point p3 = vmap.findByKey(Key<Point>(box.pid3));
  if (p3.pid == 0)
    return marker;

  Point p4 = vmap.findByKey(Key<Point>(box.pid4));
  if (p4.pid == 0)
    return marker;

  std::vector<geometry_msgs::Point> bottom_points(4);
  bottom_points[0] = convertPointToGeomPoint(p1);
  bottom_points[1] = convertPointToGeomPoint(p2);
  bottom_points[2] = convertPointToGeomPoint(p3);
  bottom_points[3] = convertPointToGeomPoint(p4);

  std::vector<geometry_msgs::Point> top_points(4);
  for (size_t i = 0; i < 4; ++i)
  {
    top_points[i] = bottom_points[i];
    top_points[i].z += box.height;
  }

  for (size_t i = 0; i < 4; ++i)
  {
    marker.points.push_back(bottom_points[i]);
    marker.points.push_back(top_points[i]);
    marker.points.push_back(top_points[i]);
    if (i != 3)
    {
      marker.points.push_back(top_points[i + 1]);
      marker.points.push_back(top_points[i + 1]);
      marker.points.push_back(bottom_points[i + 1]);
    }
    else
    {
      marker.points.push_back(top_points[0]);
      marker.points.push_back(top_points[0]);
      marker.points.push_back(bottom_points[0]);
    }
  }
  for (size_t i = 0; i < 4; ++i)
  {
    marker.points.push_back(bottom_points[i]);
    if (i != 3)
      marker.points.push_back(bottom_points[i + 1]);
    else
      marker.points.push_back(bottom_points[0]);
  }

  marker.scale.x = MAKER_SCALE_BOX;
  marker.color = createColorRGBA(color);
  if (marker.color.a == COLOR_VALUE_MIN)
    return marker;

  enableMarker(marker);
  return marker;
}

double convertDegreeToRadian(double degree)
{
  return degree * M_PI / 180;
}

double convertRadianToDegree(double radian)
{
  return radian * 180 / M_PI;
}

geometry_msgs::Point convertPointToGeomPoint(const Point& point)
{
  geometry_msgs::Point geom_point;
  // NOTE: Autwoare use Japan Plane Rectangular Coordinate System.
  // Therefore we swap x and y axis.
  geom_point.x = point.ly;
  geom_point.y = point.bx;
  geom_point.z = point.h;
  return geom_point;
}

Point convertGeomPointToPoint(const geometry_msgs::Point& geom_point)
{
  Point point;
  // NOTE: Autwoare use Japan Plane Rectangular Coordinate System.
  // Therefore we swap x and y axis.
  point.bx = geom_point.y;
  point.ly = geom_point.x;
  point.h = geom_point.z;
  return point;
}

geometry_msgs::Quaternion convertVectorToGeomQuaternion(const Vector& vector)
{
  double pitch = convertDegreeToRadian(vector.vang - 90); // convert vertical angle to pitch
  double yaw = convertDegreeToRadian(-vector.hang + 90); // convert horizontal angle to yaw
  return tf::createQuaternionMsgFromRollPitchYaw(0, pitch, yaw);
}

Vector convertGeomQuaternionToVector(const geometry_msgs::Quaternion& geom_quaternion)
{
  tf::Quaternion quaternion(geom_quaternion.x, geom_quaternion.y, geom_quaternion.z, geom_quaternion.w);
  double roll, pitch, yaw;
  tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  Vector vector;
  vector.vang = convertRadianToDegree(pitch) + 90;
  vector.hang = -convertRadianToDegree(yaw) + 90;
  return vector;
}
} // namespace vector_map

std::ostream& operator<<(std::ostream& os, const vector_map::Point& obj)
{
  os << obj.pid << ","
     << obj.b << ","
     << obj.l << ","
     << obj.h << ","
     << obj.bx << ","
     << obj.ly << ","
     << obj.ref << ","
     << obj.mcode1 << ","
     << obj.mcode2 << ","
     << obj.mcode3;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::Vector& obj)
{
  os << obj.vid << ","
     << obj.pid << ","
     << obj.hang << ","
     << obj.vang;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::Line& obj)
{
  os << obj.lid << ","
     << obj.bpid << ","
     << obj.fpid << ","
     << obj.blid << ","
     << obj.flid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::Area& obj)
{
  os << obj.aid << ","
     << obj.slid << ","
     << obj.elid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::Pole& obj)
{
  os << obj.plid << ","
     << obj.vid << ","
     << obj.length << ","
     << obj.dim;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::Box& obj)
{
  os << obj.bid << ","
     << obj.pid1 << ","
     << obj.pid2 << ","
     << obj.pid3 << ","
     << obj.pid4 << ","
     << obj.height;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::DTLane& obj)
{
  os << obj.did << ","
     << obj.dist << ","
     << obj.pid << ","
     << obj.dir << ","
     << obj.apara << ","
     << obj.r << ","
     << obj.slope << ","
     << obj.cant << ","
     << obj.lw << ","
     << obj.rw;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::Node& obj)
{
  os << obj.nid << ","
     << obj.pid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::Lane& obj)
{
  os << obj.lnid << ","
     << obj.did << ","
     << obj.blid << ","
     << obj.flid << ","
     << obj.bnid << ","
     << obj.fnid << ","
     << obj.jct << ","
     << obj.blid2 << ","
     << obj.blid3 << ","
     << obj.blid4 << ","
     << obj.flid2 << ","
     << obj.flid3 << ","
     << obj.flid4 << ","
     << obj.clossid << ","
     << obj.span << ","
     << obj.lcnt << ","
     << obj.lno << ","
     << obj.lanetype << ","
     << obj.limitvel << ","
     << obj.refvel << ","
     << obj.roadsecid << ","
     << obj.lanecfgfg << ","
     << obj.linkwaid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::WayArea& obj)
{
  os << obj.waid << ","
     << obj.aid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::RoadEdge& obj)
{
  os << obj.id << ","
     << obj.lid << ","
     << obj.linkid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::Gutter& obj)
{
  os << obj.id << ","
     << obj.aid << ","
     << obj.type << ","
     << obj.linkid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::Curb& obj)
{
  os << obj.id << ","
     << obj.lid << ","
     << obj.height << ","
     << obj.width << ","
     << obj.dir << ","
     << obj.linkid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::WhiteLine& obj)
{
  os << obj.id << ","
     << obj.lid << ","
     << obj.width << ","
     << obj.color << ","
     << obj.type << ","
     << obj.linkid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::StopLine& obj)
{
  os << obj.id << ","
     << obj.lid << ","
     << obj.tlid << ","
     << obj.signid << ","
     << obj.linkid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::ZebraZone& obj)
{
  os << obj.id << ","
     << obj.aid << ","
     << obj.linkid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::CrossWalk& obj)
{
  os << obj.id << ","
     << obj.aid << ","
     << obj.type << ","
     << obj.bdid << ","
     << obj.linkid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::RoadMark& obj)
{
  os << obj.id << ","
     << obj.aid << ","
     << obj.type << ","
     << obj.linkid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::RoadPole& obj)
{
  os << obj.id << ","
     << obj.plid << ","
     << obj.linkid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::RoadSign& obj)
{
  os << obj.id << ","
     << obj.vid << ","
     << obj.plid << ","
     << obj.type << ","
     << obj.linkid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::Signal& obj)
{
  os << obj.id << ","
     << obj.vid << ","
     << obj.plid << ","
     << obj.type << ","
     << obj.linkid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::StreetLight& obj)
{
  os << obj.id << ","
     << obj.lid << ","
     << obj.plid << ","
     << obj.linkid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::UtilityPole& obj)
{
  os << obj.id << ","
     << obj.plid << ","
     << obj.linkid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::GuardRail& obj)
{
  os << obj.id << ","
     << obj.aid << ","
     << obj.type << ","
     << obj.linkid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::SideWalk& obj)
{
  os << obj.id << ","
     << obj.aid << ","
     << obj.linkid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::DriveOnPortion& obj)
{
  os << obj.id << ","
     << obj.aid << ","
     << obj.linkid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::CrossRoad& obj)
{
  os << obj.id << ","
     << obj.aid << ","
     << obj.linkid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::SideStrip& obj)
{
  os << obj.id << ","
     << obj.lid << ","
     << obj.linkid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::CurveMirror& obj)
{
  os << obj.id << ","
     << obj.vid << ","
     << obj.plid << ","
     << obj.type << ","
     << obj.linkid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::Wall& obj)
{
  os << obj.id << ","
     << obj.aid << ","
     << obj.linkid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::Fence& obj)
{
  os << obj.id << ","
     << obj.aid << ","
     << obj.linkid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const vector_map::RailCrossing& obj)
{
  os << obj.id << ","
     << obj.aid << ","
     << obj.linkid;
  return os;
}

std::istream& operator>>(std::istream& is, vector_map::Point& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.pid = std::stoi(columns[0]);
  obj.b = std::stod(columns[1]);
  obj.l = std::stod(columns[2]);
  obj.h = std::stod(columns[3]);
  obj.bx = std::stod(columns[4]);
  obj.ly = std::stod(columns[5]);
  obj.ref = std::stoi(columns[6]);
  obj.mcode1 = std::stoi(columns[7]);
  obj.mcode2 = std::stoi(columns[8]);
  obj.mcode3 = std::stoi(columns[9]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::Vector& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.vid = std::stoi(columns[0]);
  obj.pid = std::stoi(columns[1]);
  obj.hang = std::stod(columns[2]);
  obj.vang = std::stod(columns[3]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::Line& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.lid = std::stoi(columns[0]);
  obj.bpid = std::stoi(columns[1]);
  obj.fpid = std::stoi(columns[2]);
  obj.blid = std::stoi(columns[3]);
  obj.flid = std::stoi(columns[4]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::Area& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.aid = std::stoi(columns[0]);
  obj.slid = std::stoi(columns[1]);
  obj.elid = std::stoi(columns[2]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::Pole& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.plid = std::stoi(columns[0]);
  obj.vid = std::stoi(columns[1]);
  obj.length = std::stod(columns[2]);
  obj.dim = std::stod(columns[3]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::Box& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.bid = std::stoi(columns[0]);
  obj.pid1 = std::stoi(columns[1]);
  obj.pid2 = std::stoi(columns[2]);
  obj.pid3 = std::stoi(columns[3]);
  obj.pid4 = std::stoi(columns[4]);
  obj.height = std::stod(columns[5]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::DTLane& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.did = std::stoi(columns[0]);
  obj.dist = std::stod(columns[1]);
  obj.pid = std::stoi(columns[2]);
  obj.dir = std::stod(columns[3]);
  obj.apara = std::stod(columns[4]);
  obj.r = std::stod(columns[5]);
  obj.slope = std::stod(columns[6]);
  obj.cant = std::stod(columns[7]);
  obj.lw = std::stod(columns[8]);
  obj.rw = std::stod(columns[9]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::Node& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.nid = std::stoi(columns[0]);
  obj.pid = std::stoi(columns[1]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::Lane& obj)
{
  std::vector<std::string> columns;
  std::string column;
  int n = 0;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
    ++n;
  }
  obj.lnid = std::stoi(columns[0]);
  obj.did = std::stoi(columns[1]);
  obj.blid = std::stoi(columns[2]);
  obj.flid = std::stoi(columns[3]);
  obj.bnid = std::stoi(columns[4]);
  obj.fnid = std::stoi(columns[5]);
  obj.jct = std::stoi(columns[6]);
  obj.blid2 = std::stoi(columns[7]);
  obj.blid3 = std::stoi(columns[8]);
  obj.blid4 = std::stoi(columns[9]);
  obj.flid2 = std::stoi(columns[10]);
  obj.flid3 = std::stoi(columns[11]);
  obj.flid4 = std::stoi(columns[12]);
  obj.clossid = std::stoi(columns[13]);
  obj.span = std::stod(columns[14]);
  obj.lcnt = std::stoi(columns[15]);
  obj.lno = std::stoi(columns[16]);
  if (n == 17)
  {
    obj.lanetype = 0;
    obj.limitvel = 0;
    obj.refvel = 0;
    obj.roadsecid = 0;
    obj.lanecfgfg = 0;
    obj.linkwaid = 0;
    return is;
  }
  obj.lanetype = std::stoi(columns[17]);
  obj.limitvel = std::stoi(columns[18]);
  obj.refvel = std::stoi(columns[19]);
  obj.roadsecid = std::stoi(columns[20]);
  obj.lanecfgfg = std::stoi(columns[21]);
  if (n == 22)
  {
    obj.linkwaid = 0;
    return is;
  }
  obj.linkwaid = std::stoi(columns[22]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::WayArea& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.waid = std::stoi(columns[0]);
  obj.aid = std::stoi(columns[1]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::RoadEdge& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.id = std::stoi(columns[0]);
  obj.lid = std::stoi(columns[1]);
  obj.linkid = std::stoi(columns[2]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::Gutter& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.id = std::stoi(columns[0]);
  obj.aid = std::stoi(columns[1]);
  obj.type = std::stoi(columns[2]);
  obj.linkid = std::stoi(columns[3]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::Curb& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.id = std::stoi(columns[0]);
  obj.lid = std::stoi(columns[1]);
  obj.height = std::stod(columns[2]);
  obj.width = std::stod(columns[3]);
  obj.dir = std::stoi(columns[4]);
  obj.linkid = std::stoi(columns[5]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::WhiteLine& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.id = std::stoi(columns[0]);
  obj.lid = std::stoi(columns[1]);
  obj.width = std::stod(columns[2]);
  obj.color = columns[3].c_str()[0];
  obj.type = std::stoi(columns[4]);
  obj.linkid = std::stoi(columns[5]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::StopLine& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.id = std::stoi(columns[0]);
  obj.lid = std::stoi(columns[1]);
  obj.tlid = std::stoi(columns[2]);
  obj.signid = std::stoi(columns[3]);
  obj.linkid = std::stoi(columns[4]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::ZebraZone& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.id = std::stoi(columns[0]);
  obj.aid = std::stoi(columns[1]);
  obj.linkid = std::stoi(columns[2]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::CrossWalk& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.id = std::stoi(columns[0]);
  obj.aid = std::stoi(columns[1]);
  obj.type = std::stoi(columns[2]);
  obj.bdid = std::stoi(columns[3]);
  obj.linkid = std::stoi(columns[4]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::RoadMark& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.id = std::stoi(columns[0]);
  obj.aid = std::stoi(columns[1]);
  obj.type = std::stoi(columns[2]);
  obj.linkid = std::stoi(columns[3]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::RoadPole& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.id = std::stoi(columns[0]);
  obj.plid = std::stoi(columns[1]);
  obj.linkid = std::stoi(columns[2]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::RoadSign& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.id = std::stoi(columns[0]);
  obj.vid = std::stoi(columns[1]);
  obj.plid = std::stoi(columns[2]);
  obj.type = std::stoi(columns[3]);
  obj.linkid = std::stoi(columns[4]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::Signal& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.id = std::stoi(columns[0]);
  obj.vid = std::stoi(columns[1]);
  obj.plid = std::stoi(columns[2]);
  obj.type = std::stoi(columns[3]);
  obj.linkid = std::stoi(columns[4]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::StreetLight& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.id = std::stoi(columns[0]);
  obj.lid = std::stoi(columns[1]);
  obj.plid = std::stoi(columns[2]);
  obj.linkid = std::stoi(columns[3]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::UtilityPole& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.id = std::stoi(columns[0]);
  obj.plid = std::stoi(columns[1]);
  obj.linkid = std::stoi(columns[2]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::GuardRail& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.id = std::stoi(columns[0]);
  obj.aid = std::stoi(columns[1]);
  obj.type = std::stoi(columns[2]);
  obj.linkid = std::stoi(columns[3]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::SideWalk& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.id = std::stoi(columns[0]);
  obj.aid = std::stoi(columns[1]);
  obj.linkid = std::stoi(columns[2]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::DriveOnPortion& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.id = std::stoi(columns[0]);
  obj.aid = std::stoi(columns[1]);
  obj.linkid = std::stoi(columns[2]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::CrossRoad& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.id = std::stoi(columns[0]);
  obj.aid = std::stoi(columns[1]);
  obj.linkid = std::stoi(columns[2]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::SideStrip& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.id = std::stoi(columns[0]);
  obj.lid = std::stoi(columns[1]);
  obj.linkid = std::stoi(columns[2]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::CurveMirror& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.id = std::stoi(columns[0]);
  obj.vid = std::stoi(columns[1]);
  obj.plid = std::stoi(columns[2]);
  obj.type = std::stoi(columns[3]);
  obj.linkid = std::stoi(columns[4]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::Wall& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.id = std::stoi(columns[0]);
  obj.aid = std::stoi(columns[1]);
  obj.linkid = std::stoi(columns[2]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::Fence& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.id = std::stoi(columns[0]);
  obj.aid = std::stoi(columns[1]);
  obj.linkid = std::stoi(columns[2]);
  return is;
}

std::istream& operator>>(std::istream& is, vector_map::RailCrossing& obj)
{
  std::vector<std::string> columns;
  std::string column;
  while (std::getline(is, column, ','))
  {
    columns.push_back(column);
  }
  obj.id = std::stoi(columns[0]);
  obj.aid = std::stoi(columns[1]);
  obj.linkid = std::stoi(columns[2]);
  return is;
}
