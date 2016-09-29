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

#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector_map/vector_map.h>
#include <map_file/get_file.h>
#include <sys/stat.h>

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

using vector_map::isValidMarker;
using vector_map::createVectorMarker;
using vector_map::createAreaMarker;
using vector_map::createPoleMarker;

namespace
{
void printUsage()
{
  ROS_ERROR_STREAM("Usage:");
  ROS_ERROR_STREAM("rosrun map_file vector_map_loader [CSV]...");
  ROS_ERROR_STREAM("rosrun map_file vector_map_loader download [X] [Y]");
}

bool isDownloaded(const std::string& local_path)
{
  struct stat st;
  return stat(local_path.c_str(), &st) == 0;
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
    case Signal::OTHER:
      vector_marker = createVectorMarker("signal", id++, other_color, vmap, vector);
      break;
    default:
      ROS_ERROR_STREAM("[createSignalMarkerArray] unknown signal.type: " << signal);
      continue;
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

void insertMarkerArray(visualization_msgs::MarkerArray& a1, const visualization_msgs::MarkerArray& a2)
{
  a1.markers.insert(a1.markers.end(), a2.markers.begin(), a2.markers.end());
}
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vector_map_loader");
  ros::NodeHandle nh;

  if (argc < 2)
  {
    printUsage();
    return EXIT_FAILURE;
  }

  std::string mode(argv[1]);
  if (mode == "download" && argc < 4)
  {
    printUsage();
    return EXIT_FAILURE;
  }

  ros::Publisher point_pub = nh.advertise<PointArray>("vector_map_info/point", 1, true);
  ros::Publisher vector_pub = nh.advertise<VectorArray>("vector_map_info/vector", 1, true);
  ros::Publisher line_pub = nh.advertise<LineArray>("vector_map_info/line", 1, true);
  ros::Publisher area_pub = nh.advertise<AreaArray>("vector_map_info/area", 1, true);
  ros::Publisher pole_pub = nh.advertise<PoleArray>("vector_map_info/pole", 1, true);
  ros::Publisher box_pub = nh.advertise<BoxArray>("vector_map_info/box", 1, true);
  ros::Publisher dtlane_pub = nh.advertise<DTLaneArray>("vector_map_info/dtlane", 1, true);
  ros::Publisher node_pub = nh.advertise<NodeArray>("vector_map_info/node", 1, true);
  ros::Publisher lane_pub = nh.advertise<LaneArray>("vector_map_info/lane", 1, true);
  ros::Publisher way_area_pub = nh.advertise<WayAreaArray>("vector_map_info/way_area", 1, true);
  ros::Publisher road_edge_pub = nh.advertise<RoadEdgeArray>("vector_map_info/road_edge", 1, true);
  ros::Publisher gutter_pub = nh.advertise<GutterArray>("vector_map_info/gutter", 1, true);
  ros::Publisher curb_pub = nh.advertise<CurbArray>("vector_map_info/curb", 1, true);
  ros::Publisher white_line_pub = nh.advertise<WhiteLineArray>("vector_map_info/white_line", 1, true);
  ros::Publisher stop_line_pub = nh.advertise<StopLineArray>("vector_map_info/stop_line", 1, true);
  ros::Publisher zebra_zone_pub = nh.advertise<ZebraZoneArray>("vector_map_info/zebra_zone", 1, true);
  ros::Publisher cross_walk_pub = nh.advertise<CrossWalkArray>("vector_map_info/cross_walk", 1, true);
  ros::Publisher road_mark_pub = nh.advertise<RoadMarkArray>("vector_map_info/road_mark", 1, true);
  ros::Publisher road_pole_pub = nh.advertise<RoadPoleArray>("vector_map_info/road_pole", 1, true);
  ros::Publisher road_sign_pub = nh.advertise<RoadSignArray>("vector_map_info/road_sign", 1, true);
  ros::Publisher signal_pub = nh.advertise<SignalArray>("vector_map_info/signal", 1, true);
  ros::Publisher street_light_pub = nh.advertise<StreetLightArray>("vector_map_info/street_light", 1, true);
  ros::Publisher utility_pole_pub = nh.advertise<UtilityPoleArray>("vector_map_info/utility_pole", 1, true);
  ros::Publisher guard_rail_pub = nh.advertise<GuardRailArray>("vector_map_info/guard_rail", 1, true);
  ros::Publisher side_walk_pub = nh.advertise<SideWalkArray>("vector_map_info/side_walk", 1, true);
  ros::Publisher drive_on_portion_pub = nh.advertise<DriveOnPortionArray>("vector_map_info/drive_on_portion", 1, true);
  ros::Publisher cross_road_pub = nh.advertise<CrossRoadArray>("vector_map_info/cross_road", 1, true);
  ros::Publisher side_strip_pub = nh.advertise<SideStripArray>("vector_map_info/side_strip", 1, true);
  ros::Publisher curve_mirror_pub = nh.advertise<CurveMirrorArray>("vector_map_info/curve_mirror", 1, true);
  ros::Publisher wall_pub = nh.advertise<WallArray>("vector_map_info/wall", 1, true);
  ros::Publisher fence_pub = nh.advertise<FenceArray>("vector_map_info/fence", 1, true);
  ros::Publisher rail_crossing_pub = nh.advertise<RailCrossingArray>("vector_map_info/rail_crossing", 1, true);

  ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("vector_map", 1, true);
  ros::Publisher stat_pub = nh.advertise<std_msgs::Bool>("vmap_stat", 1, true);

  std_msgs::Bool stat;
  stat.data = false;
  stat_pub.publish(stat);

  std::vector<std::string> file_paths;
  if (mode == "download")
  {
    std::string host_name;
    nh.param<std::string>("vector_map_loader/host_name", host_name, HTTP_HOSTNAME);
    int port;
    nh.param<int>("vector_map_loader/port", port, HTTP_PORT);
    std::string user;
    nh.param<std::string>("vector_map_loader/user", user, HTTP_USER);
    std::string password;
    nh.param<std::string>("vector_map_loader/password", password, HTTP_PASSWORD);
    GetFile gf = GetFile(host_name, port, user, password);

    std::string remote_path = "/data/map";
    int x = std::atoi(argv[2]);
    int y = std::atoi(argv[3]);
    x -= x % 1000 + 1000;
    y -= y % 1000 + 1000;
    remote_path += std::to_string(x) + "/" + std::to_string(y) + "/vector";

    std::string local_path = "/tmp" + remote_path;
    if (!isDownloaded(local_path))
    {
      std::istringstream iss(remote_path);
      std::string column;
      std::getline(iss, column, '/');
      std::string mkdir_path = "/tmp";
      while (std::getline(iss, column, '/'))
      {
        mkdir_path += "/" + column;
        mkdir(mkdir_path.c_str(), 0755);
      }
    }

    std::vector<std::string> file_names
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
    for (const auto& file_name : file_names)
    {
      if (gf.GetHTTPFile(remote_path + "/" + file_name) == 0)
        file_paths.push_back("/tmp" + remote_path + "/" + file_name);
      else
        ROS_ERROR_STREAM("download failure: " << remote_path + "/" + file_name);
    }
  }
  else
  {
    for (int i = 1; i < argc; ++i)
    {
      std::string file_path(argv[i]);
      file_paths.push_back(file_path);
    }
  }

  vector_map::category_t category = Category::NONE;
  for (const auto& file_path : file_paths)
  {
    std::string file_name(basename(file_path.c_str()));
    if (file_name == "idx.csv")
    {
      ; // XXX: This version of Autoware don't support index csv file now.
    }
    else if (file_name == "point.csv")
    {
      point_pub.publish(createObjectArray<Point, PointArray>(file_path));
      category |= Category::POINT;
    }
    else if (file_name == "vector.csv")
    {
      vector_pub.publish(createObjectArray<Vector, VectorArray>(file_path));
      category |= Category::VECTOR;
    }
    else if (file_name == "line.csv")
    {
      line_pub.publish(createObjectArray<Line, LineArray>(file_path));
      category |= Category::LINE;
    }
    else if (file_name == "area.csv")
    {
      area_pub.publish(createObjectArray<Area, AreaArray>(file_path));
      category |= Category::AREA;
    }
    else if (file_name == "pole.csv")
    {
      pole_pub.publish(createObjectArray<Pole, PoleArray>(file_path));
      category |= Category::POLE;
    }
    else if (file_name == "box.csv")
    {
      box_pub.publish(createObjectArray<Box, BoxArray>(file_path));
      category |= Category::BOX;
    }
    else if (file_name == "dtlane.csv")
    {
      dtlane_pub.publish(createObjectArray<DTLane, DTLaneArray>(file_path));
      category |= Category::DTLANE;
    }
    else if (file_name == "node.csv")
    {
      node_pub.publish(createObjectArray<Node, NodeArray>(file_path));
      category |= Category::NODE;
    }
    else if (file_name == "lane.csv")
    {
      lane_pub.publish(createObjectArray<Lane, LaneArray>(file_path));
      category |= Category::LANE;
    }
    else if (file_name == "wayarea.csv")
    {
      way_area_pub.publish(createObjectArray<WayArea, WayAreaArray>(file_path));
      category |= Category::WAY_AREA;
    }
    else if (file_name == "roadedge.csv")
    {
      road_edge_pub.publish(createObjectArray<RoadEdge, RoadEdgeArray>(file_path));
      category |= Category::ROAD_EDGE;
    }
    else if (file_name == "gutter.csv")
    {
      gutter_pub.publish(createObjectArray<Gutter, GutterArray>(file_path));
      category |= Category::GUTTER;
    }
    else if (file_name == "curb.csv")
    {
      curb_pub.publish(createObjectArray<Curb, CurbArray>(file_path));
      category |= Category::CURB;
    }
    else if (file_name == "whiteline.csv")
    {
      white_line_pub.publish(createObjectArray<WhiteLine, WhiteLineArray>(file_path));
      category |= Category::WHITE_LINE;
    }
    else if (file_name == "stopline.csv")
    {
      stop_line_pub.publish(createObjectArray<StopLine, StopLineArray>(file_path));
      category |= Category::STOP_LINE;
    }
    else if (file_name == "zebrazone.csv")
    {
      zebra_zone_pub.publish(createObjectArray<ZebraZone, ZebraZoneArray>(file_path));
      category |= Category::ZEBRA_ZONE;
    }
    else if (file_name == "crosswalk.csv")
    {
      cross_walk_pub.publish(createObjectArray<CrossWalk, CrossWalkArray>(file_path));
      category |= Category::CROSS_WALK;
    }
    else if (file_name == "road_surface_mark.csv")
    {
      road_mark_pub.publish(createObjectArray<RoadMark, RoadMarkArray>(file_path));
      category |= Category::ROAD_MARK;
    }
    else if (file_name == "poledata.csv")
    {
      road_pole_pub.publish(createObjectArray<RoadPole, RoadPoleArray>(file_path));
      category |= Category::ROAD_POLE;
    }
    else if (file_name == "roadsign.csv")
    {
      road_sign_pub.publish(createObjectArray<RoadSign, RoadSignArray>(file_path));
      category |= Category::ROAD_SIGN;
    }
    else if (file_name == "signaldata.csv")
    {
      signal_pub.publish(createObjectArray<Signal, SignalArray>(file_path));
      category |= Category::SIGNAL;
    }
    else if (file_name == "streetlight.csv")
    {
      street_light_pub.publish(createObjectArray<StreetLight, StreetLightArray>(file_path));
      category |= Category::STREET_LIGHT;
    }
    else if (file_name == "utilitypole.csv")
    {
      utility_pole_pub.publish(createObjectArray<UtilityPole, UtilityPoleArray>(file_path));
      category |= Category::UTILITY_POLE;
    }
    else if (file_name == "guardrail.csv")
    {
      guard_rail_pub.publish(createObjectArray<GuardRail, GuardRailArray>(file_path));
      category |= Category::GUARD_RAIL;
    }
    else if (file_name == "sidewalk.csv")
    {
      side_walk_pub.publish(createObjectArray<SideWalk, SideWalkArray>(file_path));
      category |= Category::SIDE_WALK;
    }
    else if (file_name == "driveon_portion.csv")
    {
      drive_on_portion_pub.publish(createObjectArray<DriveOnPortion, DriveOnPortionArray>(file_path));
      category |= Category::DRIVE_ON_PORTION;
    }
    else if (file_name == "intersection.csv")
    {
      cross_road_pub.publish(createObjectArray<CrossRoad, CrossRoadArray>(file_path));
      category |= Category::CROSS_ROAD;
    }
    else if (file_name == "sidestrip.csv")
    {
      side_strip_pub.publish(createObjectArray<SideStrip, SideStripArray>(file_path));
      category |= Category::SIDE_STRIP;
    }
    else if (file_name == "curvemirror.csv")
    {
      curve_mirror_pub.publish(createObjectArray<CurveMirror, CurveMirrorArray>(file_path));
      category |= Category::CURVE_MIRROR;
    }
    else if (file_name == "wall.csv")
    {
      wall_pub.publish(createObjectArray<Wall, WallArray>(file_path));
      category |= Category::WALL;
    }
    else if (file_name == "fence.csv")
    {
      fence_pub.publish(createObjectArray<Fence, FenceArray>(file_path));
      category |= Category::FENCE;
    }
    else if (file_name == "railroad_crossing.csv")
    {
      rail_crossing_pub.publish(createObjectArray<RailCrossing, RailCrossingArray>(file_path));
      category |= Category::RAIL_CROSSING;
    }
    else
      ROS_ERROR_STREAM("unknown csv file: " << file_path);
  }

  VectorMap vmap;
  vmap.subscribe(nh, category);

  visualization_msgs::MarkerArray marker_array;
  insertMarkerArray(marker_array, createRoadEdgeMarkerArray(vmap, Color::GRAY));
  insertMarkerArray(marker_array, createGutterMarkerArray(vmap, Color::GRAY, Color::GRAY, Color::GRAY));
  insertMarkerArray(marker_array, createCurbMarkerArray(vmap, Color::GRAY));
  insertMarkerArray(marker_array, createWhiteLineMarkerArray(vmap, Color::WHITE, Color::YELLOW));
  insertMarkerArray(marker_array, createStopLineMarkerArray(vmap, Color::WHITE));
  insertMarkerArray(marker_array, createZebraZoneMarkerArray(vmap, Color::WHITE));
  insertMarkerArray(marker_array, createCrossWalkMarkerArray(vmap, Color::WHITE));
  insertMarkerArray(marker_array, createRoadMarkMarkerArray(vmap, Color::WHITE));
  insertMarkerArray(marker_array, createRoadPoleMarkerArray(vmap, Color::GRAY));
  insertMarkerArray(marker_array, createRoadSignMarkerArray(vmap, Color::GREEN, Color::GRAY));
  insertMarkerArray(marker_array, createSignalMarkerArray(vmap, Color::RED, Color::BLUE, Color::YELLOW, Color::CYAN,
                                                          Color::GRAY));
  insertMarkerArray(marker_array, createStreetLightMarkerArray(vmap, Color::YELLOW, Color::GRAY));
  insertMarkerArray(marker_array, createUtilityPoleMarkerArray(vmap, Color::GRAY));
  insertMarkerArray(marker_array, createGuardRailMarkerArray(vmap, Color::LIGHT_BLUE));
  insertMarkerArray(marker_array, createSideWalkMarkerArray(vmap, Color::GRAY));
  insertMarkerArray(marker_array, createDriveOnPortionMarkerArray(vmap, Color::LIGHT_CYAN));
  insertMarkerArray(marker_array, createCrossRoadMarkerArray(vmap, Color::LIGHT_GREEN));
  insertMarkerArray(marker_array, createSideStripMarkerArray(vmap, Color::GRAY));
  insertMarkerArray(marker_array, createCurveMirrorMarkerArray(vmap, Color::MAGENTA, Color::GRAY));
  insertMarkerArray(marker_array, createWallMarkerArray(vmap, Color::LIGHT_YELLOW));
  insertMarkerArray(marker_array, createFenceMarkerArray(vmap, Color::LIGHT_RED));
  insertMarkerArray(marker_array, createRailCrossingMarkerArray(vmap, Color::LIGHT_MAGENTA));
  marker_array_pub.publish(marker_array);

  stat.data = true;
  stat_pub.publish(stat);

  ros::spin();

  return EXIT_SUCCESS;
}
