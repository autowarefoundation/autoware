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


// #include <sys/stat.h>
#include <ros/console.h>
#include <vector_map/vector_map.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <std_msgs/Bool.h>

using vector_map_msgs::Point;
using vector_map_msgs::Vector;
using vector_map_msgs::Line;
using vector_map_msgs::Area;
using vector_map_msgs::Pole;
using vector_map_msgs::Box;
using vector_map_msgs::DTLane;
using vector_map_msgs::Node;
using vector_map_msgs::Lane;
using vector_map_msgs::WayArea;
using vector_map_msgs::RoadEdge;
using vector_map_msgs::Gutter;
using vector_map_msgs::Curb;
using vector_map_msgs::WhiteLine;
using vector_map_msgs::StopLine;
using vector_map_msgs::ZebraZone;
using vector_map_msgs::CrossWalk;
using vector_map_msgs::RoadMark;
using vector_map_msgs::RoadPole;
using vector_map_msgs::RoadSign;
using vector_map_msgs::Signal;
using vector_map_msgs::StreetLight;
using vector_map_msgs::UtilityPole;
using vector_map_msgs::GuardRail;
using vector_map_msgs::SideWalk;
using vector_map_msgs::DriveOnPortion;
using vector_map_msgs::CrossRoad;
using vector_map_msgs::SideStrip;
using vector_map_msgs::CurveMirror;
using vector_map_msgs::Wall;
using vector_map_msgs::Fence;
using vector_map_msgs::RailCrossing;

using vector_map_msgs::PointArray;
using vector_map_msgs::VectorArray;
using vector_map_msgs::LineArray;
using vector_map_msgs::AreaArray;
using vector_map_msgs::PoleArray;
using vector_map_msgs::BoxArray;
using vector_map_msgs::DTLaneArray;
using vector_map_msgs::NodeArray;
using vector_map_msgs::LaneArray;
using vector_map_msgs::WayAreaArray;
using vector_map_msgs::RoadEdgeArray;
using vector_map_msgs::GutterArray;
using vector_map_msgs::CurbArray;
using vector_map_msgs::WhiteLineArray;
using vector_map_msgs::StopLineArray;
using vector_map_msgs::ZebraZoneArray;
using vector_map_msgs::CrossWalkArray;
using vector_map_msgs::RoadMarkArray;
using vector_map_msgs::RoadPoleArray;
using vector_map_msgs::RoadSignArray;
using vector_map_msgs::SignalArray;
using vector_map_msgs::StreetLightArray;
using vector_map_msgs::UtilityPoleArray;
using vector_map_msgs::GuardRailArray;
using vector_map_msgs::SideWalkArray;
using vector_map_msgs::DriveOnPortionArray;
using vector_map_msgs::CrossRoadArray;
using vector_map_msgs::SideStripArray;
using vector_map_msgs::CurveMirrorArray;
using vector_map_msgs::WallArray;
using vector_map_msgs::FenceArray;
using vector_map_msgs::RailCrossingArray;

namespace
{
void printUsage()
{
  ROS_ERROR_STREAM("Usage:");
  ROS_ERROR_STREAM("rosrun map_file vector_map_loader [CSV]...");
  ROS_ERROR_STREAM("rosrun map_file vector_map_loader download [x] [y]");
}

template <class T, class U>
void publishVectorMap(ros::Publisher& pub, const std::string& file_path)
{
  U msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.data = vector_map::parse<T>(file_path);
  pub.publish(msg);
}

// bool isDownloaded(const std::string& file_path)
// {
//   struct stat st;
//   return stat(file_path.c_str(), &st) == 0;
// }
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vector_map_loader");
  ros::NodeHandle n;

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

  ros::Publisher point_pub = n.advertise<PointArray>("vector_map_info/point", 1, true);
  ros::Publisher vector_pub = n.advertise<VectorArray>("vector_map_info/vector", 1, true);
  ros::Publisher line_pub = n.advertise<LineArray>("vector_map_info/line", 1, true);
  ros::Publisher area_pub = n.advertise<AreaArray>("vector_map_info/area", 1, true);
  ros::Publisher pole_pub = n.advertise<PoleArray>("vector_map_info/pole", 1, true);
  ros::Publisher box_pub = n.advertise<BoxArray>("vector_map_info/box", 1, true);
  ros::Publisher dtlane_pub = n.advertise<DTLaneArray>("vector_map_info/dtlane", 1, true);
  ros::Publisher node_pub = n.advertise<NodeArray>("vector_map_info/node", 1, true);
  ros::Publisher lane_pub = n.advertise<LaneArray>("vector_map_info/lane", 1, true);
  ros::Publisher way_area_pub = n.advertise<WayAreaArray>("vector_map_info/way_area", 1, true);
  ros::Publisher road_edge_pub = n.advertise<RoadEdgeArray>("vector_map_info/road_edge", 1, true);
  ros::Publisher gutter_pub = n.advertise<GutterArray>("vector_map_info/gutter", 1, true);
  ros::Publisher curb_pub = n.advertise<CurbArray>("vector_map_info/curb", 1, true);
  ros::Publisher white_line_pub = n.advertise<WhiteLineArray>("vector_map_info/white_line", 1, true);
  ros::Publisher stop_line_pub = n.advertise<StopLineArray>("vector_map_info/stop_line", 1, true);
  ros::Publisher zebra_zone_pub = n.advertise<ZebraZoneArray>("vector_map_info/zebra_zone", 1, true);
  ros::Publisher cross_walk_pub = n.advertise<CrossWalkArray>("vector_map_info/cross_walk", 1, true);
  ros::Publisher road_mark_pub = n.advertise<RoadMarkArray>("vector_map_info/road_mark", 1, true);
  ros::Publisher road_pole_pub = n.advertise<RoadPoleArray>("vector_map_info/road_pole", 1, true);
  ros::Publisher road_sign_pub = n.advertise<RoadSignArray>("vector_map_info/road_sign", 1, true);
  ros::Publisher signal_pub = n.advertise<SignalArray>("vector_map_info/signal", 1, true);
  ros::Publisher street_light_pub = n.advertise<StreetLightArray>("vector_map_info/street_light", 1, true);
  ros::Publisher utility_pole_pub = n.advertise<UtilityPoleArray>("vector_map_info/utility_pole", 1, true);
  ros::Publisher guard_rail_pub = n.advertise<GuardRailArray>("vector_map_info/guard_rail", 1, true);
  ros::Publisher side_walk_pub = n.advertise<SideWalkArray>("vector_map_info/side_walk", 1, true);
  ros::Publisher drive_on_portion_pub = n.advertise<DriveOnPortionArray>("vector_map_info/drive_on_portion", 1, true);
  ros::Publisher cross_road_pub = n.advertise<CrossRoadArray>("vector_map_info/cross_road", 1, true);
  ros::Publisher side_strip_pub = n.advertise<SideStripArray>("vector_map_info/side_strip", 1, true);
  ros::Publisher curve_mirror_pub = n.advertise<CurveMirrorArray>("vector_map_info/curve_mirror", 1, true);
  ros::Publisher wall_pub = n.advertise<WallArray>("vector_map_info/wall", 1, true);
  ros::Publisher fence_pub = n.advertise<FenceArray>("vector_map_info/fence", 1, true);
  ros::Publisher rail_crossing_pub = n.advertise<RailCrossingArray>("vector_map_info/rail_crossing", 1, true);

  // ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("vector_map", 1, true);
  // ros::Publisher stat_pub = n.advertise<std_msgs::Bool>("vmap_stat", 1, true);

  std::vector<std::string> file_paths;
  if (mode == "download")
  {
    // do nothing now
  }
  else
  {
    for (int i = 1; i < argc; ++i)
    {
      std::string file_path(argv[i]);
      file_paths.push_back(file_path);
    }
  }

  for (const auto& file_path: file_paths)
  {
    std::string file_name(basename(file_path.c_str()));
    if (file_name == "point.csv")
      publishVectorMap<Point, PointArray>(point_pub, file_path);
    else if (file_name == "vector.csv")
      publishVectorMap<Vector, VectorArray>(vector_pub, file_path);
    else if (file_name == "line.csv")
      publishVectorMap<Line, LineArray>(line_pub, file_path);
    else if (file_name == "area.csv")
      publishVectorMap<Area, AreaArray>(area_pub, file_path);
    else if (file_name == "pole.csv")
      publishVectorMap<Pole, PoleArray>(pole_pub, file_path);
    else if (file_name == "box.csv")
      publishVectorMap<Box, BoxArray>(box_pub, file_path);
    else if (file_name == "dtlane.csv")
      publishVectorMap<DTLane, DTLaneArray>(dtlane_pub, file_path);
    else if (file_name == "node.csv")
      publishVectorMap<Node, NodeArray>(node_pub, file_path);
    else if (file_name == "lane.csv")
      publishVectorMap<Lane, LaneArray>(lane_pub, file_path);
    else if (file_name == "wayarea.csv")
      publishVectorMap<WayArea, WayAreaArray>(way_area_pub, file_path);
    else if (file_name == "roadedge.csv")
      publishVectorMap<RoadEdge, RoadEdgeArray>(road_edge_pub, file_path);
    else if (file_name == "gutter.csv")
      publishVectorMap<Gutter, GutterArray>(gutter_pub, file_path);
    else if (file_name == "curb.csv")
      publishVectorMap<Curb, CurbArray>(curb_pub, file_path);
    else if (file_name == "whiteline.csv")
      publishVectorMap<WhiteLine, WhiteLineArray>(white_line_pub, file_path);
    else if (file_name == "stopline.csv")
      publishVectorMap<StopLine, StopLineArray>(stop_line_pub, file_path);
    else if (file_name == "zebrazone.csv")
      publishVectorMap<ZebraZone, ZebraZoneArray>(zebra_zone_pub, file_path);
    else if (file_name == "crosswalk.csv")
      publishVectorMap<CrossWalk, CrossWalkArray>(cross_walk_pub, file_path);
    else if (file_name == "road_surface_mark.csv")
      publishVectorMap<RoadMark, RoadMarkArray>(road_mark_pub, file_path);
    else if (file_name == "poledata.csv")
      publishVectorMap<RoadPole, RoadPoleArray>(road_pole_pub, file_path);
    else if (file_name == "roadsign.csv")
      publishVectorMap<RoadSign, RoadSignArray>(road_sign_pub, file_path);
    else if (file_name == "signaldata.csv")
      publishVectorMap<Signal, SignalArray>(signal_pub, file_path);
    else if (file_name == "streetlight.csv")
      publishVectorMap<StreetLight, StreetLightArray>(street_light_pub, file_path);
    else if (file_name == "utilitypole.csv")
      publishVectorMap<UtilityPole, UtilityPoleArray>(utility_pole_pub, file_path);
    else if (file_name == "guardrail.csv")
      publishVectorMap<GuardRail, GuardRailArray>(guard_rail_pub, file_path);
    else if (file_name == "sidewalk.csv")
      publishVectorMap<SideWalk, SideWalkArray>(side_walk_pub, file_path);
    else if (file_name == "driveon_portion.csv")
      publishVectorMap<DriveOnPortion, DriveOnPortionArray>(drive_on_portion_pub, file_path);
    else if (file_name == "intersection.csv")
      publishVectorMap<CrossRoad, CrossRoadArray>(cross_road_pub, file_path);
    else if (file_name == "sidestrip.csv")
      publishVectorMap<SideStrip, SideStripArray>(side_strip_pub, file_path);
    else if (file_name == "curvemirror.csv")
      publishVectorMap<CurveMirror, CurveMirrorArray>(curve_mirror_pub, file_path);
    else if (file_name == "wall.csv")
      publishVectorMap<Wall, WallArray>(wall_pub, file_path);
    else if (file_name == "fence.csv")
      publishVectorMap<Fence, FenceArray>(fence_pub, file_path);
    else if (file_name == "railroad_crossing.csv")
      publishVectorMap<RailCrossing, RailCrossingArray>(rail_crossing_pub, file_path);
    else
      ROS_ERROR_STREAM("unknown vector map: " << file_path);
  }

  // publish marker messages

  // publish status message

  ros::spin();

  return EXIT_SUCCESS;
}
