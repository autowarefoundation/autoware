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

#ifndef VECTOR_MAP_VECTOR_MAP_H
#define VECTOR_MAP_VECTOR_MAP_H

#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>

#include <vector_map_msgs/PointArray.h>
#include <vector_map_msgs/VectorArray.h>
#include <vector_map_msgs/LineArray.h>
#include <vector_map_msgs/AreaArray.h>
#include <vector_map_msgs/PoleArray.h>
#include <vector_map_msgs/BoxArray.h>
#include <vector_map_msgs/DTLaneArray.h>
#include <vector_map_msgs/NodeArray.h>
#include <vector_map_msgs/LaneArray.h>
#include <vector_map_msgs/WayAreaArray.h>
#include <vector_map_msgs/RoadEdgeArray.h>
#include <vector_map_msgs/GutterArray.h>
#include <vector_map_msgs/CurbArray.h>
#include <vector_map_msgs/WhiteLineArray.h>
#include <vector_map_msgs/StopLineArray.h>
#include <vector_map_msgs/ZebraZoneArray.h>
#include <vector_map_msgs/CrossWalkArray.h>
#include <vector_map_msgs/RoadMarkArray.h>
#include <vector_map_msgs/RoadPoleArray.h>
#include <vector_map_msgs/RoadSignArray.h>
#include <vector_map_msgs/SignalArray.h>
#include <vector_map_msgs/StreetLightArray.h>
#include <vector_map_msgs/UtilityPoleArray.h>
#include <vector_map_msgs/GuardRailArray.h>
#include <vector_map_msgs/SideWalkArray.h>
#include <vector_map_msgs/DriveOnPortionArray.h>
#include <vector_map_msgs/CrossRoadArray.h>
#include <vector_map_msgs/SideStripArray.h>
#include <vector_map_msgs/CurveMirrorArray.h>
#include <vector_map_msgs/WallArray.h>
#include <vector_map_msgs/FenceArray.h>
#include <vector_map_msgs/RailCrossingArray.h>

namespace vector_map
{
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

using category_t = unsigned long long;

enum Category : category_t
{
  NONE = 0LLU,

  // Graphical Primitive Class
  POINT = 1LLU << 0,
  VECTOR = 1LLU << 1,
  LINE = 1LLU << 2,
  AREA = 1LLU << 3,
  POLE = 1LLU << 4,
  BOX = 1LLU << 5,

  // Road Data
  DTLANE = 1LLU << 6,
  NODE = 1LLU << 7,
  LANE = 1LLU << 8,
  WAY_AREA = 1LLU << 9,

  // Object Data
  ROAD_EDGE = 1LLU << 10,
  GUTTER = 1LLU << 11,
  CURB = 1LLU << 12,
  WHITE_LINE = 1LLU << 13,
  STOP_LINE = 1LLU << 14,
  ZEBRA_ZONE = 1LLU << 15,
  CROSS_WALK = 1LLU << 16,
  ROAD_MARK = 1LLU << 17,
  ROAD_POLE = 1LLU << 18,
  ROAD_SIGN = 1LLU << 19,
  SIGNAL = 1LLU << 20,
  STREET_LIGHT = 1LLU << 21,
  UTILITY_POLE = 1LLU << 22,
  GUARD_RAIL = 1LLU << 23,
  SIDE_WALK = 1LLU << 24,
  DRIVE_ON_PORTION = 1LLU << 25,
  CROSS_ROAD = 1LLU << 26,
  SIDE_STRIP = 1LLU << 27,
  CURVE_MIRROR = 1LLU << 28,
  WALL = 1LLU << 29,
  FENCE = 1LLU << 30,
  RAIL_CROSSING = 1LLU << 31,

  ALL = (1LLU << 32) - 1
};

enum Color : int
{
  BLACK,
  GRAY,
  LIGHT_RED,
  LIGHT_GREEN,
  LIGHT_BLUE,
  LIGHT_YELLOW,
  LIGHT_CYAN,
  LIGHT_MAGENTA,
  RED,
  GREEN,
  BLUE,
  YELLOW,
  CYAN,
  MAGENTA,
  WHITE
};

template <class T>
class Key
{
private:
  int id_;

public:
  Key()
  {
  }

  explicit Key(int id)
    : id_(id)
  {
  }

  void setId(int id)
  {
    id_ = id;
  }

  int getId() const
  {
    return id_;
  }

  bool operator<(const Key<T>& right) const
  {
    return id_ < right.getId();
  }
};

template <class T, class U>
using Updater = std::function<void(std::map<Key<T>, T>&, const U&)>;

template <class T>
using Callback = std::function<void(const T&)>;

template <class T>
using Filter = std::function<bool(const T&)>;

template <class T, class U>
class Handle
{
private:
  ros::Subscriber sub_;
  Updater<T, U> update_;
  std::vector<Callback<U>> cbs_;
  std::map<Key<T>, T> map_;

  void subscribe(const U& msg)
  {
    update_(map_, msg);
    for (const auto& cb : cbs_)
      cb(msg);
  }

public:
  Handle()
  {
  }

  void registerSubscriber(ros::NodeHandle& nh, const std::string& topic_name)
  {
    sub_ = nh.subscribe(topic_name, 1, &Handle<T, U>::subscribe, this);
  }

  void registerUpdater(const Updater<T, U>& update)
  {
    update_ = update;
  }

  void registerCallback(const Callback<U>& cb)
  {
    cbs_.push_back(cb);
  }

  T findByKey(const Key<T>& key) const
  {
    auto it = map_.find(key);
    if (it == map_.end())
      return T();
    return it->second;
  }

  std::vector<T> findByFilter(const Filter<T>& filter) const
  {
    std::vector<T> vector;
    for (const auto& pair : map_)
    {
      if (filter(pair.second))
        vector.push_back(pair.second);
    }
    return vector;
  }

  bool empty() const
  {
    return map_.empty();
  }
};

template <class T>
std::vector<T> parse(const std::string& csv_file)
{
  std::ifstream ifs(csv_file.c_str());
  std::string line;
  std::getline(ifs, line); // remove first line
  std::vector<T> objs;
  while (std::getline(ifs, line))
  {
    T obj;
    std::istringstream iss(line);
    iss >> obj;
    objs.push_back(obj);
  }
  return objs;
}

namespace
{
void updatePoint(std::map<Key<Point>, Point>& map, const PointArray& msg);
void updateVector(std::map<Key<Vector>, Vector>& map, const VectorArray& msg);
void updateLine(std::map<Key<Line>, Line>& map, const LineArray& msg);
void updateArea(std::map<Key<Area>, Area>& map, const AreaArray& msg);
void updatePole(std::map<Key<Pole>, Pole>& map, const PoleArray& msg);
void updateBox(std::map<Key<Box>, Box>& map, const BoxArray& msg);
void updateDTLane(std::map<Key<DTLane>, DTLane>& map, const DTLaneArray& msg);
void updateNode(std::map<Key<Node>, Node>& map, const NodeArray& msg);
void updateLane(std::map<Key<Lane>, Lane>& map, const LaneArray& msg);
void updateWayArea(std::map<Key<WayArea>, WayArea>& map, const WayAreaArray& msg);
void updateRoadEdge(std::map<Key<RoadEdge>, RoadEdge>& map, const RoadEdgeArray& msg);
void updateGutter(std::map<Key<Gutter>, Gutter>& map, const GutterArray& msg);
void updateCurb(std::map<Key<Curb>, Curb>& map, const CurbArray& msg);
void updateWhiteLine(std::map<Key<WhiteLine>, WhiteLine>& map, const WhiteLineArray& msg);
void updateStopLine(std::map<Key<StopLine>, StopLine>& map, const StopLineArray& msg);
void updateZebraZone(std::map<Key<ZebraZone>, ZebraZone>& map, const ZebraZoneArray& msg);
void updateCrossWalk(std::map<Key<CrossWalk>, CrossWalk>& map, const CrossWalkArray& msg);
void updateRoadMark(std::map<Key<RoadMark>, RoadMark>& map, const RoadMarkArray& msg);
void updateRoadPole(std::map<Key<RoadPole>, RoadPole>& map, const RoadPoleArray& msg);
void updateRoadSign(std::map<Key<RoadSign>, RoadSign>& map, const RoadSignArray& msg);
void updateSignal(std::map<Key<Signal>, Signal>& map, const SignalArray& msg);
void updateStreetLight(std::map<Key<StreetLight>, StreetLight>& map, const StreetLightArray& msg);
void updateUtilityPole(std::map<Key<UtilityPole>, UtilityPole>& map, const UtilityPoleArray& msg);
void updateGuardRail(std::map<Key<GuardRail>, GuardRail>& map, const GuardRailArray& msg);
void updateSideWalk(std::map<Key<SideWalk>, SideWalk>& map, const SideWalkArray& msg);
void updateDriveOnPortion(std::map<Key<DriveOnPortion>, DriveOnPortion>& map, const DriveOnPortionArray& msg);
void updateCrossRoad(std::map<Key<CrossRoad>, CrossRoad>& map, const CrossRoadArray& msg);
void updateSideStrip(std::map<Key<SideStrip>, SideStrip>& map, const SideStripArray& msg);
void updateCurveMirror(std::map<Key<CurveMirror>, CurveMirror>& map, const CurveMirrorArray& msg);
void updateWall(std::map<Key<Wall>, Wall>& map, const WallArray& msg);
void updateFence(std::map<Key<Fence>, Fence>& map, const FenceArray& msg);
void updateRailCrossing(std::map<Key<RailCrossing>, RailCrossing>& map, const RailCrossingArray& msg);
} // namespace

class VectorMap
{
private:
  Handle<Point, PointArray> point_;
  Handle<Vector, VectorArray> vector_;
  Handle<Line, LineArray> line_;
  Handle<Area, AreaArray> area_;
  Handle<Pole, PoleArray> pole_;
  Handle<Box, BoxArray> box_;
  Handle<DTLane, DTLaneArray> dtlane_;
  Handle<Node, NodeArray> node_;
  Handle<Lane, LaneArray> lane_;
  Handle<WayArea, WayAreaArray> way_area_;
  Handle<RoadEdge, RoadEdgeArray> road_edge_;
  Handle<Gutter, GutterArray> gutter_;
  Handle<Curb, CurbArray> curb_;
  Handle<WhiteLine, WhiteLineArray> white_line_;
  Handle<StopLine, StopLineArray> stop_line_;
  Handle<ZebraZone, ZebraZoneArray> zebra_zone_;
  Handle<CrossWalk, CrossWalkArray> cross_walk_;
  Handle<RoadMark, RoadMarkArray> road_mark_;
  Handle<RoadPole, RoadPoleArray> road_pole_;
  Handle<RoadSign, RoadSignArray> road_sign_;
  Handle<Signal, SignalArray> signal_;
  Handle<StreetLight, StreetLightArray> street_light_;
  Handle<UtilityPole, UtilityPoleArray> utility_pole_;
  Handle<GuardRail, GuardRailArray> guard_rail_;
  Handle<SideWalk, SideWalkArray> side_walk_;
  Handle<DriveOnPortion, DriveOnPortionArray> drive_on_portion_;
  Handle<CrossRoad, CrossRoadArray> cross_road_;
  Handle<SideStrip, SideStripArray> side_strip_;
  Handle<CurveMirror, CurveMirrorArray> curve_mirror_;
  Handle<Wall, WallArray> wall_;
  Handle<Fence, FenceArray> fence_;
  Handle<RailCrossing, RailCrossingArray> rail_crossing_;

  bool hasSubscribed(category_t category) const;
  void registerSubscriber(ros::NodeHandle& nh, category_t category);

public:
  VectorMap();

  void subscribe(ros::NodeHandle& nh, category_t category);
  void subscribe(ros::NodeHandle& nh, category_t category, const ros::Duration& timeout);

  Point findByKey(const Key<Point>& key) const;
  Vector findByKey(const Key<Vector>& key) const;
  Line findByKey(const Key<Line>& key) const;
  Area findByKey(const Key<Area>& key) const;
  Pole findByKey(const Key<Pole>& key) const;
  Box findByKey(const Key<Box>& key) const;
  DTLane findByKey(const Key<DTLane>& key) const;
  Node findByKey(const Key<Node>& key) const;
  Lane findByKey(const Key<Lane>& key) const;
  WayArea findByKey(const Key<WayArea>& key) const;
  RoadEdge findByKey(const Key<RoadEdge>& key) const;
  Gutter findByKey(const Key<Gutter>& key) const;
  Curb findByKey(const Key<Curb>& key) const;
  WhiteLine findByKey(const Key<WhiteLine>& key) const;
  StopLine findByKey(const Key<StopLine>& key) const;
  ZebraZone findByKey(const Key<ZebraZone>& key) const;
  CrossWalk findByKey(const Key<CrossWalk>& key) const;
  RoadMark findByKey(const Key<RoadMark>& key) const;
  RoadPole findByKey(const Key<RoadPole>& key) const;
  RoadSign findByKey(const Key<RoadSign>& key) const;
  Signal findByKey(const Key<Signal>& key) const;
  StreetLight findByKey(const Key<StreetLight>& key) const;
  UtilityPole findByKey(const Key<UtilityPole>& key) const;
  GuardRail findByKey(const Key<GuardRail>& key) const;
  SideWalk findByKey(const Key<SideWalk>& key) const;
  DriveOnPortion findByKey(const Key<DriveOnPortion>& key) const;
  CrossRoad findByKey(const Key<CrossRoad>& key) const;
  SideStrip findByKey(const Key<SideStrip>& key) const;
  CurveMirror findByKey(const Key<CurveMirror>& key) const;
  Wall findByKey(const Key<Wall>& key) const;
  Fence findByKey(const Key<Fence>& key) const;
  RailCrossing findByKey(const Key<RailCrossing>& key) const;

  std::vector<Point> findByFilter(const Filter<Point>& filter) const;
  std::vector<Vector> findByFilter(const Filter<Vector>& filter) const;
  std::vector<Line> findByFilter(const Filter<Line>& filter) const;
  std::vector<Area> findByFilter(const Filter<Area>& filter) const;
  std::vector<Pole> findByFilter(const Filter<Pole>& filter) const;
  std::vector<Box> findByFilter(const Filter<Box>& filter) const;
  std::vector<DTLane> findByFilter(const Filter<DTLane>& filter) const;
  std::vector<Node> findByFilter(const Filter<Node>& filter) const;
  std::vector<Lane> findByFilter(const Filter<Lane>& filter) const;
  std::vector<WayArea> findByFilter(const Filter<WayArea>& filter) const;
  std::vector<RoadEdge> findByFilter(const Filter<RoadEdge>& filter) const;
  std::vector<Gutter> findByFilter(const Filter<Gutter>& filter) const;
  std::vector<Curb> findByFilter(const Filter<Curb>& filter) const;
  std::vector<WhiteLine> findByFilter(const Filter<WhiteLine>& filter) const;
  std::vector<StopLine> findByFilter(const Filter<StopLine>& filter) const;
  std::vector<ZebraZone> findByFilter(const Filter<ZebraZone>& filter) const;
  std::vector<CrossWalk> findByFilter(const Filter<CrossWalk>& filter) const;
  std::vector<RoadMark> findByFilter(const Filter<RoadMark>& filter) const;
  std::vector<RoadPole> findByFilter(const Filter<RoadPole>& filter) const;
  std::vector<RoadSign> findByFilter(const Filter<RoadSign>& filter) const;
  std::vector<Signal> findByFilter(const Filter<Signal>& filter) const;
  std::vector<StreetLight> findByFilter(const Filter<StreetLight>& filter) const;
  std::vector<UtilityPole> findByFilter(const Filter<UtilityPole>& filter) const;
  std::vector<GuardRail> findByFilter(const Filter<GuardRail>& filter) const;
  std::vector<SideWalk> findByFilter(const Filter<SideWalk>& filter) const;
  std::vector<DriveOnPortion> findByFilter(const Filter<DriveOnPortion>& filter) const;
  std::vector<CrossRoad> findByFilter(const Filter<CrossRoad>& filter) const;
  std::vector<SideStrip> findByFilter(const Filter<SideStrip>& filter) const;
  std::vector<CurveMirror> findByFilter(const Filter<CurveMirror>& filter) const;
  std::vector<Wall> findByFilter(const Filter<Wall>& filter) const;
  std::vector<Fence> findByFilter(const Filter<Fence>& filter) const;
  std::vector<RailCrossing> findByFilter(const Filter<RailCrossing>& filter) const;

  void registerCallback(const Callback<PointArray>& cb);
  void registerCallback(const Callback<VectorArray>& cb);
  void registerCallback(const Callback<LineArray>& cb);
  void registerCallback(const Callback<AreaArray>& cb);
  void registerCallback(const Callback<PoleArray>& cb);
  void registerCallback(const Callback<BoxArray>& cb);
  void registerCallback(const Callback<DTLaneArray>& cb);
  void registerCallback(const Callback<NodeArray>& cb);
  void registerCallback(const Callback<LaneArray>& cb);
  void registerCallback(const Callback<WayAreaArray>& cb);
  void registerCallback(const Callback<RoadEdgeArray>& cb);
  void registerCallback(const Callback<GutterArray>& cb);
  void registerCallback(const Callback<CurbArray>& cb);
  void registerCallback(const Callback<WhiteLineArray>& cb);
  void registerCallback(const Callback<StopLineArray>& cb);
  void registerCallback(const Callback<ZebraZoneArray>& cb);
  void registerCallback(const Callback<CrossWalkArray>& cb);
  void registerCallback(const Callback<RoadMarkArray>& cb);
  void registerCallback(const Callback<RoadPoleArray>& cb);
  void registerCallback(const Callback<RoadSignArray>& cb);
  void registerCallback(const Callback<SignalArray>& cb);
  void registerCallback(const Callback<StreetLightArray>& cb);
  void registerCallback(const Callback<UtilityPoleArray>& cb);
  void registerCallback(const Callback<GuardRailArray>& cb);
  void registerCallback(const Callback<SideWalkArray>& cb);
  void registerCallback(const Callback<DriveOnPortionArray>& cb);
  void registerCallback(const Callback<CrossRoadArray>& cb);
  void registerCallback(const Callback<SideStripArray>& cb);
  void registerCallback(const Callback<CurveMirrorArray>& cb);
  void registerCallback(const Callback<WallArray>& cb);
  void registerCallback(const Callback<FenceArray>& cb);
  void registerCallback(const Callback<RailCrossingArray>& cb);
};

extern const double COLOR_VALUE_MIN;
extern const double COLOR_VALUE_MAX;
extern const double COLOR_VALUE_MEDIAN;
extern const double COLOR_VALUE_LIGHT_LOW;
extern const double COLOR_VALUE_LIGHT_HIGH;

std_msgs::ColorRGBA createColorRGBA(Color color);

void enableMarker(visualization_msgs::Marker& marker);
void disableMarker(visualization_msgs::Marker& marker);
bool isValidMarker(const visualization_msgs::Marker& marker);

extern const double MAKER_SCALE_POINT;
extern const double MAKER_SCALE_VECTOR;
extern const double MAKER_SCALE_VECTOR_LENGTH;
extern const double MAKER_SCALE_LINE;
extern const double MAKER_SCALE_AREA;
extern const double MAKER_SCALE_BOX;

visualization_msgs::Marker createMarker(const std::string& ns, int id, int type);
visualization_msgs::Marker createPointMarker(const std::string& ns, int id, Color color, const Point& point);
visualization_msgs::Marker createVectorMarker(const std::string& ns, int id, Color color, const VectorMap& vmap,
                                              const Vector& vector);
visualization_msgs::Marker createLineMarker(const std::string& ns, int id, Color color, const VectorMap& vmap,
                                            const Line& line);
visualization_msgs::Marker createAreaMarker(const std::string& ns, int id, Color color, const VectorMap& vmap,
                                            const Area& area);
visualization_msgs::Marker createPoleMarker(const std::string& ns, int id, Color color, const VectorMap& vmap,
                                            const Pole& pole);
visualization_msgs::Marker createBoxMarker(const std::string& ns, int id, Color color, const VectorMap& vmap,
                                           const Box& box);

double convertDegreeToRadian(double degree);
double convertRadianToDegree(double radian);
geometry_msgs::Point convertPointToGeomPoint(const Point& point);
Point convertGeomPointToPoint(const geometry_msgs::Point& geom_point);
geometry_msgs::Quaternion convertVectorToGeomQuaternion(const Vector& vector);
Vector convertGeomQuaternionToVector(const geometry_msgs::Quaternion& geom_quaternion);
} // namespace vector_map

std::ostream& operator<<(std::ostream& os, const vector_map::Point& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::Vector& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::Line& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::Area& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::Pole& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::Box& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::DTLane& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::Node& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::Lane& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::WayArea& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::RoadEdge& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::Gutter& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::Curb& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::WhiteLine& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::StopLine& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::ZebraZone& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::CrossWalk& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::RoadMark& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::RoadPole& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::RoadSign& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::Signal& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::StreetLight& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::UtilityPole& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::GuardRail& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::SideWalk& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::DriveOnPortion& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::CrossRoad& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::SideStrip& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::CurveMirror& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::Wall& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::Fence& obj);
std::ostream& operator<<(std::ostream& os, const vector_map::RailCrossing& obj);

std::istream& operator>>(std::istream& is, vector_map::Point& obj);
std::istream& operator>>(std::istream& is, vector_map::Vector& obj);
std::istream& operator>>(std::istream& is, vector_map::Line& obj);
std::istream& operator>>(std::istream& is, vector_map::Area& obj);
std::istream& operator>>(std::istream& is, vector_map::Pole& obj);
std::istream& operator>>(std::istream& is, vector_map::Box& obj);
std::istream& operator>>(std::istream& is, vector_map::DTLane& obj);
std::istream& operator>>(std::istream& is, vector_map::Node& obj);
std::istream& operator>>(std::istream& is, vector_map::Lane& obj);
std::istream& operator>>(std::istream& is, vector_map::WayArea& obj);
std::istream& operator>>(std::istream& is, vector_map::RoadEdge& obj);
std::istream& operator>>(std::istream& is, vector_map::Gutter& obj);
std::istream& operator>>(std::istream& is, vector_map::Curb& obj);
std::istream& operator>>(std::istream& is, vector_map::WhiteLine& obj);
std::istream& operator>>(std::istream& is, vector_map::StopLine& obj);
std::istream& operator>>(std::istream& is, vector_map::ZebraZone& obj);
std::istream& operator>>(std::istream& is, vector_map::CrossWalk& obj);
std::istream& operator>>(std::istream& is, vector_map::RoadMark& obj);
std::istream& operator>>(std::istream& is, vector_map::RoadPole& obj);
std::istream& operator>>(std::istream& is, vector_map::RoadSign& obj);
std::istream& operator>>(std::istream& is, vector_map::Signal& obj);
std::istream& operator>>(std::istream& is, vector_map::StreetLight& obj);
std::istream& operator>>(std::istream& is, vector_map::UtilityPole& obj);
std::istream& operator>>(std::istream& is, vector_map::GuardRail& obj);
std::istream& operator>>(std::istream& is, vector_map::SideWalk& obj);
std::istream& operator>>(std::istream& is, vector_map::DriveOnPortion& obj);
std::istream& operator>>(std::istream& is, vector_map::CrossRoad& obj);
std::istream& operator>>(std::istream& is, vector_map::SideStrip& obj);
std::istream& operator>>(std::istream& is, vector_map::CurveMirror& obj);
std::istream& operator>>(std::istream& is, vector_map::Wall& obj);
std::istream& operator>>(std::istream& is, vector_map::Fence& obj);
std::istream& operator>>(std::istream& is, vector_map::RailCrossing& obj);

#endif // VECTOR_MAP_VECTOR_MAP_H
