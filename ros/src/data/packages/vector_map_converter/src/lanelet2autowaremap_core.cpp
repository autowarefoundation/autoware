/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <vector_map_converter/lanelet2autowaremap.hpp>
#include <amathutils_lib/amathutils.hpp>
#include <geodesy/utm.h>

using namespace lanelet;

#define BLINKER_RIGHT 2
#define BLINKER_LEFT 1
#define SIGNAL_RED 1
#define SIGNAL_GREEN 2
#define SIGNAL_YELLOW 3

BOOST_GEOMETRY_REGISTER_MULTI_POINT(decltype(std::vector<BasicPoint2d>{} ))

template <class T>
void write(std::ofstream &ofs, std::vector<T> objs)
{
  for( auto obj : objs)
  {
    ofs << obj << std::endl;
  }
}

/**
 * [calculates MGRS x,y from lat/lng information]
 * @method fixPointCoordinate
 * @param  point              [point with lat/lng and information]
 */

void fixPointCoordinate(autoware_map_msgs::Point &point)
{
  geographic_msgs::GeoPoint wgs_point;
  wgs_point.latitude = point.lat;
  wgs_point.longitude = point.lng;
  wgs_point.altitude = point.z;

  geodesy::UTMPoint utm_point;
  geodesy::fromMsg(wgs_point,utm_point);
  point.x = fmod(utm_point.easting,1e5);
  point.y = fmod(utm_point.northing,1e5);
}

void writeAutowareMapMsgs(std::string output_dir,
                          std::vector<autoware_map_msgs::Area> &areas,
                          std::vector<autoware_map_msgs::Lane> &lanes,
                          std::vector<autoware_map_msgs::LaneAttributeRelation> &lane_attribute_relations,
                          std::vector<autoware_map_msgs::LaneChangeRelation> &lane_change_relations,
                          std::vector<autoware_map_msgs::LaneRelation> &lane_relations,
                          std::vector<autoware_map_msgs::LaneSignalLightRelation> &lane_signal_light_relations,
                          std::vector<autoware_map_msgs::OppositeLaneRelation> &opposite_lane_relations,
                          std::vector<autoware_map_msgs::Point> &points,
                          std::vector<autoware_map_msgs::Signal> &signals,
                          std::vector<autoware_map_msgs::SignalLight> &signal_lights,
                          std::vector<autoware_map_msgs::Wayarea> &wayareas,
                          std::vector<autoware_map_msgs::Waypoint> &waypoints,
                          std::vector<autoware_map_msgs::WaypointLaneRelation> &waypoint_lane_relations,
                          std::vector<autoware_map_msgs::WaypointRelation> &waypoint_relations,
                          std::vector<autoware_map_msgs::WaypointSignalRelation> &waypoint_signal_relations)
{
  std::ofstream ofs;
  std::string filename;

  //areas
  if(!areas.empty())
  {
    filename = output_dir + "/areas.csv";
    ofs.open(filename);
    ofs << "area_id,point_ids" << std::endl;
    write(ofs, areas);
    ofs.close();
  }

  //lanes
  if(!lanes.empty())
  {
    filename = output_dir + "/lanes.csv";
    ofs.open(filename);
    ofs << "lane_id,start_waypoint_id,end_waypoint_id,lane_number,num_of_lanes,speed_limit,length,width_limit,height_limit,weight_limit" << std::endl;
    write(ofs, lanes);
    ofs.close();
  }
  //lane_attribute_relations
  if(!lane_attribute_relations.empty())
  {
    filename = output_dir + "/lane_attribute_relations.csv";
    ofs.open(filename);
    ofs << "lane_id,attribute_type,area_id" << std::endl;
    write(ofs, lane_attribute_relations);
    ofs.close();
  }

  //lane_change_relations
  if(!lane_change_relations.empty())
  {

    filename = output_dir + "/lane_change_relations.csv";
    ofs.open(filename);
    ofs << "lane_id,next_lane_id,blinker" << std::endl;
    write(ofs, lane_change_relations);
    ofs.close();
  }
  //lane_relations
  if(!lane_relations.empty())
  {
    filename = output_dir + "/lane_relations.csv";
    ofs.open(filename);
    ofs << "lane_id,next_lane_id,blinker" << std::endl;
    write(ofs, lane_relations);
    ofs.close();
  }
  //lane_signal_light_relations
  if(!lane_signal_light_relations.empty())
  {
    filename = output_dir + "/lane_signal_light_relations.csv";
    ofs.open(filename);
    ofs << "lane_id,signal_light_id" << std::endl;
    write(ofs, lane_signal_light_relations);
    ofs.close();
  }
  //opposite_lane_relations
  if(!opposite_lane_relations.empty())
  {
    filename = output_dir + "/opposite_lane_relations.csv";
    ofs.open(filename);
    ofs << "lane_id,opposite_lane_id" << std::endl;
    write(ofs, opposite_lane_relations);
    ofs.close();
  }
  //points
  if(!points.empty())
  {
    filename = output_dir + "/points.csv";
    ofs.open(filename);
    ofs << "point_id,y,x,z,lat,lng,pcd,mgrs,epsg" << std::endl;
    write(ofs, points);
    ofs.close();
  }
  //signals
  if(!signals.empty())
  {
    filename = output_dir + "/signals.csv";
    ofs.open(filename);
    ofs << "signal_id" << std::endl;
    write(ofs, signals);
    ofs.close();
  }
  //signal_lights
  if(!signal_lights.empty())
  {
    filename = output_dir + "/signal_lights.csv";
    ofs.open(filename);
    ofs << "signal_light_id,signal_id,point_id,horizontal_angle,vertical_angle,color_type,arrow_type" << std::endl;
    write(ofs, signal_lights);
    ofs.close();
  }
  //wayareas
  if(!wayareas.empty())
  {
    filename = output_dir + "/wayareas.csv";
    ofs.open(filename);
    ofs << "wayarea_id,area_id" << std::endl;
    write(ofs, wayareas);
    ofs.close();
  }
  //waypoints
  if(!waypoints.empty())
  {
    filename = output_dir + "/waypoints.csv";
    ofs.open(filename);
    ofs << "waypoint_id,point_id,velocity,stop_line,width,height" << std::endl;
    write(ofs, waypoints);
    ofs.close();
  }
  //waypoint_lane_relations
  if(!waypoint_lane_relations.empty())
  {
    filename = output_dir + "/waypoint_lane_relations.csv";
    ofs.open(filename);
    ofs << "waypoint_id,lane_id,order" << std::endl;
    write(ofs, waypoint_lane_relations);
    ofs.close();
  }
  //waypoint_relations
  if(!waypoint_relations.empty())
  {
    filename = output_dir + "/waypoint_relations.csv";
    ofs.open(filename);
    ofs << "waypoint_id,next_waypoint_id,yaw,blinker,distance" << std::endl;
    write(ofs, waypoint_relations);
    ofs.close();
  }
  if(!waypoint_signal_relations.empty())
  {
    filename = output_dir + "/waypoint_signal_relations.csv";
    ofs.open(filename);
    ofs << "waypoint_id,signal_id" << std::endl;
    write(ofs, waypoint_signal_relations);
    ofs.close();
  }
}

autoware_map_msgs::Point convertPoint(const lanelet::BasicPoint3d lanelet_point, const projection::UtmProjector &projector)
{
  autoware_map_msgs::Point awmap_point;
  GPSPoint utm_point = projector.reverse(lanelet_point);

  awmap_point.x = lanelet_point.x();
  awmap_point.y = lanelet_point.y();
  awmap_point.z = utm_point.ele;
  awmap_point.pcd="";
  awmap_point.mgrs=0;
  awmap_point.epsg = 0;
  awmap_point.lat = utm_point.lat;
  awmap_point.lng = utm_point.lon;
  return awmap_point;
}

boost::optional<ConstLanelet> getClosestOppositeLanes(LaneletMapPtr map,const routing::RoutingGraphPtr graph, const ConstLanelet &lanelet,const traffic_rules::TrafficRulesPtr traffic_rules)
{
  double padding = 1;
  BasicPoint2d min,max;
  for (auto beside : graph->besides(lanelet))
  {
    for(auto pt : beside.rightBound())
    {
      min.x() = (pt.x() < min.x()) ? pt.x() : min.x();
      min.y() = (pt.y() < min.y()) ? pt.y() : min.y();
      max.x() = (pt.x() > max.x()) ? pt.x() : max.x();
      max.y() = (pt.y() > max.y()) ? pt.y() : max.y();
    }
  }
  min.x() -= padding;
  min.y() -= padding;
  max.x() += padding;
  max.y() += padding;

  auto local_lanelets = map->laneletLayer.search(BoundingBox2d(min,max));

  for (auto beside : graph->besides(lanelet))
  {
    for( auto l : local_lanelets )
    {
      if(!traffic_rules->canPass(l)) continue;
      if ( beside.rightBound() == l.rightBound().invert()) {
        return l;
      }
      if ( beside.leftBound() == l.leftBound().invert()) {
        return l;
      }
    }
  }
  return boost::none;
}

std::vector<BasicPoint3d>  fine_centerline(const ConstLanelet &lanelet)
{
  double resolution = 1;
  std::vector<BasicPoint3d> centerline;
  std::vector<BasicPoint3d> left_points;
  std::vector<BasicPoint3d> right_points;

  auto left =lanelet.leftBound();
  auto right = lanelet.rightBound();
  double left_distance = 0;
  double right_distance = 0;

  //get distance of left and right bounds
  BasicPoint3d prev_pt = left.front().basicPoint();
  for(auto pt : left)
  {
    left_distance += geometry::distance2d(pt, prev_pt);
    prev_pt = pt.basicPoint();
  }
  prev_pt = right.front().basicPoint();
  for(auto pt : right)
  {
    right_distance += geometry::distance2d(pt, prev_pt);
    prev_pt = pt.basicPoint();
  }

  double longer_distance = (left_distance > right_distance) ? left_distance : right_distance;

  unsigned int partitions = ceil(longer_distance/resolution);
  double left_resolution = left_distance / partitions;
  double right_resolution = right_distance / partitions;

  left_points.push_back(left.front());
  double epsilon = 1e-2;
  double residue = 0;

  prev_pt = left.front().basicPoint();
  for(BasicPoint3d pt : left)
  {

    //continue if not points are made
    double local_distance = geometry::distance2d(pt, prev_pt);
    if(local_distance + residue < left_resolution) {
      residue += local_distance;
      prev_pt = pt;
      continue;
    }

    BasicPoint3d direction = ( pt - prev_pt ) / local_distance;
    for(double length = left_resolution - residue; length < local_distance; length += left_resolution)
    {
      BasicPoint3d partition_point = prev_pt + direction * length;
      left_points.push_back(partition_point);
      if(left_points.size() >= partitions) break;
      residue = local_distance - length;
    }
    if(left_points.size() >= partitions) break;
    prev_pt = pt;
  }
  left_points.push_back(left.back());

  prev_pt = right.front().basicPoint();
  residue = 0;
  right_points.push_back(right.front());
  for(BasicPoint3d pt : right)
  {
    //continue if not points are made
    double local_distance = geometry::distance2d(pt, prev_pt);
    if(local_distance +  residue < right_resolution) {
      residue += local_distance;
      prev_pt = pt;
      continue;
    }

    BasicPoint3d direction = ( pt - prev_pt ) / local_distance;
    for(double length = right_resolution - residue; length < local_distance; length += right_resolution)
    {
      BasicPoint3d partition_point = prev_pt + direction * length;
      right_points.push_back(partition_point);
      if(right_points.size() >= partitions) break;
      residue = local_distance - length;
    }
    if(right_points.size() >= partitions) break;
    prev_pt = pt;
  }
  right_points.push_back(right.back());

  if(right_points.size() != left_points.size()) {
    ROS_ERROR_STREAM("left and right has different number of points. (right,left) " << right_points.size() << ","<< left_points.size() << "failed to calculate centerline!!!" << std::endl);
    exit(1);
  }

  for (unsigned int i = 0; i < left_points.size(); i++ )
  {
    BasicPoint3d center_point = (right_points.at(i) + left_points.at(i)) / 2;
    if(USE_FIXED_HEIGHT) {
      center_point.z() = FIXED_HEIGHT;
    }
    centerline.push_back(center_point);
  }
  return centerline;
}

int getNewId(const int id, const std::unordered_map<int, int> &waypoint_id_correction )
{
  auto new_waypoint_id = waypoint_id_correction.find(id);
  if(new_waypoint_id != waypoint_id_correction.end()) {
    return new_waypoint_id->second;
  }else{
    return id;
  }

}

void convertLanelet2AutowareMap(LaneletMapPtr map,
                                projection::UtmProjector projector,
                                std::vector<autoware_map_msgs::Area> &areas,
                                std::vector<autoware_map_msgs::Lane> &lanes,
                                std::vector<autoware_map_msgs::LaneAttributeRelation> &lane_attribute_relations,
                                std::vector<autoware_map_msgs::LaneChangeRelation> &lane_change_relations,
                                std::vector<autoware_map_msgs::LaneRelation> &lane_relations,
                                std::vector<autoware_map_msgs::LaneSignalLightRelation> &lane_signal_light_relations,
                                std::vector<autoware_map_msgs::OppositeLaneRelation> &opposite_lane_relations,
                                std::vector<autoware_map_msgs::Point> &points,
                                std::vector<autoware_map_msgs::Signal> &signals,
                                std::vector<autoware_map_msgs::SignalLight> &signal_lights,
                                std::vector<autoware_map_msgs::Wayarea> &wayareas,
                                std::vector<autoware_map_msgs::Waypoint> &waypoints,
                                std::vector<autoware_map_msgs::WaypointLaneRelation> &waypoint_lane_relations,
                                std::vector<autoware_map_msgs::WaypointRelation> &waypoint_relations,
                                std::vector<autoware_map_msgs::WaypointSignalRelation> &waypoint_signal_relations)
{

  traffic_rules::TrafficRulesPtr traffic_rules =
    traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
  routing::RoutingGraphPtr vehicle_graph = routing::RoutingGraph::build(*map, *traffic_rules);
  traffic_rules::TrafficRulesPtr pedestrian_rules =
    traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Pedestrian);
  routing::RoutingGraphConstPtr pedestrian_graph = routing::RoutingGraph::build(*map, *pedestrian_rules);
  routing::RoutingGraphContainer overall_graphs({vehicle_graph, pedestrian_graph});

  //get Lanes
  int waypoint_id = 1;
  int point_id = 1;
  int lane_id = 1;
  std::unordered_map<long long int,int> lane_id_table;
  std::unordered_map<int, autoware_map_msgs::Lane> lane_map;
  std::unordered_map<int, autoware_map_msgs::Point> waypoint_id_to_point_map;
  std::unordered_map<int, int> waypoint_id_correction;
  std::unordered_map<int, std::shared_ptr<autoware_map_msgs::Waypoint> > waypoints_map;

  std::vector<ConstLanelet> vehicle_lanelets;

  for (const auto &lanelet : map->laneletLayer)
  {
    if(traffic_rules->canPass(lanelet)) {
      if(lanelet.leftBound().empty() || lanelet.rightBound().empty()) {
        continue;
      }
      vehicle_lanelets.push_back(lanelet);
      lane_id_table[lanelet.id()] = lane_id++;
    }
  }


  for (const auto &lanelet : vehicle_lanelets)
  {
    int order = 1;
    int start_waypoint_id = -1;
    int end_waypoint_id = -1;
    double lane_distance = 0;

    lanelet::BasicPoint3d prev_center_point;
    autoware_map_msgs::Waypoint prev_awmap_waypoint;

    double width_limit = std::numeric_limits<double>::max();
    for( auto center_point : fine_centerline(lanelet))
    {
      //point
      autoware_map_msgs::Waypoint awmap_waypoint;
      autoware_map_msgs::Point awmap_point = convertPoint(center_point, projector);
      awmap_point.point_id = point_id++;
      points.push_back(awmap_point);

      //waypoint
      awmap_waypoint.waypoint_id = waypoint_id++;
      awmap_waypoint.point_id = awmap_point.point_id;
      awmap_waypoint.velocity = traffic_rules->speedLimit(lanelet).speedLimit.value();
      awmap_waypoint.stop_line = 0;
      double distance_left = geometry::distance2d(center_point, lanelet.leftBound());
      double distance_right= geometry::distance2d(center_point, lanelet.rightBound());
      awmap_waypoint.left_width = distance_left;
      awmap_waypoint.right_width = distance_right;
      double width = distance_left + distance_right;
      width_limit = (width_limit > width) ? width_limit : width;
      awmap_waypoint.height = 0;
      // waypoints.push_back(awmap_waypoint);
      waypoints_map[awmap_waypoint.waypoint_id] = std::make_shared<autoware_map_msgs::Waypoint>(awmap_waypoint);
      waypoint_id_to_point_map[awmap_waypoint.waypoint_id] = awmap_point;

      //waypoint lane relation
      autoware_map_msgs::WaypointLaneRelation waypoint_lane_relation;
      waypoint_lane_relation.waypoint_id = awmap_waypoint.waypoint_id;
      waypoint_lane_relation.lane_id = lane_id_table.at(lanelet.id());
      waypoint_lane_relation.order = order;
      waypoint_lane_relations.push_back(waypoint_lane_relation);
      if (order == 1) {
        start_waypoint_id = awmap_waypoint.waypoint_id;
      }
      if (order > 1 ) {
        //waypoint_relation
        autoware_map_msgs::WaypointRelation waypoint_relation;
        waypoint_relation.waypoint_id = prev_awmap_waypoint.waypoint_id;
        waypoint_relation.next_waypoint_id = awmap_waypoint.waypoint_id;
        waypoint_relation.yaw = M_PI/2 - atan2(center_point.y() - prev_center_point.y(), center_point.x() - prev_center_point.x());
        waypoint_relation.blinker = 0;
        waypoint_relation.distance = geometry::distance2d(center_point, prev_center_point);
        waypoint_relations.push_back(waypoint_relation);

        lane_distance += waypoint_relation.distance;
      }
      prev_awmap_waypoint = awmap_waypoint;
      prev_center_point = BasicPoint3d(center_point.x(), center_point.y(), center_point.z());
      order++;
      end_waypoint_id = awmap_waypoint.waypoint_id;
    }

    autoware_map_msgs::Lane awmap_lane;
    awmap_lane.lane_id = lane_id_table.at(lanelet.id());
    awmap_lane.start_waypoint_id = start_waypoint_id;
    awmap_lane.end_waypoint_id = end_waypoint_id;
    awmap_lane.speed_limit = traffic_rules->speedLimit(lanelet).speedLimit.value();
    awmap_lane.lane_number =vehicle_graph->lefts(lanelet).size() + 1;
    awmap_lane.length = lane_distance;
    awmap_lane.width_limit = lanelet::geometry::distance(lanelet.rightBound().front(), lanelet.leftBound().front());
    awmap_lane.height_limit = 0;
    awmap_lane.width_limit = width_limit;
    // lanes.push_back(awmap_lane);
    lane_map[awmap_lane.lane_id] = awmap_lane;
  }

  //stop_line
  for (auto line : map->lineStringLayer)
  {
    if(line.attributes()[AttributeName::Type]==AttributeValueString::StopLine) {
      double min = 100;
      for(auto relation : waypoint_relations)
      {
        auto wp = waypoints_map.at(relation.waypoint_id);
        auto next_wp = waypoints_map.at(relation.next_waypoint_id);
        auto pt = waypoint_id_to_point_map.at(wp->waypoint_id);
        auto next_pt = waypoint_id_to_point_map.at(next_wp->waypoint_id);
        Point3d p1(pt.x,pt.y,pt.z);
        Point3d p2(next_pt.x, next_pt.y, next_pt.z);
        double epsilon = 0.1;

        if(amathutils::distanceFromSegment(line.front().x(),line.front().y(),
                                           line.back().x(),line.back().y(),
                                           pt.x,pt.y) <= epsilon )
        {
          wp->stop_line = 1;
          continue;
        }
        if(amathutils::distanceFromSegment(line.front().x(),line.front().y(),
                                           line.back().x(),line.back().y(),
                                           next_pt.x,next_pt.y) <= epsilon) {
          next_wp->stop_line = 1;
          continue;
        }
        geometry_msgs::Point l1_p1, l1_p2, l2_p1, l2_p2;
        l1_p1.x = line.front().x();
        l1_p1.y = line.front().y();
        l1_p2.x = line.back().x();
        l1_p2.y = line.back().y();
        l2_p1.x = pt.x;
        l2_p1.y = pt.y;
        l2_p2.x = next_pt.x;
        l2_p2.y = next_pt.y;

        if (amathutils::isIntersectLine(l1_p1, l1_p2, l2_p1, l2_p2)) {
          wp->stop_line = 1;
        }


      }
    }
  }
  for(auto item : waypoints_map)
  {
    waypoints.push_back(*(item.second));
  }

  //Get lane relation
  for (auto lanelet : vehicle_lanelets)
  {
    for( auto following : vehicle_graph->following(lanelet))
    {
      autoware_map_msgs::LaneRelation lane_relation;
      int awmap_lane_id = lane_id_table.at(lanelet.id());
      int awmap_following_lane_id = lane_id_table.at(following.id());
      lane_relation.lane_id = awmap_lane_id;
      lane_relation.next_lane_id = awmap_following_lane_id;
      lane_relation.blinker = 0;
      lane_relations.push_back(lane_relation);
    }
  }

  //Get lane change relation
  for (auto lanelet : vehicle_lanelets)
  {
    if( vehicle_graph->right(lanelet) ) {
      ConstLanelet right_lanelet = vehicle_graph->right(lanelet).value();
      autoware_map_msgs::LaneChangeRelation lane_change_relation;
      lane_change_relation.lane_id = lane_id_table.at(lanelet.id());
      lane_change_relation.next_lane_id = lane_id_table.at(right_lanelet.id());
      lane_change_relation.blinker = BLINKER_RIGHT;
      lane_change_relations.push_back(lane_change_relation);
    }
    if( vehicle_graph->left(lanelet)) {
      ConstLanelet left_lanelet = vehicle_graph->left(lanelet).value();
      autoware_map_msgs::LaneChangeRelation lane_change_relation;
      lane_change_relation.lane_id = lane_id_table.at(lanelet.id());
      lane_change_relation.next_lane_id = lane_id_table.at(left_lanelet.id());
      lane_change_relation.blinker = BLINKER_LEFT;
      lane_change_relations.push_back(lane_change_relation);
    }
  }

  //Get opposite lane relation
  for (auto lanelet : vehicle_lanelets)
  {
    auto closest_opposite_opt = getClosestOppositeLanes(map,vehicle_graph,lanelet,traffic_rules);
    if(closest_opposite_opt!=boost::none) {
      ConstLanelet closest_opposite = closest_opposite_opt.value();
      for(opposite_lane : vehicle_graph->besides(closest_opposite))
      {
        autoware_map_msgs::OppositeLaneRelation opposite_lane_relation;
        opposite_lane_relation.lane_id = lane_id_table.at(lanelet.id());
        opposite_lane_relation.opposite_lane_id = lane_id_table.at(opposite_lane.id());
        bool already_added = false;
        for(auto relation : opposite_lane_relations)
        {
          if(relation.lane_id == opposite_lane_relation.lane_id &&
             relation.opposite_lane_id == opposite_lane_relation.opposite_lane_id) {
            already_added =true;
          }
          if(relation.opposite_lane_id == opposite_lane_relation.lane_id &&
             relation.lane_id == opposite_lane_relation.opposite_lane_id) {
            already_added =true;
          }
        }
        if(already_added ==false) {
          opposite_lane_relations.push_back(opposite_lane_relation);
        }
      }
    }
  }

  //signal_light
  int signal_light_id = 1;
  int signal_id = 1;
  for (auto lanelet : vehicle_lanelets)
  {
    auto traffic_lights = lanelet.regulatoryElementsAs<TrafficLight>();
    for (auto light : traffic_lights)
    {
      for( auto line : light->trafficLights())
      {
        autoware_map_msgs::Signal awmap_signal;
        awmap_signal.signal_id = signal_id++;
        signals.push_back(awmap_signal);
        if(!line.lineString()) continue;
        auto bulbs = ConstHybridLineString3d(line.lineString().value());
        int color_count = 1;

        autoware_map_msgs::WaypointRelation relation;
        for(auto wp_relation : waypoint_relations)
        {
          if(wp_relation.next_waypoint_id == lane_map.at(lane_id_table.at(lanelet.id())).end_waypoint_id) {
            relation = wp_relation;
            break;
          }
        }

        auto wp = waypoints_map.at(relation.waypoint_id);
        auto next_wp = waypoints_map.at(relation.next_waypoint_id);
        auto pt = waypoint_id_to_point_map.at(wp->waypoint_id);
        auto next_pt = waypoint_id_to_point_map.at(next_wp->waypoint_id);
        double horizontal_angle = (M_PI /2 - atan2(next_pt.y - pt.y, next_pt.x - pt.x)) * 180.0/M_PI;
        horizontal_angle = 180 + horizontal_angle;


        for(auto bulb : bulbs)
        {
          autoware_map_msgs::Waypoint awmap_waypoint;
          autoware_map_msgs::Point awmap_point;
          awmap_point = convertPoint(bulb, projector);
          awmap_point.point_id = point_id++;
          points.push_back(awmap_point);

          //signal_light
          autoware_map_msgs::SignalLight awmap_signal_light;
          awmap_signal_light.signal_light_id = signal_light_id++;
          awmap_signal_light.point_id = awmap_point.point_id;
          awmap_signal_light.signal_id = awmap_signal.signal_id;
          awmap_signal_light.horizontal_angle = horizontal_angle;
          awmap_signal_light.vertical_angle = 90;
          awmap_signal_light.arrow_type = 0;           //normal
          if(color_count == 1) {
            awmap_signal_light.color_type = SIGNAL_RED;
          }else if (color_count == 2)
          {
            awmap_signal_light.color_type = SIGNAL_YELLOW;
          }
          else if (color_count == 3)
          {
            awmap_signal_light.color_type = SIGNAL_GREEN;
          }
          color_count++;
          signal_lights.push_back(awmap_signal_light);

          //lane signal relations
          autoware_map_msgs::LaneSignalLightRelation lane_signal_light_relation;
          lane_signal_light_relation.lane_id = lane_id_table.at(lanelet.id());
          lane_signal_light_relation.signal_light_id = awmap_signal_light.signal_light_id;
          lane_signal_light_relations.push_back(lane_signal_light_relation);
        }
        //waypoint signal relation
        autoware_map_msgs::WaypointSignalRelation waypoint_signal_relation;
        waypoint_signal_relation.signal_id = awmap_signal.signal_id;
        waypoint_signal_relation.waypoint_id = lane_map.at(lane_id_table.at(lanelet.id())).end_waypoint_id;
        waypoint_signal_relations.push_back(waypoint_signal_relation);
      }
    }
  }

  //stop_lines
  // for (auto lanelet : vehicle_lanelets)
  // {
  //     auto traffic_lights = lanelet.regulatoryElementsAs<TrafficLight>();
  //     for (auto light : traffic_lights)
  //     {
  //         if(!light->stopLine()) {
  //             continue;
  //         }
  //         ConstLineString3d stop_line = light->stopLine().value();
  //         for(auto &waypoint : waypoints)
  //         {
  //             auto point = waypoint_id_to_point_map.at(waypoint.waypoint_id);
  //             auto point3d = Point3d(point.point_id, point.x, point.y, point.z);
  //             if(geometry::distance(point3d, stop_line) == 0.0) {
  //                 waypoint.stop_line = 1;
  //             }
  //         }
  //     }
  // }

  //intersection
  int area_id = 1;
  std::unordered_map<unsigned long long int,bool> already_calculated;
  for (auto lanelet : vehicle_lanelets)
  {
    //continue if already calculated
    if(already_calculated.find(lanelet.id()) != already_calculated.end()) {
      continue;
    }
    auto conflicting_lanelets_or_areas = vehicle_graph->conflicting(lanelet);
    if(conflicting_lanelets_or_areas.size() == 0) continue;
    //convert to laneletArea -> lanelet
    std::vector<ConstLanelet> conflicting_lanelets;
    conflicting_lanelets.push_back(lanelet);
    already_calculated[lanelet.id()] = true;
    for ( auto conflicting_lanelet_area : conflicting_lanelets_or_areas)
    {
      if(!conflicting_lanelet_area.lanelet()) continue;
      ConstLanelet conflicting_lanelet = conflicting_lanelet_area.lanelet().value();
      if(already_calculated[conflicting_lanelet.id()]) continue;
      conflicting_lanelets.push_back(conflicting_lanelet);
      already_calculated[conflicting_lanelet.id()] = true;
    }
    //continue if it is only conflicting with area
    if(conflicting_lanelets.size() == 1) {
      continue;
    }
    //get all linked intersecting lanes
    std::stack<ConstLanelet> stack;
    //initial setup for stack
    for ( auto conflicting_lanelet : conflicting_lanelets)
    {
      stack.push(conflicting_lanelet);
    }
    while(!stack.empty()) {
      auto tmp_conflicting_lanelets_or_areas = vehicle_graph->conflicting(stack.top());
      stack.pop();

      for (auto la : tmp_conflicting_lanelets_or_areas)
      {
        if(!la.lanelet()) continue;
        ConstLanelet l = la.lanelet().value();
        if(!already_calculated[l.id()]) {
          stack.push(l);
          conflicting_lanelets.push_back(l);
          already_calculated[l.id()] = true;
        }
      }
    }
    //change lane information
    for ( auto conflicting_lanelet : conflicting_lanelets)
    {
      int id = lane_id_table.at(conflicting_lanelet.id());

      lane_map.at(id).num_of_lanes = 0;
      lane_map.at(id).lane_number = 0;
    }

    //calculate convex hull
    std::vector<BasicPoint2d> vertices;
    std::vector<BasicPoint2d> hull;
    for ( auto conflicting_lanelet : conflicting_lanelets)
    {
      for( auto pt : conflicting_lanelet.polygon3d())
      {
        vertices.push_back(utils::to2D(pt.basicPoint()));
      }
    }
    boost::geometry::convex_hull(vertices, hull);
    std::vector<int> ids;
    for( auto pt : hull)
    {
      auto pt3d = utils::to3D(pt);
      autoware_map_msgs::Point awmap_point;
      awmap_point = convertPoint(pt3d, projector);
      awmap_point.point_id = point_id++;
      if(USE_FIXED_HEIGHT) {
        awmap_point.z = FIXED_HEIGHT;
      }
      points.push_back(awmap_point);
      ids.push_back(awmap_point.point_id);
    }
    //intersection area
    autoware_map_msgs::Area awmap_area;
    awmap_area.area_id = area_id++;
    awmap_area.point_ids = ids;
    areas.push_back(awmap_area);

    //lane attribute relations
    autoware_map_msgs::LaneAttributeRelation lane_attribute_relation;
    lane_attribute_relation.lane_id = lane_id_table.at(lanelet.id());
    lane_attribute_relation.attribute_type = autoware_map_msgs::LaneAttributeRelation::INTERSECTION;
    lane_attribute_relation.area_id = awmap_area.area_id;
    lane_attribute_relations.push_back(lane_attribute_relation);
    for ( auto conflicting_lanelet : conflicting_lanelets)
    {
      lane_attribute_relation.lane_id = lane_id_table.at(conflicting_lanelet.id());
      lane_attribute_relations.push_back(lane_attribute_relation);
    }
  }

  //cross_walks
  std::unordered_map<long long int, int> cross_walk_id_conversion;
  for (auto lanelet : vehicle_lanelets)
  {
    auto cross_walk_lanelets = overall_graphs.conflictingInGraph(lanelet,1);
    for ( auto cross_walk_lanelet : cross_walk_lanelets)
    {
      int cross_walk_area_id;
      if(cross_walk_id_conversion.find(cross_walk_lanelet.id()) == cross_walk_id_conversion.end() )
      {
        std::vector<BasicPoint3d> vertices;
        std::vector<BasicPoint3d> hull;
        for( auto pt : cross_walk_lanelet.polygon3d())
        {
          vertices.push_back(pt.basicPoint());
        }
        boost::geometry::convex_hull(vertices, hull);
        //add points
        std::vector<int> ids;
        for( auto pt : hull)
        {
          // auto pt3d = utils::to3D(pt);
          autoware_map_msgs::Point awmap_point;
          awmap_point = convertPoint(pt, projector);
          awmap_point.point_id = point_id++;
          if(USE_FIXED_HEIGHT) {
            awmap_point.z = FIXED_HEIGHT;
          }
          points.push_back(awmap_point);
          ids.push_back(awmap_point.point_id);
        }
        //cross walk area
        autoware_map_msgs::Area awmap_area;
        awmap_area.area_id = area_id++;
        awmap_area.point_ids = ids;
        areas.push_back(awmap_area);
        cross_walk_area_id = awmap_area.area_id;
        //lane attribute relations
        cross_walk_id_conversion[cross_walk_lanelet.id()] = awmap_area.area_id;

      }else{
        cross_walk_area_id = cross_walk_id_conversion.at(cross_walk_lanelet.id());
      }

      autoware_map_msgs::LaneAttributeRelation lane_attribute_relation;
      lane_attribute_relation.lane_id = lane_id_table.at(lanelet.id());
      lane_attribute_relation.attribute_type = autoware_map_msgs::LaneAttributeRelation::CROSS_WALK;
      lane_attribute_relation.area_id = cross_walk_area_id;
      lane_attribute_relations.push_back(lane_attribute_relation);
    }
  }

  for(auto item : lane_map)
  {
    lanes.push_back(item.second);
  }


  //preparation to remove overlapping waypoint
  std::unordered_map<int, bool> wp_checklist;

  std::vector<int> start_end_points;
  for(auto lane : lanes)
  {
    start_end_points.push_back(lane.start_waypoint_id);
    start_end_points.push_back(lane.end_waypoint_id);
  }

  double epsilon = 1e-6;
  for (int i = 0; i < start_end_points.size(); i++)
  // for (auto i_id : start_end_points)
  {
    int i_id = start_end_points.at(i);
    if(wp_checklist.find(i_id) != wp_checklist.end()) continue;
    std::vector<int> same_points;
    same_points.push_back(i_id);
    for(int j = i+1; j < start_end_points.size(); j++)
    // for(auto j_id : start_end_points)
    {
      int j_id = start_end_points.at(j);
      if (i_id == j_id || wp_checklist.find(i_id) != wp_checklist.end() ) continue;
      auto i_point = waypoint_id_to_point_map.at(i_id);
      auto j_point = waypoint_id_to_point_map.at(j_id);
      auto i_point3d = Point3d(i_point.x, i_point.y, i_point.z);
      auto j_point3d = Point3d(j_point.x, j_point.y, j_point.z);
      if(geometry::distance2d(i_point3d, j_point3d) < epsilon) {
        same_points.push_back(j_id);
      }
    }
    int smallest_id = i_id;
    for(auto id : same_points)
    {
      if(smallest_id > id) smallest_id = id;
    }
    for(auto id : same_points)
    {
      wp_checklist[id] = true;
      if(smallest_id == id) continue;
      waypoint_id_correction[id] = smallest_id;
    }
  }

  for(auto &lane : lanes)
  {
    lane.start_waypoint_id = getNewId(lane.start_waypoint_id, waypoint_id_correction);
    lane.end_waypoint_id = getNewId(lane.end_waypoint_id, waypoint_id_correction);
  }
  for(auto &relation : waypoint_lane_relations)
  {
    relation.waypoint_id = getNewId(relation.waypoint_id, waypoint_id_correction);
  }
  for(auto &relation : waypoint_relations)
  {
    relation.waypoint_id = getNewId(relation.waypoint_id, waypoint_id_correction);
    relation.next_waypoint_id = getNewId(relation.next_waypoint_id, waypoint_id_correction);
  }
  for(auto &relation : waypoint_signal_relations)
  {
    relation.waypoint_id = getNewId(relation.waypoint_id, waypoint_id_correction);
  }

  //remove overlapping waypoints
  //i.e. end of lane and beginning of following lane
  waypoints.erase(std::remove_if(waypoints.begin(), waypoints.end(),
                                 [&](autoware_map_msgs::Waypoint wp){
                                   auto iter = waypoint_id_correction.find(wp.waypoint_id);
                                   return iter != waypoint_id_correction.end();
                                 }));

  for( auto &point : points)
  {
    fixPointCoordinate(point);
  }
}
