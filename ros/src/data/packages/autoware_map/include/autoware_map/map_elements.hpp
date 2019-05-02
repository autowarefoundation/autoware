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

/**
 * @file map_element.hpp
 * @brief Contains wrappers for autoware_map_msgs
 * @author Ryohsuke Mitsudome
 * @date 2019/03/28
 */

#ifndef __AUTOWARE_MAP_ELEMENTS_HPP__
#define __AUTOWARE_MAP_ELEMENTS_HPP__
#include <memory>
#include <ros/ros.h>
#include <autoware_map_msgs/LaneArray.h>
#include <autoware_map_msgs/LaneAttributeRelationArray.h>
#include <autoware_map_msgs/LaneRelationArray.h>
#include <autoware_map_msgs/LaneSignalLightRelationArray.h>
#include <autoware_map_msgs/LaneChangeRelationArray.h>
#include <autoware_map_msgs/OppositeLaneRelationArray.h>
#include <autoware_map_msgs/RouteArray.h>
#include <autoware_map_msgs/PointArray.h>
#include <autoware_map_msgs/AreaArray.h>
#include <autoware_map_msgs/SignalArray.h>
#include <autoware_map_msgs/SignalLightArray.h>
#include <autoware_map_msgs/WayareaArray.h>
#include <autoware_map_msgs/WaypointArray.h>
#include <autoware_map_msgs/WaypointLaneRelationArray.h>
#include <autoware_map_msgs/WaypointRelationArray.h>
#include <autoware_map_msgs/WaypointSignalRelationArray.h>
#include <unordered_map>

template<typename T>
using shared_vector = std::vector<std::shared_ptr<T> >;
template<typename T>
using weak_vector = std::vector<std::weak_ptr<T> >;
template<typename U,typename T>
using shared_unordered_map = std::unordered_map<U,std::shared_ptr<T> >;

template<typename T>
std::vector<T> weakToStdVector(weak_vector<T> weak_vec)
{
  std::vector<T> vec;
  for (auto weak : weak_vec)
  {
    vec.push_back(*(weak.lock()));
  }
  return vec;
}

template<typename T>
shared_vector<T> weakToSharedVector(weak_vector<T> weak_vec)
{
  shared_vector<T> shared_vec;
  for (auto weak : weak_vec)
  {
    shared_vec.push_back(weak.lock());
  }
  return shared_vec;
}

namespace autoware_map {
using AreaMsg = autoware_map_msgs::Area;
using LaneMsg = autoware_map_msgs::Lane;
using LaneAttributeRelationMsg = autoware_map_msgs::LaneAttributeRelation;
using LaneChangeRelationMsg = autoware_map_msgs::LaneChangeRelation;
using LaneRelationMsg = autoware_map_msgs::LaneRelation;
using LaneSignalLightRelationMsg = autoware_map_msgs::LaneSignalLightRelation;
using OppositeLaneRelationMsg = autoware_map_msgs::OppositeLaneRelation;
using PointMsg = autoware_map_msgs::Point;
using SignalLightMsg = autoware_map_msgs::SignalLight;
using SignalMsg = autoware_map_msgs::Signal;
using WayareaMsg = autoware_map_msgs::Wayarea;
using WaypointMsg = autoware_map_msgs::Waypoint;
using WaypointLaneRelationMsg = autoware_map_msgs::WaypointLaneRelation;
using WaypointRelationMsg = autoware_map_msgs::WaypointRelation;
using WaypointSignalRelationMsg = autoware_map_msgs::WaypointSignalRelation;

class Area;
class Lane;
class LaneAttributeRelation;
class LaneChangeRelation;
class LaneRelation;
class LaneSignalLightRelation;
class OppositeLaneRelation;
class Point;
class SignalLight;
class Signal;
class Wayarea;
class Waypoint;
class WaypointLaneRelation;
class WaypointRelation;
class WaypointSignalRelation;


class WaypointLaneRelation : public WaypointLaneRelationMsg {
public:
  std::weak_ptr<Waypoint> waypoint;
  std::weak_ptr<Lane> lane;
  WaypointLaneRelation()
  {
  }
  WaypointLaneRelation(autoware_map_msgs::WaypointLaneRelation relation)
  {
    waypoint_id = relation.waypoint_id;
    lane_id = relation.lane_id;
  }
  std::shared_ptr<Waypoint> getWaypointPtr()
  {
    return waypoint.lock();
  }
  std::shared_ptr<Lane> getLane()
  {
    return lane.lock();
  }
};

class WaypointRelation : public WaypointRelationMsg {
public:
  std::weak_ptr<Waypoint> waypoint;
  std::weak_ptr<Waypoint> next_waypoint;
  WaypointRelation()
  {
  }
  WaypointRelation(autoware_map_msgs::WaypointRelation relation)
  {
    waypoint_id = relation.waypoint_id;
    next_waypoint_id = relation.next_waypoint_id;
    yaw = relation.yaw;
    blinker = relation.blinker;
    distance =relation.distance;
  }
  std::shared_ptr<Waypoint> getWaypointPtr()
  {
    return waypoint.lock();
  }
  std::shared_ptr<Waypoint> getNextWaypointPtr()
  {
    return next_waypoint.lock();
  }
};

class WaypointSignalRelation : public WaypointSignalRelationMsg {
public:
  std::weak_ptr<Waypoint> waypoint;
  std::weak_ptr<Signal> signal;
  WaypointSignalRelation(WaypointSignalRelationMsg relation)
  {
    waypoint_id = relation.waypoint_id;
    signal_id = relation.signal_id;
  }
  std::shared_ptr<Waypoint> getWaypoint()
  {
    return waypoint.lock();
  }
  std::shared_ptr<Signal> getSignal()
  {
    return signal.lock();
  }
};

class LaneAttributeRelation : public LaneAttributeRelationMsg {
public:
  std::weak_ptr<Lane> lane;
  std::weak_ptr<Area> area;

  LaneAttributeRelation(LaneAttributeRelationMsg relation)
  {
    this->lane_id = relation.lane_id;
    this->attribute_type = relation.attribute_type;
    this->area_id = relation.area_id;
  }
  std::shared_ptr<Lane> getLane()
  {
    return lane.lock();
  }
  std::shared_ptr<Area> getArea()
  {
    if(area_id == 0) {
      ROS_ERROR_STREAM( "no area available!!" << std::endl );
      exit(1);
    }
    return area.lock();
  }
};

class LaneChangeRelation : public LaneChangeRelationMsg {
public:
  std::weak_ptr<Lane> lane;
  std::weak_ptr<Lane> next_lane;

  LaneChangeRelation(LaneChangeRelationMsg relation)
  {
    lane_id = relation.lane_id;
    next_lane_id = relation.next_lane_id;
    blinker = blinker;
  }
  std::shared_ptr<Lane> getLane()
  {
    return lane.lock();
  }
  std::shared_ptr<Lane> getNextLane()
  {
    return next_lane.lock();
  }

};

class LaneRelation : public LaneRelationMsg {
public:
  std::weak_ptr<Lane> lane;
  std::weak_ptr<Lane> next_lane;

  LaneRelation(LaneRelationMsg relation)
  {
    lane_id = relation.lane_id;
    next_lane_id = relation.next_lane_id;
    blinker = relation.blinker;
  }
  std::shared_ptr<Lane> getLane()
  {
    return lane.lock();
  }
  std::shared_ptr<Lane> getNextLane()
  {
    return next_lane.lock();
  }

};

class LaneSignalLightRelation : public LaneSignalLightRelationMsg {
public:
  std::weak_ptr<Lane> lane;
  std::weak_ptr<SignalLight> signal_light;

  LaneSignalLightRelation(LaneSignalLightRelationMsg relation)
  {
    lane_id = relation.lane_id;
    signal_light_id = relation.signal_light_id;
  }
  std::shared_ptr<Lane> getLane(){ return lane.lock(); }
  std::shared_ptr<SignalLight> getSignalLight(){ return signal_light.lock(); }

};

class OppositeLaneRelation : public OppositeLaneRelationMsg {
public:
  std::weak_ptr<Lane> lane;
  std::weak_ptr<Lane> opposite_lane;

  OppositeLaneRelation(OppositeLaneRelationMsg relation)
  {
    lane_id = relation.lane_id;
    opposite_lane_id = relation.opposite_lane_id;
  }
  std::shared_ptr<Lane> getLane(){ return lane.lock(); }
  std::shared_ptr<Lane> getOppositeLane(){ return opposite_lane.lock(); }

};


class Area : public AreaMsg {
public:
  int id;
  weak_vector<Point> points;
  Area(AreaMsg area)
  {
    id = area.area_id;
    area_id = area.area_id;
    point_ids = area.point_ids;
  }
  shared_vector<Point> getPoints()
  {
    return weakToSharedVector(points);
  }
};

class Lane : public LaneMsg
{
public:
  int id;
  weak_vector<Waypoint> waypoints;

  weak_vector<LaneAttributeRelation> lane_attribute_relations;
  weak_vector<LaneChangeRelation> lane_change_relations;

  weak_vector<LaneRelation> lane_relations;
  weak_vector<LaneSignalLightRelation> signal_light_relations;
  weak_vector<OppositeLaneRelation> opposite_lane_relations;
  weak_vector<WaypointLaneRelation> waypoint_lane_relations;
  std::weak_ptr<Waypoint> start_waypoint;
  std::weak_ptr<Waypoint> end_waypoint;

  shared_vector<Waypoint> getWaypoints()
  {
    return weakToSharedVector(waypoints);
  }

  std::shared_ptr<Waypoint> getStartWaypoint()
  {
    return start_waypoint.lock();
  }
  std::shared_ptr<Waypoint> getEndWaypoint()
  {
    return end_waypoint.lock();
  }

  Lane(LaneMsg lane)
  {
    id = lane.lane_id;
    lane_id = lane.lane_id;
    start_waypoint_id = lane.start_waypoint_id;
    end_waypoint_id = lane.end_waypoint_id;
    lane_number = lane.lane_number;
    num_of_lanes = lane.num_of_lanes;
    speed_limit = lane.speed_limit;
    length = lane.length;
    width_limit = lane.width_limit;
    height_limit = lane.height_limit;
    weight_limit = lane.weight_limit;
  }
  Lane& operator=(LaneMsg lane)
  {
    this->id = lane.lane_id;
    this->lane_id = lane.lane_id;
    this->start_waypoint_id = lane.start_waypoint_id;
    this->end_waypoint_id = lane.end_waypoint_id;
    this->lane_number = lane.lane_number;
    this->num_of_lanes = lane.num_of_lanes;
    this->speed_limit = lane.speed_limit;
    this->length = lane.length;
    this->width_limit = lane.width_limit;
    this->height_limit = lane.height_limit;
    this->weight_limit = lane.weight_limit;
    return *this;
  }
};


class Point : public autoware_map_msgs::Point {
public:
  int id;
  Point(PointMsg point)
  {
    id = point.point_id;
    point_id = point.point_id;
    x = point.x;
    y = point.y;
    z = point.z;
    mgrs = point.mgrs;
    epsg = point.epsg;
    pcd = point.pcd;
    lat = point.lat;
    lng = point.lng;
  }
};

class Signal : public SignalMsg {
public:
  int id;
  weak_vector<SignalLight> signal_lights;
  weak_vector<WaypointSignalRelation> waypoint_signal_relations;

  Signal(autoware_map_msgs::Signal signal)
  {
    signal_id = signal.signal_id;
    id = signal.signal_id;
  }

};

class SignalLight : public SignalLightMsg {
public:
  int id;
  std::weak_ptr<Signal> signal;
  weak_vector<LaneSignalLightRelation> signal_light_relations;

  SignalLight(autoware_map_msgs::SignalLight light)
  {
    id = light.signal_light_id;
    signal_light_id = light.signal_light_id;
    signal_id = light.signal_id;
    point_id = light.point_id;
    horizontal_angle = light.horizontal_angle;
    vertical_angle = light.vertical_angle;
    color_type = light.color_type;
    arrow_type = light.arrow_type;
  }
};

class Wayarea : public WayareaMsg {
public:
  int id;
  std::weak_ptr<Area> area;

  Wayarea(autoware_map_msgs::Wayarea wayarea)
  {
    id = wayarea.wayarea_id;
    wayarea_id = wayarea.wayarea_id;
    area_id = wayarea.area_id;
  }
};

class Waypoint : public WaypointMsg {
public:
  int id;
  std::weak_ptr<Point> point;

  weak_vector<WaypointLaneRelation> waypoint_lane_relations;
  weak_vector<WaypointRelation> waypoint_relations;
  weak_vector<WaypointSignalRelation> waypoint_signal_relations;

  Waypoint( autoware_map_msgs::Waypoint waypoint)
  {
    id = waypoint.waypoint_id;
    waypoint_id = waypoint.waypoint_id;
    point_id = waypoint.point_id;
    velocity= waypoint.velocity;
    stop_line= waypoint.stop_line;
    left_width= waypoint.left_width;
    right_width= waypoint.right_width;
    height= waypoint.height;
  }

  std::shared_ptr<Point> getPointPtr()
  {
    return point.lock();
  }

  shared_vector<WaypointRelation> getWaypointRelations()
  {
    return weakToSharedVector( waypoint_relations);
  }
  std::vector<WaypointRelation> getStdWaypointRelations()
  {
    return weakToStdVector( waypoint_relations);
  }


  shared_vector<Waypoint> getNextWaypoints()
  {
    shared_vector<Waypoint> waypoints;
    for(auto relation : waypoint_relations)
    {
      if(relation.lock()->waypoint_id == id) {
        waypoints.push_back(relation.lock()->next_waypoint.lock());
      }
    }
    return waypoints;
  }

  bool getWaypointRelation(int lane_id, WaypointRelationMsg &relation_msg)
  {
    for (auto relation : getWaypointRelations())
    {
      if(relation->getNextWaypointPtr()->belongLane(lane_id)) {
        relation_msg = *relation;
        return true;
      }
    }
    return false;
  }

  bool belongLane(int lane_id)
  {
    for(auto relation : waypoint_lane_relations)
    {
      if(relation.lock()->lane_id == lane_id) {
        return true;
      }
    }
    return false;
  }
  shared_vector<Lane> getBelongingLanes()
  {
    shared_vector<Lane> belonging_lanes;
    for(const auto &relation : waypoint_lane_relations)
    {
      belonging_lanes.push_back(relation.lock()->lane.lock());
    }
    return belonging_lanes;
  }
  std::shared_ptr<Waypoint> getNextWaypoint(int lane_id)
  {
    shared_vector<Waypoint> waypoints = getNextWaypoints();
    for(auto waypoint : waypoints)
    {
      if(waypoint->belongLane(lane_id)) {
        return waypoint;
      }
    }
    //return itself if there is not next waypoint
    ROS_WARN_STREAM("failed to find next waypoint from waypoint " << id << " in lane " << lane_id);
    return std::make_shared<Waypoint>(*this);
  }
};

}

#endif
