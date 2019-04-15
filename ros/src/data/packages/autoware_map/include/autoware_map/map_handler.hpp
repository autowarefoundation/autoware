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

#ifndef __AUTOWARE_MAP_HANDLER_HPP__
#define __AUTOWARE_MAP_HANDLER_HPP__

#include <autoware_map/map_elements.hpp>
namespace autoware_map {

using category_t = uint64_t;

enum Category : category_t
{
  NONE = 0LLU,

  AREA = 1LLU << 0,
  LANE = 1LLU << 1,
  LANE_ATTRIBUTE_RELATION = 1LLU << 2,
  LANE_RELATION = 1LLU << 3,
  LANE_SIGNAL_LIGHT_RELATION = 1LLU << 4,
  LANE_CHANGE_RELATION = 1LLU << 5,
  OPPOSITE_LANE_RELATION = 1LLU << 6,
  POINT = 1LLU << 7,
  SIGNAL = 1LLU << 8,
  SIGNAL_LIGHT = 1LLU << 9,
  WAYAREA = 1LLU << 10,
  WAYPOINT = 1LLU << 11,
  WAYPOINT_LANE_RELATION = 1LLU << 12,
  WAYPOINT_RELATION = 1LLU << 13,
  WAYPOINT_SIGNAL_RELATION = 1LLU << 14,

  ALL = (1LLU << 32) - 1
};

template <class T>
using Filter = std::function<bool(const T&)>;

class AutowareMapHandler {
  //objects
  shared_unordered_map<int, Area> areas_;
  shared_unordered_map<int, Lane> lanes_;
  shared_unordered_map<int, Point> points_;
  shared_unordered_map<int, Signal> signals_;
  shared_unordered_map<int, SignalLight> signal_lights_;
  shared_unordered_map<int, Wayarea> wayareas_;
  shared_unordered_map<int, Waypoint> waypoints_;

  //relations
  shared_vector<LaneAttributeRelation> lane_attribute_relations_;
  shared_vector<LaneChangeRelation> lane_change_relations_;
  shared_vector<LaneRelation> lane_relations_;
  shared_vector<LaneSignalLightRelation> lane_signal_light_relations_;
  shared_vector<OppositeLaneRelation> opposite_lane_relations_;
  shared_vector<WaypointLaneRelation> waypoint_lane_relations_;
  shared_vector<WaypointRelation> waypoint_relations_;
  shared_vector<WaypointSignalRelation> waypoint_signal_relations_;

public:
  AutowareMapHandler(){};
  void registerSubscriber(ros::NodeHandle& nh, category_t category);
  void subscribe(ros::NodeHandle& nh, category_t category);
  void subscribe(ros::NodeHandle& nh, category_t category, const ros::Duration& timeout);
  void subscribe(ros::NodeHandle& nh, category_t category, const size_t max_retries);


  void setFromAutowareMapMsgs(const std::vector<autoware_map_msgs::Area> areas,
                              const std::vector<autoware_map_msgs::Lane> lanes,
                              const std::vector<autoware_map_msgs::LaneAttributeRelation> lane_attribute_relations,
                              const std::vector<autoware_map_msgs::LaneChangeRelation> lane_change_relations,
                              const std::vector<autoware_map_msgs::LaneRelation> lane_relations,
                              const std::vector<autoware_map_msgs::LaneSignalLightRelation> lane_signal_light_relations,
                              const std::vector<autoware_map_msgs::OppositeLaneRelation> opposite_lane_relations,
                              const std::vector<autoware_map_msgs::Point> points,
                              const std::vector<autoware_map_msgs::Signal> signals,
                              const std::vector<autoware_map_msgs::SignalLight> signal_lights,
                              const std::vector<autoware_map_msgs::Wayarea> wayareas,
                              const std::vector<autoware_map_msgs::Waypoint> waypoints,
                              const std::vector<autoware_map_msgs::WaypointLaneRelation> waypoint_lane_relations,
                              const std::vector<autoware_map_msgs::WaypointRelation> waypoint_relations,
                              const std::vector<autoware_map_msgs::WaypointSignalRelation> waypoint_signal_relations
                              );
  void resolveRelations();

  template<typename T>
  T findById(const int id) const;

  std::vector<Area> findByFilter(const Filter<Area>& filter) const;
  std::vector<Lane> findByFilter(const Filter<Lane>& filter) const;
  std::vector<Point> findByFilter(const Filter<Point>& filter) const;
  std::vector<Signal> findByFilter(const Filter<Signal>& filter) const;
  std::vector<SignalLight> findByFilter(const Filter<SignalLight>& filter) const;
  std::vector<Wayarea> findByFilter(const Filter<Wayarea>& filter) const;
  std::vector<Waypoint> findByFilter(const Filter<Waypoint>& filter) const;
  std::vector<LaneAttributeRelation> findByFilter(const Filter<LaneAttributeRelation>& filter) const;
  std::vector<LaneChangeRelation> findByFilter(const Filter<LaneChangeRelation>& filter) const;
  std::vector<LaneRelation> findByFilter(const Filter<LaneRelation>& filter) const;
  std::vector<LaneSignalLightRelation> findByFilter(const Filter<LaneSignalLightRelation>& filter) const;
  std::vector<OppositeLaneRelation> findByFilter(const Filter<OppositeLaneRelation>& filter) const;
  std::vector<WaypointLaneRelation> findByFilter(const Filter<WaypointLaneRelation>& filter) const;
  std::vector<WaypointRelation> findByFilter(const Filter<WaypointRelation>& filter) const;
  std::vector<WaypointSignalRelation> findByFilter(const Filter<WaypointSignalRelation>& filter) const;

};
}


#endif
