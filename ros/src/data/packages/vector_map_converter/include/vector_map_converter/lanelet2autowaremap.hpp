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
#ifndef __LANELET2AUTOWAREMAP_HPP__
#define __LANELET2AUTOWAREMAP_HPP__

#include <typeinfo>
#include <lanelet2_core/primitives/Lanelet.h>

#include <lanelet2_core/geometry/Area.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>

#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/multi/geometries/register/multi_point.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

#include <ros/ros.h>
#include <autoware_map/util.h>
#include <autoware_map/map_handler.hpp>
#include <vector_map_msgs/Area.h>
#include <vector_map_msgs/CrossRoad.h>
#include <vector_map_msgs/CrossWalk.h>
#include <vector_map_msgs/DTLane.h>
#include <vector_map_msgs/Lane.h>
#include <vector_map_msgs/Line.h>
#include <vector_map_msgs/Node.h>
#include <vector_map_msgs/Point.h>
#include <vector_map_msgs/Pole.h>
#include <vector_map_msgs/RoadSign.h>
#include <vector_map_msgs/Signal.h>
#include <vector_map_msgs/StopLine.h>
#include <vector_map_msgs/UtilityPole.h>
#include <vector_map_msgs/Vector.h>
#include <vector_map_msgs/WayArea.h>
#include <vector_map_msgs/WhiteLine.h>

#define USE_FIXED_HEIGHT 1
#define FIXED_HEIGHT 110

void convertLanelet2AutowareMap(lanelet::LaneletMapPtr map,
                                lanelet::projection::UtmProjector projector,
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
                                std::vector<autoware_map_msgs::WaypointSignalRelation> &waypoint_signal_relations);

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
                          std::vector<autoware_map_msgs::WaypointSignalRelation> &waypoint_signal_relations);

autoware_map_msgs::Point convertPoint(const lanelet::BasicPoint3d lanelet_point, const lanelet::projection::UtmProjector &projector);
void fixPointCoordinate(autoware_map_msgs::Point &point);

#endif
