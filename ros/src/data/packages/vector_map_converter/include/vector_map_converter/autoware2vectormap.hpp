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

#ifndef __AUTOWARE2VECTORMAP_HPP__
#define __AUTOWARE2VECTORMAP_HPP__

#include <autoware_map/map_handler.hpp>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <fstream>
#include <autoware_map/util.h>
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

struct WaypointWithYaw{
    autoware_map_msgs::Waypoint waypoint;
    autoware_map_msgs::Point point;
    autoware_map_msgs::Point left_point;
    autoware_map_msgs::Point right_point;
    std::vector<double> yaws;
    double yaw_avg;
};

void getVectorMapMsgs(  const autoware_map::AutowareMapHandler &map_handler,
                        std::vector<vector_map_msgs::Area> &vmap_areas,
                        std::vector<vector_map_msgs::CrossRoad> &vmap_cross_roads,
                        std::vector<vector_map_msgs::CrossWalk> &vmap_cross_walks,
                        std::vector<vector_map_msgs::DTLane> &vmap_dtlanes,
                        std::vector<vector_map_msgs::Lane> &vmap_lanes,
                        std::vector<vector_map_msgs::Line> &vmap_lines,
                        std::vector<vector_map_msgs::Node> &vmap_nodes,
                        std::vector<vector_map_msgs::Point> &vmap_points,
                        std::vector<vector_map_msgs::Pole> &vmap_dummy_poles,
                        std::vector<vector_map_msgs::RoadSign> &vmap_road_signs,
                        std::vector<vector_map_msgs::Signal> &vmap_signals,
                        std::vector<vector_map_msgs::StopLine> &vmap_stop_lines,
                        std::vector<vector_map_msgs::UtilityPole> &vmap_dummy_utility_poles,
                        std::vector<vector_map_msgs::Vector> &vmap_vectors,
                        std::vector<vector_map_msgs::WayArea> &vmap_way_areas,
                        std::vector<vector_map_msgs::WhiteLine> &vmap_white_lines);
void writeVectorMapMsgs(const std::string output_dir,
                        const std::vector<vector_map_msgs::Area> &vmap_areas,
                        const std::vector<vector_map_msgs::CrossRoad> &vmap_cross_roads,
                        const std::vector<vector_map_msgs::CrossWalk> &vmap_cross_walks,
                        const std::vector<vector_map_msgs::DTLane> &vmap_dtlanes,
                        const std::vector<vector_map_msgs::Lane> &vmap_lanes,
                        const std::vector<vector_map_msgs::Line> &vmap_lines,
                        const std::vector<vector_map_msgs::Node> &vmap_nodes,
                        const std::vector<vector_map_msgs::Point> &vmap_points,
                        const std::vector<vector_map_msgs::Pole> &vmap_poles,
                        const std::vector<vector_map_msgs::RoadSign> &vmap_road_signs,
                        const std::vector<vector_map_msgs::Signal> &vmap_signals,
                        const std::vector<vector_map_msgs::StopLine> &vmap_stop_lines,
                        const std::vector<vector_map_msgs::UtilityPole> &vmap_utility_poles,
                        const std::vector<vector_map_msgs::Vector> &vmap_vectors,
                        const std::vector<vector_map_msgs::WayArea> &vmap_way_areas,
                        const std::vector<vector_map_msgs::WhiteLine> &vmap_white_lines
                        );
void createAreas(const autoware_map::AutowareMapHandler &map_handler, std::vector<vector_map_msgs::Area> &vmap_areas, std::vector<vector_map_msgs::Line> &vmap_lines);
void createPoints(const autoware_map::AutowareMapHandler &awmap, std::vector<vector_map_msgs::Point> &vmap_points);
void createNodes(const autoware_map::AutowareMapHandler &awmap, std::vector<vector_map_msgs::Node> &vmap_nodes);
void createDTLanes(const autoware_map::AutowareMapHandler &awmap,
                   std::vector<vector_map_msgs::DTLane> &vmap_dtlanes,
                   std::vector<vector_map_msgs::Lane> &vmap_lanes);
void createCrossRoads(const autoware_map::AutowareMapHandler &awmap, std::vector<vector_map_msgs::CrossRoad> &vmap_cross_roads);
void createCrossWalks(const autoware_map::AutowareMapHandler &awmap, std::vector<vector_map_msgs::CrossWalk> &vmap_cross_walks,
                      std::vector<vector_map_msgs::Area> &vmap_areas, std::vector<vector_map_msgs::Line> &vmap_lines,
                      std::vector<vector_map_msgs::Point> &vmap_points);
void createSignals( const autoware_map::AutowareMapHandler &awmap,
                    std::vector<vector_map_msgs::Signal> &vmap_signals,
                    std::vector<vector_map_msgs::Vector> &vmap_vectors,
                    std::vector<vector_map_msgs::Pole> &vmap_dummy_poles);
void createStopLines( const autoware_map::AutowareMapHandler &awmap,
                      std::vector<vector_map_msgs::Line> &vmap_lines,
                      std::vector<vector_map_msgs::Point> &vmap_points,
                      std::vector<vector_map_msgs::StopLine> &vmap_stop_lines,
                      std::vector<vector_map_msgs::RoadSign> &vmap_road_signs,
                      std::vector<vector_map_msgs::Vector> &vmap_vectors,
                      std::vector<vector_map_msgs::Pole> &vmap_poles);
void createWayAreas(const autoware_map::AutowareMapHandler &awmap, std::vector<vector_map_msgs::WayArea> &vmap_way_areas);
void createWayAreasFromLanes(const autoware_map::AutowareMapHandler &awmap,
                             std::vector<vector_map_msgs::WayArea> &vmap_way_areas,
                             std::vector<vector_map_msgs::Area> &vmap_areas,
                             std::vector<vector_map_msgs::Line> &vmap_lines,
                             std::vector<vector_map_msgs::Point> &vmap_points );
void createWhitelines(const autoware_map::AutowareMapHandler &awmap,
                      std::vector<vector_map_msgs::Point> &vmap_points,
                      std::vector<vector_map_msgs::Line> &vmap_lines,
                      std::vector<vector_map_msgs::WhiteLine> &vmap_white_lines);
void createDummyUtilityPoles(const std::vector<vector_map_msgs::Pole> vmap_poles,
                             std::vector<vector_map_msgs::UtilityPole> &vmap_utility_poles);
void convertPoint(vector_map_msgs::Point &vmap_point,const autoware_map_msgs::Point awmap_point);

//keep angles within (M_PI, -M_PI]
double addAngles(double angle1, double angle2);
double getAngleAverage(std::vector<double> angles);
double angleAverage(double angle1, double angle2);
double convertDecimalToDDMMSS(const double decimal);
void getMinMax(autoware_map_msgs::Point &min, autoware_map_msgs::Point &max, const std::vector<autoware_map_msgs::Point>points);
//determine whether point lies within an area by counting winding number
bool isWithinArea(double x, double y, const std::vector<autoware_map_msgs::Point> vertices);
bool getIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double &intersect_x, double &intersect_y );
int convertESPGToRef(int epsg);
int getMaxId(std::vector<vector_map_msgs::Point> points);
int getMaxId(std::vector<vector_map_msgs::Line> lines);
int getMaxId(std::vector<vector_map_msgs::StopLine> stop_lines);
int getMaxId(std::vector<vector_map_msgs::RoadSign> signs);
int getMaxId(std::vector<vector_map_msgs::Area> areas);
int getMaxId(const std::vector<vector_map_msgs::Vector> vectors);
int getMaxId(const std::vector<vector_map_msgs::Pole> poles);
int getMaxId(const std::vector<vector_map_msgs::WhiteLine> white_lines);
int getMaxId(const std::vector<vector_map_msgs::WayArea> wayareas);

#endif
