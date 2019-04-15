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
#ifndef __LANELET2__LANELET2VECTORMAP_HPP__
#define __LANELET2__LANELET2VECTORMAP_HPP__
#include <vector_map_converter/autoware2vectormap.hpp>
#include <vector_map_converter/lanelet2autowaremap.hpp>

void complementVectorMap( lanelet::LaneletMapPtr map,
                          lanelet::projection::UtmProjector projector,
                          std::vector<vector_map_msgs::Line> &vmap_lines,
                          std::vector<vector_map_msgs::Point> &vmap_points,
                          std::vector<vector_map_msgs::Pole> &vmap_dummy_poles,
                          std::vector<vector_map_msgs::RoadSign> &vmap_road_signs,
                          std::vector<vector_map_msgs::StopLine> &vmap_stop_lines,
                          std::vector<vector_map_msgs::Vector> &vmap_vectors,
                          std::vector<vector_map_msgs::WhiteLine> &vmap_white_lines
                          );

void convertLineString2WhiteLine(lanelet::ConstLineString3d line_string,
                                 lanelet::projection::UtmProjector projector,
                                 std::vector<vector_map_msgs::Point> &vmap_points,
                                 std::vector<vector_map_msgs::Line> &vmap_lines,
                                 std::vector<vector_map_msgs::WhiteLine> &vmap_white_lines,
                                 int &point_id, int &line_id, int &white_line_id );

void createWhitelines(const std::vector<lanelet::ConstLanelet> &lanelets,
                      lanelet::projection::UtmProjector projector,
                      std::vector<vector_map_msgs::Point> &vmap_points,
                      std::vector<vector_map_msgs::Line> &vmap_lines,
                      std::vector<vector_map_msgs::WhiteLine> &vmap_white_lines
                      );

#endif
