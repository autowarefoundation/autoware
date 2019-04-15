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
#include <vector_map_converter/lanelet2vectormap.hpp>
#include <vector_map_converter/autoware2vectormap.hpp>
#include <vector_map_converter/lanelet2autowaremap.hpp>
using namespace lanelet;
void complementVectorMap(LaneletMapPtr map,
                         projection::UtmProjector projector,
                         std::vector<vector_map_msgs::Line> &vmap_lines,
                         std::vector<vector_map_msgs::Point> &vmap_points,
                         std::vector<vector_map_msgs::Pole> &vmap_dummy_poles,
                         std::vector<vector_map_msgs::RoadSign> &vmap_road_signs,
                         std::vector<vector_map_msgs::StopLine> &vmap_stop_lines,
                         std::vector<vector_map_msgs::Vector> &vmap_vectors,
                         std::vector<vector_map_msgs::WhiteLine> &vmap_white_lines
                         )
{
    traffic_rules::TrafficRulesPtr traffic_rules =
        traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
    // routing::RoutingGraphPtr vehicle_graph = routing::RoutingGraph::build(*map, *traffic_rules);

    std::vector<ConstLanelet> vehicle_lanelets;
    for (const auto &lanelet : map->laneletLayer)
    {
        if(traffic_rules->canPass(lanelet)) {
            if(lanelet.leftBound().empty() || lanelet.rightBound().empty()) {
                continue;
            }
            vehicle_lanelets.push_back(lanelet);
        }
    }
    // createStopLines(map, vmap_lines, vmap_points, vmap_stop_lines, vmap_road_signs, vmap_vectors, vmap_dummy_poles);
    createWhitelines(vehicle_lanelets, projector, vmap_points, vmap_lines, vmap_white_lines);
}

void convertLineString2WhiteLine(ConstLineString3d line_string,
                                 projection::UtmProjector projector,
                                 std::vector<vector_map_msgs::Point> &vmap_points,
                                 std::vector<vector_map_msgs::Line> &vmap_lines,
                                 std::vector<vector_map_msgs::WhiteLine> &vmap_white_lines,
                                 int &point_id, int &line_id, int &white_line_id )
{

    int cnt = 0;
    for(auto lanelet_point : line_string)
    {
        autoware_map_msgs::Point amm_point;
        vector_map_msgs::Point vmm_point;
        amm_point = convertPoint(lanelet_point, projector);
        fixPointCoordinate(amm_point);
        convertPoint(vmm_point, amm_point);
        vmm_point.pid = point_id;
        if(USE_FIXED_HEIGHT){
          vmm_point.h = FIXED_HEIGHT;
        }
        vmap_points.push_back(vmm_point);
        if(cnt == 0) {
            cnt++;
            point_id++;
            continue;
        }

        vector_map_msgs::Line line;
        line.lid = line_id;
        line.bpid = point_id - 1;
        line.fpid = point_id;
        line.blid = 0;
        line.flid = 0;
        vmap_lines.push_back(line);

        vector_map_msgs::WhiteLine white_line;
        white_line.id = white_line_id;
        white_line.lid = line_id;
        white_line.width = 0.1;
        white_line.color = 'W';
        white_line.type = 0;
        white_line.linkid = 0;
        vmap_white_lines.push_back(white_line);

        white_line_id++;
        line_id++;
        point_id++;
        cnt++;
    }

}


void createWhitelines(const std::vector<ConstLanelet> &lanelets,
                      projection::UtmProjector projector,
                      std::vector<vector_map_msgs::Point> &vmap_points,
                      std::vector<vector_map_msgs::Line> &vmap_lines,
                      std::vector<vector_map_msgs::WhiteLine> &vmap_white_lines
                      )
{
    int line_id = getMaxId(vmap_lines) + 1;
    int point_id = getMaxId(vmap_points) + 1;
    int white_line_id = getMaxId(vmap_white_lines) + 1;
    std::unordered_map<long long int, bool> checklist;
    for (const auto &lanelet : lanelets)
    {
        if(checklist.find(lanelet.leftBound().id()) == checklist.end()) {
            convertLineString2WhiteLine(lanelet.leftBound(), projector, vmap_points, vmap_lines, vmap_white_lines, point_id, line_id, white_line_id);
            checklist[lanelet.leftBound().id()] = true;
        }
        if(checklist.find(lanelet.rightBound().id()) == checklist.end()) {
            convertLineString2WhiteLine(lanelet.rightBound(), projector, vmap_points, vmap_lines, vmap_white_lines,point_id, line_id, white_line_id);
            checklist[lanelet.rightBound().id()] = true;
        }
    }
}
