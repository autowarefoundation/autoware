// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <lanelet2_extension/io/autoware_osm_parser.hpp>
#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <iostream>
#include <unordered_set>
#include <vector>

using lanelet::utils::getId;
using lanelet::utils::to2D;

bool loadLaneletMap(
  const std::string & llt_map_path, lanelet::LaneletMapPtr & lanelet_map_ptr,
  lanelet::Projector & projector)
{
  lanelet::LaneletMapPtr lanelet_map;
  lanelet::ErrorMessages errors;
  lanelet_map_ptr = lanelet::load(llt_map_path, "autoware_osm_handler", projector, &errors);

  for (const auto & error : errors) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("loadLaneletMap"), error);
  }
  if (!errors.empty()) {
    return false;
  }
  std::cout << "Loaded Lanelet2 map" << std::endl;
  return true;
}

bool exists(std::unordered_set<lanelet::Id> & set, lanelet::Id element)
{
  return std::find(set.begin(), set.end(), element) != set.end();
}

lanelet::Lanelets convertToVector(lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  lanelet::Lanelets lanelets;
  for (lanelet::Lanelet lanelet : lanelet_map_ptr->laneletLayer) {
    lanelets.push_back(lanelet);
  }
  return lanelets;
}
void fixTags(lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  auto lanelets = convertToVector(lanelet_map_ptr);
  lanelet::traffic_rules::TrafficRulesPtr trafficRules =
    lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  lanelet::routing::RoutingGraphUPtr routingGraph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr, *trafficRules);

  for (auto & llt : lanelets) {
    if (!routingGraph->conflicting(llt).empty()) {
      continue;
    }
    llt.attributes().erase("turn_direction");
    if (!!routingGraph->adjacentRight(llt)) {
      llt.rightBound().attributes()["lane_change"] = "yes";
    }
    if (!!routingGraph->adjacentLeft(llt)) {
      llt.leftBound().attributes()["lane_change"] = "yes";
    }
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("fix_lane_change_tags");

  const auto llt_map_path = node->declare_parameter<std::string>("llt_map_path");
  const auto output_path = node->declare_parameter<std::string>("output_path");

  lanelet::LaneletMapPtr llt_map_ptr(new lanelet::LaneletMap);
  lanelet::projection::MGRSProjector projector;

  if (!loadLaneletMap(llt_map_path, llt_map_ptr, projector)) {
    return EXIT_FAILURE;
  }

  fixTags(llt_map_ptr);
  lanelet::write(output_path, *llt_map_ptr, projector);

  rclcpp::shutdown();

  return 0;
}
