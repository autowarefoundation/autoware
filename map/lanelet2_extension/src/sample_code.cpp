// Copyright 2015-2019 Autoware Foundation
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

#include "lanelet2_extension/projection/mgrs_projector.hpp"
#include "lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp"

#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <iostream>
#include <string>

void loadingAutowareOSMFile(const std::string map_file_path)
{
  lanelet::LaneletMapPtr lanelet_map;
  lanelet::ErrorMessages errors;
  lanelet::GPSPoint gps_point;
  gps_point.lat = 49.0;
  gps_point.lon = 8.4;
  lanelet::Origin origin(gps_point);
  lanelet::projection::UtmProjector projector(lanelet::Origin(gps_point), true, false);

  // Autoware OSM file parser is registered into lanelet2 library.
  // Therefore, you can used it by just specifying "autoware_osm_handler" parser
  // in load function.
  lanelet_map = lanelet::load(map_file_path, "autoware_osm_handler", projector, &errors);

  // If you want to use default parser, explicitly name the parser
  lanelet_map = lanelet::load(map_file_path, "osm_handler", projector, &errors);
}

void usingMGRSProjector()
{
  // MGRS Projector projects lat/lon to x,y,z point in MGRS 100km grid.
  // The origin is automatically calculated so you don't have to select any
  // origin.
  lanelet::projection::MGRSProjector projector;

  // Let's convert lat/lng point into mgrs xyz point.
  lanelet::GPSPoint gps_point;
  gps_point.lat = 49.0;
  gps_point.lon = 8.4;
  gps_point.ele = 0.0;

  lanelet::BasicPoint3d mgrs_point = projector.forward(gps_point);
  std::cout << mgrs_point << std::endl;

  // You can get MGRS Code of projected grid.
  std::string mgrs_grid = projector.getProjectedMGRSGrid();
  std::cout << mgrs_grid << std::endl;

  // You can also reverse project from MGRS point to lat/lon.
  // You have to set which MGRS grid it is from or it will reuse last projected
  // grid
  lanelet::GPSPoint projected_gps_point = projector.reverse(mgrs_point);
  std::cout << projected_gps_point.lat << " " << projected_gps_point.lon << std::endl;
  lanelet::GPSPoint projected_gps_point2 = projector.reverse(mgrs_point, mgrs_grid);
  std::cout << projected_gps_point2.lat << " " << projected_gps_point2.lon << " " << std::endl;
}

void usingAutowareTrafficLight(const std::string map_file_path)
{
  lanelet::LaneletMapPtr lanelet_map;
  lanelet::ErrorMessages errors;
  lanelet::projection::MGRSProjector projector;
  lanelet_map = lanelet::load(map_file_path, "autoware_osm_handler", projector, &errors);

  for (auto lanelet : lanelet_map->laneletLayer) {
    // You can access to traffic light element as AutowareTrafficLight class
    auto autoware_traffic_lights =
      lanelet.regulatoryElementsAs<lanelet::autoware::AutowareTrafficLight>();
    for (auto light : autoware_traffic_lights) {
      // You can access to the position of each lamps(light bulb) in traffic
      // light
      for (auto light_bulb_string : light->lightBulbs()) {
        std::cout << light_bulb_string.id() << std::endl;
      }
      // Since AutowareTrafficLight class is inheriting lanelet::TrafficLight
      // class, you can also access to outline of traffic light by the same
      // method.
      for (auto light_string : light->trafficLights()) {
        std::cout << light_string.id() << std::endl;
      }
    }

    // You can also access to same traffic light element as default TrafficLight
    // class
    auto traffic_lights = lanelet.regulatoryElementsAs<lanelet::TrafficLight>();
    for (auto light : traffic_lights) {
      for (auto light_string : light->trafficLights()) {
        std::cout << light_string.id() << std::endl;
      }
    }
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("sample_code");
  const std::string map_file_path = node->declare_parameter("map_file", "");
  loadingAutowareOSMFile(map_file_path);
  usingMGRSProjector();
  usingAutowareTrafficLight(map_file_path);
  return 0;
}
