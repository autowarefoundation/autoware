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
#include <lanelet2_io/Io.h>

#include <iostream>
#include <unordered_set>
#include <vector>

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

lanelet::Points3d convertPointsLayerToPoints(lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  lanelet::Points3d points;
  for (const lanelet::Point3d & pt : lanelet_map_ptr->pointLayer) {
    points.push_back(pt);
  }
  return points;
}

// lanelet::LineString3d mergeClosePoints(const lanelet::ConstLineString3d& line1, const
// lanelet::ConstLineString3d& line2)
// {
//   lanelet::Points3d new_points;
//   for (const auto& p1 : line1)
//   {
//     p1_basic_point = p1.basicPoint();
//     lanelet::BasicPoint3d p2 = getClosestPointOnLine(line2, p1);
//     lanelet::BasicPoint3d new_basic_point = (p1_basic_point + p2_basic_point)/2;
//     lanelet::Point3d new_point(lanelet::utils::getId(), new_basic_point);
//     new_points.push_back(new_point);
//   }
//   return lanelet::LineString3d(lanelet::utils::getId(), new_points);
// }

void mergePoints(lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  const auto & points = convertPointsLayerToPoints(lanelet_map_ptr);

  for (size_t i = 0; i < points.size(); i++) {
    auto point_i = points.at(i);
    for (size_t j = 0; j < i; j++) {
      auto point_j = points.at(j);

      double distance = boost::geometry::distance(point_i, point_j);
      if (distance < 0.1) {
        const auto new_point = (point_i.basicPoint() + point_j.basicPoint()) / 2;
        // const auto new_pt3d = lanelet::Point3d(lanelet::utils::getId(), new_point);
        point_i.x() = new_point.x();
        point_i.y() = new_point.y();
        point_i.z() = new_point.z();
        point_j.x() = new_point.x();
        point_j.y() = new_point.y();
        point_j.z() = new_point.z();
        point_i.setId(point_j.id());
      }
    }
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("merge_close_points");

  const auto llt_map_path = node->declare_parameter<std::string>("llt_map_path");
  const auto output_path = node->declare_parameter<std::string>("output_path");

  lanelet::LaneletMapPtr llt_map_ptr(new lanelet::LaneletMap);
  lanelet::projection::MGRSProjector projector;

  if (!loadLaneletMap(llt_map_path, llt_map_ptr, projector)) {
    return EXIT_FAILURE;
  }

  mergePoints(llt_map_ptr);
  lanelet::write(output_path, *llt_map_ptr, projector);

  rclcpp::shutdown();

  return 0;
}
