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

lanelet::LineStrings3d convertLineLayerToLineStrings(lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  lanelet::LineStrings3d lines;
  for (const lanelet::LineString3d & line : lanelet_map_ptr->lineStringLayer) {
    lines.push_back(line);
  }
  return lines;
}

lanelet::ConstPoint3d get3DPointFrom2DArcLength(
  const lanelet::ConstLineString3d & line, const double s)
{
  double accumulated_distance2d = 0;
  if (line.size() < 2) {
    return lanelet::Point3d();
  }
  auto prev_pt = line.front();
  for (size_t i = 1; i < line.size(); i++) {
    const auto & pt = line[i];
    double distance2d = lanelet::geometry::distance2d(to2D(prev_pt), to2D(pt));
    if (accumulated_distance2d + distance2d >= s) {
      double ratio = (s - accumulated_distance2d) / distance2d;
      auto interpolated_pt = prev_pt.basicPoint() * (1 - ratio) + pt.basicPoint() * ratio;
      std::cout << interpolated_pt << std::endl;
      return lanelet::ConstPoint3d(
        lanelet::utils::getId(), interpolated_pt.x(), interpolated_pt.y(), interpolated_pt.z());
    }
    accumulated_distance2d += distance2d;
    prev_pt = pt;
  }
  RCLCPP_ERROR(rclcpp::get_logger("merge_close_lines"), "interpolation failed");
  return lanelet::ConstPoint3d();
}

double getLineLength(const lanelet::ConstLineString3d & line)
{
  return boost::geometry::length(line.basicLineString());
}

bool areLinesSame(
  const lanelet::ConstLineString3d & line1, const lanelet::ConstLineString3d & line2)
{
  bool same_ends = false;
  if (line1.front() == line2.front() && line1.back() == line2.back()) {
    same_ends = true;
  }
  if (line1.front() == line2.back() && line1.back() == line2.front()) {
    same_ends = true;
  }
  if (!same_ends) {
    return false;
  }

  double sum_distance = 0;
  for (const auto & pt : line1) {
    sum_distance += boost::geometry::distance(pt.basicPoint(), line2);
  }
  for (const auto & pt : line2) {
    sum_distance += boost::geometry::distance(pt.basicPoint(), line1);
  }

  double avg_distance = sum_distance / (line1.size() + line2.size());
  std::cout << line1 << " " << line2 << " " << avg_distance << std::endl;
  if (avg_distance < 1.0) {
    return true;
  } else {
    return false;
  }
}

lanelet::BasicPoint3d getClosestPointOnLine(
  const lanelet::BasicPoint3d & search_point, const lanelet::ConstLineString3d & line)
{
  auto arc_coordinate = lanelet::geometry::toArcCoordinates(to2D(line), to2D(search_point));
  std::cout << arc_coordinate.length << " " << arc_coordinate.distance << std::endl;
  return get3DPointFrom2DArcLength(line, arc_coordinate.length).basicPoint();
}

lanelet::LineString3d mergeTwoLines(
  const lanelet::LineString3d & line1, const lanelet::ConstLineString3d & line2)
{
  lanelet::Points3d new_points;
  for (const auto & p1 : line1) {
    lanelet::BasicPoint3d p1_basic_point = p1.basicPoint();
    lanelet::BasicPoint3d p2_basic_point = getClosestPointOnLine(p1, line2);
    lanelet::BasicPoint3d new_basic_point = (p1_basic_point + p2_basic_point) / 2;
    lanelet::Point3d new_point(lanelet::utils::getId(), new_basic_point);
    new_points.push_back(new_point);
  }
  return lanelet::LineString3d(lanelet::utils::getId(), new_points);
}

void copyData(lanelet::LineString3d & dst, lanelet::LineString3d & src)
{
  lanelet::Points3d points;
  dst.clear();
  for (lanelet::Point3d & pt : src) {
    dst.push_back(pt);
  }
}

void mergeLines(lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  auto lines = convertLineLayerToLineStrings(lanelet_map_ptr);

  for (size_t i = 0; i < lines.size(); i++) {
    auto line_i = lines.at(i);
    for (size_t j = 0; j < i; j++) {
      auto line_j = lines.at(j);
      if (areLinesSame(line_i, line_j)) {
        auto merged_line = mergeTwoLines(line_i, line_j);
        copyData(line_i, merged_line);
        copyData(line_j, merged_line);
        line_i.setId(line_j.id());
        std::cout << line_j << " " << line_i << std::endl;
        // lanelet_map_ptr->add(merged_line);
        for (lanelet::Point3d & pt : merged_line) {
          lanelet_map_ptr->add(pt);
        }
        break;
      }
    }
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("merge_close_lines");

  const auto llt_map_path = node->declare_parameter<std::string>("llt_map_path");
  const auto output_path = node->declare_parameter<std::string>("output_path");

  lanelet::LaneletMapPtr llt_map_ptr(new lanelet::LaneletMap);
  lanelet::projection::MGRSProjector projector;

  if (!loadLaneletMap(llt_map_path, llt_map_ptr, projector)) {
    return EXIT_FAILURE;
  }

  mergeLines(llt_map_ptr);
  lanelet::write(output_path, *llt_map_ptr, projector);

  rclcpp::shutdown();

  return 0;
}
