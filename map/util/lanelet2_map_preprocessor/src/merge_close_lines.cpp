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

#include <autoware_lanelet2_extension/io/autoware_osm_parser.hpp>
#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <lanelet2_io/Io.h>

#include <iostream>
#include <unordered_set>
#include <vector>

bool load_lanelet_map(
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

lanelet::LineStrings3d convert_line_layer_to_line_strings(
  const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  lanelet::LineStrings3d lines;
  std::copy(
    lanelet_map_ptr->lineStringLayer.begin(), lanelet_map_ptr->lineStringLayer.end(),
    std::back_inserter(lines));
  return lines;
}

lanelet::ConstPoint3d get3d_point_from2d_arc_length(
  const lanelet::ConstLineString3d & line, const double s)
{
  double accumulated_distance2d = 0;
  if (line.size() < 2) {
    return lanelet::Point3d();
  }
  auto prev_pt = line.front();
  for (size_t i = 1; i < line.size(); i++) {
    const auto & pt = line[i];
    double distance2d =
      lanelet::geometry::distance2d(lanelet::utils::to2D(prev_pt), lanelet::utils::to2D(pt));
    if (accumulated_distance2d + distance2d >= s) {
      double ratio = (s - accumulated_distance2d) / distance2d;
      auto interpolated_pt = prev_pt.basicPoint() * (1 - ratio) + pt.basicPoint() * ratio;
      std::cout << interpolated_pt << std::endl;
      return lanelet::ConstPoint3d{
        lanelet::utils::getId(), interpolated_pt.x(), interpolated_pt.y(), interpolated_pt.z()};
    }
    accumulated_distance2d += distance2d;
    prev_pt = pt;
  }
  RCLCPP_ERROR(rclcpp::get_logger("merge_close_lines"), "interpolation failed");
  return {};
}

bool are_lines_same(
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

  double sum_distance =
    std::accumulate(line1.begin(), line1.end(), 0.0, [&line2](double sum, const auto & pt) {
      return sum + boost::geometry::distance(pt.basicPoint(), line2);
    });
  sum_distance +=
    std::accumulate(line2.begin(), line2.end(), 0.0, [&line1](double sum, const auto & pt) {
      return sum + boost::geometry::distance(pt.basicPoint(), line1);
    });

  double avg_distance = sum_distance / static_cast<double>(line1.size() + line2.size());
  std::cout << line1 << " " << line2 << " " << avg_distance << std::endl;
  return avg_distance < 1.0;
}

lanelet::BasicPoint3d get_closest_point_on_line(
  const lanelet::BasicPoint3d & search_point, const lanelet::ConstLineString3d & line)
{
  auto arc_coordinate = lanelet::geometry::toArcCoordinates(
    lanelet::utils::to2D(line), lanelet::utils::to2D(search_point));
  std::cout << arc_coordinate.length << " " << arc_coordinate.distance << std::endl;
  return get3d_point_from2d_arc_length(line, arc_coordinate.length).basicPoint();
}

lanelet::LineString3d merge_two_lines(
  const lanelet::LineString3d & line1, const lanelet::ConstLineString3d & line2)
{
  lanelet::Points3d new_points;
  for (const auto & p1 : line1) {
    const lanelet::BasicPoint3d & p1_basic_point = p1.basicPoint();
    lanelet::BasicPoint3d p2_basic_point = get_closest_point_on_line(p1, line2);
    lanelet::BasicPoint3d new_basic_point = (p1_basic_point + p2_basic_point) / 2;
    lanelet::Point3d new_point(lanelet::utils::getId(), new_basic_point);
    new_points.push_back(new_point);
  }
  return lanelet::LineString3d{lanelet::utils::getId(), new_points};
}

void copy_data(lanelet::LineString3d & dst, const lanelet::LineString3d & src)
{
  dst.clear();
  for (const lanelet::ConstPoint3d & pt : src) {
    dst.push_back(static_cast<lanelet::Point3d>(pt));
  }
}

void merge_lines(lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  auto lines = convert_line_layer_to_line_strings(lanelet_map_ptr);

  for (size_t i = 0; i < lines.size(); i++) {
    auto line_i = lines.at(i);
    for (size_t j = 0; j < i; j++) {
      auto line_j = lines.at(j);
      if (are_lines_same(line_i, line_j)) {
        auto merged_line = merge_two_lines(line_i, line_j);
        copy_data(line_i, merged_line);
        copy_data(line_j, merged_line);
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

  if (!load_lanelet_map(llt_map_path, llt_map_ptr, projector)) {
    return EXIT_FAILURE;
  }

  merge_lines(llt_map_ptr);
  lanelet::write(output_path, *llt_map_ptr, projector);

  rclcpp::shutdown();

  return 0;
}
