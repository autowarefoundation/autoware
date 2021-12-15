// Copyright 2020 TierIV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

//lint -e537 pclint vs cpplint NOLINT


#include "had_map_utils/had_map_utils.hpp"

#include <lanelet2_core/geometry/Lanelet.h>
#include <common/types.hpp>
#include <utility>
#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using autoware::common::types::float64_t;

namespace autoware
{
namespace common
{
namespace had_map_utils
{

std::vector<float64_t> calculateSegmentDistances(const lanelet::ConstLineString3d & line_string)
{
  std::vector<float64_t> segment_distances;
  segment_distances.reserve(line_string.size() - 1);

  for (size_t i = 1; i < line_string.size(); ++i) {
    const auto distance = lanelet::geometry::distance(line_string[i], line_string[i - 1]);
    segment_distances.push_back(distance);
  }

  return segment_distances;
}

std::vector<float64_t> calculateAccumulatedLengths(const lanelet::ConstLineString3d & line_string)
{
  const auto segment_distances = calculateSegmentDistances(line_string);

  std::vector<float64_t> accumulated_lengths{0};
  accumulated_lengths.reserve(segment_distances.size() + 1);
  std::partial_sum(
    std::begin(segment_distances), std::end(segment_distances),
    std::back_inserter(accumulated_lengths));

  return accumulated_lengths;
}

std::pair<size_t, size_t> findNearestIndexPair(
  const std::vector<float64_t> & accumulated_lengths, const float64_t target_length)
{
  // List size
  const auto N = accumulated_lengths.size();

  // Front
  if (target_length < accumulated_lengths.at(1)) {
    return std::make_pair(0, 1);
  }

  // Back
  if (target_length > accumulated_lengths.at(N - 2)) {
    return std::make_pair(N - 2, N - 1);
  }

  // Middle
  for (size_t i = 1; i < N; ++i) {
    if (
      accumulated_lengths.at(i - 1) <= target_length &&
      target_length <= accumulated_lengths.at(i))
    {
      return std::make_pair(i - 1, i);
    }
  }

  // Throw an exception because this never happens
  throw std::runtime_error(
          "findNearestIndexPair(): No nearest point found.");
}

std::vector<lanelet::BasicPoint3d> resamplePoints(
  const lanelet::ConstLineString3d & line_string, const int32_t num_segments)
{
  // Calculate length
  const auto line_length = lanelet::geometry::length(line_string);

  // Calculate accumulated lengths
  const auto accumulated_lengths = calculateAccumulatedLengths(line_string);

  // Create each segment
  std::vector<lanelet::BasicPoint3d> resampled_points;
  for (auto i = 0; i <= num_segments; ++i) {
    // Find two nearest points
    const float64_t target_length = (static_cast<float64_t>(i) / num_segments) *
      static_cast<float64_t>(line_length);
    const auto index_pair = findNearestIndexPair(accumulated_lengths, target_length);

    // Apply linear interpolation
    const lanelet::BasicPoint3d back_point = line_string[index_pair.first];
    const lanelet::BasicPoint3d front_point = line_string[index_pair.second];
    const auto direction_vector = (front_point - back_point);

    const auto back_length = accumulated_lengths.at(index_pair.first);
    const auto front_length = accumulated_lengths.at(index_pair.second);
    const auto segment_length = front_length - back_length;
    const auto target_point =
      back_point + (direction_vector * (target_length - back_length) / segment_length);

    // Add to list
    resampled_points.push_back(target_point);
  }

  return resampled_points;
}

lanelet::LineString3d generateFineCenterline(
  const lanelet::ConstLanelet & lanelet_obj, const float64_t resolution)
{
  // Get length of longer border
  const float64_t left_length =
    static_cast<float64_t>(lanelet::geometry::length(lanelet_obj.leftBound()));
  const float64_t right_length =
    static_cast<float64_t>(lanelet::geometry::length(lanelet_obj.rightBound()));
  const float64_t longer_distance = (left_length > right_length) ? left_length : right_length;
  const int32_t num_segments =
    std::max(static_cast<int32_t>(ceil(longer_distance / resolution)), 1);

  // Resample points
  const auto left_points = resamplePoints(lanelet_obj.leftBound(), num_segments);
  const auto right_points = resamplePoints(lanelet_obj.rightBound(), num_segments);

  // Create centerline
  lanelet::LineString3d centerline(lanelet::utils::getId());
  for (size_t i = 0; i < static_cast<size_t>(num_segments + 1); i++) {
    // Add ID for the average point of left and right
    const auto center_basic_point = (right_points.at(i) + left_points.at(i)) / 2.0;
    const lanelet::Point3d center_point(
      lanelet::utils::getId(), center_basic_point.x(), center_basic_point.y(),
      center_basic_point.z());
    centerline.push_back(center_point);
  }
  return centerline;
}

void overwriteLaneletsCenterline(
  lanelet::LaneletMapPtr lanelet_map,
  const autoware::common::types::bool8_t force_overwrite)
{
  for (auto & lanelet_obj : lanelet_map->laneletLayer) {
    if (force_overwrite || !lanelet_obj.hasCustomCenterline()) {
      const auto fine_center_line = generateFineCenterline(lanelet_obj, 2.0);
      lanelet_obj.setCenterline(fine_center_line);
    }
  }
}

}  // namespace had_map_utils
}  // namespace common
}  // namespace autoware
