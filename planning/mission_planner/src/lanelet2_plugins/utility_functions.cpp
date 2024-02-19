// Copyright 2019 Autoware Foundation
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

#include "utility_functions.hpp"

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <unordered_set>
#include <utility>

tier4_autoware_utils::Polygon2d convert_linear_ring_to_polygon(
  tier4_autoware_utils::LinearRing2d footprint)
{
  tier4_autoware_utils::Polygon2d footprint_polygon;
  boost::geometry::append(footprint_polygon.outer(), footprint[0]);
  boost::geometry::append(footprint_polygon.outer(), footprint[1]);
  boost::geometry::append(footprint_polygon.outer(), footprint[2]);
  boost::geometry::append(footprint_polygon.outer(), footprint[3]);
  boost::geometry::append(footprint_polygon.outer(), footprint[4]);
  boost::geometry::append(footprint_polygon.outer(), footprint[5]);
  boost::geometry::correct(footprint_polygon);
  return footprint_polygon;
}

void insert_marker_array(
  visualization_msgs::msg::MarkerArray * a1, const visualization_msgs::msg::MarkerArray & a2)
{
  a1->markers.insert(a1->markers.end(), a2.markers.begin(), a2.markers.end());
}

lanelet::ConstLanelet combine_lanelets_with_shoulder(
  const lanelet::ConstLanelets & lanelets, const lanelet::ConstLanelets & shoulder_lanelets)
{
  lanelet::Points3d lefts;
  lanelet::Points3d rights;
  lanelet::Points3d centers;
  std::vector<uint64_t> left_bound_ids;
  std::vector<uint64_t> right_bound_ids;

  for (const auto & llt : lanelets) {
    if (llt.id() != lanelet::InvalId) {
      left_bound_ids.push_back(llt.leftBound().id());
      right_bound_ids.push_back(llt.rightBound().id());
    }
  }

  for (const auto & llt : lanelets) {
    // lambda to check if shoulder lane which share left bound with lanelets exist
    const auto find_bound_shared_shoulder =
      [&shoulder_lanelets](const auto & lanelet_bound, const auto & get_shoulder_bound) {
        return std::find_if(
          shoulder_lanelets.begin(), shoulder_lanelets.end(),
          [&lanelet_bound, &get_shoulder_bound](const auto & shoulder_llt) {
            return lanelet_bound.id() == get_shoulder_bound(shoulder_llt).id();
          });
      };

    // lambda to add bound to target_bound
    const auto add_bound = [](const auto & bound, auto & target_bound) {
      std::transform(
        bound.begin(), bound.end(), std::back_inserter(target_bound),
        [](const auto & pt) { return lanelet::Point3d(pt); });
    };

    // check if shoulder lanelets which has RIGHT bound same to LEFT bound of lanelet exist
    const auto left_shared_shoulder_it = find_bound_shared_shoulder(
      llt.leftBound(), [](const auto & shoulder_llt) { return shoulder_llt.rightBound(); });
    if (left_shared_shoulder_it != shoulder_lanelets.end()) {
      // if exist, add left bound of SHOULDER lanelet to lefts
      add_bound(left_shared_shoulder_it->leftBound(), lefts);
    } else if (
      // if not exist, add left bound of lanelet to lefts
      // if the **left** of this lanelet does not match any of the **right** bounds of `lanelets`,
      // then its left bound constitutes the left boundary of the entire merged lanelet
      std::count(right_bound_ids.begin(), right_bound_ids.end(), llt.leftBound().id()) < 1) {
      add_bound(llt.leftBound(), lefts);
    }

    // check if shoulder lanelets which has LEFT bound same to RIGHT bound of lanelet exist
    const auto right_shared_shoulder_it = find_bound_shared_shoulder(
      llt.rightBound(), [](const auto & shoulder_llt) { return shoulder_llt.leftBound(); });
    if (right_shared_shoulder_it != shoulder_lanelets.end()) {
      // if exist, add right bound of SHOULDER lanelet to rights
      add_bound(right_shared_shoulder_it->rightBound(), rights);
    } else if (
      // if not exist, add right bound of lanelet to rights
      // if the **right** of this lanelet does not match any of the **left** bounds of `lanelets`,
      // then its right bound constitutes the right boundary of the entire merged lanelet
      std::count(left_bound_ids.begin(), left_bound_ids.end(), llt.rightBound().id()) < 1) {
      add_bound(llt.rightBound(), rights);
    }
  }

  const auto left_bound = lanelet::LineString3d(lanelet::InvalId, lefts);
  const auto right_bound = lanelet::LineString3d(lanelet::InvalId, rights);
  auto combined_lanelet = lanelet::Lanelet(lanelet::InvalId, left_bound, right_bound);
  return std::move(combined_lanelet);
}

std::vector<geometry_msgs::msg::Point> convertCenterlineToPoints(const lanelet::Lanelet & lanelet)
{
  std::vector<geometry_msgs::msg::Point> centerline_points;
  for (const auto & point : lanelet.centerline()) {
    geometry_msgs::msg::Point center_point;
    center_point.x = point.basicPoint().x();
    center_point.y = point.basicPoint().y();
    center_point.z = point.basicPoint().z();
    centerline_points.push_back(center_point);
  }
  return centerline_points;
}

geometry_msgs::msg::Pose convertBasicPoint3dToPose(
  const lanelet::BasicPoint3d & point, const double lane_yaw)
{
  // calculate new orientation of goal
  geometry_msgs::msg::Pose pose;
  pose.position.x = point.x();
  pose.position.y = point.y();
  pose.position.z = point.z();

  pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(lane_yaw);

  return pose;
}
