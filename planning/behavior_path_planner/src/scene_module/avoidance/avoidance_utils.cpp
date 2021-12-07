// Copyright 2021 Tier IV, Inc.
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

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/avoidance/avoidance_module.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <autoware_utils/autoware_utils.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

#include <algorithm>
#include <iomanip>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace behavior_path_planner
{
bool isOnRight(const ObjectData & obj) { return obj.lateral < 0.0; }

lanelet::ConstLanelets calcLaneAroundPose(
  const std::shared_ptr<const PlannerData> & planner_data, const geometry_msgs::msg::Pose & pose,
  const double backward_length)
{
  const auto & p = planner_data->parameters;
  const auto & route_handler = planner_data->route_handler;

  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithinRoute(pose, &current_lane)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("behavior_path_planner").get_child("avoidance"),
      "failed to find closest lanelet within route!!!");
    return {};  // TODO(Horibe)
  }

  // For current_lanes with desired length
  lanelet::ConstLanelets current_lanes =
    route_handler->getLaneletSequence(current_lane, pose, backward_length, p.forward_path_length);

  return current_lanes;
}

ShiftedPath toShiftedPath(const PathWithLaneId & path)
{
  ShiftedPath out;
  out.path = path;
  out.shift_length.resize(path.points.size());
  std::fill(out.shift_length.begin(), out.shift_length.end(), 0.0);
  return out;
}

ShiftPointArray toShiftPointArray(const AvoidPointArray & avoid_points)
{
  ShiftPointArray shift_points;
  for (const auto & ap : avoid_points) {
    shift_points.push_back(ap);
  }
  return shift_points;
}

size_t findPathIndexFromArclength(
  const std::vector<double> & path_arclength_arr, const double target_arc)
{
  if (path_arclength_arr.empty()) {
    return 0;
  }

  for (size_t i = 0; i < path_arclength_arr.size(); ++i) {
    if (path_arclength_arr.at(i) > target_arc) {
      return i;
    }
  }
  return path_arclength_arr.size() - 1;
}

std::vector<size_t> concatParentIds(
  const std::vector<size_t> & ids1, const std::vector<size_t> & ids2)
{
  std::set<size_t> id_set{ids1.begin(), ids1.end()};
  for (const auto id : ids2) {
    id_set.insert(id);
  }
  const auto v = std::vector<size_t>{id_set.begin(), id_set.end()};
  return v;
}

double lerpShiftLengthOnArc(double arc, const AvoidPoint & ap)
{
  if (ap.start_longitudinal <= arc && arc < ap.end_longitudinal) {
    if (std::abs(ap.getRelativeLongitudinal()) < 1.0e-5) {
      return ap.length;
    }
    const auto start_weight = (ap.end_longitudinal - arc) / ap.getRelativeLongitudinal();
    return start_weight * ap.start_length + (1.0 - start_weight) * ap.length;
  }
  return 0.0;
}

void clipByMinStartIdx(const AvoidPointArray & shift_points, PathWithLaneId & path)
{
  if (path.points.empty()) {
    return;
  }

  size_t min_start_idx = std::numeric_limits<size_t>::max();
  for (const auto & sp : shift_points) {
    min_start_idx = std::min(min_start_idx, sp.start_idx);
  }
  min_start_idx = std::min(min_start_idx, path.points.size() - 1);
  path.points =
    std::vector<PathPointWithLaneId>{path.points.begin() + min_start_idx, path.points.end()};
}

double calcDistanceToClosestFootprintPoint(
  const PathWithLaneId & path, const PredictedObject & object, const Point & ego_pos)
{
  autoware_utils::Polygon2d object_poly{};
  util::calcObjectPolygon(object, &object_poly);

  double distance = autoware_utils::calcSignedArcLength(
    path.points, ego_pos, object.kinematics.initial_pose_with_covariance.pose.position);
  for (const auto & p : object_poly.outer()) {
    const auto point = autoware_utils::createPoint(p.x(), p.y(), 0.0);
    distance = std::min(distance, autoware_utils::calcSignedArcLength(path.points, ego_pos, point));
  }
  return distance;
}

double calcOverhangDistance(const ObjectData & object_data, const Pose & base_pose)
{
  double largest_overhang = isOnRight(object_data) ? -100.0 : 100.0;  // large number

  autoware_utils::Polygon2d object_poly{};
  util::calcObjectPolygon(object_data.object, &object_poly);

  for (const auto & p : object_poly.outer()) {
    const auto point = autoware_utils::createPoint(p.x(), p.y(), 0.0);
    const auto lateral = autoware_utils::calcLateralDeviation(base_pose, point);
    largest_overhang = isOnRight(object_data) ? std::max(largest_overhang, lateral)
                                              : std::min(largest_overhang, lateral);
  }
  return largest_overhang;
}

void setEndData(
  AvoidPoint & ap, const double length, const geometry_msgs::msg::Pose & end, const size_t end_idx,
  const double end_dist)
{
  ap.length = length;
  ap.end = end;
  ap.end_idx = end_idx;
  ap.end_longitudinal = end_dist;
}

void setStartData(
  AvoidPoint & ap, const double start_length, const geometry_msgs::msg::Pose & start,
  const size_t start_idx, const double start_dist)
{
  ap.start_length = start_length;
  ap.start = start;
  ap.start_idx = start_idx;
  ap.start_longitudinal = start_dist;
}

}  // namespace behavior_path_planner
