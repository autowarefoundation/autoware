// Copyright 2023 TIER IV, Inc.
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

#include "behavior_path_planner/scene_module/lane_change/avoidance_by_lane_change.hpp"

#include "behavior_path_planner/utils/avoidance/utils.hpp"
#include "behavior_path_planner/utils/path_utils.hpp"

#include <lanelet2_extension/utility/utilities.hpp>

#include <utility>

namespace behavior_path_planner
{
AvoidanceByLaneChange::AvoidanceByLaneChange(
  const std::shared_ptr<LaneChangeParameters> & parameters,
  std::shared_ptr<AvoidanceParameters> avoidance_parameters,
  std::shared_ptr<AvoidanceByLCParameters> avoidance_by_lane_change_parameters)
: NormalLaneChange(parameters, LaneChangeModuleType::AVOIDANCE_BY_LANE_CHANGE, Direction::NONE),
  avoidance_parameters_(std::move(avoidance_parameters)),
  avoidance_by_lane_change_parameters_(std::move(avoidance_by_lane_change_parameters))
{
}

std::pair<bool, bool> AvoidanceByLaneChange::getSafePath(LaneChangePath & safe_path) const
{
  const auto & avoidance_objects = avoidance_data_.target_objects;
  const auto execute_object_num = avoidance_by_lane_change_parameters_->execute_object_num;

  if (avoidance_objects.size() < execute_object_num) {
    return {false, false};
  }

  const auto current_lanes = getCurrentLanes();

  if (current_lanes.empty()) {
    return {false, false};
  }

  const auto & o_front = avoidance_objects.front();

  // check distance from ego to o_front vs acceptable longitudinal margin
  const auto execute_object_longitudinal_margin =
    avoidance_by_lane_change_parameters_->execute_object_longitudinal_margin;
  if (execute_object_longitudinal_margin > o_front.longitudinal) {
    return {false, false};
  }

  const auto direction = utils::avoidance::isOnRight(o_front) ? Direction::LEFT : Direction::RIGHT;
  const auto target_lanes = getLaneChangeLanes(current_lanes, direction);

  if (target_lanes.empty()) {
    return {false, false};
  }

  // find candidate paths
  LaneChangePaths valid_paths{};
  const auto found_safe_path =
    getLaneChangePaths(current_lanes, target_lanes, direction, &valid_paths);

  if (valid_paths.empty()) {
    return {false, false};
  }

  if (found_safe_path) {
    safe_path = valid_paths.back();
  } else {
    // force candidate
    safe_path = valid_paths.front();
  }

  const auto to_lane_change_end_distance = motion_utils::calcSignedArcLength(
    safe_path.path.points, getEgoPose().position, safe_path.shift_line.end.position);
  const auto lane_change_finish_before_object = o_front.longitudinal > to_lane_change_end_distance;
  const auto execute_only_when_lane_change_finish_before_object =
    avoidance_by_lane_change_parameters_->execute_only_when_lane_change_finish_before_object;
  if (!lane_change_finish_before_object && execute_only_when_lane_change_finish_before_object) {
    return {false, found_safe_path};
  }
  return {true, found_safe_path};
}

void AvoidanceByLaneChange::updateSpecialData()
{
  avoidance_debug_data_ = DebugData();
  avoidance_data_ = calcAvoidancePlanningData(avoidance_debug_data_);

  utils::avoidance::updateRegisteredObject(
    registered_objects_, avoidance_data_.target_objects,
    avoidance_by_lane_change_parameters_->avoidance);
  utils::avoidance::compensateDetectionLost(
    registered_objects_, avoidance_data_.target_objects, avoidance_data_.other_objects);

  std::sort(
    avoidance_data_.target_objects.begin(), avoidance_data_.target_objects.end(),
    [](auto a, auto b) { return a.longitudinal < b.longitudinal; });
}

AvoidancePlanningData AvoidanceByLaneChange::calcAvoidancePlanningData(
  AvoidanceDebugData & debug) const
{
  AvoidancePlanningData data;

  // reference pose
  data.reference_pose = getEgoPose();

  data.reference_path_rough = *prev_module_path_;

  const auto resample_interval =
    avoidance_by_lane_change_parameters_->avoidance->resample_interval_for_planning;
  data.reference_path = utils::resamplePathWithSpline(data.reference_path_rough, resample_interval);

  data.current_lanelets = getCurrentLanes();

  // get related objects from dynamic_objects, and then separates them as target objects and non
  // target objects
  fillAvoidanceTargetObjects(data, debug);

  return data;
}

void AvoidanceByLaneChange::fillAvoidanceTargetObjects(
  AvoidancePlanningData & data, DebugData & debug) const
{
  const auto left_expand_dist = avoidance_parameters_->detection_area_left_expand_dist;
  const auto right_expand_dist = avoidance_parameters_->detection_area_right_expand_dist;

  const auto expanded_lanelets = lanelet::utils::getExpandedLanelets(
    data.current_lanelets, left_expand_dist, right_expand_dist * (-1.0));

  const auto [object_within_target_lane, object_outside_target_lane] =
    utils::separateObjectsByLanelets(*planner_data_->dynamic_object, expanded_lanelets);

  // Assume that the maximum allocation for data.other object is the sum of
  // objects_within_target_lane and object_outside_target_lane. The maximum allocation for
  // data.target_objects is equal to object_within_target_lane
  {
    const auto other_objects_size =
      object_within_target_lane.objects.size() + object_outside_target_lane.objects.size();
    data.other_objects.reserve(other_objects_size);
    data.target_objects.reserve(object_within_target_lane.objects.size());
  }

  {
    const auto & objects = object_outside_target_lane.objects;
    std::transform(
      objects.cbegin(), objects.cend(), std::back_inserter(data.other_objects),
      [](const auto & object) {
        ObjectData other_object;
        other_object.object = object;
        other_object.reason = "OutOfTargetArea";
        return other_object;
      });
  }

  ObjectDataArray target_lane_objects;
  target_lane_objects.reserve(object_within_target_lane.objects.size());
  {
    const auto & objects = object_within_target_lane.objects;
    std::transform(
      objects.cbegin(), objects.cend(), std::back_inserter(target_lane_objects),
      [&](const auto & object) { return createObjectData(data, object); });
  }

  utils::avoidance::filterTargetObjects(
    target_lane_objects, data, debug, planner_data_, avoidance_parameters_);
}

ObjectData AvoidanceByLaneChange::createObjectData(
  const AvoidancePlanningData & data, const PredictedObject & object) const
{
  using boost::geometry::return_centroid;

  const auto & path_points = data.reference_path.points;
  const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
  const auto object_closest_index =
    motion_utils::findNearestIndex(path_points, object_pose.position);
  const auto object_closest_pose = path_points.at(object_closest_index).point.pose;

  ObjectData object_data{};

  object_data.object = object;

  // Calc envelop polygon.
  utils::avoidance::fillObjectEnvelopePolygon(
    object_data, registered_objects_, object_closest_pose, avoidance_parameters_);

  // calc object centroid.
  object_data.centroid = return_centroid<Point2d>(object_data.envelope_poly);

  // Calc moving time.
  utils::avoidance::fillObjectMovingTime(object_data, stopped_objects_, avoidance_parameters_);

  // Calc lateral deviation from path to target object.
  object_data.lateral =
    tier4_autoware_utils::calcLateralDeviation(object_closest_pose, object_pose.position);

  // Find the footprint point closest to the path, set to object_data.overhang_distance.
  object_data.overhang_dist = utils::avoidance::calcEnvelopeOverhangDistance(
    object_data, object_closest_pose, object_data.overhang_pose.position);

  // Check whether the the ego should avoid the object.
  const auto t = utils::getHighestProbLabel(object.classification);
  const auto object_parameter = avoidance_parameters_->object_parameters.at(t);
  const auto vehicle_width = planner_data_->parameters.vehicle_width;
  const auto safety_margin = 0.5 * vehicle_width + object_parameter.safety_buffer_lateral;
  object_data.avoid_required =
    (utils::avoidance::isOnRight(object_data) &&
     std::abs(object_data.overhang_dist) < safety_margin) ||
    (!utils::avoidance::isOnRight(object_data) && object_data.overhang_dist < safety_margin);

  return object_data;
}
}  // namespace behavior_path_planner
