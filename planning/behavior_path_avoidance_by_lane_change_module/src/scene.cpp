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

#include "behavior_path_avoidance_by_lane_change_module/scene.hpp"

#include "behavior_path_avoidance_module/utils.hpp"
#include "behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"
#include "behavior_path_planner_common/utils/path_utils.hpp"
#include "behavior_path_planner_common/utils/utils.hpp"

#include <behavior_path_avoidance_module/data_structs.hpp>
#include <behavior_path_lane_change_module/utils/utils.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/logging.hpp>

#include <boost/geometry/algorithms/centroid.hpp>
#include <boost/geometry/strategies/cartesian/centroid_bashein_detmer.hpp>

#include <limits>
#include <optional>
#include <utility>

namespace behavior_path_planner
{
using behavior_path_planner::utils::lane_change::debug::createExecutionArea;

AvoidanceByLaneChange::AvoidanceByLaneChange(
  const std::shared_ptr<LaneChangeParameters> & parameters,
  std::shared_ptr<AvoidanceByLCParameters> avoidance_parameters)
: NormalLaneChange(parameters, LaneChangeModuleType::AVOIDANCE_BY_LANE_CHANGE, Direction::NONE),
  avoidance_parameters_(std::move(avoidance_parameters)),
  avoidance_helper_{std::make_shared<AvoidanceHelper>(avoidance_parameters_)}
{
}

bool AvoidanceByLaneChange::specialRequiredCheck() const
{
  const auto & data = avoidance_data_;

  if (data.target_objects.empty()) {
    RCLCPP_DEBUG(logger_, "no empty objects");
    return false;
  }

  const auto & object_parameters = avoidance_parameters_->object_parameters;

  const auto count_target_object = [&](const auto sum, const auto & p) {
    const auto & objects = avoidance_data_.target_objects;

    const auto is_avoidance_target = [&p](const auto & object) {
      const auto target_class = utils::getHighestProbLabel(object.object.classification) == p.first;
      return target_class && object.avoid_required;
    };

    return sum + std::count_if(objects.begin(), objects.end(), is_avoidance_target);
  };
  const auto num_of_avoidance_targets =
    std::accumulate(object_parameters.begin(), object_parameters.end(), 0UL, count_target_object);

  if (num_of_avoidance_targets < 1) {
    RCLCPP_DEBUG(logger_, "no avoidance target");
    return false;
  }

  const auto & nearest_object = data.target_objects.front();
  const auto minimum_avoid_length = calcMinAvoidanceLength(nearest_object);
  const auto minimum_lane_change_length = calcMinimumLaneChangeLength();

  lane_change_debug_.execution_area = createExecutionArea(
    getCommonParam().vehicle_info, getEgoPose(),
    std::max(minimum_lane_change_length, minimum_avoid_length), calcLateralOffset());

  RCLCPP_DEBUG(
    logger_, "Conditions ? %f, %f, %f", nearest_object.longitudinal, minimum_lane_change_length,
    minimum_avoid_length);
  return nearest_object.longitudinal > std::max(minimum_lane_change_length, minimum_avoid_length);
}

bool AvoidanceByLaneChange::specialExpiredCheck() const
{
  return !specialRequiredCheck();
}

void AvoidanceByLaneChange::updateSpecialData()
{
  const auto p = std::dynamic_pointer_cast<AvoidanceParameters>(avoidance_parameters_);

  avoidance_debug_data_ = DebugData();
  avoidance_data_ = calcAvoidancePlanningData(avoidance_debug_data_);

  if (avoidance_data_.target_objects.empty()) {
    direction_ = Direction::NONE;
  } else {
    direction_ = utils::avoidance::isOnRight(avoidance_data_.target_objects.front())
                   ? Direction::LEFT
                   : Direction::RIGHT;
  }

  utils::avoidance::updateRegisteredObject(registered_objects_, avoidance_data_.target_objects, p);
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

  data.reference_path_rough = prev_module_output_.path;

  const auto resample_interval = avoidance_parameters_->resample_interval_for_planning;
  data.reference_path = utils::resamplePathWithSpline(data.reference_path_rough, resample_interval);

  data.current_lanelets = getCurrentLanes();

  fillAvoidanceTargetObjects(data, debug);

  return data;
}

void AvoidanceByLaneChange::fillAvoidanceTargetObjects(
  AvoidancePlanningData & data, [[maybe_unused]] DebugData & debug) const
{
  const auto p = std::dynamic_pointer_cast<AvoidanceParameters>(avoidance_parameters_);

  const auto [object_within_target_lane, object_outside_target_lane] =
    utils::path_safety_checker::separateObjectsByLanelets(
      *planner_data_->dynamic_object, data.current_lanelets,
      utils::path_safety_checker::isPolygonOverlapLanelet);

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
  for (const auto & obj : object_within_target_lane.objects) {
    const auto target_lane_object = createObjectData(data, obj);
    if (!target_lane_object) {
      continue;
    }

    target_lane_objects.push_back(*target_lane_object);
  }

  data.target_objects = target_lane_objects;
}

std::optional<ObjectData> AvoidanceByLaneChange::createObjectData(
  const AvoidancePlanningData & data, const PredictedObject & object) const
{
  using boost::geometry::return_centroid;
  using motion_utils::findNearestIndex;
  using tier4_autoware_utils::calcDistance2d;
  using tier4_autoware_utils::calcLateralDeviation;

  const auto p = std::dynamic_pointer_cast<AvoidanceParameters>(avoidance_parameters_);

  const auto & path_points = data.reference_path.points;
  const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
  const auto object_closest_index = findNearestIndex(path_points, object_pose.position);
  const auto object_closest_pose = path_points.at(object_closest_index).point.pose;
  const auto t = utils::getHighestProbLabel(object.classification);
  const auto & object_parameter = avoidance_parameters_->object_parameters.at(t);

  ObjectData object_data{};
  // Calc lateral deviation from path to target object.
  object_data.to_centerline =
    lanelet::utils::getArcCoordinates(data.current_lanelets, object_pose).distance;

  if (
    std::abs(object_data.to_centerline) <
    avoidance_parameters_->threshold_distance_object_is_on_center) {
    return std::nullopt;
  }

  object_data.object = object;

  const auto lower = p->lower_distance_for_polygon_expansion;
  const auto upper = p->upper_distance_for_polygon_expansion;
  const auto clamp =
    std::clamp(calcDistance2d(getEgoPose(), object_pose) - lower, 0.0, upper) / upper;
  object_data.distance_factor = object_parameter.max_expand_ratio * clamp + 1.0;

  // Calc envelop polygon.
  utils::avoidance::fillObjectEnvelopePolygon(
    object_data, registered_objects_, object_closest_pose, p);

  // calc object centroid.
  object_data.centroid = return_centroid<Point2d>(object_data.envelope_poly);

  // Calc moving time.
  utils::avoidance::fillObjectMovingTime(object_data, stopped_objects_, p);

  object_data.direction = calcLateralDeviation(object_closest_pose, object_pose.position) > 0.0
                            ? Direction::LEFT
                            : Direction::RIGHT;

  // Find the footprint point closest to the path, set to object_data.overhang_distance.
  object_data.overhang_points =
    utils::avoidance::calcEnvelopeOverhangDistance(object_data, data.reference_path);

  // Check whether the the ego should avoid the object.
  const auto & vehicle_width = planner_data_->parameters.vehicle_width;
  utils::avoidance::fillAvoidanceNecessity(object_data, registered_objects_, vehicle_width, p);

  utils::avoidance::fillLongitudinalAndLengthByClosestEnvelopeFootprint(
    data.reference_path_rough, getEgoPosition(), object_data);
  return object_data;
}

double AvoidanceByLaneChange::calcMinAvoidanceLength(const ObjectData & nearest_object) const
{
  const auto ego_width = getCommonParam().vehicle_width;
  const auto nearest_object_type = utils::getHighestProbLabel(nearest_object.object.classification);
  const auto nearest_object_parameter =
    avoidance_parameters_->object_parameters.at(nearest_object_type);
  const auto lateral_hard_margin = std::max(
    nearest_object_parameter.lateral_hard_margin,
    nearest_object_parameter.lateral_hard_margin_for_parked_vehicle);
  const auto avoid_margin = lateral_hard_margin * nearest_object.distance_factor +
                            nearest_object_parameter.lateral_soft_margin + 0.5 * ego_width;

  avoidance_helper_->setData(planner_data_);
  const auto shift_length = avoidance_helper_->getShiftLength(
    nearest_object, utils::avoidance::isOnRight(nearest_object), avoid_margin);

  return avoidance_helper_->getMinAvoidanceDistance(shift_length);
}

double AvoidanceByLaneChange::calcMinimumLaneChangeLength() const
{
  const auto current_lanes = getCurrentLanes();
  if (current_lanes.empty()) {
    RCLCPP_DEBUG(logger_, "no empty lanes");
    return std::numeric_limits<double>::infinity();
  }

  return utils::lane_change::calcMinimumLaneChangeLength(
    getRouteHandler(), current_lanes.back(), *lane_change_parameters_, direction_);
}

double AvoidanceByLaneChange::calcLateralOffset() const
{
  auto additional_lat_offset{0.0};
  for (const auto & [type, p] : avoidance_parameters_->object_parameters) {
    const auto lateral_hard_margin =
      std::max(p.lateral_hard_margin, p.lateral_hard_margin_for_parked_vehicle);
    const auto offset =
      2.0 * p.envelope_buffer_margin + lateral_hard_margin + p.lateral_soft_margin;
    additional_lat_offset = std::max(additional_lat_offset, offset);
  }
  return additional_lat_offset;
}
}  // namespace behavior_path_planner
