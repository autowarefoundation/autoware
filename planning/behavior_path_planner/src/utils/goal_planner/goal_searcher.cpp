// Copyright 2022 TIER IV, Inc.
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

#include "behavior_path_planner/utils/goal_planner/goal_searcher.hpp"

#include "behavior_path_planner/utils/goal_planner/util.hpp"
#include "behavior_path_planner/utils/path_safety_checker/objects_filtering.hpp"
#include "behavior_path_planner/utils/path_utils.hpp"
#include "lanelet2_extension/utility/utilities.hpp"
#include "motion_utils/trajectory/path_with_lane_id.hpp"

#include <boost/geometry/algorithms/union.hpp>
#include <boost/optional.hpp>

#include <memory>
#include <vector>

namespace behavior_path_planner
{
using lane_departure_checker::LaneDepartureChecker;
using lanelet::autoware::NoParkingArea;
using lanelet::autoware::NoStoppingArea;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::inverseTransformPose;

GoalSearcher::GoalSearcher(
  const GoalPlannerParameters & parameters, const LinearRing2d & vehicle_footprint,
  const std::shared_ptr<OccupancyGridBasedCollisionDetector> & occupancy_grid_map)
: GoalSearcherBase{parameters},
  vehicle_footprint_{vehicle_footprint},
  occupancy_grid_map_{occupancy_grid_map},
  left_side_parking_{parameters.parking_policy == ParkingPolicy::LEFT_SIDE}
{
}

GoalCandidates GoalSearcher::search(const Pose & original_goal_pose)
{
  GoalCandidates goal_candidates{};

  const auto & route_handler = planner_data_->route_handler;
  const double forward_length = parameters_.forward_goal_search_length;
  const double backward_length = parameters_.backward_goal_search_length;
  const double margin_from_boundary = parameters_.margin_from_boundary;
  const double lateral_offset_interval = parameters_.lateral_offset_interval;
  const double max_lateral_offset = parameters_.max_lateral_offset;
  const double ignore_distance_from_lane_start = parameters_.ignore_distance_from_lane_start;
  const double vehicle_width = planner_data_->parameters.vehicle_width;
  const double base_link2front = planner_data_->parameters.base_link2front;
  const double base_link2rear = planner_data_->parameters.base_link2rear;

  const auto pull_over_lanes = goal_planner_utils::getPullOverLanes(
    *route_handler, left_side_parking_, parameters_.backward_goal_search_length,
    parameters_.forward_goal_search_length);
  auto lanes = utils::getExtendedCurrentLanes(
    planner_data_, backward_length, forward_length,
    /*forward_only_in_route*/ false);
  lanes.insert(lanes.end(), pull_over_lanes.begin(), pull_over_lanes.end());

  const auto goal_arc_coords =
    lanelet::utils::getArcCoordinates(pull_over_lanes, original_goal_pose);
  const double s_start = std::max(0.0, goal_arc_coords.length - backward_length);
  const double s_end = goal_arc_coords.length + forward_length;
  auto center_line_path = utils::resamplePathWithSpline(
    route_handler->getCenterLinePath(pull_over_lanes, s_start, s_end),
    parameters_.goal_search_interval);

  std::vector<Pose> original_search_poses{};  // for search area visualizing
  size_t goal_id = 0;
  for (const auto & p : center_line_path.points) {
    // todo(kosuke55): fix orientation for inverseTransformPoint temporarily
    Pose center_pose = p.point.pose;
    center_pose.orientation =
      tier4_autoware_utils::createQuaternionFromYaw(tf2::getYaw(center_pose.orientation));

    // ignore goal_pose near lane start
    const double distance_from_lane_start =
      lanelet::utils::getArcCoordinates(pull_over_lanes, center_pose).length;
    if (distance_from_lane_start < ignore_distance_from_lane_start) {
      continue;
    }

    const auto distance_from_bound = utils::getSignedDistanceFromBoundary(
      pull_over_lanes, vehicle_width, base_link2front, base_link2rear, center_pose,
      left_side_parking_);
    if (!distance_from_bound) continue;

    const double sign = left_side_parking_ ? -1.0 : 1.0;
    const double offset_from_center_line =
      -distance_from_bound.value() + sign * margin_from_boundary;
    const Pose original_search_pose = calcOffsetPose(center_pose, 0, offset_from_center_line, 0);
    const double longitudinal_distance_from_original_goal =
      std::abs(motion_utils::calcSignedArcLength(
        center_line_path.points, original_goal_pose.position, original_search_pose.position));
    original_search_poses.push_back(original_search_pose);  // for createAreaPolygon
    Pose search_pose{};
    // search goal_pose in lateral direction
    double lateral_offset = 0.0;
    for (double dy = 0; dy <= max_lateral_offset; dy += lateral_offset_interval) {
      lateral_offset = dy;
      search_pose = calcOffsetPose(original_search_pose, 0, sign * dy, 0);

      const auto transformed_vehicle_footprint =
        transformVector(vehicle_footprint_, tier4_autoware_utils::pose2transform(search_pose));

      if (isInAreas(transformed_vehicle_footprint, getNoParkingAreaPolygons(pull_over_lanes))) {
        continue;
      }

      if (isInAreas(transformed_vehicle_footprint, getNoStoppingAreaPolygons(pull_over_lanes))) {
        continue;
      }

      if (LaneDepartureChecker::isOutOfLane(lanes, transformed_vehicle_footprint)) {
        continue;
      }

      GoalCandidate goal_candidate{};
      goal_candidate.goal_pose = search_pose;
      goal_candidate.lateral_offset = lateral_offset;
      goal_candidate.id = goal_id;
      goal_id++;
      // use longitudinal_distance as distance_from_original_goal
      goal_candidate.distance_from_original_goal = longitudinal_distance_from_original_goal;
      goal_candidates.push_back(goal_candidate);
    }
  }
  createAreaPolygons(original_search_poses);

  // Sort with distance from original goal
  std::sort(goal_candidates.begin(), goal_candidates.end());

  return goal_candidates;
}

void GoalSearcher::update(GoalCandidates & goal_candidates) const
{
  // update is_safe
  for (auto & goal_candidate : goal_candidates) {
    const Pose goal_pose = goal_candidate.goal_pose;

    // check collision with footprint
    if (checkCollision(goal_pose)) {
      goal_candidate.is_safe = false;
      continue;
    }

    // check longitudinal margin with pull over lane objects
    const auto pull_over_lanes = goal_planner_utils::getPullOverLanes(
      *(planner_data_->route_handler), left_side_parking_, parameters_.backward_goal_search_length,
      parameters_.forward_goal_search_length);
    const auto [pull_over_lane_objects, others] =
      utils::path_safety_checker::separateObjectsByLanelets(
        *(planner_data_->dynamic_object), pull_over_lanes);
    const auto pull_over_lane_stop_objects = utils::path_safety_checker::filterObjectsByVelocity(
      pull_over_lane_objects, parameters_.th_moving_object_velocity);
    constexpr bool filter_inside = true;
    const auto target_objects = goal_planner_utils::filterObjectsByLateralDistance(
      goal_pose, planner_data_->parameters.vehicle_width, pull_over_lane_stop_objects,
      parameters_.object_recognition_collision_check_margin, filter_inside);
    if (checkCollisionWithLongitudinalDistance(goal_pose, target_objects)) {
      goal_candidate.is_safe = false;
      continue;
    }

    goal_candidate.is_safe = true;
  }
}

bool GoalSearcher::checkCollision(const Pose & pose) const
{
  if (parameters_.use_occupancy_grid) {
    const Pose pose_grid_coords = global2local(occupancy_grid_map_->getMap(), pose);
    const auto idx = pose2index(
      occupancy_grid_map_->getMap(), pose_grid_coords, occupancy_grid_map_->getParam().theta_size);
    const bool check_out_of_range = false;
    if (occupancy_grid_map_->detectCollision(idx, check_out_of_range)) {
      return true;
    }
  }

  if (parameters_.use_object_recognition) {
    const auto pull_over_lanes = goal_planner_utils::getPullOverLanes(
      *(planner_data_->route_handler), left_side_parking_, parameters_.backward_goal_search_length,
      parameters_.forward_goal_search_length);
    const auto [pull_over_lane_objects, others] =
      utils::path_safety_checker::separateObjectsByLanelets(
        *(planner_data_->dynamic_object), pull_over_lanes);
    const auto pull_over_lane_stop_objects = utils::path_safety_checker::filterObjectsByVelocity(
      pull_over_lane_objects, parameters_.th_moving_object_velocity);
    if (utils::checkCollisionBetweenFootprintAndObjects(
          vehicle_footprint_, pose, pull_over_lane_stop_objects,
          parameters_.object_recognition_collision_check_margin)) {
      return true;
    }
  }
  return false;
}

bool GoalSearcher::checkCollisionWithLongitudinalDistance(
  const Pose & ego_pose, const PredictedObjects & dynamic_objects) const
{
  if (parameters_.use_occupancy_grid && parameters_.use_occupancy_grid_for_longitudinal_margin) {
    constexpr bool check_out_of_range = false;
    const double offset = std::max(
      parameters_.longitudinal_margin - parameters_.occupancy_grid_collision_check_margin, 0.0);

    // check forward collision
    const Pose ego_pose_moved_forward = calcOffsetPose(ego_pose, offset, 0, 0);
    const Pose forward_pose_grid_coords =
      global2local(occupancy_grid_map_->getMap(), ego_pose_moved_forward);
    const auto forward_idx = pose2index(
      occupancy_grid_map_->getMap(), forward_pose_grid_coords,
      occupancy_grid_map_->getParam().theta_size);
    if (occupancy_grid_map_->detectCollision(forward_idx, check_out_of_range)) {
      return true;
    }

    // check backward collision
    const Pose ego_pose_moved_backward = calcOffsetPose(ego_pose, -offset, 0, 0);
    const Pose backward_pose_grid_coords =
      global2local(occupancy_grid_map_->getMap(), ego_pose_moved_backward);
    const auto backward_idx = pose2index(
      occupancy_grid_map_->getMap(), backward_pose_grid_coords,
      occupancy_grid_map_->getParam().theta_size);
    if (occupancy_grid_map_->detectCollision(backward_idx, check_out_of_range)) {
      return true;
    }
  }

  if (parameters_.use_object_recognition) {
    if (
      utils::calcLongitudinalDistanceFromEgoToObjects(
        ego_pose, planner_data_->parameters.base_link2front,
        planner_data_->parameters.base_link2rear,
        dynamic_objects) < parameters_.longitudinal_margin) {
      return true;
    }
  }
  return false;
}

void GoalSearcher::createAreaPolygons(std::vector<Pose> original_search_poses)
{
  using tier4_autoware_utils::MultiPolygon2d;
  using tier4_autoware_utils::Point2d;
  using tier4_autoware_utils::Polygon2d;

  const double vehicle_width = planner_data_->parameters.vehicle_width;
  const double base_link2front = planner_data_->parameters.base_link2front;
  const double base_link2rear = planner_data_->parameters.base_link2rear;
  const double max_lateral_offset = parameters_.max_lateral_offset;

  const auto appendPointToPolygon =
    [](Polygon2d & polygon, const geometry_msgs::msg::Point & geom_point) {
      Point2d point{};
      point.x() = geom_point.x;
      point.y() = geom_point.y;
      boost::geometry::append(polygon.outer(), point);
    };

  boost::geometry::clear(area_polygons_);
  for (const auto p : original_search_poses) {
    Polygon2d footprint{};

    const double left_front_offset =
      left_side_parking_ ? vehicle_width / 2 : vehicle_width / 2 + max_lateral_offset;
    const Point p_left_front = calcOffsetPose(p, base_link2front, left_front_offset, 0).position;
    appendPointToPolygon(footprint, p_left_front);

    const double right_front_offset =
      left_side_parking_ ? -vehicle_width / 2 - max_lateral_offset : -vehicle_width / 2;
    const Point p_right_front = calcOffsetPose(p, base_link2front, right_front_offset, 0).position;
    appendPointToPolygon(footprint, p_right_front);

    const double right_back_offset =
      left_side_parking_ ? -vehicle_width / 2 - max_lateral_offset : -vehicle_width / 2;
    const Point p_right_back = calcOffsetPose(p, -base_link2rear, right_back_offset, 0).position;
    appendPointToPolygon(footprint, p_right_back);

    const double left_back_offset =
      left_side_parking_ ? vehicle_width / 2 : vehicle_width / 2 + max_lateral_offset;
    const Point p_left_back = calcOffsetPose(p, -base_link2rear, left_back_offset, 0).position;
    appendPointToPolygon(footprint, p_left_back);

    appendPointToPolygon(footprint, p_left_front);

    MultiPolygon2d current_result{};
    boost::geometry::union_(footprint, area_polygons_, current_result);
    area_polygons_ = current_result;
  }
}

BasicPolygons2d GoalSearcher::getNoParkingAreaPolygons(const lanelet::ConstLanelets & lanes) const
{
  BasicPolygons2d area_polygons{};
  for (const auto & ll : lanes) {
    for (const auto & reg_elem : ll.regulatoryElementsAs<NoParkingArea>()) {
      for (const auto & area : reg_elem->noParkingAreas()) {
        const auto & area_poly = lanelet::utils::to2D(area).basicPolygon();
        area_polygons.push_back(area_poly);
      }
    }
  }
  return area_polygons;
}

BasicPolygons2d GoalSearcher::getNoStoppingAreaPolygons(const lanelet::ConstLanelets & lanes) const
{
  BasicPolygons2d area_polygons{};
  for (const auto & ll : lanes) {
    for (const auto & reg_elem : ll.regulatoryElementsAs<NoStoppingArea>()) {
      for (const auto & area : reg_elem->noStoppingAreas()) {
        const auto & area_poly = lanelet::utils::to2D(area).basicPolygon();
        area_polygons.push_back(area_poly);
      }
    }
  }
  return area_polygons;
}

bool GoalSearcher::isInAreas(const LinearRing2d & footprint, const BasicPolygons2d & areas) const
{
  for (const auto & area : areas) {
    if (boost::geometry::intersects(area, footprint)) {
      return true;
    }
  }
  return false;
}

}  // namespace behavior_path_planner
