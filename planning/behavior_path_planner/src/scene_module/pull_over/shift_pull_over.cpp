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

#include "behavior_path_planner/scene_module/pull_over/shift_pull_over.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/pull_over/util.hpp"

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

#include <memory>
#include <vector>

namespace behavior_path_planner
{
ShiftPullOver::ShiftPullOver(
  rclcpp::Node & node, const PullOverParameters & parameters,
  const LaneDepartureChecker & lane_departure_checker,
  const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map)
: PullOverPlannerBase{node, parameters},
  lane_departure_checker_{lane_departure_checker},
  occupancy_grid_map_{occupancy_grid_map}
{
}

boost::optional<PullOverPath> ShiftPullOver::plan(const Pose & goal_pose)
{
  const auto & route_handler = planner_data_->route_handler;

  const auto road_lanes = util::getExtendedCurrentLanes(planner_data_);
  const auto shoulder_lanes = pull_over_utils::getPullOverLanes(*route_handler);
  if (road_lanes.empty() || shoulder_lanes.empty()) {
    return {};
  }

  lanelet::ConstLanelets lanes;
  lanes.insert(lanes.end(), road_lanes.begin(), road_lanes.end());
  lanes.insert(lanes.end(), shoulder_lanes.begin(), shoulder_lanes.end());

  // generate candidate paths
  const auto pull_over_paths = generatePullOverPaths(road_lanes, shoulder_lanes, goal_pose);
  if (pull_over_paths.empty()) {
    return {};
  }

  // select valid paths which have enough distance and no lane departure
  const auto valid_paths = selectValidPaths(
    pull_over_paths, road_lanes, shoulder_lanes,
    route_handler->isInGoalRouteSection(road_lanes.back()), goal_pose);
  if (valid_paths.empty()) {
    return {};
  }

  // select safe path
  for (const auto & path : valid_paths) {
    if (parameters_.use_occupancy_grid || !occupancy_grid_map_) {
      const bool check_out_of_range = false;
      if (occupancy_grid_map_->hasObstacleOnPath(path.shifted_path.path, check_out_of_range)) {
        continue;
      }
    }

    if (parameters_.use_object_recognition) {
      if (util::checkCollisionBetweenPathFootprintsAndObjects(
            vehicle_footprint_, path.shifted_path.path, *(planner_data_->dynamic_object),
            parameters_.object_recognition_collision_check_margin)) {
        continue;
      }
    }

    // found safe path
    return path;
  }

  // not found safe path
  return {};
}

std::vector<PullOverPath> ShiftPullOver::generatePullOverPaths(
  const lanelet::ConstLanelets & road_lanes, const lanelet::ConstLanelets & shoulder_lanes,
  const Pose & goal_pose) const
{
  // rename parameter
  const auto & route_handler = planner_data_->route_handler;
  const auto & common_parameters = planner_data_->parameters;
  const Pose & current_pose = planner_data_->self_pose->pose;
  const double backward_path_length = common_parameters.backward_path_length;
  const double pull_over_velocity = parameters_.pull_over_velocity;
  const double after_pull_over_straight_distance = parameters_.after_pull_over_straight_distance;
  const double minimum_lateral_jerk = parameters_.minimum_lateral_jerk;
  const double maximum_lateral_jerk = parameters_.maximum_lateral_jerk;
  const double deceleration_interval = parameters_.deceleration_interval;
  const int pull_over_sampling_num = parameters_.pull_over_sampling_num;
  const double jerk_resolution =
    std::abs(maximum_lateral_jerk - minimum_lateral_jerk) / pull_over_sampling_num;

  // calc lateral offset from road lane center line to shoulder target line.
  lanelet::ConstLanelet goal_closest_road_lane;
  lanelet::utils::query::getClosestLanelet(road_lanes, goal_pose, &goal_closest_road_lane);
  const auto closest_center_pose =
    lanelet::utils::getClosestCenterPose(goal_closest_road_lane, goal_pose.position);
  const double distance_from_shoulder_left_bound =
    util::getDistanceToShoulderBoundary(shoulder_lanes, closest_center_pose);
  const double margin_from_boundary =
    std::abs(util::getDistanceToShoulderBoundary(shoulder_lanes, goal_pose));
  const double offset_from_road_line_center =
    distance_from_shoulder_left_bound + margin_from_boundary;

  // shift end point in shoulder lane
  const auto shift_end_point = std::invoke([&]() {
    const auto arc_position_goal = lanelet::utils::getArcCoordinates(shoulder_lanes, goal_pose);
    const double s_start =
      std::max(arc_position_goal.length - after_pull_over_straight_distance, 0.0);
    const double s_end = s_start + std::numeric_limits<double>::epsilon();
    const auto path = route_handler->getCenterLinePath(shoulder_lanes, s_start, s_end, true);
    return path.points.front();
  });

  std::vector<PullOverPath> candidate_paths;
  for (double lateral_jerk = minimum_lateral_jerk; lateral_jerk <= maximum_lateral_jerk;
       lateral_jerk += jerk_resolution) {
    PathShifter path_shifter;
    ShiftedPath shifted_path;
    PullOverPath candidate_path;

    const double pull_over_distance = path_shifter.calcLongitudinalDistFromJerk(
      std::abs(offset_from_road_line_center), lateral_jerk, pull_over_velocity);

    // calculate straight distance before pull over
    const double straight_distance = std::invoke([&]() {
      const auto arc_position_goal = lanelet::utils::getArcCoordinates(road_lanes, goal_pose);
      const auto arc_position_pose = lanelet::utils::getArcCoordinates(road_lanes, current_pose);
      return arc_position_goal.length - after_pull_over_straight_distance - pull_over_distance -
             arc_position_pose.length;
    });

    PathWithLaneId road_lane_reference_path;
    {
      const auto arc_position = lanelet::utils::getArcCoordinates(road_lanes, current_pose);
      const auto arc_position_ref2_front =
        lanelet::utils::getArcCoordinates(road_lanes, shift_end_point.point.pose);
      const double s_start = arc_position.length - backward_path_length;
      const double s_end = arc_position_ref2_front.length - pull_over_distance;
      road_lane_reference_path = route_handler->getCenterLinePath(road_lanes, s_start, s_end);
      // decelerate velocity linearly to minimum pull over velocity
      // ( or accelerate if original velocity is lower than minimum velocity )
      for (auto & point : road_lane_reference_path.points) {
        const auto arclength =
          lanelet::utils::getArcCoordinates(road_lanes, point.point.pose).length;
        const double distance_to_pull_over_start = std::max(0.0, s_end - arclength);
        point.point.longitudinal_velocity_mps = std::min(
          point.point.longitudinal_velocity_mps,
          static_cast<float>(
            (distance_to_pull_over_start / deceleration_interval) *
              (point.point.longitudinal_velocity_mps - pull_over_velocity) +
            pull_over_velocity));
      }
    }
    // resample road straight path and shift source path respectively
    constexpr double resample_interval{1.0};
    road_lane_reference_path =
      util::resamplePathWithSpline(road_lane_reference_path, resample_interval);

    if (road_lane_reference_path.points.empty()) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("behavior_path_planner").get_child("pull_over").get_child("util"),
        "reference path is empty!! something wrong...");
      continue;
    }

    PathWithLaneId target_lane_reference_path;
    {
      const lanelet::ArcCoordinates pull_over_start_arc_position =
        lanelet::utils::getArcCoordinates(
          shoulder_lanes, road_lane_reference_path.points.back().point.pose);
      const double s_start = pull_over_start_arc_position.length;
      const auto arc_position_goal = lanelet::utils::getArcCoordinates(shoulder_lanes, goal_pose);
      const double s_end = arc_position_goal.length;
      target_lane_reference_path = route_handler->getCenterLinePath(shoulder_lanes, s_start, s_end);
      // distance between shoulder lane's left boundary and shoulder lane center
      const double distance_shoulder_to_left_bound =
        util::getDistanceToShoulderBoundary(shoulder_lanes, shift_end_point.point.pose);

      // distance between shoulder lane center and target line
      const double distance_shoulder_to_target =
        distance_shoulder_to_left_bound + margin_from_boundary;

      // Apply shifting shoulder lane to adjust to target line
      const double offset = -distance_shoulder_to_target;
      for (size_t i = 0; i < target_lane_reference_path.points.size(); ++i) {
        {
          auto & p = target_lane_reference_path.points.at(i).point.pose;
          p = tier4_autoware_utils::calcOffsetPose(p, 0, offset, 0);
        }
      }
    }
    path_shifter.setPath(
      util::resamplePathWithSpline(target_lane_reference_path, resample_interval));

    ShiftPoint shift_point;
    {
      shift_point.start = road_lane_reference_path.points.back().point.pose;
      shift_point.end = shift_end_point.point.pose;

      // distance between shoulder lane's left boundary and current lane center
      const double distance_road_to_left_boundary = util::getDistanceToShoulderBoundary(
        shoulder_lanes, road_lane_reference_path.points.back().point.pose);
      // distance between shoulder lane's left boundary and current lane center
      const double distance_road_to_target = distance_road_to_left_boundary + margin_from_boundary;

      shift_point.length = distance_road_to_target;
      path_shifter.addShiftPoint(shift_point);
    }

    // offset front side from reference path
    const bool offset_back = false;
    if (!path_shifter.generate(&shifted_path, offset_back)) {
      continue;
    }

    const auto shift_end_idx =
      motion_utils::findNearestIndex(shifted_path.path.points, shift_end_point.point.pose);
    const auto goal_idx = motion_utils::findNearestIndex(shifted_path.path.points, goal_pose);
    if (shift_end_idx && goal_idx) {
      // get target shoulder lane
      lanelet::ConstLanelet target_shoulder_lanelet;
      lanelet::utils::query::getClosestLanelet(
        shoulder_lanes, shifted_path.path.points.back().point.pose, &target_shoulder_lanelet);

      for (size_t i = 0; i < shifted_path.path.points.size(); ++i) {
        auto & point = shifted_path.path.points.at(i);

        // add road lane_ids if not found
        for (const auto id : road_lane_reference_path.points.back().lane_ids) {
          if (std::find(point.lane_ids.begin(), point.lane_ids.end(), id) == point.lane_ids.end()) {
            point.lane_ids.push_back(id);
          }
        }

        // add shoulder lane_id if not found
        if (
          std::find(point.lane_ids.begin(), point.lane_ids.end(), target_shoulder_lanelet.id()) ==
          point.lane_ids.end()) {
          point.lane_ids.push_back(target_shoulder_lanelet.id());
        }

        // set velocity
        if (i < *shift_end_idx) {
          // set velocity during shift
          point.point.longitudinal_velocity_mps = std::min(
            point.point.longitudinal_velocity_mps,
            road_lane_reference_path.points.back().point.longitudinal_velocity_mps);
          continue;
        } else if (i >= *goal_idx) {
          // set velocity after goal
          point.point.longitudinal_velocity_mps = 0.0;
          continue;
        }
        point.point.longitudinal_velocity_mps = pull_over_velocity;
      }

      candidate_path.straight_path = road_lane_reference_path;
      candidate_path.path =
        pull_over_utils::combineReferencePath(road_lane_reference_path, shifted_path.path);
      // shift path is connected to one, so partial_paths have only one
      candidate_path.partial_paths.push_back(
        pull_over_utils::combineReferencePath(road_lane_reference_path, shifted_path.path));
      candidate_path.shifted_path = shifted_path;
      shift_point.start_idx = path_shifter.getShiftPoints().front().start_idx;
      shift_point.end_idx = path_shifter.getShiftPoints().front().end_idx;
      candidate_path.start_pose = path_shifter.getShiftPoints().front().start;
      candidate_path.end_pose = path_shifter.getShiftPoints().front().end;
      candidate_path.shifted_path.shift_length = shifted_path.shift_length;
      candidate_path.shift_point = shift_point;
      candidate_path.preparation_length = straight_distance;
      candidate_path.pull_over_length = pull_over_distance;
    } else {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("behavior_path_planner").get_child("pull_over").get_child("util"),
        "lane change end idx not found on target path.");
      continue;
    }

    candidate_paths.push_back(candidate_path);
  }

  return candidate_paths;
}

std::vector<PullOverPath> ShiftPullOver::selectValidPaths(
  const std::vector<PullOverPath> & paths, const lanelet::ConstLanelets & road_lanes,
  const lanelet::ConstLanelets & shoulder_lanes, const bool is_in_goal_route_section,
  const Pose & goal_pose) const
{
  // combine road and shoulder lanes
  lanelet::ConstLanelets lanes = road_lanes;
  lanes.insert(lanes.end(), shoulder_lanes.begin(), shoulder_lanes.end());

  std::vector<PullOverPath> available_paths;
  for (const auto & path : paths) {
    if (!hasEnoughDistance(path, road_lanes, is_in_goal_route_section, goal_pose)) {
      continue;
    }

    if (lane_departure_checker_.checkPathWillLeaveLane(lanes, path.shifted_path.path)) {
      continue;
    }

    available_paths.push_back(path);
  }

  return available_paths;
}

bool ShiftPullOver::hasEnoughDistance(
  const PullOverPath & path, const lanelet::ConstLanelets & road_lanes,
  const bool is_in_goal_route_section, const Pose & goal_pose) const
{
  const auto & current_pose = planner_data_->self_pose->pose;
  const double current_vel = planner_data_->self_odometry->twist.twist.linear.x;

  if (!pull_over_utils::hasEnoughDistanceToParkingStart(
        path.path, current_pose, path.start_pose, current_vel, parameters_.maximum_deceleration,
        parameters_.decide_path_distance, planner_data_->parameters.ego_nearest_dist_threshold,
        planner_data_->parameters.ego_nearest_yaw_threshold)) {
    return false;
  }

  const double pull_over_prepare_distance = path.preparation_length;
  const double pull_over_distance = path.pull_over_length;
  const double pull_over_total_distance = pull_over_prepare_distance + pull_over_distance;

  if (pull_over_total_distance > util::getDistanceToEndOfLane(current_pose, road_lanes)) {
    return false;
  }

  if (
    is_in_goal_route_section &&
    pull_over_total_distance > util::getSignedDistance(current_pose, goal_pose, road_lanes)) {
    return false;
  }

  return true;
}

}  // namespace behavior_path_planner
