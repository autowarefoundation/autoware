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

#include "behavior_path_planner/scene_module/pull_out/util.hpp"

#include "behavior_path_planner/path_shifter/path_shifter.hpp"
#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/util/create_vehicle_footprint.hpp"

#include <autoware_utils/geometry/boost_geometry.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/geometry/algorithms/dispatch/distance.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{
namespace pull_out_utils
{
PathWithLaneId combineReferencePath(const PathWithLaneId path1, const PathWithLaneId path2)
{
  PathWithLaneId path;
  path.points.insert(path.points.end(), path1.points.begin(), path1.points.end());

  // skip overlapping point
  path.points.insert(path.points.end(), next(path2.points.begin()), path2.points.end());

  return path;
}

bool isPathInLanelets4pullover(
  const PathWithLaneId & path, const lanelet::ConstLanelets & original_lanelets,
  const lanelet::ConstLanelets & target_lanelets)
{
  for (const auto & pt : path.points) {
    bool is_in_lanelet = false;
    const lanelet::BasicPoint2d p(pt.point.pose.position.x, pt.point.pose.position.y);
    for (const auto & llt : original_lanelets) {
      if (lanelet::utils::isInLanelet(pt.point.pose, llt, 1)) {
        is_in_lanelet = true;
      }
    }
    for (const auto & llt : target_lanelets) {
      if (lanelet::utils::isInLanelet(pt.point.pose, llt, 1)) {
        is_in_lanelet = true;
      }
    }
    if (!is_in_lanelet) {
      return false;
    }
  }
  return true;
}

std::vector<PullOutPath> getPullOutPaths(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & road_lanelets,
  const lanelet::ConstLanelets & shoulder_lanelets, const Pose & pose,
  const BehaviorPathPlannerParameters & common_parameter, const PullOutParameters & parameter,
  const bool is_retreat_path)
{
  std::vector<PullOutPath> candidate_paths;

  if (road_lanelets.empty() || shoulder_lanelets.empty()) {
    return candidate_paths;
  }
  // rename parameter
  const double backward_path_length = common_parameter.backward_path_length;
  const double forward_path_length = common_parameter.forward_path_length;
  const double minimum_pull_out_velocity = parameter.minimum_pull_out_velocity;
  const double before_pull_out_straight_distance = parameter.before_pull_out_straight_distance;
  const double minimum_lateral_jerk = parameter.minimum_lateral_jerk;
  const double maximum_lateral_jerk = parameter.maximum_lateral_jerk;
  const int pull_out_sampling_num = parameter.pull_out_sampling_num;
  const double jerk_resolution =
    std::abs(maximum_lateral_jerk - minimum_lateral_jerk) / pull_out_sampling_num;
  // maximum lateral jerk is set on retreat path
  double initial_lateral_jerk = is_retreat_path ? maximum_lateral_jerk : minimum_lateral_jerk;

  for (double lateral_jerk = initial_lateral_jerk; lateral_jerk <= maximum_lateral_jerk;
       lateral_jerk += jerk_resolution) {
    PathWithLaneId reference_path;
    PathShifter path_shifter;
    ShiftedPath shifted_path;
    const double v1 = minimum_pull_out_velocity;

    const double distance_to_road_center =
      lanelet::utils::getArcCoordinates(road_lanelets, pose).distance;

    const double distance_to_shoulder_center =
      lanelet::utils::getArcCoordinates(shoulder_lanelets, pose).distance;

    double pull_out_distance =
      path_shifter.calcLongitudinalDistFromJerk(abs(distance_to_road_center), lateral_jerk, v1);

    PathWithLaneId reference_path1;
    {
      const auto arc_position = lanelet::utils::getArcCoordinates(shoulder_lanelets, pose);
      const double s_start = arc_position.length - backward_path_length;
      double s_end = arc_position.length + before_pull_out_straight_distance;
      s_end = std::max(s_end, s_start + std::numeric_limits<double>::epsilon());
      reference_path1 = route_handler.getCenterLinePath(shoulder_lanelets, s_start, s_end);
    }
    for (auto & point : reference_path1.points) {
      point.point.longitudinal_velocity_mps = std::min(
        point.point.longitudinal_velocity_mps, static_cast<float>(minimum_pull_out_velocity));
    }

    // Apply shifting before shift
    for (size_t i = 0; i < reference_path1.points.size(); ++i) {
      {
        if (fabs(distance_to_shoulder_center) < 1.0e-8) {
          RCLCPP_WARN_STREAM(
            rclcpp::get_logger("behavior_path_planner").get_child("pull_out").get_child("util"),
            "no offset from current lane center.");
        }

        auto & p = reference_path1.points.at(i).point.pose;
        double yaw = tf2::getYaw(p.orientation);
        p.position.x -= std::sin(yaw) * distance_to_shoulder_center;
        p.position.y += std::cos(yaw) * distance_to_shoulder_center;
      }
    }

    PathWithLaneId reference_path2;
    {
      const auto arc_position_goal =
        lanelet::utils::getArcCoordinates(shoulder_lanelets, route_handler.getGoalPose());
      const auto arc_position_ref1_back =
        lanelet::utils::getArcCoordinates(road_lanelets, reference_path1.points.back().point.pose);
      double s_start = arc_position_ref1_back.length + pull_out_distance;
      double s_end = arc_position_goal.length;
      s_end = std::max(s_end, s_start + std::numeric_limits<double>::epsilon());
      reference_path2 = route_handler.getCenterLinePath(road_lanelets, s_start, s_end);
    }

    if (reference_path1.points.empty() || reference_path2.points.empty()) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("behavior_path_planner").get_child("pull_out").get_child("util"),
        "reference path is empty!! something wrong...");
      continue;
    }

    PullOutPath candidate_path;
    // candidate_path.acceleration = acceleration;
    candidate_path.preparation_length = before_pull_out_straight_distance;
    candidate_path.pull_out_length = pull_out_distance;
    PathWithLaneId target_lane_reference_path;
    {
      const lanelet::ArcCoordinates pull_out_start_arc_position =
        lanelet::utils::getArcCoordinates(road_lanelets, reference_path1.points.back().point.pose);
      double s_start = pull_out_start_arc_position.length;
      double s_end = s_start + pull_out_distance + forward_path_length;
      target_lane_reference_path = route_handler.getCenterLinePath(road_lanelets, s_start, s_end);
    }

    ShiftPoint shift_point;
    {
      const Pose pull_out_start_on_shoulder_lane = reference_path1.points.back().point.pose;
      const Pose pull_out_end_on_road_lane = reference_path2.points.front().point.pose;
      shift_point.start = pull_out_start_on_shoulder_lane;
      shift_point.end = pull_out_end_on_road_lane;

      shift_point.length = distance_to_road_center;
    }
    path_shifter.addShiftPoint(shift_point);
    path_shifter.setPath(target_lane_reference_path);

    // offset front side
    bool offset_back = false;
    if (!path_shifter.generate(&shifted_path, offset_back)) {
      // ROS_ERROR("failed to generate shifted path.");
      continue;
    }

    const auto pull_out_end_idx = autoware_utils::findNearestIndex(
      shifted_path.path.points, reference_path2.points.front().point.pose);

    const auto goal_idx =
      autoware_utils::findNearestIndex(shifted_path.path.points, route_handler.getGoalPose());

    if (pull_out_end_idx && goal_idx) {
      const auto distance_pull_out_end_to_goal = autoware_utils::calcDistance2d(
        shifted_path.path.points.at(*pull_out_end_idx).point.pose,
        shifted_path.path.points.at(*goal_idx).point.pose);
      for (size_t i = 0; i < shifted_path.path.points.size(); ++i) {
        auto & point = shifted_path.path.points.at(i);
        if (i < *pull_out_end_idx) {
          point.point.longitudinal_velocity_mps = std::min(
            point.point.longitudinal_velocity_mps,
            reference_path1.points.back().point.longitudinal_velocity_mps);
          continue;
        } else if (i > *goal_idx) {
          point.point.longitudinal_velocity_mps = 0.0;
          continue;
        }

        auto distance_to_goal = autoware_utils::calcDistance2d(
          point.point.pose, shifted_path.path.points.at(*goal_idx).point.pose);
        point.point.longitudinal_velocity_mps = std::min(
          minimum_pull_out_velocity,
          std::max(
            0.0, (distance_to_goal / distance_pull_out_end_to_goal * minimum_pull_out_velocity)));
        point.lane_ids = reference_path2.points.front().lane_ids;
      }
      candidate_path.path = combineReferencePath(reference_path1, shifted_path.path);
      candidate_path.shifted_path = shifted_path;
      candidate_path.shift_point = shift_point;
    } else {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("behavior_path_planner").get_child("pull_out").get_child("util"),
        "lane change end idx not found on target path.");
      continue;
    }

    // check candidate path is in lanelet
    if (!isPathInLanelets4pullover(candidate_path.path, road_lanelets, shoulder_lanelets)) {
      continue;
    }

    // ROS_ERROR("candidate path is push backed");
    candidate_paths.push_back(candidate_path);
  }

  return candidate_paths;
}

PullOutPath getBackPaths(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & shoulder_lanelets,
  const Pose & pose, const BehaviorPathPlannerParameters & common_parameter,
  [[maybe_unused]] const PullOutParameters & parameter, [[maybe_unused]] const double back_distance)
{
  PathShifter path_shifter;
  ShiftedPath shifted_path;

  // rename parameter
  const double backward_path_length = common_parameter.backward_path_length;

  const double distance_to_shoulder_center =
    lanelet::utils::getArcCoordinates(shoulder_lanelets, pose).distance;

  PathWithLaneId reference_path1;
  {
    const auto arc_position = lanelet::utils::getArcCoordinates(shoulder_lanelets, pose);
    const double s_start = arc_position.length + backward_path_length;
    double s_end = s_start - 50;
    reference_path1 = route_handler.getCenterLinePath(shoulder_lanelets, s_end, s_start);
    // ROS_ERROR("ref1 s_start:%f s_end%f", s_start, s_end);
    for (auto & point : reference_path1.points) {
      // auto arc_length =
      //   lanelet::utils::getArcCoordinates(shoulder_lanelets, point.point.pose).length;
      point.point.longitudinal_velocity_mps = -5;
      // ROS_ERROR("back_distance:%f", back_distance);
      // if (arc_position.length - arc_length > back_distance) {
      //   point.point.longitudinal_velocity_mps = 0;
      // }
    }
    // std::reverse(reference_path1.points.begin(), reference_path1.points.end());
    // reference_path1.points.front().point.longitudinal_velocity_mps = 0;
  }

  // Apply shifting before shift
  for (size_t i = 0; i < reference_path1.points.size(); ++i) {
    {
      if (fabs(distance_to_shoulder_center) < 1.0e-8) {
        RCLCPP_WARN_STREAM(
          rclcpp::get_logger("behavior_path_planner").get_child("pull_out").get_child("util"),
          "no offset from current lane center.");
        continue;
      }

      auto & p = reference_path1.points.at(i).point.pose;
      double yaw = tf2::getYaw(p.orientation);
      p.position.x -= std::sin(yaw) * distance_to_shoulder_center;
      p.position.y += std::cos(yaw) * distance_to_shoulder_center;
    }
  }

  PullOutPath candidate_path;

  candidate_path.path = reference_path1;

  return candidate_path;
}

Pose getBackedPose(
  const Pose & current_pose, const double & yaw_shoulder_lane, const double & back_distance)
{
  Pose backed_pose;
  backed_pose = current_pose;
  backed_pose.position.x -= std::cos(yaw_shoulder_lane) * back_distance;
  backed_pose.position.y -= std::sin(yaw_shoulder_lane) * back_distance;

  return backed_pose;
}

std::vector<PullOutPath> selectValidPaths(
  const std::vector<PullOutPath> & paths, const lanelet::ConstLanelets & road_lanes,
  const lanelet::ConstLanelets & shoulder_lanes,
  const lanelet::routing::RoutingGraphContainer & overall_graphs, const Pose & current_pose,
  const bool isInGoalRouteSection, const Pose & goal_pose)
{
  std::vector<PullOutPath> available_paths;

  for (const auto & path : paths) {
    if (hasEnoughDistance(
          path, road_lanes, shoulder_lanes, current_pose, isInGoalRouteSection, goal_pose,
          overall_graphs)) {
      available_paths.push_back(path);
    }
  }

  return available_paths;
}

bool selectSafePath(
  const std::vector<PullOutPath> & paths, const lanelet::ConstLanelets & road_lanes,
  const lanelet::ConstLanelets & shoulder_lanes,
  const PredictedObjects::ConstSharedPtr & dynamic_objects,
  [[maybe_unused]] const Pose & current_pose, [[maybe_unused]] const Twist & current_twist,
  [[maybe_unused]] const double vehicle_width, const PullOutParameters & ros_parameters,
  const autoware_utils::LinearRing2d & local_vehicle_footprint, PullOutPath * selected_path)
{
  const bool use_dynamic_object = ros_parameters.use_dynamic_object;
  for (const auto & path : paths) {
    if (isPullOutPathSafe(
          path, road_lanes, shoulder_lanes, dynamic_objects, ros_parameters,
          local_vehicle_footprint, true, use_dynamic_object)) {
      *selected_path = path;
      return true;
    }
  }

  // set first path for force pullover if no valid path found
  if (!paths.empty()) {
    *selected_path = paths.front();
    return false;
  }

  return false;
}

bool hasEnoughDistance(
  const PullOutPath & path, const lanelet::ConstLanelets & road_lanes,
  [[maybe_unused]] const lanelet::ConstLanelets & target_lanes, const Pose & current_pose,
  const bool isInGoalRouteSection, const Pose & goal_pose,
  [[maybe_unused]] const lanelet::routing::RoutingGraphContainer & overall_graphs)
{
  const double pull_out_prepare_distance = path.preparation_length;
  const double pull_out_distance = path.pull_out_length;
  const double pull_out_total_distance = pull_out_prepare_distance + pull_out_distance;

  if (pull_out_total_distance > util::getDistanceToEndOfLane(current_pose, road_lanes)) {
    return false;
  }

  // if (pull_out_total_distance >
  // util::getDistanceToNextIntersection(current_pose, current_lanes)) {
  //   return false;
  // }

  if (
    isInGoalRouteSection &&
    pull_out_total_distance > util::getSignedDistance(current_pose, goal_pose, road_lanes)) {
    return false;
  }

  // if (
  //   pull_out_total_distance >
  //   util::getDistanceToCrosswalk(current_pose, current_lanes, overall_graphs)) {
  //   return false;
  // }
  return true;
}

bool isPullOutPathSafe(
  const behavior_path_planner::PullOutPath & path, const lanelet::ConstLanelets & road_lanes,
  const lanelet::ConstLanelets & shoulder_lanes,
  const PredictedObjects::ConstSharedPtr & dynamic_objects,
  const PullOutParameters & ros_parameters,
  const autoware_utils::LinearRing2d & local_vehicle_footprint, const bool use_buffer,
  const bool use_dynamic_object)
{
  // TODO(sugahara) check road lanes safety and output road lanes safety
  // and shoulder lanes safety respectively
  if (path.path.points.empty()) {
    return false;
  }
  if (shoulder_lanes.empty() || road_lanes.empty()) {
    return false;
  }
  if (dynamic_objects == nullptr) {
    return true;
  }

  const double min_thresh = ros_parameters.min_stop_distance;

  double buffer;
  // double lateral_buffer;
  if (use_buffer) {
    buffer = ros_parameters.hysteresis_buffer_distance;
    // lateral_buffer = 0.5;
  } else {
    buffer = 0.0;
    // lateral_buffer = 0.0;
  }

  // find obstacle in shoulder lanes
  const auto shoulder_lane_object_indices =
    util::filterObjectsByLanelets(*dynamic_objects, shoulder_lanes);

  // Collision check for objects in shoulder lane
  if (use_dynamic_object) {
    for (const auto & i : shoulder_lane_object_indices) {
      const auto & obj = dynamic_objects->objects.at(i);
      std::vector<PredictedPath> predicted_paths;

      bool is_object_in_shoulder = false;
      if (ros_parameters.use_predicted_path_outside_lanelet) {
        is_object_in_shoulder = true;
      } else {
        for (const auto & llt : shoulder_lanes) {
          if (lanelet::utils::isInLanelet(
                obj.kinematics.initial_pose_with_covariance.pose, llt, 0.1)) {
            is_object_in_shoulder = true;
          }
        }
      }
      // TODO(sugahara) static object judge
      if (is_object_in_shoulder) {
        const double distance = util::getDistanceBetweenPredictedPathAndObjectPolygon(
          obj, path, local_vehicle_footprint, 1, road_lanes);

        double thresh = min_thresh + buffer;
        if (distance < thresh) {
          return false;
        }
      } else {
        const double distance = util::getDistanceBetweenPredictedPathAndObjectPolygon(
          obj, path, local_vehicle_footprint, 1, road_lanes);
        double thresh = min_thresh + buffer;
        if (distance < thresh) {
          RCLCPP_WARN_STREAM(
            rclcpp::get_logger("behavior_path_planner").get_child("pull_out").get_child("util"),
            "object is not in shoulder but close to the path.");
          // return false;
        }
      }
    }
  }

  return true;
}

bool isObjectFront(const Pose & ego_pose, const Pose & obj_pose)
{
  tf2::Transform tf_map2ego, tf_map2obj;
  Pose obj_from_ego;
  tf2::fromMsg(ego_pose, tf_map2ego);
  tf2::fromMsg(obj_pose, tf_map2obj);
  tf2::toMsg(tf_map2ego.inverse() * tf_map2obj, obj_from_ego);

  return obj_from_ego.position.x > 0;
}

}  // namespace pull_out_utils
}  // namespace behavior_path_planner
