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

#include "behavior_path_planner_common/turn_signal_decider.hpp"

#include "behavior_path_planner_common/utils/utils.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <motion_utils/constants.hpp>
#include <motion_utils/resample/resample.hpp>
#include <motion_utils/trajectory/path_with_lane_id.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/normalization.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>

#include <limits>
#include <queue>
#include <string>
#include <utility>

namespace behavior_path_planner
{
double calc_distance(
  const PathWithLaneId & path, const Pose & current_pose, const size_t current_seg_idx,
  const Pose & input_point, const double nearest_dist_threshold, const double nearest_yaw_threshold)
{
  const size_t nearest_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    path.points, input_point, nearest_dist_threshold, nearest_yaw_threshold);
  return motion_utils::calcSignedArcLength(
    path.points, current_pose.position, current_seg_idx, input_point.position, nearest_seg_idx);
}

TurnIndicatorsCommand TurnSignalDecider::getTurnSignal(
  const std::shared_ptr<RouteHandler> & route_handler, const PathWithLaneId & path,
  const TurnSignalInfo & turn_signal_info, const Pose & current_pose, const double current_vel,
  const BehaviorPathPlannerParameters & parameters, TurnSignalDebugData & debug_data)
{
  debug_data.behavior_turn_signal_info = turn_signal_info;

  // Guard
  if (path.points.empty()) {
    return turn_signal_info.turn_signal;
  }

  // Get current lanelets
  const double forward_length = parameters.forward_path_length;
  const double nearest_dist_threshold = parameters.ego_nearest_dist_threshold;
  const double nearest_yaw_threshold = parameters.ego_nearest_yaw_threshold;
  const double backward_length = 50.0;
  const lanelet::ConstLanelets current_lanes =
    utils::calcLaneAroundPose(route_handler, current_pose, forward_length, backward_length);

  if (current_lanes.empty()) {
    return turn_signal_info.turn_signal;
  }

  const PathWithLaneId extended_path = utils::getCenterLinePath(
    *route_handler, current_lanes, current_pose, backward_length, forward_length, parameters);

  if (extended_path.points.empty()) {
    return turn_signal_info.turn_signal;
  }

  // Closest ego segment
  const size_t ego_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    extended_path.points, current_pose, nearest_dist_threshold, nearest_yaw_threshold);

  // Get closest intersection turn signal if exists
  const auto intersection_turn_signal_info = getIntersectionTurnSignalInfo(
    extended_path, current_pose, current_vel, ego_seg_idx, *route_handler, nearest_dist_threshold,
    nearest_yaw_threshold);

  if (intersection_turn_signal_info) {
    debug_data.intersection_turn_signal_info = *intersection_turn_signal_info;
  }

  if (!intersection_turn_signal_info) {
    initialize_intersection_info();
    const auto & desired_end_point = turn_signal_info.desired_end_point;
    const double dist_to_end_point = calc_distance(
      extended_path, current_pose, ego_seg_idx, desired_end_point, nearest_dist_threshold,
      nearest_yaw_threshold);
    if (dist_to_end_point < 0.0) {
      TurnIndicatorsCommand updated_turn_signal;
      updated_turn_signal.stamp = turn_signal_info.turn_signal.stamp;
      updated_turn_signal.command = TurnIndicatorsCommand::NO_COMMAND;
      return updated_turn_signal;
    }
    return turn_signal_info.turn_signal;
  } else if (
    turn_signal_info.turn_signal.command == TurnIndicatorsCommand::NO_COMMAND ||
    turn_signal_info.turn_signal.command == TurnIndicatorsCommand::DISABLE) {
    set_intersection_info(
      extended_path, current_pose, ego_seg_idx, *intersection_turn_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    return intersection_turn_signal_info->turn_signal;
  }

  return resolve_turn_signal(
    extended_path, current_pose, ego_seg_idx, *intersection_turn_signal_info, turn_signal_info,
    nearest_dist_threshold, nearest_yaw_threshold);
}

std::pair<bool, bool> TurnSignalDecider::getIntersectionTurnSignalFlag()
{
  return std::make_pair(intersection_turn_signal_, approaching_intersection_turn_signal_);
}

std::pair<Pose, double> TurnSignalDecider::getIntersectionPoseAndDistance()
{
  return std::make_pair(intersection_pose_point_, intersection_distance_);
}

std::optional<TurnSignalInfo> TurnSignalDecider::getIntersectionTurnSignalInfo(
  const PathWithLaneId & path, const Pose & current_pose, const double current_vel,
  const size_t current_seg_idx, const RouteHandler & route_handler,
  const double nearest_dist_threshold, const double nearest_yaw_threshold)
{
  // base search distance
  const double base_search_distance =
    intersection_search_time_ * current_vel + intersection_search_distance_;

  // unique lane ids
  std::vector<lanelet::Id> unique_lane_ids;
  for (size_t i = 0; i < path.points.size(); ++i) {
    for (const auto & lane_id : path.points.at(i).lane_ids) {
      if (
        std::find(unique_lane_ids.begin(), unique_lane_ids.end(), lane_id) ==
        unique_lane_ids.end()) {
        unique_lane_ids.push_back(lane_id);
      }
    }
  }

  // combine consecutive lanes of the same turn direction
  // stores lanes that have already been combine
  std::set<int> processed_lanes;
  // since combined_lane does not inherit id and attribute,
  // and ConstantLanelet does not rewrite the value,
  // we keep front_lane together as a representative.
  std::vector<std::pair<lanelet::ConstLanelet, lanelet::ConstLanelet>> combined_and_front_vec{};
  for (const auto & lane_id : unique_lane_ids) {
    // Skip if already processed
    if (processed_lanes.find(lane_id) != processed_lanes.end()) continue;
    auto current_lane = route_handler.getLaneletsFromId(lane_id);
    lanelet::ConstLanelets combined_lane_elems{};
    while (rclcpp::ok()) {
      processed_lanes.insert(current_lane.id());
      combined_lane_elems.push_back(current_lane);

      // Get the lane and its attribute
      const std::string lane_attribute =
        current_lane.attributeOr("turn_direction", std::string("none"));

      // check next lane has the same attribute
      lanelet::ConstLanelet next_lane{};
      if (!route_handler.getNextLaneletWithinRoute(current_lane, &next_lane)) {
        break;
      }
      if (next_lane.attributeOr("turn_direction", std::string("none")) != lane_attribute) {
        break;
      }

      current_lane = next_lane;
    }

    if (!combined_lane_elems.empty()) {
      // store combined lane and its front lane
      const auto & combined_and_first = std::pair<lanelet::ConstLanelet, lanelet::ConstLanelet>(
        lanelet::utils::combineLaneletsShape(combined_lane_elems), combined_lane_elems.front());
      combined_and_front_vec.push_back(combined_and_first);
    }
  }

  std::queue<TurnSignalInfo> signal_queue;
  for (const auto & combined_and_front : combined_and_front_vec) {
    // use combined_lane's centerline
    const auto & combined_lane = combined_and_front.first;
    if (combined_lane.centerline3d().size() < 2) {
      continue;
    }

    // use front lane's id, attribute, and search distance as a representative
    const auto & front_lane = combined_and_front.second;
    const auto lane_id = front_lane.id();
    const double search_distance =
      front_lane.attributeOr("turn_signal_distance", base_search_distance);
    const std::string lane_attribute =
      front_lane.attributeOr("turn_direction", std::string("none"));

    // lane front and back pose
    Pose lane_front_pose;
    Pose lane_back_pose;
    lane_front_pose.position =
      lanelet::utils::conversion::toGeomMsgPt(combined_lane.centerline3d().front());
    lane_back_pose.position =
      lanelet::utils::conversion::toGeomMsgPt(combined_lane.centerline3d().back());

    {
      const auto & current_point = lane_front_pose.position;
      const auto & next_point =
        lanelet::utils::conversion::toGeomMsgPt(combined_lane.centerline3d()[1]);
      lane_front_pose.orientation = calc_orientation(current_point, next_point);
    }

    {
      const auto & current_point = lanelet::utils::conversion::toGeomMsgPt(
        combined_lane.centerline3d()[combined_lane.centerline3d().size() - 2]);
      const auto & next_point = lane_back_pose.position;
      lane_back_pose.orientation = calc_orientation(current_point, next_point);
    }

    const size_t front_nearest_seg_idx =
      motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
        path.points, lane_front_pose, nearest_dist_threshold, nearest_yaw_threshold);
    const size_t back_nearest_seg_idx =
      motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
        path.points, lane_back_pose, nearest_dist_threshold, nearest_yaw_threshold);

    // Distance from ego vehicle front pose to front point of the lane
    const double dist_to_front_point = motion_utils::calcSignedArcLength(
                                         path.points, current_pose.position, current_seg_idx,
                                         lane_front_pose.position, front_nearest_seg_idx) -
                                       base_link2front_;

    // Distance from ego vehicle base link to the terminal point of the lane
    const double dist_to_back_point = motion_utils::calcSignedArcLength(
      path.points, current_pose.position, current_seg_idx, lane_back_pose.position,
      back_nearest_seg_idx);

    if (dist_to_back_point < 0.0) {
      // Vehicle is already passed this lane
      if (desired_start_point_map_.find(lane_id) != desired_start_point_map_.end()) {
        desired_start_point_map_.erase(lane_id);
      }
      continue;
    } else if (search_distance < dist_to_front_point) {
      continue;
    }

    constexpr double stop_velocity_threshold = 0.1;
    if (dist_to_front_point < search_distance) {
      if (
        lane_attribute == "right" || lane_attribute == "left" ||
        (lane_attribute == "straight" && current_vel < stop_velocity_threshold)) {
        // update map if necessary
        if (desired_start_point_map_.find(lane_id) == desired_start_point_map_.end()) {
          desired_start_point_map_.emplace(lane_id, current_pose);
        }

        TurnSignalInfo turn_signal_info{};
        turn_signal_info.desired_start_point = desired_start_point_map_.at(lane_id);
        turn_signal_info.required_start_point = lane_front_pose;
        turn_signal_info.required_end_point = get_required_end_point(combined_lane.centerline3d());
        turn_signal_info.desired_end_point = lane_back_pose;
        turn_signal_info.turn_signal.command = g_signal_map.at(lane_attribute);
        signal_queue.push(turn_signal_info);
      }
    }
  }

  // Resolve the conflict between several turn signal requirements
  while (!signal_queue.empty()) {
    if (signal_queue.size() == 1) {
      return signal_queue.front();
    }

    const auto & turn_signal_info = signal_queue.front();
    const auto & required_end_point = turn_signal_info.required_end_point;
    const size_t nearest_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, required_end_point, nearest_dist_threshold, nearest_yaw_threshold);
    const double dist_to_end_point = motion_utils::calcSignedArcLength(
      path.points, current_pose.position, current_seg_idx, required_end_point.position,
      nearest_seg_idx);

    if (dist_to_end_point >= 0.0) {
      // we haven't finished the current mandatory turn signal
      return turn_signal_info;
    }

    signal_queue.pop();
  }

  return {};
}

TurnIndicatorsCommand TurnSignalDecider::resolve_turn_signal(
  const PathWithLaneId & path, const Pose & current_pose, const size_t current_seg_idx,
  const TurnSignalInfo & intersection_signal_info, const TurnSignalInfo & behavior_signal_info,
  const double nearest_dist_threshold, const double nearest_yaw_threshold)
{
  const auto get_distance = [&](const Pose & input_point) {
    return calc_distance(
      path, current_pose, current_seg_idx, input_point, nearest_dist_threshold,
      nearest_yaw_threshold);
  };

  const auto & inter_desired_start_point = intersection_signal_info.desired_start_point;
  const auto & inter_desired_end_point = intersection_signal_info.desired_end_point;
  const auto & inter_required_start_point = intersection_signal_info.required_start_point;
  const auto & inter_required_end_point = intersection_signal_info.required_end_point;
  const auto & behavior_desired_start_point = behavior_signal_info.desired_start_point;
  const auto & behavior_desired_end_point = behavior_signal_info.desired_end_point;
  const auto & behavior_required_start_point = behavior_signal_info.required_start_point;
  const auto & behavior_required_end_point = behavior_signal_info.required_end_point;

  const double dist_to_intersection_desired_start =
    get_distance(inter_desired_start_point) - base_link2front_;
  const double dist_to_intersection_desired_end = get_distance(inter_desired_end_point);
  const double dist_to_intersection_required_start =
    get_distance(inter_required_start_point) - base_link2front_;
  const double dist_to_intersection_required_end = get_distance(inter_required_end_point);
  const double dist_to_behavior_desired_start =
    get_distance(behavior_desired_start_point) - base_link2front_;
  const double dist_to_behavior_desired_end = get_distance(behavior_desired_end_point);
  const double dist_to_behavior_required_start =
    get_distance(behavior_required_start_point) - base_link2front_;
  const double dist_to_behavior_required_end = get_distance(behavior_required_end_point);

  // If we still do not reach the desired front point we ignore it
  if (dist_to_intersection_desired_start > 0.0 && dist_to_behavior_desired_start > 0.0) {
    TurnIndicatorsCommand empty_signal_command;
    empty_signal_command.command = TurnIndicatorsCommand::DISABLE;
    initialize_intersection_info();
    return empty_signal_command;
  } else if (dist_to_intersection_desired_start > 0.0) {
    initialize_intersection_info();
    return behavior_signal_info.turn_signal;
  } else if (dist_to_behavior_desired_start > 0.0) {
    set_intersection_info(
      path, current_pose, current_seg_idx, intersection_signal_info, nearest_dist_threshold,
      nearest_yaw_threshold);
    return intersection_signal_info.turn_signal;
  }

  // If we already passed the desired end point, return the other signal
  if (dist_to_intersection_desired_end < 0.0 && dist_to_behavior_desired_end < 0.0) {
    TurnIndicatorsCommand empty_signal_command;
    empty_signal_command.command = TurnIndicatorsCommand::DISABLE;
    initialize_intersection_info();
    return empty_signal_command;
  } else if (dist_to_intersection_desired_end < 0.0) {
    initialize_intersection_info();
    return behavior_signal_info.turn_signal;
  } else if (dist_to_behavior_desired_end < 0.0) {
    set_intersection_info(
      path, current_pose, current_seg_idx, intersection_signal_info, nearest_dist_threshold,
      nearest_yaw_threshold);
    return intersection_signal_info.turn_signal;
  }

  if (dist_to_intersection_desired_start <= dist_to_behavior_desired_start) {
    // intersection signal is prior than behavior signal
    const auto enable_prior = use_prior_turn_signal(
      dist_to_intersection_required_start, dist_to_intersection_required_end,
      dist_to_behavior_required_start, dist_to_behavior_required_end);

    if (enable_prior) {
      set_intersection_info(
        path, current_pose, current_seg_idx, intersection_signal_info, nearest_dist_threshold,
        nearest_yaw_threshold);
      return intersection_signal_info.turn_signal;
    }
    initialize_intersection_info();
    return behavior_signal_info.turn_signal;
  }

  // behavior signal is prior than intersection signal
  const auto enable_prior = use_prior_turn_signal(
    dist_to_behavior_required_start, dist_to_behavior_required_end,
    dist_to_intersection_required_start, dist_to_intersection_required_end);
  if (enable_prior) {
    initialize_intersection_info();
    return behavior_signal_info.turn_signal;
  }
  set_intersection_info(
    path, current_pose, current_seg_idx, intersection_signal_info, nearest_dist_threshold,
    nearest_yaw_threshold);
  return intersection_signal_info.turn_signal;
}

TurnSignalInfo TurnSignalDecider::use_prior_turn_signal(
  const PathWithLaneId & path, const Pose & current_pose, const size_t current_seg_idx,
  const TurnSignalInfo & original_signal, const TurnSignalInfo & new_signal,
  const double nearest_dist_threshold, const double nearest_yaw_threshold)
{
  const auto get_distance = [&](const Pose & input_point) {
    const size_t nearest_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, input_point, nearest_dist_threshold, nearest_yaw_threshold);
    return motion_utils::calcSignedArcLength(
      path.points, current_pose.position, current_seg_idx, input_point.position, nearest_seg_idx);
  };

  const auto & original_desired_start_point = original_signal.desired_start_point;
  const auto & original_desired_end_point = original_signal.desired_end_point;
  const auto & original_required_start_point = original_signal.required_start_point;
  const auto & original_required_end_point = original_signal.required_end_point;
  const auto & new_desired_start_point = new_signal.desired_start_point;
  const auto & new_desired_end_point = new_signal.desired_end_point;
  const auto & new_required_start_point = new_signal.required_start_point;
  const auto & new_required_end_point = new_signal.required_end_point;

  const double dist_to_original_desired_start =
    get_distance(original_desired_start_point) - base_link2front_;
  const double dist_to_original_desired_end = get_distance(original_desired_end_point);
  const double dist_to_original_required_start =
    get_distance(original_required_start_point) - base_link2front_;
  const double dist_to_original_required_end = get_distance(original_required_end_point);
  const double dist_to_new_desired_start = get_distance(new_desired_start_point) - base_link2front_;
  const double dist_to_new_desired_end = get_distance(new_desired_end_point);
  const double dist_to_new_required_start =
    get_distance(new_required_start_point) - base_link2front_;
  const double dist_to_new_required_end = get_distance(new_required_end_point);

  // If we still do not reach the desired front point we ignore it
  if (dist_to_original_desired_start > 0.0 && dist_to_new_desired_start > 0.0) {
    TurnSignalInfo empty_signal_info;
    return empty_signal_info;
  } else if (dist_to_original_desired_start > 0.0) {
    return new_signal;
  } else if (dist_to_new_desired_start > 0.0) {
    return original_signal;
  }

  // If we already passed the desired end point, return the other signal
  if (dist_to_original_desired_end < 0.0 && dist_to_new_desired_end < 0.0) {
    TurnSignalInfo empty_signal_info;
    return empty_signal_info;
  } else if (dist_to_original_desired_end < 0.0) {
    return new_signal;
  } else if (dist_to_new_desired_end < 0.0) {
    return original_signal;
  }

  if (dist_to_original_desired_start <= dist_to_new_desired_start) {
    const auto enable_prior = use_prior_turn_signal(
      dist_to_original_required_start, dist_to_original_required_end, dist_to_new_required_start,
      dist_to_new_required_end);

    if (enable_prior) {
      return original_signal;
    }
    return new_signal;
  }

  const auto enable_prior = use_prior_turn_signal(
    dist_to_new_required_start, dist_to_new_required_end, dist_to_original_required_start,
    dist_to_original_required_end);
  if (enable_prior) {
    return new_signal;
  }
  return original_signal;
}

bool TurnSignalDecider::use_prior_turn_signal(
  const double dist_to_prior_required_start, const double dist_to_prior_required_end,
  const double dist_to_subsequent_required_start, const double dist_to_subsequent_required_end)
{
  const bool before_prior_required = dist_to_prior_required_start > 0.0;
  const bool before_subsequent_required = dist_to_subsequent_required_start > 0.0;
  const bool inside_prior_required =
    dist_to_prior_required_start < 0.0 && 0.0 <= dist_to_prior_required_end;

  if (dist_to_prior_required_start < dist_to_subsequent_required_start) {
    // subsequent signal required section is completely overlapped the prior signal required section
    if (dist_to_subsequent_required_end < dist_to_prior_required_end) {
      return true;
    }

    // Vehicle is inside or in front of the prior required section
    if (before_prior_required || inside_prior_required) {
      return true;
    }

    // passed prior required section but in front of the subsequent required section
    if (before_subsequent_required) {
      return true;
    }

    // within or passed subsequent required section and completely passed prior required section
    return false;
  }

  // Subsequent required section starts faster than prior required starts section

  // If the prior section is inside of the subsequent required section
  if (dist_to_prior_required_end < dist_to_subsequent_required_end) {
    if (before_prior_required || inside_prior_required) {
      return true;
    }
    return false;
  }

  // inside or passed the intersection required
  if (before_prior_required) {
    return false;
  }

  return true;
}

geometry_msgs::msg::Pose TurnSignalDecider::get_required_end_point(
  const lanelet::ConstLineString3d & centerline)
{
  std::vector<geometry_msgs::msg::Pose> converted_centerline(centerline.size());
  for (size_t i = 0; i < centerline.size(); ++i) {
    converted_centerline.at(i).position = lanelet::utils::conversion::toGeomMsgPt(centerline[i]);
  }
  motion_utils::insertOrientation(converted_centerline, true);

  const double length = motion_utils::calcArcLength(converted_centerline);

  // Create resampling intervals
  const double resampling_interval = 1.0;
  std::vector<double> resampling_arclength;
  for (double s = 0.0; s < length; s += resampling_interval) {
    resampling_arclength.push_back(s);
  }

  // Insert terminal point
  if (length - resampling_arclength.back() < motion_utils::overlap_threshold) {
    resampling_arclength.back() = length;
  } else {
    resampling_arclength.push_back(length);
  }

  const auto resampled_centerline =
    motion_utils::resamplePoseVector(converted_centerline, resampling_arclength);

  const double terminal_yaw = tf2::getYaw(resampled_centerline.back().orientation);
  for (size_t i = 0; i < resampled_centerline.size(); ++i) {
    const double yaw = tf2::getYaw(resampled_centerline.at(i).orientation);
    const double yaw_diff = tier4_autoware_utils::normalizeRadian(yaw - terminal_yaw);
    if (std::fabs(yaw_diff) < tier4_autoware_utils::deg2rad(intersection_angle_threshold_deg_)) {
      return resampled_centerline.at(i);
    }
  }

  return resampled_centerline.back();
}

void TurnSignalDecider::set_intersection_info(
  const PathWithLaneId & path, const Pose & current_pose, const size_t current_seg_idx,
  const TurnSignalInfo & intersection_turn_signal_info, const double nearest_dist_threshold,
  const double nearest_yaw_threshold)
{
  const auto get_distance = [&](const Pose & input_point) {
    const size_t nearest_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, input_point, nearest_dist_threshold, nearest_yaw_threshold);
    return motion_utils::calcSignedArcLength(
      path.points, current_pose.position, current_seg_idx, input_point.position, nearest_seg_idx);
  };

  const auto & inter_desired_start_point = intersection_turn_signal_info.desired_start_point;
  const auto & inter_desired_end_point = intersection_turn_signal_info.desired_end_point;
  const auto & inter_required_start_point = intersection_turn_signal_info.required_start_point;

  const double dist_to_intersection_desired_start =
    get_distance(inter_desired_start_point) - base_link2front_;
  const double dist_to_intersection_desired_end = get_distance(inter_desired_end_point);
  const double dist_to_intersection_required_start =
    get_distance(inter_required_start_point) - base_link2front_;

  if (dist_to_intersection_desired_start < 0.0 && dist_to_intersection_desired_end > 0.0) {
    if (dist_to_intersection_required_start > 0.0) {
      intersection_turn_signal_ = false;
      approaching_intersection_turn_signal_ = true;
    } else {
      intersection_turn_signal_ = true;
      approaching_intersection_turn_signal_ = false;
    }
    intersection_distance_ = dist_to_intersection_required_start;
    intersection_pose_point_ = inter_required_start_point;
  } else {
    initialize_intersection_info();
  }
}

void TurnSignalDecider::initialize_intersection_info()
{
  intersection_turn_signal_ = false;
  approaching_intersection_turn_signal_ = false;
  intersection_pose_point_ = Pose();
  intersection_distance_ = std::numeric_limits<double>::max();
}

geometry_msgs::msg::Quaternion TurnSignalDecider::calc_orientation(
  const Point & src_point, const Point & dst_point)
{
  const double pitch = tier4_autoware_utils::calcElevationAngle(src_point, dst_point);
  const double yaw = tier4_autoware_utils::calcAzimuthAngle(src_point, dst_point);
  return tier4_autoware_utils::createQuaternionFromRPY(0.0, pitch, yaw);
}
}  // namespace behavior_path_planner
