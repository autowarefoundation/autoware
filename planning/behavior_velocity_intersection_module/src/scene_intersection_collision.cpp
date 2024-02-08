// Copyright 2024 Tier IV, Inc.
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

#include "scene_intersection.hpp"
#include "util.hpp"

#include <behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>  // for toGeomPoly
#include <behavior_velocity_planner_common/utilization/trajectory_utils.hpp>       // for smoothPath
#include <lanelet2_extension/utility/utilities.hpp>
#include <magic_enum.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>  // for toPolygon2d
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/algorithms/within.hpp>

#include <fmt/format.h>
#include <lanelet2_core/geometry/Polygon.h>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

bool IntersectionModule::isTargetCollisionVehicleType(
  const autoware_auto_perception_msgs::msg::PredictedObject & object) const
{
  const auto label = object.classification.at(0).label;
  const auto & p = planner_param_.collision_detection.target_type;

  if (label == autoware_auto_perception_msgs::msg::ObjectClassification::CAR && p.car) {
    return true;
  }
  if (label == autoware_auto_perception_msgs::msg::ObjectClassification::BUS && p.bus) {
    return true;
  }
  if (label == autoware_auto_perception_msgs::msg::ObjectClassification::TRUCK && p.truck) {
    return true;
  }
  if (label == autoware_auto_perception_msgs::msg::ObjectClassification::TRAILER && p.trailer) {
    return true;
  }
  if (
    label == autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE && p.motorcycle) {
    return true;
  }
  if (label == autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE && p.bicycle) {
    return true;
  }
  if (label == autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN && p.unknown) {
    return true;
  }
  return false;
}

void IntersectionModule::updateObjectInfoManagerArea()
{
  const auto & intersection_lanelets = intersection_lanelets_.value();
  const auto & attention_lanelets = intersection_lanelets.attention();
  const auto & attention_lanelet_stoplines = intersection_lanelets.attention_stoplines();
  const auto & adjacent_lanelets = intersection_lanelets.adjacent();
  const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto & assigned_lanelet = lanelet_map_ptr->laneletLayer.get(lane_id_);
  const auto intersection_area = util::getIntersectionArea(assigned_lanelet, lanelet_map_ptr);

  // ==========================================================================================
  // entries that are not observed in this iteration need to be cleared
  //
  // NOTE: old_map is not referenced because internal data of object_info_manager is cleared
  // ==========================================================================================
  const auto old_map = object_info_manager_.getObjectsMap();
  object_info_manager_.clearObjects();

  for (const auto & predicted_object : planner_data_->predicted_objects->objects) {
    if (!isTargetCollisionVehicleType(predicted_object)) {
      continue;
    }

    // ==========================================================================================
    // NOTE: is_parked_vehicle is used because sometimes slow vehicle direction is
    // incorrect/reversed/flipped due to tracking. if is_parked_vehicle is true, object direction
    // is not checked
    // ==========================================================================================
    const auto object_direction =
      util::getObjectPoseWithVelocityDirection(predicted_object.kinematics);
    const auto is_parked_vehicle =
      std::fabs(predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x) <
      planner_param_.occlusion.ignore_parked_vehicle_speed_threshold;

    const auto belong_adjacent_lanelet_id =
      checkAngleForTargetLanelets(object_direction, adjacent_lanelets, false);
    if (belong_adjacent_lanelet_id) {
      continue;
    }
    const auto belong_attention_lanelet_id =
      checkAngleForTargetLanelets(object_direction, attention_lanelets, is_parked_vehicle);
    const auto & obj_pos = predicted_object.kinematics.initial_pose_with_covariance.pose.position;
    const bool in_intersection_area = [&]() {
      if (!intersection_area) {
        return false;
      }
      return bg::within(
        tier4_autoware_utils::Point2d{obj_pos.x, obj_pos.y}, intersection_area.value());
    }();
    std::optional<lanelet::ConstLanelet> attention_lanelet{std::nullopt};
    std::optional<lanelet::ConstLineString3d> stopline{std::nullopt};
    if (!belong_attention_lanelet_id && !in_intersection_area) {
      continue;
    } else if (belong_attention_lanelet_id) {
      const auto idx = belong_attention_lanelet_id.value();
      attention_lanelet = attention_lanelets.at(idx);
      stopline = attention_lanelet_stoplines.at(idx);
    }

    const auto object_it = old_map.find(predicted_object.object_id);
    if (object_it != old_map.end()) {
      auto object_info = object_it->second;
      object_info_manager_.registerExistingObject(
        predicted_object.object_id, belong_attention_lanelet_id.has_value(), in_intersection_area,
        is_parked_vehicle, object_info);
      object_info->initialize(predicted_object, attention_lanelet, stopline);
    } else {
      auto object_info = object_info_manager_.registerObject(
        predicted_object.object_id, belong_attention_lanelet_id.has_value(), in_intersection_area,
        is_parked_vehicle);
      object_info->initialize(predicted_object, attention_lanelet, stopline);
    }
  }
}

void IntersectionModule::updateObjectInfoManagerCollision(
  const intersection::PathLanelets & path_lanelets,
  const IntersectionModule::TimeDistanceArray & time_distance_array,
  const IntersectionModule::TrafficPrioritizedLevel & traffic_prioritized_level,
  const bool passed_1st_judge_line_first_time, const bool passed_2nd_judge_line_first_time)
{
  const auto & intersection_lanelets = intersection_lanelets_.value();

  if (passed_1st_judge_line_first_time) {
    object_info_manager_.setPassed1stPassJudgeLineFirstTime(clock_->now());
  }
  if (passed_2nd_judge_line_first_time) {
    object_info_manager_.setPassed2ndPassJudgeLineFirstTime(clock_->now());
  }

  const double passing_time = time_distance_array.back().first;
  const auto & concat_lanelets = path_lanelets.all;
  const auto closest_arc_coords =
    lanelet::utils::getArcCoordinates(concat_lanelets, planner_data_->current_odometry->pose);
  const auto & ego_lane = path_lanelets.ego_or_entry2exit;
  debug_data_.ego_lane = ego_lane.polygon3d();
  const auto ego_poly = ego_lane.polygon2d().basicPolygon();

  // ==========================================================================================
  // dynamically change TTC margin according to traffic light color to gradually relax from green to
  // red
  // ==========================================================================================
  const auto [collision_start_margin_time, collision_end_margin_time] = [&]() {
    if (traffic_prioritized_level == TrafficPrioritizedLevel::FULLY_PRIORITIZED) {
      return std::make_pair(
        planner_param_.collision_detection.fully_prioritized.collision_start_margin_time,
        planner_param_.collision_detection.fully_prioritized.collision_end_margin_time);
    }
    if (traffic_prioritized_level == TrafficPrioritizedLevel::PARTIALLY_PRIORITIZED) {
      return std::make_pair(
        planner_param_.collision_detection.partially_prioritized.collision_start_margin_time,
        planner_param_.collision_detection.partially_prioritized.collision_end_margin_time);
    }
    return std::make_pair(
      planner_param_.collision_detection.not_prioritized.collision_start_margin_time,
      planner_param_.collision_detection.not_prioritized.collision_end_margin_time);
  }();

  for (auto & object_info : object_info_manager_.attentionObjects()) {
    const auto & predicted_object = object_info->predicted_object();
    bool safe_under_traffic_control = false;
    const auto label = predicted_object.classification.at(0).label;
    const auto expected_deceleration =
      (label == autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE ||
       label == autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE)
        ? planner_param_.collision_detection.ignore_on_amber_traffic_light
            .object_expected_deceleration.bike
        : planner_param_.collision_detection.ignore_on_amber_traffic_light
            .object_expected_deceleration.car;
    if (
      traffic_prioritized_level == TrafficPrioritizedLevel::PARTIALLY_PRIORITIZED &&
      object_info->can_stop_before_stopline(expected_deceleration)) {
      safe_under_traffic_control = true;
    }
    if (
      traffic_prioritized_level == TrafficPrioritizedLevel::FULLY_PRIORITIZED &&
      object_info->can_stop_before_ego_lane(
        expected_deceleration,
        planner_param_.collision_detection.ignore_on_red_traffic_light.object_margin_to_path,
        ego_lane)) {
      safe_under_traffic_control = true;
    }

    // ==========================================================================================
    // check the PredictedPath in the ascending order of its confidence to save the safe/unsafe
    // CollisionKnowledge for most probable path
    // ==========================================================================================
    std::list<const autoware_auto_perception_msgs::msg::PredictedPath *> sorted_predicted_paths;
    for (unsigned i = 0; i < predicted_object.kinematics.predicted_paths.size(); ++i) {
      sorted_predicted_paths.push_back(&predicted_object.kinematics.predicted_paths.at(i));
    }
    sorted_predicted_paths.sort(
      [](const auto path1, const auto path2) { return path1->confidence > path2->confidence; });

    // ==========================================================================================
    // if all of the predicted path is lower confidence/geometrically does not intersect with ego
    // path, both will be null, which is interpreted as SAFE. if any of the path is "normal", either
    // of them has value, not both
    // ==========================================================================================
    std::optional<intersection::CollisionInterval> unsafe_interval{std::nullopt};
    std::optional<intersection::CollisionInterval> safe_interval{std::nullopt};

    for (const auto & predicted_path_ptr : sorted_predicted_paths) {
      auto predicted_path = *predicted_path_ptr;
      if (
        predicted_path.confidence <
        planner_param_.collision_detection.min_predicted_path_confidence) {
        continue;
      }
      cutPredictPathWithinDuration(
        planner_data_->predicted_objects->header.stamp, passing_time, &predicted_path);
      const auto object_passage_interval_opt = intersection::findPassageInterval(
        predicted_path, predicted_object.shape, ego_poly,
        intersection_lanelets.first_attention_lane(),
        intersection_lanelets.second_attention_lane());
      if (!object_passage_interval_opt) {
        // there is no chance of geometric collision for the entire prediction horizon
        continue;
      }
      const auto & object_passage_interval = object_passage_interval_opt.value();
      const auto [object_enter_time, object_exit_time] = object_passage_interval.interval_time;
      const auto ego_start_itr = std::lower_bound(
        time_distance_array.begin(), time_distance_array.end(),
        object_enter_time - collision_start_margin_time,
        [](const auto & a, const double b) { return a.first < b; });
      if (ego_start_itr == time_distance_array.end()) {
        // ==========================================================================================
        // this is the case where at time "object_enter_time - collision_start_margin_time", ego is
        // arriving at the exit of the intersection, which means even if we assume that the object
        // accelerates and the first collision happens faster by the TTC margin, ego will be already
        // arriving at the exist of the intersection.
        // ==========================================================================================
        continue;
      }
      auto ego_end_itr = std::lower_bound(
        time_distance_array.begin(), time_distance_array.end(),
        object_exit_time + collision_end_margin_time,
        [](const auto & a, const double b) { return a.first < b; });
      if (ego_end_itr == time_distance_array.end()) {
        ego_end_itr = time_distance_array.end() - 1;
      }
      const double ego_start_arc_length = std::max(
        0.0, closest_arc_coords.length + ego_start_itr->second -
               planner_data_->vehicle_info_.rear_overhang_m);
      const double ego_end_arc_length = std::min(
        closest_arc_coords.length + ego_end_itr->second +
          planner_data_->vehicle_info_.max_longitudinal_offset_m,
        lanelet::utils::getLaneletLength2d(concat_lanelets));
      const auto trimmed_ego_polygon = lanelet::utils::getPolygonFromArcLength(
        concat_lanelets, ego_start_arc_length, ego_end_arc_length);
      if (trimmed_ego_polygon.empty()) {
        continue;
      }
      Polygon2d polygon{};
      for (const auto & p : trimmed_ego_polygon) {
        polygon.outer().emplace_back(p.x(), p.y());
      }
      bg::correct(polygon);
      debug_data_.candidate_collision_ego_lane_polygon = toGeomPoly(polygon);

      const auto & object_path = object_passage_interval.path;
      const auto [begin, end] = object_passage_interval.interval_position;
      bool collision_detected = false;
      for (auto i = begin; i <= end; ++i) {
        if (bg::intersects(
              polygon,
              tier4_autoware_utils::toPolygon2d(object_path.at(i), predicted_object.shape))) {
          collision_detected = true;
          break;
        }
      }
      if (collision_detected) {
        // if judged as UNSAFE, return
        safe_interval = std::nullopt;
        unsafe_interval = object_passage_interval;
        break;
      }
      if (!safe_interval) {
        // ==========================================================================================
        // save the safe_decision_knowledge for the most probable path. this value is nullified if
        // judged UNSAFE during the iteration
        // ==========================================================================================
        safe_interval = object_passage_interval;
      }
    }
    object_info->update_safety(unsafe_interval, safe_interval, safe_under_traffic_control);
    if (passed_1st_judge_line_first_time) {
      object_info->setDecisionAt1stPassJudgeLinePassage(intersection::CollisionKnowledge{
        clock_->now(),  // stamp
        unsafe_interval
          ? intersection::CollisionKnowledge::SafeType::UNSAFE
          : (safe_under_traffic_control
               ? intersection::CollisionKnowledge::SafeType::SAFE_UNDER_TRAFFIC_CONTROL
               : intersection::CollisionKnowledge::SafeType::SAFE),  // safe
        unsafe_interval ? unsafe_interval : safe_interval,           // interval
        predicted_object.kinematics.initial_twist_with_covariance.twist.linear
          .x  // observed_velocity
      });
    }
    if (passed_2nd_judge_line_first_time) {
      object_info->setDecisionAt2ndPassJudgeLinePassage(intersection::CollisionKnowledge{
        clock_->now(),  // stamp
        unsafe_interval
          ? intersection::CollisionKnowledge::SafeType::UNSAFE
          : (safe_under_traffic_control
               ? intersection::CollisionKnowledge::SafeType::SAFE_UNDER_TRAFFIC_CONTROL
               : intersection::CollisionKnowledge::SafeType::SAFE),  // safe
        unsafe_interval ? unsafe_interval : safe_interval,           // interval
        predicted_object.kinematics.initial_twist_with_covariance.twist.linear
          .x  // observed_velocity
      });
    }
  }
}

void IntersectionModule::cutPredictPathWithinDuration(
  const builtin_interfaces::msg::Time & object_stamp, const double time_thr,
  autoware_auto_perception_msgs::msg::PredictedPath * path) const
{
  const rclcpp::Time current_time = clock_->now();
  const auto original_path = path->path;
  path->path.clear();

  for (size_t k = 0; k < original_path.size(); ++k) {  // each path points
    const auto & predicted_pose = original_path.at(k);
    const auto predicted_time =
      rclcpp::Time(object_stamp) + rclcpp::Duration(path->time_step) * static_cast<double>(k);
    if ((predicted_time - current_time).seconds() < time_thr) {
      path->path.push_back(predicted_pose);
    }
  }
}

std::optional<intersection::NonOccludedCollisionStop>
IntersectionModule::isGreenPseudoCollisionStatus(
  const size_t closest_idx, const size_t collision_stopline_idx,
  const intersection::IntersectionStopLines & intersection_stoplines) const
{
  // ==========================================================================================
  // if there are any vehicles on the attention area when ego entered the intersection on green
  // light, do pseudo collision detection because collision is likely to happen.
  // ==========================================================================================
  if (initial_green_light_observed_time_) {
    const auto now = clock_->now();
    const bool still_wait =
      (rclcpp::Duration((now - initial_green_light_observed_time_.value())).seconds() <
       planner_param_.collision_detection.yield_on_green_traffic_light.duration);
    if (!still_wait) {
      return std::nullopt;
    }
    const auto & attention_objects = object_info_manager_.attentionObjects();
    const bool exist_close_vehicles = std::any_of(
      attention_objects.begin(), attention_objects.end(), [&](const auto & object_info) {
        return object_info->before_stopline_by(
          planner_param_.collision_detection.yield_on_green_traffic_light.object_dist_to_stopline);
      });
    if (exist_close_vehicles) {
      const auto occlusion_stopline_idx = intersection_stoplines.occlusion_peeking_stopline.value();
      return intersection::NonOccludedCollisionStop{
        closest_idx, collision_stopline_idx, occlusion_stopline_idx, std::string("")};
    }
  }
  return std::nullopt;
}

std::string IntersectionModule::generateDetectionBlameDiagnosis(
  const std::vector<std::pair<
    IntersectionModule::CollisionStatus::BlameType, std::shared_ptr<intersection::ObjectInfo>>> &
    too_late_detect_objects,
  const std::vector<std::pair<
    IntersectionModule::CollisionStatus::BlameType, std::shared_ptr<intersection::ObjectInfo>>> &
    misjudge_objects) const
{
  std::string diag;
  if (!safely_passed_1st_judge_line_time_) {
    return diag;
  }
  const auto [passed_1st_judge_line_time, passed_1st_judge_line_pose] =
    safely_passed_1st_judge_line_time_.value();
  const auto passed_1st_judge_line_time_double =
    static_cast<double>(passed_1st_judge_line_time.nanoseconds()) / 1e+9;

  const auto now = clock_->now();
  const auto now_double = static_cast<double>(now.nanoseconds()) / 1e+9;

  // CAVEAT: format library causes runtime fault if the # of placeholders is more than the # of
  // vargs
  for (const auto & [blame_type, object_info] : too_late_detect_objects) {
    if (
      blame_type == CollisionStatus::BLAME_AT_FIRST_PASS_JUDGE && object_info->unsafe_interval()) {
      const auto & unsafe_interval = object_info->unsafe_interval().value();
      const double time_diff = now_double - passed_1st_judge_line_time_double;
      diag += fmt::format(
        "object {0} was not detected when ego passed the 1st pass judge line at {1}, but now at "
        "{2}, collision is detected after {3}~{4} seconds on the first attention lanelet of type "
        "{5}.\n",
        object_info->uuid_str,                                // 0
        passed_1st_judge_line_time_double,                    // 1
        now_double,                                           // 2
        unsafe_interval.interval_time.first,                  // 3
        unsafe_interval.interval_time.second,                 // 4
        magic_enum::enum_name(unsafe_interval.lane_position)  // 5
      );
      const auto past_position_opt = object_info->estimated_past_position(time_diff);
      if (past_position_opt) {
        const auto & past_position = past_position_opt.value();
        diag += fmt::format(
          "this object is estimated to have been at x = {0}, y = {1} when ego passed the 1st pass "
          "judge line({2} seconds before from now) given the estimated current velocity {3}[m/s]. "
          "ego was at x = {4}, y = {5} when it passed the 1st pass judge line so it is the fault "
          "of detection side that failed to detect around {6}[m] range at that time.\n",
          past_position.x,                                                                 // 0
          past_position.y,                                                                 // 1
          time_diff,                                                                       // 2
          object_info->observed_velocity(),                                                // 3
          passed_1st_judge_line_pose.position.x,                                           // 4
          passed_1st_judge_line_pose.position.y,                                           // 5
          tier4_autoware_utils::calcDistance2d(passed_1st_judge_line_pose, past_position)  // 6
        );
      }
    }
    if (
      safely_passed_2nd_judge_line_time_ &&
      blame_type == CollisionStatus::BLAME_AT_SECOND_PASS_JUDGE && object_info->unsafe_interval()) {
      const auto [passed_2nd_judge_line_time, passed_2nd_judge_line_pose] =
        safely_passed_2nd_judge_line_time_.value();
      const auto passed_2nd_judge_line_time_double =
        static_cast<double>(passed_2nd_judge_line_time.nanoseconds()) / 1e+9;

      const auto & unsafe_interval = object_info->unsafe_interval().value();
      const double time_diff = now_double - passed_2nd_judge_line_time_double;
      diag += fmt::format(
        "object {0} was not detected when ego passed the 2nd pass judge line at {1}, but now at "
        "{2}, collision is detected after {3}~{4} seconds on the lanelet of type {5}.\n",
        object_info->uuid_str,                                // 0
        passed_2nd_judge_line_time_double,                    // 1
        now_double,                                           // 2
        unsafe_interval.interval_time.first,                  // 3
        unsafe_interval.interval_time.second,                 // 4
        magic_enum::enum_name(unsafe_interval.lane_position)  // 5
      );
      const auto past_position_opt = object_info->estimated_past_position(time_diff);
      if (past_position_opt) {
        const auto & past_position = past_position_opt.value();
        diag += fmt::format(
          "this object is estimated to have been at x = {0}, y = {1} when ego passed the 2nd pass "
          "judge line({2} seconds before from now) given the estimated current velocity {3}[m/s]. "
          "ego was at x = {4}, y = {5} when it passed the 2nd pass judge line so it is the fault "
          "of detection side that failed to detect around {6}[m] range at that time.\n",
          past_position.x,                                                                 // 0
          past_position.y,                                                                 // 1
          time_diff,                                                                       // 2
          object_info->observed_velocity(),                                                // 3
          passed_2nd_judge_line_pose.position.x,                                           // 4
          passed_2nd_judge_line_pose.position.y,                                           // 5
          tier4_autoware_utils::calcDistance2d(passed_2nd_judge_line_pose, past_position)  // 6
        );
      }
    }
  }
  for (const auto & [blame_type, object_info] : misjudge_objects) {
    if (
      blame_type == CollisionStatus::BLAME_AT_FIRST_PASS_JUDGE && object_info->unsafe_interval() &&
      object_info->decision_at_1st_pass_judge_line_passage()) {
      const auto & decision_at_1st_pass_judge_line =
        object_info->decision_at_1st_pass_judge_line_passage().value();
      const auto decision_at_1st_pass_judge_line_time =
        static_cast<double>(decision_at_1st_pass_judge_line.stamp.nanoseconds()) / 1e+9;
      const auto & unsafe_interval = object_info->unsafe_interval().value();
      diag += fmt::format(
        "object {0} was judged as {1} when ego passed the 1st pass judge line at time {2} "
        "previously with the estimated velocity {3}[m/s], but now at {4} collision is detected "
        "after {5}~{6} seconds on the first attention lanelet of type {7} with the estimated "
        "current velocity {8}[m/s]\n",
        object_info->uuid_str,                                             // 0
        magic_enum::enum_name(decision_at_1st_pass_judge_line.safe_type),  // 1
        decision_at_1st_pass_judge_line_time,                              // 2
        decision_at_1st_pass_judge_line.observed_velocity,                 // 3
        now_double,                                                        // 4
        unsafe_interval.interval_time.first,                               // 5
        unsafe_interval.interval_time.second,                              // 6
        magic_enum::enum_name(unsafe_interval.lane_position),              // 7
        object_info->observed_velocity()                                   // 8
      );
    }
    if (
      blame_type == CollisionStatus::BLAME_AT_SECOND_PASS_JUDGE && object_info->unsafe_interval() &&
      object_info->decision_at_2nd_pass_judge_line_passage()) {
      const auto & decision_at_2nd_pass_judge_line =
        object_info->decision_at_2nd_pass_judge_line_passage().value();
      const auto decision_at_2nd_pass_judge_line_time =
        static_cast<double>(decision_at_2nd_pass_judge_line.stamp.nanoseconds()) / 1e+9;
      const auto & unsafe_interval = object_info->unsafe_interval().value();
      diag += fmt::format(
        "object {0} was judged as {1} when ego passed the 2nd pass judge line at time {2} "
        "previously with the estimated velocity {3}[m/s], but now at {4} collision is detected "
        "after {5}~{6} seconds on the lanelet of type {7} with the estimated current velocity "
        "{8}[m/s]\n",
        object_info->uuid_str,                                             // 0
        magic_enum::enum_name(decision_at_2nd_pass_judge_line.safe_type),  // 1
        decision_at_2nd_pass_judge_line_time,                              // 2
        decision_at_2nd_pass_judge_line.observed_velocity,                 // 3
        now_double,                                                        // 4
        unsafe_interval.interval_time.first,                               // 5
        unsafe_interval.interval_time.second,                              // 6
        magic_enum::enum_name(unsafe_interval.lane_position),              // 7
        object_info->observed_velocity()                                   // 8
      );
    }
  }
  return diag;
}

std::string IntersectionModule::generateEgoRiskEvasiveDiagnosis(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t closest_idx,
  const IntersectionModule::TimeDistanceArray & ego_time_distance_array,
  const std::vector<std::pair<
    IntersectionModule::CollisionStatus::BlameType, std::shared_ptr<intersection::ObjectInfo>>> &
    too_late_detect_objects,
  [[maybe_unused]] const std::vector<std::pair<
    IntersectionModule::CollisionStatus::BlameType, std::shared_ptr<intersection::ObjectInfo>>> &
    misjudge_objects) const
{
  static constexpr double min_vel = 1e-2;
  std::string diag;
  const double ego_vel = planner_data_->current_velocity->twist.linear.x;
  for (const auto & [blame_type, object_info] : too_late_detect_objects) {
    if (!object_info->unsafe_interval()) {
      continue;
    }
    // object side
    const auto & unsafe_interval = object_info->unsafe_interval().value();
    const auto [begin, end] = unsafe_interval.interval_position;
    const auto &p1 = unsafe_interval.path.at(begin).position,
               p2 = unsafe_interval.path.at(end).position;
    const auto collision_pos =
      tier4_autoware_utils::createPoint((p1.x + p2.x) / 2, (p1.y + p2.y) / 2, (p1.z + p2.z) / 2);
    const auto object_dist_to_margin_point =
      tier4_autoware_utils::calcDistance2d(
        object_info->predicted_object().kinematics.initial_pose_with_covariance.pose.position,
        collision_pos) -
      planner_param_.collision_detection.avoid_collision_by_acceleration
          .object_time_margin_to_collision_point *
        object_info->observed_velocity();
    if (object_dist_to_margin_point <= 0.0) {
      continue;
    }
    const auto object_eta_to_margin_point =
      object_dist_to_margin_point / std::max(min_vel, object_info->observed_velocity());
    // ego side
    const double ego_dist_to_collision_pos =
      motion_utils::calcSignedArcLength(path.points, closest_idx, collision_pos);
    const auto ego_eta_to_collision_pos_it = std::lower_bound(
      ego_time_distance_array.begin(), ego_time_distance_array.end(), ego_dist_to_collision_pos,
      [](const auto & a, const double b) { return a.second < b; });
    if (ego_eta_to_collision_pos_it == ego_time_distance_array.end()) {
      continue;
    }
    const double ego_eta_to_collision_pos = ego_eta_to_collision_pos_it->first;
    if (ego_eta_to_collision_pos < object_eta_to_margin_point) {
      // this case, ego will pass the collision point before this object get close to the collision
      // point within margin just by keeping current plan, so no need to accelerate
      continue;
    }
    diag += fmt::format(
      "the object is expected to approach the collision point by the TTC margin in {0} seconds, "
      "while ego will arrive there in {1} seconds, so ego needs to accelerate from current "
      "velocity {2}[m/s] to {3}[m/s]\n",
      object_eta_to_margin_point,                             // 0
      ego_eta_to_collision_pos,                               // 1
      ego_vel,                                                // 2
      ego_dist_to_collision_pos / object_eta_to_margin_point  // 3
    );
  }
  return diag;
}

IntersectionModule::CollisionStatus IntersectionModule::detectCollision(
  const bool is_over_1st_pass_judge_line,
  const std::optional<bool> is_over_2nd_pass_judge_line) const
{
  // ==========================================================================================
  // if collision is detected for multiple objects, we prioritize collision on the first
  // attention lanelet
  // ==========================================================================================
  bool collision_at_first_lane = false;
  bool collision_at_non_first_lane = false;

  // ==========================================================================================
  // find the objects which are judged as UNSAFE after ego passed pass judge lines.
  //
  // misjudge_objects are those that were once judged as safe when ego passed the pass judge line
  //
  // too_late_detect_objects are those that (1) were not detected when ego passed the pass judge
  // line (2) were judged as dangerous at the same time when ego passed the pass judge line, which
  // means they were expected to have been detected when ego passed the pass judge lines or in the
  // prior iteration, because ego could have judged them as UNSAFE if their information was
  // available at that time.
  //
  // that case is both "too late to stop" and "too late to go" for the planner. and basically
  // detection side is responsible for this fault
  // ==========================================================================================
  std::vector<std::pair<CollisionStatus::BlameType, std::shared_ptr<intersection::ObjectInfo>>>
    misjudge_objects;
  std::vector<std::pair<CollisionStatus::BlameType, std::shared_ptr<intersection::ObjectInfo>>>
    too_late_detect_objects;
  for (const auto & object_info : object_info_manager_.attentionObjects()) {
    if (object_info->is_safe_under_traffic_control()) {
      debug_data_.safe_under_traffic_control_targets.objects.push_back(
        object_info->predicted_object());
      continue;
    }
    if (!object_info->unsafe_info()) {
      continue;
    }
    const auto & unsafe_info = object_info->unsafe_info().value();
    // ==========================================================================================
    // if ego is over the pass judge lines, then the visualization as "too_late_objects" or
    // "misjudge_objects" is more important than that for "unsafe"
    //
    // NOTE: consider a vehicle which was not detected at 1st_pass_judge_passage, and now collision
    // detected on the 1st lane, which is "too_late" for 1st lane passage, but once it decelerated
    // or yielded, so it turned safe, and ego passed the 2nd pass judge line, but at the same it
    // accelerated again, which is "misjudge" for 2nd lane passage. In this case this vehicle is
    // visualized as "misjudge"
    // ==========================================================================================
    auto * debug_container = &debug_data_.unsafe_targets.objects;
    if (unsafe_info.lane_position == intersection::CollisionInterval::LanePosition::FIRST) {
      collision_at_first_lane = true;
    } else {
      collision_at_non_first_lane = true;
    }
    if (
      is_over_1st_pass_judge_line &&
      unsafe_info.lane_position == intersection::CollisionInterval::LanePosition::FIRST) {
      const auto & decision_at_1st_pass_judge_opt =
        object_info->decision_at_1st_pass_judge_line_passage();
      if (!decision_at_1st_pass_judge_opt) {
        too_late_detect_objects.emplace_back(
          CollisionStatus::BlameType::BLAME_AT_FIRST_PASS_JUDGE, object_info);
        debug_container = &debug_data_.too_late_detect_targets.objects;
      } else {
        const auto & decision_at_1st_pass_judge = decision_at_1st_pass_judge_opt.value();
        if (
          decision_at_1st_pass_judge.safe_type !=
          intersection::CollisionKnowledge::SafeType::UNSAFE) {
          misjudge_objects.emplace_back(
            CollisionStatus::BlameType::BLAME_AT_FIRST_PASS_JUDGE, object_info);
          debug_container = &debug_data_.misjudge_targets.objects;
        } else {
          too_late_detect_objects.emplace_back(
            CollisionStatus::BlameType::BLAME_AT_FIRST_PASS_JUDGE, object_info);
          debug_container = &debug_data_.too_late_detect_targets.objects;
        }
      }
    }
    if (is_over_2nd_pass_judge_line && is_over_2nd_pass_judge_line.value()) {
      const auto & decision_at_2nd_pass_judge_opt =
        object_info->decision_at_2nd_pass_judge_line_passage();
      if (!decision_at_2nd_pass_judge_opt) {
        too_late_detect_objects.emplace_back(
          CollisionStatus::BlameType::BLAME_AT_SECOND_PASS_JUDGE, object_info);
        debug_container = &debug_data_.too_late_detect_targets.objects;
      } else {
        const auto & decision_at_2nd_pass_judge = decision_at_2nd_pass_judge_opt.value();
        if (
          decision_at_2nd_pass_judge.safe_type !=
          intersection::CollisionKnowledge::SafeType::UNSAFE) {
          misjudge_objects.emplace_back(
            CollisionStatus::BlameType::BLAME_AT_SECOND_PASS_JUDGE, object_info);
          debug_container = &debug_data_.misjudge_targets.objects;
        } else {
          too_late_detect_objects.emplace_back(
            CollisionStatus::BlameType::BLAME_AT_SECOND_PASS_JUDGE, object_info);
          debug_container = &debug_data_.too_late_detect_targets.objects;
        }
      }
    }
    debug_container->emplace_back(object_info->predicted_object());
  }
  if (collision_at_first_lane) {
    return {
      true, intersection::CollisionInterval::FIRST, too_late_detect_objects, misjudge_objects};
  } else if (collision_at_non_first_lane) {
    return {true, intersection::CollisionInterval::ELSE, too_late_detect_objects, misjudge_objects};
  }
  return {false, intersection::CollisionInterval::ELSE, too_late_detect_objects, misjudge_objects};
}

/*
bool IntersectionModule::checkCollision(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, TargetObjects * target_objects,
  const intersection::PathLanelets & path_lanelets, const size_t closest_idx,
  const size_t last_intersection_stopline_candidate_idx, const double time_delay,
  const TrafficPrioritizedLevel & traffic_prioritized_level)
{
  using lanelet::utils::getArcCoordinates;
  using lanelet::utils::getPolygonFromArcLength;

  // check collision between target_objects predicted path and ego lane
  // cut the predicted path at passing_time
  tier4_debug_msgs::msg::Float64MultiArrayStamped ego_ttc_time_array;
  const auto time_distance_array = calcIntersectionPassingTime(
    path, closest_idx, last_intersection_stopline_candidate_idx, time_delay, &ego_ttc_time_array);

  if (
    std::find(planner_param_.debug.ttc.begin(), planner_param_.debug.ttc.end(), lane_id_) !=
    planner_param_.debug.ttc.end()) {
    ego_ttc_time_array.stamp = path.header.stamp;
    ego_ttc_pub_->publish(ego_ttc_time_array);
  }

  const double passing_time = time_distance_array.back().first;
  cutPredictPathWithDuration(target_objects, passing_time);

  const auto & concat_lanelets = path_lanelets.all;
  const auto closest_arc_coords = getArcCoordinates(
    concat_lanelets, tier4_autoware_utils::getPose(path.points.at(closest_idx).point));
  const auto & ego_lane = path_lanelets.ego_or_entry2exit;
  debug_data_.ego_lane = ego_lane.polygon3d();
  const auto ego_poly = ego_lane.polygon2d().basicPolygon();

  // change TTC margin based on ego traffic light color
  const auto [collision_start_margin_time, collision_end_margin_time] = [&]() {
    if (traffic_prioritized_level == TrafficPrioritizedLevel::FULLY_PRIORITIZED) {
      return std::make_pair(
        planner_param_.collision_detection.fully_prioritized.collision_start_margin_time,
        planner_param_.collision_detection.fully_prioritized.collision_end_margin_time);
    }
    if (traffic_prioritized_level == TrafficPrioritizedLevel::PARTIALLY_PRIORITIZED) {
      return std::make_pair(
        planner_param_.collision_detection.partially_prioritized.collision_start_margin_time,
        planner_param_.collision_detection.partially_prioritized.collision_end_margin_time);
    }
    return std::make_pair(
      planner_param_.collision_detection.not_prioritized.collision_start_margin_time,
      planner_param_.collision_detection.not_prioritized.collision_end_margin_time);
  }();
  const auto expectedToStopBeforeStopLine = [&](const TargetObject & target_object) {
    if (!target_object.dist_to_stopline) {
      return false;
    }
    const double dist_to_stopline = target_object.dist_to_stopline.value();
    if (dist_to_stopline < 0) {
      return false;
    }
    const double v = target_object.object.kinematics.initial_twist_with_covariance.twist.linear.x;
    const double braking_distance =
      v * v /
      (2.0 * std::fabs(planner_param_.collision_detection.ignore_on_amber_traffic_light
                         .object_expected_deceleration));
    return dist_to_stopline > braking_distance;
  };
  const auto isTolerableOvershoot = [&](const TargetObject & target_object) {
    if (
      !target_object.attention_lanelet || !target_object.dist_to_stopline ||
      !target_object.stopline) {
      return false;
    }
    const double dist_to_stopline = target_object.dist_to_stopline.value();
    const double v = target_object.object.kinematics.initial_twist_with_covariance.twist.linear.x;
    const double braking_distance =
      v * v /
      (2.0 * std::fabs(planner_param_.collision_detection.ignore_on_amber_traffic_light
                         .object_expected_deceleration));
    if (dist_to_stopline > braking_distance) {
      return false;
    }
    const auto stopline_front = target_object.stopline.value().front();
    const auto stopline_back = target_object.stopline.value().back();
    tier4_autoware_utils::LineString2d object_line;
    object_line.emplace_back(
      (stopline_front.x() + stopline_back.x()) / 2.0,
      (stopline_front.y() + stopline_back.y()) / 2.0);
    const auto stopline_mid = object_line.front();
    const auto endpoint = target_object.attention_lanelet.value().centerline().back();
    object_line.emplace_back(endpoint.x(), endpoint.y());
    std::vector<tier4_autoware_utils::Point2d> intersections;
    bg::intersection(object_line, ego_lane.centerline2d().basicLineString(), intersections);
    if (intersections.empty()) {
      return false;
    }
    const auto collision_point = intersections.front();
    // distance from object expected stop position to collision point
    const double stopline_to_object = -1.0 * dist_to_stopline + braking_distance;
    const double stopline_to_collision =
      std::hypot(collision_point.x() - stopline_mid.x(), collision_point.y() - stopline_mid.y());
    const double object2collision = stopline_to_collision - stopline_to_object;
    const double margin =
      planner_param_.collision_detection.ignore_on_red_traffic_light.object_margin_to_path;
    return (object2collision > margin) || (object2collision < 0);
  };
  // check collision between predicted_path and ego_area
  tier4_debug_msgs::msg::Float64MultiArrayStamped object_ttc_time_array;
  object_ttc_time_array.layout.dim.resize(3);
  object_ttc_time_array.layout.dim.at(0).label = "objects";
  object_ttc_time_array.layout.dim.at(0).size = 1;  // incremented in the loop, first row is lane_id
  object_ttc_time_array.layout.dim.at(1).label =
    "[x, y, th, length, width, speed, dangerous, ref_obj_enter_time, ref_obj_exit_time, "
    "start_time, start_dist, "
    "end_time, end_dist, first_collision_x, first_collision_y, last_collision_x, last_collision_y, "
    "prd_x[0], ... pred_x[19], pred_y[0], ... pred_y[19]]";
  object_ttc_time_array.layout.dim.at(1).size = 57;
  for (unsigned i = 0; i < object_ttc_time_array.layout.dim.at(1).size; ++i) {
    object_ttc_time_array.data.push_back(lane_id_);
  }
  bool collision_detected = false;
  for (const auto & target_object : target_objects->all_attention_objects) {
    const auto & object = target_object.object;
    // If the vehicle is expected to stop before their stopline, ignore
    const bool expected_to_stop_before_stopline = expectedToStopBeforeStopLine(target_object);
    if (
      traffic_prioritized_level == TrafficPrioritizedLevel::PARTIALLY_PRIORITIZED &&
      expected_to_stop_before_stopline) {
      debug_data_.amber_ignore_targets.objects.push_back(object);
      continue;
    }
    const bool is_tolerable_overshoot = isTolerableOvershoot(target_object);
    if (
      traffic_prioritized_level == TrafficPrioritizedLevel::FULLY_PRIORITIZED &&
      !expected_to_stop_before_stopline && is_tolerable_overshoot) {
      debug_data_.red_overshoot_ignore_targets.objects.push_back(object);
      continue;
    }
    for (const auto & predicted_path : object.kinematics.predicted_paths) {
      if (
        predicted_path.confidence <
        planner_param_.collision_detection.min_predicted_path_confidence) {
        // ignore the predicted path with too low confidence
        continue;
      }

      // collision point
      const auto first_itr = std::adjacent_find(
        predicted_path.path.cbegin(), predicted_path.path.cend(),
        [&ego_poly, &object](const auto & a, const auto & b) {
          return bg::intersects(ego_poly, ::createOneStepPolygon(a, b, object.shape));
        });
      if (first_itr == predicted_path.path.cend()) continue;
      const auto last_itr = std::adjacent_find(
        predicted_path.path.crbegin(), predicted_path.path.crend(),
        [&ego_poly, &object](const auto & a, const auto & b) {
          return bg::intersects(ego_poly, ::createOneStepPolygon(a, b, object.shape));
        });
      if (last_itr == predicted_path.path.crend()) continue;

      // possible collision time interval
      const double ref_object_enter_time =
        static_cast<double>(first_itr - predicted_path.path.begin()) *
        rclcpp::Duration(predicted_path.time_step).seconds();
      auto start_time_distance_itr = time_distance_array.begin();
      if (ref_object_enter_time - collision_start_margin_time > 0) {
        // start of possible ego position in the intersection
        start_time_distance_itr = std::lower_bound(
          time_distance_array.begin(), time_distance_array.end(),
          ref_object_enter_time - collision_start_margin_time,
          [](const auto & a, const double b) { return a.first < b; });
        if (start_time_distance_itr == time_distance_array.end()) {
          // ego is already at the exit of intersection when npc is at collision point even if npc
          // accelerates so ego's position interval is empty
          continue;
        }
      }
      const double ref_object_exit_time =
        static_cast<double>(last_itr.base() - predicted_path.path.begin()) *
        rclcpp::Duration(predicted_path.time_step).seconds();
      auto end_time_distance_itr = std::lower_bound(
        time_distance_array.begin(), time_distance_array.end(),
        ref_object_exit_time + collision_end_margin_time,
        [](const auto & a, const double b) { return a.first < b; });
      if (end_time_distance_itr == time_distance_array.end()) {
        // ego is already passing the intersection, when npc is is at collision point
        // so ego's position interval is up to the end of intersection lane
        end_time_distance_itr = time_distance_array.end() - 1;
      }
      const double start_arc_length = std::max(
        0.0, closest_arc_coords.length + (*start_time_distance_itr).second -
               planner_data_->vehicle_info_.rear_overhang_m);
      const double end_arc_length = std::min(
        closest_arc_coords.length + (*end_time_distance_itr).second +
          planner_data_->vehicle_info_.max_longitudinal_offset_m,
        lanelet::utils::getLaneletLength2d(concat_lanelets));

      const auto trimmed_ego_polygon =
        getPolygonFromArcLength(concat_lanelets, start_arc_length, end_arc_length);

      if (trimmed_ego_polygon.empty()) {
        continue;
      }

      Polygon2d polygon{};
      for (const auto & p : trimmed_ego_polygon) {
        polygon.outer().emplace_back(p.x(), p.y());
      }
      bg::correct(polygon);
      debug_data_.candidate_collision_ego_lane_polygon = toGeomPoly(polygon);

      for (auto itr = first_itr; itr != last_itr.base(); ++itr) {
        const auto footprint_polygon = tier4_autoware_utils::toPolygon2d(*itr, object.shape);
        if (bg::intersects(polygon, footprint_polygon)) {
          collision_detected = true;
          break;
        }
      }
      object_ttc_time_array.layout.dim.at(0).size++;
      const auto & pos = object..position;
      const auto & shape = object.shape;
      object_ttc_time_array.data.insert(
        object_ttc_time_array.data.end(),
        {pos.x, pos.y, tf2::getYaw(object.kinematics.initial_pose_with_covariance.pose.orientation),
         shape.dimensions.x, shape.dimensions.y,
         object.kinematics.initial_twist_with_covariance.twist.linear.x,
         1.0 * static_cast<int>(collision_detected), ref_object_enter_time, ref_object_exit_time,
         start_time_distance_itr->first, start_time_distance_itr->second,
         end_time_distance_itr->first, end_time_distance_itr->second, first_itr->position.x,
         first_itr->position.y, last_itr->position.x, last_itr->position.y});
      for (unsigned i = 0; i < 20; i++) {
        const auto & pos =
          predicted_path.path.at(std::min<size_t>(i, predicted_path.path.size() - 1)).position;
        object_ttc_time_array.data.push_back(pos.x);
        object_ttc_time_array.data.push_back(pos.y);
      }
      if (collision_detected) {
        debug_data_.conflicting_targets.objects.push_back(object);
        break;
      }
    }
  }

  if (
    std::find(planner_param_.debug.ttc.begin(), planner_param_.debug.ttc.end(), lane_id_) !=
    planner_param_.debug.ttc.end()) {
    object_ttc_time_array.stamp = path.header.stamp;
    object_ttc_pub_->publish(object_ttc_time_array);
  }

  return collision_detected;
}
*/

std::optional<size_t> IntersectionModule::checkAngleForTargetLanelets(

  const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & target_lanelets,
  const bool is_parked_vehicle) const
{
  const double detection_area_angle_thr = planner_param_.common.attention_area_angle_threshold;
  const bool consider_wrong_direction_vehicle =
    planner_param_.common.attention_area_angle_threshold;
  const double dist_margin = planner_param_.common.attention_area_margin;

  for (unsigned i = 0; i < target_lanelets.size(); ++i) {
    const auto & ll = target_lanelets.at(i);
    if (!lanelet::utils::isInLanelet(pose, ll, dist_margin)) {
      continue;
    }
    const double ll_angle = lanelet::utils::getLaneletAngle(ll, pose.position);
    const double pose_angle = tf2::getYaw(pose.orientation);
    const double angle_diff = tier4_autoware_utils::normalizeRadian(ll_angle - pose_angle, -M_PI);
    if (consider_wrong_direction_vehicle) {
      if (std::fabs(angle_diff) > 1.57 || std::fabs(angle_diff) < detection_area_angle_thr) {
        return std::make_optional<size_t>(i);
      }
    } else {
      if (std::fabs(angle_diff) < detection_area_angle_thr) {
        return std::make_optional<size_t>(i);
      }
      // NOTE: sometimes parked vehicle direction is reversed even if its longitudinal velocity is
      // positive
      if (
        is_parked_vehicle && (std::fabs(angle_diff) < detection_area_angle_thr ||
                              (std::fabs(angle_diff + M_PI) < detection_area_angle_thr))) {
        return std::make_optional<size_t>(i);
      }
    }
  }
  return std::nullopt;
}

IntersectionModule::TimeDistanceArray IntersectionModule::calcIntersectionPassingTime(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const bool is_prioritized,
  const intersection::IntersectionStopLines & intersection_stoplines,
  tier4_debug_msgs::msg::Float64MultiArrayStamped * debug_ttc_array) const
{
  const double intersection_velocity =
    planner_param_.collision_detection.velocity_profile.default_velocity;
  const double minimum_ego_velocity =
    planner_param_.collision_detection.velocity_profile.minimum_default_velocity;
  const bool use_upstream_velocity =
    planner_param_.collision_detection.velocity_profile.use_upstream;
  const double minimum_upstream_velocity =
    planner_param_.collision_detection.velocity_profile.minimum_upstream_velocity;
  const double current_velocity = planner_data_->current_velocity->twist.linear.x;

  // ==========================================================================================
  // if ego is waiting for collision detection, the entry time into the intersection
  // is a bit delayed for the chattering hold, so we need to "shift" the TimeDistanceArray by
  // this delay
  // ==========================================================================================
  const bool is_go_out = (activated_ && occlusion_activated_);
  const double time_delay = (is_go_out || is_prioritized)
                              ? 0.0
                              : (planner_param_.collision_detection.collision_detection_hold_time -
                                 collision_state_machine_.getDuration());

  // ==========================================================================================
  // to account for the stopline generated by upstream behavior_velocity modules (like walkway,
  // crosswalk), if use_upstream flag is true, the raw velocity of path points after
  // last_intersection_stopline_candidate_idx is used, which maybe almost-zero. at those almost-zero
  // velocity path points, ego future profile is almost "fixed" there.
  //
  // last_intersection_stopline_candidate_idx must be carefully decided especially when ego
  // velocity is almost zero, because if last_intersection_stopline_candidate_idx is at the
  // closest_idx for example, ego is almost "fixed" at current position for the entire
  // spatiotemporal profile, which is judged as SAFE because that profile does not collide
  // with the predicted paths of objects.
  //
  // if second_attention_lane exists, second_attention_stopline_idx is used. if not,
  // max(occlusion_stopline_idx, first_attention_stopline_idx) is used because
  // occlusion_stopline_idx varies depending on the peeking offset parameter
  // ==========================================================================================
  const auto second_attention_stopline_idx = intersection_stoplines.second_attention_stopline;
  const auto occlusion_stopline_idx = intersection_stoplines.occlusion_peeking_stopline.value();
  const auto first_attention_stopline_idx = intersection_stoplines.first_attention_stopline.value();
  const auto closest_idx = intersection_stoplines.closest_idx;
  const auto last_intersection_stopline_candidate_idx =
    second_attention_stopline_idx ? second_attention_stopline_idx.value()
                                  : std::max(occlusion_stopline_idx, first_attention_stopline_idx);

  int assigned_lane_found = false;
  // crop intersection part of the path, and set the reference velocity to intersection_velocity
  // for ego's ttc
  PathWithLaneId reference_path;
  std::optional<size_t> upstream_stopline{std::nullopt};
  for (size_t i = 0; i < path.points.size() - 1; ++i) {
    auto reference_point = path.points.at(i);
    // assume backward velocity is current ego velocity
    if (i < closest_idx) {
      reference_point.point.longitudinal_velocity_mps = current_velocity;
    }
    if (
      i > last_intersection_stopline_candidate_idx &&
      std::fabs(reference_point.point.longitudinal_velocity_mps) <
        std::numeric_limits<double>::epsilon() &&
      !upstream_stopline) {
      upstream_stopline = i;
    }
    if (!use_upstream_velocity) {
      reference_point.point.longitudinal_velocity_mps = intersection_velocity;
    }
    reference_path.points.push_back(reference_point);
    bool has_objective_lane_id = util::hasLaneIds(path.points.at(i), associative_ids_);
    if (assigned_lane_found && !has_objective_lane_id) {
      break;
    }
    assigned_lane_found = has_objective_lane_id;
  }
  if (!assigned_lane_found) {
    return {{0.0, 0.0}};  // has already passed the intersection.
  }

  std::vector<std::pair<double, double>> original_path_xy;
  for (size_t i = 0; i < reference_path.points.size(); ++i) {
    const auto & p = reference_path.points.at(i).point.pose.position;
    original_path_xy.emplace_back(p.x, p.y);
  }

  // apply smoother to reference velocity
  PathWithLaneId smoothed_reference_path = reference_path;
  if (!smoothPath(reference_path, smoothed_reference_path, planner_data_)) {
    smoothed_reference_path = reference_path;
  }

  // calculate when ego is going to reach each (interpolated) points on the path
  TimeDistanceArray time_distance_array{};
  double dist_sum = 0.0;
  double passing_time = time_delay;
  time_distance_array.emplace_back(passing_time, dist_sum);

  // NOTE: `reference_path` is resampled in `reference_smoothed_path`, so
  // `last_intersection_stopline_candidate_idx` makes no sense
  const auto smoothed_path_closest_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
    smoothed_reference_path.points, path.points.at(closest_idx).point.pose,
    planner_data_->ego_nearest_dist_threshold, planner_data_->ego_nearest_yaw_threshold);

  const std::optional<size_t> upstream_stopline_idx_opt = [&]() -> std::optional<size_t> {
    if (upstream_stopline) {
      const auto upstream_stopline_point = path.points.at(upstream_stopline.value()).point.pose;
      return motion_utils::findFirstNearestIndexWithSoftConstraints(
        smoothed_reference_path.points, upstream_stopline_point,
        planner_data_->ego_nearest_dist_threshold, planner_data_->ego_nearest_yaw_threshold);
    } else {
      return std::nullopt;
    }
  }();

  for (size_t i = smoothed_path_closest_idx; i < smoothed_reference_path.points.size() - 1; ++i) {
    const auto & p1 = smoothed_reference_path.points.at(i);
    const auto & p2 = smoothed_reference_path.points.at(i + 1);

    const double dist = tier4_autoware_utils::calcDistance2d(p1, p2);
    dist_sum += dist;

    // use average velocity between p1 and p2
    const double average_velocity =
      (p1.point.longitudinal_velocity_mps + p2.point.longitudinal_velocity_mps) / 2.0;
    const double passing_velocity = [=]() {
      if (use_upstream_velocity) {
        if (upstream_stopline_idx_opt && i > upstream_stopline_idx_opt.value()) {
          return minimum_upstream_velocity;
        }
        return std::max<double>(average_velocity, minimum_ego_velocity);
      } else {
        return std::max<double>(average_velocity, minimum_ego_velocity);
      }
    }();
    passing_time += (dist / passing_velocity);

    time_distance_array.emplace_back(passing_time, dist_sum);
  }
  debug_ttc_array->layout.dim.resize(3);
  debug_ttc_array->layout.dim.at(0).label = "lane_id_@[0][0], ttc_time, ttc_dist, path_x, path_y";
  debug_ttc_array->layout.dim.at(0).size = 5;
  debug_ttc_array->layout.dim.at(1).label = "values";
  debug_ttc_array->layout.dim.at(1).size = time_distance_array.size();
  debug_ttc_array->data.reserve(
    time_distance_array.size() * debug_ttc_array->layout.dim.at(0).size);
  for (unsigned i = 0; i < time_distance_array.size(); ++i) {
    debug_ttc_array->data.push_back(lane_id_);
  }
  for (const auto & [t, d] : time_distance_array) {
    debug_ttc_array->data.push_back(t);
  }
  for (const auto & [t, d] : time_distance_array) {
    debug_ttc_array->data.push_back(d);
  }
  for (size_t i = smoothed_path_closest_idx; i < smoothed_reference_path.points.size(); ++i) {
    const auto & p = smoothed_reference_path.points.at(i).point.pose.position;
    debug_ttc_array->data.push_back(p.x);
  }
  for (size_t i = smoothed_path_closest_idx; i < smoothed_reference_path.points.size(); ++i) {
    const auto & p = smoothed_reference_path.points.at(i).point.pose.position;
    debug_ttc_array->data.push_back(p.y);
  }
  return time_distance_array;
}

}  // namespace behavior_velocity_planner
