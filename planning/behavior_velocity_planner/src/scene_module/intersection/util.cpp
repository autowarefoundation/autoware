// Copyright 2020 Tier IV, Inc.
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

#include <interpolation/spline_interpolation.hpp>
#include <lanelet2_extension/regulatory_elements/road_marking.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scene_module/intersection/util.hpp>
#include <utilization/path_utilization.hpp>
#include <utilization/util.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <algorithm>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

namespace util
{
int insertPoint(
  const geometry_msgs::msg::Pose & in_pose,
  autoware_auto_planning_msgs::msg::PathWithLaneId * inout_path)
{
  static constexpr double dist_thr = 10.0;
  static constexpr double angle_thr = M_PI / 1.5;
  const auto closest_idx_opt =
    motion_utils::findNearestIndex(inout_path->points, in_pose, dist_thr, angle_thr);
  if (!closest_idx_opt) {
    return -1;
  }
  const size_t closest_idx = closest_idx_opt.get();

  int insert_idx = closest_idx;
  if (planning_utils::isAheadOf(in_pose, inout_path->points.at(closest_idx).point.pose)) {
    ++insert_idx;
  }

  autoware_auto_planning_msgs::msg::PathPointWithLaneId inserted_point;
  inserted_point = inout_path->points.at(closest_idx);
  inserted_point.point.pose = in_pose;

  auto it = inout_path->points.begin() + insert_idx;
  inout_path->points.insert(it, inserted_point);

  return insert_idx;
}

bool hasLaneId(const autoware_auto_planning_msgs::msg::PathPointWithLaneId & p, const int id)
{
  for (const auto & pid : p.lane_ids) {
    if (pid == id) {
      return true;
    }
  }
  return false;
}

bool getDuplicatedPointIdx(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Point & point, int * duplicated_point_idx)
{
  for (size_t i = 0; i < path.points.size(); i++) {
    const auto & p = path.points.at(i).point.pose.position;

    constexpr double min_dist = 0.001;
    if (tier4_autoware_utils::calcDistance2d(p, point) < min_dist) {
      *duplicated_point_idx = static_cast<int>(i);
      return true;
    }
  }

  return false;
}

int getFirstPointInsidePolygons(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const std::vector<lanelet::CompoundPolygon3d> & polygons)
{
  int first_idx_inside_lanelet = -1;
  for (size_t i = 0; i < path.points.size(); ++i) {
    bool is_in_lanelet = false;
    auto p = path.points.at(i).point.pose.position;
    for (const auto & polygon : polygons) {
      const auto polygon_2d = lanelet::utils::to2D(polygon);
      is_in_lanelet = bg::within(to_bg2d(p), polygon_2d);
      if (is_in_lanelet) {
        first_idx_inside_lanelet = static_cast<int>(i);
        break;
      }
    }
    if (is_in_lanelet) {
      break;
    }
  }
  return first_idx_inside_lanelet;
}

bool generateStopLine(
  const int lane_id, const std::vector<lanelet::CompoundPolygon3d> detection_areas,
  const std::shared_ptr<const PlannerData> & planner_data, const double stop_line_margin,
  const double keep_detection_line_margin,
  autoware_auto_planning_msgs::msg::PathWithLaneId * original_path,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & target_path,
  StopLineIdx * stop_line_idxs, const rclcpp::Logger logger)
{
  /* set judge line dist */
  const double current_vel = planner_data->current_velocity->twist.linear.x;
  const double current_acc = planner_data->current_acceleration->accel.accel.linear.x;
  const double max_acc = planner_data->max_stop_acceleration_threshold;
  const double max_jerk = planner_data->max_stop_jerk_threshold;
  const double delay_response_time = planner_data->delay_response_time;
  const double pass_judge_line_dist = planning_utils::calcJudgeLineDistWithJerkLimit(
    current_vel, current_acc, max_acc, max_jerk, delay_response_time);

  /* set parameters */
  constexpr double interval = 0.2;

  const int stop_line_margin_idx_dist = std::ceil(stop_line_margin / interval);
  const int keep_detection_line_margin_idx_dist = std::ceil(keep_detection_line_margin / interval);
  const int base2front_idx_dist =
    std::ceil(planner_data->vehicle_info_.max_longitudinal_offset_m / interval);
  const int pass_judge_idx_dist = std::ceil(pass_judge_line_dist / interval);

  /* spline interpolation */
  autoware_auto_planning_msgs::msg::PathWithLaneId path_ip;
  if (!splineInterpolate(target_path, interval, path_ip, logger)) {
    return false;
  }

  int first_idx_inside_lane = -1;
  int pass_judge_line_idx = -1;
  int stop_line_idx = -1;
  int keep_detection_line_idx = -1;
  /* generate stop point */
  // If a stop_line is defined in lanelet_map, use it.
  // else, generates a local stop_line with considering the lane conflicts.
  int first_idx_ip_inside_lane;  // first stop point index for interpolated path.
  int stop_idx_ip;               // stop point index for interpolated path.
  if (getStopPoseIndexFromMap(path_ip, lane_id, planner_data, stop_idx_ip, 10.0, logger)) {
    stop_idx_ip = std::max(stop_idx_ip - base2front_idx_dist, 0);
  } else {
    // get idx of first_inside_lane point
    first_idx_ip_inside_lane = getFirstPointInsidePolygons(path_ip, detection_areas);
    if (first_idx_ip_inside_lane == -1) {
      RCLCPP_DEBUG(logger, "generate stopline, but no intersect line found.");
      return false;
    }
    // only for visualization
    const auto first_inside_point = path_ip.points.at(first_idx_ip_inside_lane).point.pose;

    const auto first_idx_inside_lane_opt =
      motion_utils::findNearestIndex(original_path->points, first_inside_point, 10.0, M_PI_4);
    if (first_idx_inside_lane_opt) {
      first_idx_inside_lane = first_idx_inside_lane_opt.get();
    }

    if (first_idx_inside_lane == 0) {
      RCLCPP_DEBUG(
        logger,
        "path[0] is already in the detection area. This happens if you have "
        "already crossed the stop line or are very far from the intersection. Ignore computation.");
      stop_line_idxs->first_idx_inside_lane = 0;
      stop_line_idxs->pass_judge_line_idx = 0;
      stop_line_idxs->stop_line_idx = 0;
      stop_line_idxs->keep_detection_line_idx = 0;
      return true;
    }
    stop_idx_ip =
      std::max(first_idx_ip_inside_lane - 1 - stop_line_margin_idx_dist - base2front_idx_dist, 0);
  }

  /* insert keep_detection_line */
  const int keep_detection_idx_ip = std::min(
    stop_idx_ip + keep_detection_line_margin_idx_dist, static_cast<int>(path_ip.points.size()) - 1);
  const auto inserted_keep_detection_point = path_ip.points.at(keep_detection_idx_ip).point.pose;
  if (!util::getDuplicatedPointIdx(
        *original_path, inserted_keep_detection_point.position, &keep_detection_line_idx)) {
    keep_detection_line_idx = util::insertPoint(inserted_keep_detection_point, original_path);
  }

  /* insert stop_point */
  const auto inserted_stop_point = path_ip.points.at(stop_idx_ip).point.pose;
  // if path has too close (= duplicated) point to the stop point, do not insert it
  // and consider the index of the duplicated point as stop_line_idx
  if (!util::getDuplicatedPointIdx(*original_path, inserted_stop_point.position, &stop_line_idx)) {
    stop_line_idx = util::insertPoint(inserted_stop_point, original_path);
    ++keep_detection_line_idx;  // the index is incremented by judge stop line insertion
  }

  /* if another stop point exist before intersection stop_line, disable judge_line. */
  bool has_prior_stopline = false;
  for (int i = 0; i < stop_line_idx; ++i) {
    if (std::fabs(original_path->points.at(i).point.longitudinal_velocity_mps) < 0.1) {
      has_prior_stopline = true;
      break;
    }
  }

  /* insert judge point */
  const int pass_judge_idx_ip = std::min(
    static_cast<int>(path_ip.points.size()) - 1, std::max(stop_idx_ip - pass_judge_idx_dist, 0));
  if (has_prior_stopline || stop_idx_ip == pass_judge_idx_ip) {
    pass_judge_line_idx = stop_line_idx;
  } else {
    const auto inserted_pass_judge_point = path_ip.points.at(pass_judge_idx_ip).point.pose;
    // if path has too close (= duplicated) point to the pass judge point, do not insert it
    // and consider the index of the duplicated point as pass_judge_line_idx
    if (!util::getDuplicatedPointIdx(
          *original_path, inserted_pass_judge_point.position, &pass_judge_line_idx)) {
      pass_judge_line_idx = util::insertPoint(inserted_pass_judge_point, original_path);
      ++stop_line_idx;            // stop index is incremented by judge line insertion
      ++keep_detection_line_idx;  // same.
    }
  }

  stop_line_idxs->first_idx_inside_lane = first_idx_inside_lane;
  stop_line_idxs->pass_judge_line_idx = pass_judge_line_idx;
  stop_line_idxs->stop_line_idx = stop_line_idx;
  stop_line_idxs->keep_detection_line_idx = keep_detection_line_idx;

  RCLCPP_DEBUG(
    logger,
    "generateStopLine() : stop_idx = %d, pass_judge_idx = %d, stop_idx_ip = "
    "%d, pass_judge_idx_ip = %d, has_prior_stopline = %d",
    stop_line_idx, pass_judge_line_idx, stop_idx_ip, pass_judge_idx_ip, has_prior_stopline);

  return true;
}

bool getStopPoseIndexFromMap(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int lane_id,
  const std::shared_ptr<const PlannerData> & planner_data, int & stop_idx_ip, int dist_thr,
  const rclcpp::Logger logger)
{
  lanelet::ConstLanelet lanelet =
    planner_data->route_handler_->getLaneletMapPtr()->laneletLayer.get(lane_id);
  const auto road_markings = lanelet.regulatoryElementsAs<lanelet::autoware::RoadMarking>();
  lanelet::ConstLineStrings3d stop_line;
  for (const auto & road_marking : road_markings) {
    const std::string type =
      road_marking->roadMarking().attributeOr(lanelet::AttributeName::Type, "none");
    if (type == lanelet::AttributeValueString::StopLine) {
      stop_line.push_back(road_marking->roadMarking());
      break;  // only one stop_line exists.
    }
  }
  if (stop_line.empty()) {
    return false;
  }

  const auto p_start = stop_line.front().front();
  const auto p_end = stop_line.front().back();
  const LineString2d extended_stop_line =
    planning_utils::extendLine(p_start, p_end, planner_data->stop_line_extend_length);

  for (size_t i = 0; i < path.points.size() - 1; i++) {
    const auto & p_front = path.points.at(i).point.pose.position;
    const auto & p_back = path.points.at(i + 1).point.pose.position;

    const LineString2d path_segment = {{p_front.x, p_front.y}, {p_back.x, p_back.y}};
    std::vector<Point2d> collision_points;
    bg::intersection(extended_stop_line, path_segment, collision_points);

    if (collision_points.empty()) {
      continue;
    }

    stop_idx_ip = i;

    RCLCPP_DEBUG(logger, "found collision point");

    return true;
  }

  geometry_msgs::msg::Pose stop_point_from_map;
  stop_point_from_map.position.x = 0.5 * (p_start.x() + p_end.x());
  stop_point_from_map.position.y = 0.5 * (p_start.y() + p_end.y());
  stop_point_from_map.position.z = 0.5 * (p_start.z() + p_end.z());

  const auto stop_idx_ip_opt =
    motion_utils::findNearestIndex(path.points, stop_point_from_map, static_cast<double>(dist_thr));
  if (!stop_idx_ip_opt) {
    RCLCPP_DEBUG(logger, "found stop line, but not found stop index");
    return false;
  }
  stop_idx_ip = stop_idx_ip_opt.get();

  RCLCPP_DEBUG(logger, "found stop line and stop index");

  return true;
}

bool getObjectiveLanelets(
  lanelet::LaneletMapConstPtr lanelet_map_ptr, lanelet::routing::RoutingGraphPtr routing_graph_ptr,
  const int lane_id, const double detection_area_length,
  std::vector<lanelet::ConstLanelets> * conflicting_lanelets_result,
  lanelet::ConstLanelets * objective_lanelets_result, const rclcpp::Logger logger)
{
  const auto & assigned_lanelet = lanelet_map_ptr->laneletLayer.get(lane_id);

  lanelet::ConstLanelets yield_lanelets;

  // for low priority lane
  // If ego_lane has right of way (i.e. is high priority),
  // ignore yieldLanelets (i.e. low priority lanes)
  const auto right_of_ways = assigned_lanelet.regulatoryElementsAs<lanelet::RightOfWay>();
  for (const auto & right_of_way : right_of_ways) {
    if (lanelet::utils::contains(right_of_way->rightOfWayLanelets(), assigned_lanelet)) {
      for (const auto & yield_lanelet : right_of_way->yieldLanelets()) {
        yield_lanelets.push_back(yield_lanelet);
        for (const auto & previous_lanelet : routing_graph_ptr->previous(yield_lanelet)) {
          yield_lanelets.push_back(previous_lanelet);
        }
      }
    }
  }

  lanelet::ConstLanelets ego_lanelets;

  // for the behind ego-car lane.
  for (const auto & previous_lanelet : routing_graph_ptr->previous(assigned_lanelet)) {
    ego_lanelets.push_back(previous_lanelet);
    for (const auto & following_lanelet : routing_graph_ptr->following(previous_lanelet)) {
      if (lanelet::utils::contains(ego_lanelets, following_lanelet)) {
        continue;
      }
      ego_lanelets.push_back(following_lanelet);
    }
  }

  // get conflicting lanes on assigned lanelet
  const auto & conflicting_lanelets =
    lanelet::utils::getConflictingLanelets(routing_graph_ptr, assigned_lanelet);

  std::vector<lanelet::ConstLanelets>                      // conflicting lanes with "lane_id"
    conflicting_lanelets_ex_yield_ego;                     // excluding ego lanes and yield lanes
  std::vector<lanelet::ConstLanelets> objective_lanelets;  // final objective lanelets

  // exclude yield lanelets and ego lanelets from objective_lanelets
  for (const auto & conflicting_lanelet : conflicting_lanelets) {
    if (lanelet::utils::contains(yield_lanelets, conflicting_lanelet)) {
      continue;
    }
    if (lanelet::utils::contains(ego_lanelets, conflicting_lanelet)) {
      continue;
    }
    conflicting_lanelets_ex_yield_ego.push_back({conflicting_lanelet});
    objective_lanelets.push_back({conflicting_lanelet});
  }

  // get possible lanelet path that reaches conflicting_lane longer than given length
  const double length = detection_area_length;
  lanelet::ConstLanelets objective_and_preceding_lanelets;
  std::set<lanelet::Id> objective_ids;
  for (const auto & ll : objective_lanelets) {
    // Preceding lanes does not include objective_lane so add them at the end
    for (const auto & l : ll) {
      const auto & inserted = objective_ids.insert(l.id());
      if (inserted.second) objective_and_preceding_lanelets.push_back(l);
    }
    // get preceding lanelets without ego_lanelets
    // to prevent the detection area from including the ego lanes and its' preceding lanes.
    const auto lanelet_sequences = lanelet::utils::query::getPrecedingLaneletSequences(
      routing_graph_ptr, ll.front(), length, ego_lanelets);
    for (const auto & ls : lanelet_sequences) {
      for (const auto & l : ls) {
        const auto & inserted = objective_ids.insert(l.id());
        if (inserted.second) objective_and_preceding_lanelets.push_back(l);
      }
    }
  }

  *conflicting_lanelets_result = conflicting_lanelets_ex_yield_ego;
  *objective_lanelets_result = objective_and_preceding_lanelets;

  // set this flag true when debugging
  const bool is_debug = false;
  if (!is_debug) return true;
  std::stringstream ss_c, ss_y, ss_e, ss_o, ss_os;
  for (const auto & l : conflicting_lanelets) {
    ss_c << l.id() << ", ";
  }
  for (const auto & l : yield_lanelets) {
    ss_y << l.id() << ", ";
  }
  for (const auto & l : ego_lanelets) {
    ss_e << l.id() << ", ";
  }
  for (const auto & l : objective_lanelets) {
    ss_o << l.front().id() << ", ";
  }
  for (const auto & l : objective_and_preceding_lanelets) {
    ss_os << l.id() << ", ";
  }
  RCLCPP_INFO(
    logger, "getObjectiveLanelets() conflict = %s yield = %s ego = %s", ss_c.str().c_str(),
    ss_y.str().c_str(), ss_e.str().c_str());
  RCLCPP_INFO(
    logger, "getObjectiveLanelets() object = %s object_sequences = %s", ss_o.str().c_str(),
    ss_os.str().c_str());
  return true;
}

std::vector<lanelet::CompoundPolygon3d> getPolygon3dFromLaneletsVec(
  const std::vector<lanelet::ConstLanelets> & ll_vec, double clip_length)
{
  std::vector<lanelet::CompoundPolygon3d> p_vec;
  for (const auto & ll : ll_vec) {
    const double path_length = lanelet::utils::getLaneletLength3d(ll);
    const auto polygon3d =
      lanelet::utils::getPolygonFromArcLength(ll, path_length - clip_length, path_length);
    p_vec.push_back(polygon3d);
  }
  return p_vec;
}

std::vector<lanelet::CompoundPolygon3d> getPolygon3dFromLanelets(
  const lanelet::ConstLanelets & ll_vec, double clip_length)
{
  std::vector<lanelet::CompoundPolygon3d> p_vec;
  for (const auto & ll : ll_vec) {
    const double path_length = lanelet::utils::getLaneletLength3d({ll});
    const auto polygon3d =
      lanelet::utils::getPolygonFromArcLength({ll}, path_length - clip_length, path_length);
    p_vec.push_back(polygon3d);
  }
  return p_vec;
}

std::vector<int> getLaneletIdsFromLaneletsVec(const std::vector<lanelet::ConstLanelets> & ll_vec)
{
  std::vector<int> id_list;
  for (const auto & lls : ll_vec) {
    for (const auto & ll : lls) {
      id_list.emplace_back(ll.id());
    }
  }
  return id_list;
}

lanelet::ConstLanelet generateOffsetLanelet(
  const lanelet::ConstLanelet lanelet, double right_margin, double left_margin)
{
  lanelet::Points3d lefts, rights;

  const double right_offset = right_margin;
  const double left_offset = left_margin;
  const auto offset_rightBound = lanelet::utils::getRightBoundWithOffset(lanelet, right_offset);
  const auto offset_leftBound = lanelet::utils::getLeftBoundWithOffset(lanelet, left_offset);

  const auto original_left_bound = offset_leftBound;
  const auto original_right_bound = offset_rightBound;

  for (const auto & pt : original_left_bound) {
    lefts.push_back(lanelet::Point3d(pt));
  }
  for (const auto & pt : original_right_bound) {
    rights.push_back(lanelet::Point3d(pt));
  }
  const auto left_bound = lanelet::LineString3d(lanelet::InvalId, lefts);
  const auto right_bound = lanelet::LineString3d(lanelet::InvalId, rights);
  auto lanelet_with_margin = lanelet::Lanelet(lanelet::InvalId, left_bound, right_bound);
  const auto centerline = lanelet::utils::generateFineCenterline(lanelet_with_margin, 5.0);
  lanelet_with_margin.setCenterline(centerline);
  return std::move(lanelet_with_margin);
}

bool generateStopLineBeforeIntersection(
  const int lane_id, lanelet::LaneletMapConstPtr lanelet_map_ptr,
  const std::shared_ptr<const PlannerData> & planner_data,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input_path,
  autoware_auto_planning_msgs::msg::PathWithLaneId * output_path, int * stuck_stop_line_idx,
  int * pass_judge_line_idx, const rclcpp::Logger logger)
{
  /* set judge line dist */
  const double current_vel = planner_data->current_velocity->twist.linear.x;
  const double current_acc = planner_data->current_acceleration->accel.accel.linear.x;
  const double max_acc = planner_data->max_stop_acceleration_threshold;
  const double max_jerk = planner_data->max_stop_jerk_threshold;
  const double delay_response_time = planner_data->delay_response_time;
  const double pass_judge_line_dist = planning_utils::calcJudgeLineDistWithJerkLimit(
    current_vel, current_acc, max_acc, max_jerk, delay_response_time);

  /* set parameters */
  constexpr double interval = 0.2;
  const int base2front_idx_dist =
    std::ceil(planner_data->vehicle_info_.max_longitudinal_offset_m / interval);
  const int pass_judge_idx_dist = std::ceil(pass_judge_line_dist / interval);

  /* spline interpolation */
  autoware_auto_planning_msgs::msg::PathWithLaneId path_ip;
  if (!splineInterpolate(input_path, interval, path_ip, logger)) {
    return false;
  }
  const auto & assigned_lanelet = lanelet_map_ptr->laneletLayer.get(lane_id);
  for (size_t i = 0; i < path_ip.points.size(); i++) {
    const auto & p = path_ip.points.at(i).point.pose;
    if (lanelet::utils::isInLanelet(p, assigned_lanelet, 0.1)) {
      if (static_cast<int>(i) <= 0) {
        RCLCPP_DEBUG(logger, "generate stopline, but no within lanelet.");
        return false;
      }
      int stop_idx_ip;  // stop point index for interpolated path.
      stop_idx_ip = std::max(static_cast<int>(i) - base2front_idx_dist, 0);

      /* insert stop_point */
      const auto inserted_stop_point = path_ip.points.at(stop_idx_ip).point.pose;
      // if path has too close (= duplicated) point to the stop point, do not insert it
      // and consider the index of the duplicated point as *stuck_stop_line_idx
      if (!util::getDuplicatedPointIdx(
            *output_path, inserted_stop_point.position, stuck_stop_line_idx)) {
        *stuck_stop_line_idx = util::insertPoint(inserted_stop_point, output_path);
      }

      /* if another stop point exist before intersection stop_line, disable judge_line. */
      bool has_prior_stopline = false;
      for (int i = 0; i < *stuck_stop_line_idx; ++i) {
        if (std::fabs(output_path->points.at(i).point.longitudinal_velocity_mps) < 0.1) {
          has_prior_stopline = true;
          break;
        }
      }

      /* insert judge point */
      const int pass_judge_idx_ip = std::min(
        static_cast<int>(path_ip.points.size()) - 1,
        std::max(stop_idx_ip - pass_judge_idx_dist, 0));
      if (has_prior_stopline || stop_idx_ip == pass_judge_idx_ip) {
        *pass_judge_line_idx = *stuck_stop_line_idx;
      } else {
        const auto inserted_pass_judge_point = path_ip.points.at(pass_judge_idx_ip).point.pose;
        // if path has too close (= duplicated) point to the pass judge point, do not insert it
        // and consider the index of the duplicated point as pass_judge_line_idx
        if (!util::getDuplicatedPointIdx(
              *output_path, inserted_pass_judge_point.position, pass_judge_line_idx)) {
          *pass_judge_line_idx = util::insertPoint(inserted_pass_judge_point, output_path);
          ++(*stuck_stop_line_idx);  // stop index is incremented by judge line insertion
        }
      }

      RCLCPP_DEBUG(
        logger,
        "generateStopLineBeforeIntersection() : stuck_stop_line_idx = %d, pass_judge_idx = %d,"
        "stop_idx_ip = %d, pass_judge_idx_ip = %d, has_prior_stopline = %d",
        *stuck_stop_line_idx, *pass_judge_line_idx, stop_idx_ip, pass_judge_idx_ip,
        has_prior_stopline);
      return true;
    }
  }
  return false;
}

geometry_msgs::msg::Pose toPose(const geometry_msgs::msg::Point & p)
{
  geometry_msgs::msg::Pose pose;
  pose.position = p;
  return pose;
}

static std::string getTurnDirection(lanelet::ConstLanelet lane)
{
  return lane.attributeOr("turn_direction", "else");
}

bool isOverTargetIndex(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int closest_idx,
  const geometry_msgs::msg::Pose & current_pose, const int target_idx)
{
  if (closest_idx == target_idx) {
    const geometry_msgs::msg::Pose target_pose = path.points.at(target_idx).point.pose;
    return planning_utils::isAheadOf(current_pose, target_pose);
  }
  return static_cast<bool>(closest_idx > target_idx);
}

bool isBeforeTargetIndex(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int closest_idx,
  const geometry_msgs::msg::Pose & current_pose, const int target_idx)
{
  if (closest_idx == target_idx) {
    const geometry_msgs::msg::Pose target_pose = path.points.at(target_idx).point.pose;
    return planning_utils::isAheadOf(target_pose, current_pose);
  }
  return static_cast<bool>(target_idx > closest_idx);
}

static std::vector<int> getAllAdjacentLanelets(
  const lanelet::routing::RoutingGraphPtr routing_graph, lanelet::ConstLanelet lane)
{
  std::set<int> results;

  results.insert(lane.id());

  auto it = routing_graph->adjacentRight(lane);
  // take all lane on the right side
  while (!!it) {
    results.insert(it.get().id());
    it = routing_graph->adjacentRight(it.get());
  }
  // take all lane on the left side
  it = routing_graph->adjacentLeft(lane);
  while (!!it) {
    results.insert(it.get().id());
    it = routing_graph->adjacentLeft(it.get());
  }

  return std::vector<int>(results.begin(), results.end());
}

std::vector<int> extendedAdjacentDirectionLanes(
  const lanelet::LaneletMapPtr map, const lanelet::routing::RoutingGraphPtr routing_graph,
  lanelet::ConstLanelet lane)
{
  // some of the intersections are not well-formed, and "adjacent" turning
  // lanelets are not sharing the LineStrings
  const std::string turn_direction = getTurnDirection(lane);
  if (turn_direction != "left" && turn_direction != "right" && turn_direction != "straight")
    return std::vector<int>();

  std::set<int> previous_lanelet_ids;
  for (auto && previous_lanelet : routing_graph->previous(lane)) {
    previous_lanelet_ids.insert(previous_lanelet.id());
  }

  std::set<int> besides_previous_lanelet_ids;
  for (auto && previous_lanelet_id : previous_lanelet_ids) {
    lanelet::ConstLanelet previous_lanelet = map->laneletLayer.get(previous_lanelet_id);
    for (auto && beside_lanelet : getAllAdjacentLanelets(routing_graph, previous_lanelet)) {
      besides_previous_lanelet_ids.insert(beside_lanelet);
    }
  }

  std::set<int> following_turning_lanelets;
  following_turning_lanelets.insert(lane.id());
  for (auto && besides_previous_lanelet_id : besides_previous_lanelet_ids) {
    lanelet::ConstLanelet besides_previous_lanelet =
      map->laneletLayer.get(besides_previous_lanelet_id);
    for (auto && following_lanelet : routing_graph->following(besides_previous_lanelet)) {
      // if this has {"turn_direction", "${turn_direction}"}, take this
      if (getTurnDirection(following_lanelet) == turn_direction)
        following_turning_lanelets.insert(following_lanelet.id());
    }
  }
  return std::vector<int>(following_turning_lanelets.begin(), following_turning_lanelets.end());
}

std::optional<Polygon2d> getIntersectionArea(
  lanelet::ConstLanelet assigned_lane, lanelet::LaneletMapConstPtr lanelet_map_ptr)
{
  const std::string area_id_str = assigned_lane.attributeOr("intersection_area", "else");
  if (area_id_str == "else") return std::nullopt;

  const int area_id = std::atoi(area_id_str.c_str());
  const auto poly_3d = lanelet_map_ptr->polygonLayer.get(area_id);
  Polygon2d poly{};
  for (const auto & p : poly_3d) poly.outer().emplace_back(p.x(), p.y());
  return std::make_optional(poly);
}

}  // namespace util
}  // namespace behavior_velocity_planner
