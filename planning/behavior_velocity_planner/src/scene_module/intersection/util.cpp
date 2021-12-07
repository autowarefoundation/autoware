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
#include <utilization/interpolate.hpp>
#include <utilization/util.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <algorithm>
#include <memory>
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
  int closest_idx = -1;
  if (!planning_utils::calcClosestIndex(*inout_path, in_pose, closest_idx, dist_thr, angle_thr)) {
    return -1;
  }
  int insert_idx = closest_idx;
  if (isAheadOf(in_pose, inout_path->points.at(closest_idx).point.pose)) {
    ++insert_idx;
  }

  autoware_auto_planning_msgs::msg::PathPointWithLaneId inserted_point;
  inserted_point = inout_path->points.at(closest_idx);
  inserted_point.point.pose = in_pose;

  auto it = inout_path->points.begin() + insert_idx;
  inout_path->points.insert(it, inserted_point);

  return insert_idx;
}

bool splineInterpolate(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input, const double interval,
  autoware_auto_planning_msgs::msg::PathWithLaneId * output, const rclcpp::Logger logger)
{
  *output = input;

  if (input.points.size() <= 1) {
    RCLCPP_WARN(logger, "Do not interpolate because path size is 1.");
    return false;
  }

  static constexpr double ep = 1.0e-8;

  // calc arclength for path
  std::vector<double> base_x;
  std::vector<double> base_y;
  std::vector<double> base_z;
  for (const auto & p : input.points) {
    base_x.push_back(p.point.pose.position.x);
    base_y.push_back(p.point.pose.position.y);
    base_z.push_back(p.point.pose.position.z);
  }
  std::vector<double> base_s = interpolation::calcEuclidDist(base_x, base_y);

  // remove duplicating sample points
  {
    size_t Ns = base_s.size();
    size_t i = 1;
    while (i < Ns) {
      if (std::fabs(base_s[i - 1] - base_s[i]) < ep) {
        base_s.erase(base_s.begin() + i);
        base_x.erase(base_x.begin() + i);
        base_y.erase(base_y.begin() + i);
        base_z.erase(base_z.begin() + i);
        Ns -= 1;
        i -= 1;
      }
      ++i;
    }
  }

  std::vector<double> resampled_s;
  for (double d = 0.0; d < base_s.back() - ep; d += interval) {
    resampled_s.push_back(d);
  }

  // do spline for xy
  const std::vector<double> resampled_x = ::interpolation::slerp(base_s, base_x, resampled_s);
  const std::vector<double> resampled_y = ::interpolation::slerp(base_s, base_y, resampled_s);
  const std::vector<double> resampled_z = ::interpolation::slerp(base_s, base_z, resampled_s);

  // set xy
  output->points.clear();
  for (size_t i = 0; i < resampled_s.size(); i++) {
    autoware_auto_planning_msgs::msg::PathPointWithLaneId p;
    p.point.pose.position.x = resampled_x.at(i);
    p.point.pose.position.y = resampled_y.at(i);
    p.point.pose.position.z = resampled_z.at(i);
    output->points.push_back(p);
  }

  // set yaw
  for (int i = 1; i < static_cast<int>(resampled_s.size()) - 1; i++) {
    auto p = output->points.at(i - 1).point.pose.position;
    auto n = output->points.at(i + 1).point.pose.position;
    double yaw = std::atan2(n.y - p.y, n.x - p.x);
    output->points.at(i).point.pose.orientation = planning_utils::getQuaternionFromYaw(yaw);
  }
  if (output->points.size() > 1) {
    size_t l = resampled_s.size();
    output->points.front().point.pose.orientation = output->points.at(1).point.pose.orientation;
    output->points.back().point.pose.orientation = output->points.at(l - 1).point.pose.orientation;
  }
  return true;
}

geometry_msgs::msg::Pose getAheadPose(
  const size_t start_idx, const double ahead_dist,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  if (path.points.size() == 0) {
    return geometry_msgs::msg::Pose{};
  }

  double curr_dist = 0.0;
  double prev_dist = 0.0;
  for (size_t i = start_idx; i < path.points.size() - 1; ++i) {
    const geometry_msgs::msg::Pose p0 = path.points.at(i).point.pose;
    const geometry_msgs::msg::Pose p1 = path.points.at(i + 1).point.pose;
    curr_dist += planning_utils::calcDist2d(p0, p1);
    if (curr_dist > ahead_dist) {
      const double dl = std::max(curr_dist - prev_dist, 0.0001 /* avoid 0 divide */);
      const double w_p0 = (curr_dist - ahead_dist) / dl;
      const double w_p1 = (ahead_dist - prev_dist) / dl;
      geometry_msgs::msg::Pose p;
      p.position.x = w_p0 * p0.position.x + w_p1 * p1.position.x;
      p.position.y = w_p0 * p0.position.y + w_p1 * p1.position.y;
      p.position.z = w_p0 * p0.position.z + w_p1 * p1.position.z;
      tf2::Quaternion q0_tf, q1_tf;
      tf2::fromMsg(p0.orientation, q0_tf);
      tf2::fromMsg(p1.orientation, q1_tf);
      p.orientation = tf2::toMsg(q0_tf.slerp(q1_tf, w_p1));
      return p;
    }
    prev_dist = curr_dist;
  }
  return path.points.back().point.pose;
}

bool setVelocityFrom(
  const size_t idx, const double vel, autoware_auto_planning_msgs::msg::PathWithLaneId * input)
{
  for (size_t i = idx; i < input->points.size(); ++i) {
    input->points.at(i).point.longitudinal_velocity_mps =
      std::min(static_cast<float>(vel), input->points.at(i).point.longitudinal_velocity_mps);
  }
  return true;
}

bool isAheadOf(const geometry_msgs::msg::Pose & target, const geometry_msgs::msg::Pose & origin)
{
  geometry_msgs::msg::Pose p = planning_utils::transformRelCoordinate2D(target, origin);
  bool is_target_ahead = (p.position.x > 0.0);
  return is_target_ahead;
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

bool hasDuplicatedPoint(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Point & point, int * duplicated_point_idx)
{
  for (size_t i = 0; i < path.points.size(); i++) {
    const auto & p = path.points.at(i).point.pose.position;

    constexpr double min_dist = 0.001;
    if (planning_utils::calcDist2d(p, point) < min_dist) {
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
  const std::shared_ptr<const PlannerData> & planner_data,
  const IntersectionModule::PlannerParam & planner_param,
  autoware_auto_planning_msgs::msg::PathWithLaneId * original_path,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & target_path, int * stop_line_idx,
  int * pass_judge_line_idx, int * first_idx_inside_lane, const rclcpp::Logger logger)
{
  /* set judge line dist */
  const double current_vel = planner_data->current_velocity->twist.linear.x;
  const double max_acc = planner_data->max_stop_acceleration_threshold;
  const double delay_response_time = planner_data->delay_response_time;
  const double pass_judge_line_dist =
    planning_utils::calcJudgeLineDistWithAccLimit(current_vel, max_acc, delay_response_time);

  /* set parameters */
  constexpr double interval = 0.2;

  const int margin_idx_dist = std::ceil(planner_param.stop_line_margin / interval);
  const int base2front_idx_dist =
    std::ceil(planner_data->vehicle_info_.max_longitudinal_offset_m / interval);
  const int pass_judge_idx_dist = std::ceil(pass_judge_line_dist / interval);

  /* spline interpolation */
  autoware_auto_planning_msgs::msg::PathWithLaneId path_ip;
  if (!util::splineInterpolate(target_path, interval, &path_ip, logger)) {
    return false;
  }

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
    planning_utils::calcClosestIndex(
      *original_path, first_inside_point, *first_idx_inside_lane, 10.0);
    if (*first_idx_inside_lane == 0) {
      RCLCPP_DEBUG(
        logger,
        "path[0] is already in the detection area. This happens if you have "
        "already crossed the stop line or are very far from the intersection. Ignore computation.");
      *stop_line_idx = 0;
      *pass_judge_line_idx = 0;
      return true;
    }
    stop_idx_ip = std::max(first_idx_ip_inside_lane - 1 - margin_idx_dist - base2front_idx_dist, 0);
  }

  /* insert stop_point */
  const auto inserted_stop_point = path_ip.points.at(stop_idx_ip).point.pose;
  // if path has too close (= duplicated) point to the stop point, do not insert it
  // and consider the index of the duplicated point as stop_line_idx
  if (!util::hasDuplicatedPoint(*original_path, inserted_stop_point.position, stop_line_idx)) {
    *stop_line_idx = util::insertPoint(inserted_stop_point, original_path);
  }

  /* if another stop point exist before intersection stop_line, disable judge_line. */
  bool has_prior_stopline = false;
  for (int i = 0; i < *stop_line_idx; ++i) {
    if (std::fabs(original_path->points.at(i).point.longitudinal_velocity_mps) < 0.1) {
      has_prior_stopline = true;
      break;
    }
  }

  /* insert judge point */
  const int pass_judge_idx_ip = std::min(
    static_cast<int>(path_ip.points.size()) - 1, std::max(stop_idx_ip - pass_judge_idx_dist, 0));
  if (has_prior_stopline || stop_idx_ip == pass_judge_idx_ip) {
    *pass_judge_line_idx = *stop_line_idx;
  } else {
    const auto inserted_pass_judge_point = path_ip.points.at(pass_judge_idx_ip).point.pose;
    // if path has too close (= duplicated) point to the pass judge point, do not insert it
    // and consider the index of the duplicated point as pass_judge_line_idx
    if (!util::hasDuplicatedPoint(
          *original_path, inserted_pass_judge_point.position, pass_judge_line_idx)) {
      *pass_judge_line_idx = util::insertPoint(inserted_pass_judge_point, original_path);
      ++(*stop_line_idx);  // stop index is incremented by judge line insertion
    }
  }

  RCLCPP_DEBUG(
    logger,
    "generateStopLine() : stop_idx = %d, pass_judge_idx = %d, stop_idx_ip = "
    "%d, pass_judge_idx_ip = %d, has_prior_stopline = %d",
    *stop_line_idx, *pass_judge_line_idx, stop_idx_ip, pass_judge_idx_ip, has_prior_stopline);

  return true;
}

bool getStopPoseIndexFromMap(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int lane_id,
  const std::shared_ptr<const PlannerData> & planner_data, int & stop_idx_ip, int dist_thr,
  const rclcpp::Logger logger)
{
  lanelet::ConstLanelet lanelet = planner_data->lanelet_map->laneletLayer.get(lane_id);
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

  geometry_msgs::msg::Point stop_point_from_map;
  stop_point_from_map.x = 0.5 * (p_start.x() + p_end.x());
  stop_point_from_map.y = 0.5 * (p_start.y() + p_end.y());
  stop_point_from_map.z = 0.5 * (p_start.z() + p_end.z());

  if (!planning_utils::calcClosestIndex(path, stop_point_from_map, stop_idx_ip, dist_thr)) {
    RCLCPP_DEBUG(logger, "found stop line, but not found stop index");
    return false;
  }

  RCLCPP_DEBUG(logger, "found stop line and stop index");

  return true;
}

bool getObjectiveLanelets(
  lanelet::LaneletMapConstPtr lanelet_map_ptr, lanelet::routing::RoutingGraphPtr routing_graph_ptr,
  const int lane_id, const IntersectionModule::PlannerParam & planner_param,
  std::vector<lanelet::ConstLanelets> * conflicting_lanelets_result,
  std::vector<lanelet::ConstLanelets> * objective_lanelets_result, const rclcpp::Logger logger)
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
  const double length = planner_param.detection_area_length;
  std::vector<lanelet::ConstLanelets> objective_lanelets_sequences;
  for (const auto & ll : objective_lanelets) {
    // Preceding lanes does not include objective_lane so add them at the end
    objective_lanelets_sequences.push_back(ll);
    // get preceding lanelets without ego_lanelets
    // to prevent the detection area from including the ego lanes and its' preceding lanes.
    const auto lanelet_sequences = lanelet::utils::query::getPrecedingLaneletSequences(
      routing_graph_ptr, ll.front(), length, ego_lanelets);
    for (auto & l : lanelet_sequences) {
      objective_lanelets_sequences.push_back(l);
    }
  }

  *conflicting_lanelets_result = conflicting_lanelets_ex_yield_ego;
  *objective_lanelets_result = objective_lanelets_sequences;

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
  for (const auto & l : objective_lanelets_sequences) {
    for (const auto & ll : l) {
      ss_os << ll.id() << ", ";
    }
  }
  RCLCPP_DEBUG(
    logger, "getObjectiveLanelets() conflict = %s yield = %s ego = %s", ss_c.str().c_str(),
    ss_y.str().c_str(), ss_e.str().c_str());
  RCLCPP_DEBUG(
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

double calcArcLengthFromPath(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input_path, const size_t src_idx,
  const size_t dst_idx)
{
  double length{0.0};
  for (size_t i = src_idx; i < dst_idx; ++i) {
    const double dx_wp = input_path.points.at(i + 1).point.pose.position.x -
                         input_path.points.at(i).point.pose.position.x;
    const double dy_wp = input_path.points.at(i + 1).point.pose.position.y -
                         input_path.points.at(i).point.pose.position.y;
    length += std::hypot(dx_wp, dy_wp);
  }
  return length;
}

}  // namespace util
}  // namespace behavior_velocity_planner
