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

#include <autoware_auto_tf2/tf2_autoware_auto_msgs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scene_module/crosswalk/scene_crosswalk.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <utilization/path_utilization.hpp>
#include <utilization/util.hpp>

#include <cmath>
#include <vector>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;
using Line = bg::model::linestring<Point>;
using motion_utils::calcArcLength;
using motion_utils::calcLateralOffset;
using motion_utils::calcLongitudinalOffsetPoint;
using motion_utils::calcLongitudinalOffsetPose;
using motion_utils::calcSignedArcLength;
using motion_utils::findNearestSegmentIndex;
using motion_utils::insertTargetPoint;
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::getPoint;
using tier4_autoware_utils::getPose;
using tier4_autoware_utils::pose2transform;

namespace
{
std::string toHexString(const unique_identifier_msgs::msg::UUID & id)
{
  std::stringstream ss;
  for (auto i = 0; i < 16; ++i) {
    ss << std::hex << std::setfill('0') << std::setw(2) << +id.uuid[i];
  }
  return ss.str();
}
geometry_msgs::msg::Point32 createPoint32(const double x, const double y, const double z)
{
  geometry_msgs::msg::Point32 p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}
geometry_msgs::msg::Polygon toMsg(const Polygon & polygon, const double z)
{
  geometry_msgs::msg::Polygon ret;
  for (const auto & p : polygon.outer()) {
    ret.points.push_back(createPoint32(p.x(), p.y(), z));
  }
  return ret;
}
Polygon createOneStepPolygon(
  const geometry_msgs::msg::Pose & p_front, const geometry_msgs::msg::Pose & p_back,
  const geometry_msgs::msg::Polygon & base_polygon)
{
  Polygon one_step_polygon{};

  {
    geometry_msgs::msg::Polygon out_polygon{};
    geometry_msgs::msg::TransformStamped geometry_tf{};
    geometry_tf.transform = pose2transform(p_front);
    tf2::doTransform(base_polygon, out_polygon, geometry_tf);

    for (const auto & p : out_polygon.points) {
      one_step_polygon.outer().push_back(Point(p.x, p.y));
    }
  }

  {
    geometry_msgs::msg::Polygon out_polygon{};
    geometry_msgs::msg::TransformStamped geometry_tf{};
    geometry_tf.transform = pose2transform(p_back);
    tf2::doTransform(base_polygon, out_polygon, geometry_tf);

    for (const auto & p : out_polygon.points) {
      one_step_polygon.outer().push_back(Point(p.x, p.y));
    }
  }

  Polygon hull_polygon{};
  bg::convex_hull(one_step_polygon, hull_polygon);
  bg::correct(hull_polygon);

  return hull_polygon;
}
void sortCrosswalksByDistance(
  const PathWithLaneId & ego_path, const geometry_msgs::msg::Point & ego_pos,
  lanelet::ConstLanelets & crosswalks)
{
  const auto compare = [&](const lanelet::ConstLanelet & l1, const lanelet::ConstLanelet & l2) {
    const std::vector<Point> l1_intersects =
      getPolygonIntersects(ego_path, l1.polygon2d().basicPolygon(), ego_pos, 2);
    const std::vector<Point> l2_intersects =
      getPolygonIntersects(ego_path, l2.polygon2d().basicPolygon(), ego_pos, 2);

    if (l1_intersects.empty() || l2_intersects.empty()) {
      return true;
    }

    const auto dist_l1 = calcSignedArcLength(
      ego_path.points, size_t(0),
      createPoint(l1_intersects.front().x(), l1_intersects.front().y(), ego_pos.z));

    const auto dist_l2 = calcSignedArcLength(
      ego_path.points, size_t(0),
      createPoint(l2_intersects.front().x(), l2_intersects.front().y(), ego_pos.z));

    return dist_l1 < dist_l2;
  };

  std::sort(crosswalks.begin(), crosswalks.end(), compare);
}
}  // namespace

CrosswalkModule::CrosswalkModule(
  const int64_t module_id, lanelet::ConstLanelet crosswalk, const PlannerParam & planner_param,
  const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  module_id_(module_id),
  crosswalk_(std::move(crosswalk)),
  planner_param_(planner_param)
{
  passed_safety_slow_point_ = false;
}

bool CrosswalkModule::modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason)
{
  stop_watch_.tic("total_processing_time");
  RCLCPP_INFO_EXPRESSION(
    logger_, planner_param_.show_processing_time,
    "=========== module_id: %ld ===========", module_id_);

  debug_data_ = DebugData();
  debug_data_.base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  *stop_reason = planning_utils::initializeStopReason(StopReason::CROSSWALK);

  auto ego_path = *path;

  RCLCPP_INFO_EXPRESSION(
    logger_, planner_param_.show_processing_time, "- step1: %f ms",
    stop_watch_.toc("total_processing_time", false));

  path_intersects_.clear();

  const auto & ego_pos = planner_data_->current_pose.pose.position;
  const auto intersects =
    getPolygonIntersects(ego_path, crosswalk_.polygon2d().basicPolygon(), ego_pos, 2);

  for (const auto & p : intersects) {
    path_intersects_.push_back(createPoint(p.x(), p.y(), ego_pos.z));
  }

  for (const auto & p : crosswalk_.polygon2d().basicPolygon()) {
    debug_data_.crosswalk_polygon.push_back(createPoint(p.x(), p.y(), ego_pos.z));
  }

  if (crosswalk_.hasAttribute("safety_slow_down_speed")) {
    // Safety slow down is on
    if (applySafetySlowDownSpeed(*path)) {
      ego_path = *path;
    }
  }

  RCLCPP_INFO_EXPRESSION(
    logger_, planner_param_.show_processing_time, "- step2: %f ms",
    stop_watch_.toc("total_processing_time", false));

  StopFactor stop_factor{};
  StopFactor stop_factor_rtc{};

  const auto nearest_stop_point = findNearestStopPoint(ego_path, stop_factor);
  const auto rtc_stop_point = findRTCStopPoint(ego_path, stop_factor_rtc);

  RCLCPP_INFO_EXPRESSION(
    logger_, planner_param_.show_processing_time, "- step3: %f ms",
    stop_watch_.toc("total_processing_time", false));

  setSafe(!nearest_stop_point);

  if (isActivated()) {
    if (!nearest_stop_point) {
      if (!rtc_stop_point) {
        setDistance(std::numeric_limits<double>::lowest());
        return true;
      }

      const auto crosswalk_distance =
        calcSignedArcLength(ego_path.points, ego_pos, getPoint(rtc_stop_point.get()));
      setDistance(crosswalk_distance);
      return true;
    }

    const auto target_velocity = calcTargetVelocity(nearest_stop_point.get(), ego_path);
    insertDecelPointWithDebugInfo(
      nearest_stop_point.get(), std::max(planner_param_.min_slow_down_velocity, target_velocity),
      *path);

    return true;
  }

  if (nearest_stop_point) {
    insertDecelPointWithDebugInfo(nearest_stop_point.get(), 0.0, *path);
    planning_utils::appendStopReason(stop_factor, stop_reason);
  } else if (rtc_stop_point) {
    insertDecelPointWithDebugInfo(rtc_stop_point.get(), 0.0, *path);
    planning_utils::appendStopReason(stop_factor_rtc, stop_reason);
  }

  RCLCPP_INFO_EXPRESSION(
    logger_, planner_param_.show_processing_time, "- step4: %f ms",
    stop_watch_.toc("total_processing_time", false));

  return true;
}

boost::optional<std::pair<double, geometry_msgs::msg::Point>> CrosswalkModule::getStopLine(
  const PathWithLaneId & ego_path, bool & exist_stopline_in_map) const
{
  const auto & ego_pos = planner_data_->current_pose.pose.position;

  const auto stop_line = getStopLineFromMap(module_id_, planner_data_, "crosswalk_id");
  exist_stopline_in_map = static_cast<bool>(stop_line);
  if (stop_line) {
    const auto intersects = getLinestringIntersects(
      ego_path, lanelet::utils::to2D(stop_line.get()).basicLineString(), ego_pos, 2);
    if (!intersects.empty()) {
      const auto p_stop_line =
        createPoint(intersects.front().x(), intersects.front().y(), ego_pos.z);
      const auto dist_ego_to_stop = calcSignedArcLength(ego_path.points, ego_pos, p_stop_line);
      return std::make_pair(dist_ego_to_stop, p_stop_line);
    }
  }

  {
    if (!path_intersects_.empty()) {
      const auto p_stop_line = path_intersects_.front();
      const auto dist_ego_to_stop = calcSignedArcLength(ego_path.points, ego_pos, p_stop_line) -
                                    planner_param_.stop_line_distance;
      return std::make_pair(dist_ego_to_stop, p_stop_line);
    }
  }

  return {};
}

boost::optional<geometry_msgs::msg::Point> CrosswalkModule::findRTCStopPoint(
  const PathWithLaneId & ego_path, StopFactor & stop_factor)
{
  const auto & base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;

  bool exist_stopline_in_map;
  const auto p_stop_line = getStopLine(ego_path, exist_stopline_in_map);
  if (!p_stop_line) {
    return {};
  }

  const auto & p_stop = p_stop_line.get().second;
  const auto stop_line_distance = exist_stopline_in_map ? 0.0 : planner_param_.stop_line_distance;
  const auto margin = stop_line_distance + base_link2front;
  const auto stop_pose = calcLongitudinalOffsetPose(ego_path.points, p_stop, -margin);

  if (!stop_pose) {
    return {};
  }

  stop_factor.stop_pose = stop_pose.get();

  return stop_pose.get().position;
}

boost::optional<geometry_msgs::msg::Point> CrosswalkModule::findNearestStopPoint(
  const PathWithLaneId & ego_path, StopFactor & stop_factor)
{
  bool found_pedestrians = false;
  bool found_stuck_vehicle = false;

  PathWithLaneId sparse_resample_path{};
  constexpr double RESAMPLE_INTERVAL = 4.0;
  if (!splineInterpolate(ego_path, RESAMPLE_INTERVAL, sparse_resample_path, logger_)) {
    return {};
  }

  const auto crosswalk_attention_range = getAttentionRange(sparse_resample_path);
  const auto & ego_pos = planner_data_->current_pose.pose.position;
  const auto & objects_ptr = planner_data_->predicted_objects;
  const auto & base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;

  bool exist_stopline_in_map;
  const auto p_stop_line = getStopLine(sparse_resample_path, exist_stopline_in_map);
  if (!p_stop_line) {
    return {};
  }

  geometry_msgs::msg::Point first_stop_point{};
  double minimum_stop_dist = std::numeric_limits<double>::max();

  const auto now = clock_->now();

  const auto ignore_crosswalk = debug_data_.ignore_crosswalk = isRedSignalForPedestrians();

  const auto ego_polygon = createVehiclePolygon(planner_data_->vehicle_info_);

  Polygon attention_area;
  for (size_t j = 0; j < sparse_resample_path.points.size() - 1; ++j) {
    const auto & p_ego_front = sparse_resample_path.points.at(j).point.pose;
    const auto & p_ego_back = sparse_resample_path.points.at(j + 1).point.pose;
    const auto front_length =
      calcSignedArcLength(sparse_resample_path.points, ego_pos, p_ego_front.position);
    const auto back_length =
      calcSignedArcLength(sparse_resample_path.points, ego_pos, p_ego_back.position);

    if (back_length < crosswalk_attention_range.first) {
      continue;
    }

    if (crosswalk_attention_range.second < front_length) {
      break;
    }

    const auto ego_one_step_polygon = createOneStepPolygon(p_ego_front, p_ego_back, ego_polygon);

    debug_data_.ego_polygons.push_back(toMsg(ego_one_step_polygon, ego_pos.z));

    std::vector<Polygon> unions;
    bg::union_(attention_area, ego_one_step_polygon, unions);
    if (!unions.empty()) {
      attention_area = unions.front();
      bg::correct(attention_area);
    }
  }

  for (const auto & object : objects_ptr->objects) {
    const auto & obj_pos = object.kinematics.initial_pose_with_covariance.pose.position;
    const auto & obj_vel = object.kinematics.initial_twist_with_covariance.twist.linear;
    const auto obj_uuid = toHexString(object.object_id);

    if (isVehicle(object)) {
      found_stuck_vehicle = found_stuck_vehicle || isStuckVehicle(sparse_resample_path, object);
    }

    if (!isTargetType(object)) {
      continue;
    }

    if (ignore_crosswalk) {
      continue;
    }

    for (auto & cp : getCollisionPoints(
           sparse_resample_path, object, attention_area, crosswalk_attention_range)) {
      const auto is_ignore_object = ignore_objects_.count(obj_uuid) != 0;
      if (is_ignore_object) {
        cp.state = CollisionPointState::IGNORE;
      }

      const auto is_stop_object =
        std::hypot(obj_vel.x, obj_vel.y) < planner_param_.stop_object_velocity;
      if (!is_stop_object) {
        ignore_objects_.erase(obj_uuid);
        stopped_objects_.erase(obj_uuid);
      }

      debug_data_.collision_points.push_back(cp);

      if (cp.state != CollisionPointState::YIELD) {
        continue;
      }

      found_pedestrians = true;
      stop_factor.stop_factor_points.push_back(obj_pos);

      const auto dist_ego2cp =
        calcSignedArcLength(sparse_resample_path.points, ego_pos, cp.collision_point) -
        planner_param_.stop_margin;

      if (dist_ego2cp < minimum_stop_dist) {
        first_stop_point = cp.collision_point;
        minimum_stop_dist = dist_ego2cp;
      }

      if (!is_stop_object) {
        continue;
      }

      const auto reached_stop_point =
        dist_ego2cp - base_link2front < planner_param_.stop_position_threshold ||
        p_stop_line.get().first - base_link2front < planner_param_.stop_position_threshold;

      const auto is_yielding_now =
        planner_data_->isVehicleStopped(planner_param_.ego_yield_query_stop_duration) &&
        reached_stop_point;
      if (!is_yielding_now) {
        continue;
      }

      const auto has_object_stopped = stopped_objects_.count(obj_uuid) != 0;
      if (!has_object_stopped) {
        stopped_objects_.insert(std::make_pair(obj_uuid, now));
        continue;
      }

      const auto stopped_time = (now - stopped_objects_.at(obj_uuid)).seconds();
      const auto no_intent_to_cross = stopped_time > planner_param_.max_yield_timeout;
      if (!no_intent_to_cross) {
        continue;
      }

      ignore_objects_.insert(std::make_pair(obj_uuid, now));
    }
  }

  const auto need_to_stop = found_pedestrians || found_stuck_vehicle;
  if (!need_to_stop) {
    return {};
  }

  if (!found_pedestrians) {
    minimum_stop_dist = p_stop_line.get().first;
    first_stop_point = p_stop_line.get().second;
  }

  const auto within_stop_line_margin =
    p_stop_line.get().first < minimum_stop_dist &&
    minimum_stop_dist < p_stop_line.get().first + planner_param_.stop_line_margin;

  const auto stop_at_stop_line = !found_pedestrians || within_stop_line_margin;

  const auto & p_stop = stop_at_stop_line ? p_stop_line.get().second : first_stop_point;
  const auto stop_line_distance = exist_stopline_in_map ? 0.0 : planner_param_.stop_line_distance;
  const auto margin = stop_at_stop_line ? stop_line_distance + base_link2front
                                        : planner_param_.stop_margin + base_link2front;
  const auto stop_pose = calcLongitudinalOffsetPose(sparse_resample_path.points, p_stop, -margin);

  if (!stop_pose) {
    return {};
  }

  stop_factor.stop_pose = stop_pose.get();

  return stop_pose.get().position;
}

std::pair<double, double> CrosswalkModule::getAttentionRange(const PathWithLaneId & ego_path)
{
  stop_watch_.tic(__func__);

  const auto & ego_pos = planner_data_->current_pose.pose.position;

  double near_attention_range = 0.0;
  double far_attention_range = 0.0;
  if (path_intersects_.size() < 2) {
    return std::make_pair(near_attention_range, far_attention_range);
  }

  near_attention_range = calcSignedArcLength(ego_path.points, ego_pos, path_intersects_.front());
  far_attention_range = calcSignedArcLength(ego_path.points, ego_pos, path_intersects_.back());

  near_attention_range -= planner_param_.crosswalk_attention_range;
  far_attention_range += planner_param_.crosswalk_attention_range;

  clampAttentionRangeByNeighborCrosswalks(ego_path, near_attention_range, far_attention_range);

  RCLCPP_INFO_EXPRESSION(
    logger_, planner_param_.show_processing_time, "%s : %f ms", __func__,
    stop_watch_.toc(__func__, true));

  return std::make_pair(std::max(0.0, near_attention_range), std::max(0.0, far_attention_range));
}

void CrosswalkModule::insertDecelPointWithDebugInfo(
  const geometry_msgs::msg::Point & stop_point, const float target_velocity,
  PathWithLaneId & output)
{
  const auto stop_pose = planning_utils::insertDecelPoint(stop_point, output, target_velocity);
  if (!stop_pose) {
    return;
  }
  const auto & ego_pos = planner_data_->current_pose.pose.position;

  setDistance(calcSignedArcLength(output.points, ego_pos, stop_pose->position));

  debug_data_.first_stop_pose = getPose(*stop_pose);

  if (std::abs(target_velocity) < 1e-3) {
    debug_data_.stop_poses.push_back(*stop_pose);
  } else {
    debug_data_.slow_poses.push_back(*stop_pose);
  }
}

float CrosswalkModule::calcTargetVelocity(
  const geometry_msgs::msg::Point & stop_point, const PathWithLaneId & ego_path) const
{
  const auto & max_jerk = planner_param_.max_slow_down_jerk;
  const auto & max_accel = planner_param_.max_slow_down_accel;
  const auto & ego_pos = planner_data_->current_pose.pose.position;
  const auto & ego_vel = planner_data_->current_velocity->twist.linear.x;

  if (ego_vel < planner_param_.no_relax_velocity) {
    return 0.0;
  }

  const auto ego_acc = planner_data_->current_acceleration->accel.accel.linear.x;
  const auto dist_deceleration = calcSignedArcLength(ego_path.points, ego_pos, stop_point);
  const auto feasible_velocity = planning_utils::calcDecelerationVelocityFromDistanceToTarget(
    max_jerk, max_accel, ego_acc, ego_vel, dist_deceleration);

  constexpr double margin_velocity = 0.5;  // 1.8 km/h
  return margin_velocity < feasible_velocity ? feasible_velocity : 0.0;
}

void CrosswalkModule::clampAttentionRangeByNeighborCrosswalks(
  const PathWithLaneId & ego_path, double & near_attention_range, double & far_attention_range)
{
  stop_watch_.tic(__func__);

  const auto & ego_pos = planner_data_->current_pose.pose.position;

  const auto p_near = calcLongitudinalOffsetPoint(ego_path.points, ego_pos, near_attention_range);
  const auto p_far = calcLongitudinalOffsetPoint(ego_path.points, ego_pos, far_attention_range);

  if (!p_near || !p_far) {
    return;
  }

  const auto near_idx = findNearestSegmentIndex(ego_path.points, p_near.get());
  const auto far_idx = findNearestSegmentIndex(ego_path.points, p_far.get()) + 1;

  std::set<int64_t> lane_ids;
  for (size_t i = near_idx; i < far_idx; ++i) {
    for (const auto & id : ego_path.points.at(i).lane_ids) {
      lane_ids.insert(id);
    }
  }

  lanelet::ConstLanelets crosswalks{};
  constexpr int PEDESTRIAN_GRAPH_ID = 1;
  const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto overall_graphs_ptr = planner_data_->route_handler_->getOverallGraphPtr();

  for (const auto & id : lane_ids) {
    const auto ll = lanelet_map_ptr->laneletLayer.get(id);
    const auto conflicting_crosswalks =
      overall_graphs_ptr->conflictingInGraph(ll, PEDESTRIAN_GRAPH_ID);
    for (const auto & crosswalk : conflicting_crosswalks) {
      const auto itr = std::find_if(
        crosswalks.begin(), crosswalks.end(),
        [&](const lanelet::ConstLanelet & ll) { return ll.id() == crosswalk.id(); });
      if (itr == crosswalks.end()) {
        crosswalks.push_back(crosswalk);
      }
    }
  }

  sortCrosswalksByDistance(ego_path, ego_pos, crosswalks);

  boost::optional<lanelet::ConstLanelet> prev_crosswalk{boost::none};
  boost::optional<lanelet::ConstLanelet> next_crosswalk{boost::none};

  if (!crosswalks.empty()) {
    for (size_t i = 0; i < crosswalks.size() - 1; ++i) {
      const auto ll_front = crosswalks.at(i);
      const auto ll_back = crosswalks.at(i + 1);

      if (ll_front.id() == module_id_ && ll_back.id() != module_id_) {
        next_crosswalk = ll_back;
      }

      if (ll_front.id() != module_id_ && ll_back.id() == module_id_) {
        prev_crosswalk = ll_front;
      }
    }
  }

  if (prev_crosswalk) {
    auto reverse_ego_path = ego_path;
    std::reverse(reverse_ego_path.points.begin(), reverse_ego_path.points.end());

    const std::vector<Point> prev_crosswalk_intersects = getPolygonIntersects(
      reverse_ego_path, prev_crosswalk.get().polygon2d().basicPolygon(), ego_pos, 2);

    if (!prev_crosswalk_intersects.empty()) {
      const auto dist_to_prev_crosswalk = calcSignedArcLength(
        ego_path.points, ego_pos,
        createPoint(
          prev_crosswalk_intersects.front().x(), prev_crosswalk_intersects.front().y(), ego_pos.z));

      near_attention_range = std::max(near_attention_range, dist_to_prev_crosswalk);
    }
  }

  if (next_crosswalk) {
    const std::vector<Point> next_crosswalk_intersects =
      getPolygonIntersects(ego_path, next_crosswalk.get().polygon2d().basicPolygon(), ego_pos, 2);

    if (!next_crosswalk_intersects.empty()) {
      const auto dist_to_next_crosswalk = calcSignedArcLength(
        ego_path.points, ego_pos,
        createPoint(
          next_crosswalk_intersects.front().x(), next_crosswalk_intersects.front().y(), ego_pos.z));

      far_attention_range = std::min(far_attention_range, dist_to_next_crosswalk);
    }
  }

  const auto update_p_near =
    calcLongitudinalOffsetPoint(ego_path.points, ego_pos, near_attention_range);
  const auto update_p_far =
    calcLongitudinalOffsetPoint(ego_path.points, ego_pos, far_attention_range);

  if (update_p_near && update_p_far) {
    debug_data_.range_near_point = update_p_near.get();
    debug_data_.range_far_point = update_p_far.get();
  }

  RCLCPP_INFO_EXPRESSION(
    logger_, planner_param_.show_processing_time, "%s : %f ms", __func__,
    stop_watch_.toc(__func__, true));
}

std::vector<CollisionPoint> CrosswalkModule::getCollisionPoints(
  const PathWithLaneId & ego_path, const PredictedObject & object, const Polygon & attention_area,
  const std::pair<double, double> & crosswalk_attention_range)
{
  stop_watch_.tic(__func__);

  std::vector<CollisionPoint> ret{};

  const auto & obj_vel = object.kinematics.initial_twist_with_covariance.twist.linear;

  const auto & ego_pos = planner_data_->current_pose.pose.position;
  const auto & ego_vel = planner_data_->current_velocity->twist.linear;

  const auto obj_polygon =
    createObjectPolygon(object.shape.dimensions.x, object.shape.dimensions.y);

  for (const auto & obj_path : object.kinematics.predicted_paths) {
    for (size_t i = 0; i < obj_path.path.size() - 1; ++i) {
      const auto p_obj_front = obj_path.path.at(i);
      const auto p_obj_back = obj_path.path.at(i + 1);
      const auto obj_one_step_polygon = createOneStepPolygon(p_obj_front, p_obj_back, obj_polygon);

      std::vector<Point> tmp_intersects{};
      bg::intersection(obj_one_step_polygon, attention_area, tmp_intersects);

      if (bg::within(obj_one_step_polygon, attention_area)) {
        for (const auto & p : obj_one_step_polygon.outer()) {
          const Point point{p.x(), p.y()};
          tmp_intersects.push_back(point);
        }
      }

      if (tmp_intersects.empty()) {
        continue;
      }

      double minimum_stop_dist = std::numeric_limits<double>::max();
      geometry_msgs::msg::Point nearest_collision_point{};
      for (const auto & p : tmp_intersects) {
        geometry_msgs::msg::Point cp = createPoint(p.x(), p.y(), ego_pos.z);
        const auto dist_ego2cp = calcSignedArcLength(ego_path.points, ego_pos, cp);

        if (dist_ego2cp < minimum_stop_dist) {
          minimum_stop_dist = dist_ego2cp;
          nearest_collision_point = cp;
        }
      }

      const auto dist_ego2cp =
        calcSignedArcLength(ego_path.points, ego_pos, nearest_collision_point);
      constexpr double eps = 1e-3;
      const auto dist_obj2cp =
        calcArcLength(obj_path.path) < eps
          ? 0.0
          : calcSignedArcLength(obj_path.path, size_t(0), nearest_collision_point);

      if (
        dist_ego2cp < crosswalk_attention_range.first ||
        crosswalk_attention_range.second < dist_ego2cp) {
        continue;
      }

      ret.push_back(
        createCollisionPoint(nearest_collision_point, dist_ego2cp, dist_obj2cp, ego_vel, obj_vel));

      debug_data_.obj_polygons.push_back(toMsg(obj_one_step_polygon, ego_pos.z));

      break;
    }
  }

  RCLCPP_INFO_EXPRESSION(
    logger_, planner_param_.show_processing_time, "%s : %f ms", __func__,
    stop_watch_.toc(__func__, true));

  return ret;
}

CollisionPoint CrosswalkModule::createCollisionPoint(
  const geometry_msgs::msg::Point & nearest_collision_point, const double dist_ego2cp,
  const double dist_obj2cp, const geometry_msgs::msg::Vector3 & ego_vel,
  const geometry_msgs::msg::Vector3 & obj_vel) const
{
  constexpr double min_ego_velocity = 1.38;  // [m/s]

  const auto & base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;

  const auto estimated_velocity = std::hypot(obj_vel.x, obj_vel.y);
  const auto velocity = std::max(planner_param_.min_object_velocity, estimated_velocity);

  CollisionPoint collision_point{};
  collision_point.collision_point = nearest_collision_point;
  collision_point.time_to_collision =
    std::max(0.0, dist_ego2cp - planner_param_.stop_margin - base_link2front) /
    std::max(ego_vel.x, min_ego_velocity);
  collision_point.time_to_vehicle = std::max(0.0, dist_obj2cp) / velocity;
  collision_point.state =
    getCollisionPointState(collision_point.time_to_collision, collision_point.time_to_vehicle);

  return collision_point;
}

CollisionPointState CrosswalkModule::getCollisionPointState(
  const double ttc, const double ttv) const
{
  if (ttc + planner_param_.ego_pass_first_margin < ttv) {
    return CollisionPointState::EGO_PASS_FIRST;
  }

  if (ttv + planner_param_.ego_pass_later_margin < ttc) {
    return CollisionPointState::EGO_PASS_LATER;
  }

  return CollisionPointState::YIELD;
}

bool CrosswalkModule::applySafetySlowDownSpeed(PathWithLaneId & output)
{
  if (path_intersects_.empty()) {
    return false;
  }

  const auto & ego_pos = planner_data_->current_pose.pose.position;
  const auto ego_path = output;

  // Safety slow down speed in [m/s]
  const auto & safety_slow_down_speed =
    static_cast<float>(crosswalk_.attribute("safety_slow_down_speed").asDouble().get());

  if (!passed_safety_slow_point_) {
    // Safety slow down distance [m]
    double safety_slow_down_distance = crosswalk_.attributeOr("safety_slow_down_distance", 2.0);

    // the range until to the point where ego will have a const safety slow down speed
    auto safety_slow_point_range =
      calcSignedArcLength(ego_path.points, ego_pos, path_intersects_.front());
    const auto & safety_slow_margin =
      planner_data_->vehicle_info_.max_longitudinal_offset_m + safety_slow_down_distance;
    safety_slow_point_range -= safety_slow_margin;

    const auto & p_safety_slow =
      calcLongitudinalOffsetPoint(ego_path.points, ego_pos, safety_slow_point_range);

    insertDecelPointWithDebugInfo(p_safety_slow.get(), safety_slow_down_speed, output);

    if (safety_slow_point_range < 0.0) {
      passed_safety_slow_point_ = true;
    }
  } else {
    // the range until to the point where ego will start accelerate
    auto safety_slow_end_point_range =
      calcSignedArcLength(ego_path.points, ego_pos, path_intersects_.back());

    if (safety_slow_end_point_range > 0) {
      // insert constant ego speed until the end of the crosswalk
      for (auto & p : output.points) {
        const auto & original_velocity = p.point.longitudinal_velocity_mps;
        p.point.longitudinal_velocity_mps = std::min(original_velocity, safety_slow_down_speed);
      }
    }
  }
  return true;
}

bool CrosswalkModule::isStuckVehicle(
  const PathWithLaneId & ego_path, const PredictedObject & object) const
{
  const auto & obj_vel = object.kinematics.initial_twist_with_covariance.twist.linear;
  if (planner_param_.stuck_vehicle_velocity < std::hypot(obj_vel.x, obj_vel.y)) {
    return false;
  }

  const auto & obj_pos = object.kinematics.initial_pose_with_covariance.pose.position;
  const auto lateral_offset = calcLateralOffset(ego_path.points, obj_pos);
  if (planner_param_.max_lateral_offset < std::abs(lateral_offset)) {
    return false;
  }

  const auto & ego_pos = planner_data_->current_pose.pose.position;

  double near_attention_range = 0.0;
  double far_attention_range = 0.0;
  if (path_intersects_.size() < 2) {
    return false;
  }

  near_attention_range = calcSignedArcLength(ego_path.points, ego_pos, path_intersects_.front());
  far_attention_range = calcSignedArcLength(ego_path.points, ego_pos, path_intersects_.back());
  near_attention_range = far_attention_range;
  far_attention_range += planner_param_.stuck_vehicle_attention_range;

  const auto dist_ego2obj = calcSignedArcLength(ego_path.points, ego_pos, obj_pos);

  return near_attention_range < dist_ego2obj && dist_ego2obj < far_attention_range;
}

bool CrosswalkModule::isRedSignalForPedestrians() const
{
  const auto traffic_lights_reg_elems =
    crosswalk_.regulatoryElementsAs<const lanelet::TrafficLight>();

  for (const auto & traffic_lights_reg_elem : traffic_lights_reg_elems) {
    lanelet::ConstLineStringsOrPolygons3d traffic_lights = traffic_lights_reg_elem->trafficLights();
    for (const auto & traffic_light : traffic_lights) {
      const auto ll_traffic_light = static_cast<lanelet::ConstLineString3d>(traffic_light);
      const auto traffic_signal_stamped = planner_data_->getTrafficSignal(ll_traffic_light.id());
      if (!traffic_signal_stamped) {
        continue;
      }

      if (
        planner_param_.tl_state_timeout <
        (clock_->now() - traffic_signal_stamped->header.stamp).seconds()) {
        continue;
      }

      const auto & lights = traffic_signal_stamped->signal.lights;
      if (lights.empty()) {
        continue;
      }

      if (lights.front().color == TrafficLight::RED) {
        return true;
      }
    }
  }

  return false;
}

bool CrosswalkModule::isVehicle(const PredictedObject & object)
{
  if (object.classification.empty()) {
    return false;
  }

  const auto & label = object.classification.front().label;

  if (label == ObjectClassification::CAR) {
    return true;
  }

  if (label == ObjectClassification::TRUCK) {
    return true;
  }

  if (label == ObjectClassification::BUS) {
    return true;
  }

  if (label == ObjectClassification::TRAILER) {
    return true;
  }

  if (label == ObjectClassification::MOTORCYCLE) {
    return true;
  }

  return false;
}

bool CrosswalkModule::isTargetType(const PredictedObject & object) const
{
  if (object.classification.empty()) {
    return false;
  }

  const auto & label = object.classification.front().label;

  if (label == ObjectClassification::UNKNOWN && planner_param_.look_unknown) {
    return true;
  }

  if (label == ObjectClassification::BICYCLE && planner_param_.look_bicycle) {
    return true;
  }

  if (label == ObjectClassification::MOTORCYCLE && planner_param_.look_motorcycle) {
    return true;
  }

  if (label == ObjectClassification::PEDESTRIAN && planner_param_.look_pedestrian) {
    return true;
  }

  return false;
}

bool CrosswalkModule::isTargetExternalInputStatus(const int target_status) const
{
  const auto & ex_input = planner_data_->external_crosswalk_status_input;
  if (!ex_input) {
    return false;
  }

  const auto time_delta = (clock_->now() - ex_input.get().header.stamp).seconds();
  if (planner_param_.external_input_timeout < time_delta) {
    return false;
  }

  return ex_input.get().status == target_status;
}

geometry_msgs::msg::Polygon CrosswalkModule::createObjectPolygon(
  const double width_m, const double length_m)
{
  geometry_msgs::msg::Polygon polygon{};

  polygon.points.push_back(createPoint32(length_m / 2.0, -width_m / 2.0, 0.0));
  polygon.points.push_back(createPoint32(length_m / 2.0, width_m / 2.0, 0.0));
  polygon.points.push_back(createPoint32(-length_m / 2.0, width_m / 2.0, 0.0));
  polygon.points.push_back(createPoint32(-length_m / 2.0, -width_m / 2.0, 0.0));

  return polygon;
}

geometry_msgs::msg::Polygon CrosswalkModule::createVehiclePolygon(
  const vehicle_info_util::VehicleInfo & vehicle_info)
{
  const auto & i = vehicle_info;
  const auto & front_m = i.max_longitudinal_offset_m;
  const auto & width_m = i.vehicle_width_m / 2.0;
  const auto & back_m = i.rear_overhang_m;

  geometry_msgs::msg::Polygon polygon{};

  polygon.points.push_back(createPoint32(front_m, -width_m, 0.0));
  polygon.points.push_back(createPoint32(front_m, width_m, 0.0));
  polygon.points.push_back(createPoint32(-back_m, width_m, 0.0));
  polygon.points.push_back(createPoint32(-back_m, -width_m, 0.0));

  return polygon;
}
}  // namespace behavior_velocity_planner
