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
#include <utilization/util.hpp>

#include <cmath>
#include <vector>

namespace behavior_velocity_planner
{

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;
using Line = bg::model::linestring<Point>;
using tier4_autoware_utils::calcArcLength;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcLateralOffset;
using tier4_autoware_utils::calcLongitudinalOffsetToSegment;
using tier4_autoware_utils::calcSignedArcLength;
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::findNearestIndex;
using tier4_autoware_utils::findNearestSegmentIndex;
using tier4_autoware_utils::getPoint;
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
std::vector<Point> getPolygonIntersects(
  const PathWithLaneId & ego_path, const lanelet::BasicPolygon2d & polygon,
  const geometry_msgs::msg::Point & ego_pos,
  const size_t max_num = std::numeric_limits<size_t>::max())
{
  std::vector<Point> intersects{};

  bool found_max_num = false;
  for (size_t i = 0; i < ego_path.points.size() - 1; ++i) {
    const auto & p_back = ego_path.points.at(i).point.pose.position;
    const auto & p_front = ego_path.points.at(i + 1).point.pose.position;
    const Line segment{{p_back.x, p_back.y}, {p_front.x, p_front.y}};

    std::vector<Point> tmp_intersects{};
    bg::intersection(segment, polygon, tmp_intersects);

    for (const auto & p : tmp_intersects) {
      intersects.push_back(p);
      if (intersects.size() == max_num) {
        found_max_num = true;
        break;
      }
    }

    if (found_max_num) {
      break;
    }
  }

  const auto compare = [&](const Point & p1, const Point & p2) {
    const auto dist_l1 =
      calcSignedArcLength(ego_path.points, size_t(0), createPoint(p1.x(), p1.y(), ego_pos.z));

    const auto dist_l2 =
      calcSignedArcLength(ego_path.points, size_t(0), createPoint(p2.x(), p2.y(), ego_pos.z));

    return dist_l1 < dist_l2;
  };

  std::sort(intersects.begin(), intersects.end(), compare);

  return intersects;
}
std::vector<Point> getLinestringIntersects(
  const PathWithLaneId & ego_path, const lanelet::BasicLineString2d & linestring,
  const geometry_msgs::msg::Point & ego_pos,
  const size_t max_num = std::numeric_limits<size_t>::max())
{
  std::vector<Point> intersects{};

  bool found_max_num = false;
  for (size_t i = 0; i < ego_path.points.size() - 1; ++i) {
    const auto & p_back = ego_path.points.at(i).point.pose.position;
    const auto & p_front = ego_path.points.at(i + 1).point.pose.position;
    const Line segment{{p_back.x, p_back.y}, {p_front.x, p_front.y}};

    std::vector<Point> tmp_intersects{};
    bg::intersection(segment, linestring, tmp_intersects);

    for (const auto & p : tmp_intersects) {
      intersects.push_back(p);
      if (intersects.size() == max_num) {
        found_max_num = true;
        break;
      }
    }

    if (found_max_num) {
      break;
    }
  }

  const auto compare = [&](const Point & p1, const Point & p2) {
    const auto dist_l1 =
      calcSignedArcLength(ego_path.points, size_t(0), createPoint(p1.x(), p1.y(), ego_pos.z));

    const auto dist_l2 =
      calcSignedArcLength(ego_path.points, size_t(0), createPoint(p2.x(), p2.y(), ego_pos.z));

    return dist_l1 < dist_l2;
  };

  std::sort(intersects.begin(), intersects.end(), compare);

  return intersects;
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
bool checkValidIndex(
  const geometry_msgs::msg::Pose & p_base, const geometry_msgs::msg::Pose & p_next,
  const geometry_msgs::msg::Pose & p_target)
{
  const Eigen::Vector2d base2target(
    p_target.position.x - p_base.position.x, p_target.position.y - p_base.position.y);
  const Eigen::Vector2d target2next(
    p_next.position.x - p_target.position.x, p_next.position.y - p_target.position.y);
  return base2target.dot(target2next) > 0.0;
}
PathPointWithLaneId getBackwardPointFromBasePoint(
  const PathPointWithLaneId & p_from, const PathPointWithLaneId & p_to,
  const PathPointWithLaneId & p_base, const double backward_length)
{
  PathPointWithLaneId output;
  const double dx = p_to.point.pose.position.x - p_from.point.pose.position.x;
  const double dy = p_to.point.pose.position.y - p_from.point.pose.position.y;
  const double norm = std::hypot(dx, dy);

  output = p_base;
  output.point.pose.position.x += backward_length * dx / norm;
  output.point.pose.position.y += backward_length * dy / norm;

  return output;
}
boost::optional<std::pair<size_t, PathPointWithLaneId>> getForwardInsertPointFromBasePoint(
  const size_t base_idx, const PathWithLaneId & ego_path, const double margin)
{
  if (base_idx + 1 > ego_path.points.size()) {
    return {};
  }

  if (margin < std::numeric_limits<double>::epsilon()) {
    return std::make_pair(base_idx, ego_path.points.at(base_idx));
  }

  double length_sum = 0.0;
  double length_residual = 0.0;

  for (size_t i = base_idx; i < ego_path.points.size() - 1; ++i) {
    const auto & p_front = ego_path.points.at(i);
    const auto & p_back = ego_path.points.at(i + 1);

    length_sum += calcDistance2d(p_front, p_back);
    length_residual = length_sum - margin;

    if (length_residual >= std::numeric_limits<double>::epsilon()) {
      const auto p_insert = getBackwardPointFromBasePoint(p_back, p_front, p_back, length_residual);

      // p_front(ego_path.points.at(i)) is insert base point
      return std::make_pair(i, p_insert);
    }
  }

  if (length_residual < std::numeric_limits<double>::epsilon()) {
    return std::make_pair(ego_path.points.size() - 1, ego_path.points.back());
  }

  return {};
}
boost::optional<std::pair<size_t, PathPointWithLaneId>> getBackwardInsertPointFromBasePoint(
  const size_t base_idx, const PathWithLaneId & ego_path, const double margin)
{
  if (base_idx + 1 > ego_path.points.size()) {
    return {};
  }

  if (margin < std::numeric_limits<double>::epsilon()) {
    return std::make_pair(base_idx, ego_path.points.at(base_idx));
  }

  double length_sum = 0.0;
  double length_residual = 0.0;

  for (size_t i = base_idx; 0 < i; --i) {
    const auto & p_front = ego_path.points.at(i - 1);
    const auto & p_back = ego_path.points.at(i);

    length_sum += calcDistance2d(p_front, p_back);
    length_residual = length_sum - margin;

    if (length_residual >= std::numeric_limits<double>::epsilon()) {
      const auto p_insert =
        getBackwardPointFromBasePoint(p_front, p_back, p_front, length_residual);

      // p_front(ego_path.points.points.at(i-1)) is insert base point
      return std::make_pair(i - 1, p_insert);
    }
  }

  if (length_residual < std::numeric_limits<double>::epsilon()) {
    return std::make_pair(size_t(0), ego_path.points.front());
  }

  return {};
}
}  // namespace

CrosswalkModule::CrosswalkModule(
  const int64_t module_id, const lanelet::ConstLanelet & crosswalk,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock), module_id_(module_id), crosswalk_(crosswalk)
{
  planner_param_ = planner_param;
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

  const auto ego_path = *path;

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

  RCLCPP_INFO_EXPRESSION(
    logger_, planner_param_.show_processing_time, "- step2: %f ms",
    stop_watch_.toc("total_processing_time", false));

  const auto nearest_stop_point = findNearestStopPoint(ego_path, *stop_reason);

  RCLCPP_INFO_EXPRESSION(
    logger_, planner_param_.show_processing_time, "- step3: %f ms",
    stop_watch_.toc("total_processing_time", false));

  setSafe(!nearest_stop_point);

  if (!nearest_stop_point) {
    return true;
  }

  if (!isActivated()) {
    insertDecelPoint(nearest_stop_point.get(), *path);
  }

  RCLCPP_INFO_EXPRESSION(
    logger_, planner_param_.show_processing_time, "- step4: %f ms",
    stop_watch_.toc("total_processing_time", false));

  return true;
}

boost::optional<std::pair<double, geometry_msgs::msg::Point>> CrosswalkModule::getStopLine(
  const PathWithLaneId & ego_path) const
{
  const auto & ego_pos = planner_data_->current_pose.pose.position;

  const auto stop_line = getStopLineFromMap(module_id_, planner_data_, "crosswalk_id");
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

boost::optional<std::pair<size_t, PathPointWithLaneId>> CrosswalkModule::findNearestStopPoint(
  const PathWithLaneId & ego_path, StopReason & stop_reason)
{
  StopFactor stop_factor{};

  bool found_pedestrians = false;
  bool found_stuck_vehicle = false;
  const bool external_slowdown = isTargetExternalInputStatus(CrosswalkStatus::SLOWDOWN);
  const bool external_stop = isTargetExternalInputStatus(CrosswalkStatus::STOP);
  const bool external_go = isTargetExternalInputStatus(CrosswalkStatus::GO);

  if (external_go) {
    return {};
  }

  const auto crosswalk_attention_range = getAttentionRange(ego_path);
  const auto & ego_pos = planner_data_->current_pose.pose.position;
  const auto & objects_ptr = planner_data_->predicted_objects;
  const auto & base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;

  const auto p_stop_line = getStopLine(ego_path);
  if (!p_stop_line) {
    return {};
  }

  geometry_msgs::msg::Point first_stop_point{};
  double minimum_stop_dist = std::numeric_limits<double>::max();

  const auto now = clock_->now();

  const auto ignore_crosswalk = debug_data_.ignore_crosswalk = isRedSignalForPedestrians();

  for (const auto & object : objects_ptr->objects) {
    const auto & obj_pos = object.kinematics.initial_pose_with_covariance.pose.position;
    const auto & obj_vel = object.kinematics.initial_twist_with_covariance.twist.linear;
    const auto obj_uuid = toHexString(object.object_id);

    if (!isTargetType(object)) {
      found_stuck_vehicle = found_stuck_vehicle || isStuckVehicle(ego_path, object);
      continue;
    }

    if (ignore_crosswalk) {
      continue;
    }

    for (auto & cp : getCollisionPoints(ego_path, object, crosswalk_attention_range)) {
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

      const auto dist_ego2cp = calcSignedArcLength(ego_path.points, ego_pos, cp.collision_point) -
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

      const auto is_yielding_now = planner_data_->isVehicleStopped(0.1) && reached_stop_point;
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

  const auto need_to_stop =
    found_pedestrians || found_stuck_vehicle || external_stop || external_slowdown;
  if (!need_to_stop) {
    setDistance(
      std::abs(p_stop_line.get().first - planner_param_.stop_line_distance - base_link2front));
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
  const auto & margin =
    stop_at_stop_line ? planner_param_.stop_line_distance : planner_param_.stop_margin;

  const size_t base_idx = findNearestSegmentIndex(ego_path.points, p_stop);
  const auto residual_length = calcLongitudinalOffsetToSegment(ego_path.points, base_idx, p_stop);
  const auto update_margin = margin - residual_length + base_link2front;

  const auto stop_point = getBackwardInsertPointFromBasePoint(base_idx, ego_path, update_margin);
  stop_factor.stop_pose = stop_point.get().second.point.pose;
  planning_utils::appendStopReason(stop_factor, &stop_reason);

  setDistance(
    std::abs(calcSignedArcLength(ego_path.points, ego_pos, p_stop) - margin - base_link2front));

  return stop_point;
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

void CrosswalkModule::insertDecelPoint(
  const std::pair<size_t, PathPointWithLaneId> & stop_point, PathWithLaneId & output)
{
  const auto traj_end_idx = output.points.size() - 1;
  const auto & stop_idx = stop_point.first;

  const auto & p_base = output.points.at(stop_idx);
  const auto & p_next = output.points.at(std::min(stop_idx + 1, traj_end_idx));
  const auto & p_insert = stop_point.second;

  constexpr double min_dist = 1e-3;

  const auto is_p_base_and_p_insert_overlap = calcDistance2d(p_base, p_insert) < min_dist;
  const auto is_p_next_and_p_insert_overlap = calcDistance2d(p_next, p_insert) < min_dist;
  const auto is_valid_index =
    checkValidIndex(p_base.point.pose, p_next.point.pose, p_insert.point.pose);

  auto update_stop_idx = stop_idx;

  if (!is_p_base_and_p_insert_overlap && !is_p_next_and_p_insert_overlap && is_valid_index) {
    // insert: start_idx and end_idx are shifted by one
    output.points.insert(output.points.begin() + stop_idx + 1, p_insert);
    update_stop_idx = std::min(update_stop_idx + 1, traj_end_idx);
  } else if (is_p_next_and_p_insert_overlap) {
    // not insert: p_insert is merged into p_next
    update_stop_idx = std::min(update_stop_idx + 1, traj_end_idx);
  }

  const auto target_velocity = calcTargetVelocity(p_insert, output);
  for (size_t i = update_stop_idx; i < output.points.size(); ++i) {
    const auto & original_velocity = output.points.at(i).point.longitudinal_velocity_mps;
    output.points.at(i).point.longitudinal_velocity_mps =
      std::min(original_velocity, target_velocity);
  }

  debug_data_.first_stop_pose = stop_point.second.point.pose;
  debug_data_.stop_poses.push_back(stop_point.second.point.pose);
}

float CrosswalkModule::calcTargetVelocity(
  const PathPointWithLaneId & stop_point, const PathWithLaneId & ego_path) const
{
  if (isTargetExternalInputStatus(CrosswalkStatus::SLOWDOWN)) {
    return planner_param_.slow_velocity;
  }

  if (isTargetExternalInputStatus(CrosswalkStatus::STOP)) {
    return 0.0;
  }

  const auto & max_jerk = planner_param_.max_slow_down_jerk;
  const auto & max_accel = planner_param_.max_slow_down_accel;
  const auto & ego_pos = planner_data_->current_pose.pose.position;
  const auto & ego_vel = planner_data_->current_velocity->twist.linear.x;

  if (ego_vel < planner_param_.no_relax_velocity) {
    return 0.0;
  }

  const auto ego_acc = planner_data_->current_accel.get();
  const auto dist_deceleration =
    calcSignedArcLength(ego_path.points, ego_pos, stop_point.point.pose.position);
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
  const auto base_idx = findNearestSegmentIndex(ego_path.points, ego_pos);
  const auto residual_length = calcLongitudinalOffsetToSegment(ego_path.points, base_idx, ego_pos);

  const auto p_near =
    getForwardInsertPointFromBasePoint(base_idx, ego_path, near_attention_range + residual_length);
  const auto p_far =
    getForwardInsertPointFromBasePoint(base_idx, ego_path, far_attention_range + residual_length);

  if (!p_near || !p_far) {
    return;
  }

  const auto near_idx =
    findNearestSegmentIndex(ego_path.points, getPoint(p_near.get().second.point));
  const auto far_idx =
    findNearestSegmentIndex(ego_path.points, getPoint(p_far.get().second.point)) + 1;

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
    getForwardInsertPointFromBasePoint(base_idx, ego_path, near_attention_range + residual_length);
  const auto update_p_far =
    getForwardInsertPointFromBasePoint(base_idx, ego_path, far_attention_range + residual_length);

  if (update_p_near && update_p_far) {
    debug_data_.range_near_point = getPoint(update_p_near.get().second.point);
    debug_data_.range_far_point = getPoint(update_p_far.get().second.point);
  }

  RCLCPP_INFO_EXPRESSION(
    logger_, planner_param_.show_processing_time, "%s : %f ms", __func__,
    stop_watch_.toc(__func__, true));
}

std::vector<CollisionPoint> CrosswalkModule::getCollisionPoints(
  const PathWithLaneId & ego_path, const PredictedObject & object,
  const std::pair<double, double> & crosswalk_attention_range)
{
  stop_watch_.tic(__func__);

  std::vector<CollisionPoint> ret{};

  const auto & obj_vel = object.kinematics.initial_twist_with_covariance.twist.linear;

  const auto & ego_pos = planner_data_->current_pose.pose.position;
  const auto & ego_vel = planner_data_->current_velocity->twist.linear;

  const auto ego_polygon = createVehiclePolygon(planner_data_->vehicle_info_);
  const auto obj_polygon =
    createObjectPolygon(object.shape.dimensions.x, object.shape.dimensions.y);

  Polygon attention_area;
  for (size_t j = 0; j < ego_path.points.size() - 1; ++j) {
    const auto & p_ego_front = ego_path.points.at(j).point.pose;
    const auto & p_ego_back = ego_path.points.at(j + 1).point.pose;
    const auto front_length = calcSignedArcLength(ego_path.points, ego_pos, p_ego_front.position);
    const auto back_length = calcSignedArcLength(ego_path.points, ego_pos, p_ego_back.position);

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
  } else if (ttv + planner_param_.ego_pass_later_margin < ttc) {
    return CollisionPointState::EGO_PASS_LATER;
  } else {
    return CollisionPointState::YIELD;
  }

  return CollisionPointState::IGNORE;
}

bool CrosswalkModule::isStuckVehicle(
  const PathWithLaneId & ego_path, const PredictedObject & object) const
{
  const auto & label = object.classification.at(0).label;
  const auto is_target =
    label == ObjectClassification::CAR || label == ObjectClassification::TRUCK ||
    label == ObjectClassification::BUS || label == ObjectClassification::TRAILER ||
    label == ObjectClassification::MOTORCYCLE;
  if (!is_target) {
    return false;
  }

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
  const double width_m, const double length_m) const
{
  geometry_msgs::msg::Polygon polygon{};

  polygon.points.push_back(createPoint32(length_m / 2.0, -width_m / 2.0, 0.0));
  polygon.points.push_back(createPoint32(length_m / 2.0, width_m / 2.0, 0.0));
  polygon.points.push_back(createPoint32(-length_m / 2.0, width_m / 2.0, 0.0));
  polygon.points.push_back(createPoint32(-length_m / 2.0, -width_m / 2.0, 0.0));

  return polygon;
}

geometry_msgs::msg::Polygon CrosswalkModule::createVehiclePolygon(
  const vehicle_info_util::VehicleInfo & vehicle_info) const
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
