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

#include "scene.hpp"

#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <motion_utils/trajectory/trajectory.hpp>

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/intersection.hpp>

#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

namespace
{
bool getBackwardPointFromBasePoint(
  const Eigen::Vector2d & line_point1, const Eigen::Vector2d & line_point2,
  const Eigen::Vector2d & base_point, const double backward_length, Eigen::Vector2d & output_point)
{
  Eigen::Vector2d line_vec = line_point2 - line_point1;
  Eigen::Vector2d backward_vec = backward_length * line_vec.normalized();
  output_point = base_point + backward_vec;
  return true;
}

std::optional<Point2d> findNearestCollisionPoint(
  const LineString2d & line1, const LineString2d & line2, const Point2d & origin)
{
  std::vector<Point2d> collision_points;
  bg::intersection(line1, line2, collision_points);

  if (collision_points.empty()) {
    return std::nullopt;
  }

  // check nearest collision point
  Point2d nearest_collision_point;
  double min_dist = 0.0;

  for (size_t i = 0; i < collision_points.size(); ++i) {
    double dist = bg::distance(origin, collision_points.at(i));
    if (i == 0 || dist < min_dist) {
      min_dist = dist;
      nearest_collision_point = collision_points.at(i);
    }
  }
  return nearest_collision_point;
}

bool createTargetPoint(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input, const LineString2d & stop_line,
  const double offset, size_t & target_point_idx, Eigen::Vector2d & target_point)
{
  if (input.points.size() < 2) {
    return false;
  }
  for (size_t i = 0; i < input.points.size() - 1; ++i) {
    Point2d path_line_begin = {
      input.points.at(i).point.pose.position.x, input.points.at(i).point.pose.position.y};
    Point2d path_line_end = {
      input.points.at(i + 1).point.pose.position.x, input.points.at(i + 1).point.pose.position.y};
    LineString2d path_line = {path_line_begin, path_line_end};

    // check nearest collision point
    const auto nearest_collision_point =
      findNearestCollisionPoint(path_line, stop_line, path_line_begin);
    if (!nearest_collision_point) {
      continue;
    }

    // search target point index
    target_point_idx = 0;
    double length_sum = 0;

    Eigen::Vector2d point1, point2;
    if (0 <= offset) {
      point1 << nearest_collision_point->x(), nearest_collision_point->y();
      point2 << path_line_begin.x(), path_line_begin.y();
      length_sum += (point2 - point1).norm();
      for (size_t j = i; 0 < j; --j) {
        if (offset < length_sum) {
          target_point_idx = j + 1;
          break;
        }
        point1 << input.points.at(j).point.pose.position.x,
          input.points.at(j).point.pose.position.y;
        point2 << input.points.at(j - 1).point.pose.position.x,
          input.points.at(j - 1).point.pose.position.y;
        length_sum += (point2 - point1).norm();
      }
    } else {
      point1 << nearest_collision_point->x(), nearest_collision_point->y();
      point2 << path_line_end.x(), path_line_end.y();
      length_sum -= (point2 - point1).norm();
      for (size_t j = i + 1; j < input.points.size() - 1; ++j) {
        if (length_sum < offset) {
          target_point_idx = j;
          break;
        }
        point1 << input.points.at(j).point.pose.position.x,
          input.points.at(j).point.pose.position.y;
        point2 << input.points.at(j + 1).point.pose.position.x,
          input.points.at(j + 1).point.pose.position.y;
        length_sum -= (point2 - point1).norm();
      }
    }
    // create target point
    getBackwardPointFromBasePoint(
      point2, point1, point2, std::fabs(length_sum - offset), target_point);
    return true;
  }
  return false;
}

bool calcStopPointAndInsertIndex(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input_path,
  const lanelet::ConstLineString3d & lanelet_stop_lines, const double & offset,
  const double & stop_line_extend_length, Eigen::Vector2d & stop_line_point,
  size_t & stop_line_point_idx)
{
  LineString2d stop_line;

  for (size_t i = 0; i < lanelet_stop_lines.size() - 1; ++i) {
    stop_line = planning_utils::extendLine(
      lanelet_stop_lines[i], lanelet_stop_lines[i + 1], stop_line_extend_length);

    // Calculate stop pose and insert index,
    // if there is a collision point between path and stop line
    if (createTargetPoint(input_path, stop_line, offset, stop_line_point_idx, stop_line_point)) {
      return true;
    }
  }
  return false;
}
}  // namespace

TrafficLightModule::TrafficLightModule(
  const int64_t module_id, const int64_t lane_id,
  const lanelet::TrafficLight & traffic_light_reg_elem, lanelet::ConstLanelet lane,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  lane_id_(lane_id),
  traffic_light_reg_elem_(traffic_light_reg_elem),
  lane_(lane),
  state_(State::APPROACH),
  is_prev_state_stop_(false)
{
  velocity_factor_.init(PlanningBehavior::TRAFFIC_SIGNAL);
  planner_param_ = planner_param;
}

bool TrafficLightModule::modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason)
{
  debug_data_ = DebugData();
  debug_data_.base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  first_stop_path_point_index_ = static_cast<int>(path->points.size()) - 1;
  first_ref_stop_path_point_index_ = static_cast<int>(path->points.size()) - 1;
  *stop_reason = planning_utils::initializeStopReason(StopReason::TRAFFIC_LIGHT);

  const auto input_path = *path;

  const auto & self_pose = planner_data_->current_odometry;

  // Get lanelet2 stop lines.
  lanelet::ConstLineString3d lanelet_stop_lines = *(traffic_light_reg_elem_.stopLine());

  // Calculate stop pose and insert index
  Eigen::Vector2d stop_line_point{};
  size_t stop_line_point_idx{};
  if (!calcStopPointAndInsertIndex(
        input_path, lanelet_stop_lines,
        planner_param_.stop_margin + planner_data_->vehicle_info_.max_longitudinal_offset_m,
        planner_data_->stop_line_extend_length, stop_line_point, stop_line_point_idx)) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "Failed to calculate stop point and insert index");
    setSafe(true);
    setDistance(std::numeric_limits<double>::lowest());
    return false;
  }

  // Calculate dist to stop pose
  geometry_msgs::msg::Point stop_line_point_msg;
  stop_line_point_msg.x = stop_line_point.x();
  stop_line_point_msg.y = stop_line_point.y();
  const double signed_arc_length_to_stop_point = motion_utils::calcSignedArcLength(
    input_path.points, self_pose->pose.position, stop_line_point_msg);
  setDistance(signed_arc_length_to_stop_point);

  // Check state
  if (state_ == State::APPROACH) {
    // Move to go out state if ego vehicle over deadline.
    constexpr double signed_deadline_length = -2.0;
    if (signed_arc_length_to_stop_point < signed_deadline_length) {
      RCLCPP_INFO(logger_, "APPROACH -> GO_OUT");
      state_ = State::GO_OUT;
      stop_signal_received_time_ptr_.reset();
      return true;
    }

    first_ref_stop_path_point_index_ = stop_line_point_idx;

    // Check if stop is coming.
    const bool is_stop_signal = isStopSignal();

    // Update stop signal received time
    if (is_stop_signal) {
      if (!stop_signal_received_time_ptr_) {
        stop_signal_received_time_ptr_ = std::make_unique<Time>(clock_->now());
      }
    } else {
      stop_signal_received_time_ptr_.reset();
    }

    // Check hysteresis
    const double time_diff =
      stop_signal_received_time_ptr_
        ? std::max((clock_->now() - *stop_signal_received_time_ptr_).seconds(), 0.0)
        : 0.0;
    const bool to_be_stopped =
      is_stop_signal && (is_prev_state_stop_ || time_diff > planner_param_.stop_time_hysteresis);

    setSafe(!to_be_stopped);
    if (isActivated()) {
      is_prev_state_stop_ = false;
      return true;
    }

    // Decide whether to stop or pass even if a stop signal is received.
    if (!isPassthrough(signed_arc_length_to_stop_point)) {
      *path = insertStopPose(input_path, stop_line_point_idx, stop_line_point, stop_reason);
      is_prev_state_stop_ = true;
    }
    return true;
  } else if (state_ == State::GO_OUT) {
    // Initialize if vehicle is far from stop_line
    constexpr bool use_initialization_after_start = true;
    constexpr double restart_length = 1.0;
    if (use_initialization_after_start) {
      if (signed_arc_length_to_stop_point > restart_length) {
        RCLCPP_INFO(logger_, "GO_OUT(RESTART) -> APPROACH");
        state_ = State::APPROACH;
      }
    }
    stop_signal_received_time_ptr_.reset();
    return true;
  }

  return false;
}

bool TrafficLightModule::isStopSignal()
{
  updateTrafficSignal();

  // If it never receives traffic signal, it will PASS.
  if (!traffic_signal_stamp_) {
    return false;
  }

  if (isTrafficSignalTimedOut()) {
    return true;
  }

  return isTrafficSignalStop(looking_tl_state_);
}

void TrafficLightModule::updateTrafficSignal()
{
  TrafficSignalStamped signal;
  if (!findValidTrafficSignal(signal)) {
    // Don't stop if it never receives traffic light topic.
    return;
  }

  traffic_signal_stamp_ = signal.stamp;

  // Found signal associated with the lanelet
  looking_tl_state_ = signal.signal;
  return;
}

bool TrafficLightModule::isPassthrough(const double & signed_arc_length) const
{
  const double max_acc = planner_data_->max_stop_acceleration_threshold;
  const double max_jerk = planner_data_->max_stop_jerk_threshold;
  const double delay_response_time = planner_data_->delay_response_time;

  const double reachable_distance =
    planner_data_->current_velocity->twist.linear.x * planner_param_.yellow_lamp_period;

  // Calculate distance until ego vehicle decide not to stop,
  // taking into account the jerk and acceleration.
  const double pass_judge_line_distance = planning_utils::calcJudgeLineDistWithJerkLimit(
    planner_data_->current_velocity->twist.linear.x,
    planner_data_->current_acceleration->accel.accel.linear.x, max_acc, max_jerk,
    delay_response_time);

  const bool distance_stoppable = pass_judge_line_distance < signed_arc_length;
  const bool slow_velocity = planner_data_->current_velocity->twist.linear.x < 2.0;
  const bool stoppable = distance_stoppable || slow_velocity;
  const bool reachable = signed_arc_length < reachable_distance;

  const auto & enable_pass_judge = planner_param_.enable_pass_judge;

  if (enable_pass_judge && !stoppable && !is_prev_state_stop_) {
    // Cannot stop under acceleration and jerk limits.
    // However, ego vehicle can't enter the intersection while the light is yellow.
    // It is called dilemma zone. Make a stop decision to be safe.
    if (!reachable) {
      // dilemma zone: emergency stop
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "[traffic_light] cannot pass through intersection during yellow lamp!");
      return false;
    } else {
      // pass through
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000, "[traffic_light] can pass through intersection during yellow lamp");
      return true;
    }
  } else {
    return false;
  }
}

bool TrafficLightModule::isTrafficSignalStop(
  const autoware_perception_msgs::msg::TrafficSignal & tl_state) const
{
  if (hasTrafficLightCircleColor(tl_state, TrafficSignalElement::GREEN)) {
    return false;
  }

  const std::string turn_direction = lane_.attributeOr("turn_direction", "else");

  if (turn_direction == "else") {
    return true;
  }
  if (
    turn_direction == "right" &&
    hasTrafficLightShape(tl_state, TrafficSignalElement::RIGHT_ARROW)) {
    return false;
  }
  if (
    turn_direction == "left" && hasTrafficLightShape(tl_state, TrafficSignalElement::LEFT_ARROW)) {
    return false;
  }
  if (
    turn_direction == "straight" &&
    hasTrafficLightShape(tl_state, TrafficSignalElement::UP_ARROW)) {
    return false;
  }

  return true;
}

bool TrafficLightModule::findValidTrafficSignal(TrafficSignalStamped & valid_traffic_signal) const
{
  // get traffic signal associated with the regulatory element id
  const auto traffic_signal_stamped = planner_data_->getTrafficSignal(traffic_light_reg_elem_.id());
  if (!traffic_signal_stamped) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000 /* ms */,
      "the traffic signal data associated with regulatory element id is not received");
    return false;
  }

  valid_traffic_signal = *traffic_signal_stamped;
  return true;
}

bool TrafficLightModule::isTrafficSignalTimedOut() const
{
  if (!traffic_signal_stamp_) {
    return false;
  }

  const auto is_traffic_signal_timeout =
    (clock_->now() - *traffic_signal_stamp_).seconds() > planner_param_.tl_state_timeout;
  if (is_traffic_signal_timeout) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000 /* ms */, "the received traffic signal data is outdated");
    RCLCPP_WARN_STREAM_THROTTLE(
      logger_, *clock_, 5000 /* ms */,
      "time diff: " << (clock_->now() - *traffic_signal_stamp_).seconds());
    return true;
  }
  return false;
}

autoware_auto_planning_msgs::msg::PathWithLaneId TrafficLightModule::insertStopPose(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input,
  const size_t & insert_target_point_idx, const Eigen::Vector2d & target_point,
  tier4_planning_msgs::msg::StopReason * stop_reason)
{
  autoware_auto_planning_msgs::msg::PathWithLaneId modified_path;
  modified_path = input;

  // Create stop pose
  const int target_velocity_point_idx = std::max(static_cast<int>(insert_target_point_idx - 1), 0);
  auto target_point_with_lane_id = modified_path.points.at(target_velocity_point_idx);
  target_point_with_lane_id.point.pose.position.x = target_point.x();
  target_point_with_lane_id.point.pose.position.y = target_point.y();
  target_point_with_lane_id.point.longitudinal_velocity_mps = 0.0;
  debug_data_.stop_poses.push_back(target_point_with_lane_id.point.pose);

  // Insert stop pose into path or replace with zero velocity
  size_t insert_index = insert_target_point_idx;
  planning_utils::insertVelocity(modified_path, target_point_with_lane_id, 0.0, insert_index);
  if (static_cast<int>(target_velocity_point_idx) < first_stop_path_point_index_) {
    first_stop_path_point_index_ = static_cast<int>(target_velocity_point_idx);
    debug_data_.first_stop_pose = target_point_with_lane_id.point.pose;
  }

  // Get stop point and stop factor
  tier4_planning_msgs::msg::StopFactor stop_factor;
  stop_factor.stop_pose = debug_data_.first_stop_pose;
  if (debug_data_.highest_confidence_traffic_light_point != std::nullopt) {
    stop_factor.stop_factor_points = std::vector<geometry_msgs::msg::Point>{
      debug_data_.highest_confidence_traffic_light_point.value()};
  }
  velocity_factor_.set(
    modified_path.points, planner_data_->current_odometry->pose,
    target_point_with_lane_id.point.pose, VelocityFactor::UNKNOWN);
  planning_utils::appendStopReason(stop_factor, stop_reason);

  return modified_path;
}

bool TrafficLightModule::hasTrafficLightCircleColor(
  const autoware_perception_msgs::msg::TrafficSignal & tl_state, const uint8_t & lamp_color) const
{
  const auto it_lamp =
    std::find_if(tl_state.elements.begin(), tl_state.elements.end(), [&lamp_color](const auto & x) {
      return x.shape == TrafficSignalElement::CIRCLE && x.color == lamp_color;
    });

  return it_lamp != tl_state.elements.end();
}

bool TrafficLightModule::hasTrafficLightShape(
  const autoware_perception_msgs::msg::TrafficSignal & tl_state, const uint8_t & lamp_shape) const
{
  const auto it_lamp = std::find_if(
    tl_state.elements.begin(), tl_state.elements.end(),
    [&lamp_shape](const auto & x) { return x.shape == lamp_shape; });

  return it_lamp != tl_state.elements.end();
}

}  // namespace behavior_velocity_planner
