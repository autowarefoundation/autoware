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

#include <scene_module/virtual_traffic_light/scene.hpp>
#include <tier4_autoware_utils/trajectory/trajectory.hpp>
#include <utilization/util.hpp>

#include <tier4_v2x_msgs/msg/key_value.hpp>

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace tier4_autoware_utils
{
template <>
inline geometry_msgs::msg::Pose getPose(
  const autoware_auto_planning_msgs::msg::PathPointWithLaneId & p)
{
  return p.point.pose;
}
}  // namespace tier4_autoware_utils

namespace behavior_velocity_planner
{
namespace
{
namespace bg = boost::geometry;
using tier4_autoware_utils::calcDistance2d;

struct SegmentIndexWithPoint
{
  size_t index;
  geometry_msgs::msg::Point point;
};

struct SegmentIndexWithOffset
{
  size_t index;
  double offset;
};

tier4_v2x_msgs::msg::KeyValue createKeyValue(const std::string & key, const std::string & value)
{
  return tier4_v2x_msgs::build<tier4_v2x_msgs::msg::KeyValue>().key(key).value(value);
}

tier4_autoware_utils::LineString3d toAutowarePoints(const lanelet::ConstLineString3d & line_string)
{
  tier4_autoware_utils::LineString3d output;
  for (const auto & p : line_string) {
    output.emplace_back(p.x(), p.y(), p.z());
  }
  return output;
}

boost::optional<tier4_autoware_utils::LineString3d> toAutowarePoints(
  const lanelet::Optional<lanelet::ConstLineString3d> & line_string)
{
  if (!line_string) {
    return {};
  }
  return toAutowarePoints(*line_string);
}

std::vector<tier4_autoware_utils::LineString3d> toAutowarePoints(
  const lanelet::ConstLineStrings3d & line_strings)
{
  std::vector<tier4_autoware_utils::LineString3d> output;
  for (const auto & line_string : line_strings) {
    output.emplace_back(toAutowarePoints(line_string));
  }
  return output;
}

tier4_autoware_utils::LineString2d to_2d(const tier4_autoware_utils::LineString3d & line_string)
{
  tier4_autoware_utils::LineString2d output;
  for (const auto & p : line_string) {
    output.emplace_back(p.x(), p.y());
  }
  return output;
}

tier4_autoware_utils::Point3d calcCenter(const tier4_autoware_utils::LineString3d & line_string)
{
  const auto p1 = line_string.front();
  const auto p2 = line_string.back();
  const auto p_center = (p1 + p2) / 2;
  return {p_center.x(), p_center.y(), p_center.z()};
}

geometry_msgs::msg::Pose calcHeadPose(
  const geometry_msgs::msg::Pose & base_link_pose, const double base_link_to_front)
{
  return tier4_autoware_utils::calcOffsetPose(base_link_pose, base_link_to_front, 0.0, 0.0);
}

template <class T>
boost::optional<SegmentIndexWithPoint> findCollision(
  const T & points, const tier4_autoware_utils::LineString3d & line)
{
  for (size_t i = 0; i < points.size() - 1; ++i) {
    // Create path segment
    const auto & p_front = tier4_autoware_utils::getPoint(points.at(i));
    const auto & p_back = tier4_autoware_utils::getPoint(points.at(i + 1));
    const tier4_autoware_utils::LineString2d path_segment{
      {p_front.x, p_front.y}, {p_back.x, p_back.y}};

    // Find intersection
    std::vector<tier4_autoware_utils::Point2d> collision_points;
    bg::intersection(to_2d(line), path_segment, collision_points);

    // Ignore if no collision found
    if (collision_points.empty()) {
      continue;
    }

    // Select first collision if found
    const auto collision_point_2d = collision_points.front();

    // Calculate interpolation ratio
    const auto interpolate_ratio =
      calcDistance2d(p_front, toMsg(collision_point_2d.to_3d(0))) / calcDistance2d(p_front, p_back);

    // Interpolate z
    const double interpolated_z = p_front.z + interpolate_ratio * (p_back.z - p_front.z);

    // To point
    const auto collision_point = tier4_autoware_utils::createPoint(
      collision_point_2d.x(), collision_point_2d.y(), interpolated_z);

    return SegmentIndexWithPoint{i, collision_point};
  }

  return {};
}

template <class T>
boost::optional<SegmentIndexWithPoint> findCollision(
  const T & points, const std::vector<tier4_autoware_utils::LineString3d> & lines)
{
  for (const auto & line : lines) {
    const auto collision = findCollision(points, line);
    if (collision) {
      return collision;
    }
  }

  return {};
}

SegmentIndexWithOffset findForwardOffsetSegment(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t base_idx,
  const double offset_length)
{
  double sum_length = 0.0;
  for (size_t i = base_idx; i < path.points.size() - 1; ++i) {
    const auto p_front = path.points.at(i);
    const auto p_back = path.points.at(i + 1);

    const auto segment_length = calcDistance2d(p_front, p_back);
    sum_length += segment_length;

    // If it's over offset length, return front index and offset from front point
    if (sum_length >= offset_length) {
      const auto remain_length = sum_length - offset_length;
      return SegmentIndexWithOffset{i, segment_length - remain_length};
    }
  }

  // No enough length
  const auto last_segment_length =
    calcDistance2d(path.points.at(path.points.size() - 2), path.points.at(path.points.size() - 1));

  return {path.points.size() - 2, offset_length - sum_length + last_segment_length};
}

SegmentIndexWithOffset findBackwardOffsetSegment(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t base_idx,
  const double offset_length)
{
  double sum_length = 0.0;
  for (size_t i = base_idx; i > 0; --i) {
    const auto p_front = path.points.at(i - 1);
    const auto p_back = path.points.at(i);

    const auto segment_length = calcDistance2d(p_front, p_back);
    sum_length += segment_length;

    // If it's over offset length, return front index and offset from front point
    if (sum_length >= offset_length) {
      const auto remain_length = sum_length - offset_length;
      return SegmentIndexWithOffset{i - 1, remain_length};
    }
  }

  // No enough length
  return {0, sum_length - offset_length};
}

SegmentIndexWithOffset findOffsetSegment(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t index,
  const double offset)
{
  if (offset >= 0) {
    return findForwardOffsetSegment(path, index, offset);
  }

  return findBackwardOffsetSegment(path, index, -offset);
}

geometry_msgs::msg::Pose calcInterpolatedPose(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t index,
  const double offset)
{
  // Get segment points
  const auto & p_front = path.points.at(index).point.pose.position;
  const auto & p_back = path.points.at(index + 1).point.pose.position;

  // To Eigen point
  const auto p_eigen_front = Eigen::Vector2d(p_front.x, p_front.y);
  const auto p_eigen_back = Eigen::Vector2d(p_back.x, p_back.y);

  // Calculate interpolation ratio
  const auto interpolate_ratio = offset / (p_eigen_back - p_eigen_front).norm();

  // Add offset to front point
  const auto interpolated_point_2d =
    p_eigen_front + interpolate_ratio * (p_eigen_back - p_eigen_front);
  const double interpolated_z = p_front.z + interpolate_ratio * (p_back.z - p_front.z);

  // Calculate orientation so that X-axis would be along the trajectory
  tf2::Quaternion quat;
  quat.setRPY(0, 0, tier4_autoware_utils::calcAzimuthAngle(p_front, p_back));

  // To Pose
  geometry_msgs::msg::Pose interpolated_pose;
  interpolated_pose.position.x = interpolated_point_2d.x();
  interpolated_pose.position.y = interpolated_point_2d.y();
  interpolated_pose.position.z = interpolated_z;
  interpolated_pose.orientation = tf2::toMsg(quat);

  return interpolated_pose;
}

void insertStopVelocityFromStart(autoware_auto_planning_msgs::msg::PathWithLaneId * path)
{
  for (auto & p : path->points) {
    p.point.longitudinal_velocity_mps = 0.0;
  }
}

size_t insertStopVelocityAtCollision(
  const SegmentIndexWithPoint & collision, const double offset,
  autoware_auto_planning_msgs::msg::PathWithLaneId * path)
{
  const auto collision_offset = tier4_autoware_utils::calcLongitudinalOffsetToSegment(
    path->points, collision.index, collision.point);

  const auto offset_segment = findOffsetSegment(*path, collision.index, offset + collision_offset);
  const auto interpolated_pose =
    calcInterpolatedPose(*path, offset_segment.index, offset_segment.offset);

  if (offset_segment.offset < 0) {
    insertStopVelocityFromStart(path);
    return 0;
  }

  const auto insert_index = offset_segment.index + 1;
  auto insert_point = path->points.at(insert_index);
  insert_point.point.pose = interpolated_pose;

  path->points.insert(path->points.begin() + insert_index, insert_point);

  // Insert 0 velocity after stop point
  for (size_t i = insert_index; i < path->points.size(); ++i) {
    path->points.at(i).point.longitudinal_velocity_mps = 0.0;
  }

  return insert_index;
}
}  // namespace

VirtualTrafficLightModule::VirtualTrafficLightModule(
  const int64_t module_id, const lanelet::autoware::VirtualTrafficLight & reg_elem,
  lanelet::ConstLanelet lane, const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  module_id_(module_id),
  reg_elem_(reg_elem),
  lane_(lane),
  planner_param_(planner_param)
{
  // Get map data
  const auto instrument = reg_elem_.getVirtualTrafficLight();
  const auto instrument_bottom_line = toAutowarePoints(instrument);

  // Information from instrument
  {
    map_data_.instrument_type = *instrument.attribute("type").as<std::string>();
    map_data_.instrument_id = std::to_string(instrument.id());
    map_data_.instrument_center = calcCenter(instrument_bottom_line);
  }

  // Information from regulatory_element
  {
    map_data_.stop_line = toAutowarePoints(reg_elem_.getStopLine());
    map_data_.start_line = toAutowarePoints(reg_elem_.getStartLine());
    map_data_.end_lines = toAutowarePoints(reg_elem_.getEndLines());
  }

  // Custom tags
  {
    // Map attributes
    for (const auto & attr : instrument.attributes()) {
      const auto & key = attr.first;
      const auto & value = *attr.second.as<std::string>();

      // Ignore mandatory tags
      if (key == "type") {
        continue;
      }

      map_data_.custom_tags.emplace_back(createKeyValue(key, value));
    }

    // Lane ID
    map_data_.custom_tags.emplace_back(createKeyValue("lane_id", std::to_string(lane_.id())));

    // Turn direction
    map_data_.custom_tags.emplace_back(
      createKeyValue("turn_direction", lane_.attributeOr("turn_direction", "straight")));
  }

  // Set command
  command_.type = map_data_.instrument_type;
  command_.id = map_data_.instrument_id;
  command_.custom_tags = map_data_.custom_tags;
}

bool VirtualTrafficLightModule::modifyPathVelocity(
  autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  tier4_planning_msgs::msg::StopReason * stop_reason)
{
  // Initialize
  setInfrastructureCommand({});
  *stop_reason = planning_utils::initializeStopReason(
    tier4_planning_msgs::msg::StopReason::VIRTUAL_TRAFFIC_LIGHT);
  module_data_ = {};

  // Copy data
  module_data_.head_pose = calcHeadPose(
    planner_data_->current_pose.pose, planner_data_->vehicle_info_.max_longitudinal_offset_m);
  module_data_.path = *path;

  // Do nothing if vehicle is before start line
  if (isBeforeStartLine()) {
    RCLCPP_DEBUG(logger_, "before start_line");
    state_ = State::NONE;
    updateInfrastructureCommand();
    return true;
  }

  // Do nothing if vehicle is after any end line
  if (isAfterAnyEndLine()) {
    RCLCPP_DEBUG(logger_, "after end_line");
    state_ = State::FINALIZED;
    updateInfrastructureCommand();
    return true;
  }

  // Set state
  state_ = State::REQUESTING;

  // Don't need to stop if there is no stop_line
  if (!map_data_.stop_line) {
    updateInfrastructureCommand();
    return true;
  }

  // Find corresponding state
  const auto virtual_traffic_light_state = findCorrespondingState();

  // Stop at stop_line if no message received
  if (!virtual_traffic_light_state) {
    RCLCPP_DEBUG(logger_, "no message received");
    insertStopVelocityAtStopLine(path, stop_reason);
    updateInfrastructureCommand();
    return true;
  }

  // Stop at stop_line if no right is given
  if (!hasRightOfWay(*virtual_traffic_light_state)) {
    RCLCPP_DEBUG(logger_, "no right is given");
    insertStopVelocityAtStopLine(path, stop_reason);
    updateInfrastructureCommand();
    return true;
  }

  // Stop at stop_line if state is timeout before stop_line
  if (isBeforeStopLine()) {
    if (isStateTimeout(*virtual_traffic_light_state)) {
      RCLCPP_DEBUG(logger_, "state is timeout before stop line");
      insertStopVelocityAtStopLine(path, stop_reason);
    }

    updateInfrastructureCommand();
    return true;
  }

  // After stop_line
  state_ = State::PASSING;

  // Check timeout after stop_line if the option is on
  if (
    planner_param_.check_timeout_after_stop_line && isStateTimeout(*virtual_traffic_light_state)) {
    RCLCPP_DEBUG(logger_, "state is timeout after stop line");
    insertStopVelocityAtStopLine(path, stop_reason);
    updateInfrastructureCommand();
    return true;
  }

  // Stop at stop_line if finalization isn't completed
  if (!virtual_traffic_light_state->is_finalized) {
    RCLCPP_DEBUG(logger_, "finalization isn't completed");
    insertStopVelocityAtEndLine(path, stop_reason);

    if (isNearAnyEndLine() && planner_data_->isVehicleStopped()) {
      state_ = State::FINALIZING;
    }
  }

  updateInfrastructureCommand();
  return true;
}

void VirtualTrafficLightModule::updateInfrastructureCommand()
{
  command_.stamp = clock_->now();
  command_.state = static_cast<uint8_t>(state_);
  setInfrastructureCommand(command_);
}

void VirtualTrafficLightModule::setStopReason(
  const geometry_msgs::msg::Pose & stop_pose, tier4_planning_msgs::msg::StopReason * stop_reason)
{
  tier4_planning_msgs::msg::StopFactor stop_factor;
  stop_factor.stop_pose = stop_pose;
  stop_factor.stop_factor_points.push_back(toMsg(map_data_.instrument_center));
  planning_utils::appendStopReason(stop_factor, stop_reason);
}

bool VirtualTrafficLightModule::isBeforeStartLine()
{
  const auto collision = findCollision(module_data_.path.points, map_data_.start_line);

  // Since the module is registered, a collision should be detected usually.
  // Therefore if no collision found, vehicle's path is fully after the line.
  if (!collision) {
    return false;
  }

  const double max_dist = std::numeric_limits<double>::max();
  const auto signed_arc_length = tier4_autoware_utils::calcSignedArcLength(
    module_data_.path.points, module_data_.head_pose, collision->point, max_dist,
    planner_param_.max_yaw_deviation_rad);

  return *signed_arc_length > 0;
}

bool VirtualTrafficLightModule::isBeforeStopLine()
{
  const auto collision = findCollision(module_data_.path.points, *map_data_.stop_line);

  // Since the module is registered, a collision should be detected usually.
  // Therefore if no collision found, vehicle's path is fully after the line.
  if (!collision) {
    return false;
  }

  const double max_dist = std::numeric_limits<double>::max();
  const auto signed_arc_length = tier4_autoware_utils::calcSignedArcLength(
    module_data_.path.points, module_data_.head_pose, collision->point, max_dist,
    planner_param_.max_yaw_deviation_rad);

  return *signed_arc_length > -planner_param_.dead_line_margin;
}

bool VirtualTrafficLightModule::isAfterAnyEndLine()
{
  // Assume stop line is before end lines
  if (isBeforeStopLine()) {
    return false;
  }

  const auto collision = findCollision(module_data_.path.points, map_data_.end_lines);

  // If the goal is set before the end line, collision will not be detected.
  // Therefore if there is no collision, the ego vehicle is assumed to be before the end line.
  if (!collision) {
    return false;
  }

  const double max_dist = std::numeric_limits<double>::max();
  const auto signed_arc_length = tier4_autoware_utils::calcSignedArcLength(
    module_data_.path.points, module_data_.head_pose, collision->point, max_dist,
    planner_param_.max_yaw_deviation_rad);

  return *signed_arc_length < -planner_param_.dead_line_margin;
}

bool VirtualTrafficLightModule::isNearAnyEndLine()
{
  const auto collision = findCollision(module_data_.path.points, map_data_.end_lines);

  if (!collision) {
    return false;
  }

  const double max_dist = std::numeric_limits<double>::max();
  const auto signed_arc_length = tier4_autoware_utils::calcSignedArcLength(
    module_data_.path.points, module_data_.head_pose, collision->point, max_dist,
    planner_param_.max_yaw_deviation_rad);

  return std::abs(*signed_arc_length) < planner_param_.near_line_distance;
}

boost::optional<tier4_v2x_msgs::msg::VirtualTrafficLightState>
VirtualTrafficLightModule::findCorrespondingState()
{
  // No message
  if (!planner_data_->virtual_traffic_light_states) {
    return {};
  }

  for (const auto & state : planner_data_->virtual_traffic_light_states->states) {
    if (state.id == map_data_.instrument_id) {
      return state;
    }
  }

  return {};
}

bool VirtualTrafficLightModule::isStateTimeout(
  const tier4_v2x_msgs::msg::VirtualTrafficLightState & state)
{
  const auto delay = (clock_->now() - rclcpp::Time(state.stamp)).seconds();
  if (delay > planner_param_.max_delay_sec) {
    RCLCPP_DEBUG(logger_, "delay=%f, max_delay=%f", delay, planner_param_.max_delay_sec);
    return true;
  }

  return false;
}

bool VirtualTrafficLightModule::hasRightOfWay(
  const tier4_v2x_msgs::msg::VirtualTrafficLightState & state)
{
  return state.approval;
}

void VirtualTrafficLightModule::insertStopVelocityAtStopLine(
  autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  tier4_planning_msgs::msg::StopReason * stop_reason)
{
  const auto collision = findCollision(path->points, *map_data_.stop_line);

  geometry_msgs::msg::Pose stop_pose{};
  if (!collision) {
    insertStopVelocityFromStart(path);
    stop_pose = planner_data_->current_pose.pose;
  } else {
    const auto offset = -planner_data_->vehicle_info_.max_longitudinal_offset_m;
    const auto insert_index = insertStopVelocityAtCollision(*collision, offset, path);
    stop_pose = path->points.at(insert_index).point.pose;
  }

  // Set StopReason
  setStopReason(stop_pose, stop_reason);

  // Set data for visualization
  module_data_.stop_head_pose_at_stop_line =
    calcHeadPose(stop_pose, planner_data_->vehicle_info_.max_longitudinal_offset_m);
}

void VirtualTrafficLightModule::insertStopVelocityAtEndLine(
  autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  tier4_planning_msgs::msg::StopReason * stop_reason)
{
  const auto collision = findCollision(path->points, map_data_.end_lines);

  geometry_msgs::msg::Pose stop_pose{};
  if (!collision) {
    // No enough length
    if (isBeforeStopLine()) {
      return;
    }

    insertStopVelocityFromStart(path);
    stop_pose = planner_data_->current_pose.pose;
  } else {
    const auto offset = -planner_data_->vehicle_info_.max_longitudinal_offset_m;
    const auto insert_index = insertStopVelocityAtCollision(*collision, offset, path);
    stop_pose = path->points.at(insert_index).point.pose;
  }

  // Set StopReason
  setStopReason(stop_pose, stop_reason);

  // Set data for visualization
  module_data_.stop_head_pose_at_end_line =
    calcHeadPose(stop_pose, planner_data_->vehicle_info_.max_longitudinal_offset_m);
}
}  // namespace behavior_velocity_planner
