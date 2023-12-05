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

#include "scene.hpp"

#include <behavior_velocity_planner_common/utilization/arc_lane_util.hpp>
#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <motion_utils/trajectory/trajectory.hpp>

#include <tier4_v2x_msgs/msg/key_value.hpp>

#include <string>
#include <vector>

namespace behavior_velocity_planner
{
namespace
{
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

std::optional<tier4_autoware_utils::LineString3d> toAutowarePoints(
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

[[maybe_unused]] tier4_autoware_utils::LineString2d to_2d(
  const tier4_autoware_utils::LineString3d & line_string)
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

geometry_msgs::msg::Point convertToGeomPoint(const tier4_autoware_utils::Point3d & p)
{
  geometry_msgs::msg::Point geom_p;
  geom_p.x = p.x();
  geom_p.y = p.y();

  return geom_p;
}

template <class T>
std::optional<SegmentIndexWithPoint> findLastCollisionBeforeEndLine(
  const T & points, const tier4_autoware_utils::LineString3d & target_line,
  const size_t end_line_idx)
{
  const auto target_line_p1 = convertToGeomPoint(target_line.at(0));
  const auto target_line_p2 = convertToGeomPoint(target_line.at(1));

  for (size_t i = end_line_idx; 0 < i;
       --i) {  // NOTE: size_t can be used since it will not be negative.
    const auto & p1 = tier4_autoware_utils::getPoint(points.at(i));
    const auto & p2 = tier4_autoware_utils::getPoint(points.at(i - 1));
    const auto collision_point =
      arc_lane_utils::checkCollision(p1, p2, target_line_p1, target_line_p2);

    if (collision_point) {
      return SegmentIndexWithPoint{i, collision_point.value()};
    }
  }

  return {};
}

template <class T>
std::optional<SegmentIndexWithPoint> findLastCollisionBeforeEndLine(
  const T & points, const std::vector<tier4_autoware_utils::LineString3d> & lines,
  const size_t end_line_idx)
{
  for (const auto & line : lines) {
    const auto collision = findLastCollisionBeforeEndLine(points, line, end_line_idx);
    if (collision) {
      return collision;
    }
  }

  return {};
}

void insertStopVelocityFromStart(autoware_auto_planning_msgs::msg::PathWithLaneId * path)
{
  for (auto & p : path->points) {
    p.point.longitudinal_velocity_mps = 0.0;
  }
}

std::optional<size_t> insertStopVelocityAtCollision(
  const SegmentIndexWithPoint & collision, const double offset,
  autoware_auto_planning_msgs::msg::PathWithLaneId * path)
{
  const auto collision_offset =
    motion_utils::calcLongitudinalOffsetToSegment(path->points, collision.index, collision.point);

  const auto offset_segment =
    arc_lane_utils::findOffsetSegment(*path, collision.index, offset + collision_offset);
  if (!offset_segment) {
    return {};
  }

  const auto interpolated_pose = arc_lane_utils::calcTargetPose(*path, *offset_segment);

  if (offset_segment->second < 0) {
    insertStopVelocityFromStart(path);
    return 0;
  }

  auto insert_index = static_cast<size_t>(offset_segment->first + 1);
  auto insert_point = path->points.at(insert_index);
  insert_point.point.pose = interpolated_pose;
  // Insert 0 velocity after stop point or replace velocity with 0
  behavior_velocity_planner::planning_utils::insertVelocity(*path, insert_point, 0.0, insert_index);
  return insert_index;
}
}  // namespace

VirtualTrafficLightModule::VirtualTrafficLightModule(
  const int64_t module_id, const int64_t lane_id,
  const lanelet::autoware::VirtualTrafficLight & reg_elem, lanelet::ConstLanelet lane,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  lane_id_(lane_id),
  reg_elem_(reg_elem),
  lane_(lane),
  planner_param_(planner_param)
{
  velocity_factor_.init(PlanningBehavior::VIRTUAL_TRAFFIC_LIGHT);

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

  // Add instrument information to the logger
  logger_ = logger_.get_child((map_data_.instrument_type + "_" + map_data_.instrument_id).c_str());
}

bool VirtualTrafficLightModule::modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason)
{
  // Initialize
  setInfrastructureCommand({});
  *stop_reason = planning_utils::initializeStopReason(StopReason::VIRTUAL_TRAFFIC_LIGHT);

  module_data_ = {};

  // Copy data
  module_data_.head_pose = calcHeadPose(
    planner_data_->current_odometry->pose, planner_data_->vehicle_info_.max_longitudinal_offset_m);
  module_data_.path = *path;

  // Calculate path index of end line
  // NOTE: In order to deal with u-turn or self-crossing path, only start/stop lines before the end
  // line are used when whether the ego is before/after the start/stop/end lines is calculated.
  const auto opt_end_line_idx = getPathIndexOfFirstEndLine();
  if (!opt_end_line_idx) {
    return true;
  }
  const size_t end_line_idx = opt_end_line_idx.value();

  // Do nothing if vehicle is before start line
  if (isBeforeStartLine(end_line_idx)) {
    RCLCPP_DEBUG(logger_, "before start_line");
    state_ = State::NONE;
    updateInfrastructureCommand();
    return true;
  }

  // Do nothing if vehicle is after any end line
  if (isAfterAnyEndLine(end_line_idx) || state_ == State::FINALIZED) {
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
    insertStopVelocityAtStopLine(path, stop_reason, end_line_idx);
    updateInfrastructureCommand();
    return true;
  }

  // Stop at stop_line if no right is given
  if (!hasRightOfWay(*virtual_traffic_light_state)) {
    RCLCPP_DEBUG(logger_, "no right is given");
    insertStopVelocityAtStopLine(path, stop_reason, end_line_idx);
    updateInfrastructureCommand();
    return true;
  }

  // Stop at stop_line if state is timeout before stop_line
  if (isBeforeStopLine(end_line_idx)) {
    if (isStateTimeout(*virtual_traffic_light_state)) {
      RCLCPP_DEBUG(logger_, "state is timeout before stop line");
      insertStopVelocityAtStopLine(path, stop_reason, end_line_idx);
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
    insertStopVelocityAtStopLine(path, stop_reason, end_line_idx);
    updateInfrastructureCommand();
    return true;
  }

  // Stop at stop_line if finalization isn't completed
  if (!virtual_traffic_light_state->is_finalized) {
    RCLCPP_DEBUG(logger_, "finalization isn't completed");
    insertStopVelocityAtEndLine(path, stop_reason, end_line_idx);

    if (isNearAnyEndLine(end_line_idx) && planner_data_->isVehicleStopped()) {
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

std::optional<size_t> VirtualTrafficLightModule::getPathIndexOfFirstEndLine()
{
  std::optional<size_t> min_seg_idx;
  for (const auto & end_line : map_data_.end_lines) {
    geometry_msgs::msg::Point end_line_p1;
    end_line_p1.x = end_line.front().x();
    end_line_p1.y = end_line.front().y();

    geometry_msgs::msg::Point end_line_p2;
    end_line_p2.x = end_line.back().x();
    end_line_p2.y = end_line.back().y();

    const auto collision =
      arc_lane_utils::findCollisionSegment(module_data_.path, end_line_p1, end_line_p2, lane_id_);
    if (!collision) {
      continue;
    }

    const size_t collision_seg_idx = collision->first;

    if (!min_seg_idx || collision_seg_idx < min_seg_idx.value()) {
      min_seg_idx =
        collision_seg_idx + 1;  // NOTE: In order that min_seg_idx will be after the end line
    }
  }

  return min_seg_idx;
}

bool VirtualTrafficLightModule::isBeforeStartLine(const size_t end_line_idx)
{
  const auto collision =
    findLastCollisionBeforeEndLine(module_data_.path.points, map_data_.start_line, end_line_idx);

  // Since the module is registered, a collision should be detected usually.
  // Therefore if no collision found, vehicle's path is fully after the line.
  if (!collision) {
    return false;
  }
  const size_t collision_seg_idx = planning_utils::calcSegmentIndexFromPointIndex(
    module_data_.path.points, collision->point, collision->index);

  const auto & ego_pose = planner_data_->current_odometry->pose;
  const size_t ego_seg_idx = findEgoSegmentIndex(module_data_.path.points);
  const auto signed_arc_length = motion_utils::calcSignedArcLength(
                                   module_data_.path.points, ego_pose.position, ego_seg_idx,
                                   collision->point, collision_seg_idx) -
                                 planner_data_->vehicle_info_.max_longitudinal_offset_m;

  return signed_arc_length > 0;
}

bool VirtualTrafficLightModule::isBeforeStopLine(const size_t end_line_idx)
{
  const auto collision =
    findLastCollisionBeforeEndLine(module_data_.path.points, *map_data_.stop_line, end_line_idx);

  // Since the module is registered, a collision should be detected usually.
  // Therefore if no collision found, vehicle's path is fully after the line.
  if (!collision) {
    return false;
  }
  const size_t collision_seg_idx = planning_utils::calcSegmentIndexFromPointIndex(
    module_data_.path.points, collision->point, collision->index);

  const auto & ego_pose = planner_data_->current_odometry->pose;
  const size_t ego_seg_idx = findEgoSegmentIndex(module_data_.path.points);
  const auto signed_arc_length = motion_utils::calcSignedArcLength(
                                   module_data_.path.points, ego_pose.position, ego_seg_idx,
                                   collision->point, collision_seg_idx) -
                                 planner_data_->vehicle_info_.max_longitudinal_offset_m;

  return signed_arc_length > -planner_param_.dead_line_margin;
}

bool VirtualTrafficLightModule::isAfterAnyEndLine(const size_t end_line_idx)
{
  // Assume stop line is before end lines
  if (isBeforeStopLine(end_line_idx)) {
    return false;
  }

  const auto collision =
    findLastCollisionBeforeEndLine(module_data_.path.points, map_data_.end_lines, end_line_idx);

  // If the goal is set before the end line, collision will not be detected.
  // Therefore if there is no collision, the ego vehicle is assumed to be before the end line.
  if (!collision) {
    return false;
  }
  const size_t collision_seg_idx = planning_utils::calcSegmentIndexFromPointIndex(
    module_data_.path.points, collision->point, collision->index);

  const auto & ego_pose = planner_data_->current_odometry->pose;
  const size_t ego_seg_idx = findEgoSegmentIndex(module_data_.path.points);
  const auto signed_arc_length = motion_utils::calcSignedArcLength(
                                   module_data_.path.points, ego_pose.position, ego_seg_idx,
                                   collision->point, collision_seg_idx) -
                                 planner_data_->vehicle_info_.max_longitudinal_offset_m;

  return signed_arc_length < -planner_param_.dead_line_margin;
}

bool VirtualTrafficLightModule::isNearAnyEndLine(const size_t end_line_idx)
{
  const auto collision =
    findLastCollisionBeforeEndLine(module_data_.path.points, map_data_.end_lines, end_line_idx);

  if (!collision) {
    return false;
  }
  const size_t collision_seg_idx = planning_utils::calcSegmentIndexFromPointIndex(
    module_data_.path.points, collision->point, collision->index);

  const auto & ego_pose = planner_data_->current_odometry->pose;
  const size_t ego_seg_idx = findEgoSegmentIndex(module_data_.path.points);
  const auto signed_arc_length = motion_utils::calcSignedArcLength(
                                   module_data_.path.points, ego_pose.position, ego_seg_idx,
                                   collision->point, collision_seg_idx) -
                                 planner_data_->vehicle_info_.max_longitudinal_offset_m;

  return std::abs(signed_arc_length) < planner_param_.near_line_distance;
}

std::optional<tier4_v2x_msgs::msg::VirtualTrafficLightState>
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
  tier4_planning_msgs::msg::StopReason * stop_reason, const size_t end_line_idx)
{
  const auto collision =
    findLastCollisionBeforeEndLine(path->points, *map_data_.stop_line, end_line_idx);
  const auto offset = -planner_data_->vehicle_info_.max_longitudinal_offset_m;

  geometry_msgs::msg::Pose stop_pose{};
  if (!collision) {
    insertStopVelocityFromStart(path);
    stop_pose = planner_data_->current_odometry->pose;
  } else {
    const auto & ego_pose = planner_data_->current_odometry->pose;
    const size_t ego_seg_idx = findEgoSegmentIndex(path->points);
    const size_t collision_seg_idx = planning_utils::calcSegmentIndexFromPointIndex(
      path->points, collision->point, collision->index);

    const auto stop_distance =
      motion_utils::calcSignedArcLength(
        path->points, ego_pose.position, ego_seg_idx, collision.value().point, collision_seg_idx) +
      offset;
    const auto is_stopped = planner_data_->isVehicleStopped();

    if (stop_distance < planner_param_.hold_stop_margin_distance && is_stopped) {
      SegmentIndexWithPoint new_collision;
      const auto ego_pos_on_path =
        motion_utils::calcLongitudinalOffsetPoint(path->points, ego_pose.position, 0.0);

      if (ego_pos_on_path) {
        new_collision.point = ego_pos_on_path.value();
        new_collision.index = ego_seg_idx;
        insertStopVelocityAtCollision(new_collision, 0.0, path);
      }

      // for virtual wall
      {
        auto path_tmp = path;
        const auto insert_index = insertStopVelocityAtCollision(*collision, offset, path_tmp);
        if (insert_index) {
          stop_pose = path_tmp->points.at(insert_index.value()).point.pose;
        }
      }

    } else {
      const auto insert_index = insertStopVelocityAtCollision(*collision, offset, path);
      if (insert_index) {
        stop_pose = path->points.at(insert_index.value()).point.pose;
      }
    }
  }

  // Set StopReason
  setStopReason(stop_pose, stop_reason);
  velocity_factor_.set(
    path->points, planner_data_->current_odometry->pose, stop_pose, VelocityFactor::UNKNOWN,
    command_.type);

  // Set data for visualization
  module_data_.stop_head_pose_at_stop_line =
    calcHeadPose(stop_pose, planner_data_->vehicle_info_.max_longitudinal_offset_m);
}

void VirtualTrafficLightModule::insertStopVelocityAtEndLine(
  autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  tier4_planning_msgs::msg::StopReason * stop_reason, const size_t end_line_idx)
{
  const auto collision =
    findLastCollisionBeforeEndLine(path->points, map_data_.end_lines, end_line_idx);

  geometry_msgs::msg::Pose stop_pose{};
  if (!collision) {
    // No enough length
    if (isBeforeStopLine(end_line_idx)) {
      return;
    }

    insertStopVelocityFromStart(path);
    stop_pose = planner_data_->current_odometry->pose;
  } else {
    const auto offset = -planner_data_->vehicle_info_.max_longitudinal_offset_m;
    const auto insert_index = insertStopVelocityAtCollision(*collision, offset, path);
    if (insert_index) {
      stop_pose = path->points.at(insert_index.value()).point.pose;
    }
  }

  // Set StopReason
  setStopReason(stop_pose, stop_reason);
  velocity_factor_.set(
    path->points, planner_data_->current_odometry->pose, stop_pose, VelocityFactor::UNKNOWN);

  // Set data for visualization
  module_data_.stop_head_pose_at_end_line =
    calcHeadPose(stop_pose, planner_data_->vehicle_info_.max_longitudinal_offset_m);
}
}  // namespace behavior_velocity_planner
