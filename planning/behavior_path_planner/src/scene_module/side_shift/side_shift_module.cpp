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

#include "behavior_path_planner/scene_module/side_shift/side_shift_module.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/side_shift/util.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <lanelet2_extension/utility/utilities.hpp>
#include <opencv2/opencv.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <memory>
#include <string>

namespace
{
lanelet::ConstLanelets calcLaneAroundPose(
  const std::shared_ptr<const behavior_path_planner::PlannerData> & planner_data,
  const geometry_msgs::msg::Pose & pose, const double backward_length)
{
  const auto & p = planner_data->parameters;
  const auto & route_handler = planner_data->route_handler;

  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithinRoute(pose, &current_lane)) {
    return {};  // TODO(Horibe)
  }

  // For current_lanes with desired length
  lanelet::ConstLanelets current_lanes =
    route_handler->getLaneletSequence(current_lane, pose, backward_length, p.forward_path_length);

  return current_lanes;
}
}  // namespace

namespace behavior_path_planner
{
using geometry_msgs::msg::Point;
using geometry_msgs::msg::PoseStamped;

SideShiftModule::SideShiftModule(
  const std::string & name, rclcpp::Node & node, const SideShiftParameters & parameters)
: SceneModuleInterface{name, node}, parameters_{parameters}
{
  using std::placeholders::_1;

  lateral_offset_subscriber_ = node.create_subscription<LateralOffset>(
    "~/input/lateral_offset", 1, std::bind(&SideShiftModule::onLateralOffset, this, _1));

  // If lateral offset is subscribed, it approves side shift module automatically
  approval_handler_.clearWaitApproval();
}

void SideShiftModule::initVariables()
{
  reference_path_ = std::make_shared<PathWithLaneId>();
  start_pose_reset_request_ = false;
  lateral_offset_ = 0.0;
  prev_output_ = ShiftedPath{};
  path_shifter_ = PathShifter{};
}

void SideShiftModule::onEntry()
{
  // write me... (Don't initialize variables, otherwise lateral offset gets zero on entry.)
  start_pose_reset_request_ = false;
}

void SideShiftModule::onExit()
{
  // write me...
  initVariables();

  current_state_ = BT::NodeStatus::IDLE;
}

void SideShiftModule::setParameters(const SideShiftParameters & parameters)
{
  parameters_ = parameters;
}

bool SideShiftModule::isExecutionRequested() const
{
  if (current_state_ == BT::NodeStatus::RUNNING) {
    return true;
  }

  // If the desired offset has a non-zero value, return true as we want to execute the plan.

  const bool has_request = !isAlmostZero(lateral_offset_);
  RCLCPP_DEBUG_STREAM(
    getLogger(), "ESS::isExecutionRequested() : " << std::boolalpha << has_request);

  return has_request;
}

bool SideShiftModule::isExecutionReady() const
{
  return true;  // TODO(Horibe) is it ok to say "always safe"?
}

BT::NodeStatus SideShiftModule::updateState()
{
  // Never return the FAILURE. When the desired offset is zero and the vehicle is in the original
  // drivable area,this module can stop the computation and return SUCCESS.

  const bool no_request = isAlmostZero(lateral_offset_);

  const auto no_shifted_plan = [&]() {
    if (prev_output_.shift_length.empty()) {
      return true;
    } else {
      const auto max_planned_shift_length = std::max_element(
        prev_output_.shift_length.begin(), prev_output_.shift_length.end(),
        [](double a, double b) { return std::abs(a) < std::abs(b); });
      return std::abs(*max_planned_shift_length) < 0.01;
    }
  }();

  RCLCPP_DEBUG(
    getLogger(), "ESS::updateState() : no_request = %d, no_shifted_plan = %d", no_request,
    no_shifted_plan);

  if (no_request && no_shifted_plan) {
    current_state_ = BT::NodeStatus::SUCCESS;
  } else {
    current_state_ = BT::NodeStatus::RUNNING;
  }

  return current_state_;
}

void SideShiftModule::updateData()
{
  const auto reference_pose = prev_output_.shift_length.empty() ? *planner_data_->self_pose
                                                                : getUnshiftedEgoPose(prev_output_);
  const auto centerline_path = calcCenterLinePath(planner_data_, reference_pose);

  constexpr double resample_interval = 1.0;
  *reference_path_ = util::resamplePathWithSpline(centerline_path, resample_interval);

  path_shifter_.setPath(*reference_path_);

  const auto & route_handler = planner_data_->route_handler;
  const auto & p = planner_data_->parameters;

  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithinRoute(reference_pose.pose, &current_lane)) {
    RCLCPP_ERROR(getLogger(), "failed to find closest lanelet within route!!!");
  }

  // For current_lanes with desired length
  current_lanelets_ = route_handler->getLaneletSequence(
    current_lane, reference_pose.pose, p.backward_path_length, p.forward_path_length);

  path_shifter_.removeBehindShiftPointAndSetBaseOffset(planner_data_->self_pose->pose.position);
}

bool SideShiftModule::addShiftPoint()
{
  auto shift_points = path_shifter_.getShiftPoints();

  const auto calcLongitudinal = [this](const auto & sp) {
    return autoware_utils::calcSignedArcLength(
      reference_path_->points, getEgoPose().pose.position, sp.start.position);
  };

  // remove shift points on a far position.
  for (int i = static_cast<int>(shift_points.size()) - 1; i >= 0; --i) {
    const auto dist_to_start = calcLongitudinal(shift_points.at(i));
    const double remove_threshold =
      std::max(planner_data_->self_odometry->twist.twist.linear.x * 1.0 /* sec */, 2.0 /* m */);
    if (dist_to_start > remove_threshold) {  // TODO(Horibe)
      shift_points.erase(shift_points.begin() + i);
    }
  }

  // check if the new_shift_point has conflicts with existing shift points.
  const auto new_sp = calcShiftPoint();
  const auto new_sp_longitudinal = calcLongitudinal(new_sp);
  for (const auto & sp : shift_points) {
    if (calcLongitudinal(sp) >= new_sp_longitudinal) {
      RCLCPP_WARN(
        getLogger(),
        "try to add shift point, but shift point already exists behind the proposed point. "
        "Ignore the current proposal.");
      return false;
    }
  }

  // if no conflict, then add the new point.
  shift_points.push_back(new_sp);

  // set to path_shifter
  path_shifter_.setShiftPoints(shift_points);
  lateral_offset_change_request_ = false;

  return true;
}

BehaviorModuleOutput SideShiftModule::plan()
{
  // Update shift point
  if (lateral_offset_change_request_) {
    addShiftPoint();
  } else {
    RCLCPP_DEBUG(getLogger(), "change is not requested");
  }

  // Refine path
  ShiftedPath shifted_path;
  path_shifter_.generate(&shifted_path);

  // Reset orientation
  setOrientation(&shifted_path.path);

  adjustDrivableArea(&shifted_path);

  BehaviorModuleOutput output;
  output.path = std::make_shared<PathWithLaneId>(shifted_path.path);

  prev_output_ = shifted_path;

  return output;
}

PathWithLaneId SideShiftModule::planCandidate() const
{
  auto path_shifter_local = path_shifter_;

  path_shifter_local.addShiftPoint(calcShiftPoint());

  // Refine path
  ShiftedPath shifted_path;
  path_shifter_local.generate(&shifted_path);

  // Reset orientation
  setOrientation(&shifted_path.path);

  return shifted_path.path;
}

BehaviorModuleOutput SideShiftModule::planWaitingApproval()
{
  // Refine path
  ShiftedPath shifted_path;
  path_shifter_.generate(&shifted_path);

  // Reset orientation
  setOrientation(&shifted_path.path);

  adjustDrivableArea(&shifted_path);

  BehaviorModuleOutput output;
  output.path = std::make_shared<PathWithLaneId>(shifted_path.path);
  output.path_candidate = std::make_shared<PathWithLaneId>(planCandidate());

  prev_output_ = shifted_path;

  return output;
}

void SideShiftModule::onLateralOffset(const LateralOffset::ConstSharedPtr lateral_offset_msg)
{
  const double new_lateral_offset = lateral_offset_msg->lateral_offset;

  RCLCPP_DEBUG(
    getLogger(), "onLateralOffset start : lateral offset current = %f, new = &%f", lateral_offset_,
    new_lateral_offset);

  // offset is not changed.
  if (std::abs(lateral_offset_ - new_lateral_offset) < 1e-4) {
    return;
  }

  // new offset is requested.
  lateral_offset_change_request_ = true;

  lateral_offset_ = new_lateral_offset;
}

ShiftPoint SideShiftModule::calcShiftPoint() const
{
  const auto & p = parameters_;
  const auto ego_speed = std::abs(planner_data_->self_odometry->twist.twist.linear.x);
  const auto ego_point = planner_data_->self_pose->pose.position;

  const double dist_to_start =
    std::max(p.min_distance_to_start_shifting, ego_speed * p.time_to_start_shifting);

  const double dist_to_end = [&]() {
    const double shift_length = lateral_offset_ - getClosestShiftLength();
    const double jerk_shifting_distance = path_shifter_.calcLongitudinalDistFromJerk(
      shift_length, p.shifting_lateral_jerk, std::min(ego_speed, p.min_shifting_speed));
    const double shifting_distance = std::max(jerk_shifting_distance, p.min_shifting_distance);
    const double dist_to_end = dist_to_start + shifting_distance;
    RCLCPP_DEBUG(
      getLogger(), "min_distance_to_start_shifting = %f, dist_to_start = %f, dist_to_end = %f",
      parameters_.min_distance_to_start_shifting, dist_to_start, dist_to_end);
    return dist_to_end;
  }();

  ShiftPoint shift_point;
  shift_point.length = lateral_offset_;
  shift_point.start_idx = util::getIdxByArclength(*reference_path_, ego_point, dist_to_start);
  shift_point.start = reference_path_->points.at(shift_point.start_idx).point.pose;
  shift_point.end_idx = util::getIdxByArclength(*reference_path_, ego_point, dist_to_end);
  shift_point.end = reference_path_->points.at(shift_point.end_idx).point.pose;

  return shift_point;
}

double SideShiftModule::getClosestShiftLength() const
{
  if (prev_output_.shift_length.empty()) {
    return 0.0;
  }

  const auto ego_point = planner_data_->self_pose->pose.position;
  const auto closest = autoware_utils::findNearestIndex(prev_output_.path.points, ego_point);
  return prev_output_.shift_length.at(closest);
}

void SideShiftModule::adjustDrivableArea(ShiftedPath * path) const
{
  const auto itr = std::minmax_element(path->shift_length.begin(), path->shift_length.end());

  const double threshold = 0.1;
  const double margin = 0.5;
  const double right_offset = std::min(*itr.first - (*itr.first < -threshold ? margin : 0.0), 0.0);
  const double left_offset = std::max(*itr.second + (*itr.first > threshold ? margin : 0.0), 0.0);

  const auto extended_lanelets =
    lanelet::utils::getExpandedLanelets(current_lanelets_, left_offset, right_offset);

  {
    const auto & p = planner_data_->parameters;
    path->path.drivable_area = util::generateDrivableArea(
      extended_lanelets, *(planner_data_->self_pose), p.drivable_area_width, p.drivable_area_height,
      p.drivable_area_resolution, p.vehicle_length, *(planner_data_->route_handler));
  }
}

// NOTE: this function is ported from avoidance.
PoseStamped SideShiftModule::getUnshiftedEgoPose(const ShiftedPath & prev_path) const
{
  const auto ego_pose = getEgoPose();

  // un-shifted fot current ideal pose
  const auto closest =
    autoware_utils::findNearestIndex(prev_path.path.points, ego_pose.pose.position);

  PoseStamped unshifted_pose{};
  unshifted_pose.header = ego_pose.header;
  unshifted_pose.pose = prev_path.path.points.at(closest).point.pose;

  util::shiftPose(&unshifted_pose.pose, -prev_path.shift_length.at(closest));

  return unshifted_pose;
}

// NOTE: this function is ported from avoidance.
PathWithLaneId SideShiftModule::calcCenterLinePath(
  const std::shared_ptr<const PlannerData> & planner_data, const PoseStamped & pose) const
{
  const auto & p = planner_data->parameters;
  const auto & route_handler = planner_data->route_handler;

  PathWithLaneId centerline_path;

  // special for avoidance: take behind distance upt ot shift-start-point if it exist.
  const auto longest_dist_to_shift_point = [&]() {
    double max_dist = 0.0;
    for (const auto & pnt : path_shifter_.getShiftPoints()) {
      max_dist = std::max(max_dist, autoware_utils::calcDistance2d(getEgoPose(), pnt.start));
    }
    return max_dist;
  }();
  const auto extra_margin = 10.0;  // Since distance does not consider arclength, but just line.
  const auto backward_length =
    std::max(p.backward_path_length, longest_dist_to_shift_point + extra_margin);

  RCLCPP_DEBUG(
    getLogger(),
    "p.backward_path_length = %f, longest_dist_to_shift_point = %f, backward_length = %f",
    p.backward_path_length, longest_dist_to_shift_point, backward_length);

  const lanelet::ConstLanelets current_lanes =
    calcLaneAroundPose(planner_data, pose.pose, backward_length);
  centerline_path = util::getCenterLinePath(
    *route_handler, current_lanes, pose.pose, backward_length, p.forward_path_length, p);

  centerline_path.header = route_handler->getRouteHeader();

  return centerline_path;
}

}  // namespace behavior_path_planner
