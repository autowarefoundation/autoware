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

#include "behavior_path_planner/scene_module/lane_following/lane_following_module.hpp"

#include "behavior_path_planner/utilities.hpp"

#include <memory>
#include <string>

namespace behavior_path_planner
{
LaneFollowingModule::LaneFollowingModule(
  const std::string & name, rclcpp::Node & node, const LaneFollowingParameters & parameters)
: SceneModuleInterface{name, node}, parameters_{parameters}
{
  initParam();
}

void LaneFollowingModule::initParam()
{
  approval_handler_.clearWaitApproval();  // no need approval
}

bool LaneFollowingModule::isExecutionRequested() const { return true; }

bool LaneFollowingModule::isExecutionReady() const { return true; }

BT::NodeStatus LaneFollowingModule::updateState()
{
  current_state_ = BT::NodeStatus::SUCCESS;
  return current_state_;
}

BehaviorModuleOutput LaneFollowingModule::plan()
{
  BehaviorModuleOutput output;
  output.path = std::make_shared<PathWithLaneId>(getReferencePath());
  return output;
}
PathWithLaneId LaneFollowingModule::planCandidate() const { return getReferencePath(); }
void LaneFollowingModule::onEntry()
{
  initParam();
  current_state_ = BT::NodeStatus::RUNNING;
  RCLCPP_DEBUG(getLogger(), "LANE_FOLLOWING onEntry");
}
void LaneFollowingModule::onExit()
{
  initParam();
  current_state_ = BT::NodeStatus::IDLE;
  RCLCPP_DEBUG(getLogger(), "LANE_FOLLOWING onExit");
}

void LaneFollowingModule::setParameters(const LaneFollowingParameters & parameters)
{
  parameters_ = parameters;
}

PathWithLaneId LaneFollowingModule::getReferencePath() const
{
  PathWithLaneId reference_path{};

  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = planner_data_->self_pose->pose;
  const auto p = planner_data_->parameters;

  // Set header
  reference_path.header = route_handler->getRouteHeader();

  lanelet::ConstLanelet current_lane;
  if (!planner_data_->route_handler->getClosestLaneletWithinRoute(current_pose, &current_lane)) {
    RCLCPP_ERROR(getLogger(), "failed to find closest lanelet within route!!!");
    return reference_path;  // TODO(Horibe)
  }

  // For current_lanes with desired length
  lanelet::ConstLanelets current_lanes = planner_data_->route_handler->getLaneletSequence(
    current_lane, current_pose, p.backward_path_length, p.forward_path_length);

  if (current_lanes.empty()) {
    return reference_path;
  }

  reference_path = util::getCenterLinePath(
    *route_handler, current_lanes, current_pose, p.backward_path_length, p.forward_path_length, p);

  {
    // buffer for min_lane_change_length
    const double buffer = p.backward_length_buffer_for_end_of_lane;
    const int num_lane_change =
      std::abs(route_handler->getNumLaneToPreferredLane(current_lanes.back()));
    const double lane_change_buffer = num_lane_change * (p.minimum_lane_change_length + buffer);
    reference_path = util::setDecelerationVelocity(
      *route_handler, reference_path, current_lanes, parameters_.lane_change_prepare_duration,
      lane_change_buffer);
  }

  if (parameters_.expand_drivable_area) {
    lanelet::ConstLanelets expand_lanes{};
    for (const auto & current_lane : current_lanes) {
      const std::string r_type =
        current_lane.rightBound().attributeOr(lanelet::AttributeName::Type, "none");
      const std::string l_type =
        current_lane.leftBound().attributeOr(lanelet::AttributeName::Type, "none");

      const double r_offset =
        r_type.compare("road_border") != 0 ? -parameters_.right_bound_offset : 0.0;
      const double l_offset =
        l_type.compare("road_border") != 0 ? parameters_.left_bound_offset : 0.0;

      expand_lanes.push_back(lanelet::utils::getExpandedLanelet(current_lane, l_offset, r_offset));
    }

    current_lanes = expand_lanes;
  }

  reference_path.drivable_area = util::generateDrivableArea(
    current_lanes, *planner_data_->self_pose, p.drivable_area_width, p.drivable_area_height,
    p.drivable_area_resolution, p.vehicle_length, *route_handler);

  return reference_path;
}

}  // namespace behavior_path_planner
