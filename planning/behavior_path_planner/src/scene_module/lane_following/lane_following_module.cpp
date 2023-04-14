// Copyright 2021-2023 Tier IV, Inc.
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

#include "behavior_path_planner/utils/path_utils.hpp"
#include "behavior_path_planner/utils/utils.hpp"

#include <lanelet2_extension/utility/utilities.hpp>

#include <memory>
#include <string>

namespace behavior_path_planner
{
LaneFollowingModule::LaneFollowingModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<LaneFollowingParameters> & parameters)
// RTCInterface is temporarily registered, but not used.
: SceneModuleInterface{name, node, createRTCInterfaceMap(node, name, {""})}, parameters_{parameters}
{
  initParam();
  // TODO(murooka) The following is temporary implementation for new architecture's refactoring
  steering_factor_interface_ptr_ =
    std::make_unique<SteeringFactorInterface>(&node, "lane_following");
}

void LaneFollowingModule::initParam()
{
  clearWaitingApproval();  // no need approval
}

bool LaneFollowingModule::isExecutionRequested() const
{
  return true;
}

bool LaneFollowingModule::isExecutionReady() const
{
  return true;
}

BT::NodeStatus LaneFollowingModule::updateState()
{
  current_state_ = BT::NodeStatus::SUCCESS;
  return current_state_;
}

BehaviorModuleOutput LaneFollowingModule::plan()
{
  return getReferencePath();
}
CandidateOutput LaneFollowingModule::planCandidate() const
{
  const auto path = getReferencePath().path;
  if (!path) {
    return {};
  }
  return CandidateOutput(*path);
}
void LaneFollowingModule::processOnEntry()
{
  initParam();
  current_state_ = BT::NodeStatus::RUNNING;
}
void LaneFollowingModule::processOnExit()
{
  initParam();
  current_state_ = BT::NodeStatus::SUCCESS;
}

void LaneFollowingModule::setParameters(const std::shared_ptr<LaneFollowingParameters> & parameters)
{
  parameters_ = parameters;
}

BehaviorModuleOutput LaneFollowingModule::getReferencePath() const
{
  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = planner_data_->self_odometry->pose.pose;

  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithConstrainsWithinRoute(
        current_pose, &current_lane, parameters_->distance_threshold, parameters_->yaw_threshold)) {
    RCLCPP_ERROR_THROTTLE(
      getLogger(), *clock_, 5000, "failed to find closest lanelet within route!!!");
    return {};  // TODO(Horibe)
  }

  return util::getReferencePath(current_lane, parameters_, planner_data_);
}
}  // namespace behavior_path_planner
