// Copyright 2024 Tier IV, Inc.
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

#ifndef DECISION_RESULT_HPP_
#define DECISION_RESULT_HPP_

#include <optional>
#include <string>
#include <variant>

namespace autoware::behavior_velocity_planner
{

/**
 * @brief internal error
 */
struct InternalError
{
  std::string error;
};

/**
 * @brief
 */
struct OverPassJudge
{
  std::string safety_report;
  std::string evasive_report;
};

/**
 * @brief detected stuck vehicle
 */
struct StuckStop
{
  size_t closest_idx{0};
  size_t stuck_stopline_idx{0};
  std::optional<size_t> occlusion_stopline_idx{std::nullopt};
};

/**
 * @brief yielded by vehicle on the attention area
 */
struct YieldStuckStop
{
  size_t closest_idx{0};
  size_t stuck_stopline_idx{0};
  std::string occlusion_report;
};

/**
 * @brief only collision is detected
 */
struct NonOccludedCollisionStop
{
  size_t closest_idx{0};
  size_t collision_stopline_idx{0};
  size_t occlusion_stopline_idx{0};
  std::string occlusion_report;
};

/**
 * @brief occlusion is detected so ego needs to stop at the default stop line position
 */
struct FirstWaitBeforeOcclusion
{
  bool is_actually_occlusion_cleared{false};
  size_t closest_idx{0};
  size_t first_stopline_idx{0};
  size_t occlusion_stopline_idx{0};
  std::string occlusion_report;
};

/**
 * @brief ego is approaching the boundary of attention area in the presence of traffic light
 */
struct PeekingTowardOcclusion
{
  //! if intersection_occlusion is disapproved externally through RTC, it indicates
  //! "is_forcefully_occluded"
  bool is_actually_occlusion_cleared{false};
  bool temporal_stop_before_attention_required{false};
  size_t closest_idx{0};
  size_t collision_stopline_idx{0};
  size_t first_attention_stopline_idx{0};
  size_t occlusion_stopline_idx{0};
  //! if null, it is dynamic occlusion and shows up intersection_occlusion(dyn). if valid, it
  //! contains the remaining time to release the static occlusion stuck and shows up
  //! intersection_occlusion(x.y)
  std::optional<double> static_occlusion_timeout{std::nullopt};
  std::string occlusion_report;
};

/**
 * @brief both collision and occlusion are detected in the presence of traffic light
 */
struct OccludedCollisionStop
{
  bool is_actually_occlusion_cleared{false};
  bool temporal_stop_before_attention_required{false};
  size_t closest_idx{0};
  size_t collision_stopline_idx{0};
  size_t first_attention_stopline_idx{0};
  size_t occlusion_stopline_idx{0};
  //! if null, it is dynamic occlusion and shows up intersection_occlusion(dyn). if valid, it
  //! contains the remaining time to release the static occlusion stuck
  std::optional<double> static_occlusion_timeout{std::nullopt};
  std::string occlusion_report;
};

/**
 * @brief at least occlusion is detected in the absence of traffic light
 */
struct OccludedAbsenceTrafficLight
{
  bool is_actually_occlusion_cleared{false};
  bool collision_detected{false};
  bool temporal_stop_before_attention_required{false};
  size_t closest_idx{0};
  size_t first_attention_area_stopline_idx{0};
  size_t peeking_limit_line_idx{0};
  std::string occlusion_report;
};

/**
 * @brief both collision and occlusion are not detected
 */
struct Safe
{
  size_t closest_idx{0};
  size_t collision_stopline_idx{0};
  size_t occlusion_stopline_idx{0};
  std::string occlusion_report;
};

/**
 * @brief traffic light is red or arrow signal
 */
struct FullyPrioritized
{
  bool collision_detected{false};
  size_t closest_idx{0};
  size_t collision_stopline_idx{0};
  size_t occlusion_stopline_idx{0};
  std::string safety_report;
};

using DecisionResult = std::variant<
  InternalError,                //! internal process error
  OverPassJudge,                //! over the pass judge lines
  StuckStop,                    //! detected stuck vehicle
  YieldStuckStop,               //! detected yield stuck vehicle
  NonOccludedCollisionStop,     //! detected collision while FOV is clear
  FirstWaitBeforeOcclusion,     //! stop for a while before peeking to occlusion
  PeekingTowardOcclusion,       //! peeking into occlusion while collision is not detected
  OccludedCollisionStop,        //! occlusion and collision are both detected
  OccludedAbsenceTrafficLight,  //! occlusion is detected in the absence of traffic light
  Safe,                         //! judge as safe
  FullyPrioritized              //! only detect vehicles violating traffic rules
  >;

std::string formatDecisionResult(const DecisionResult & decision_result);

}  // namespace autoware::behavior_velocity_planner

#endif  // DECISION_RESULT_HPP_
