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

namespace behavior_velocity_planner::intersection
{

/**
 * @struct
 * @brief Internal error or ego already passed pass_judge_line
 */
struct Indecisive
{
  std::string error;
};

/**
 * @struct
 * @brief detected stuck vehicle
 */
struct StuckStop
{
  size_t closest_idx{0};
  size_t stuck_stopline_idx{0};
  std::optional<size_t> occlusion_stopline_idx{std::nullopt};
};

/**
 * @struct
 * @brief yielded by vehicle on the attention area
 */
struct YieldStuckStop
{
  size_t closest_idx{0};
  size_t stuck_stopline_idx{0};
};

/**
 * @struct
 * @brief only collision is detected
 */
struct NonOccludedCollisionStop
{
  size_t closest_idx{0};
  size_t collision_stopline_idx{0};
  size_t occlusion_stopline_idx{0};
};

/**
 * @struct
 * @brief occlusion is detected so ego needs to stop at the default stop line position
 */
struct FirstWaitBeforeOcclusion
{
  bool is_actually_occlusion_cleared{false};
  size_t closest_idx{0};
  size_t first_stopline_idx{0};
  size_t occlusion_stopline_idx{0};
};

/**
 * @struct
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
};

/**
 * @struct
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
};

/**
 * @struct
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
};

/**
 * @struct
 * @brief both collision and occlusion are not detected
 */
struct Safe
{
  size_t closest_idx{0};
  size_t collision_stopline_idx{0};
  size_t occlusion_stopline_idx{0};
};

/**
 * @struct
 * @brief traffic light is red or arrow signal
 */
struct FullyPrioritized
{
  bool collision_detected{false};
  size_t closest_idx{0};
  size_t collision_stopline_idx{0};
  size_t occlusion_stopline_idx{0};
};

using DecisionResult = std::variant<
  Indecisive,                   //! internal process error, or over the pass judge line
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

inline std::string formatDecisionResult(const DecisionResult & decision_result)
{
  if (std::holds_alternative<Indecisive>(decision_result)) {
    const auto indecisive = std::get<Indecisive>(decision_result);
    return "Indecisive because " + indecisive.error;
  }
  if (std::holds_alternative<StuckStop>(decision_result)) {
    return "StuckStop";
  }
  if (std::holds_alternative<YieldStuckStop>(decision_result)) {
    return "YieldStuckStop";
  }
  if (std::holds_alternative<NonOccludedCollisionStop>(decision_result)) {
    return "NonOccludedCollisionStop";
  }
  if (std::holds_alternative<FirstWaitBeforeOcclusion>(decision_result)) {
    return "FirstWaitBeforeOcclusion";
  }
  if (std::holds_alternative<PeekingTowardOcclusion>(decision_result)) {
    return "PeekingTowardOcclusion";
  }
  if (std::holds_alternative<OccludedCollisionStop>(decision_result)) {
    return "OccludedCollisionStop";
  }
  if (std::holds_alternative<OccludedAbsenceTrafficLight>(decision_result)) {
    return "OccludedAbsenceTrafficLight";
  }
  if (std::holds_alternative<Safe>(decision_result)) {
    return "Safe";
  }
  if (std::holds_alternative<FullyPrioritized>(decision_result)) {
    return "FullyPrioritized";
  }
  return "";
}

}  // namespace behavior_velocity_planner::intersection

#endif  // DECISION_RESULT_HPP_
