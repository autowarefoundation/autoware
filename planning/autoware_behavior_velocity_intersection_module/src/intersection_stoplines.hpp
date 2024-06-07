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

#ifndef INTERSECTION_STOPLINES_HPP_
#define INTERSECTION_STOPLINES_HPP_

#include <optional>

namespace autoware::behavior_velocity_planner
{

/**
 * @brief see the document for more details of IntersectionStopLines
 */
struct IntersectionStopLines
{
  size_t closest_idx{0};

  /**
   * stuck_stopline is null if ego path does not intersect with first_conflicting_area
   */
  std::optional<size_t> stuck_stopline{std::nullopt};

  /**
   * default_stopline is null if it is calculated negative from first_attention_stopline
   */
  std::optional<size_t> default_stopline{std::nullopt};

  /**
   * first_attention_stopline is null if ego footprint along the path does not intersect with
   * attention area. if path[0] satisfies the condition, it is 0
   */
  std::optional<size_t> first_attention_stopline{std::nullopt};

  /**
   * second_attention_stopline is null if ego footprint along the path does not intersect with
   * second_attention_lane. if path[0] satisfies the condition, it is 0
   */
  std::optional<size_t> second_attention_stopline{std::nullopt};

  /**
   * occlusion_peeking_stopline is null if path[0] is already inside the attention area
   */
  std::optional<size_t> occlusion_peeking_stopline{std::nullopt};

  /**
   * first_pass_judge_line is before first_attention_stopline by the braking distance. if its value
   * is calculated negative, it is 0
   */
  size_t first_pass_judge_line{0};

  /**
   * second_pass_judge_line is before second_attention_stopline by the braking distance. if
   * second_attention_lane is null, it is null
   */
  std::optional<size_t> second_pass_judge_line{std::nullopt};

  /**
   * occlusion_wo_tl_pass_judge_line is null if ego footprint along the path does not intersect with
   * the centerline of the first_attention_lane
   */
  size_t occlusion_wo_tl_pass_judge_line{0};
};
}  // namespace autoware::behavior_velocity_planner

#endif  // INTERSECTION_STOPLINES_HPP_
