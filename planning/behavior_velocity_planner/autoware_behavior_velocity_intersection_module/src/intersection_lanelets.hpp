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

#ifndef INTERSECTION_LANELETS_HPP_
#define INTERSECTION_LANELETS_HPP_

#include "interpolated_path_info.hpp"

#include <autoware/universe_utils/geometry/boost_geometry.hpp>

#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_routing/Forward.h>

#include <optional>
#include <vector>

namespace autoware::behavior_velocity_planner
{

/**
 * @brief see the document for more details of IntersectionLanelets
 */
struct IntersectionLanelets
{
public:
  /**
   * update conflicting lanelets and traffic priority information
   */
  void update(
    const bool is_prioritized, const InterpolatedPathInfo & interpolated_path_info,
    const autoware::universe_utils::LinearRing2d & footprint, const double vehicle_length,
    lanelet::routing::RoutingGraphPtr routing_graph_ptr);

  const lanelet::ConstLanelets & attention() const
  {
    return is_prioritized_ ? attention_non_preceding_ : attention_;
  }
  const std::vector<std::optional<lanelet::ConstLineString3d>> & attention_stoplines() const
  {
    return is_prioritized_ ? attention_non_preceding_stoplines_ : attention_stoplines_;
  }
  const lanelet::ConstLanelets & conflicting() const { return conflicting_; }
  const lanelet::ConstLanelets & adjacent() const { return adjacent_; }
  const lanelet::ConstLanelets & occlusion_attention() const
  {
    return is_prioritized_ ? attention_non_preceding_ : occlusion_attention_;
  }
  const lanelet::ConstLanelets & attention_non_preceding() const
  {
    return attention_non_preceding_;
  }
  const std::vector<lanelet::CompoundPolygon3d> & attention_area() const
  {
    return is_prioritized_ ? attention_non_preceding_area_ : attention_area_;
  }
  const std::vector<lanelet::CompoundPolygon3d> & conflicting_area() const
  {
    return conflicting_area_;
  }
  const std::vector<lanelet::CompoundPolygon3d> & adjacent_area() const { return adjacent_area_; }
  const std::vector<lanelet::CompoundPolygon3d> & occlusion_attention_area() const
  {
    return occlusion_attention_area_;
  }
  const std::optional<lanelet::ConstLanelet> & first_conflicting_lane() const
  {
    return first_conflicting_lane_;
  }
  const std::optional<lanelet::CompoundPolygon3d> & first_conflicting_area() const
  {
    return first_conflicting_area_;
  }
  const std::optional<lanelet::ConstLanelet> & first_attention_lane() const
  {
    return first_attention_lane_;
  }
  const std::optional<lanelet::CompoundPolygon3d> & first_attention_area() const
  {
    return first_attention_area_;
  }
  const std::optional<lanelet::ConstLanelet> & second_attention_lane() const
  {
    return second_attention_lane_;
  }
  const std::optional<lanelet::CompoundPolygon3d> & second_attention_area() const
  {
    return second_attention_area_;
  }

  /**
   * the set of attention lanelets which is topologically merged
   */
  lanelet::ConstLanelets attention_;
  std::vector<lanelet::CompoundPolygon3d> attention_area_;

  /**
   * the stop lines for each attention_lanelets associated with traffic lights. At intersection
   * without traffic lights, each value is null
   */
  std::vector<std::optional<lanelet::ConstLineString3d>> attention_stoplines_;

  /**
   * the conflicting part of attention lanelets
   */
  lanelet::ConstLanelets attention_non_preceding_;
  std::vector<lanelet::CompoundPolygon3d> attention_non_preceding_area_;

  /**
   * the stop lines for each attention_non_preceding_
   */
  std::vector<std::optional<lanelet::ConstLineString3d>> attention_non_preceding_stoplines_;

  /**
   * the conflicting lanelets of the objective intersection lanelet
   */
  lanelet::ConstLanelets conflicting_;
  std::vector<lanelet::CompoundPolygon3d> conflicting_area_;

  /**
   *
   */
  lanelet::ConstLanelets adjacent_;
  std::vector<lanelet::CompoundPolygon3d> adjacent_area_;

  /**
   * the set of attention lanelets for occlusion detection which is topologically merged
   */
  lanelet::ConstLanelets occlusion_attention_;
  std::vector<lanelet::CompoundPolygon3d> occlusion_attention_area_;

  /**
   * the vector of sum of each occlusion_attention lanelet
   */
  std::vector<double> occlusion_attention_size_;

  /**
   * the first conflicting lanelet which ego path points intersect for the first time
   */
  std::optional<lanelet::ConstLanelet> first_conflicting_lane_{std::nullopt};
  std::optional<lanelet::CompoundPolygon3d> first_conflicting_area_{std::nullopt};

  /**
   * the first attention lanelet which ego path points intersect for the first time
   */
  std::optional<lanelet::ConstLanelet> first_attention_lane_{std::nullopt};
  std::optional<lanelet::CompoundPolygon3d> first_attention_area_{std::nullopt};

  /**
   * the second attention lanelet which ego path points intersect next to the
   * first_attention_lanelet
   */
  bool second_attention_lane_empty_{false};
  std::optional<lanelet::ConstLanelet> second_attention_lane_{std::nullopt};
  std::optional<lanelet::CompoundPolygon3d> second_attention_area_{std::nullopt};

  /**
   * flag if the intersection is prioritized by the traffic light
   */
  bool is_prioritized_{false};
};

/**
 * @brief see the document for more details of PathLanelets
 */
struct PathLanelets
{
  lanelet::ConstLanelets prev;
  // lanelet::ConstLanelet entry2ego; this is included in `all` if exists
  lanelet::ConstLanelet
    ego_or_entry2exit;  // this is `assigned lane` part of the path(not from
                        // ego) if ego is before the intersection, otherwise from ego to exit
  std::optional<lanelet::ConstLanelet> next =
    std::nullopt;  // this is nullopt is the goal is inside intersection
  lanelet::ConstLanelets all;
  lanelet::ConstLanelets
    conflicting_interval_and_remaining;  // the left/right-most interval of path conflicting with
                                         // conflicting lanelets plus the next lane part of the
                                         // path
};
}  // namespace autoware::behavior_velocity_planner

#endif  // INTERSECTION_LANELETS_HPP_
